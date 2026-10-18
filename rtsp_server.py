#!/usr/bin/env python3
"""Live RTSP/1.0 server: OpenCV capture, FFmpeg encoding, socket RTP/RTCP.

Implements the unicast, video-only playback subset of RFC 2326, RFC 3550,
and RFC 6184. FFmpeg is only an encoder; all network protocols live here.
"""
from __future__ import annotations

import argparse
import base64
import logging
import math
import queue
import re
import secrets
import signal
import socket
import struct
import subprocess
import threading
import time
from email.utils import formatdate
from urllib.parse import urlsplit

import cv2
import numpy as np

LOG = logging.getLogger("rtsp")
START_CODE = re.compile(b"\x00\x00(?:\x00)?\x01")
METHODS = "OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER"
REASONS = {200: "OK", 400: "Bad Request", 404: "Not Found",
           405: "Method Not Allowed", 406: "Not Acceptable", 451: "Parameter Not Understood",
           454: "Session Not Found", 455: "Method Not Valid in This State",
           457: "Invalid Range", 461: "Unsupported Transport", 501: "Not Implemented",
           503: "Service Unavailable", 505: "RTSP Version Not Supported",
           551: "Option Not Supported"}


class AnnexB:
    """Incremental Annex B parser; start codes can cross pipe read boundaries."""
    def __init__(self):
        self.buffer = b""

    def feed(self, data, final=False):
        self.buffer += data
        matches = list(START_CODE.finditer(self.buffer))
        nals = []
        for first, second in zip(matches, matches[1:]):
            nal = self.buffer[first.end():second.start()].rstrip(b"\x00")
            if nal:
                nals.append(nal)
        if matches:
            last = matches[-1]
            if final:
                nal = self.buffer[last.end():].rstrip(b"\x00")
                if nal:
                    nals.append(nal)
                self.buffer = b""
            else:
                self.buffer = self.buffer[last.start():]
        return nals


class Packetizer:
    def __init__(self, mtu=1200):
        self.mtu = mtu
        self.seq = secrets.randbits(16)
        self.ssrc = secrets.randbits(32)
        self.packets = self.octets = 0

    def encode(self, nals, timestamp):
        payloads = []
        for nal in nals:
            if len(nal) <= self.mtu - 12:
                payloads.append(nal)
            else:
                size = self.mtu - 14
                data = nal[1:]
                for offset in range(0, len(data), size):
                    end = offset + size >= len(data)
                    header = (nal[0] & 31) | (0x80 if offset == 0 else 0) | (0x40 if end else 0)
                    payloads.append(bytes([(nal[0] & 0xe0) | 28, header]) + data[offset:offset + size])
        for i, payload in enumerate(payloads):
            packet = struct.pack("!BBHII", 0x80, 96 | (0x80 if i == len(payloads)-1 else 0),
                                 self.seq, timestamp & 0xffffffff, self.ssrc) + payload
            self.seq = (self.seq + 1) & 0xffff
            self.packets += 1
            self.octets += len(payload)
            yield packet

    def report(self, timestamp, wall_time=None, bye=False):
        now = time.time() if wall_time is None else wall_time
        ntp = now + 2208988800
        sr = struct.pack("!BBHIIIIII", 0x80, 200, 6, self.ssrc, int(ntp),
                         int((ntp % 1) * 2**32), timestamp & 0xffffffff,
                         self.packets & 0xffffffff, self.octets & 0xffffffff)
        cname = f"{self.ssrc:08x}@opencv-rtsp".encode()
        chunk = struct.pack("!I", self.ssrc) + bytes([1, len(cname)]) + cname + b"\x00"
        chunk += b"\x00" * (-len(chunk) % 4)
        result = sr + struct.pack("!BBH", 0x81, 202, len(chunk)//4) + chunk
        if bye:
            result += struct.pack("!BBHI", 0x81, 203, 1, self.ssrc)
        return result


class Media:
    def __init__(self, args):
        self.args = args
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self.subscribers = set()
        self.sps = self.pps = None
        self.error = None
        self.capture = None
        self.process = None
        self.threads = []
        self.clock_base = secrets.randbits(32)
        self.wall_base = time.time()

    def start(self):
        a = self.args
        if a.source != "test":
            source = int(a.source) if a.source.isdecimal() else a.source
            self.capture = cv2.VideoCapture(source)
            if not self.capture.isOpened():
                self.capture.release()
                raise RuntimeError(f"Cannot open VideoCapture({source!r})")
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
            self.capture.set(cv2.CAP_PROP_FPS, a.fps)
        gop = max(1, round(a.fps))
        command = [a.ffmpeg, "-hide_banner", "-loglevel", "warning", "-f", "rawvideo",
                   "-pix_fmt", "bgr24", "-s", f"{a.width}x{a.height}", "-r", str(a.fps),
                   "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "ultrafast",
                   "-tune", "zerolatency", "-pix_fmt", "yuv420p", "-profile:v", "baseline",
                   "-b:v", a.bitrate, "-g", str(gop), "-keyint_min", str(gop), "-bf", "0",
                   "-x264-params", "aud=1:repeat-headers=1:scenecut=0", "-f", "h264", "pipe:1"]
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE, bufsize=0)
        except Exception:
            if self.capture:
                self.capture.release()
            raise
        self.wall_base = time.time()
        for target in (self._capture, self._read, self._stderr):
            thread = threading.Thread(target=target, daemon=True)
            self.threads.append(thread)
            thread.start()
        if not self.ready.wait(10) or self.error:
            self.close()
            raise RuntimeError(self.error or "Encoder produced no SPS/PPS in 10 seconds")

    def _fail(self, error):
        if not self.stop.is_set():
            self.error = str(error)
            LOG.error("Media stopped: %s", error)
        self.stop.set()
        self.ready.set()

    def _capture(self):
        a = self.args
        deadline = time.monotonic()
        count = 0
        try:
            while not self.stop.is_set():
                if self.capture:
                    ok, frame = self.capture.read()
                    if not ok:
                        if a.loop and not a.source.isdecimal():
                            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                            ok, frame = self.capture.read()
                        if not ok:
                            raise RuntimeError("Capture reached EOF or failed")
                    #frame = cv2.resize(frame, (a.width, a.height))
                else:
                    frame = np.zeros((a.height, a.width, 3), dtype=np.uint8)
                    cv2.circle(frame, (int(count*7 % a.width), a.height//2), 40, (0, 200, 255), -1)
                    cv2.putText(frame, f"OpenCV RTSP frame {count}", (20, 45),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                data = memoryview(np.ascontiguousarray(frame)).cast("B")
                while data and not self.stop.is_set():
                    written = self.process.stdin.write(data)
                    if not written:
                        raise BrokenPipeError("FFmpeg input closed")
                    data = data[written:]
                count += 1
                deadline += 1/a.fps
                self.stop.wait(max(0, deadline-time.monotonic()))
        except Exception as exc:
            self._fail(exc)
        finally:
            if self.capture:
                self.capture.release()
            try:
                self.process.stdin.close()
            except OSError:
                pass

    def _read(self):
        parser = AnnexB()
        unit = []
        frame = 0
        try:
            while not self.stop.is_set():
                data = self.process.stdout.read(65536)
                for nal in parser.feed(data, final=not data):
                    kind = nal[0] & 31
                    if kind == 9 and unit:
                        if any(n[0] & 31 in (1, 5) for n in unit):
                            timestamp = (self.clock_base + round(frame*90000/self.args.fps)) & 0xffffffff
                            wall_time = self.wall_base + frame/self.args.fps
                            frame += 1
                            with self.lock:
                                for q in self.subscribers:
                                    try:
                                        q.put_nowait((tuple(unit), timestamp, wall_time))
                                    except queue.Full:
                                        # Disconnect slow consumers rather than corrupting their prediction chain.
                                        while not q.empty():
                                            q.get_nowait()
                                        q.put_nowait(None)
                        unit = []
                    if kind == 7:
                        self.sps = nal
                    if kind == 8:
                        self.pps = nal
                    if self.sps and self.pps:
                        self.ready.set()
                    if kind != 9:
                        unit.append(nal)
                if not data:
                    break
            if not self.stop.is_set():
                self._fail("FFmpeg output closed")
        except Exception as exc:
            self._fail(exc)

    def _stderr(self):
        for line in self.process.stderr:
            LOG.warning("ffmpeg: %s", line.decode(errors="replace").strip())

    def subscribe(self):
        q = queue.Queue(maxsize=max(10, round(self.args.fps*2)))
        with self.lock:
            self.subscribers.add(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.subscribers.discard(q)

    def close(self):
        self.stop.set()
        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
            for t in self.threads:
                t.join(timeout=2)
            for pipe in (self.process.stdin, self.process.stdout, self.process.stderr):
                pipe.close()


def udp_pair(host):
    for _ in range(100):
        rtp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rtcp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            rtp.bind((host, 0))
            port = rtp.getsockname()[1]
            if port % 2 or port == 65535:
                rtp.close()
                rtcp.close()
                continue
            rtcp.bind((host, port+1))
            rtcp.setblocking(False)
            return rtp, rtcp
        except OSError:
            rtp.close()
            rtcp.close()
    raise OSError("Could not allocate an even/odd RTP/RTCP port pair")


class Client:
    def __init__(self, server, conn, peer):
        self.server, self.conn, self.peer = server, conn, peer
        self.conn.settimeout(1)
        self.buffer = b""
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.session = None
        self.state = "INIT"
        self.transport = None
        self.udp = None
        self.channels = (0, 1)
        self.ports = None
        self.packetizer = Packetizer(server.args.mtu)
        self.media_queue = None
        self.sender = None
        self.last_activity = time.monotonic()
        self.timestamp = 0
        self.wall_time = time.time()

    def response(self, code, cseq, headers=None, body=b""):
        fields = {"CSeq": cseq, "Date": formatdate(usegmt=True), "Server": "OpenCV-RTSP/1.0"}
        if self.session:
            fields["Session"] = f"{self.session};timeout={self.server.args.timeout}"
        fields.update(headers or {})
        fields["Content-Length"] = str(len(body))
        data = f"RTSP/1.0 {code} {REASONS[code]}\r\n".encode()
        data += "".join(f"{k}: {v}\r\n" for k, v in fields.items()).encode() + b"\r\n" + body
        with self.lock:
            self.conn.sendall(data)

    def read_message(self):
        while not self.closed.is_set():
            if self.buffer.startswith(b"$"):
                if len(self.buffer) >= 4:
                    channel, size = self.buffer[1], struct.unpack("!H", self.buffer[2:4])[0]
                    if len(self.buffer) >= size+4:
                        payload = self.buffer[4:size+4]
                        self.buffer = self.buffer[size+4:]
                        if self.transport == "tcp" and channel == self.channels[1]:
                            self.receive_rtcp(payload)
                        continue
            else:
                end = self.buffer.find(b"\r\n\r\n")
                if end >= 0:
                    if end > 16384:
                        raise ValueError("Header too large")
                    lines = self.buffer[:end].decode("ascii").split("\r\n")
                    method, uri, version = lines[0].split()
                    headers = {}
                    for line in lines[1:]:
                        key, value = line.split(":", 1)
                        key = key.lower().strip()
                        if key in headers:
                            raise ValueError("Duplicate header")
                        headers[key] = value.strip()
                    length = int(headers.get("content-length", "0"))
                    if not 0 <= length <= 65536:
                        raise ValueError("Invalid body size")
                    if len(self.buffer) >= end+4+length:
                        body = self.buffer[end+4:end+4+length]
                        self.buffer = self.buffer[end+4+length:]
                        return method, uri, version, headers, body
                elif len(self.buffer) > 16384:
                    raise ValueError("Header too large")
            if time.monotonic()-self.last_activity > self.server.args.timeout:
                raise TimeoutError("Session expired")
            try:
                data = self.conn.recv(8192)
            except socket.timeout:
                continue
            if not data:
                raise ConnectionError("Peer disconnected")
            self.buffer += data
        raise ConnectionError("Closed")

    def receive_rtcp(self, data):
        # Validate compound packet framing. Receiver reports are accepted;
        # retransmission and congestion adaptation are outside this live subset.
        offset = 0
        while offset+4 <= len(data):
            first, kind, length = struct.unpack("!BBH", data[offset:offset+4])
            size = (length+1)*4
            if first >> 6 != 2 or offset+size > len(data):
                return
            if kind in (200, 201, 202, 203):
                self.last_activity = time.monotonic()
            offset += size

    def handle(self, method, uri, version, h, body):
        cseq = h.get("cseq", "0")
        if not cseq.isdecimal():
            return self.response(400, "0")
        if version != "RTSP/1.0":
            return self.response(505, cseq)
        if "cseq" not in h:
            return self.response(400, "0")
        self.last_activity = time.monotonic()
        required = ",".join(h[k] for k in ("require", "proxy-require") if k in h)
        if required:
            return self.response(551, cseq, {"Unsupported": required})
        if method == "OPTIONS":
            return self.response(200, cseq, {"Public": METHODS})
        path = urlsplit(uri).path.rstrip("/")
        base = self.server.args.path.rstrip("/")
        if uri != "*" and path not in (base, base+"/trackID=0"):
            return self.response(404, cseq)
        if method == "DESCRIBE":
            if h.get("accept") and not any(x.split(";", 1)[0].strip() in ("application/sdp", "*/*", "application/*") for x in h["accept"].split(",")):
                return self.response(406, cseq)
            if self.server.media.error:
                return self.response(503, cseq)
            sps, pps = self.server.media.sps, self.server.media.pps
            ip = self.conn.getsockname()[0]
            sdp = (f"v=0\r\no=- {self.server.sdp_id} 1 IN IP4 {ip}\r\n"
                   "s=OpenCV live stream\r\nc=IN IP4 0.0.0.0\r\nt=0 0\r\n"
                   "a=control:*\r\na=range:npt=now-\r\nm=video 0 RTP/AVP 96\r\n"
                   "a=rtpmap:96 H264/90000\r\n"
                   f"a=fmtp:96 packetization-mode=1;profile-level-id={sps[1:4].hex()};"
                   f"sprop-parameter-sets={base64.b64encode(sps).decode()},{base64.b64encode(pps).decode()}\r\n"
                   "a=control:trackID=0\r\n").encode()
            return self.response(200, cseq, {"Content-Type": "application/sdp", "Content-Base": uri.rstrip("/")+"/"}, sdp)
        supplied = h.get("session", "").split(";", 1)[0]
        if supplied and supplied != self.session:
            return self.response(454, cseq)
        if method == "SETUP":
            if path != base+"/trackID=0":
                return self.response(404, cseq)
            if self.state != "INIT":
                return self.response(455, cseq)
            selected = self.setup(h.get("transport", ""))
            if selected is None:
                return self.response(461, cseq)
            self.session = secrets.token_hex(12)
            self.state = "READY"
            return self.response(200, cseq, {"Transport": selected})
        if method not in METHODS.split(", "):
            return self.response(501, cseq)
        if not self.session or supplied != self.session:
            return self.response(454, cseq)
        if method == "GET_PARAMETER":
            return self.response(451 if body.strip() else 200, cseq)
        if method == "PLAY":
            if h.get("range", "npt=now-") not in ("npt=now-", "npt=0-", "npt=0.000-"):
                return self.response(457, cseq)
            if h.get("scale", "1") not in ("1", "1.0"):
                return self.response(501, cseq)
            if self.server.media.error:
                return self.response(503, cseq)
            with self.lock:
                self.response(200, cseq, {"Range": "npt=now-"})
                if self.state != "PLAYING":
                    self.state = "PLAYING"
                    self.media_queue = self.server.media.subscribe()
                    self.sender = threading.Thread(target=self.stream, args=(self.media_queue,), daemon=True)
                    self.sender.start()
        elif method == "PAUSE":
            if self.state != "PLAYING":
                return self.response(455, cseq)
            self.pause()
            self.response(200, cseq)
        elif method == "TEARDOWN":
            self.pause()
            with self.lock:
                if self.transport:
                    self.send_packet(self.packetizer.report(self.timestamp, bye=True), True)
                self.response(200, cseq)
                if self.udp:
                    for sock in self.udp:
                        sock.close()
                self.udp = None
                self.transport = None
                self.session = None
                self.state = "INIT"

    def setup(self, header):
        for alternative in header.split(","):
            parts = [p.strip() for p in alternative.split(";")]
            params = dict(p.split("=", 1) if "=" in p else (p, "") for p in parts[1:])
            if "multicast" in params or params.get("mode", "PLAY").strip('"').upper() != "PLAY":
                continue
            if parts[0].upper() == "RTP/AVP/TCP":
                match = re.fullmatch(r"(\d+)-(\d+)", params.get("interleaved", "0-1"))
                if not match:
                    continue
                channels = tuple(map(int, match.groups()))
                if not (0 <= channels[0] < channels[1] <= 255):
                    continue
                self.channels = channels
                self.transport = "tcp"
                return f"RTP/AVP/TCP;unicast;interleaved={channels[0]}-{channels[1]};ssrc={self.packetizer.ssrc:08X}"
            if parts[0].upper() in ("RTP/AVP", "RTP/AVP/UDP"):
                match = re.fullmatch(r"(\d+)-(\d+)", params.get("client_port", ""))
                if not match:
                    continue
                ports = tuple(map(int, match.groups()))
                if not (1 <= ports[0] < ports[1] <= 65535):
                    continue
                self.udp = udp_pair(self.conn.getsockname()[0])
                self.ports = ports
                self.transport = "udp"
                port = self.udp[0].getsockname()[1]
                return (f"RTP/AVP/UDP;unicast;client_port={ports[0]}-{ports[1]};"
                        f"server_port={port}-{port+1};ssrc={self.packetizer.ssrc:08X}")
        return None

    def send_packet(self, packet, rtcp=False):
        if self.transport == "tcp":
            self.conn.sendall(struct.pack("!BBH", 36, self.channels[int(rtcp)], len(packet)) + packet)
        else:
            self.udp[int(rtcp)].sendto(packet, (self.peer[0], self.ports[int(rtcp)]))

    def stream(self, q):
        waiting_idr = True
        last_report = 0
        try:
            while not self.closed.is_set() and not self.server.media.stop.is_set():
                try:
                    item = q.get(timeout=0.5)
                except queue.Empty:
                    with self.lock:
                        if self.media_queue is not q:
                            return
                    continue
                if item is None:
                    raise ConnectionError("Client cannot keep up with stream")
                nals, timestamp, wall_time = item
                if waiting_idr:
                    if not any(n[0] & 31 == 5 for n in nals):
                        continue
                    waiting_idr = False
                with self.lock:
                    if self.media_queue is not q:
                        return
                    self.timestamp, self.wall_time = timestamp, wall_time
                    for packet in self.packetizer.encode(nals, timestamp):
                        self.send_packet(packet)
                    now = time.monotonic()
                    if now-last_report >= 5:
                        self.send_packet(self.packetizer.report(timestamp, wall_time), True)
                        last_report = now
                    if self.udp:
                        for _ in range(16):
                            try:
                                data, peer = self.udp[1].recvfrom(65535)
                                if peer == (self.peer[0], self.ports[1]):
                                    self.receive_rtcp(data)
                            except BlockingIOError:
                                break
        except (OSError, ConnectionError):
            self.close()

    def pause(self):
        with self.lock:
            if self.media_queue is not None:
                self.server.media.unsubscribe(self.media_queue)
                self.media_queue = None
            self.state = "READY"

    def close(self):
        self.closed.set()
        # Interrupt a blocked send before taking the lock.
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        with self.lock:
            self.pause()
            if self.udp:
                for sock in self.udp:
                    sock.close()
                self.udp = None
            self.conn.close()

    def run(self):
        LOG.info("Client connected: %s:%s", *self.peer)
        try:
            while not self.closed.is_set():
                request = self.read_message()
                LOG.debug("%s %s", request[0], request[1])
                self.handle(*request)
        except (OSError, ValueError, UnicodeError, ConnectionError) as exc:
            LOG.debug("Client ended: %s", exc)
        finally:
            self.close()
            with self.server.lock:
                self.server.clients.discard(self)


class Server:
    def __init__(self, args):
        self.args = args
        self.media = Media(args)
        self.stop = threading.Event()
        self.clients = set()
        self.lock = threading.Lock()
        self.sdp_id = secrets.randbits(32)

    def run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((self.args.host, self.args.port))
                listener.listen(16)
                listener.settimeout(1)
                self.media.start()
                LOG.info("Streaming source %s at rtsp://%s:%d%s", self.args.source,
                         self.args.host, listener.getsockname()[1], self.args.path)
                while not self.stop.is_set() and not self.media.stop.is_set():
                    try:
                        conn, peer = listener.accept()
                    except socket.timeout:
                        continue
                    client = Client(self, conn, peer)
                    with self.lock:
                        if len(self.clients) >= self.args.max_clients:
                            conn.close()
                            continue
                        self.clients.add(client)
                    threading.Thread(target=client.run, daemon=True).start()
                if self.media.error:
                    raise RuntimeError(self.media.error)
        finally:
            self.stop.set()
            with self.lock:
                clients = list(self.clients)
            for client in clients:
                client.close()
            self.media.close()


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="0", help="OpenCV camera index, file/URL, or 'test' (default: 0)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8554)
    parser.add_argument("--path", default="/live")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=25)
    parser.add_argument("--bitrate", default="1500k")
    parser.add_argument("--mtu", type=int, default=1200, help="Maximum RTP packet size")
    parser.add_argument("--timeout", type=int, default=60)
    parser.add_argument("--max-clients", type=int, default=16)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--loop", action="store_true", help="Loop file input")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)
    if args.width < 2 or args.height < 2 or args.width % 2 or args.height % 2:
        parser.error("width and height must be positive even numbers")
    if not math.isfinite(args.fps) or not 0 < args.fps <= 240:
        parser.error("fps must be between 0 and 240")
    if not 256 <= args.mtu <= 65507 or not 1 <= args.port <= 65535:
        parser.error("invalid MTU or port")
    if args.timeout < 5 or args.max_clients < 1:
        parser.error("timeout must be >=5 and max-clients >=1")
    if not args.path.startswith("/") or args.path == "/" or any(c in args.path for c in "?#\r\n "):
        parser.error("path must be a non-root absolute URL path")
    return args


def main():
    args = arguments()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    server = Server(args)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: server.stop.set())
    try:
        server.run()
    except (RuntimeError, OSError) as exc:
        LOG.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
