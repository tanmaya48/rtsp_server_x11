// Live RTSP/1.0 server: OpenCV capture, FFmpeg encoding, socket RTP/RTCP.
//
// Implements the unicast, video-only playback subset of RFC 2326, RFC 3550,
// and RFC 6184. FFmpeg is only an encoder; all network protocols live here.
//
// Build (Linux/macOS), add "-DWITH_X11 -lX11" to enable --source screen:
//   g++ -std=c++17 -O2 -pthread rtsp_server.cpp -o rtsp_server
//       $(pkg-config --cflags --libs opencv4)   (append on the same line)
//
// Run:
//   ./rtsp_server --source test            # synthetic test pattern
//   ./rtsp_server --source 0               # first camera
//   ./rtsp_server --source video.mp4 --loop
//   ./rtsp_server --source screen          # X11 screen share (build with -DWITH_X11 -lX11)
//   ffplay rtsp://localhost:8554/live

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#ifdef WITH_X11
#include "screen_capture.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using Bytes = std::vector<uint8_t>;
using Fields = std::vector<std::pair<std::string, std::string>>;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------
struct NetError : std::runtime_error { using std::runtime_error::runtime_error; };
struct ProtocolError : std::runtime_error { using std::runtime_error::runtime_error; };

static std::string errstr(const std::string &what) {
    return what + ": " + std::strerror(errno);
}

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
enum { LOG_DEBUG = 10, LOG_INFO = 20, LOG_WARNING = 30, LOG_ERROR = 40 };
static std::atomic<int> g_log_level{LOG_INFO};
static std::mutex g_log_mutex;

static void log_msg(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void log_msg(int level, const char *fmt, ...) {
    if (level < g_log_level.load()) return;
    char msg[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    timeval tv;
    gettimeofday(&tv, nullptr);
    tm tmv;
    time_t secs = tv.tv_sec;
    localtime_r(&secs, &tmv);
    char ts[32];
    strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", &tmv);
    const char *name = level >= LOG_ERROR ? "ERROR" : level >= LOG_WARNING ? "WARNING"
                     : level >= LOG_INFO ? "INFO" : "DEBUG";
    std::lock_guard<std::mutex> g(g_log_mutex);
    fprintf(stderr, "%s,%03d %s %s\n", ts, (int)(tv.tv_usec / 1000), name, msg);
}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------
namespace util {

double mono_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double wall_now() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string strip(const std::string &s, const char *chars = " \t\r\n\f\v") {
    size_t b = s.find_first_not_of(chars);
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(chars);
    return s.substr(b, e - b + 1);
}
std::string lower(std::string s) {
    for (auto &c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}
std::string upper(std::string s) {
    for (auto &c : s) c = (char)std::toupper((unsigned char)c);
    return s;
}
std::string rstrip_slash(const std::string &s) {
    size_t e = s.size();
    while (e > 0 && s[e - 1] == '/') --e;
    return s.substr(0, e);
}
// Python-style split: always returns at least one element.
std::vector<std::string> split(const std::string &s, char sep) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        size_t p = s.find(sep, start);
        if (p == std::string::npos) { out.push_back(s.substr(start)); break; }
        out.push_back(s.substr(start, p - start));
        start = p + 1;
    }
    return out;
}
std::vector<std::string> split_str(const std::string &s, const std::string &sep) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        size_t p = s.find(sep, start);
        if (p == std::string::npos) { out.push_back(s.substr(start)); break; }
        out.push_back(s.substr(start, p - start));
        start = p + sep.size();
    }
    return out;
}
std::vector<std::string> split_ws(const std::string &s) {
    std::istringstream is(s);
    std::vector<std::string> out;
    std::string tok;
    while (is >> tok) out.push_back(tok);
    return out;
}
bool is_decimal(const std::string &s) {
    if (s.empty()) return false;
    for (unsigned char c : s) if (c < '0' || c > '9') return false;
    return true;
}
std::string hex(const uint8_t *d, size_t n, bool uppercase = false) {
    static const char *lo = "0123456789abcdef", *up = "0123456789ABCDEF";
    const char *t = uppercase ? up : lo;
    std::string out;
    for (size_t i = 0; i < n; ++i) { out += t[d[i] >> 4]; out += t[d[i] & 15]; }
    return out;
}
std::string hex32_upper(uint32_t v) {
    char b[16];
    snprintf(b, sizeof b, "%08X", v);
    return b;
}
std::string base64(const Bytes &in) {
    static const char *tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    size_t i = 0;
    for (; i + 2 < in.size(); i += 3) {
        uint32_t v = (in[i] << 16) | (in[i + 1] << 8) | in[i + 2];
        out += tbl[v >> 18]; out += tbl[(v >> 12) & 63]; out += tbl[(v >> 6) & 63]; out += tbl[v & 63];
    }
    if (i + 1 == in.size()) {
        uint32_t v = in[i] << 16;
        out += tbl[v >> 18]; out += tbl[(v >> 12) & 63]; out += "==";
    } else if (i + 2 == in.size()) {
        uint32_t v = (in[i] << 16) | (in[i + 1] << 8);
        out += tbl[v >> 18]; out += tbl[(v >> 12) & 63]; out += tbl[(v >> 6) & 63]; out += '=';
    }
    return out;
}
uint64_t random_bits(int bits) {
    std::random_device rd;
    uint64_t v = ((uint64_t)rd() << 32) ^ (uint64_t)rd();
    return bits >= 64 ? v : (v & ((1ULL << bits) - 1));
}
std::string token_hex(size_t nbytes) {
    std::random_device rd;
    Bytes b(nbytes);
    for (auto &x : b) x = (uint8_t)(rd() & 0xff);
    return hex(b.data(), b.size());
}
std::string http_date() {
    time_t t = time(nullptr);
    tm g;
    gmtime_r(&t, &g);
    char buf[64];
    strftime(buf, sizeof buf, "%a, %d %b %Y %H:%M:%S GMT", &g);
    return buf;
}
void put_u16(Bytes &b, uint16_t v) { b.push_back(v >> 8); b.push_back(v & 0xff); }
void put_u32(Bytes &b, uint32_t v) {
    b.push_back(v >> 24); b.push_back((v >> 16) & 0xff); b.push_back((v >> 8) & 0xff); b.push_back(v & 0xff);
}
void set_cloexec(int fd) {
    int f = fcntl(fd, F_GETFD);
    if (f >= 0) fcntl(fd, F_SETFD, f | FD_CLOEXEC);
}
void set_timeouts(int fd, int seconds) {
    timeval tv{seconds, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
}
std::string sockname_ip(int fd) {
    sockaddr_in sa{};
    socklen_t len = sizeof sa;
    if (getsockname(fd, (sockaddr *)&sa, &len) < 0) throw NetError(errstr("getsockname"));
    char buf[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &sa.sin_addr, buf, sizeof buf);
    return buf;
}
// Equivalent of urlsplit(uri).path
std::string url_path(const std::string &uri) {
    std::string rest = uri;
    size_t scheme = rest.find("://");
    if (scheme != std::string::npos && scheme > 0 && std::isalpha((unsigned char)rest[0])) {
        bool ok = true;
        for (size_t i = 0; i < scheme; ++i) {
            unsigned char c = rest[i];
            if (!(std::isalnum(c) || c == '+' || c == '-' || c == '.')) { ok = false; break; }
        }
        if (ok) {
            rest = rest.substr(scheme + 3);
            size_t slash = rest.find_first_of("/?#");
            rest = slash == std::string::npos ? "" : rest.substr(slash);
        }
    }
    size_t cut = rest.find_first_of("?#");
    if (cut != std::string::npos) rest = rest.substr(0, cut);
    return rest;
}
// Matches ^(\d+)-(\d+)$ ; false on mismatch. Out-of-range magnitudes are reported as -1.
bool parse_pair(const std::string &s, long &a, long &b) {
    size_t dash = s.find('-');
    if (dash == std::string::npos) return false;
    std::string x = s.substr(0, dash), y = s.substr(dash + 1);
    if (!is_decimal(x) || !is_decimal(y)) return false;
    a = x.size() > 9 ? -1 : std::stol(x);
    b = y.size() > 9 ? -1 : std::stol(y);
    return true;
}

class Event {
public:
    void set() { { std::lock_guard<std::mutex> g(m_); f_ = true; } c_.notify_all(); }
    bool is_set() { std::lock_guard<std::mutex> g(m_); return f_; }
    bool wait(double seconds) {
        std::unique_lock<std::mutex> l(m_);
        c_.wait_for(l, std::chrono::duration<double>(seconds), [&] { return f_; });
        return f_;
    }
private:
    std::mutex m_;
    std::condition_variable c_;
    bool f_ = false;
};

}  // namespace util
using namespace util;

// ---------------------------------------------------------------------------
// Arguments
// ---------------------------------------------------------------------------
struct Args {
    std::string backend = "auto", source = "0", host = "0.0.0.0", path = "/live", bitrate = "1500k", ffmpeg = "ffmpeg";
    int port = 8554, width = 640, height = 480, mtu = 1200, timeout = 60, max_clients = 16;
    double fps = 25;
    bool loop = false, verbose = false;
    bool width_set = false, height_set = false;
};

static const char *DESCRIPTION =
    "Live RTSP/1.0 server: OpenCV capture, FFmpeg encoding, socket RTP/RTCP.";

static void usage(const char *prog, FILE *out) {
    fprintf(out,
        "usage: %s [-h] [--source SOURCE] [--backend B] [--host HOST] [--port PORT] [--path PATH]\n"
        "          [--width W] [--height H] [--fps FPS] [--bitrate BITRATE] [--mtu MTU]\n"
        "          [--timeout SEC] [--max-clients N] [--ffmpeg FFMPEG] [--loop] [--verbose]\n\n"
        "%s\n\n"
        "  --source SOURCE   OpenCV camera index, file/URL, 'screen' (X11), or 'test' (default: 0)\n"
        "  --backend B       auto|any|v4l2|ffmpeg|gstreamer (default: auto)\n"
        "  --host HOST       (default: 0.0.0.0)\n"
        "  --port PORT       (default: 8554)\n"
        "  --path PATH       (default: /live)\n"
        "  --width W         (default: 640)\n"
        "  --height H        (default: 480)\n"
        "  --fps FPS         (default: 25)\n"
        "  --bitrate B       (default: 1500k)\n"
        "  --mtu MTU         Maximum RTP packet size (default: 1200)\n"
        "  --timeout SEC     (default: 60)\n"
        "  --max-clients N   (default: 16)\n"
        "  --ffmpeg PATH     (default: ffmpeg)\n"
        "  --loop            Loop file input\n"
        "  --verbose\n",
        prog, DESCRIPTION);
}

[[noreturn]] static void arg_error(const char *prog, const std::string &msg) {
    usage(prog, stderr);
    fprintf(stderr, "%s: error: %s\n", prog, msg.c_str());
    std::exit(2);
}

static int to_int(const char *prog, const std::string &name, const std::string &v) {
    char *end = nullptr;
    errno = 0;
    long x = std::strtol(v.c_str(), &end, 10);
    if (v.empty() || *end || errno || x < INT_MIN || x > INT_MAX)
        arg_error(prog, "argument " + name + ": invalid int value: '" + v + "'");
    return (int)x;
}
static double to_double(const char *prog, const std::string &name, const std::string &v) {
    char *end = nullptr;
    errno = 0;
    double x = std::strtod(v.c_str(), &end);
    if (v.empty() || *end || errno == ERANGE)
        arg_error(prog, "argument " + name + ": invalid float value: '" + v + "'");
    return x;
}

static Args parse_args(int argc, char **argv) {
    Args a;
    const char *prog = argv[0];
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i], name = arg, value;
        bool has_val = false;
        if (arg.rfind("--", 0) == 0) {
            size_t eq = arg.find('=');
            if (eq != std::string::npos) { name = arg.substr(0, eq); value = arg.substr(eq + 1); has_val = true; }
        }
        auto need = [&]() -> std::string {
            if (has_val) return value;
            if (i + 1 >= argc) arg_error(prog, "argument " + name + ": expected one argument");
            return argv[++i];
        };
        if (name == "-h" || name == "--help") { usage(prog, stdout); std::exit(0); }
        else if (name == "--source") a.source = need();
        else if (name == "--backend") a.backend = need();
        else if (name == "--host") a.host = need();
        else if (name == "--port") a.port = to_int(prog, name, need());
        else if (name == "--path") a.path = need();
        else if (name == "--width") a.width = to_int(prog, name, need()), a.width_set = true;
        else if (name == "--height") a.height = to_int(prog, name, need()), a.height_set = true;
        else if (name == "--fps") a.fps = to_double(prog, name, need());
        else if (name == "--bitrate") a.bitrate = need();
        else if (name == "--mtu") a.mtu = to_int(prog, name, need());
        else if (name == "--timeout") a.timeout = to_int(prog, name, need());
        else if (name == "--max-clients") a.max_clients = to_int(prog, name, need());
        else if (name == "--ffmpeg") a.ffmpeg = need();
        else if (name == "--loop") a.loop = true;
        else if (name == "--verbose") a.verbose = true;
        else arg_error(prog, "unrecognized arguments: " + arg);
    }
    if (a.backend != "auto" && a.backend != "any" && a.backend != "v4l2" &&
        a.backend != "ffmpeg" && a.backend != "gstreamer")
        arg_error(prog, "argument --backend: invalid choice: '" + a.backend + "'");
    if (a.width < 2 || a.height < 2 || a.width % 2 || a.height % 2)
        arg_error(prog, "width and height must be positive even numbers");
    if (!std::isfinite(a.fps) || !(a.fps > 0 && a.fps <= 240))
        arg_error(prog, "fps must be between 0 and 240");
    if (!(a.mtu >= 256 && a.mtu <= 65507) || !(a.port >= 1 && a.port <= 65535))
        arg_error(prog, "invalid MTU or port");
    if (a.timeout < 5 || a.max_clients < 1)
        arg_error(prog, "timeout must be >=5 and max-clients >=1");
    if (a.path.empty() || a.path[0] != '/' || a.path == "/" ||
        a.path.find_first_of("?#\r\n ") != std::string::npos)
        arg_error(prog, "path must be a non-root absolute URL path");
    return a;
}

// For --source screen, default the stream size to the screen size.
static void apply_screen_defaults(Args &a) {
    if (a.source != "screen") return;
#ifdef WITH_X11
    int w, h;
    ScreenCapture::query_size(w, h);
    if (!a.width_set) a.width = w;
    if (!a.height_set) a.height = h;
#else
    throw std::runtime_error("--source screen requires building with -DWITH_X11 -lX11");
#endif
}

// ---------------------------------------------------------------------------
// Annex B parser: start codes can cross pipe read boundaries.
// ---------------------------------------------------------------------------
class AnnexB {
public:
    std::vector<Bytes> feed(const uint8_t *data, size_t n, bool final = false) {
        buffer_.insert(buffer_.end(), data, data + n);
        struct Match { size_t start, end; };
        std::vector<Match> matches;
        const size_t sz = buffer_.size();
        size_t i = 0;
        while (i + 3 <= sz) {
            if (buffer_[i] == 0 && buffer_[i + 1] == 0) {
                if (buffer_[i + 2] == 1) { matches.push_back({i, i + 3}); i += 3; continue; }
                if (i + 3 < sz && buffer_[i + 2] == 0 && buffer_[i + 3] == 1) {
                    matches.push_back({i, i + 4}); i += 4; continue;
                }
            }
            ++i;
        }
        std::vector<Bytes> nals;
        for (size_t k = 0; k + 1 < matches.size(); ++k) {
            Bytes nal(buffer_.begin() + matches[k].end, buffer_.begin() + matches[k + 1].start);
            while (!nal.empty() && nal.back() == 0) nal.pop_back();
            if (!nal.empty()) nals.push_back(std::move(nal));
        }
        if (!matches.empty()) {
            const Match &last = matches.back();
            if (final) {
                Bytes nal(buffer_.begin() + last.end, buffer_.end());
                while (!nal.empty() && nal.back() == 0) nal.pop_back();
                if (!nal.empty()) nals.push_back(std::move(nal));
                buffer_.clear();
            } else {
                buffer_.erase(buffer_.begin(), buffer_.begin() + last.start);
            }
        }
        return nals;
    }
private:
    Bytes buffer_;
};

// ---------------------------------------------------------------------------
// RTP packetizer (RFC 6184, packetization-mode=1: single NAL + FU-A) and RTCP
// ---------------------------------------------------------------------------
class Packetizer {
public:
    explicit Packetizer(int mtu)
        : mtu_(mtu), seq_((uint16_t)random_bits(16)), ssrc_((uint32_t)random_bits(32)) {}

    uint32_t ssrc() const { return ssrc_; }

    std::vector<Bytes> encode(const std::vector<Bytes> &nals, uint32_t timestamp) {
        std::vector<Bytes> payloads;
        const size_t single_max = (size_t)mtu_ - 12;
        for (const Bytes &nal : nals) {
            if (nal.size() <= single_max) {
                payloads.push_back(nal);
            } else {
                const size_t size = (size_t)mtu_ - 14;
                const size_t data_len = nal.size() - 1;  // data = nal[1:]
                for (size_t offset = 0; offset < data_len; offset += size) {
                    bool end = offset + size >= data_len;
                    uint8_t header = (nal[0] & 31) | (offset == 0 ? 0x80 : 0) | (end ? 0x40 : 0);
                    size_t chunk = std::min(size, data_len - offset);
                    Bytes p;
                    p.reserve(2 + chunk);
                    p.push_back((nal[0] & 0xe0) | 28);
                    p.push_back(header);
                    p.insert(p.end(), nal.begin() + 1 + offset, nal.begin() + 1 + offset + chunk);
                    payloads.push_back(std::move(p));
                }
            }
        }
        std::vector<Bytes> packets;
        packets.reserve(payloads.size());
        for (size_t i = 0; i < payloads.size(); ++i) {
            Bytes pkt;
            pkt.reserve(12 + payloads[i].size());
            pkt.push_back(0x80);
            pkt.push_back(96 | (i + 1 == payloads.size() ? 0x80 : 0));
            put_u16(pkt, seq_);
            put_u32(pkt, timestamp);
            put_u32(pkt, ssrc_);
            pkt.insert(pkt.end(), payloads[i].begin(), payloads[i].end());
            seq_ = (uint16_t)(seq_ + 1);
            ++packets_;
            octets_ += (uint32_t)payloads[i].size();
            packets.push_back(std::move(pkt));
        }
        return packets;
    }

    Bytes report(uint32_t timestamp, std::optional<double> wall_time = std::nullopt, bool bye = false) {
        double now = wall_time ? *wall_time : wall_now();
        double ntp = now + 2208988800.0;
        double int_part;
        double frac = std::modf(ntp, &int_part);
        Bytes r;
        // Sender report
        r.push_back(0x80); r.push_back(200); put_u16(r, 6);
        put_u32(r, ssrc_);
        put_u32(r, (uint32_t)int_part);
        put_u32(r, (uint32_t)(frac * 4294967296.0));
        put_u32(r, timestamp);
        put_u32(r, packets_);
        put_u32(r, octets_);
        // SDES with CNAME
        std::string cname = hex_ssrc() + "@opencv-rtsp";
        Bytes chunk;
        put_u32(chunk, ssrc_);
        chunk.push_back(1);
        chunk.push_back((uint8_t)cname.size());
        chunk.insert(chunk.end(), cname.begin(), cname.end());
        chunk.push_back(0);
        while (chunk.size() % 4) chunk.push_back(0);
        r.push_back(0x81); r.push_back(202); put_u16(r, (uint16_t)(chunk.size() / 4));
        r.insert(r.end(), chunk.begin(), chunk.end());
        if (bye) {
            r.push_back(0x81); r.push_back(203); put_u16(r, 1);
            put_u32(r, ssrc_);
        }
        return r;
    }

private:
    std::string hex_ssrc() const {
        char b[16];
        snprintf(b, sizeof b, "%08x", ssrc_);
        return b;
    }
    int mtu_;
    uint16_t seq_;
    uint32_t ssrc_;
    uint32_t packets_ = 0, octets_ = 0;
};

// ---------------------------------------------------------------------------
// Frame distribution
// ---------------------------------------------------------------------------
struct Unit {
    std::vector<Bytes> nals;
    uint32_t timestamp;
    double wall_time;
};
using UnitPtr = std::shared_ptr<const Unit>;

// Bounded queue. A null UnitPtr is the "consumer too slow" poison pill.
class FrameQueue {
public:
    explicit FrameQueue(size_t cap) : cap_(cap) {}

    void publish(UnitPtr u) {
        {
            std::lock_guard<std::mutex> g(m_);
            if (q_.size() >= cap_) {
                // Disconnect slow consumers rather than corrupting their prediction chain.
                q_.clear();
                q_.push_back(nullptr);
            } else {
                q_.push_back(std::move(u));
            }
        }
        c_.notify_one();
    }

    // Returns false on timeout. On success `out` may be null (poison).
    bool get(UnitPtr &out, double timeout) {
        std::unique_lock<std::mutex> l(m_);
        if (!c_.wait_for(l, std::chrono::duration<double>(timeout), [&] { return !q_.empty(); }))
            return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }

private:
    size_t cap_;
    std::mutex m_;
    std::condition_variable c_;
    std::deque<UnitPtr> q_;
};

// ---------------------------------------------------------------------------
// Media: OpenCV capture -> FFmpeg (x264) -> Annex B -> access units
// ---------------------------------------------------------------------------
class Media {
public:
    explicit Media(const Args &a)
        : args(a), clock_base((uint32_t)random_bits(32)), wall_base(wall_now()) {}
    ~Media() { close(); }

    Event stop, ready;

    void start() {
        const Args &a = args;
        if (a.source == "screen") {
#ifdef WITH_X11
            screen = std::make_unique<ScreenCapture>();
            log_msg(LOG_INFO, "Capturing X11 screen");
#else
            throw std::runtime_error("--source screen requires building with -DWITH_X11 -lX11");
#endif
        } else if (a.source != "test") {
            int api = cv::CAP_ANY;
            if (a.backend == "v4l2") api = cv::CAP_V4L2;
            else if (a.backend == "ffmpeg") api = cv::CAP_FFMPEG;
            else if (a.backend == "gstreamer") api = cv::CAP_GSTREAMER;
#ifdef __linux__
            else if (a.backend == "auto" && is_decimal(a.source)) api = cv::CAP_V4L2;
#endif
            bool ok = false;
            if (is_decimal(a.source)) {
                try { ok = capture.open(std::stoi(a.source), api); } catch (...) { ok = false; }
            } else {
                ok = capture.open(a.source, api);
            }
            if (!ok || !capture.isOpened()) {
                capture.release();
                throw std::runtime_error("Cannot open VideoCapture(" + a.source + ")");
            }
            capture.set(cv::CAP_PROP_FRAME_WIDTH, a.width);
            capture.set(cv::CAP_PROP_FRAME_HEIGHT, a.height);
            capture.set(cv::CAP_PROP_FPS, a.fps);
            use_capture = true;
            log_msg(LOG_INFO, "Capture backend: %s", capture.getBackendName().c_str());
        }
        int gop = std::max(1, (int)std::nearbyint(a.fps));
        std::ostringstream fps;
        fps << std::setprecision(10) << a.fps;
        std::vector<std::string> cmd = {
            a.ffmpeg, "-hide_banner", "-loglevel", "warning", "-f", "rawvideo",
            "-pix_fmt", "bgr24", "-s", std::to_string(a.width) + "x" + std::to_string(a.height),
            "-r", fps.str(), "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "ultrafast",
            "-tune", "zerolatency", "-pix_fmt", "yuv420p", "-profile:v", "baseline",
            "-b:v", a.bitrate, "-g", std::to_string(gop), "-keyint_min", std::to_string(gop),
            "-bf", "0", "-x264-params", "aud=1:repeat-headers=1:scenecut=0", "-f", "h264", "pipe:1"};
        try {
            spawn(cmd);
        } catch (...) {
            capture.release();
            throw;
        }
        wall_base = wall_now();
        threads.emplace_back(&Media::capture_loop, this);
        threads.emplace_back(&Media::read_loop, this);
        threads.emplace_back(&Media::stderr_loop, this);
        if (!ready.wait(10) || has_error()) {
            std::string err = get_error();
            close();
            throw std::runtime_error(err.empty() ? "Encoder produced no SPS/PPS in 10 seconds" : err);
        }
    }

    std::shared_ptr<FrameQueue> subscribe() {
        auto q = std::make_shared<FrameQueue>((size_t)std::max(10L, std::lround(args.fps * 2)));
        std::lock_guard<std::mutex> g(sub_mutex);
        subscribers.insert(q);
        return q;
    }
    void unsubscribe(const std::shared_ptr<FrameQueue> &q) {
        std::lock_guard<std::mutex> g(sub_mutex);
        subscribers.erase(q);
    }

    bool has_error() { std::lock_guard<std::mutex> g(meta_mutex); return has_error_; }
    std::string get_error() { std::lock_guard<std::mutex> g(meta_mutex); return error_; }
    bool get_parameter_sets(Bytes &s, Bytes &p) {
        std::lock_guard<std::mutex> g(meta_mutex);
        if (sps.empty() || pps.empty()) return false;
        s = sps; p = pps;
        return true;
    }

    void close() {
        stop.set();
        if (pid > 0) {
            int status = 0;
            if (waitpid(pid, &status, WNOHANG) == 0) {
                kill(pid, SIGTERM);
                bool exited = false;
                for (int i = 0; i < 150; ++i) {  // up to ~3 s
                    if (waitpid(pid, &status, WNOHANG) != 0) { exited = true; break; }
                    usleep(20000);
                }
                if (!exited) {
                    kill(pid, SIGKILL);
                    waitpid(pid, &status, 0);
                }
            }
            pid = -1;
        }
        for (auto &t : threads) if (t.joinable()) t.join();
        threads.clear();
        for (int *fd : {&in_fd, &out_fd, &err_fd}) {
            if (*fd >= 0) { ::close(*fd); *fd = -1; }
        }
    }

private:
    const Args &args;
    uint32_t clock_base;
    double wall_base;
    cv::VideoCapture capture;
    bool use_capture = false;
#ifdef WITH_X11
    std::unique_ptr<ScreenCapture> screen;
#endif
    pid_t pid = -1;
    int in_fd = -1, out_fd = -1, err_fd = -1;
    std::vector<std::thread> threads;

    std::mutex sub_mutex;
    std::set<std::shared_ptr<FrameQueue>> subscribers;

    std::mutex meta_mutex;
    Bytes sps, pps;
    std::string error_;
    bool has_error_ = false;

    static void make_pipe(int fds[2]) {
        if (pipe(fds) < 0) throw std::runtime_error(errstr("pipe"));
        set_cloexec(fds[0]);
        set_cloexec(fds[1]);
    }

    void spawn(const std::vector<std::string> &cmd) {
        int in[2], out[2], err[2], ep[2];
        make_pipe(in); make_pipe(out); make_pipe(err); make_pipe(ep);
        std::vector<char *> argv;
        for (auto &s : cmd) argv.push_back(const_cast<char *>(s.c_str()));
        argv.push_back(nullptr);

        pid_t child = fork();
        if (child < 0) {
            std::string msg = errstr("fork");
            for (int *p : {in, out, err, ep}) { ::close(p[0]); ::close(p[1]); }
            throw std::runtime_error(msg);
        }
        if (child == 0) {
            dup2(in[0], 0);
            dup2(out[1], 1);
            dup2(err[1], 2);
            long maxfd = sysconf(_SC_OPEN_MAX);
            if (maxfd < 0 || maxfd > 4096) maxfd = 4096;
            for (int fd = 3; fd < maxfd; ++fd) if (fd != ep[1]) ::close(fd);
            signal(SIGPIPE, SIG_DFL);
            execvp(argv[0], argv.data());
            int e = errno;
            ssize_t ignored = ::write(ep[1], &e, sizeof e);
            (void)ignored;
            _exit(127);
        }
        ::close(in[0]); ::close(out[1]); ::close(err[1]); ::close(ep[1]);
        int child_errno = 0;
        ssize_t n;
        do { n = ::read(ep[0], &child_errno, sizeof child_errno); } while (n < 0 && errno == EINTR);
        ::close(ep[0]);
        if (n == (ssize_t)sizeof child_errno) {
            int st;
            waitpid(child, &st, 0);
            ::close(in[1]); ::close(out[0]); ::close(err[0]);
            throw std::runtime_error(std::string("Cannot start '") + cmd[0] + "': " + std::strerror(child_errno));
        }
        pid = child;
        in_fd = in[1];
        out_fd = out[0];
        err_fd = err[0];
    }

    void fail(const std::string &error) {
        if (!stop.is_set()) {
            {
                std::lock_guard<std::mutex> g(meta_mutex);
                error_ = error;
                has_error_ = true;
            }
            log_msg(LOG_ERROR, "Media stopped: %s", error.c_str());
        }
        stop.set();
        ready.set();
    }

    static void normalize(cv::Mat &frame) {
        if (frame.depth() != CV_8U) throw std::runtime_error("Unsupported capture bit depth");
        if (frame.channels() == 1) cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
        else if (frame.channels() == 4) cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
        if (!frame.isContinuous()) frame = frame.clone();
    }

    void capture_loop() {
        const Args &a = args;
        double deadline = mono_now();
        long long count = 0;
        try {
            while (!stop.is_set()) {
                cv::Mat frame;
#ifdef WITH_X11
                if (screen) {
                    frame = screen->grab();
                    if (frame.cols != a.width || frame.rows != a.height) {
                        cv::Mat resized;
                        cv::resize(frame, resized, cv::Size(a.width, a.height), 0, 0, cv::INTER_AREA);
                        frame = resized;
                    }
                } else
#endif
                if (use_capture) {
                    bool ok = capture.read(frame) && !frame.empty();
                    if (!ok) {
                        if (a.loop && !is_decimal(a.source)) {
                            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
                            ok = capture.read(frame) && !frame.empty();
                        }
                        if (!ok) throw std::runtime_error("Capture reached EOF or failed");
                    }
                    cv::Mat resized;
                    cv::resize(frame, resized, cv::Size(a.width, a.height));
                    frame = resized;
                    normalize(frame);
                } else {
                    frame = cv::Mat::zeros(a.height, a.width, CV_8UC3);
                    cv::circle(frame, cv::Point((int)((count * 7) % a.width), a.height / 2), 40,
                               cv::Scalar(0, 200, 255), -1);
                    cv::putText(frame, "OpenCV RTSP frame " + std::to_string(count), cv::Point(20, 45),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                }
                const uint8_t *p = frame.data;
                size_t remaining = (size_t)a.width * a.height * 3;
                while (remaining && !stop.is_set()) {
                    ssize_t w = ::write(in_fd, p, remaining);
                    if (w < 0) {
                        if (errno == EINTR) continue;
                        throw std::runtime_error(errstr("FFmpeg input closed"));
                    }
                    if (w == 0) throw std::runtime_error("FFmpeg input closed");
                    p += w;
                    remaining -= (size_t)w;
                }
                ++count;
                deadline += 1.0 / a.fps;
                stop.wait(std::max(0.0, deadline - mono_now()));
            }
        } catch (const std::exception &e) {
            fail(e.what());
        }
        capture.release();
        if (in_fd >= 0) { ::close(in_fd); in_fd = -1; }
    }

    void read_loop() {
        AnnexB parser;
        std::vector<Bytes> unit;
        long long frame = 0;
        Bytes buf(65536);
        try {
            while (!stop.is_set()) {
                ssize_t n = ::read(out_fd, buf.data(), buf.size());
                if (n < 0) {
                    if (errno == EINTR) continue;
                    throw std::runtime_error(errstr("read from FFmpeg"));
                }
                for (Bytes &nal : parser.feed(buf.data(), (size_t)n, n == 0)) {
                    int kind = nal[0] & 31;
                    if (kind == 9 && !unit.empty()) {
                        bool has_slice = false;
                        for (const Bytes &x : unit) {
                            int k = x[0] & 31;
                            if (k == 1 || k == 5) { has_slice = true; break; }
                        }
                        if (has_slice) {
                            uint32_t ts = (uint32_t)(clock_base + (uint64_t)std::nearbyint((double)frame * 90000.0 / args.fps));
                            double wall = wall_base + (double)frame / args.fps;
                            ++frame;
                            auto u = std::make_shared<const Unit>(Unit{std::move(unit), ts, wall});
                            std::lock_guard<std::mutex> g(sub_mutex);
                            for (auto &q : subscribers) q->publish(u);
                        }
                        unit.clear();
                    }
                    if (kind == 7 || kind == 8) {
                        bool both;
                        {
                            std::lock_guard<std::mutex> g(meta_mutex);
                            if (kind == 7) sps = nal; else pps = nal;
                            both = !sps.empty() && !pps.empty();
                        }
                        if (both) ready.set();
                    }
                    if (kind != 9) unit.push_back(std::move(nal));
                }
                if (n == 0) break;
            }
            if (!stop.is_set()) fail("FFmpeg output closed");
        } catch (const std::exception &e) {
            fail(e.what());
        }
    }

    void stderr_loop() {
        std::string line;
        char buf[4096];
        auto flush = [&] {
            std::string s = strip(line);
            if (!s.empty()) log_msg(LOG_WARNING, "ffmpeg: %s", s.c_str());
            line.clear();
        };
        while (true) {
            ssize_t n = ::read(err_fd, buf, sizeof buf);
            if (n < 0) { if (errno == EINTR) continue; break; }
            if (n == 0) break;
            for (ssize_t i = 0; i < n; ++i) {
                if (buf[i] == '\n') flush(); else line += buf[i];
            }
        }
        flush();
    }
};

// ---------------------------------------------------------------------------
// RTSP server / client sessions
// ---------------------------------------------------------------------------
static std::atomic<bool> g_stop{false};
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "signal flag must be lock-free");
static void on_signal(int) { g_stop = true; }

static const char *METHODS = "OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER";

static const char *reason(int code) {
    switch (code) {
        case 200: return "OK";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 405: return "Method Not Allowed";
        case 406: return "Not Acceptable";
        case 451: return "Parameter Not Understood";
        case 454: return "Session Not Found";
        case 455: return "Method Not Valid in This State";
        case 457: return "Invalid Range";
        case 461: return "Unsupported Transport";
        case 501: return "Not Implemented";
        case 503: return "Service Unavailable";
        case 505: return "RTSP Version Not Supported";
        case 551: return "Option Not Supported";
        default: return "Unknown";
    }
}

// Allocate an even/odd RTP/RTCP UDP port pair.
static std::pair<int, int> udp_pair(const std::string &host) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
        throw NetError("Bad local address: " + host);
    for (int attempt = 0; attempt < 100; ++attempt) {
        int rtp = socket(AF_INET, SOCK_DGRAM, 0);
        int rtcp = socket(AF_INET, SOCK_DGRAM, 0);
        if (rtp < 0 || rtcp < 0) {
            std::string m = errstr("socket");
            if (rtp >= 0) ::close(rtp);
            if (rtcp >= 0) ::close(rtcp);
            throw NetError(m);
        }
        set_cloexec(rtp);
        set_cloexec(rtcp);
        addr.sin_port = 0;
        bool ok = ::bind(rtp, (sockaddr *)&addr, sizeof addr) == 0;
        if (ok) {
            sockaddr_in bound{};
            socklen_t len = sizeof bound;
            getsockname(rtp, (sockaddr *)&bound, &len);
            int port = ntohs(bound.sin_port);
            if (port % 2 || port == 65535) {
                ok = false;
            } else {
                addr.sin_port = htons((uint16_t)(port + 1));
                ok = ::bind(rtcp, (sockaddr *)&addr, sizeof addr) == 0;
            }
        }
        if (ok) {
            fcntl(rtcp, F_SETFL, fcntl(rtcp, F_GETFL) | O_NONBLOCK);
            return {rtp, rtcp};
        }
        ::close(rtp);
        ::close(rtcp);
    }
    throw NetError("Could not allocate an even/odd RTP/RTCP port pair");
}

class Client;

class Server {
public:
    explicit Server(const Args &a) : args(a), media(args), sdp_id((uint32_t)random_bits(32)) {}
    void run();

    Args args;
    Media media;
    std::mutex lock;
    std::set<std::shared_ptr<Client>> clients;
    std::atomic<int> active_threads{0};
    uint32_t sdp_id;
};

struct Request {
    std::string method, uri, version;
    std::map<std::string, std::string> headers;
    std::string body;
};

class Client : public std::enable_shared_from_this<Client> {
public:
    Client(Server &s, int fd, const sockaddr_in &peer_addr)
        : server(s), conn(fd), peer(peer_addr), packetizer(s.args.mtu) {
        char buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &peer.sin_addr, buf, sizeof buf);
        peer_ip = buf;
        peer_port = ntohs(peer.sin_port);
        set_cloexec(conn);
        set_timeouts(conn, 1);
        last_activity = mono_now();
        wall_time = wall_now();
    }
    ~Client() {
        if (udp_rtp >= 0) ::close(udp_rtp);
        if (udp_rtcp >= 0) ::close(udp_rtcp);
        ::close(conn);
    }

    void run() {
        log_msg(LOG_INFO, "Client connected: %s:%d", peer_ip.c_str(), peer_port);
        try {
            while (!closed) {
                Request r = read_message();
                log_msg(LOG_DEBUG, "%s %s", r.method.c_str(), r.uri.c_str());
                handle(r.method, r.uri, r.version, r.headers, r.body);
            }
        } catch (const std::exception &e) {
            log_msg(LOG_DEBUG, "Client ended: %s", e.what());
        }
        close();
    }

    void close() {
        closed = true;
        // Interrupt a blocked send/recv before taking the lock. The fd itself stays
        // open until the destructor so other threads never touch a recycled fd.
        ::shutdown(conn, SHUT_RDWR);
        std::lock_guard<std::recursive_mutex> g(lock);
        pause();
        close_udp();
    }

private:
    Server &server;
    int conn;
    sockaddr_in peer;
    std::string peer_ip;
    int peer_port = 0;
    std::string buffer;
    std::recursive_mutex lock;
    std::atomic<bool> closed{false};
    std::string session;  // empty == None
    std::string state = "INIT";
    std::string transport;  // "", "tcp", "udp"
    int udp_rtp = -1, udp_rtcp = -1;
    std::pair<int, int> channels{0, 1};
    std::pair<int, int> ports{0, 0};
    Packetizer packetizer;
    std::shared_ptr<FrameQueue> media_queue;
    std::atomic<double> last_activity{0};
    uint32_t timestamp = 0;
    double wall_time = 0;

    void close_udp() {
        if (udp_rtp >= 0) { ::close(udp_rtp); udp_rtp = -1; }
        if (udp_rtcp >= 0) { ::close(udp_rtcp); udp_rtcp = -1; }
    }

    void sendall(const void *data, size_t n) {
        const char *p = (const char *)data;
        while (n) {
            ssize_t w = ::send(conn, p, n, MSG_NOSIGNAL);
            if (w < 0) {
                if (errno == EINTR) continue;
                throw NetError(errstr("send"));
            }
            p += w;
            n -= (size_t)w;
        }
    }

    void response(int code, const std::string &cseq, const Fields &extra = {}, const std::string &body = "") {
        Fields fields = {{"CSeq", cseq}, {"Date", http_date()}, {"Server", "OpenCV-RTSP/1.0"}};
        auto put = [&](const std::string &k, const std::string &v) {
            for (auto &f : fields) if (f.first == k) { f.second = v; return; }
            fields.emplace_back(k, v);
        };
        if (!session.empty()) put("Session", session + ";timeout=" + std::to_string(server.args.timeout));
        for (auto &e : extra) put(e.first, e.second);
        put("Content-Length", std::to_string(body.size()));
        std::string data = std::string("RTSP/1.0 ") + std::to_string(code) + " " + reason(code) + "\r\n";
        for (auto &f : fields) data += f.first + ": " + f.second + "\r\n";
        data += "\r\n";
        data += body;
        std::lock_guard<std::recursive_mutex> g(lock);
        sendall(data.data(), data.size());
    }

    Request read_message() {
        while (!closed) {
            if (!buffer.empty() && buffer[0] == '$') {
                if (buffer.size() >= 4) {
                    uint8_t channel = (uint8_t)buffer[1];
                    size_t size = ((uint8_t)buffer[2] << 8) | (uint8_t)buffer[3];
                    if (buffer.size() >= size + 4) {
                        std::string payload = buffer.substr(4, size);
                        buffer.erase(0, size + 4);
                        if (transport == "tcp" && channel == channels.second)
                            receive_rtcp((const uint8_t *)payload.data(), payload.size());
                        continue;
                    }
                }
            } else {
                size_t end = buffer.find("\r\n\r\n");
                if (end != std::string::npos) {
                    if (end > 16384) throw ProtocolError("Header too large");
                    std::string head = buffer.substr(0, end);
                    for (unsigned char c : head)
                        if (c >= 0x80) throw ProtocolError("Non-ASCII header");
                    auto lines = split_str(head, "\r\n");
                    auto first = split_ws(lines[0]);
                    if (first.size() != 3) throw ProtocolError("Bad request line");
                    Request req;
                    req.method = first[0];
                    req.uri = first[1];
                    req.version = first[2];
                    for (size_t i = 1; i < lines.size(); ++i) {
                        size_t colon = lines[i].find(':');
                        if (colon == std::string::npos) throw ProtocolError("Bad header line");
                        std::string key = lower(strip(lines[i].substr(0, colon)));
                        std::string value = strip(lines[i].substr(colon + 1));
                        if (req.headers.count(key)) throw ProtocolError("Duplicate header");
                        req.headers[key] = value;
                    }
                    size_t length = 0;
                    auto cl = req.headers.find("content-length");
                    if (cl != req.headers.end()) {
                        if (!is_decimal(cl->second) || cl->second.size() > 9)
                            throw ProtocolError("Invalid body size");
                        length = (size_t)std::stoul(cl->second);
                    }
                    if (length > 65536) throw ProtocolError("Invalid body size");
                    if (buffer.size() >= end + 4 + length) {
                        req.body = buffer.substr(end + 4, length);
                        buffer.erase(0, end + 4 + length);
                        return req;
                    }
                } else if (buffer.size() > 16384) {
                    throw ProtocolError("Header too large");
                }
            }
            if (mono_now() - last_activity.load() > server.args.timeout)
                throw NetError("Session expired");
            char tmp[8192];
            ssize_t n = ::recv(conn, tmp, sizeof tmp, 0);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
                throw NetError(errstr("recv"));
            }
            if (n == 0) throw NetError("Peer disconnected");
            buffer.append(tmp, (size_t)n);
        }
        throw NetError("Closed");
    }

    // Validate compound packet framing. Receiver reports are accepted;
    // retransmission and congestion adaptation are outside this live subset.
    void receive_rtcp(const uint8_t *data, size_t len) {
        size_t offset = 0;
        while (offset + 4 <= len) {
            uint8_t first = data[offset], kind = data[offset + 1];
            size_t length = ((size_t)data[offset + 2] << 8) | data[offset + 3];
            size_t size = (length + 1) * 4;
            if ((first >> 6) != 2 || offset + size > len) return;
            if (kind >= 200 && kind <= 203) last_activity = mono_now();
            offset += size;
        }
    }

    void handle(const std::string &method, const std::string &uri, const std::string &version,
                const std::map<std::string, std::string> &h, const std::string &body) {
        auto header = [&](const char *k, const std::string &def = "") -> std::string {
            auto it = h.find(k);
            return it == h.end() ? def : it->second;
        };
        std::string cseq = header("cseq", "0");
        if (!is_decimal(cseq)) return response(400, "0");
        if (version != "RTSP/1.0") return response(505, cseq);
        if (!h.count("cseq")) return response(400, "0");
        last_activity = mono_now();

        std::string required;
        {
            std::vector<std::string> vals;
            for (const char *k : {"require", "proxy-require"}) if (h.count(k)) vals.push_back(h.at(k));
            for (size_t i = 0; i < vals.size(); ++i) required += (i ? "," : "") + vals[i];
        }
        if (!required.empty()) return response(551, cseq, {{"Unsupported", required}});

        if (method == "OPTIONS") return response(200, cseq, {{"Public", METHODS}});

        std::string path = rstrip_slash(url_path(uri));
        std::string base = rstrip_slash(server.args.path);
        if (uri != "*" && path != base && path != base + "/trackID=0") return response(404, cseq);

        if (method == "DESCRIBE") {
            std::string accept = header("accept");
            if (!accept.empty()) {
                bool any = false;
                for (const std::string &x : split(accept, ',')) {
                    std::string t = strip(split(x, ';')[0]);
                    if (t == "application/sdp" || t == "*/*" || t == "application/*") { any = true; break; }
                }
                if (!any) return response(406, cseq);
            }
            if (server.media.has_error()) return response(503, cseq);
            Bytes sps, pps;
            if (!server.media.get_parameter_sets(sps, pps)) return response(503, cseq);
            std::string ip = sockname_ip(conn);
            std::string sdp =
                "v=0\r\no=- " + std::to_string(server.sdp_id) + " 1 IN IP4 " + ip + "\r\n"
                "s=OpenCV live stream\r\nc=IN IP4 0.0.0.0\r\nt=0 0\r\n"
                "a=control:*\r\na=range:npt=now-\r\nm=video 0 RTP/AVP 96\r\n"
                "a=rtpmap:96 H264/90000\r\n"
                "a=fmtp:96 packetization-mode=1;profile-level-id=" + hex(sps.data() + 1, 3) +
                ";sprop-parameter-sets=" + base64(sps) + "," + base64(pps) + "\r\n"
                "a=control:trackID=0\r\n";
            std::string content_base = rstrip_slash(uri) + "/";
            return response(200, cseq, {{"Content-Type", "application/sdp"}, {"Content-Base", content_base}}, sdp);
        }

        std::string supplied = split(header("session"), ';')[0];
        if (!supplied.empty() && supplied != session) return response(454, cseq);

        if (method == "SETUP") {
            if (path != base + "/trackID=0") return response(404, cseq);
            if (state != "INIT") return response(455, cseq);
            std::lock_guard<std::recursive_mutex> g(lock);
            auto selected = setup(header("transport"));
            if (!selected) return response(461, cseq);
            session = token_hex(12);
            state = "READY";
            return response(200, cseq, {{"Transport", *selected}});
        }

        {
            static const std::set<std::string> known = {"OPTIONS", "DESCRIBE", "SETUP", "PLAY",
                                                         "PAUSE", "TEARDOWN", "GET_PARAMETER"};
            if (!known.count(method)) return response(501, cseq);
        }
        if (session.empty() || supplied != session) return response(454, cseq);

        if (method == "GET_PARAMETER") {
            return response(strip(body).empty() ? 200 : 451, cseq);
        } else if (method == "PLAY") {
            std::string range = header("range", "npt=now-");
            if (range != "npt=now-" && range != "npt=0-" && range != "npt=0.000-") return response(457, cseq);
            std::string scale = header("scale", "1");
            if (scale != "1" && scale != "1.0") return response(501, cseq);
            if (server.media.has_error()) return response(503, cseq);
            std::lock_guard<std::recursive_mutex> g(lock);
            response(200, cseq, {{"Range", "npt=now-"}});
            if (state != "PLAYING") {
                state = "PLAYING";
                media_queue = server.media.subscribe();
                auto q = media_queue;
                auto self = shared_from_this();
                ++server.active_threads;
                std::thread([self, q] {
                    self->stream(q);
                    --self->server.active_threads;
                }).detach();
            }
        } else if (method == "PAUSE") {
            if (state != "PLAYING") return response(455, cseq);
            pause();
            response(200, cseq);
        } else if (method == "TEARDOWN") {
            pause();
            std::lock_guard<std::recursive_mutex> g(lock);
            if (!transport.empty()) send_packet(packetizer.report(timestamp, std::nullopt, true), true);
            response(200, cseq);
            close_udp();
            transport.clear();
            session.clear();
            state = "INIT";
        }
    }

    std::optional<std::string> setup(const std::string &hdr) {
        for (const std::string &alternative : split(hdr, ',')) {
            std::vector<std::string> parts;
            for (const std::string &p : split(alternative, ';')) parts.push_back(strip(p));
            std::map<std::string, std::string> params;
            for (size_t i = 1; i < parts.size(); ++i) {
                size_t eq = parts[i].find('=');
                if (eq != std::string::npos) params[parts[i].substr(0, eq)] = parts[i].substr(eq + 1);
                else params[parts[i]] = "";
            }
            auto mode = params.find("mode");
            std::string mode_v = mode == params.end() ? "PLAY" : upper(strip(mode->second, "\""));
            if (params.count("multicast") || mode_v != "PLAY") continue;

            std::string spec = upper(parts[0]);
            if (spec == "RTP/AVP/TCP") {
                auto it = params.find("interleaved");
                long a, b;
                if (!parse_pair(it == params.end() ? "0-1" : it->second, a, b)) continue;
                if (!(0 <= a && a < b && b <= 255)) continue;
                channels = {(int)a, (int)b};
                transport = "tcp";
                return "RTP/AVP/TCP;unicast;interleaved=" + std::to_string(a) + "-" + std::to_string(b) +
                       ";ssrc=" + hex32_upper(packetizer.ssrc());
            }
            if (spec == "RTP/AVP" || spec == "RTP/AVP/UDP") {
                auto it = params.find("client_port");
                long a, b;
                if (it == params.end() || !parse_pair(it->second, a, b)) continue;
                if (!(1 <= a && a < b && b <= 65535)) continue;
                close_udp();
                auto pair = udp_pair(sockname_ip(conn));
                udp_rtp = pair.first;
                udp_rtcp = pair.second;
                ports = {(int)a, (int)b};
                transport = "udp";
                sockaddr_in sa{};
                socklen_t len = sizeof sa;
                getsockname(udp_rtp, (sockaddr *)&sa, &len);
                int port = ntohs(sa.sin_port);
                return "RTP/AVP/UDP;unicast;client_port=" + std::to_string(a) + "-" + std::to_string(b) +
                       ";server_port=" + std::to_string(port) + "-" + std::to_string(port + 1) +
                       ";ssrc=" + hex32_upper(packetizer.ssrc());
            }
        }
        return std::nullopt;
    }

    void send_packet(const Bytes &packet, bool rtcp = false) {
        if (transport == "tcp") {
            Bytes frame;
            frame.reserve(packet.size() + 4);
            frame.push_back('$');
            frame.push_back((uint8_t)(rtcp ? channels.second : channels.first));
            put_u16(frame, (uint16_t)packet.size());
            frame.insert(frame.end(), packet.begin(), packet.end());
            sendall(frame.data(), frame.size());
        } else {
            int fd = rtcp ? udp_rtcp : udp_rtp;
            if (fd < 0) throw NetError("UDP socket closed");
            sockaddr_in to = peer;
            to.sin_port = htons((uint16_t)(rtcp ? ports.second : ports.first));
            if (::sendto(fd, packet.data(), packet.size(), 0, (sockaddr *)&to, sizeof to) < 0)
                throw NetError(errstr("sendto"));
        }
    }

    void stream(std::shared_ptr<FrameQueue> q) {
        bool waiting_idr = true;
        double last_report = -1e9;
        Bytes rbuf(65535);
        try {
            while (!closed && !server.media.stop.is_set()) {
                UnitPtr item;
                if (!q->get(item, 0.5)) {
                    std::lock_guard<std::recursive_mutex> g(lock);
                    if (media_queue != q) return;
                    continue;
                }
                if (!item) throw NetError("Client cannot keep up with stream");
                if (waiting_idr) {
                    bool idr = false;
                    for (const Bytes &n : item->nals) if ((n[0] & 31) == 5) { idr = true; break; }
                    if (!idr) continue;
                    waiting_idr = false;
                }
                std::lock_guard<std::recursive_mutex> g(lock);
                if (media_queue != q) return;
                timestamp = item->timestamp;
                wall_time = item->wall_time;
                for (const Bytes &packet : packetizer.encode(item->nals, item->timestamp))
                    send_packet(packet);
                double now = mono_now();
                if (now - last_report >= 5) {
                    send_packet(packetizer.report(item->timestamp, item->wall_time), true);
                    last_report = now;
                }
                if (udp_rtcp >= 0) {
                    for (int i = 0; i < 16; ++i) {
                        sockaddr_in from{};
                        socklen_t flen = sizeof from;
                        ssize_t n = ::recvfrom(udp_rtcp, rbuf.data(), rbuf.size(), 0, (sockaddr *)&from, &flen);
                        if (n < 0) {
                            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) break;
                            throw NetError(errstr("recvfrom"));
                        }
                        if (from.sin_addr.s_addr == peer.sin_addr.s_addr && ntohs(from.sin_port) == ports.second)
                            receive_rtcp(rbuf.data(), (size_t)n);
                    }
                }
            }
        } catch (const std::exception &) {
            close();
        }
    }

    void pause() {
        std::lock_guard<std::recursive_mutex> g(lock);
        if (media_queue) {
            server.media.unsubscribe(media_queue);
            media_queue.reset();
        }
        state = "READY";
    }
};

static void resolve_ipv4(const std::string &host, int port, sockaddr_in &out) {
    addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    int rc = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res);
    if (rc != 0) throw std::runtime_error(std::string("Cannot resolve host: ") + gai_strerror(rc));
    out = *(sockaddr_in *)res->ai_addr;
    freeaddrinfo(res);
}

void Server::run() {
    int listener = -1;
    auto cleanup = [&] {
        g_stop = true;
        if (listener >= 0) { ::close(listener); listener = -1; }
        std::vector<std::shared_ptr<Client>> snapshot;
        {
            std::lock_guard<std::mutex> g(lock);
            snapshot.assign(clients.begin(), clients.end());
        }
        for (auto &c : snapshot) c->close();
        media.close();
        for (int i = 0; i < 150 && active_threads.load() > 0; ++i) usleep(20000);
    };
    try {
        listener = socket(AF_INET, SOCK_STREAM, 0);
        if (listener < 0) throw std::runtime_error(errstr("socket"));
        set_cloexec(listener);
        int one = 1;
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        sockaddr_in addr{};
        resolve_ipv4(args.host, args.port, addr);
        if (::bind(listener, (sockaddr *)&addr, sizeof addr) < 0) throw std::runtime_error(errstr("bind"));
        if (::listen(listener, 16) < 0) throw std::runtime_error(errstr("listen"));
        media.start();
        sockaddr_in bound{};
        socklen_t blen = sizeof bound;
        getsockname(listener, (sockaddr *)&bound, &blen);
        log_msg(LOG_INFO, "Streaming source %s at rtsp://%s:%d%s", args.source.c_str(),
                args.host.c_str(), ntohs(bound.sin_port), args.path.c_str());

        while (!g_stop && !media.stop.is_set()) {
            pollfd pfd{listener, POLLIN, 0};
            int r = ::poll(&pfd, 1, 1000);
            if (r <= 0) continue;  // timeout or EINTR
            sockaddr_in peer{};
            socklen_t plen = sizeof peer;
            int conn = ::accept(listener, (sockaddr *)&peer, &plen);
            if (conn < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK || errno == ECONNABORTED) continue;
                throw std::runtime_error(errstr("accept"));
            }
            std::shared_ptr<Client> client;
            {
                std::lock_guard<std::mutex> g(lock);
                if ((int)clients.size() >= args.max_clients) { ::close(conn); continue; }
                client = std::make_shared<Client>(*this, conn, peer);
                clients.insert(client);
            }
            ++active_threads;
            std::thread([this, client] {
                client->run();
                {
                    std::lock_guard<std::mutex> g(lock);
                    clients.erase(client);
                }
                --active_threads;
            }).detach();
        }
        if (media.has_error()) throw std::runtime_error(media.get_error());
    } catch (...) {
        cleanup();
        throw;
    }
    cleanup();
}

int main(int argc, char **argv) {
    Args args = parse_args(argc, argv);
    g_log_level = args.verbose ? LOG_DEBUG : LOG_INFO;
    signal(SIGPIPE, SIG_IGN);

    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    try {
        apply_screen_defaults(args);
        Server server(args);
        server.run();
    } catch (const std::exception &e) {
        log_msg(LOG_ERROR, "%s", e.what());
        return 1;
    }
    return 0;
}
