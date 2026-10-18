// X11 root-window screen grabber, used by rtsp_server.cpp for `--source screen`.
// Build the server with: -DWITH_X11 -lX11
#pragma once

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <stdexcept>
#include <string>

// Xlib's default error handler calls exit(). Record the error instead so a
// failed XGetImage becomes an ordinary exception on the capture thread.
inline std::atomic<int> g_x_error{0};
inline int screen_capture_x_error_handler(Display *, XErrorEvent *e) {
    g_x_error = e->error_code;
    return 0;
}

class ScreenCapture {
public:
    ScreenCapture() {
        XSetErrorHandler(screen_capture_x_error_handler);
        display_ = XOpenDisplay(nullptr);
        if (!display_) throw std::runtime_error("Cannot open X display (is $DISPLAY set?)");
        root_ = DefaultRootWindow(display_);
    }
    ~ScreenCapture() {
        if (display_) XCloseDisplay(display_);
    }
    ScreenCapture(const ScreenCapture &) = delete;
    ScreenCapture &operator=(const ScreenCapture &) = delete;

    // Screen size, rounded down to even numbers (x264 yuv420p needs even dimensions).
    static void query_size(int &width, int &height) {
        Display *d = XOpenDisplay(nullptr);
        if (!d) throw std::runtime_error("Cannot open X display (is $DISPLAY set?)");
        int screen = DefaultScreen(d);
        width = DisplayWidth(d, screen) & ~1;
        height = DisplayHeight(d, screen) & ~1;
        XCloseDisplay(d);
    }

    // Grab the whole root window as a continuous BGR frame.
    cv::Mat grab() {
        // Re-read the size each time so a resolution change doesn't cause BadMatch.
        XWindowAttributes attrs;
        if (!XGetWindowAttributes(display_, root_, &attrs))
            throw std::runtime_error("XGetWindowAttributes failed");

        g_x_error = 0;
        XImage *image = XGetImage(display_, root_, 0, 0, attrs.width, attrs.height, AllPlanes, ZPixmap);
        if (!image || g_x_error != 0) {
            if (image) XDestroyImage(image);
            throw std::runtime_error("XGetImage failed (X error code " + std::to_string(g_x_error.load()) + ")");
        }
        if (image->bits_per_pixel != 32) {
            int bpp = image->bits_per_pixel;
            XDestroyImage(image);
            throw std::runtime_error("Unsupported X visual: " + std::to_string(bpp) + " bits per pixel");
        }
        cv::Mat bgra(attrs.height, attrs.width, CV_8UC4, image->data, image->bytes_per_line);
        cv::Mat bgr;
        cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);  // copies out of the XImage
        XDestroyImage(image);
        return bgr;
    }

private:
    Display *display_ = nullptr;
    Window root_ = 0;
};
