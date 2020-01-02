/**
 * @brief Header implementing video example functionalities.
 *
 * @author Andreja Tonev
 */
#ifndef _CABPP_VIDEO_EXAMPLE_H_
#define _CABPP_VIDEO_EXAMPLE_H_

#include "cabpp.h"
#include "simple_ui.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace cabpp {
/***************************Helper class definitions***************************/
/**
 * Frame class holding the "raw" image and a frame number associated with it.
 */
class Frame {
 public:
  template <typename... Args>
  Frame(Args... args) : fn(0), frame(args...) {}
  Frame() = default;

  uint64_t fn;
  cv::Mat frame;
};

/**
 * Class used to calculate the FPS of a thread. Used to calculate the speed of
 * each video example thread.
 */
class Fps {
  using clock = std::chrono::high_resolution_clock;
  using time_res = std::chrono::seconds;
  using ts_format_t = std::chrono::time_point<clock, time_res>;

 public:
  Fps(std::string name = "", uint8_t row = 0) : name_(name), row_(row) {}

  void Frame(ts_format_t ts) {
    if (ts == current_ts_) {
      ++current_fps_;
    } else {
      current_ts_ = ts;
      last_fps_ = current_fps_;
      current_fps_ = 0;
      if (name_.size()) {
        ui::PrintOnLine(name_ + ": " + std::to_string(last_fps_) + "fps", row_);
      }
    }
  }

  uint16_t Get(void) { return last_fps_; }

 private:
  std::string name_;
  uint8_t row_;
  ts_format_t current_ts_;
  uint16_t last_fps_ = 0;
  uint16_t current_fps_ = 0;
};

/****************************Global const variables****************************/

/**************************Global non-const variables**************************/
/**
 * Simple implementation of inter-thread communication (passing Mat)
 */
std::mutex frame_mtx_;
Frame frame_;
void push_frame(cv::Mat frame, uint64_t fn) {
  std::unique_lock<std::mutex> lock(frame_mtx_);
  frame_.frame = frame;
  frame_.fn = fn;
}

/********************************Main functions********************************/
/**
 * Loop generating black and white test images.
 * Maximum frequency = 1/stem_ms kHz.
 * @param use_cab: atomic flag that enables/disables the usage of CABpp
 * @param cab: CABpp used for inter-thread communication
 * @param width: width of the test image
 * @param height: height of the test image
 * @param step_ms: the minimum period if the loop
 * @param run: flag used to launch and stop the loop
 * @param main_ready(out): flag set to true once the first image has been
 * generated
 */
void generate_frames(std::atomic_bool& use_cab,
                     CABpp<Frame>& cab,
                     const uint16_t width,
                     const uint16_t height,
                     // TODO: Make it atomic
                     uint16_t& step_ms,
                     std::atomic_bool& run,
                     std::atomic_bool* main_ready) {
  Fps fps("GEN ", 10);
  uint64_t fn_ = 1;
  cv::Point2f rec_points_[4];
  cv::Scalar bg_color(200.0);
  cv::Scalar fw_color(100.0);

  // Push one frame (avoids segfault on cab on/off switch)
  push_frame(cv::Mat(height, width, CV_8UC1, bg_color), 0);

  // spin-lock - wait for user's start command
  while (!run)
    ;

  auto gen_frame = [&](cv::Mat& frame) {
    for (int c_i = 0; c_i < 10; ++c_i) {
      int k = 1 << c_i;
      float k_f = static_cast<float>(k) / 512.0;
      cv::circle(frame, {width / 2, height / 2}, fn_ * k_f, fw_color);
    }
    auto rec = cv::RotatedRect(
        {static_cast<float>(width) / 2, static_cast<float>(height) / 2},
        {100.0, 100.0},
        fn_);
    rec.points(rec_points_);
    std::vector<std::vector<cv::Point>> rec_{
        {{static_cast<int>(rec_points_[0].x),
          static_cast<int>(rec_points_[0].y)},
         {static_cast<int>(rec_points_[1].x),
          static_cast<int>(rec_points_[1].y)},
         {static_cast<int>(rec_points_[2].x),
          static_cast<int>(rec_points_[2].y)},
         {static_cast<int>(rec_points_[3].x),
          static_cast<int>(rec_points_[3].y)}}};
    cv::drawContours(frame, rec_, 0, fw_color, CV_FILLED);
  };

  // Main loop
  for (int i = 0; run; ++i) {
    auto start = std::chrono::high_resolution_clock::now();
    bool use_cab_ = use_cab;

    // Generate a simple black and white image
    if (use_cab_) {
      static const cv::Mat clean(height, width, CV_8UC1, bg_color);
      auto frame_ptr = cab.Reserve();
      if (!frame_ptr) {
        std::cerr << "Reserve failed" << std::endl;
        continue;
      }
      auto& frame = frame_ptr->frame;
      clean.copyTo(frame);
      gen_frame(frame);
      frame_ptr->fn = fn_;
      if (!cab.Write(frame_ptr)) {
        std::cerr << "Write frame failed" << std::endl;
        continue;
      }
    } else {
      cv::Mat frame(height, width, CV_8UC1, bg_color);
      gen_frame(frame);
      push_frame(frame, fn_);
    }

    ++fn_;
    fps.Frame(std::chrono::time_point_cast<std::chrono::seconds>(start));

    if (i == 0) {
      *main_ready = true;
    }

    std::this_thread::sleep_until(start + std::chrono::milliseconds(step_ms));
  }
}

/**
 * Loop displaying the raw black and white test images (faster loop).
 * @param use_cab: atomic flag that enables/disables the usage of CABpp
 * @param cab: CABpp used for inter-thread communication
 * @param win_name: name of the window where the images are displayed
 * @param run: flag used to launch and stop the loop
 * @param main_ready: flag set to true once the first image has been generated
 * @note: Maximum frequency = 1kHz; due to the cv::waitKey function
 */
void draw_frames(std::atomic_bool& use_cab,
                 CABpp<Frame>& cab,
                 const std::string& win_name,
                 std::atomic_bool& run,
                 std::atomic_bool& main_ready) {
  Fps fps("DRAW", 11);
  uint64_t fn_prev = 0;
  // spin-lock - wait for the frame generation thread
  while (!main_ready)
    ;

  // Main loop
  for (int i = 0; run; ++i) {
    uint64_t fn = 0;
    decltype(cab.Read()) frame_ptr;
    cv::Mat frame;
    bool use_cab_ = use_cab;

    // Get frame
    if (use_cab_) {
      frame_ptr = cab.Read();
      //       auto& frame = frame_ptr->frame;
      fn = frame_ptr->fn;
    } else {
      std::unique_lock<std::mutex> lock(frame_mtx_);
      frame = frame_.frame;
      fn = frame_.fn;
    }

    if (fn != fn_prev) {
      fn_prev = fn;
      // Display image (faster thread)
      cv::imshow(win_name, (use_cab_ ? frame_ptr->frame : frame));
      cv::waitKey(1);
      if (use_cab_) {
        // Release the CAB slot as soon as possible
        frame_ptr.reset();
      }
      // FPS calculation
      fps.Frame(std::chrono::time_point_cast<std::chrono::seconds>(
          std::chrono::high_resolution_clock::now()));
    }
    if (use_cab_) {
      // Release the CAB slot as soon as possible
      frame_ptr.reset();
    }
    std::this_thread::yield();
  }
}

/**
 * Loop processing the test images and displaying the results (slower loop).
 * @param use_cab: atomic flag that enables/disables the usage of CABpp
 * @param cab: CABpp used for inter-thread communication
 * @param width: width of the test image
 * @param height: height of the test image
 * @param win_name: name of the window where the images are displayed
 * @param run: flag used to launch and stop the loop
 * @param main_ready: flag set to true once the first image has been generated
 * @note: Maximum frequency = 1kHz; due to the cv::waitKey function
 */
void process_frames(std::atomic_bool& use_cab,
                    CABpp<Frame>& cab,
                    const uint16_t width,
                    const uint16_t height,
                    const std::string& win_name,
                    std::atomic_bool& run,
                    std::atomic_bool& main_ready) {
  Fps fps("PROC", 12);
  uint64_t fn_prev = 0;
  // spin-lock - wait for frame generation thread
  while (!main_ready)
    ;

  // Main loop
  for (int i = 0; run; ++i) {
    uint64_t fn = 0;
    decltype(cab.Read()) frame_ptr;
    cv::Mat frame;
    bool use_cab_ = use_cab;

    // Get frame
    if (use_cab_) {
      frame_ptr = cab.Read();
      //         auto& frame = frame_ptr->frame;
      fn = frame_ptr->fn;
    } else {
      std::unique_lock<std::mutex> lock(frame_mtx_);
      frame = frame_.frame;
      fn = frame_.fn;
    }

    // Process image (slower thread)
    if (fn != fn_prev) {
      fn_prev = fn;
      cv::Mat bw = (use_cab ? frame_ptr->frame : frame) < 180;
      if (use_cab_) {
        // Release the CAB slot as soon as possible
        frame_ptr.reset();
      }
      cv::Mat labelImage(height, width, CV_32S);
      int nLabels = cv::connectedComponents(bw, labelImage, 8);
      std::vector<cv::Vec3b> colors(nLabels);
      colors[0] = cv::Vec3b(0, 0, 0);  // background
      for (int label = 1; label < nLabels; ++label) {
        auto color_from_id = [](int id) {
          uint32_t color = 0x83a47f * (id + 1);
          return cv::Vec3b(
              color & 255, (color >> 8) & 255, (color >> 16) & 255);
        };
        colors[label] = cv::Vec3b(color_from_id(label));
      }
      cv::Mat dst(height, width, CV_8UC3);
      for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {
          int label = labelImage.at<int>(r, c);
          cv::Vec3b& pixel = dst.at<cv::Vec3b>(r, c);
          pixel = colors[label];
        }
      }

      cv::Mat dst_resized;
      cv::resize(dst, dst_resized, {width / 3, height / 3});
      cv::imshow(win_name, dst_resized);
      cv::waitKey(1);

      fps.Frame(std::chrono::time_point_cast<std::chrono::seconds>(
          std::chrono::high_resolution_clock::now()));
    }
    if (use_cab_) {
      // Release the CAB slot as soon as possible
      frame_ptr.reset();
    }
    std::this_thread::yield();
  }
}

}  // namespace cabpp

#endif  //_CABPP_VIDEO_EXAMPLE_H_
