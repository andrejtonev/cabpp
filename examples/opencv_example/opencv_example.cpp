/**
 * @brief Example for the CABpp library using OpenCV
 *
 * This example showcases the benefits of using CAB for inter-thread
 * communication. Black and white 800x1500 frames are generated. These frames
 * are later used by two separate threads. One simply displays the frame,
 * while the other processes the frame and displays the result.
 *
 * Two inter-communication types are used: CAB, a simple mutex blocking a
 * global Mat variable used later by the two threads.
 * Using the CAB, there are no run-time allocations; therefor depending on 
 * the penalty of allocating a new frame each time it is generated, the 
 * benefits can be greater or smaller. Larger the image, greater the benefits.
 *
 * @author Andreja Tonev
 *
 */
#include "cabpp.h"
#include "simple_ui.h"
#include "video_func.h"

int main(int argc, char** argv) {
  constexpr int kHeight = 800;   //!< Generated frame height
  constexpr int kWidth = 1500;   //!< Generated frame width
  constexpr int kBGColor = 200;  //!< Background color

  std::atomic_bool use_cab(true);     //!< Flag enabling/disabling CAB
  std::atomic_bool run(false);         //!< Flag enabling/disabling example
  std::atomic_bool main_ready(false);  //!< True once the first img is generated

  uint16_t gen_step_ms = 3;  //!< Frame generation thread step in milliseconds

  cabpp::CABpp<cabpp::Frame> cab(4, kHeight, kWidth, CV_8UC1, kBGColor);

  std::string main_win = "Live feed";
  std::string proc_win = "Processed feed";

  cv::namedWindow(main_win);
  cv::namedWindow(proc_win);

  auto gen_thread = std::thread(cabpp::generate_frames,
                                std::ref(use_cab),
                                std::ref(cab),
                                kWidth,
                                kHeight,
                                std::ref(gen_step_ms),
                                std::ref(run),
                                &main_ready);

  auto draw_thread = std::thread(cabpp::draw_frames,
                                 std::ref(use_cab),
                                 std::ref(cab),
                                 main_win,
                                 std::ref(run),
                                 std::ref(main_ready));

  auto proc_thread = std::thread(cabpp::process_frames,
                                 std::ref(use_cab),
                                 std::ref(cab),
                                 kWidth,
                                 kHeight,
                                 proc_win,
                                 std::ref(run),
                                 std::ref(main_ready));

  run = true;

  ui::RunUI(use_cab, gen_step_ms);

  run = false;

  if (gen_thread.joinable()) gen_thread.join();
  if (draw_thread.joinable()) draw_thread.join();
  if (proc_thread.joinable()) proc_thread.join();

  cv::destroyWindow(main_win);
  cv::destroyWindow(proc_win);

  return 0;
}
