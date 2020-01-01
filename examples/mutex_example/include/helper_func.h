/** 
 * @brief Header implementing video example functionalities.
 *
 * @author Andreja Tonev
 */
#ifndef _CABPP_MUTEX_EXAMPLE_HELPER_F_H_
#define _CABPP_MUTEX_EXAMPLE_HELPER_F_H_

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>


/**************************Global non-const variables*************************/
static std::atomic_bool go;

/******************************Helper functions*******************************/
/**Call the passed read function and keep track of the calling times.
 * @param func(in): a function to call
 * @param duration(in): thread duration in us
 * @param f_time_sum(out): OPTIONAL - output time spent in the func call
 * @param f_loops(out): OPTIONAL - number of times func has been called
 */
template<typename F>
void FuncThread(F func, 
                long duration,
                unsigned long long* f_time_sum,
                unsigned long long* f_loops) {
  unsigned long long complete = 0; 
  unsigned long long loops = 0;
  using clock = std::chrono::high_resolution_clock;
  using res = std::chrono::nanoseconds;
  std::chrono::time_point<clock> start; 
  std::chrono::time_point<clock> finish;
  auto th_duration = std::chrono::microseconds(duration);
  while (!go)
    ;
  auto th_start = clock::now() + th_duration;
  while (finish < th_start) { 
    start = clock::now();
    auto r = func(loops);
    finish = clock::now();
    complete += std::chrono::duration_cast<res>(finish - start).count();
    ++loops;
    std::this_thread::yield();
  } 
  if (f_time_sum) *f_time_sum = complete;
  if (f_loops) *f_loops = loops;
}

/**Simple function that sets up all reader and writer threads.
 * @param name(in): test's name; gets printed at the beginning
 * @param write_f(in): write function to call from thread
 * @param read_f(in): read function to call from thread
 * @param n_write(in): number of write threads to create
 * @param n_read(in): number of read threads to create
 */
template<typename FW, typename FR>
void RunTest(const std::string& name, 
             FW write_f, 
             FR read_f, 
             int n_write, 
             int n_read,
             long test_duration_us) {
  std::cout << name << std::endl;
  //Create all threads
  go = false;
  std::thread threads[n_write + n_read];
  unsigned long long write_times[n_write] = {0};
  unsigned long long read_times[n_read] = {0};
  unsigned long long write_loops[n_write] = {0};
  unsigned long long read_loops[n_read] = {0};
  for (int i = 0; i < n_write; ++i) {
    threads[i] = std::thread(FuncThread<FW>, 
                             write_f, 
                             test_duration_us, 
                             write_times + i, 
                             write_loops + i);
  }
  for (int i = 0; i < n_read; ++i) {
    threads[i + n_write] = std::thread(FuncThread<FR>, 
                             read_f, 
                             test_duration_us, 
                             read_times + i, 
                             read_loops + i);
  }
  //Launch the thread loops
  go = true;
  //Join all threads
  for (auto& th : threads) {
    if (th.joinable()) th.join();
  }
  //Report
  //Sum up the times and loop counts
  unsigned long long write_time_sum = 0;
  unsigned long long write_loops_sum = 0;
  unsigned long long read_time_sum = 0;
  unsigned long long read_loops_sum = 0;
  for (int i = 0; i < n_write; ++i) {
    write_time_sum += write_times[i];
    write_loops_sum += write_loops[i];
  }
  for (int i = 0; i < n_read; ++i) {
    read_time_sum += read_times[i];
    read_loops_sum += read_loops[i];
  }
  std::cout << "Read: " << read_time_sum/read_loops_sum << "ns (" << read_loops_sum << " calls)" 
            << std::endl;
  std::cout << "Write: " << write_time_sum/write_loops_sum << "ns (" << write_loops_sum << " calls)" 
            << std::endl;
}

#endif //_CABPP_MUTEX_EXAMPLE_HELPER_F_H_
