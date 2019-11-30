/**
 * @brief Main example for the CABpp library
 *
 * This example showcases the CABpp's performance when compared to a class
 * whose inter-threaded synchronization is a simple mutex lock.
 *
 * @author Andreja Tonev
 *
 * @version 1.0.0
 *
 * @date 08/10/2019
 */
#include "cabpp.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

/****************************Global const variables***************************/
constexpr long kTestDuration_us = 1e6;
constexpr long kTestStep_us = 100;
constexpr auto kTestTime = std::chrono::microseconds(kTestDuration_us);

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
             int n_read) {
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
                             kTestDuration_us, 
                             write_times + i, 
                             write_loops + i);
  }
  for (int i = 0; i < n_read; ++i) {
    threads[i + n_write] = std::thread(FuncThread<FR>, 
                             read_f, 
                             kTestDuration_us, 
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

/***************************MutexClass implementati***************************/
/**Simple class used to showcase an implementation using mutex locks for 
 * inter-thread synchronization.
 * @note: This class is not a full implementation (and would never even work in
 * the current form) it is just there for comparison.
 */
template <class T>
class MutexClass {
public:
  template<typename... Args>
  MutexClass (long busy_loops, Args&&... args) : 
    t_(std::forward<Args>(args)...), busy_loops_(busy_loops) {}
  
  /**Return a shared pointer to the value and block for busy_loops_ loops.
   * Simulates the user copying the pointer and using the value for some time.
   * That whole time the user should be blocking write access to the file.
   * @note: using the nop loop allows the function to block without allowing 
   * a context switch
   */
  std::shared_ptr<T> ReadPtr() {
    std::lock_guard<std::mutex> lock(mtx_);
    //This way we block without a context switch
    for (int i = 0; i < busy_loops_; ++i) {
      asm("nop");
    }
    //Override delete function to not delete out object
    return std::shared_ptr<T>(&t_, [](T*){});
  }
  
  /**Lock and copy over the object.
   */
  T ReadCopy() {
    std::lock_guard<std::mutex> lock(mtx_);
    return t_;
  }
  
  /**Block and copy over the object.
   */
  void Write(const T& in) {
    std::unique_lock<std::mutex> lock(mtx_);
    t_ = in;
  }
  
  /**Block and move the object.
   */
  void Write(T&& in) {
    std::unique_lock<std::mutex> lock(mtx_);
    t_ = std::move(in);
  }
  
private:
  std::mutex mtx_; //Used for locking
  T t_; //Important object
  long busy_loops_; //Number of nop loops in Read
};

/********************************Main function********************************/
int main(int argc, char **argv) {
  using namespace std::placeholders;
  std::cout << "*********************CABpp test program**********************\n" 
            << "This program should showcase the advantages and disadvantages\n"
            << "of a CAB architecture.\n"
            << "The focus here is on performance;\n"
            << "the time it takes to execute each read/write operation.\n"
            << "The program will run through a few cases with different types\n"
            << "and different blocking times (see MutexClass implementation).\n"
            << "*************************************************************\n"
            << std::endl;
  
  std::cout << "*******************Integer (fast copy/read)******************" 
            << std::endl;
  
/*********************************Integer***********************************/
  cabpp::CABpp<int> cab_i(3, 100);
  RunTest("CABpp<int> 1 reader 1 writer...", 
          [&cab_i](int in)->int{
            cab_i.Write(in);
            return 0;
          }, 
          [&cab_i](int)->int{
            auto r = cab_i.Read();
            return *r;
          },
          1, 
          1);
  
  std::cout << std::endl;
  
  MutexClass<int> mtx_i_0(0, 100);
  RunTest("MutexClass<int> 1 reader(0 nop) 1 writer...", 
          [&mtx_i_0](int in)->int{
            mtx_i_0.Write(in);
            return 0;
          },
          [&mtx_i_0](int in)->int{
            auto r = mtx_i_0.ReadPtr();
            return *r;
          },
          1, 
          1);
  
  MutexClass<int> mtx_i_100(100, 100);
  RunTest("MutexClass<int> 1 reader(100 nop) 1 writer...", 
          [&mtx_i_100](int in)->int{
            mtx_i_100.Write(in);
            return 0;
          },
          [&mtx_i_100](int in)->int{
            auto r = mtx_i_100.ReadPtr();
            return *r;
          },
          1, 
          1);
  
  MutexClass<int> mtx_i_10000(10000, 100);
  RunTest("MutexClass<int> 1 reader(10000 nop) 1 writer...", 
          [&mtx_i_10000](int in)->int{
            mtx_i_10000.Write(in);
            return 0;
          },
          [&mtx_i_10000](int in)->int{
            auto r = mtx_i_10000.ReadPtr();
            return *r;
          },
          1, 
          1);
  
  std::cout << std::endl;
  
  std::cout << "*******************String (fast copy/read)******************" 
            << std::endl;
/************************************String***********************************/
  cabpp::CABpp<std::string> cab_str(9, "100");
  RunTest("CABpp<string> 3 readers 5 writers...", 
          [&cab_str](int in)->int{
            cab_str.Write(std::to_string(in));
            return 0;
          }, 
          [&cab_str](int)->int{
            auto r = cab_str.Read();
            return 0;
          },
          5, 
          3);
  
  std::cout << std::endl;
  
  MutexClass<std::string> mtx_str_0(0, "100");
  RunTest("MutexClass<string> 3 readers(0 nop) 5 writers...", 
          [&mtx_str_0](int in)->int{
            mtx_str_0.Write(std::to_string(in));
            return 0;
          },
          [&mtx_str_0](int in)->int{
            auto r = mtx_str_0.ReadPtr();
            return 0;
          },
          5, 
          3);
  
  MutexClass<std::string> mtx_str_100(100, "100");
  RunTest("MutexClass<string> 3 readers(100 nop) 5 writers...", 
          [&mtx_str_100](int in)->int{
            mtx_str_100.Write(std::to_string(in));
            return 0;
          },
          [&mtx_str_100](int in)->int{
            auto r = mtx_str_100.ReadPtr();
            return 0;
          },
          5, 
          3);
  
  MutexClass<std::string> mtx_str_10000(10000, "100");
  RunTest("MutexClass<string> 3 readers(10000 nop) 5 writers...", 
          [&mtx_str_10000](int in)->int{
            mtx_str_10000.Write(std::to_string(in));
            return 0;
          },
          [&mtx_str_10000](int in)->int{
            auto r = mtx_str_10000.ReadPtr();
            return 0;
          },
          5, 
          3);
  
  std::cout << std::endl;
  
  std::cout << "****************Large vector (slow copy/read)****************" 
            << std::endl;
/*********************************Big vector**********************************/
  constexpr size_t size = 10000000;
  cabpp::CABpp<std::vector<uint8_t>> cab_v(20, size, 0);
  RunTest("CABpp<vector> 16 readers 3 writers...", 
          [&cab_v](int in)->int{
            cab_v.Write(std::vector<uint8_t>(size, in));
            return 0;
          }, 
          [&cab_v](int)->int{
            auto r = cab_v.Read();
            return 0;
          },
          3, 
          16);
  
  std::cout << std::endl;
  
  MutexClass<std::vector<uint8_t>> mtx_v_0(0, size, 0);
  RunTest("MutexClass<vector> 16 readers(0 nop) 3 writers...", 
          [&mtx_v_0](int in)->int{
            mtx_v_0.Write(std::vector<uint8_t>(size, in));
            return 0;
          }, 
          [&mtx_v_0](int)->int{
            auto r = mtx_v_0.ReadPtr();
            return 0;
          },
          3, 
          16);
  
  MutexClass<std::vector<uint8_t>> mtx_v_100(100, size, 0);
  RunTest("MutexClass<vector> 16 readers(100 nop) 3 writers...", 
          [&mtx_v_100](int in)->int{
            mtx_v_100.Write(std::vector<uint8_t>(size, in));
            return 0;
          }, 
          [&mtx_v_100](int)->int{
            auto r = mtx_v_100.ReadPtr();
            return 0;
          },
          3, 
          16);
  
  MutexClass<std::vector<uint8_t>> mtx_v_10000(10000, size, 0);
  RunTest("MutexClass<vector> 16 readers(10000 nop) 3 writers...", 
          [&mtx_v_10000](int in)->int{
            mtx_v_10000.Write(std::vector<uint8_t>(size, in));
            return 0;
          }, 
          [&mtx_v_10000](int)->int{
            auto r = mtx_v_10000.ReadPtr();
            return 0;
          },
          3, 
          16);
  
  return 0;
}
