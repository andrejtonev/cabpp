/**
 * @brief Main example for the CABpp library
 *
 * This example showcases the CABpp's performance when compared to a class
 * whose inter-threaded synchronization is a simple mutex lock.
 *
 * @author Andreja Tonev
 *
 */
#include "cabpp.h"
#include "helper_func.h"
#include "mutex_class.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

/****************************Global const variables***************************/
constexpr long kTestDuration_us = 1e6;

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
          1,
          kTestDuration_us);
  
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
          1,
          kTestDuration_us);
  
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
          1,
          kTestDuration_us);
  
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
          1,
          kTestDuration_us);
  
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
          3,
          kTestDuration_us);
  
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
          3,
          kTestDuration_us);
  
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
          3,
          kTestDuration_us);
  
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
          3,
          kTestDuration_us);
  
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
          16,
          kTestDuration_us);
  
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
          16,
          kTestDuration_us);
  
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
          16,
          kTestDuration_us);
  
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
          16,
          kTestDuration_us);
  
  return 0;
}
