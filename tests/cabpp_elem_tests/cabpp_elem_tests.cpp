/** Group of GTests used to verify the CABpp implementation.
 */
#include "gtest/gtest.h"
#include "cabpp.h"

#include <atomic>
#include <thread>
#include <string>

namespace {
/** Test the main constructor on various data types.
 */
TEST(CABpp, Allocate) {
  cabpp::CABpp<int> cabpp_3(3, 11);
  EXPECT_EQ(*cabpp_3.Read(), 11);
  cabpp::CABpp<std::string> cabpp_5(5, "22");
  EXPECT_EQ(*cabpp_5.Read(), "22");
}

/** Test copy constructor and class copy operator.
 */
TEST(CABpp, Copy) {
  cabpp::CABpp<std::vector<uint8_t>> cabpp_3(3, 100, 15);
  EXPECT_EQ((*cabpp_3.Read())[0], 15);
  auto copy_cabpp = cabpp_3; //Class copy constructor
  auto ptr = copy_cabpp.Read();
  EXPECT_EQ((*copy_cabpp.Read())[10], 15);
  EXPECT_EQ((*cabpp_3.Read())[20], 15);
  decltype(cabpp_3) cabpp_0(0);
  EXPECT_FALSE(cabpp_0.Write(*ptr));
  cabpp_0 = cabpp_3; //Class copy
  EXPECT_EQ((*cabpp_3.Read())[30], 15);
}

/** Test move constructor and class move operator.
 */
TEST(CABpp, Move) {
  cabpp::CABpp<std::vector<uint8_t>> cabpp_3(3, 100, 16);
  EXPECT_EQ((*cabpp_3.Read())[0], 16);
  auto move_cabpp(std::move(cabpp_3)); //Class move constructor
  EXPECT_EQ((*move_cabpp.Read())[10], 16);
  EXPECT_EQ(cabpp_3.Read(), nullptr);
  decltype(move_cabpp) cabpp_0(0);
  EXPECT_FALSE(cabpp_0.Write(std::vector<uint8_t>(10, 1)));
  cabpp_0 = std::move(move_cabpp); //Class move
  EXPECT_EQ((*cabpp_0.Read())[30], 16);
  EXPECT_EQ(move_cabpp.Read(), nullptr);
}

/** Test single threaded read/write functions.
 */
TEST(CABpp, SingleWR) {
  cabpp::CABpp<int> cabpp_3(3, 11);
  EXPECT_TRUE(cabpp_3.Write(1));
  EXPECT_EQ(*cabpp_3.Read(), 1);
  EXPECT_TRUE(cabpp_3.Write(2));
  EXPECT_EQ(*cabpp_3.Read(), 2);
  EXPECT_TRUE(cabpp_3.Write(3));
  EXPECT_EQ(*cabpp_3.Read(), 3);
  EXPECT_TRUE(cabpp_3.Write(4));
  EXPECT_EQ(*cabpp_3.Read(), 4);
  EXPECT_TRUE(cabpp_3.Write(5));
  EXPECT_EQ(*cabpp_3.Read(), 5);
  EXPECT_TRUE(cabpp_3.Write(6));
  EXPECT_EQ(*cabpp_3.Read(), 6);
  EXPECT_TRUE(cabpp_3.Write(7));
  EXPECT_EQ(*cabpp_3.Read(), 7);
}

/** Test single threaded read/move functions.
 */
TEST(CABpp, ReadMove) {
  cabpp::CABpp<std::vector<int>> cabpp_3(3, 3, 11);
  EXPECT_EQ((*cabpp_3.Read())[0], 11);
  EXPECT_TRUE(cabpp_3.Move({1, 2, 3}));
  EXPECT_EQ((*cabpp_3.Read())[0], 1);
  std::vector<int> input(100, 13);
  EXPECT_TRUE(cabpp_3.Move(std::move(input)));
  EXPECT_EQ((*cabpp_3.Read())[20], 13);
  EXPECT_EQ(input.size(), 0);
}

/** Test reading and saving the shared pointer for later access.
 */
TEST(CABpp, ReadSave) {
  cabpp::CABpp<int> cabpp_3(3, 11);
  auto ptr = cabpp_3.Read();
  EXPECT_EQ(*ptr, 11);
  EXPECT_TRUE(cabpp_3.Write(1));
  EXPECT_EQ(*ptr, 11);
  EXPECT_TRUE(cabpp_3.Write(2));
  EXPECT_EQ(*ptr, 11);
  EXPECT_TRUE(cabpp_3.Write(3));
  EXPECT_EQ(*ptr, 11);
  EXPECT_TRUE(cabpp_3.Write(4));
  EXPECT_EQ(*ptr, 11);
}

/** Cause a write failure by having too many concurrent readers.
 *  Here the readers save the shared pointer which exhausts the buffer.
 */
TEST(CABpp, WriteFail) {
  cabpp::CABpp<int> cabpp_3(3, 11);
  auto ptr1 = cabpp_3.Read();
  EXPECT_TRUE(cabpp_3.Write(1));
  auto ptr2 = cabpp_3.Read();
  EXPECT_TRUE(cabpp_3.Write(2));
  auto ptr3 = cabpp_3.Read();
  EXPECT_FALSE(cabpp_3.Write(3));
  EXPECT_EQ(*cabpp_3.Read(), 2);
}

/** Multithread test.
 */
TEST(CABpp, Thread3Read1Write) {
  std::atomic_int idx, n_read;
  idx = 0;
  n_read = 0;
  cabpp::CABpp<std::string> cabpp_5(5, "0");
  
  auto read_test = [&](){
    while(idx.load() == 0)
      ; //Spin-lock
    int old_idx = 0;
    int new_idx = old_idx;
    while(1) {
      while(old_idx == new_idx)
        new_idx = idx.load();
      old_idx = new_idx;
      if (old_idx == 5)
        break;
      EXPECT_EQ(*cabpp_5.Read(), std::to_string(old_idx));
      n_read++;
    }
  };
  
  std::thread read_th1(read_test);
  std::thread read_th2(read_test);
  std::thread read_th3(read_test);
  
  n_read = 0;
  EXPECT_TRUE(cabpp_5.Write("1"));
  idx++; //One loop spin of the read threads
  while(n_read != 3)
    ; //Sync with the read threads
  n_read = 0;
  EXPECT_TRUE(cabpp_5.Write("2"));
  idx++; //One loop spin of the read threads
  while(n_read != 3)
    ; //Sync with the read threads
  n_read = 0;
  EXPECT_TRUE(cabpp_5.Write("3"));
  idx++; //One loop spin of the read threads
  while(n_read != 3)
    ; //Sync with the read threads
  n_read = 0;
  EXPECT_TRUE(cabpp_5.Write("4"));
  idx++; //One loop spin of the read threads
  while(n_read != 3)
    ; //Sync with the read threads
    
  idx++; //On 5 the read thread exit
  if (read_th1.joinable()) read_th1.join();
  if (read_th2.joinable()) read_th2.join();
  if (read_th3.joinable()) read_th3.join();
}

} //namespace
