/**
 * Group of GTests used to verify the CABpp implementation.
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

/** Test the constructor on user-allocated memory with various data types.
 */
TEST(CABpp, UserMem) {
  size_t mem_size = 300;
  auto mem_ = malloc(mem_size);
  void* mem = (char*)mem_ + 1;
  --mem_size;
  {
    EXPECT_TRUE(mem);
    size_t size_res = mem_size - 3*sizeof(int);
    cabpp::CABpp<int> cabpp_3(mem, mem_size, 3, 11);
    EXPECT_LE(mem_size, size_res);
    EXPECT_EQ(*cabpp_3.Read(), 11);
    size_res = mem_size - 5*sizeof(std::string);
    cabpp::CABpp<std::string> cabpp_5((char*)mem + (300 - mem_size), mem_size, 5, "22");
    EXPECT_LE(mem_size, size_res);
    EXPECT_EQ(*cabpp_5.Read(), "22");
  }
  free(mem_);
  
  //Check for failure
  mem_size = 10;
  mem = malloc(mem_size);
  cabpp::CABpp<int> cabpp_fail(mem, mem_size, 11); //11xint will fail on 10B memory
  EXPECT_EQ(cabpp_fail.Read(), nullptr);
  free(mem);
}

/** Test the constructor on user-constructed objects with various data types.
 */
TEST(CABpp, UserObj) {
  int i1 = 1;
  int i2 = 2;
  int i3 = 3;
  std::vector<int*> pi = {&i1, &i2, &i3};
  std::string s1 = "1";
  std::string s2 = "2";
  std::string s3 = "3";
  std::string s4 = "4";
  std::string s5 = "5";
  std::vector<std::string*> ps = {&s1, &s2, &s3, &s4, &s5};
  
  cabpp::CABpp<int> cabpp_3(pi);
  EXPECT_EQ((*cabpp_3.Read()), 1);
  cabpp::CABpp<std::string> cabpp_5(ps);
  EXPECT_EQ((*cabpp_5.Read()), "1");
}

/** Test move constructor and class move operator.
 */
TEST(CABpp, Move) {
  cabpp::CABpp<std::vector<uint8_t>> cabpp_3(3, 100, 15);
  EXPECT_EQ((*cabpp_3.Read())[0], 15);
  auto move_cabpp = std::move(cabpp_3); //Class move constructor
  auto ptr = move_cabpp.Read();
  EXPECT_EQ((*move_cabpp.Read())[10], 15);
  EXPECT_TRUE(cabpp_3.Read() == nullptr);
  decltype(cabpp_3) cabpp_0(0);
  EXPECT_FALSE(cabpp_0.Write(*ptr));
  cabpp_0 = std::move(move_cabpp); //Class move assignment
  EXPECT_EQ((*cabpp_0.Read())[30], 15);
  EXPECT_TRUE(move_cabpp.Read() == nullptr);
}

/** Cause a write failure by having too many concurrent readers.
 *  Here the readers save the shared pointer which exhausts the buffer.
 */
TEST(CABpp, Reserve) {
  struct test {
    int i;
    char c;
  };
  //Reserve and write
  cabpp::CABpp<test> cabpp_3(3, test({11, 'a'}));
  auto ptr1 = cabpp_3.Reserve();
  EXPECT_TRUE(ptr1);
  //Obj update
  (*ptr1).i = 20;
  ptr1->c = 't';
  EXPECT_TRUE(cabpp_3.Write(ptr1));
  EXPECT_FALSE(ptr1);
  EXPECT_EQ(cabpp_3.Read()->i, 20);
  EXPECT_EQ(cabpp_3.Read()->c, 't');
  //Over-reserve causes a failure
  ptr1 = cabpp_3.Reserve();
  EXPECT_TRUE(ptr1);
  auto ptr2 = cabpp_3.Reserve();
  EXPECT_TRUE(ptr2);
  auto ptr3 = cabpp_3.Reserve();
  EXPECT_FALSE(ptr3);
  auto ptr4 = cabpp_3.Reserve();
  EXPECT_FALSE(ptr4);
  auto ptr5 = cabpp_3.Reserve();
  EXPECT_FALSE(ptr5);
}

/** Test move constructor and class move operator.
 */
TEST(CABpp, Write) {
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
TEST(CABpp, ReadWrite) {
  cabpp::CABpp<std::vector<int>> cabpp_3(3, 3, 11);
  EXPECT_EQ((*cabpp_3.Read())[0], 11);
  EXPECT_TRUE(cabpp_3.Write({1, 2, 3}));
  EXPECT_EQ((*cabpp_3.Read())[0], 1);
  std::vector<int> input(100, 13);
  EXPECT_TRUE(cabpp_3.Write(std::move(input)));
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
