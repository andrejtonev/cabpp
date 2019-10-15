/**
 * @class cabpp::CABpp
 * 
 * @brief C++ header-only library implementing a Circular Asynchronous Buffer
 *
 * Circular Asynchronous Buffer (CAB) is a communication mechanism used between
 * periodic tasks, first proposed by Dayton Clark in 1989.
 * 
 * CAB architecture implements a block-free buffer, where reading from it 
 * returns the last value written to it and writing to it never blocks.
 * CAB allocates n_reader + n_writer + 1 slots. This way, when the 
 * user tries to write to CAB, there will always be at least one free slot. 
 * When reading, CAB simply uses the last slot used for a completed write 
 * operation.
 * 
 * The non-blocking aspect of the CAB's architecture can be useful in itself,
 * but can also lead to a significant increase in performance.
 * This performance is not necessarily seen in all cases. The cases where the 
 * reading or writing operations are slow can benefit greatly.
 * 
 * NOTE: While this implementation is lock-free it is not wait-free since there
 * are atomic operations that may use spin-locks in their implementations.
 * 
 * TODO:
 *  - Take user allocated memory and create the objects in it
 *  - Take in user constructed objects and handle just the logic
 *  - Improve performance by eliminating wasted time in the free slot search
 *
 * @author Andreja Tonev
 *
 * @version 1.0.0
 *
 * @date 08/10/2019
 */
#ifndef _CABPP_H_
#define _CABPP_H_

#include <memory>
#include <atomic>
#include <vector>
#include <functional>
#include <iostream>

namespace cabpp {
/***********************************Constants*********************************/
constexpr bool kReading = 1;
constexpr bool kWriting = 0;

/*****************************CABpp implementation****************************/
template<typename T>
class CABpp {
public:
  /**
   * Main constructor. Creates a "pages" number of objects T. Each object is
   * passed the "args" arguments at construction.
   * @param pages(in): Number of copies to create;
   *        equal to the maximum number of concurrent readers and  writers + 1
   * @note: As the Read function returns a shared pointer; the user can keep
   * "reading" long after the Read function execution. Hence here, the 
   * @param args(in): Optional inputs to pass to the object's constructors
   */
  template<typename... Args>
  CABpp (int pages, Args&&... args) : ptr_flags_(pages, kWriting) {
    ptrs_.reserve(pages);
    for (int i=0; i<pages; ++i) {
      ptrs_.push_back(new T(std::forward<Args>(args)...));
    }
    if (pages) {
      ptr_flags_.front() = kReading;
      read_sp_ = CreateSharedPtr(ptrs_.front(), 0);
    } else {
      read_sp_ = nullptr;
    }
  }
  
  /**
   * Copy constructor.
   */
  CABpp(const CABpp& in) : ptr_flags_(in.ptr_flags_) {
    ptrs_.reserve(in.ptrs_.size());
    //Call copy constructor for each element
    for (auto& ptr : in.ptrs_) {
      ptrs_.push_back(new T{*(ptr)});
    }
    //Update the shared pointer to the same index as the passed class'
    int idx = 0;
    for (auto& flag : ptr_flags_) {
      if (flag == kReading) {
        read_sp_ = CreateSharedPtr(ptrs_[idx], idx);
        break;
      } else {
        ++idx;
      }
    }
  }
  
  /**
   * Move constructor.
   */
  CABpp(CABpp&& in) : ptr_flags_(std::move(in.ptr_flags_)), 
                      ptrs_(std::move(in.ptrs_)), 
                      read_sp_(std::move(in.read_sp_)) {}

  /**
   * Copy assignment operator. 
   * @note: This will invalidate any existing shared pointers pointing to any 
   * of the managed objects.
   */
  CABpp& operator=(const CABpp& in) {
    if (&in != this) {
      //Update pointer flags
      ptr_flags_ = in.ptr_flags_;
      //Destroy all managed objects
      for (auto& ptr : ptrs_) {
        delete ptr;
      }
      ptrs_.clear();
      ptrs_.reserve(in.ptrs_.size());
      //Call copy constructor for each element
      for (auto& ptr : in.ptrs_) {
        ptrs_.push_back(new T{*(ptr)});
      }
      //Update the shared pointer to the same index as the passed class'
      int idx = 0;
      for (auto& flag : ptr_flags_) {
        if (flag == kReading) {
          std::atomic_store(&read_sp_, CreateSharedPtr(ptrs_[idx], idx));
          break;
        } else {
          ++idx;
        }
      }
    }
    return *this;
  }
  
  /**
   * Move assignment operator. 
   * @note: This will invalidate any existing shared pointers pointing to any 
   * of the managed objects.
   */
  CABpp& operator=(CABpp&& in) {
    if (&in != this) {
      //Set shared pointer to null
      std::atomic_store(&read_sp_, std::shared_ptr<T>(nullptr));
      //Update flags
      ptr_flags_ = std::move(in.ptr_flags_);
      //Destroy all managed objects
      for (auto& ptr : ptrs_) {
        delete ptr;
      }
      //Update stored pointers
      ptrs_ = std::move(in.ptrs_);
      //Update the shared pointer
      std::atomic_store(&read_sp_, in.read_sp_);
      std::atomic_store(&in.read_sp_, std::shared_ptr<T>(nullptr));
    }
    return *this;
  }
  
  /**
   * Destructor. Destroys all managed objects.
   */
  ~CABpp() {
    //Destroy all managed objects
    for (auto& ptr : ptrs_) {
      delete ptr;
    }
  }
  
  /**
   * Write function. Copies the input to the first free object. 
   * @note: Uses operator= to assign the value to the free object.
   * @param in(in): object T to copy over
   * @return bool; false if a free pointer was not found
   * @note: Due to atomic function used, the function can fail even when there
   * is an available object. It is on the user to handle spurious failures.
   */
  bool Write(const T& in) {
    auto idx = GetFreePtrIdx();
    if (idx >= ptr_flags_.size()) return false;
    //Update values and the shared ptr
    auto ptr = ptrs_[idx];
    *ptr = in;
    std::atomic_store(&read_sp_, CreateSharedPtr(ptr, idx));
    return true;
  }
  
  /**
   * Move function. Executes move semantic on the first free object.
   * @param in(in): object T to move
   * @return bool; false if a free pointer was not found
   * @note: Due to atomic function used, the function can fail even when there
   * is an available object. It is on the user to handle spurious failures.
   */
  bool Move(T&& in) {
    auto idx = GetFreePtrIdx();
    if (idx >= ptr_flags_.size()) return false;
    //Update values and the shared ptr
    auto ptr = ptrs_[idx];
    *ptr = std::move(in);
    std::atomic_store(&read_sp_, CreateSharedPtr(ptr, idx));
    return true;
  }
  
  /**
   * Read function. Atomically passes the shared pointer holding the last
   * updated object's pointer.
   * @return shared pointer of the class' template object
   */
  std::shared_ptr<T> Read() const {
    return std::atomic_load(&read_sp_);
  }
  
private:
  /**
   * Custom shared pointer delete function. Frees the appropriate pointer by
   * setting the corresponding flag to "writable".
   * @param elem(in): object's raw pointer
   * @param idx(in): index corresponding to the pointer's index in the vector
   * @note: Will fail during the move constructor.
   */
  void DeleteSP(T* elem, int idx) {
    if (idx >= ptr_flags_.size()) 
      std::cerr << "Failed to set the pointer (idx " << idx 
                << ") to writable" << std::endl;
    else
      ptr_flags_[idx] = kWriting;
  }

  /**
   * Helper function that creates a shared pointer from an object's raw 
   * pointer.
   * @param ptr(in): raw object pointer to make into a shared pointer
   * @param idx(in): pointer's index in the vector @ref ptrs_
   * @return shared pointer of the class' template object
   */
  std::shared_ptr<T> CreateSharedPtr(T* ptr, int idx) {
    using namespace std::placeholders;
    return std::shared_ptr<T>(ptr, 
                              std::bind(&CABpp<T>::DeleteSP, this, _1, idx));
  }
  
  /**
   * Helper function that finds the first "writable", saves it and switches its
   * status to "readable".
   * @return int; index of the pointer 
   * @note: equal to the vector's size of error
   */
  int GetFreePtrIdx(void) {
    //Find a pointer that is available for writing
    int idx = 0;
    for (auto& ptr_flag : ptr_flags_) {
      bool write_b = kWriting;
      if (ptr_flag.flag.compare_exchange_weak(write_b, kReading)) {
        break;
      }
      ++idx;
    }
    return idx;
  }
  
  /*
  * Private type definitions
  */
  /**
   * Atomic flag (atomic_bool) wrapper.
   * Implements the necessary functions that allow the use of an atomic_bool
   * inside a vector and allows copy/move semantics of the parent class.
   */
  struct Flag {
    Flag() {
      flag = false;
    }
    
    Flag(const bool flag_in) {
      flag = flag_in;
    }
    
    Flag(const Flag& in) {
      flag = in.flag.load();
    }
    
    Flag(Flag&& in) {
      flag = in.flag.load();
      in.flag.store(0);
    }
    
    Flag& operator=(const Flag& in) {
      if (&in != this)
        flag.store(in.flag);
      return *this;
    }
    
    Flag& operator=(bool b) {
      flag = b;
      return *this;
    }
    
    bool operator==(bool b) {
      bool saved_flag = flag.load();
      return (b == saved_flag);
    }
    
    std::atomic_bool flag;
  };

  /*
  * Private variables
  */
  std::vector<Flag> ptr_flags_; //!< Read/Write flags vector (access type definition)
  std::vector<T*> ptrs_;        //!< Object pointer vector
  std::shared_ptr<T> read_sp_;  //!< Class' shared pointer passed on reading (atomic operations)
};

} //namespace cabpp

#endif //_CABPP_H_
