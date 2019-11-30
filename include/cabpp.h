/**
 * @class cabpp::CABpp
 * 
 * @brief C++ header-only library implementing a Circular Asynchronous Buffer
 *
 * Circular Asynchronous Buffer (CAB) is a communication mechanism used between
 * periodic tasks, first proposed by Prof. D. Clark in 1989 (refered to as 
 * Periodic Data Buffer), later redefined by Prof. G. Buttazzo.
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
 *  - Improve performance by eliminating wasted time in the free slot search
 *
 * @author Andreja Tonev
 *
 * @version 1.0.1
 *
 * @date 30/11/2019
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
   * @param args(in): Optional inputs to pass to the object's constructor
   */
  template<typename... Args>
  CABpp (unsigned int pages, Args&&... args) : 
  ptr_flags_(pages, kWriting), handler_type_(OperationType::kFull) {
    if (!pages) { //Empty cab <=> nullptr
      read_sp_ = nullptr;
      return;
    }
    ptrs_.reserve(pages);
    //Create N objects
    for (int i=0; i<pages; ++i) {
      try {
        //Allocate and construct object
        ptrs_.push_back(new T(std::forward<Args>(args)...));
      } catch(...) {
        //Free all allocated objects and throw
        for (auto& ptr : ptrs_)
          delete ptr;
        throw;
      }
    }
    //Update pointer
    ptr_flags_.front() = kReading;
    read_sp_ = CreateSharedPtr(ptrs_.front(), 0);
  }
  
  /**
   * Main constructor using user-allocated memory. Creates a "pages" number of 
   * objects T in the passed allocated memory. Each object is passed the "args" 
   * arguments at construction.
   * @param ptr(in): Pointer to the user-allocated memory
   * @param mem_size(in/out): Size of the allocated memory. Value is decreased 
   *        by the memory size used for the CAB objects.
   * @param pages(in): Number of copies to create;
   *        equal to the maximum number of concurrent readers and  writers + 1
   * @param args(in): Optional inputs to pass to the object's constructors
   */
  template<typename... Args>
  CABpp(void* ptr, size_t& mem_size, unsigned int pages, Args&&... args) : 
  ptr_flags_(pages, kWriting), handler_type_(OperationType::kNoMem) {
    if (!pages) { //Empty cab <=> nullptr
      read_sp_ = nullptr;
      return;
    }
    ptrs_.reserve(pages);
    //Create N objects
    for (int i=0; i<pages; ++i) {
      //Aligns the pointer and decreases the memory size
      if (std::align(alignof(T), sizeof(T), ptr, mem_size)) {
        T* aligned = reinterpret_cast<T*>(ptr);
        //Create new object at the aligned memory
        try {
          ptrs_.push_back(new(aligned) T(std::forward<Args>(args)...));
        } catch(...) {
          //Free all allocated objects and throw
          for (auto& ptr : ptrs_)
            delete ptr;
          throw;
        }
        //Update ptr for the next object
        ptr = (char*)ptr + sizeof(T);
        mem_size -= sizeof(T);
      } else {
        read_sp_ = nullptr;
        std::cerr << "Failed while aligning memory" 
                  << ((mem_size < sizeof(T))?": out of memory":"") << std::endl;
        return;
      }
    }
    //Update pointer
    ptr_flags_.front() = kReading;
    read_sp_ = CreateSharedPtr(ptrs_.front(), 0);
  }
  
  /**
   * Main constructor using user-constructed objects. Runs only the logic level. 
   * The objects and memory allocation are handled by the user.
   * @param obj_ptrs(in): vector of pointers to the user-constructed objects
   */
  CABpp(std::vector<T*> obj_ptrs) : handler_type_(OperationType::kNoObj) {
    if (obj_ptrs.empty()) {
      read_sp_ = nullptr;
      return;
    }
    ptr_flags_ = std::vector<Flag>(obj_ptrs.size(), kWriting);
    ptrs_ = obj_ptrs;
    ptr_flags_.front() = kReading;
    read_sp_ = CreateSharedPtr(ptrs_.front(), 0);
  }
  
  /**
   * Copy constructor.
   */
  CABpp(const CABpp& in) : ptr_flags_(in.ptr_flags_), 
                           handler_type_(in.handler_type_) {
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
                      read_sp_(std::move(in.read_sp_)),
                      handler_type_(std::move(in.handler_type_)) {}

  /**
   * Copy assignment operator. 
   * @note: This will delete all objects handled by the CAB being overwritten.
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
      handler_type_ = in.handler_type_;
    }
    return *this;
  }
  
  /**
   * Move assignment operator. 
   * @note: This will delete all objects handled by the CAB being overwritten.
   */
  CABpp& operator=(CABpp&& in) {
    if (&in != this) {
      //Set shared pointer to null
      std::atomic_store(&read_sp_, std::shared_ptr<const T>(nullptr));
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
      std::atomic_store(&in.read_sp_, std::shared_ptr<const T>(nullptr));
      handler_type_ = in.handler_type_;
    }
    return *this;
  }
  
  /**
   * Destructor.
   */
  ~CABpp() {
    switch (handler_type_) {
      case OperationType::kFull:
        for (auto& ptr : ptrs_)
          delete ptr; //Destroy memory and free allocated memory
        break;
      case OperationType::kNoMem:
        for (auto& ptr : ptrs_)
          ptr->~T(); //Just call destructors without freeing the memory
        break;
      case OperationType::kNoObj:
        //Nothin to do; no memory allocated nor object created
        break;
    }
  }
  
  
  //Todo: move outside the clasa
  /*
  * Public type definition
  */
  /**
   * Pointer wrapper holding the index used by the CABpp class.
   */
  class ObjPtr {
  public:
    T* operator->() const {
      return obj_ptr_;
    }
    
    T& operator*() const {
      return *obj_ptr_;
    }
    
    operator bool() const {
      return (obj_ptr_ != nullptr || ptr_idx_ >= 0);
    }
    
  private:
    explicit ObjPtr(T* ptr, int idx) : obj_ptr_(ptr), ptr_idx_(idx) {}
    
    T* obj_ptr_;
    int ptr_idx_;
    
    friend CABpp;
  };

  /**
   * Reserve one CAB object. Returns a pointer-like object that points to a 
   * free CAB slot. By using Reserve, the user can avoid allocating additional
   * object that get moved or copied into a free CAB slot.
   * @return ObjPtr; pointer wrapper (behaves like a regular pointer)
   */
  ObjPtr Reserve(void) {
    auto idx = GetFreePtrIdx();
    if (idx >= ptr_flags_.size()) 
      return ObjPtr(nullptr, -1);
    return ObjPtr(ptrs_[idx], idx);
  }
  
  /**
   * Write function. Update the CAB slot pointed to by the ObjPtr. Releases the slot for
   * reading.
   * @param in(in/out): ObjPtr pointing to the CAB object. Gets reset on 
   *        successful write.
   * @return bool; false on error
   */
  bool Write(ObjPtr& in) {
    if (!in || in.ptr_idx_ >= ptrs_.size() || in.obj_ptr_ != ptrs_[in.ptr_idx_])
      return false;
    //Update read share pointer
    std::atomic_store(&read_sp_, CreateSharedPtr(in.obj_ptr_, in.ptr_idx_));
    //Reset ObjPtr
    in.obj_ptr_ = nullptr;
    in.ptr_idx_ = -1;
    return true;
  }
  
  /**
   * Write function. Copies the input to the first free slot. 
   * @note: Uses operator= to assign the value to the free object.
   * @param in(in): object T to copy over
   * @return bool; false if a free pointer was not found
   * @note: Due to atomic function used, the function can fail even when there
   * is an available object. It is on the user to handle spurious failures.
   */
  bool Write(const T& in) {
    auto ptr = Reserve();
    if (!ptr) return false;
    *ptr = in;
    return Write(ptr);
  }
  
  /**
   * Write function. Executes move semantic on the first free slot.
   * @param in(in): object T to move
   * @return bool; false if a free pointer was not found
   */
  bool Write(T&& in) {
    auto ptr = Reserve();
    if (!ptr) return false;
    *ptr = std::move(in);
    return Write(ptr);
  }
  
  /**
   * Read function. Atomically passes the shared pointer holding the last
   * updated CAB slot. 
   * @note: the CAB slot becomes available again only after the 
   * shared pointer is released/reset.
   * @return shared pointer of the class' template object
   */
  std::shared_ptr<const T> Read() const {
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
  std::shared_ptr<const T> CreateSharedPtr(T* ptr, int idx) {
    using namespace std::placeholders;
    return std::shared_ptr<const T>(ptr, 
                              std::bind(&CABpp<T>::DeleteSP, this, _1, idx));
  }
  
  /**
   * Helper function that finds the first "writable" slot, saves it and switches its
   * status to "readable".
   * @return int; index of the pointer 
   * @note: equal to the vector's size of error
   */
  int GetFreePtrIdx(void) {
    //Find a pointer that is available for writing
    int idx = 0;
    for (auto& ptr_flag : ptr_flags_) {
      bool write_b = kWriting;
      if (ptr_flag.flag.compare_exchange_strong(write_b, kReading)) {
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
   * inside a vector and allows for copy/move semantics.
   */
  class Flag {
  public:
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

  /**
   * CAB operation type enumeration. Defines 3 types of object handling.
   * The CAB can allocate and construct all object; construct objects on 
   * user-allocated memory; or use user-constructed objects (leaving just the logic level running).
   */
  enum class OperationType {
    kFull = 0, //!< Handle both memory allocation and object construction
    kNoMem,    //!< Handles only object construction on user-allocated memory
    kNoObj,    //!< Only logic level is active; both memory and objects are handled by the user
  };
  
  /*
  * Private variables
  */
  std::vector<Flag> ptr_flags_; //!< Read/Write flags vector (access type definition)
  std::vector<T*> ptrs_; //!< Object pointer vector
  std::shared_ptr<const T> read_sp_; //!< Class' shared pointer passed on reading (atomic operations)
  OperationType handler_type_; //!< Operation mode type flag
};

} //namespace cabpp

#endif //_CABPP_H_
