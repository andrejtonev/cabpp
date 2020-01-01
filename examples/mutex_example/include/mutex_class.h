/** 
 * @brief Header implementing video example functionalities.
 *
 * @author Andreja Tonev
 */
#ifndef _CABPP_MUTEX_EXAMPLE_MUTEX_C_H_
#define _CABPP_MUTEX_EXAMPLE_MUTEX_C_H_

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>


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

#endif //_CABPP_MUTEX_EXAMPLE_MUTEX_C_H_
