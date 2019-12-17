# CABpp
C++ header-only library implementing a Cyclic Asynchronous Buffer (CAB).

### Cyclic Asynchronous Buffer
Cyclic Asynchronous Buffer (CAB) is a communication mechanism used between
periodic tasks, first proposed by Prof. D. Clark[1](#sources) (referred to as 
Periodic Data Buffer), later redefined by Prof. G. Buttazzo[2](#sources).
 
CAB architecture implements a block-free buffer, where reading from it 
returns the last value written to it and writing to it never blocks.

In order to have non-blocking operations, *n_reader + n_writer + 1* slots are allocated.
Without the additional slot, the writer could be writing and rewriting to the same slot, 
without giving the reader a chance to read it before it gets overwritten.
When reading, CAB simply uses the last slot used for a successful write operation.

#### Architecture
In order to achieve non-blocking operations, CAB needs to allocate a number of slots
equal to the maximum number of concurrent readers and writers plus one additional slot.

*concurrent readers and writers* refers to all instances that may be blocking a slot.
A slot can be blocked by:
- not releasing the smart pointer returned by the read function
- reserving a slot (gets released after the corresponding write function call)
- writing to the slot


#### Use
**CREATION**
CABpp allows for three modes of operation: 
- internal object construction
- external object construction
- using pre-constructed objects.

**READING**
Reading from CAB returns a smart pointer pointing to the last updated object slot (multiple readers can read from the same slot). The slot is freed only once the smart pointer has been destroyed/released. This allows for repeated access to the same slot without having to recall the read function.

**WRITING**
The writer may:
- copy a pre-allocated object to a free CAB slot
- move a pre-allocated object to a free CAB slot
- reserve a free CAB slot, work directly on the slot's object and update once ready (reserve/write mode).

Reserve/write mode allows the user to work without allocating any additional objects (by using the already allocated objects in the CAB).

### Library installation
```
git clone https://github.com/andrejtonev/cabpp.git
cd cabpp
mkdir build
cd build
cmake ..
make
make install
```

### Run example
```
git clone https://github.com/andrejtonev/cabpp.git
cd cabpp
mkdir build
cd build
cmake -DBUILD_EXAMPLE=ON ..
make
./cabpp-1.0.0
```

### Run tests
```
git clone https://github.com/andrejtonev/cabpp.git
cd cabpp
mkdir build
cd build
cmake -DGTEST=ON ..
make
make test
```

### Include library in other CMake projects
```
find_package (CABpp)
target_link_libraries(*** cabpp)
```

### Sources
[1] Clark, D.: “HIC: An Operating System for Hierarchies of Servo Loops”, Proc. of IEEE Int. Conf. on Rob. and Autom., pp. 1004-1009,1989.


[2] Buttazzo, G.C.: "HARTIK: a real-time kernel for robotics applications", Proc. of IEEE Real_Time Systems Symposium, pp. 201–205,1993.



#
*Special thanks to Prof. G. Buttazzo, Gabriele Serra and Gabriele Ara for their helpful remarks.* 
