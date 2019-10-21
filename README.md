# cabpp
C++ header-only library implementing a Cyclic Asynchronous Buffer (CAB).

### Cyclic Asynchronous Buffer
Circular Asynchronous Buffer (CAB) is a communication mechanism used between
periodic tasks, first proposed by Prof. D. Clark in 1989 (refered to as 
Periodic Data Buffer), later redefined by Prof. G. Buttazzo.
 
CAB architecture implements a block-free buffer, where reading from it 
returns the last value written to it and writing to it never blocks.

CAB allocates *n_reader + n_writer + 1* slots. This way, when the 
user tries to write to CAB, there will always be at least one free slot. 
When reading, CAB simply uses the last slot used for the last successful  
write operation.
 
The non-blocking aspect of the CAB's architecture can be useful in itself,
but can also lead to a significant increase in performance.
This performance is not necessarily seen in all cases. The cases where the 
reading or writing operations are slow can benefit greatly.

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
