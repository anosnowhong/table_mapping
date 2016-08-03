ransac_primitives
==============
This is a library for extracting primitives from a PCL point cloud, see `src/test_extraction.cpp` for an example of usage.
The library implements the paper "Efficient Ransac for Point Cloud Shape Detection" by R. Schnabel et al.
There is a package named `primitive_extraction` that acts as a ROS wrapper around this library.

Trouble-shooting
----------------
Older versions of GCC (notably 4.6.3 currently the default on Ubuntu 12.04) have a problem with the AVX instructions on new versions of Intel Core i7. To get around these compilation problems, add `-mno-avx` to the `CMAKE_CXX_FLAGS` in this package.
