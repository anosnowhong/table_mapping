primitive_extraction
==============

Launch the primitive extraction with the following command:

`roslaunch primitive_extraction extraction.launch cloud:=/pointcloud2/topic`.

It also has the boolean arguments `extract_planes`, `extract_cylinders` and `extract_spheres`, which all default to true.

The launch file contains a bunch of options and parameters, notably:

* `number_disjoint_subsets` - The primitives are evaluated on more and more subsets of the point cloud, this is the number of subsets.
* `octree_leaf_size` - The size of the octree nodes, mainly influences speed.
* `normal_neigbourhood` - The size of the neighbourhood used to estimate normals from the point cloud.
* `inlier_threshold` - Largest distance from primitive for a point to be considered an inlier.
* `angle_threshold` - The largest allowed deviation of point normal from primitive surface normal.
* `error_add_probability` - The probability of extracting a primitive which is not the largest.
* `min_inliers` - Do not consider any shapes with fewer inliers than this.
* `min_terminate` - Terminate when no shapes with more inliers than this are left.
* `connectedness_dist` - Discretization of connectedness on primitive surfaces, used to extract the largest connected region.
* `distance_threshold` - Do not consider any points further away from the camera than this.

Trouble-shooting
----------------
Older versions of GCC (notably 4.6.3 currently the default on Ubuntu 12.04) have a problem with the AVX instructions on new versions of Intel Core i7. To get around these compilation problems, add `-mno-avx` to the `CMAKE_CXX_FLAGS` in this package.
