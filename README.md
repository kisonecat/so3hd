# so3hd

Compute the minimum of the Hausdorff distance between a 3-dimensional
point cloud A and a point cloud rho(B), where rho is an element of
SO(3).

To avoid the issue of building a nice cellular decomposition of SO(3),
we pay the price of adding one additional dimension.  That is, we use
box in R^4 containing unit ball; then a point in that box is rescaled
so that a point lies on the unit sphere S^3, which can then via the
quaternions be regarded as an element of SO(3).  Thus, the search
space is a box.

Rather than trying to be opinionated as to what global search
algorithm we ought to use to compute the minimum over rho in SO(3), we
use [NLopt](http://ab-initio.mit.edu/wiki/index.php/NLopt), a nice
optimization package with bindings for a variety of languages.

We rely on [PCL](http://pointclouds.org/) to manage the point cloud
data.

# Building

```
mkdir build
cd build
cmake ..
make
```
