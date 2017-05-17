/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Jim Fowler
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <nlopt.hpp>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

typedef PointXYZ PointType;
typedef PointCloud<PointXYZ> Cloud;
typedef double Scalar;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s cloud_a.pcd cloud_b.pcd\n", argv[0]);
}

bool
loadCloud (const std::string &filename, Cloud &cloud)
{
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());
  print_info ("\n");

  return (true);
}

float
hausdorff_distance (Cloud &cloud_a, Cloud &cloud_b)
{
  // compare A to B
  pcl::search::KdTree<PointType> tree_b;
  tree_b.setInputCloud (cloud_b.makeShared ());
  float max_dist_a = -std::numeric_limits<float>::max ();
  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);

    tree_b.nearestKSearch (cloud_a.points[i], 1, indices, sqr_distances);
    if (sqr_distances[0] > max_dist_a)
      max_dist_a = sqr_distances[0];
  }

  // compare B to A
  pcl::search::KdTree<PointType> tree_a;
  tree_a.setInputCloud (cloud_a.makeShared ());
  float max_dist_b = -std::numeric_limits<float>::max ();
  for (size_t i = 0; i < cloud_b.points.size (); ++i)
  {
    std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);

    tree_a.nearestKSearch (cloud_b.points[i], 1, indices, sqr_distances);
    if (sqr_distances[0] > max_dist_b)
      max_dist_b = sqr_distances[0];
  }

  max_dist_a = std::sqrt (max_dist_a);
  max_dist_b = std::sqrt (max_dist_b);

  float dist = std::max (max_dist_a, max_dist_b);

  return dist;
}

int iterations = 0;

double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
  if (!grad.empty()) {
    print_info ("WARNING: I cannot compute the gradient.\n");
  }

  Eigen::Matrix< Scalar, 3, 1 > origin(0,0,0);
  Eigen::Vector4d q(x[0], x[1], x[2], x[3]);
  q.normalize(); // should check for q = zero
  Eigen::Quaternion< Scalar > rotation(q);

  // Apply a rotation to the second point cloud
  Cloud rhoB;
  Cloud **pair = (Cloud **) my_func_data;
  Cloud &a = *(pair[0]);
  Cloud &b = *(pair[1]);
  pcl::transformPointCloud( b, rhoB, origin, rotation );

  // Find the Hausdorf distance
  float distance = hausdorff_distance( a, rhoB );
  ++iterations;
  //print_value ("(%f,%f,%f,%f)\n", x[0], x[1], x[2], x[3]);
  
  return distance;
}

float so3_hausdorff_distance (Cloud &cloud_a, Cloud &cloud_b) {
  nlopt::opt opt(nlopt::GN_DIRECT_L, 4);

  std::vector<double> lb(4);
  lb[0] = -1; lb[1] = -1; lb[2] = -1; lb[3] = -1;
  opt.set_lower_bounds(lb);

  std::vector<double> ub(4);
  ub[0] = +1; ub[1] = +1; ub[2] = +1; ub[3] = +1;
  opt.set_upper_bounds(ub);  

  Cloud *clouds[2] = {&cloud_a, &cloud_b};
  opt.set_min_objective(objective_function, clouds);
  
  opt.set_ftol_abs(1e-4);
  opt.set_stopval(1e-4);  
  
  std::vector<double> x(4);
  x[0] = 0; x[1] = 0; x[2] = 0; x[3] = 0;
  double minf;
  nlopt::result result = opt.optimize(x, minf);
  
  return minf;
}

int
main (int argc, char** argv)
{
  print_info ("Compute SO(3)-averaged Hausdorff distance between point clouds.\n");
  print_info( "  For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need two PCD files to compute Hausdorff distance.\n");
    return (-1);
  }

  // Load the first file
  Cloud::Ptr cloud_a (new Cloud);
  if (!loadCloud (argv[p_file_indices[0]], *cloud_a))
    return (-1);

  // Load the second file
  Cloud::Ptr cloud_b (new Cloud);
  if (!loadCloud (argv[p_file_indices[1]], *cloud_b))
    return (-1);

  // Compute the Hausdorff distance
  float dist = so3_hausdorff_distance (*cloud_a, *cloud_b);
  printf("found minimum after %d evaluations\n", iterations);
  print_info ("SO(3) Hausdorff Distance: "); print_value ("%f", dist);
  print_info ("\n");
}

