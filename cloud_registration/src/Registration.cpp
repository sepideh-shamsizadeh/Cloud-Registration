#include "Registration.h"


Registration::Registration(std::string cloud_source_filename, std::string cloud_target_filename)
{
  // TO COMPLETE
}


Registration::Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target)
{
  // TO COMPLETE
}


void Registration::draw_registration_result()
{
  //visualize target and source with two different colors
  // TO COMPLETE
}


void Registration::preprocess(open3d::geometry::PointCloud pcd, double voxel_size, std::shared_ptr<open3d::geometry::PointCloud> &pcd_down_ptr, std::shared_ptr<open3d::pipelines::registration::Feature> &pcd_fpfh)
{
  //downsample, estimate normals and compute FPFH features

  // TO COMPLETE
  return;
}

open3d::pipelines::registration::RegistrationResult Registration::execute_global_registration(double voxel_size)
{
  // remember to apply the transformation_ matrix to source_cloud
  // create two point cloud to contain the downsampled point cloud and two structure to contain the features
  // call the Registration::preprocess function on target and transformed source
  // execute global transformation and update the transformation matrix
  // TO COMPLETE
  open3d::pipelines::registration::RegistrationResult result;
  return result;
}

open3d::pipelines::registration::RegistrationResult Registration::execute_icp_registration(double threshold, double relative_fitness, double relative_rmse, int max_iteration)
{
  open3d::pipelines::registration::RegistrationResult result;
  return result;
}


void Registration::set_transformation(Eigen::Matrix4d init_transformation)
{
  transformation_=init_transformation;
}


Eigen::Matrix4d  Registration::get_transformation()
{
  return transformation_;
}


void Registration::write_tranformation_matrix(std::string filename)
{
  std::ofstream outfile (filename);
  if (outfile.is_open())
  {
    outfile << transformation_;
    outfile.close();
  }
}

void Registration::save_merged_cloud(std::string filename)
{
  //clone input
  open3d::geometry::PointCloud source_clone = source_;
  open3d::geometry::PointCloud target_clone = target_;

  source_clone.Transform(transformation_);
  open3d::geometry::PointCloud merged = target_clone+source_clone;
  open3d::io::WritePointCloud(filename, merged );
}

