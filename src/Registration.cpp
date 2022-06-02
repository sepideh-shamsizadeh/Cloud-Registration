#include "Registration.h"


Registration::Registration(std::string cloud_source_filename, std::string cloud_target_filename)
{
  // TO COMPLETE
    open3d::io::ReadPointCloud(cloud_source_filename, source_);
    open3d::io::ReadPointCloud(cloud_target_filename, target_);
}


Registration::Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target)
{
  // TO COMPLETE
    source_ = cloud_source;
    target_ = cloud_target;
}


void Registration::draw_registration_result()
{
    //visualize target and source with two different colors
    // TO COMPLETE
    Eigen::Vector3d color1(1, 0.706, 0);
    source_.open3d::geometry::PointCloud::PaintUniformColor(color1);
    auto source_pointer =
            std::make_shared<open3d::geometry::PointCloud>(source_);
    Eigen::Vector3d color2(0, 0.651, 0.929);
    target_.open3d::geometry::PointCloud::PaintUniformColor(color2);
    auto target_pointer =
            std::make_shared<open3d::geometry::PointCloud>(target_);
    open3d::visualization::DrawGeometries({source_pointer,
                                           target_pointer});
}


void Registration::preprocess(open3d::geometry::PointCloud pcd, double voxel_size, std::shared_ptr<open3d::geometry::PointCloud> &pcd_down_ptr, std::shared_ptr<open3d::pipelines::registration::Feature> &pcd_fpfh)
{
  //downsample, estimate normals and compute FPFH features

  // TO COMPLETE
    pcd_down_ptr = pcd.VoxelDownSample(voxel_size);
    pcd_down_ptr->
            EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
            voxel_size*2, 30));
    double radius_feature = voxel_size * 5;
    pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pcd_down_ptr,
                                                                open3d::geometry::KDTreeSearchParamHybrid(radius_feature,
                                                                                                          100));
}

open3d::pipelines::registration::RegistrationResult Registration::execute_global_registration(double voxel_size)
{
  // remember to apply the transformation_ matrix to source_cloud
  // create two point cloud to contain the downsampled point cloud and two structure to contain the features
  // call the Registration::preprocess function on target and transformed source
  // execute global transformation and update the transformation matrix
  // TO COMPLETE
    open3d::pipelines::registration::RegistrationResult result, result_ransac;
    std::shared_ptr<open3d::geometry::PointCloud> source_down_ptr, target_down_ptr;
    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh,target_fpfh;
    source_ = source_.Transform(transformation_);
    Registration::preprocess(source_,voxel_size,source_down_ptr,source_fpfh);
    Registration::preprocess(target_,voxel_size,target_down_ptr,target_fpfh);
    double distance_threshold = voxel_size * 1.5;
    std::vector<std::reference_wrapper<
            const open3d::pipelines::registration::CorrespondenceChecker>>
            correspondence_checker;
    auto correspondence_checker_edge_length =
            open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
                    0.9);
    auto correspondence_checker_distance =
            open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(
                    distance_threshold);

    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);

    result_ransac =  open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
            *source_down_ptr,*target_down_ptr,*source_fpfh,*target_fpfh,
            true, distance_threshold,
            open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
            3,correspondence_checker,
            open3d::pipelines::registration::RANSACConvergenceCriteria(4000000, 500));
    set_transformation(result_ransac.transformation_);
    std::cout<<"global registration"<<std::endl;
    std::cout<<transformation_<<std::endl;
    double threshold = 0.02;
    double relative_fitness=1e-6;
    double relative_rmse=1e-6;
    int max_iteration=1000;
    result = Registration::execute_icp_registration( threshold,  relative_fitness,  relative_rmse,  max_iteration);
    set_transformation(result.transformation_);
    std::cout<<"ICP registration"<<std::endl;
    std::cout<<transformation_<<std::endl;
    return  result;

}

open3d::pipelines::registration::RegistrationResult Registration::execute_icp_registration(double threshold, double relative_fitness, double relative_rmse, int max_iteration)
{
  open3d::pipelines::registration::RegistrationResult result;

    result = open3d::pipelines::registration::RegistrationICP(
            source_,
            target_,
            threshold,
            transformation_,
            open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
            open3d::pipelines::registration::ICPConvergenceCriteria(relative_fitness, relative_rmse, max_iteration));
    set_transformation(result.transformation_);
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

