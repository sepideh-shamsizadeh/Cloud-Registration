#include <iostream>
#include <Eigen/Dense>
#include "Registration.h"



int main(int argc, char *argv[]) {
  //load source and target cloud
  Registration registration(argv[1], argv[2]);
  //show point cloud
  registration.draw_registration_result();
  //show initial tranformation matrix
  std::cout<<registration.get_transformation()<<std::endl;
  //execute global registration
  registration.execute_global_registration();
  //show point cloud
  registration.draw_registration_result();
  //show tranformation matrix after global registration
  std::cout<<registration.get_transformation()<<std::endl;

  //icp registration
  auto result = registration.execute_icp_registration();
  //show point cloud
  registration.draw_registration_result();
  //show tranformation matrix after icp 
  std::cout<<registration.get_transformation()<<std::endl;

  //save transformation matrix and registered cloud 
  registration.write_tranformation_matrix(argv[3]);
  registration.save_merged_cloud(argv[4]);
  std::cout<<"fitness: "<<result.fitness_<<std::endl;
  std::cout<<"inlier_rmse: "<<result.inlier_rmse_<<std::endl;



 
  return 0;
}
