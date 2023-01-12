/******************************************************************************
 Copyright (c) 2020, Farbod Farshidian. All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
 contributors may be used to endorse or promote products derived from
 this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <ocs2_self_collision/externalCollision.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>


namespace ocs2 {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
externalCollision::externalCollision(PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
    : pinocchioGeometryInterface_(std::move(pinocchioGeometryInterface)), minimumDistance_(minimumDistance) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "externalCollision");
      ros::NodeHandle n_;
      sub = n_.subscribe("pos_vel", 1, &externalCollision::ObsCB,this);

      boost::interprocess::shared_memory_object shdmem(boost::interprocess::open_or_create, "OBSshm", boost::interprocess::read_write);
      shdmem.truncate(sizeof(nav_msgs::Odometry));
      boost::interprocess::mapped_region region(shdmem, boost::interprocess::read_write);
      // std::cout << std::hex << region.get_address() << std::endl;
      // std::cout << std::dec << region.get_size() << std::endl;
      nav_msgs::Odometry* ObsData = static_cast<nav_msgs::Odometry*>(region.get_address());
      ObsData->pose.pose.position.x = 1.0;
      ObsData->pose.pose.position.y = 0;
      ObsData->pose.pose.position.z = 1.2;
      ObsData->twist.twist.linear.x=0;
      ObsData->twist.twist.linear.y=0;
      ObsData->twist.twist.linear.z=0;
    }


void externalCollision::ObsCB(const nav_msgs::Odometry::ConstPtr& obsMsg){
  
  // std::cout << "----------- voxblox obsMsg Obs x : " << obsMsg->pose.pose.position.x << std::endl 
  //           << "----------- voxblox obsMsg Obs y : " << obsMsg->pose.pose.position.y << std::endl;
  boost::interprocess::shared_memory_object shdmem(boost::interprocess::open_or_create, "OBSshm", boost::interprocess::read_write);
  shdmem.truncate(sizeof(nav_msgs::Odometry));
  boost::interprocess::mapped_region region(shdmem, boost::interprocess::read_write);
  // std::cout << std::hex << region.get_address() << std::endl;
  // std::cout << std::dec << region.get_size() << std::endl;
  nav_msgs::Odometry* ObsData = static_cast<nav_msgs::Odometry*>(region.get_address());
  // posistion
  ObsData->pose.pose.position.x = obsMsg->pose.pose.position.x;
  ObsData->pose.pose.position.y = obsMsg->pose.pose.position.y;
  ObsData->pose.pose.position.z = obsMsg->pose.pose.position.z;
  // velocity
  ObsData->twist.twist.linear.x = obsMsg->twist.twist.linear.x;
  ObsData->twist.twist.linear.y = obsMsg->twist.twist.linear.y;
  ObsData->twist.twist.linear.z = obsMsg->twist.twist.linear.z;
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t externalCollision::getValue(const PinocchioInterface& pinocchioInterface) const {
  // get distance
  // const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometryInterface_.computeDistances(pinocchioInterface);
  const auto& geometryModel = pinocchioGeometryInterface_.getGeometryModel();
  const auto& data = pinocchioInterface.getData();
  const auto& collisionPair = geometryModel.collisionPairs[0];
  const auto& joint1 = geometryModel.geometryObjects[collisionPair.first].parentJoint;
  const auto& joint2 = geometryModel.geometryObjects[collisionPair.second].parentJoint;
  const vector3_t joint1Position = data.oMi[joint1].translation();
  const vector3_t joint2Position = data.oMi[joint2].translation();
  // double obsData[3] = {InData.pose.pose.position.x, InData.pose.pose.position.y, InData.pose.pose.position.z};
  // std::cout << joint2Position << std::endl<< std::endl;

  // joint1Position is base_link
  // joint2Position is link_4
  
  boost::interprocess::shared_memory_object shdmem(boost::interprocess::open_only, "OBSshm", boost::interprocess::read_only);

  boost::interprocess::mapped_region region2(shdmem, boost::interprocess::read_only);
  // std::cout << std::hex << region2.get_address() << std::endl;
  // std::cout << std::dec << region2.get_size() << std::endl;
  nav_msgs::Odometry* outObsData = static_cast<nav_msgs::Odometry*>(region2.get_address());
  // std::cout << outObsData->pose.pose.position.x << std::endl<< std::endl;
  

  double obsData[3] = {outObsData->pose.pose.position.x,outObsData->pose.pose.position.y,outObsData->pose.pose.position.z};
  
  
  std::vector<vector3_t> datavector;
  datavector.push_back(joint1Position);
  datavector.push_back(joint2Position);
  std::vector<double> distanceArray;
  // for (int i = 1; i < 2; i++)
  // {
  //   double distance = std::sqrt(std::pow(datavector[i](0)-(obsData[0]+outObsData->twist.twist.linear.x),2)
  //                              +std::pow(datavector[i](1)-(obsData[1]+outObsData->twist.twist.linear.y),2)
  //                              +std::pow(datavector[i](2)-(obsData[2]+outObsData->twist.twist.linear.z),2));
  //   distanceArray.push_back(distance);  
  // }

  // base_link
    double distance = std::sqrt(std::pow(datavector[0](0)-(obsData[0]+outObsData->twist.twist.linear.x),2)
                               +std::pow(datavector[0](1)-(obsData[1]+outObsData->twist.twist.linear.y),2));
    distanceArray.push_back(distance); 

  // link_4
    distance = std::sqrt(std::pow(datavector[1](0)-(obsData[0]+outObsData->twist.twist.linear.x),2)
                        +std::pow(datavector[1](1)-(obsData[1]+outObsData->twist.twist.linear.y),2)
                        +std::pow(datavector[1](2)-(obsData[2]+outObsData->twist.twist.linear.z),2));
    distanceArray.push_back(distance);      

  // std::vector<double> distanceArray;
  // distanceArray.push_back(0.1);
  // distanceArray.push_back(0.1);
  

  // std::cout << "distanceArray.size() is : " << distanceArray.size() << std::endl;
  vector_t violations = vector_t::Zero(distanceArray.size());
  // for (size_t i = 0; i < distanceArray.size(); ++i) {
  //   violations[i] = distanceArray.at(i) - 0.4;
  //   // std::cout << "distanceArray[i].min_distance is : " << distanceArray[i].min_distance << std::endl;
  // }

  // base_link
  violations[0] = distanceArray.at(0) - 0.7;
  // link_4
  violations[1] = distanceArray.at(1) - 0.4;

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> externalCollision::getLinearApproximation(const PinocchioInterface& pinocchioInterface) const {
  // const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometryInterface_.computeDistances(pinocchioInterface);
  // double dataret[3];
  // pinocchioGeometryInterface_.retData(dataret);

  const auto& geometryModel = pinocchioGeometryInterface_.getGeometryModel();
  const auto& data = pinocchioInterface.getData();
  const auto& collisionPair = geometryModel.collisionPairs[0];
  const auto& joint1 = geometryModel.geometryObjects[collisionPair.first].parentJoint;
  const auto& joint2 = geometryModel.geometryObjects[collisionPair.second].parentJoint;
  const vector3_t joint1Position_t = data.oMi[joint1].translation();
  const vector3_t joint2Position_t = data.oMi[joint2].translation();
  // double obsData[3] = {InData.pose.pose.position.x, InData.pose.pose.position.y, InData.pose.pose.position.z};
  // double obsData[3] = {0.5,0.0,1.0};


  boost::interprocess::shared_memory_object shdmem(boost::interprocess::open_only, "OBSshm", boost::interprocess::read_only);

  boost::interprocess::mapped_region region2(shdmem, boost::interprocess::read_only);
  // std::cout << std::hex << region2.get_address() << std::endl;
  // std::cout << std::dec << region2.get_size() << std::endl;
  nav_msgs::Odometry* outObsData = static_cast<nav_msgs::Odometry*>(region2.get_address());
  // std::cout << outObsData->pose.pose.position.x << std::endl<< std::endl;


  double obsData[3] = {outObsData->pose.pose.position.x,outObsData->pose.pose.position.y,outObsData->pose.pose.position.z};



  std::vector<vector3_t> datavector;
  datavector.push_back(joint1Position_t);
  datavector.push_back(joint2Position_t);

  
  // double obsData[3] = {0.5, 0,0};
  // std::cout << outObsData->twist.twist.linear.x <<std::endl<<std::endl;

  std::vector<double> distanceArray;
  // for (int i = 1; i < 2; i++)
  // {
  //   // double distance = std::sqrt(std::pow(datavector[i](0)-(obsData[0]-0.5),2)+std::pow(datavector[i](1)-obsData[1],2)+std::pow(datavector[i](2)-obsData[2],2));
  //   double distance = std::sqrt(std::pow(datavector[i](0)-(obsData[0]+outObsData->twist.twist.linear.x),2)
  //                              +std::pow(datavector[i](1)-(obsData[1]+outObsData->twist.twist.linear.y),2)
  //                              +std::pow(datavector[i](2)-(obsData[2]+outObsData->twist.twist.linear.z),2));
                               
  //   distanceArray.push_back(distance);  
  // }

  // base_link
    double distance = std::sqrt(std::pow(datavector[0](0)-(obsData[0]+outObsData->twist.twist.linear.x),2)
                               +std::pow(datavector[0](1)-(obsData[1]+outObsData->twist.twist.linear.y),2));
    distanceArray.push_back(distance); 

  // link_4
    distance = std::sqrt(std::pow(datavector[1](0)-(obsData[0]+outObsData->twist.twist.linear.x),2)
                        +std::pow(datavector[1](1)-(obsData[1]+outObsData->twist.twist.linear.y),2)
                        +std::pow(datavector[1](2)-(obsData[2]+outObsData->twist.twist.linear.z),2));
    distanceArray.push_back(distance);   

  // distanceArray.push_back(0.1);
  // distanceArray.push_back(0.1);
  // std::cout << "Done" << std::endl;

  const auto& model = pinocchioInterface.getModel();
  // const auto& data = pinocchioInterface.getData();

  // const auto& geometryModel = pinocchioGeometryInterface_.getGeometryModel();

  
  
  vector_t f(distanceArray.size());
  matrix_t dfdq(distanceArray.size(), model.nq);
  for (size_t i = 0; i < distanceArray.size(); ++i) {
    // Distance violation
    // f[i] = distanceArray[i].min_distance - minimumDistance_;
    f[i] = distanceArray[i] - 0.4;

    // Jacobian calculation
    const auto& collisionPair = geometryModel.collisionPairs[i];
    const auto& joint1 = geometryModel.geometryObjects[collisionPair.second].parentJoint; // link_4
    const auto& joint2 = geometryModel.geometryObjects[collisionPair.first].parentJoint;  // base_link

    // link_4

    // We need to get the jacobian of the point on the first object; use the joint jacobian translated to the point
    const vector3_t joint1Position = data.oMi[joint1].translation();
    // std::cout << "=================" << std::endl << joint1Position << std::endl << "=================" << std::endl;
    // const vector3_t pt1Offset = distanceArray[i].nearest_points[1] - joint1Position;

    matrix_t joint1Jacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, joint1, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint1Jacobian);
    // Jacobians from pinocchio are given as
    // [ position jacobian ]
    // [ rotation jacobian ]
    // const matrix_t pt1Jacobian = joint1Jacobian.topRows(3) - skewSymmetricMatrix(pt1Offset) * joint1Jacobian.bottomRows(3);

    // We need to get the jacobian of the point on the second object; use the joint jacobian translated to the point
    // const vector3_t joint2Position = data.oMi[joint2].translation();

    // std::cout << "=================" << std::endl << joint2Position << std::endl << "=================" << std::endl;

    // const vector3_t pt2Offset = distanceArray[i].nearest_points[1] - joint2Position;
    // matrix_t joint2Jacobian = matrix_t::Zero(6, model.nv);
    // pinocchio::getJointJacobian(model, data, joint2, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint2Jacobian);
    // const matrix_t pt2Jacobian = joint2Jacobian.topRows(3) - skewSymmetricMatrix(pt2Offset) * joint2Jacobian.bottomRows(3);

    // To get the (approximate) jacobian of the distance, get the difference between the two nearest point jacobians, then multiply by the
    // vector from point to point
    // const matrix_t differenceJacobian = pt2Jacobian - pt1Jacobian;
    const matrix_t differenceJacobian = -joint1Jacobian;
    // TODO(perry): is there a way to calculate a correct jacobian for the case of distanceVector = 0?
    // const vector3_t distanceVector = distanceArray[i] > 0
    //                                      ? (distanceArray[i].nearest_points[1] - distanceArray[i].nearest_points[0]).normalized()
    //                                      : (distanceArray[i].nearest_points[0] - distanceArray[i].nearest_points[1]).normalized();
    vector3_t distanceVector ; 
    distanceVector(0) = obsData[0] - joint1Position(0);
    distanceVector(1) = obsData[1] - joint1Position(1);
    distanceVector(2) = obsData[2] - joint1Position(2);

    // distanceVector.normalize();

    dfdq.row(i).noalias() = distanceVector.transpose() * differenceJacobian;

    // base_link
    ++i;
    f[i] = distanceArray[i] - 0.7;

    const vector3_t joint2Position = data.oMi[joint2].translation(); //arm tool0
    matrix_t joint2Jacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, joint2, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint2Jacobian);
    const matrix_t differenceJacobian2 = -joint2Jacobian;
    
    vector3_t distanceVector2 ; 
    distanceVector2(0) = obsData[0] - joint2Position(0);
    distanceVector2(1) = obsData[1] - joint2Position(1);
    distanceVector2(2) = obsData[2] - joint2Position(2);


    dfdq.row(i).noalias() = distanceVector2.transpose() * differenceJacobian2;
  }  // end of i loop

  return {f, dfdq};
}

}  // namespace ocs2
