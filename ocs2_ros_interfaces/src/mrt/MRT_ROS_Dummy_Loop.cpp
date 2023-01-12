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

#include "ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Dummy_Loop::MRT_ROS_Dummy_Loop(MRT_ROS_Interface& mrt, scalar_t mrtDesiredFrequency, scalar_t mpcDesiredFrequency)
    : mrt_(mrt), mrtDesiredFrequency_(mrtDesiredFrequency), mpcDesiredFrequency_(mpcDesiredFrequency) {
  if (mrtDesiredFrequency_ < 0) {
    throw std::runtime_error("MRT loop frequency should be a positive number.");
  }

  if (mpcDesiredFrequency_ > 0) {
    ROS_WARN_STREAM("MPC loop is not realtime! For realtime setting, set mpcDesiredFrequency to any negative number.");
  }
  
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "aa");
  ros::NodeHandle nh_;
  // get arm state
  nowBaseStatesub =
      nh_.subscribe("/base_pose", 1, &MRT_ROS_Dummy_Loop::BasePoseCb, this);
  
  // get arm joint state
  nowArmStatesub=
      nh_.subscribe("/joint_states", 1, &MRT_ROS_Dummy_Loop::ArmStateCb, this);
}

void MRT_ROS_Dummy_Loop::BasePoseCb(const geometry_msgs::PoseConstPtr& msgPtr){
  basePx = msgPtr->position.x ;
  basePy = msgPtr->position.y ;
  
  tf2::Quaternion q(msgPtr->orientation.x,
                    msgPtr->orientation.y,
                    msgPtr->orientation.z,
                    msgPtr->orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  baseYaw  = yaw;

}
void MRT_ROS_Dummy_Loop::ArmStateCb(const sensor_msgs::JointStateConstPtr &msgPtr){
  
  std::map<std::string, int> JointID ={
    {"shoulder_1_joint", 0}, 
    {"shoulder_2_joint", 1},
    {"elbow_joint",      2}, 
    {"wrist_1_joint",    3},
    {"wrist_2_joint",    4},
    {"wrist_3_joint",    5}
  };

  for (int i = 0; i < msgPtr->name.size(); i++){
    auto search = JointID.find(msgPtr->name[i]);

    if(search!=JointID.end()){
      // found
      // std::cout << search->second << std::endl;
      joint[search->second] = msgPtr->position[i];
      jointV[search->second] = msgPtr->velocity[i];
    }
    else{
      // not found
      continue;
    }
  }

    
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::run(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) {
  ROS_INFO_STREAM("Waiting for the initial policy ...");

  // Reset MPC node
  mrt_.resetMpcNode(initTargetTrajectories);

  // Wait for the initial policy
  while (!mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) {
    mrt_.spinMRT();
    mrt_.setCurrentObservation(initObservation);
    ros::Rate(mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  // Pick simulation loop mode
  if (mpcDesiredFrequency_ > 0.0) {
    synchronizedDummyLoop(initObservation, initTargetTrajectories);
  } else {
    realtimeDummyLoop(initObservation, initTargetTrajectories);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::synchronizedDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) {
  // Determine the ratio between MPC updates and simulation steps.
  const auto mpcUpdateRatio = std::max(static_cast<size_t>(mrtDesiredFrequency_ / mpcDesiredFrequency_), size_t(1));

  // Loop variables
  size_t loopCounter = 0;
  SystemObservation currentObservation = initObservation;

  // Helper function to check if policy is updated and starts at the given time.
  // Due to ROS message conversion delay and very fast MPC loop, we might get an old policy instead of the latest one.
  const auto policyUpdatedForTime = [this](scalar_t time) {
    constexpr scalar_t tol = 0.1;  // policy must start within this fraction of dt
    return mrt_.updatePolicy() && std::abs(mrt_.getPolicy().timeTrajectory_.front() - time) < (tol / mpcDesiredFrequency_);
  };

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) {
    std::cout << "### Current time " << currentObservation.time << "\n";

    // Trigger MRT callbacks
    mrt_.spinMRT();

    // Update the MPC policy if it is time to do so
    if (loopCounter % mpcUpdateRatio == 0) {
      // Wait for the policy to be updated
      while (!policyUpdatedForTime(currentObservation.time) && ros::ok() && ros::master::check()) {
        mrt_.spinMRT();
      }
      std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";
    }

    // User-defined modifications before publishing
    modifyObservation(currentObservation);
    
    // Forward simulation
    currentObservation = forwardSimulation(currentObservation);

    // // User-defined modifications before publishing
    // modifyObservation(currentObservation);

    // Publish observation if at the next step we want a new policy
    if ((loopCounter + 1) % mpcUpdateRatio == 0) {
      mrt_.setCurrentObservation(currentObservation);
      std::cout << ">>> Observation is published at " << currentObservation.time << "\n";
    }

    // Update observers
    for (auto& observer : observers_) {
      observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    ++loopCounter;
    ros::spinOnce();
    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::realtimeDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) {
  // Loop variables
  SystemObservation currentObservation = initObservation;

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) {
    std::cout << "### Current time " << currentObservation.time << "\n";

    // Trigger MRT callbacks
    mrt_.spinMRT();

    // Update the policy if a new on was received
    if (mrt_.updatePolicy()) {
      std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";
    }

    // Forward simulation
    currentObservation = forwardSimulation(currentObservation);

    // User-defined modifications before publishing
    // modifyObservation(currentObservation);

    // Publish observation
    mrt_.setCurrentObservation(currentObservation);

    // Update observers
    for (auto& observer : observers_) {
      observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    ros::spinOnce();
    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::modifyObservation(SystemObservation& observation){
  observation.state(0) = basePx;
  observation.state(1) = basePy;
  observation.state(2) = baseYaw ;
  for (int i = 0; i < 6; i++)
  {
    // observation.state(3+i) = joint[i];
    // std::cout <<std::endl << observation.state(3+i) - joint[i] <<std::endl<<std::endl;
  }

  for (int i = 0; i < 6; i++)
  {
    // observation.input(2+i) = jointV[i];
  }
  
  
  // observation.state(3) = 0;
  // observation.state(4) = 0;
  // observation.state(5) = 0;
  // observation.state(6) = 0;
  // observation.state(7) = 0;
  // observation.state(8) = 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation MRT_ROS_Dummy_Loop::forwardSimulation(const SystemObservation& currentObservation) {
  const scalar_t dt = 1.0 / mrtDesiredFrequency_;

  SystemObservation nextObservation;
  nextObservation.time = currentObservation.time + dt;
  if (mrt_.isRolloutSet()) {  // If available, use the provided rollout as to integrate the dynamics.
    mrt_.rolloutPolicy(currentObservation.time, currentObservation.state, dt, nextObservation.state, nextObservation.input,
                       nextObservation.mode);
  } else {  // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
    mrt_.evaluatePolicy(currentObservation.time + dt, currentObservation.state, nextObservation.state, nextObservation.input,
                        nextObservation.mode);
  }

  return nextObservation;
}

}  // namespace ocs2
