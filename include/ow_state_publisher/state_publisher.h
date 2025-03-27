/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef OPEN_WALKER_STATE_PUBLISHER_H_
#define OPEN_WALKER_STATE_PUBLISHER_H_

#include <mutex>

#include <ow_core/common/thread.h>
#include <ow_core/interfaces/i_state_publisher.h>
#include <ow_state_publisher/trajectory_publisher.h>

// interfaces with information to publish
#include <ow_core/interfaces/i_balancer.h>
#include <ow_core/interfaces/i_com_estimator.h>
#include <ow_core/interfaces/i_com_trajectory_generator.h>
#include <ow_core/interfaces/i_command_generator.h>
#include <ow_core/interfaces/i_dcm_planner.h>
#include <ow_core/interfaces/i_foot_compliance.h>
#include <ow_core/interfaces/i_foot_step_planner.h>
#include <ow_core/interfaces/i_foot_trajectory_generator.h>
#include <ow_core/interfaces/i_inverse_kinematics.h>
#include <ow_core/interfaces/i_forward_kinematics.h>
#include <ow_core/interfaces/i_zmp.h>
#include <ow_core/interfaces/i_hw_interface.h>
#include <ow_core/interfaces/i_joint_tracker.h>

// ow msgs
#include <ow_msgs/FootStepList.h>
#include <ow_msgs/DCMPointSetList.h>
#include <ow_msgs/AngularStateStamped.h>
#include <ow_msgs/CartesianStateStamped.h>
#include <ow_msgs/JointStateStamped.h>
#include <ow_msgs/LinearStateStamped.h>
#include <ow_msgs/VectorStamped.h>
#include <ow_msgs/FlagsStamped.h>

// ros msgs
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

// transformations
#include <tf/transform_broadcaster.h>

// visualizations
#include <ow_core/geometry/shapes.h>

/*!
 * \brief Open Walker state publisher module namespace. These classes implement
 * the different componets of the balance controller.
 */
namespace ow_pub
{

/*!
 * \brief The StatePublisher class
 *
 * This class implements the StatePublisher module of the
 * openwalker framework, which publishes the information of module 
 * ouputs as a ros topics.
 */
class StatePublisher : 
  public ow::IStatePublisher,
  public ow::Thread
{
public:
  typedef ow::IStatePublisher Base;
  typedef ow::Thread Thread; 

protected:
  bool stopped_;                    //!< Stop flag.
  bool is_odom_pub_;                //!< Should the fk odom be published
  ow::Scalar rate_;                 //!< Loop rate for publishing.
  std::mutex data_mutex_;           //!< Thread save data access.

  ow::Parameter parameter_;         //!< configuration
  std::string frame_w;              //!< Reference world frame.
  std::string frame_b;              //!< Floating base frame.
  std::string frame_r;              //!< Right foot frame.
  std::string frame_l;              //!< Left foot frame.

  // interface pointers
  const ow::IHwInterface* robot_;
  const ow::IBalancer* balancer_;
  const ow::ICOMEstimator* com_estimator_;
  const ow::ICOMTrajectoryGenerator* com_trajectory_generator_;
  const ow::ICommandGenerator* command_generator_;
  const ow::IDCMPlanner* dcm_planner_;
  const ow::IFootCompliance* foot_compliance_;
  const ow::IFootStepPlanner* step_planner_;
  const ow::IFootTrajectoryGenerator* foot_trajectory_generator_;
  const ow::IInverseKinematics* inverse_kinematics_;
  const ow::IForwardKinematics* forward_kinematics_;
  const ow::IForwardKinematics* forward_kinematics_cmd_;
  const ow::IZmpEstimator* zmp_estimator_;
  const ow::IJointTracker* joint_tracker_;
  std::vector<const GenericModuleBase*> interfaces_;

  // state information
  bool state_updated_;

  // flags
  ow::Flags flags_;
  ros::Publisher flags_pub_;

  // robot
  ow::ImuSensor imu_;
  ow::Wrench ft_left_;
  ow::Wrench ft_right_;
  ros::Publisher imu_pub_;
  ros::Publisher imu_pose_pub_;
  ros::Publisher ft_left_pub_;
  ros::Publisher ft_right_pub_;
  ros::Publisher f_normal_right_pub_;
  ros::Publisher f_normal_left_pub_;

  // balancer
  ow::CartesianState Xoff_com_;
  ow::LinearState p_d_w_;
  ow::Points2d support_polygon_w_;
  ros::Publisher Xoff_com_pub_;
  ros::Publisher p_d_w_pub_;

  // com estimator
  ow::CartesianState Xf_com_w_;
  ow::LinearState dcm_w_;
  ros::Publisher Xf_com_w_pub_;
  ros::Publisher dcm_w_pub_;

  // com trajectory generator
  ow::LinearState p_r_w_;
  ow::LinearState dcm_r_w_;
  ow::CartesianState X_r_com_w_;
  ros::Publisher p_r_w_pub_;
  ros::Publisher dcm_r_w_pub_;
  ros::Publisher X_r_com_w_pub_;
  TrajectoryPublisher X_r_com_w_traj_;

  // command generator
  ow::CartesianState Xcmd_l_w_;
  ow::CartesianState Xcmd_r_w_;
  ow::CartesianState Xcmd_com_w_;
  ros::Publisher Xcmd_l_w_pub_;
  ros::Publisher Xcmd_r_w_pub_;
  ros::Publisher Xcmd_com_w_pub_;
  TrajectoryPublisher Xcmd_com_w_traj_;

  // dcm planner
  ow::DCMPointSetList dcm_point_set_list_;
  ros::Publisher dcm_point_set_list_pub_;
  
  // foot step planner
  ow::FootStepList foot_steps_;
  ros::Publisher foot_steps_pub_;

  // foot trajectory generator
  ow::CartesianState Xref_l_w_;
  ow::CartesianState Xref_r_w_;
  ros::Publisher Xref_l_w_pub_;
  ros::Publisher Xref_r_w_pub_;

  // forward kinematics or real robot
  ow::CartesianState Xreal_l_w_;
  ow::CartesianState Xreal_r_w_;
  ow::CartesianState Xreal_com_w_;
  ros::Publisher Xreal_l_w_pub_;
  ros::Publisher Xreal_r_w_pub_;
  ros::Publisher Xreal_com_w_pub_;
  TrajectoryPublisher Xreal_l_w_traj_;
  TrajectoryPublisher Xreal_r_w_traj_;
  TrajectoryPublisher Xreal_com_w_traj_;

  // forward kinematics of virtual robot
  ow::CartesianState Xv_l_w_;
  ow::CartesianState Xv_r_w_;
  ow::CartesianState Xv_com_w_;
  ros::Publisher Xv_l_w_pub_;
  ros::Publisher Xv_r_w_pub_;
  ros::Publisher Xv_com_w_pub_;
  TrajectoryPublisher Xv_l_w_traj_;
  TrajectoryPublisher Xv_r_w_traj_;
  TrajectoryPublisher Xv_com_w_traj_;

  // inverse kinematics
  ow::JointState q_;
  ros::Publisher q_pub_;

  // foot compliance
  ow::CartesianState Xoff_r_;
  ow::CartesianState Xoff_l_;
  ros::Publisher Xoff_r_pub_;
  ros::Publisher Xoff_l_pub_;
  
  // zmp estimator
  ow::LinearState p_w_;
  ros::Publisher p_w_pub_;
  TrajectoryPublisher p_w_traj_;

  // joint_tracker
  ow::JointState joint_tracker_state_;
  ow::Scalar left_ankle_offset_;
  ow::Scalar left_groin_offset_;
  ow::Scalar right_ankle_offset_;
  ow::Scalar right_groin_offset_;

  // gui
  ros::Publisher joint_tracker_state_pub_;
  ros::Publisher left_ankle_offset_pub_;
  ros::Publisher left_groin_offset_pub_;
  ros::Publisher right_ankle_offset_pub_;
  ros::Publisher right_groin_offset_pub_;

  // visualization
  ros::Publisher markers_pub_;

  // world frame broadcaster
  tf::TransformBroadcaster broadcaster;

  // robot odometry publisher
  ros::Publisher odom_pub_;

public:
  /**
   * @brief Construct a new State Publisher object
   * 
   */
  StatePublisher();

  /**
   * @brief Destroy the State Publisher object
   * 
   */
  virtual ~StatePublisher();

  void add(const ow::IHwInterface* robot);
  void add(const ow::IBalancer* balancer);
  void add(const ow::ICOMEstimator* com_estimator);
  void add(const ow::ICOMTrajectoryGenerator* com_trajectory_generator);
  void add(const ow::ICommandGenerator* command_generator);
  void add(const ow::IDCMPlanner* command_generator);
  void add(const ow::IFootCompliance* foot_compliance);
  void add(const ow::IFootStepPlanner* step_planner);
  void add(const ow::IFootTrajectoryGenerator* foot_trajectory_generator);
  void add(const ow::IInverseKinematics* inverse_kinematics);
  void add(const ow::IForwardKinematics* forward_kinematics, bool is_cmd);
  void add(const ow::IZmpEstimator* zmp_estimator);
  void add(const ow::IJointTracker* joint_tracker);

  /**
   * @brief performs update step of the module, called periodically
   * 
   * @param flags the current status flags of the robot
   * @param time the current robot time
   * @param dt the elapsed time since last update call
   */
  void update(
    ow::Flags& flags, 
    const ros::Time& time, 
    const ros::Duration& dt);
    
protected:
  /*!
   * \brief Initialization of StatePublisher module
   */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

  /**
   * @brief start the module, called befor update
   * 
   * @param flags the current status flags of the robot
   * @param time the current robot time
   */
  virtual void start(ow::Flags& flags, const ros::Time& time);

  /*!
   * \brief The thread execution function.
   */
  virtual void run();

  /**
   * @brief publish the information to ros
   * 
   * @param time the current robot time
   */
  void publish(const ros::Time& time);

  /** 
  * \brief Stop the module, called befor stopping
  *
  * \param Stopping time
  */
  virtual void stop(ow::Flags& flags, const ros::Time& time);

};

}

#endif
