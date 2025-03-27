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

#include <ow_state_publisher/state_publisher.h>

namespace ow_pub
{

  StatePublisher::StatePublisher() :
    Base("state_publisher"),
    Thread(),
    stopped_(false),
    robot_(nullptr),
    balancer_(nullptr),
    com_estimator_(nullptr),
    com_trajectory_generator_(nullptr),
    command_generator_(nullptr),
    dcm_planner_(nullptr),
    foot_compliance_(nullptr),
    step_planner_(nullptr),
    foot_trajectory_generator_(nullptr),
    inverse_kinematics_(nullptr),
    forward_kinematics_(nullptr),
    forward_kinematics_cmd_(nullptr),
    zmp_estimator_(nullptr),
    joint_tracker_(nullptr),
    state_updated_(false),
    is_odom_pub_(true)
  {
  }

  StatePublisher::~StatePublisher()
  {
    if(Thread::isRunning())
    {
      stopped_ = true;
    }
  }

  bool StatePublisher::init(const ow::Parameter& parameter, ros::NodeHandle& nh)
  {
    // get global ow parameter
    parameter.get("publish_rate", rate_);

    // module parameter
    parameter_.add<std::string>("world");
    parameter_.add<std::string>("body/legs/left/base");
    parameter_.add<std::string>("body/legs/left/ee");
    parameter_.add<std::string>("body/legs/right/ee");
    if (!parameter_.load(nh, "kinematics"))
    {
      ROS_ERROR("%s::initialize: Config loading failed.", Base::name().c_str());
      return false;
    }
    parameter_.get("world", frame_w);
    parameter_.get("body/legs/left/base", frame_b);
    parameter_.get("body/legs/left/ee", frame_l);
    parameter_.get("body/legs/right/ee", frame_r);

    // setup ros connections
        
    // flags
    flags_pub_ = nh.advertise<ow_msgs::FlagsStamped>("flags", 1);

    if(robot_)
    {
      ft_left_pub_ = nh.advertise<geometry_msgs::WrenchStamped>(
            robot_->name() + "/ft_left", 1);
      ft_right_pub_ = nh.advertise<geometry_msgs::WrenchStamped>(
            robot_->name() + "/ft_right", 1);
      f_normal_left_pub_ = nh.advertise<std_msgs::Float32>(robot_->name() + "/f_normal_right", 1);
      f_normal_right_pub_ = nh.advertise<std_msgs::Float32>(robot_->name() + "/f_normal_left", 1);
      imu_pub_ = nh.advertise<sensor_msgs::Imu>(robot_->name() + "/imu", 1);
      imu_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(robot_->name() + "/imu_pose", 1);
    }

    // balancer
    if(balancer_)
    {
      Xoff_com_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            balancer_->name() + "/com_offset", 1);
      p_d_w_pub_ = nh.advertise<ow_msgs::LinearStateStamped>(
            balancer_->name() + "/zmp_desired", 1);
    }

    // com estimator
    if(com_estimator_)
    {
      Xf_com_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            com_estimator_->name() + "/com", 1);
      dcm_w_pub_ = nh.advertise<ow_msgs::LinearStateStamped>(
            com_estimator_->name() + "/dcm", 1);
    }

    // com trajectory generator
    if(com_trajectory_generator_)
    {
      p_r_w_pub_ = nh.advertise<ow_msgs::LinearStateStamped>(
            com_trajectory_generator_->name() + "/zmp", 1);
      dcm_r_w_pub_ = nh.advertise<ow_msgs::LinearStateStamped>(
            com_trajectory_generator_->name() + "/dcm", 10);
      X_r_com_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            com_trajectory_generator_->name() + "/com", 1);
      X_r_com_w_traj_.advertise(nh, com_trajectory_generator_->name()
                                + "/com_path", frame_w, 600);
    }

    // command generator
    if(command_generator_)
    {
      Xcmd_l_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            command_generator_->name() + "/left", 1);
      Xcmd_r_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            command_generator_->name() + "/right", 1);
      Xcmd_com_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            command_generator_->name() + "/com", 1);
      Xcmd_com_w_traj_.advertise(nh, command_generator_->name()
                                + "/com_path", frame_w, 600);
    }

    // dcm planner
    if(dcm_planner_)
    {
      dcm_point_set_list_pub_ = nh.advertise<ow_msgs::DCMPointSetList>(
            dcm_planner_->name() + "/dcm_points", 1);
    }
    
    // foot step planner
    if(step_planner_)
    {
      foot_steps_pub_ = nh.advertise<ow_msgs::FootStepList>(
            step_planner_->name() + "/foot_steps", 1);
    }

    // foot trajectory generator
    if(foot_trajectory_generator_)
    {
      Xref_l_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            foot_trajectory_generator_->name() + "/left", 1);
      Xref_r_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            foot_trajectory_generator_->name() + "/right", 1);
    }

    // forward kinematics real robot
    if(forward_kinematics_)
    {
      Xreal_l_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            forward_kinematics_->name() + "/real/left", 1);
      Xreal_r_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            forward_kinematics_->name() + "/real/right", 1);
      Xreal_com_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            forward_kinematics_->name() + "/real/com", 1);

      Xreal_l_w_traj_.advertise(nh, forward_kinematics_->name()
                            + "/real/left_path", frame_w, 600);
      Xreal_r_w_traj_.advertise(nh, forward_kinematics_->name()
                            + "/real/right_path", frame_w, 600);
      Xreal_com_w_traj_.advertise(nh, forward_kinematics_->name()
                            + "/real/com_path", frame_w, 600);
    }
    
    // forward kinematics virtual robot
    if(forward_kinematics_cmd_)
    {
      Xv_l_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            forward_kinematics_cmd_->name() + "/cmd/left", 1);
      Xv_r_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            forward_kinematics_cmd_->name() + "/cmd/right", 1);
      Xv_com_w_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            forward_kinematics_cmd_->name() + "/cmd/com", 1);

      Xv_l_w_traj_.advertise(nh, forward_kinematics_->name()
                            + "/cmd/left_path", frame_w, 600);
      Xv_r_w_traj_.advertise(nh, forward_kinematics_->name()
                            + "/cmd/right_path", frame_w, 600);
      Xv_com_w_traj_.advertise(nh, forward_kinematics_->name()
                            + "/cmd/com_path", frame_w, 600);
    }

    // foot compliance
    if(foot_compliance_)
    {
      Xoff_r_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            foot_compliance_->name() + "/left_offset", 1);
      Xoff_l_pub_ = nh.advertise<ow_msgs::CartesianStateStamped>(
            foot_compliance_->name() + "/right_offset", 1);
    }

    // inverse kinematics
    if(inverse_kinematics_)
    {
      q_pub_ = nh.advertise<sensor_msgs::JointState>(
            inverse_kinematics_->name() + "/jointstate", 1);
    }
    
    // zmp estimator
    if(zmp_estimator_)
    {
      p_w_pub_ = nh.advertise<ow_msgs::LinearStateStamped>(
            zmp_estimator_->name() + "/zmp", 1);
      p_w_traj_.advertise(nh, zmp_estimator_->name()
                          + "/zmp_path", frame_w, 600);
    }

    // joint tracker
    if(joint_tracker_)
    {
      joint_tracker_state_pub_ = nh.advertise<sensor_msgs::JointState>(
            joint_tracker_->name() + "/jointstate", 1);
      left_ankle_offset_pub_ = nh.advertise<std_msgs::Float32>(
            joint_tracker_->name() + "/left_ankle_offset", 1);
      left_groin_offset_pub_ = nh.advertise<std_msgs::Float32>(
            joint_tracker_->name() + "/left_groin_offset", 1);
      right_ankle_offset_pub_ = nh.advertise<std_msgs::Float32>(
            joint_tracker_->name() + "/right_ankle_offset", 1);
      right_groin_offset_pub_ = nh.advertise<std_msgs::Float32>(
            joint_tracker_->name() + "/right_groin_offset", 1);
    }

    // odometry
    if(odom_pub_ && forward_kinematics_cmd_)
    {
      odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    }

    // visualization
    markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
                    "/markers", 1);
    
    return true;
  }

  void StatePublisher::add(const ow::IHwInterface* robot)
  {
    robot_ = robot;
    interfaces_.push_back(robot_);
  }

  void StatePublisher::add(const ow::IBalancer* balancer)
  {
    balancer_ = balancer;
    interfaces_.push_back(balancer_);
  }

  void StatePublisher::add(const ow::ICOMEstimator* com_estimator)
  {
    com_estimator_ = com_estimator;
    interfaces_.push_back(com_estimator_);
  }

  void StatePublisher::add(
      const ow::ICOMTrajectoryGenerator* com_trajectory_generator)
  {
    com_trajectory_generator_ = com_trajectory_generator;
    interfaces_.push_back(com_estimator_);
  }

  void StatePublisher::add(const ow::ICommandGenerator* command_generator)
  {
    command_generator_ = command_generator;
    interfaces_.push_back(com_estimator_);
  }

  void StatePublisher::add(const ow::IDCMPlanner* dcm_planner)
  {
    dcm_planner_ = dcm_planner;
    interfaces_.push_back(dcm_planner_);
  }

  void StatePublisher::add(const ow::IFootCompliance* foot_compliance)
  {
    foot_compliance_ = foot_compliance;
    interfaces_.push_back(foot_compliance_);
  }

  void StatePublisher::add(const ow::IFootStepPlanner* step_planner)
  {
    step_planner_ = step_planner;
    interfaces_.push_back(step_planner_);
  }

  void StatePublisher::add(
      const ow::IFootTrajectoryGenerator* foot_trajectory_generator)
  {
    foot_trajectory_generator_ = foot_trajectory_generator;
    interfaces_.push_back(foot_trajectory_generator_);
  }

  void StatePublisher::add(const ow::IInverseKinematics* inverse_kinematics)
  {
    inverse_kinematics_ = inverse_kinematics;
    interfaces_.push_back(inverse_kinematics_);
  }

  void StatePublisher::add(
      const ow::IForwardKinematics* forward_kinematics, bool is_cmd)
  {
    if(!is_cmd)
    {
      forward_kinematics_ = forward_kinematics;
      interfaces_.push_back(forward_kinematics_);
    }
    else
    {
      forward_kinematics_cmd_ = forward_kinematics;
      interfaces_.push_back(forward_kinematics_cmd_);
    }
  }

  void StatePublisher::add(const ow::IZmpEstimator* zmp_estimator)
  {
    zmp_estimator_ = zmp_estimator;
    interfaces_.push_back(zmp_estimator_);
  }

  void StatePublisher::add(const ow::IJointTracker* joint_tracker)
  {
    joint_tracker_ = joint_tracker;
    interfaces_.push_back(joint_tracker_);
  }

  void StatePublisher::start(ow::Flags& flags, const ros::Time& time)
  {
    // check if modules are ready
    for(size_t i = 0; i < interfaces_.size(); ++i)
    {
      if(interfaces_[i])
      {
        if(!interfaces_[i]->isInitialized() && !interfaces_[i]->isRunning())
        {
          ROS_WARN("%s::start: Module '%s' is not initialized.", 
            Base::name().c_str(), interfaces_[i]->name().c_str());
        }
      }
    }

    // start the threads
    stopped_ = false;
    Thread::start();
  }

  void StatePublisher::run()
  {
    // loop here
    ros::Rate loop_rate(rate_);
    while(ros::ok() && !stopped_)
    { 
      ros::Time time = ros::Time::now();
      if(state_updated_)
      {
        publish(time);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void StatePublisher::publish(const ros::Time& time)
  {
    // look for message generation
    data_mutex_.lock();

    // flags
    ow_msgs::FlagsStamped flags_msg;
    flags_msg.flags = flags_;
    flags_msg.header.stamp = time;
    
    // force torque
    std_msgs::Float32 f_normal_left_msg;
    f_normal_left_msg.data = ft_left_.linear().z();
    geometry_msgs::WrenchStamped ft_left_msg;
    ft_left_msg.wrench = ft_left_;
    ft_left_msg.header.frame_id = frame_l;
    ft_left_msg.header.stamp = time;
    std_msgs::Float32 f_normal_right_msg;
    f_normal_right_msg.data = ft_right_.linear().z();
    geometry_msgs::WrenchStamped ft_right_msg;
    ft_right_msg.wrench = ft_right_;
    ft_right_msg.header.frame_id = frame_r;
    ft_right_msg.header.stamp = time;
    sensor_msgs::Imu imu_msg = imu_;
    imu_msg.header.frame_id = frame_b;
    imu_msg.header.stamp = time;
    geometry_msgs::PoseStamped imu_pose_msg;
    imu_pose_msg.header.frame_id = frame_w;
    imu_pose_msg.header.stamp = time;
    imu_pose_msg.pose.position = Xv_com_w_.pos().linear();
    imu_pose_msg.pose.orientation = imu_.Q();

    ow_msgs::CartesianStateStamped Xoff_com_msg;
    Xoff_com_msg.state = Xoff_com_;
    Xoff_com_msg.header.stamp = time;
    ow_msgs::LinearStateStamped p_d_w_msg;
    p_d_w_msg.state = p_d_w_;
    p_d_w_msg.header.stamp = time;

    ow_msgs::CartesianStateStamped Xf_com_w_msg;
    Xf_com_w_msg.state = Xf_com_w_;
    Xf_com_w_msg.header.stamp = time;
    ow_msgs::LinearStateStamped dcm_w_msg;
    dcm_w_msg.state = dcm_w_;
    dcm_w_msg.header.stamp = time;

    ow_msgs::LinearStateStamped p_r_w_msg; 
    p_r_w_msg.state = p_r_w_;
    p_r_w_msg.header.stamp = time;
    ow_msgs::LinearStateStamped dcm_r_w_msg;
    dcm_r_w_msg.state = dcm_r_w_;  
    dcm_r_w_msg.header.stamp = time;
    ow_msgs::CartesianStateStamped X_r_com_w_msg;
    X_r_com_w_msg.state = X_r_com_w_;
    X_r_com_w_msg.header.stamp = time;
    X_r_com_w_traj_.update(X_r_com_w_.pos(), time);

    ow_msgs::CartesianStateStamped Xcmd_l_w_msg;
    Xcmd_l_w_msg.state = Xcmd_l_w_;
    Xcmd_l_w_msg.header.stamp = time;
    ow_msgs::CartesianStateStamped Xcmd_r_w_msg;
    Xcmd_r_w_msg.state = Xcmd_r_w_;
    Xcmd_r_w_msg.header.stamp = time;
    ow_msgs::CartesianStateStamped Xcmd_com_w_msg;
    Xcmd_com_w_msg.state = Xcmd_com_w_;
    Xcmd_com_w_msg.header.stamp = time;
    Xcmd_com_w_traj_.update(Xcmd_com_w_.pos(), time);

    ow_msgs::CartesianStateStamped Xoff_l_msg;
    Xoff_l_msg.state = Xoff_l_;
    Xoff_l_msg.header.stamp = time;
    ow_msgs::CartesianStateStamped Xoff_r_msg;
    Xoff_r_msg.state = Xoff_r_;
    Xoff_r_msg.header.stamp = time;

    ow_msgs::DCMPointSetList dcm_point_set_list_msg;
    for(size_t i = 0; i < dcm_point_set_list_.size(); ++i)
      dcm_point_set_list_msg.point_sets.push_back(dcm_point_set_list_[i]);

    ow_msgs::FootStepList foot_steps_msg;
    for(size_t i = 0; i < foot_steps_.size(); ++i)
      foot_steps_msg.footsteps.push_back(foot_steps_[i]);
    
    ow_msgs::CartesianStateStamped Xref_l_w_msg;
    Xref_l_w_msg.state = Xref_l_w_;
    Xref_l_w_msg.header.stamp = time;
    ow_msgs::CartesianStateStamped Xref_r_w_msg;
    Xref_r_w_msg.state = Xref_r_w_;
    Xref_r_w_msg.header.stamp = time;

    ow_msgs::CartesianStateStamped Xreal_l_w_msg;
    Xreal_l_w_msg.state = Xreal_l_w_;
    Xreal_l_w_msg.header.stamp = time;
    ow_msgs::CartesianStateStamped Xreal_r_w_msg;
    Xreal_r_w_msg.state = Xreal_r_w_;
    Xreal_r_w_msg.header.stamp = time;
    ow_msgs::CartesianStateStamped Xreal_com_w_msg;
    Xreal_com_w_msg.state = Xreal_com_w_;
    Xreal_com_w_msg.header.stamp = time;
    Xreal_l_w_traj_.update(Xreal_l_w_.pos(), time);
    Xreal_r_w_traj_.update(Xreal_r_w_.pos(), time);
    Xreal_com_w_traj_.update(Xreal_com_w_.pos(), time);

    ow_msgs::CartesianStateStamped Xv_l_w_msg;
    Xv_l_w_msg.state = Xv_l_w_;
    Xv_l_w_msg.header.stamp = time;
    ow_msgs::CartesianStateStamped Xv_r_w_msg;
    Xv_r_w_msg.state = Xv_r_w_;
    Xv_r_w_msg.header.stamp = time;
    ow_msgs::CartesianStateStamped Xv_com_w_msg;
    Xv_com_w_msg.state = Xv_com_w_;
    Xv_com_w_msg.header.stamp = time;
    Xv_l_w_traj_.update(Xv_l_w_.pos(), time);
    Xv_r_w_traj_.update(Xv_r_w_.pos(), time);
    Xv_com_w_traj_.update(Xv_com_w_.pos(), time);

    sensor_msgs::JointState joint_tracker_state_msg = joint_tracker_state_;
    std_msgs::Float32 left_ankle_offset_msg;
    left_ankle_offset_msg.data = left_ankle_offset_;
    std_msgs::Float32 left_groin_offset_msg;
    left_groin_offset_msg.data = left_groin_offset_;
    std_msgs::Float32 right_ankle_offset_msg;
    right_ankle_offset_msg.data = right_ankle_offset_;
    std_msgs::Float32 right_groin_offset_msg;
    right_groin_offset_msg.data = right_groin_offset_;

    sensor_msgs::JointState q_msg = q_;

    ow_msgs::LinearStateStamped p_w_msg;
    p_w_msg.state = p_w_;
    p_w_msg.header.stamp = time;
    p_w_traj_.update(p_w_.pos(), time);

    ow::PolygonShape support_polygon(support_polygon_w_);
    ow::PointShape zmp_d_point(p_d_w_.pos().head(2));
    ow::PointShape zmp_point(p_w_.pos().head(2));

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(support_polygon.toMarkerMsg(0.0, 1.0, 1.0, 1.0, 0.03));
    markers.markers.push_back(zmp_d_point.toMarkerMsg(1.0, 0.0, 0.0, 1.0, 0.015));
    markers.markers.push_back(zmp_point.toMarkerMsg(0.0, 1.0, 0.0, 1.0, 0.015));
    for(size_t i = 0; i < markers.markers.size(); ++i)
    {
      markers.markers[i].header.frame_id = "world";
      markers.markers[i].id = i;
    }

    data_mutex_.unlock();

    // publish everything
    ow::publish_if_subscribed(flags_pub_, flags_msg);

    // robot
    if(robot_)
    {
      ow::publish_if_subscribed(f_normal_left_pub_, f_normal_left_msg);
      ow::publish_if_subscribed(f_normal_right_pub_, f_normal_right_msg);
      ow::publish_if_subscribed(ft_left_pub_, ft_left_msg);
      ow::publish_if_subscribed(ft_right_pub_, ft_right_msg);
      ow::publish_if_subscribed(imu_pub_, imu_msg);
      ow::publish_if_subscribed(imu_pose_pub_, imu_pose_msg);
    }

    // balancer
    if(balancer_)
    {
      ow::publish_if_subscribed(Xoff_com_pub_, Xoff_com_msg);
      ow::publish_if_subscribed(p_d_w_pub_, p_d_w_msg);
    }
    
    // com estimator
    if(com_estimator_)
    {
      ow::publish_if_subscribed(Xf_com_w_pub_, Xf_com_w_msg);
      ow::publish_if_subscribed(dcm_w_pub_, dcm_w_msg);
    }

    // com trajectory generator
    if(com_trajectory_generator_)
    {
      ow::publish_if_subscribed(p_r_w_pub_, p_r_w_msg);
      ow::publish_if_subscribed(dcm_r_w_pub_, dcm_r_w_msg);
      ow::publish_if_subscribed(X_r_com_w_pub_, X_r_com_w_msg);
      X_r_com_w_traj_.publishIfSubscribed(time);
    }

    // command generator
    if(command_generator_)
    {
      ow::publish_if_subscribed(Xcmd_l_w_pub_, Xcmd_l_w_msg);
      ow::publish_if_subscribed(Xcmd_r_w_pub_, Xcmd_r_w_msg);
      ow::publish_if_subscribed(Xcmd_com_w_pub_, Xcmd_com_w_msg);
      Xcmd_com_w_traj_.publishIfSubscribed(time);
    }

    // dcm planner
    if(dcm_planner_)
    {
      ow::publish_if_subscribed(dcm_point_set_list_pub_, dcm_point_set_list_msg);
    }

    // foot step planner
    if(step_planner_)
    {
      ow::publish_if_subscribed(foot_steps_pub_, foot_steps_msg);
    }

    // foot trajectory generator
    if(foot_trajectory_generator_)
    {
      ow::publish_if_subscribed(Xref_l_w_pub_, Xref_l_w_msg);
      ow::publish_if_subscribed(Xref_r_w_pub_, Xref_r_w_msg);
    }

    // forward kinematics real
    if(forward_kinematics_)
    {
      ow::publish_if_subscribed(Xreal_l_w_pub_, Xreal_l_w_msg);
      ow::publish_if_subscribed(Xreal_r_w_pub_, Xreal_r_w_msg);
      ow::publish_if_subscribed(Xreal_com_w_pub_, Xreal_com_w_msg);
      Xreal_l_w_traj_.publishIfSubscribed(time);
      Xreal_r_w_traj_.publishIfSubscribed(time);
      Xreal_com_w_traj_.publishIfSubscribed(time);
    }

    // forward kinematics cmd
    if(forward_kinematics_cmd_)
    {
      ow::publish_if_subscribed(Xv_l_w_pub_, Xv_l_w_msg);
      ow::publish_if_subscribed(Xv_r_w_pub_, Xv_r_w_msg);
      ow::publish_if_subscribed(Xv_com_w_pub_, Xv_com_w_msg);
      Xv_l_w_traj_.publishIfSubscribed(time);
      Xv_r_w_traj_.publishIfSubscribed(time);
      Xv_com_w_traj_.publishIfSubscribed(time);
    }

    // foot compliance
    if(foot_compliance_)
    {
      ow::publish_if_subscribed(Xoff_l_pub_, Xoff_l_msg);
      ow::publish_if_subscribed(Xoff_r_pub_, Xoff_r_msg);
    }

    // inverse kinematics
    if(inverse_kinematics_)
    {
      ow::publish_if_subscribed(q_pub_, q_msg); 
    }

    // zmp estimator
    if(zmp_estimator_)
    {
      ow::publish_if_subscribed(p_w_pub_, p_w_msg);
      p_w_traj_.publishIfSubscribed(time);
    }

    // joint tracker
    if(joint_tracker_)
    {
      ow::publish_if_subscribed(joint_tracker_state_pub_,joint_tracker_state_msg);
      ow::publish_if_subscribed(left_ankle_offset_pub_, left_ankle_offset_msg);
      ow::publish_if_subscribed(left_groin_offset_pub_, left_groin_offset_msg);
      ow::publish_if_subscribed(right_ankle_offset_pub_, right_ankle_offset_msg);
      ow::publish_if_subscribed(right_groin_offset_pub_, right_groin_offset_msg);
    }
      
    // world frame based on virtual robot
    tf::StampedTransform Tv_com_w_tf;
    Tv_com_w_tf.setData(Xv_com_w_.pos().toTransformTf());
    Tv_com_w_tf.stamp_ = time;
    Tv_com_w_tf.frame_id_ = frame_w;
    Tv_com_w_tf.child_frame_id_ = frame_b;
    broadcaster.sendTransform(Tv_com_w_tf);

    // robot odometry based on virtual robot (todo add velocity)
    if(odom_pub_ && forward_kinematics_cmd_)
    {
      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = time;
      odom_msg.header.frame_id = frame_w;
      odom_msg.child_frame_id = frame_b;
      odom_msg.pose.pose = Xv_com_w_.pos();
      odom_msg.pose.covariance.fill(0.0);
      odom_msg.twist.covariance.fill(0.0);
      odom_pub_.publish(odom_msg);
    }

    // visualization
    ow::publish_if_subscribed(markers_pub_, markers);
  }

  void StatePublisher::update(
    ow::Flags& flags, 
    const ros::Time& time, 
    const ros::Duration& dt)
  {
    // look for data copying
    const std::lock_guard<std::mutex> lock(data_mutex_);

    flags_ = flags; 

    if(robot_)
    {
      ft_left_ = robot_->forceTorqueLeft();
      ft_right_ = robot_->forceTorqueRight();
      imu_ = robot_->imu();
    }

    if(balancer_)
    {
      Xoff_com_ = balancer_->Xoff_com();
      p_d_w_ = balancer_->ZMPd_w();
      support_polygon_w_ = balancer_->supportPolygon_w();
    }

    if(com_estimator_)
    {
      Xf_com_w_ = com_estimator_->Xf_com_w();
      dcm_w_ = com_estimator_->DCMr_w();
    }

    if(com_trajectory_generator_)
    {
      p_r_w_ = com_trajectory_generator_->ZMP_w();
      dcm_r_w_ = com_trajectory_generator_->DCM_w();
      X_r_com_w_ = com_trajectory_generator_->X_com_w();
    }

    if(command_generator_)
    {
      Xcmd_l_w_ = command_generator_->Xcmd_l_w();
      Xcmd_r_w_ = command_generator_->Xcmd_r_w();
      Xcmd_com_w_ = command_generator_->Xcmd_com_w();
    }

    if(dcm_planner_)
    {
      dcm_point_set_list_ = dcm_planner_->dcmPointSetList();
    }

    if(step_planner_)
    {
      foot_steps_ = step_planner_->footSteps(); 
    }

    if(foot_trajectory_generator_)
    {
      Xref_l_w_ = foot_trajectory_generator_->Xref_l_w();
      Xref_r_w_ = foot_trajectory_generator_->Xref_r_w();
    }

    if(forward_kinematics_)
    {
      Xreal_l_w_ = forward_kinematics_->X_l_w();
      Xreal_r_w_ = forward_kinematics_->X_r_w();
      Xreal_com_w_ = forward_kinematics_->X_com_w();
    }

    if(forward_kinematics_cmd_)
    {
      Xv_l_w_ = forward_kinematics_cmd_->X_l_w();
      Xv_r_w_ = forward_kinematics_cmd_->X_r_w();
      Xv_com_w_ = forward_kinematics_cmd_->X_com_w();
    }

    if(foot_compliance_)
    {
      Xoff_l_ = foot_compliance_->Xoff_l();
      Xoff_r_ = foot_compliance_->Xoff_r();
    }

    if(inverse_kinematics_)
    {
      q_ = inverse_kinematics_->q();
    }

    if(zmp_estimator_)
    {
      p_w_ = zmp_estimator_->ZMP_w();
    }

    if(joint_tracker_)
    {
      joint_tracker_state_ = joint_tracker_->jointState();
      left_ankle_offset_ = joint_tracker_->leftAnkleOffset();
      left_groin_offset_ = joint_tracker_->leftGroinOffset();
      right_ankle_offset_ = joint_tracker_->rightGroinOffset();
      right_groin_offset_ = joint_tracker_->rightGroinOffset();
    } 

    state_updated_ = true;
  }

  void StatePublisher::stop(ow::Flags& flags, const ros::Time& time)
  {
    if(Thread::isRunning())
    {
      stopped_ = true;
    }
  }

}
