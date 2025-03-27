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

#ifndef OPEN_WALKER_TRAJECTORY_PUBLISHER_H_
#define OPEN_WALKER_TRAJECTORY_PUBLISHER_H_

#include <ow_core/types.h>
#include <ow_core/common/ros.h>
#include <deque>
#include <nav_msgs/Path.h>

namespace ow_pub
{

/*!
 * \brief The TrajectoryPublisher class
 *
 * This class publishes trajectories in rviz.
 */
class TrajectoryPublisher
{
public:
  enum State {CONSTRUCTED, INITIALIZED, RUNNING};

public:
  State state_;

  std::string name_;
  std::string frame_;

  ros::Duration hist_dur_;
  size_t max_size_;
  ow::Scalar d_theshold_;

  ros::Publisher path_pub_;
  ros::Publisher pose_pub_;

  nav_msgs::Path path_;
  geometry_msgs::PoseStamped pose_;
  
  std::deque<geometry_msgs::PoseStamped> data_;

public:
  /*!
  * \brief TrajectoryPublisher Default constructor.
  */
  TrajectoryPublisher();

  // destructor
  virtual ~TrajectoryPublisher();

  /*
   * \brief advertise the trajectory.
   * 
   * Limit the number of elements in the trajectory to a history_duration of
   * seconds.
   */
  bool advertise(
    ros::NodeHandle& nh, 
    const std::string& name,
    const std::string& frame, 
    const ros::Duration& hist_dur);

  /**
   * \brief advertise the trajectory
   * 
   * Limit the number of elements in the trajectory to max_size
   * elements.
   */
  bool advertise(
    ros::NodeHandle& nh, 
    const std::string& name,
    const std::string& frame, 
    size_t max_size);

  /** 
   * \brief adds a new cartesian pose to the trajectory
   *
   * \param current jointstate
   * \param current time
   */
  void update(
    const ow::CartesianPosition& X,
    const ros::Time& time);

  /** 
   * \brief adds a new cartesian pose to the trajectory
   *
   * \param current jointstate
   * \param current time
   */
  void update(
    const ow::LinearPosition& x,
    const ros::Time& time);
  
  /** 
  * \brief publish the information
  *
  * \param current time
  */
  void publish(const ros::Time& time);

  /** 
  * \brief publish the information
  *
  * \param current time
  */
  void publishIfSubscribed(const ros::Time& time);

private:
  bool checkDistanceTheshold(
    const ow::LinearPosition& x1,
    const geometry_msgs::Point& x2);

};

}

#endif
