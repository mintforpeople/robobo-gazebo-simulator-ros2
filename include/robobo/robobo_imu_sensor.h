/*******************************************************************************
 *
 *   Copyright 2019, Manufactura de Ingenios Tecnológicos S.L. 
 *   <http://www.mintforpeople.com>
 *
 *   Redistribution, modification and use of this software are permitted under
 *   terms of the Apache 2.0 License.
 *
 *   This software is distributed in the hope that it will be useful,
 *   but WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND; without even the implied
 *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   Apache 2.0 License for more details.
 *
 *   You should have received a copy of the Apache 2.0 License along with    
 *   this software. If not, see <http://www.apache.org/licenses/>.
 *
 ******************************************************************************/

/* Derived from the work: */
/* Copyright [2015] [Alessandro Settimi]
 * 
 * email: ale.settimi@gmail.com
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/


#ifndef ROBOBO_IMU_SENSOR_H
#define ROBOBO_IMU_SENSOR_H

#include <string>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/physics/World.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>


#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/quaternion.hpp>


namespace gazebo
{
  namespace sensors
  {
    class ImuSensor;
  }
  /**
  @anchor GazeboRosImuSensor
  \ref GazeboRosImuSensor is a plugin to simulate an Inertial Motion Unit sensor, the main differences from \ref GazeboRosIMU are:
  - inheritance from SensorPlugin instead of ModelPlugin,
  - measurements are given by gazebo ImuSensor instead of being computed by the ros plugin,
  - gravity is included in inertial measurements.
  */
  /** @brief Gazebo Ros imu sensor plugin. */
  class RoboboImuSensor : public SensorPlugin
  {
  public:
    /// \brief Constructor.
    RoboboImuSensor();
    /// \brief Destructor.
    virtual ~RoboboImuSensor();
    /// \brief Load the sensor.
    /// \param sensor_ pointer to the sensor.
    /// \param sdf_ pointer to the sdf config file.
    virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

  protected:
    /// \brief Update the sensor.
    virtual void UpdateChild(const gazebo::common::UpdateInfo &/*_info*/);

  private:
    /// \brief Load the parameters from the sdf file.
    bool LoadParameters();
    /// \brief Gaussian noise generator.
    /// \param mu offset value.
    /// \param sigma scaling value.
    double GuassianKernel(double mu, double sigma);
    
    /// \brief Ros node pointer.
    gazebo_ros::Node::SharedPtr node;
    /// \brief Ros Publisher for acceleration data.
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_data_publisher;
    /// \brief Ros Publisher for orientation data.
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orient_data_publisher;
    /// \brief Ros acceleration message.
    geometry_msgs::msg::Accel accel_msg;
    // \brief Ros orientation message.
    geometry_msgs::msg::Quaternion orient_msg;

    /// \brief last time on which the data was published.
    common::Time last_time;
    /// \brief Pointer to the update event connection.
    gazebo::event::ConnectionPtr connection;
    /// \brief Pointer to the sensor.
    sensors::ImuSensor* sensor;
    /// \brief Pointer to the sdf config file.
    sdf::ElementPtr sdf;
    /// \brief Orientation data from the sensor.
    ignition::math::Quaterniond orientation;
    /// \brief Linear acceleration data from the sensor.
    ignition::math::Vector3d accelerometer_data;
    /// \brief Angular velocity data from the sensor.
    ignition::math::Vector3d gyroscope_data;
    
    /// \brief Seed for the Gaussian noise generator.
    unsigned int seed;

    //loaded parameters
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
    std::string robot_namespace;
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
//    std::string topic_name;
    /// \brief Name of the link of the IMU.
//    std::string body_name;
    /// \brief Sensor update rate.
    double update_rate;
    /// \brief Gaussian noise.
    double gaussian_noise;
    /// \brief Offset parameter, position part is unused.
    ignition::math::Pose3d offset;
  };
}

#endif //ROBOBO_IMU_SENSOR_H
