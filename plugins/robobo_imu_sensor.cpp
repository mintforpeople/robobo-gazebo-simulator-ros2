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


/* Derived from the work :
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

 /*
*Modified by David Casal
*2019
*/

#include <iostream>

#include "robobo/robobo_imu_sensor.h"


GZ_REGISTER_SENSOR_PLUGIN(gazebo::RoboboImuSensor)

gazebo::RoboboImuSensor::RoboboImuSensor(): SensorPlugin()
{
  accelerometer_data = ignition::math::Vector3d(0, 0, 0);
  gyroscope_data = ignition::math::Vector3d(0, 0, 0);
  orientation = ignition::math::Quaterniond(1,0,0,0);
  seed=0;
  sensor=NULL;
}

void gazebo::RoboboImuSensor::Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf=sdf_;
  sensor=dynamic_cast<gazebo::sensors::ImuSensor*>(sensor_.get());

  if(sensor==NULL)
  {
    RCLCPP_FATAL(node->get_logger(),"Error: Sensor pointer is NULL!");
    return;
  }

  sensor->SetActive(true);

  if(!LoadParameters())
  {
    RCLCPP_FATAL(node->get_logger(),"Error Loading Parameters!");
    return;
  }

  node = gazebo_ros::Node::Get(sdf_);

  accel_data_publisher = node->create_publisher<geometry_msgs::msg::Accel>(robot_namespace + "accel",1);
  orient_data_publisher = node->create_publisher<geometry_msgs::msg::Quaternion>(robot_namespace + "orientation",1);

  connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RoboboImuSensor::UpdateChild, this, std::placeholders::_1));

  last_time = sensor->LastUpdateTime();
}

void gazebo::RoboboImuSensor::UpdateChild(const gazebo::common::UpdateInfo &/*_info*/)
{ 
  common::Time current_time = sensor->LastUpdateTime();

  if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) //update rate check
    return;

//  if(imu_data_publisher.getNumSubscribers() > 0)
//  {
    orientation = offset.Rot()*sensor->Orientation(); //applying offsets to the orientation measurement
    accelerometer_data = sensor->LinearAcceleration();
    gyroscope_data = sensor->AngularVelocity();

    //Guassian noise is applied to all measurements
    orient_msg.x = orientation.X() + GuassianKernel(0,gaussian_noise);
    orient_msg.y = orientation.Y() + GuassianKernel(0,gaussian_noise);
    orient_msg.z = orientation.Z() + GuassianKernel(0,gaussian_noise);
    orient_msg.w = orientation.W() + GuassianKernel(0,gaussian_noise);

    accel_msg.linear.x = accelerometer_data.X() + GuassianKernel(0,gaussian_noise);
    accel_msg.linear.y = accelerometer_data.Y() + GuassianKernel(0,gaussian_noise);
    accel_msg.linear.z = accelerometer_data.Z() + GuassianKernel(0,gaussian_noise);

    // In the real Robobo only linear acceleration is provided
//    accel_msg.angular.x = gyroscope_data.X() + GuassianKernel(0,gaussian_noise);
//    accel_msg.angular.y = gyroscope_data.Y() + GuassianKernel(0,gaussian_noise);
//    accel_msg.angular.z = gyroscope_data.Z() + GuassianKernel(0,gaussian_noise);

    //covariance is related to the Gaussian noise
//    double gn2 = gaussian_noise*gaussian_noise;
//    imu_msg.orientation_covariance[0] = gn2;
//    imu_msg.orientation_covariance[4] = gn2;
//    imu_msg.orientation_covariance[8] = gn2;
//    imu_msg.angular_velocity_covariance[0] = gn2;
//    imu_msg.angular_velocity_covariance[4] = gn2;
//    imu_msg.angular_velocity_covariance[8] = gn2;
//    imu_msg.linear_acceleration_covariance[0] = gn2;
//    imu_msg.linear_acceleration_covariance[4] = gn2;
//    imu_msg.linear_acceleration_covariance[8] = gn2;

    //preparing message header
//    imu_msg.header.frame_id = body_name;
//    imu_msg.header.stamp.sec = current_time.sec;
//    imu_msg.header.stamp.nsec = current_time.nsec;

    //publishing data
    accel_data_publisher->publish(accel_msg);
    orient_data_publisher->publish(orient_msg);

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);
    executor.spin_once();


//  }

  last_time = current_time;
}

double gazebo::RoboboImuSensor::GuassianKernel(double mu, double sigma)
{
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool gazebo::RoboboImuSensor::LoadParameters()
{
  //loading parameters from the sdf file

  //NAMESPACE
    std::string scoped_name = sensor->ParentName();
    std::size_t it = scoped_name.find(":");
    robot_namespace = "/" +scoped_name.substr(0,it)+"/";

//  //TOPIC
//  if (sdf->HasElement("topicName"))
//  {
//    topic_name =  robot_namespace + sdf->Get<std::string>("topicName");
//    ROS_INFO_STREAM("<topicName> set to: "<<topic_name);
//  }
//  else
//  {
//    topic_name = robot_namespace + "/imu_data";
//    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
//  }
//
//  //BODY NAME
//  if (sdf->HasElement("frameName"))
//  {
//    body_name =  sdf->Get<std::string>("frameName");
//    ROS_INFO_STREAM("<frameName> set to: "<<body_name);
//  }
//  else
//  {
//    ROS_FATAL("missing <frameName>, cannot proceed");
//    return false;
//  }
  
  //UPDATE RATE
  if (sdf->HasElement("updateRateHZ"))
  {
    update_rate =  sdf->Get<double>("updateRateHZ");
    RCLCPP_INFO(node->get_logger(), "<updateRateHZ> set to: '%f'", update_rate);
  }
  else
  {
    update_rate = 1.0;
    RCLCPP_WARN(node->get_logger(), "missing <updateRateHZ>, set to default: '%f'", update_rate);
  }

  //NOISE
  if (sdf->HasElement("gaussianNoise"))
  {
    gaussian_noise =  sdf->Get<double>("gaussianNoise");
    RCLCPP_INFO(node->get_logger(), "<gaussianNoise> set to: '%f'", gaussian_noise);
  }
  else
  {
    gaussian_noise = 0.0;
    RCLCPP_WARN(node->get_logger(), "missing <gaussianNoise>, set to default: '%f'", gaussian_noise);
  }

  //POSITION OFFSET, UNUSED
  if (sdf->HasElement("xyzOffset"))
  {
    offset.Pos() =  sdf->Get<ignition::math::Vector3d>("xyzOffset");
    RCLCPP_INFO(node->get_logger(), "<xyzOffset> set to: '%f' '%f' '%f'", offset.Pos()[0], offset.Pos()[1], offset.Pos()[2]);
  }
  else
  {
    offset.Pos() = ignition::math::Vector3d(0, 0, 0);
    RCLCPP_WARN(node->get_logger(), "missing <xyzOffset>, set to default: '%f' '%f' '%f'", offset.Pos()[0], offset.Pos()[1], offset.Pos()[2]);
  }

  //ORIENTATION OFFSET
  if (sdf->HasElement("rpyOffset"))
  {
    offset.Rot() = ignition::math::Quaterniond(sdf->Get<ignition::math::Vector3d>("rpyOffset"));
    RCLCPP_INFO(node->get_logger(), "<rpyOffset> set to: '%f' '%f' '%f'", 
      offset.Rot().Roll(), offset.Rot().Pitch(), offset.Rot().Yaw());
  }
  else
  {
    offset.Rot() = ignition::math::Quaterniond::Identity;
    RCLCPP_WARN(node->get_logger(), "missing <rpyOffset>, set to default: '%f' '%f' '%f'",
      offset.Rot().Roll(), offset.Rot().Pitch(), offset.Rot().Yaw());
  }
  return true;
}

gazebo::RoboboImuSensor::~RoboboImuSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  rclcpp::shutdown();
}