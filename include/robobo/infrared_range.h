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
/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jose Capriles. */


#ifndef INFRARED_RANGE_H
#define INFRARED_RANGE_H


#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msgs/Range.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

namespace gazebo
{

class InfraredRange : public RayPlugin
{

    /// \brief Constructor
    public: InfraredRange();

    /// \brief Destructor
    public: ~InfraredRange();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void OnNewLaserScans();

    /// \brief Put range data to the ROS topic
    private: 

        void PutRangeData(common::Time &_updateTime);

        /// \brief Keep track of number of connctions
        int range_connect_count_;
        void RangeConnect();
        void RangeDisconnect();

        // Pointer to the model
        physics::WorldPtr world_;
        /// \brief The parent sensor
        sensors::SensorPtr parent_sensor_;
        sensors::RaySensorPtr parent_ray_sensor_;

        gazebo_ros::Node::SharedPtr node;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;

        /// \brief ros message
        sensor_msgs::Range range_msg_;

        /// \brief topic name
        std::string topic_name_;

        /// \brief frame transform name, should match link name
        std::string frame_name_;

        /// \brief sensor field of view
        double fov_;
        /// \brief Gaussian noise
        double gaussian_noise_;

        /// \brief Gaussian noise generator
        double GaussianKernel(double mu, double sigma);

        /// \brief mutex to lock access to fields that are used in message callbacks
        boost::mutex lock_;

        /// \brief hack to mimic hokuyo intensity cutoff of 100
        double hokuyo_min_intensity_;

        /// update rate of this sensor
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;

        /// \brief for setting ROS name space
        std::string robot_namespace_;

        ros::CallbackQueue range_queue_;
        void RangeQueueThread();
        boost::thread callback_queue_thread_;

        // deferred load in case ros is blocking
        sdf::ElementPtr sdf;
        void LoadThread();
        boost::thread deferred_load_thread_;
        unsigned int seed;
};
}
#endif // INFRARED_RANGE_H
