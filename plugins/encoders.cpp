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

/* This plugin is used to publish the encoders values of the Gazebo model of Robobo and to create the ResetWheels service to reset wheel position values
/** \author David Casal. */

#ifndef _ENCODERS_HH_
#define _ENCODERS_HH_

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/physics.hh>
#include "gazebo_msgs/msg/link_states.hpp"
#include "std_msgs/msg/int16.hpp"
#include "robobo_msgs/msg/wheels.hpp"
#include "robobo_msgs/srv/reset_wheels.hpp"

namespace gazebo {

    class Encoders : public ModelPlugin
    {

        public: Encoders() {}

        private: 

		  rclcpp::Publisher<robobo_msgs::msg::Wheels>::SharedPtr pubWheels;
            rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pubPan;
            rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pubTilt;
        	robobo_msgs::msg::Wheels msgWheels;
       		std_msgs::msg::Int16 msgPan;
        	std_msgs::msg::Int16 msgTilt;
        	int RWPos;
        	int RWVel;
        	int LWPos;
        	int LWVel;
        	int r_reset = 0;
        	int l_reset = 0;
        	int pan;
       	    int tilt;

		  physics::ModelPtr model;
	
		  gazebo_ros::Node::SharedPtr ros_node_;

		  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr subscription_;

          rclcpp::Service<robobo_msgs::srv::ResetWheels>::SharedPtr srv_;

          rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp1_;

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {	
            // Safety check
            if (_model->GetJointCount() == 0)
            {
                std::cerr << "Invalid joint count, model not loaded\n";
                return;
            }

            this->model = _model;

            this->ros_node_ = gazebo_ros::Node::Get(_sdf);
	
            // Create topics to publish
            this->pubWheels = this->ros_node_->create_publisher<robobo_msgs::msg::Wheels>("/" + this->model->GetName() + "/wheels", 1);
            this->pubPan = this->ros_node_->create_publisher<std_msgs::msg::Int16>("/" + this->model->GetName() + "/pan", 1);
            this->pubTilt = this->ros_node_->create_publisher<std_msgs::msg::Int16>("/" + this->model->GetName() + "/tilt", 1);

            subscription_ = ros_node_->create_subscription<gazebo_msgs::msg::LinkStates>("/gazebo/link_states",1, 
						std::bind(&Encoders::Callback, this, std::placeholders::_1));

            cb_grp1_ = this->ros_node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

             // Create ResetWheels service
            srv_ = ros_node_->create_service<robobo_msgs::srv::ResetWheels>("/" + this->model->GetName() + "/reset_wheels",
                   std::bind(&Encoders::CallbackResetWheels, this, std::placeholders::_1, std::placeholders::_2), 
                   rmw_qos_profile_services_default,cb_grp1_);
        }

        public: void Callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
        {   
            // Read and transform values to degrees of right wheel joint
            RWPos = int (round(this->model->GetJoint("right_motor")->Position()) - this->r_reset);
            RWVel = int (round(this->model->GetJoint("right_motor")->GetVelocity(0) * 180 / M_PI));

            // Read and transform values to degrees of left wheel joint
            LWPos = int (round(this->model->GetJoint("left_motor")->Position()) - this->l_reset);
            LWVel = int (round(this->model->GetJoint("left_motor")->GetVelocity(0) * 180 / M_PI));

            // Save data in Wheels msg
            this->msgWheels.wheelposr.data = RWPos;
            this->msgWheels.wheelspeedr.data = RWVel;
            this->msgWheels.wheelposl.data = LWPos;
            this->msgWheels.wheelspeedl.data = LWVel;

            // Publish msg in topic
            this->pubWheels->publish(this->msgWheels);

            // Read position values of pan and tilt
            pan = int (round(this->model->GetJoint("pan_motor")->Position()));
            tilt = int (round(this->model->GetJoint("tilt_motor")->Position()));

            // Save data in msg
            this->msgPan.data = pan;
            this->msgTilt.data = tilt;

            // Publish msg in topic
            this->pubPan->publish(this->msgPan);
            this->pubTilt->publish(this->msgTilt);
        }

        public: void CallbackResetWheels(const std::shared_ptr<robobo_msgs::srv::ResetWheels::Request> req, 
                                        const std::shared_ptr<robobo_msgs::srv::ResetWheels::Response> res)
        { 
	           // asumo que son grados, comprobar que este bien
            this->r_reset = round(this->model->GetJoint("right_motor")->Position());
            this->l_reset = round(this->model->GetJoint("left_motor")->Position());
        }


    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(Encoders)
}
#endif
