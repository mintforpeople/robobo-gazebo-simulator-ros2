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

/* This plugin is used to create the MoveWheels service to move wheels motors in the Gazebo model of Robobo */

#ifndef _MOVE_WHEELS_HH_
#define _MOVE_WHEELS_HH_

#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/physics.hh>

#include <robobo_msgs/srv/move_wheels.hpp>

namespace gazebo
{

    class MoveWheels: public ModelPlugin
    {
        public: MoveWheels() {}

 	        gazebo_ros::Node::SharedPtr ros_node_;

        private: 

            physics::ModelPtr model;
            std::thread MoveWheelsThread;
            rclcpp::Service<robobo_msgs::srv::MoveWheels>::SharedPtr srv_;

            int lwp;
            int rwp;
            float time;

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;
            this->ros_node_ = gazebo_ros::Node::Get(_sdf);

            // Safety check
            if (_model->GetJointCount() == 0)
            {   
                RCLCPP_ERROR(this->ros_node_->get_logger(), "Invalid joint count, model not loaded");
                return;
            }

            // Create MoveWheels service
            this->srv_ = this->ros_node_->create_service<robobo_msgs::srv::MoveWheels>
                    ("/" + this->model->GetName() + "/move_wheels",
                    std::bind(&MoveWheels::Callback, this, std::placeholders::_1, std::placeholders::_2));
        }

        public: bool Callback(const std::shared_ptr<robobo_msgs::srv::MoveWheels::Request> req,
                            const std::shared_ptr<robobo_msgs::srv::MoveWheels::Response> res)
        {
            // Save service parameter values
            if (req->lspeed.data < -100){

                this->lwp = -100;
            }
            else if (req->rspeed.data > 100)
            {
                this->lwp = 100;
            }
            else
            {
                this->lwp = req->lspeed.data;
            }
            if (req->rspeed.data < -100)
            {
                this->rwp = -100;
            }
            else if (req->rspeed.data > 100)
            {
                this->rwp = 100;
            }
            else
            {
                this->rwp = req->rspeed.data;
            }
            this->time = req->time.data / 1000;
            this->MoveWheelsThread = std::thread(std::bind (&MoveWheels::Handle_MoveWheels, this));
            this->MoveWheelsThread.detach();
            return true;
        }

        private: void Handle_MoveWheels ()
        {
            double lspeedGazebo;
            double rspeedGazebo;

            // Use enough force to achieve the target velocity
            this->model->GetJoint("right_motor")->SetParam("fmax", 0, 0.5);
            this->model->GetJoint("left_motor")->SetParam("fmax", 0, 0.5);

            // Calculate joints target velocity
            lspeedGazebo = ((-4.625E-05 * pow(abs(this->lwp),3) + 5.219E-03 * pow(abs(this->lwp),2) + 6.357 * abs(this->lwp) + 5.137E+01) + (
                     -3.253E-04 * pow(abs(this->lwp),3) + 4.285E-02 * pow(abs(this->lwp),2) + -2.064 * abs(this->lwp) - 1.770E+01) / this->time) * M_PI / 180;
            if (this->lwp < 0)
            {
                lspeedGazebo = -lspeedGazebo;
            }
            else if (this->lwp == 0)
            {
                lspeedGazebo = 0;
            }
            rspeedGazebo = ((-4.625E-05 * pow(abs(this->rwp),3) + 5.219E-03 * pow(abs(this->rwp),2) +
                    6.357 * abs(this->rwp) + 5.137E+01) + (-3.253E-04 * pow(abs(this->rwp),3) + 4.285E-02 *
                        pow(abs(this->rwp),2) + -2.064 * abs(this->rwp) - 1.770E+01) / this->time) * M_PI / 180;
            if (this->rwp < 0)
            {
                rspeedGazebo = - rspeedGazebo;
            }
            else if (this->rwp == 0 )
            {
                rspeedGazebo = 0;
            }

            // Set joints velocity
            this->model->GetJoint("left_motor")->SetParam("vel", 0, lspeedGazebo);
            this->model->GetJoint("right_motor")->SetParam("vel", 0, rspeedGazebo);

            usleep(this->time * 1E6);
            this->model->GetJoint("left_motor")->SetParam("vel", 0, double(0));
            this->model->GetJoint("right_motor")->SetParam("vel", 0, double(0));
        }



    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(MoveWheels)
}
#endif
