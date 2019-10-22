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

/* This plugin is used to create the MovePanTilt service to move pan and tilt motors in the Gazebo model of Robobo */

#ifndef _MOVE_PAN_TILT_HH_
#define _MOVE_PAN_TILT_HH_

#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/physics.hh>

#include <robobo_msgs/srv/move_pan_tilt.hpp>


namespace gazebo
{

    class MovePanTilt : public ModelPlugin
    {

        public: MovePanTilt() {}

            /// \brief A node use for ROS transport
            gazebo_ros::Node::SharedPtr ros_node_;
            
        private:
            /// \brief Pointer to the model.
            physics::ModelPtr model;

            /// \brief A thread the keeps running position setting
            std::thread MovePanTiltThread;

            rclcpp::Service<robobo_msgs::srv::MovePanTilt>::SharedPtr srv_;

            // MovePanTilt parameters
            int panPos;
            int pp;
            int tiltPos;
            int tp;


        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Safety check
            if (_model->GetJointCount() == 0)
            {
                std::cerr << "Invalid joint count, model not loaded\n";
                return;
            }

            this->model = _model;

            ros_node_ = gazebo_ros::Node::Get(_sdf);

            /           
            // Create MovePanTilt service
            this->srv_ = ros_node_->create_service<robobo_msgs::srv::MovePanTilt>
                    ("/" + this->model->GetName() + "/move_pan_tilt", 
                    std::bind(&MovePanTilt::Callback, this, std::placeholders::_1, std::placeholders::_2));

            // Set pan and tilt initial positions
            this->model->GetJoint("tilt_motor")->SetPosition(0, 1.22);
            this->model->GetJoint("tilt_motor")->SetLowStop(0, 1.22);
            this->model->GetJoint("tilt_motor")->SetHighStop(0, 1.22);
            this->model->GetJoint("pan_motor")->SetPosition(0, M_PI);
            this->model->GetJoint("pan_motor")->SetLowStop(0, M_PI);
            this->model->GetJoint("pan_motor")->SetHighStop(0, M_PI);
        }

        public: bool Callback(const std::shared_ptr<robobo_msgs::srv::MovePanTilt::Request> req, 
                             const std::shared_ptr<robobo_msgs::srv::MovePanTilt::Response> res)
        {
            this->panPos = req.panPos.data;
            this->tiltPos = req.tiltPos.data;
            if (req.panSpeed.data > 100)
            {
                this->pp = 100;
            }
            else if (req.panSpeed.data < 0)
            {
                // If the velocity command is negative, Pan won't move
                this->panPos = 0;
            }
            else
            {
                this->pp = req.panSpeed.data;
            }
            if (req.tiltSpeed.data > 100)
            {
                this->tp = 100;
            }
            else if (req.tiltSpeed.data < 0)
            {
                // If the velocity command is negative, Tilt won't move
                this->tiltPos = 0;
            }
            else
            {
                this->tp = req.tiltSpeed.data;
            }
            this->MovePanTiltThread = std::thread(std::bind (&MovePanTilt::Handle_MoveWheels, this));
            this->MovePanTiltThread.join();
            return true;
        }

        private: void Handle_MoveWheels ()
        {   
            double pspeedGazebo;
            double tspeedGazebo;

            // Use no more than 0.15 Nm to achieve pan target velocity
            this->model->GetJoint("pan_motor")->SetParam("fmax", 0, 0.15);
            // Use no more than 0.22 Nm to achieve tilt target velocity
            this->model->GetJoint("tilt_motor")->SetParam("fmax", 0, 0.22);

            //Set pan position
            if (10 < this->panPos & this->panPos < 345 & this->panPos != round(this->model->GetJoint("pan_motor")->Position()))
            {
                // Calculate joint target velocity
                int difPan = this->panPos - this->model->GetJoint("pan_motor")->Position();
                pspeedGazebo = (abs(difPan) * (-3.06E-05 * pow(this->pp,3) + 3.877E-03 * pow(this->pp,2) + 0.84747 * this->pp + 8.05468) /
                    (abs(difPan) - (-1.99E-05 * pow(this->pp,3) + 1.064E-03 * pow(this->pp,2) - 0.33034 * this->pp - 8.9074E-01)) ) * M_PI / 180;

                // Unlock joint position and set target velocity
                this->model->GetJoint("pan_motor")->SetLowStop(0, 0.191);
                this->model->GetJoint("pan_motor")->SetHighStop(0, 6.004);
                if (this->model->GetJoint("pan_motor")->Position() < this->panPos)
                {
                    while (this->model->GetJoint("pan_motor")->Position() < this->panPos)
                    {
                        this->model->GetJoint("pan_motor")->SetParam("vel", 0, pspeedGazebo);
                    }
                }
                else
                {
                    while (this->model->GetJoint("pan_motor")->Position() > this->panPos)
                    {
                        this->model->GetJoint("pan_motor")->SetParam("vel", 0, -pspeedGazebo);
                    }
                }
                // Lock joint position
                this->model->GetJoint("pan_motor")->SetParam("vel", 0, 0);
                this->model->GetJoint("pan_motor")->SetLowStop(0, this->model->GetJoint("pan_motor")->Position());
                this->model->GetJoint("pan_motor")->SetHighStop(0, this->model->GetJoint("pan_motor")->Position());
            }

            // Set tilt position
            if (4 < this->tiltPos & this->tiltPos < 111 & this->tiltPos != round(this->model->GetJoint("tilt_motor")->Position()))
            {
                // Calculate joint target velocity
                int diftilt = this->tiltPos - this->model->GetJoint("tilt_motor")->Position();
                pspeedGazebo = (abs(diftilt) * (1.409E-05 * pow(this->tp,3) - 2.598E-03 * pow(this->tp,2) + 0.4809 * this->tp + 3.182) /
                    (abs(diftilt) - ( -8.84E-05 * pow(this->tp,3) + 1.233E-02 * pow(this->tp,2) + -0.5495 * this->tp + 4.738)) ) * M_PI / 180;

                // Unlock joint position and set target velocity
                this->model->GetJoint("tilt_motor")->SetLowStop(0, 0.0873);
                this->model->GetJoint("tilt_motor")->SetHighStop(0, 1.9199);
                if (this->model->GetJoint("tilt_motor")->Position() < this->tiltPos)
                {
                    while (this->model->GetJoint("tilt_motor")->Position() < this->tiltPos)
                    {
                        this->model->GetJoint("tilt_motor")->SetParam("vel", 0, pspeedGazebo);
                    }
                }
                else
                {
                    while (this->model->GetJoint("tilt_motor")->Position() > this->tiltPos)
                    {
                        // Unlock joint position and set target velocity
                        this->model->GetJoint("tilt_motor")->SetParam("vel", 0, -pspeedGazebo);
                    }
                }
                // Lock joint position
                this->model->GetJoint("tilt_motor")->SetParam("vel", 0, 0);
                this->model->GetJoint("tilt_motor")->SetLowStop(0, this->model->GetJoint("tilt_motor")->Position());
                this->model->GetJoint("tilt_motor")->SetHighStop(0, this->model->GetJoint("tilt_motor")->Position());
            }
        }


    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(MovePanTilt)
}
#endif
