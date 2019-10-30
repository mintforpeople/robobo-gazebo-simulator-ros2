#!/usr/bin/python3
# -*- coding: utf-8 -*-
#/*******************************************************************************
# *
# *   Copyright 2019, Manufactura de Ingenios Tecnológicos S.L. 
# *   <http://www.mintforpeople.com>
# *
# *   Redistribution, modification and use of this software are permitted under
# *   terms of the Apache 2.0 License.
# *
# *   This software is distributed in the hope that it will be useful,
# *   but WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND; without even the implied
# *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# *   Apache 2.0 License for more details.
# *
# *   You should have received a copy of the Apache 2.0 License along with    
# *   this software. If not, see <http://www.apache.org/licenses/>.
# *
# ******************************************************************************/

"""
spawn_robobo.py

Script used to spawn a robobo in a generic position
"""
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

def main():
    """ Main for spwaning robobo node """
    # Get input arguments from user
    argv = sys.argv[1:]

    # Start node
    rclpy.init()
    
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the robobo model
    sdf_file_path = os.path.join(get_package_share_directory("robobo_gazebo"), "models", "robobo", "model.sdf")
    
    # Set data for request
    request = SpawnEntity.Request()
    request.name = argv[0]
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()