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

import time
import rclpy

from std_msgs.msg import Int32, Int16, Int8
from robobo_msgs.msg import IRs
from robobo_msgs.srv import MoveWheels, MovePanTilt

def moveWheels(node, rspeed, lspeed, time):
    
    client = node.create_client(MoveWheels, "/robobo/move_wheels")
    request = MoveWheels.Request()
    request.lspeed = lspeed
    request.rspeed = rspeed
    request.time = time*1000
    request.unlockid = 0
   
    node.get_logger().info("Sending service request to `/robobo/move_wheels`")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = client.call_async(request)

    while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            if future.result() is not None:
                print('response: %r' % future.result())
            else:
                node.get_logger().info('Service call failed %r' % (future.exception(),))
            break

def movePanTilt(node, panpos, panspeed, tiltpos, tiltspeed):
    client = node.create_client(MovePanTilt, "/robobo/move_pan_tilt")
    request = MovePanTilt.Request()
    request.panpos = Int16()
    request.panpos.data = panpos
    request.panspeed = Int8()
    request.panspeed.data = panspeed
    request.panunlockid = Int16()
    request.panunlockid.data = 0
    request.tiltpos = Int16()
    request.tiltpos.data = tiltpos
    request.tiltspeed = Int8()
    request.tiltspeed.data = tiltspeed
    request.tiltunlockid = Int16()
    request.tiltunlockid.data = 0
       
    node.get_logger().info("Sending service request to `/robobo/move_pan_tilt`")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = client.call_async(request)

    while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            if future.result() is not None:
                print('response: %r' % future.result())
            else:
                node.get_logger().info('Service call failed %r' % (future.exception(),))
            break


def main(args=None):

    # The real Robobo infrared sensors do not publish before a change so it is necessary to initialize the variables
    FrontC = 0
    FrontLL = 0
    FrontRR = 0
    BackC = 0
    BackL = 0
    BackR = 0
        
    rclpy.init(args=args)
    node = rclpy.create_node('robobo_validation')
        
    SPEED = 15
    MAX_IR = 250
    # Time while Robobo will be exploring
    exploring_time = 30

    movePanTilt(node, 180, 15, 85, 15)

    end_time = node.get_clock().now() + rclpy.time.Duration(seconds=exploring_time)

    moveWheels(node, SPEED, SPEED, 5)

    #while node.get_clock().now() < end_time:
        #exploreIR(SPEED, MAX_IR)
        #moveWheels(node, SPEED, SPEED, 100)

    #moveWheels(node, 0, 0, 1)

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()