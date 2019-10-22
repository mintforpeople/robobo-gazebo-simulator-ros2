#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import rclpy

from std_msgs.msg import Int32, Int16, Int8
from robobo_msgs.msg import IRs
from robobo_msgs.srv import MoveWheels, MovePanTilt

def moveWheels(node, rspeed, lspeed, time):
    
    client = node.create_client(MoveWheels, "/robobo/moveWheels")
    request = MoveWheels.Request()
    request.lspeed = lspeed
    request.rspeed = rspeed
    request.time = time*1000
    request.unlockid = 0
   
    node.get_logger().info("Sending service request to `/robobo/moveWheels`")
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
    client = node.create_client(MovePanTilt, "/robobo/movePanTilt")
    request = MovePanTilt.Request
    request.panpos = panpos
    request.panspeed == panspeed
    request.panunlockid = 0
    request.tiltpos = tiltpos
    request.tiltspeed = tiltspeed
    request.tiltunlockid = 0
       
    node.get_logger().info("Sending service request to `/robobo/movePanTilt`")
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
    EX_TIME = 30

    movePanTilt(node, 180, 15, 85, 15)

    end_time = node.get_clock().now() + rclpy.time.Duration(seconds=EX_TIME)

    # 1 segundo pienso
    moveWheels(node, SPEED, SPEED, 1)

    #while node.get_clock().now() < end_time:
        #exploreIR(SPEED, MAX_IR)
    #    moveWheels(node, SPEED, SPEED, 100)

    #moveWheels(node, 0, 0, 1)

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()