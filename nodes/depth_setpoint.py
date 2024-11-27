#!/usr/bin/env python3
"""
This node computes a square-wave setpoint for the depth controller, i.e.
the setpoint jumps between two different depth values with a set duration.
You can change this code to try out other setpoint functions, e.g. a sin wave.
"""

import rclpy
from hippo_msgs.msg import Float64Stamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import math

class DepthSetpointNode(Node):
    def __init__(self):
        super().__init__(node_name='depth_setpoint_publisher')

        self.start_time = self.get_clock().now()

        # change these parameters to adjust the setpoint
        self.setpoint_1 = -0.4  # in m
        self.setpoint_2 = -0.6  # in m
        self.duration = 20.0    # in seconds
        self.function = 0       # 0 for sin, 1 for triangular, 2 for square, 3 for sawtooth, 4 for static setpoint

        self.depth_setpoint_pub = self.create_publisher(
            msg_type=Float64Stamped,
            topic='depth_setpoint',
            qos_profile=1,
        )
        self.timer = self.create_timer(
            timer_period_sec=1 / 50,
            callback=self.on_timer,
        )

    def on_timer(self) -> None:
        # change this for other setpoint functions
        now = self.get_clock().now()
        time = self.start_time - now
        i = time.nanoseconds * 1e-9 % (self.duration * 2)
        
        if (self.function == 0):                                                #Sinus wave
            offset = (self.setpoint_1 + self.setpoint_2) / 2                       
            amplitude = self.setpoint_1 - offset
            setpoint = amplitude * math.sin(i/(self.duration * 2) * 2 * math.pi) + offset
        
        elif (self.function == 1):                                              #Triangular wave
            if i > (self.duration):                                            
                setpoint = self.setpoint_1 - (2 - i/(self.duration)) * (self.setpoint_1 - self.setpoint_2)
            else:
                setpoint = self.setpoint_2 + (1 - i/self.duration) * (self.setpoint_1 - self.setpoint_2)
        
        elif (self.function == 2):                                              #Square wave
            if i > (self.duration):                                           
                setpoint = self.setpoint_1
            else:
                setpoint = self.setpoint_2
    
        elif (self.function == 3):                                              #Sawtooth wave
            setpoint = self.setpoint_2 - (i/(2*self.duration) -1) * (self.setpoint_1 - self.setpoint_2) 
        
        elif (self.function == 4):                                              #Static
            setpoint = (self.setpoint_1 + self.setpoint_2) / 2   
        
        else:                                                                   #Invalid Input
            self.get_logger().info(
            "Invalid input, no according function found. Aborting.",
            throttle_duration_sec=1,
            )
            pass
            return -0.1 
        now = self.get_clock().now()
        self.publish_setpoint(setpoint=setpoint, now=now)
        return setpoint

    def publish_setpoint(self, setpoint: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = setpoint
        msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_setpoint_pub.publish(msg)


def main():
    rclpy.init()
    node = DepthSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
