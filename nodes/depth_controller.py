#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""

import rclpy
from hippo_control_msgs.msg import ActuatorSetpoint
from hippo_msgs.msg import DepthStamped, Float64Stamped
from rclpy.node import Node

import time

import rclpy.time


class DepthControlNode(Node):
    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.current_setpoint = 0.0
        self.current_depth = 0.0

        self.past = None
        self.dt = None

        self.prior_error = 0.0
        self.error = None
        self.integral = 0.0
        self.diff = None

        self.k_d = 1
        self.k_i = 3
        self.k_p = 3
        self.bias = 0.0

        self.windup_limit = 0.05

        self.thrust_pub = self.create_publisher(
            msg_type=ActuatorSetpoint, topic='thrust_setpoint', qos_profile=1
        )

        self.setpoint_sub = self.create_subscription(
            msg_type=Float64Stamped,
            topic='depth_setpoint',
            callback=self.on_setpoint,
            qos_profile=1,
        )
        self.depth_sub = self.create_subscription(
            msg_type=DepthStamped,
            topic='depth',
            callback=self.on_depth,
            qos_profile=1,
        )

        # PID-related publishers

        self.setpoint_error_pub = self.create_publisher(
            msg_type=Float64Stamped, topic='setpoint_error', qos_profile=1
        )
        self.p_gain_pub = self.create_publisher(
            msg_type=Float64Stamped, topic='p__gain', qos_profile=1
        )
        self.i_gain_pub = self.create_publisher(
            msg_type=Float64Stamped, topic='i_gain', qos_profile=1
        )
        self.d_gain_pub = self.create_publisher(
            msg_type=Float64Stamped, topic='d_gain', qos_profile=1
        )

    #PID-related functions to message the publishers

    def publish_setpointerror_msg(self, setpoint_error: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = setpoint_error
        msg.header.stamp = now.to_msg()
        self.setpoint_error_pub.publish(msg)

    def publish_pgain_msg(self, p_gain: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = p_gain
        msg.header.stamp = now.to_msg()
        self.p_gain_pub.publish(msg)

    def publish_igain_msg(self, i_gain: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = i_gain
        msg.header.stamp = now.to_msg()
        self.i_gain_pub.publish(msg)

    def publish_dgain_msg(self, d_gain: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = d_gain
        msg.header.stamp = now.to_msg()
        self.d_gain_pub.publish(msg)

    # Subsribers to setpoint and depth from the other Nodes

    def on_setpoint(self, setpoint_msg: Float64Stamped):
        
        self.current_setpoint = setpoint_msg.data

        # Safety check for depth setpoint
        if self.current_setpoint < -0.8: 
            if self.current_setpoint > -0.1:
             self.get_logger().warn(f"Setpoint {self.current_setpoint} out of safe range! Sending null thrust")
             # Send null thrust to actuators
             self.publish_vertical_thrust(thrust = 0.0, timestamp = self.get_clock().now())
             return 

    def on_depth(self, depth_msg: DepthStamped):
        # We received a new depth message! Now we can get to action!
        current_depth = depth_msg.depth

        self.get_logger().info(
            f"Hi! I'm your controller running. "
            f'I received a depth of {current_depth} m.',
            throttle_duration_sec=1,
        )

        thrust = self.compute_control_output(current_depth)
        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # timestamp = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    # Motor Output

    def publish_vertical_thrust(self, thrust: float, timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    def compute_control_output(self, current_depth: float) -> float:                                #Our actual code
        current_time = self.get_clock().now()   #getting time

        if self.past is None:                   #for the first loop
            self.past = current_time
            self.dt = 0.0
        else:
            self.dt = (current_time - self.past).nanoseconds * 1e-09
            self.past = current_time

        # P - Controller

        self.error = self.current_setpoint - current_depth
        

        # I - Controller

        if(     self.k_i*(self.integral + (self.error * self.dt)) < self.windup_limit
           and   self.k_i*(self.integral + (self.error * self.dt)) >- self.windup_limit):
            self.integral += (self.error * self.dt)
        else:
            self.integral = self.integral/abs(self.integral) * self.windup_limit/self.k_i

        # D - Controller

        if self.dt > 0.0:
            self.diff = (self.error - self.prior_error) / self.dt   
        else:
            self.diff = 0.0
        
        self.prior_error = self.error

        # PID - Controller Output

        thrust_z = (self.k_p * self.error) + (self.k_d * self.diff) + (self.k_i * self.integral)
        
        #Publishing all relevant data for plotjuggler

        self.publish_setpointerror_msg( setpoint_error = self.error,        now = current_time) 
        self.publish_pgain_msg(         p_gain = self.k_p * self.error,     now = current_time)
        self.publish_dgain_msg(         d_gain = self.k_d * self.diff,      now = current_time)
        self.publish_igain_msg(         i_gain = self.k_i * self.integral,  now = current_time)      
        
        #Safety

        if((current_depth < -0.8 and thrust_z < 0.0) or (current_depth > -0.1 and thrust_z >0.0)):
            return 0.0
        else:
            return thrust_z
        

def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
