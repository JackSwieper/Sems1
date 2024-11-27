#!/usr/bin/env python3
"""
This node takes as input the pressure data and computes a resulting water depth.
"""

import rclpy
from hippo_msgs.msg import DepthStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import FluidPressure
import numpy as np


class DepthCalculator(Node):

    def __init__(self):
        super().__init__(node_name='depth_calculator')

        self.alpha = 0.05  #low pass filter coeffcient, works better with highher numbers


        self.prev_filtered_pressure = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.depth_pub = self.create_publisher(msg_type=DepthStamped,
                                               topic='depth',
                                               qos_profile=1)
        self.pressure_sub = self.create_subscription(
            msg_type=FluidPressure,
            topic='pressure',
            callback=self.on_pressure,
            qos_profile=qos,
        )

    def low_pass_filter(self, current_presssure):

        if self.prev_filtered_pressure is None:
            self.prev_filtered_pressure = current_presssure
        else:
            self.prev_filtered_pressure = self.alpha * current_presssure + (
                1 - self.alpha
            ) * self.prev_filtered_pressure  # First order low pass filter

        return self.prev_filtered_pressure

    def on_pressure(self, pressure_msg: FluidPressure) -> None:
        #test_noise = np.random.normal(-10.0, 10.0)
        pressure = pressure_msg.fluid_pressure #+ test_noise             #Comment this ütest_noise out for the real test

        filtered_pressure = self.low_pass_filter(pressure)
        # TODO: you can remove this logging function, when you are done with the
        # depth calculator implementation.
        #self.get_logger().info(
        #    f'Hello, I received a pressure of {pressure} Pa. I need to calculate the depth based on this measurement.',
        #    throttle_duration_sec=1,
        #)

        # TODO: implement the following pressure_to_depth function.
        depth = self.pressure_to_depth(filtered_pressure=filtered_pressure)
        now = self.get_clock().now()
        self.publish_depth_msg(depth=depth, now=now)

    def publish_depth_msg(self, depth: float, now: rclpy.time.Time) -> None:
        msg = DepthStamped()  # Let's add a time stamp
        msg.header.stamp = now.to_msg()  # and populate the depth field
        msg.depth = depth
        self.depth_pub.publish(msg)

    def pressure_to_depth(self, filtered_pressure: float) -> float:
        # TODO: implement the required depth calculation

        depth = -(
            filtered_pressure - 102370.98
        ) / 9810 - 0.10  # Formel umgestellt aus p = 1bar + d*g*roh, anschließend um Offset-Summand ergänzt sollte der Sensor nicht mittig sitzen.
        return depth


def main():
    rclpy.init()
    node = DepthCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
