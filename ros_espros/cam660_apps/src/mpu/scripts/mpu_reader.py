#!/usr/bin/env python3

import rospy
import socket
import json
import math
from sensor_msgs.msg import Imu


class MPUReceiver:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5005))
        self.sock.settimeout(1.0)

        # Scaling Factors
        self.ACCEL_SCALE = 16384.0
        self.GYRO_SCALE = 131.0
        self.GRAVITY = 9.80665

        self.pub_imu = rospy.Publisher(
            "/imu/data_raw",
            Imu,
            queue_size=10
        )

        self.run()

    def run(self):
        rospy.loginfo("Started ESP32 UDP Receiver")

        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(1024)
                raw_data = json.loads(data.decode('utf-8'))
                # Debug print
                # print(f"Accel X: {raw_data['AcX']} | Accel Y: {raw_data['AcY']}")
                # Publish to publisher
                msg = Imu()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "imu_link"

                msg.linear_acceleration.x = (raw_data['AcX'] / self.ACCEL_SCALE)*self.GRAVITY
                msg.linear_acceleration.y = (raw_data['AcY'] / self.ACCEL_SCALE)*self.GRAVITY
                msg.linear_acceleration.z = (raw_data['AcZ'] / self.ACCEL_SCALE)*self.GRAVITY

                msg.angular_velocity.x = math.radians(raw_data['GyX'] / self.GYRO_SCALE)
                msg.angular_velocity.y = math.radians(raw_data['GyY'] / self.GYRO_SCALE)
                msg.angular_velocity.z = math.radians(raw_data['GyZ'] / self.GYRO_SCALE)

                # Covariance
                msg.linear_acceleration_covariance[0] = 0.01
                msg.linear_acceleration_covariance[4] = 0.01
                msg.linear_acceleration_covariance[8] = 0.01

                msg.angular_velocity_covariance[0] = 0.01
                msg.angular_velocity_covariance[4] = 0.01
                msg.angular_velocity_covariance[8] = 0.01

                # Orientation?
                msg.orientation_covariance[0] = 0.01
                msg.orientation_covariance[4] = 0.01
                msg.orientation_covariance[8] = 0.01

                self.pub_imu.publish(msg)

            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error: {e}")


if __name__ == "__main__":
    rospy.init_node("mpu_reader")
    try:
        MPUReceiver()
    except rospy.ROSInterruptException:
        pass

