'''
Provide the ability to control the 2 DOF motor actuators via a PID (or PD) partial state feedback.
Calculate the required (voltage1, voltage2) for the motors from some desired reference
(theta1, theta2) given by a topic desiredJoint.
'''

import time
import control as ctrl
# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray

class PID_Controller(Node):
    def __init__(self, kp:float, ki:float, kd:float):
        super().__init__('pid_controller_node')
        # PID tune parameters 
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Error, integral and derivative 
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0
        # Auxiliar variables 
        self.last_error = 0.0
        self.last_update_time = time.time()
        self.target = 0.0
        self.measurement = 0.0
        # Output 
        self.output = 0.0
        # ROS 
        self.publisher = self.create_publisher(Float64, 'PID_Output', 10)
        self.client_target = self.create_subscription(Float64, 'desiredAnglesPID', self.target_changed_callback)
        self.client_measurement = self.create_subscription(Float32MultiArray, 'joint_state_node_publisher', self.measurement_changed_callback)

    def target_changed_callback(self, data):
        self.target = data

    def measurement_changed_callback(self, data):
        self.measurement = data
        self.updatePID()

    def updatePID(self):
        self.error = self.target - self.measurement
        delta_error = (self.error - self.last_error) / (time.time() - self.last_update_time)
        self.output = (self.error * self.kp) + (self.error_integral * self.ki) + (delta_error *  self.kd)
        self.error_integral += self.error
        self.last_error = self.error
        self.lastUpdateTime = time.time()
        # Publish to topic 
        output_msg = Float64()
        output_msg.data = self.output
        self.publisher.publish(output_msg)
        

def main():
    rclpy.init()
    motor1_node = PID_Controller(2.29, 0.0, 2.08)
    motor2_node = PID_Controller(2.29, 0.0, 2.08)
    rclpy.spin(motor1_node)
    rclpy.spin(motor2_node)
    motor1_node.destroy_node()
    motor2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()