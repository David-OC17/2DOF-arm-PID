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
from std_msgs.msg import Float64, Float32MultiArray, Float64MultiArray

class PID_Controller(Node):
    def __init__(self, kp:float, ki:float, kd:float):
        super().__init__('pid_controller_node')
        # PID tune parameters 
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Error, integral and derivative 
        self.error = [0.0, 0.0]
        self.error_integral = [0.0, 0.0]
        self.error_derivative = [0.0, 0.0]
        # Auxiliar variables 
        self.last_error = [0.0, 0.0]
        self.target = [0.0, 0.0]
        self.measurement = [0.0, 0.0]
        self.last_update_time = time.time()
        # Output 
        self.output = [0.0, 0.0]
        # ROS 
        self.publisher = self.create_publisher(Float64MultiArray, 'PID_Output', 10)
        self.client_target = self.create_subscription(Float64MultiArray, 'desiredAnglesPID', self.target_changed_callback)
        self.client_measurement = self.create_subscription(Float32MultiArray, 'desired_angles', self.measurement_changed_callback)

    def target_changed_callback(self, data):
        ''' Target in format [theta1, theta2] for joints '''
        self.target = data[:]

    def measurement_changed_callback(self, data):
        ''' Retrieve the motor positions from Kalman filter '''
        self.measurement = data[0:1]
        self.updatePID()

    def updatePID(self):
        ''' Compute PID for both motor joints '''
        dt = time.time() - self.last_update_time
        if(dt <= 0):
            self.get_logger().info("Invalid PID iteration: dt = 0")
            return

        for i in range(2):
            self.error[i] = self.target[i] - self.measurement[i]
            delta_error = (self.error[i] - self.last_error[i]) / dt
            self.output[i] = (self.error[i] * self.kp) + (self.error_integral[i] * self.ki) + (delta_error *  self.kd)
            self.error_integral[i] += (self.error[i] * dt)
            self.last_error[i] = self.error[i]

        self.last_update_time = time.time()

        # Publish to topic 
        output_msg = Float64MultiArray()
        output_msg.data = [self.output[0], self.output[1]]
        self.publisher.publish(output_msg)
        

def main():
    rclpy.init()
    motor_node = PID_Controller(2.29, 0.0, 2.08)
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()