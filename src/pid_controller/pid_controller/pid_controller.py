'''
Provide the ability to control the 2 DOF motor actuators via a PID (or PD) complete state feedback.
Calculate the required (voltage1, voltage2) for the motors from some desired reference
(velocity1, velocity2, theta1, theta2) given by a topic desiredJoint. (Angular velocity)
'''

import time
# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray

def constrain(x, a, b):
    return max(a, min(x, b))

class PID_Controller(Node):
    def __init__(self, kp1: float, ki1: float, kd1: float, kp2: float, ki2: float, kd2: float):
        super().__init__('pid_controller_node')
        # Variables
        self.current_position = [0.0, 0.0]
        self.desired_position = [0.0, 0.0]
        self.desired_velocity = [0.0, 0.0]
        self.control_signals = [0.0, 0.0]


        # PID parameters 
        self.Kp = [kp1, kp2]
        self.Ki = [ki1, ki2]
        self.Kd = [kd1, kd2]

        self.lastInput = [0.0, 0.0]
        self.integral = [0.0, 0.0]
        self.lastTime = self.get_clock().now()
        self.deltaTime = self.get_clock().now()

        
        # ROS interfaces
        self.publisher = self.create_publisher(Float64MultiArray, 'control_law', 10)
        self.target_sub = self.create_subscription(Float64MultiArray, 'desiredJoint', self.target_callback, 10)
        self.joint_state_sub = self.create_subscription(Float32MultiArray, 'joint_states', self.joint_state_callback, 10)

        self.timer = self.create_timer(1.0, self.updatePID)

        self.get_logger().info('Ready to control!!')


    def target_callback(self, msg):
        ''' Handle desired angles '''
        if len(msg.data) == 2:
            self.desired_position = list(msg.data)
            self.get_logger().info(f"New target received: {self.desired_position}")
        else:
            self.get_logger().error("Invalid desiredAnglesPID message. Expected 2 angles.")

    def joint_state_callback(self, msg):
        ''' Update actual state  '''
        if len(msg.data) == 4:
            self.actual_vel = list(msg.data[0:2])
            self.current_position = list(msg.data[2:4])
        else:
            self.get_logger().error("Invalid joint_states message. Expected 4 elements.")

    def updatePID(self):
        ''' Compute control output using latest available data '''
    
        current_time = self.get_clock().now()
        self.deltaTime = (current_time - self.lastTime).nanoseconds / 1000000000.0
        self.lastTime = self.get_clock().now()

        error1 = self.current_position[0] - self.desired_position[0]
        error2 = self.current_position[1] - self.desired_position[1]

        self.integral[0] += error1 * self.deltaTime
        self.integral[0] = constrain(self.integral[0], 100, -100)
        self.integral[1] += error2 * self.deltaTime
        self.integral[1] = constrain(self.integral[1], 100, -100)

        derivative1 = (self.current_position[0] - self.lastInput[0]) / self.deltaTime
        derivative2 = (self.current_position[1] - self.lastInput[1]) / self.deltaTime

        self.control_signals[0] = self.Kp[0]*error1 + self.Ki[0]*self.integral[0] + self.Kd[0]*derivative1
        self.control_signals[1] = self.Kp[1]*error2 + self.Ki[1]*self.integral[1] + self.Kd[1]*derivative2

        self.control_signals[0] = constrain(self.control_signals[0], -255, 255)
        self.control_signals[1] = constrain(self.control_signals[1], -255, 255)
    
        # Publish immediately
        output_msg = Float64MultiArray()
        output_msg.data = self.control_signals
        self.publisher.publish(output_msg)
        
        self.get_logger().info(f'Control Signal: {self.control_signals}')

def main():
    rclpy.init()
    motor_node = PID_Controller(1.7, 0.8, 0.001, 1.2, 0.8, 0.001)
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
