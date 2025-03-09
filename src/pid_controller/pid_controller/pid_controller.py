'''
Provide the ability to control the 2 DOF motor actuators via a PID (or PD) complete state feedback.
Calculate the required (voltage1, voltage2) for the motors from some desired reference
(velocity1, velocity2, theta1, theta2) given by a topic desiredJoint. (Angular velocity)
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
        # PID parameters 
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # State initialization (crucial for startup)
        self.target_angles = [0.0, 0.0]  # Desired angles
        self.actual_vel = [0.0, 0.0]     # Initialize to zero
        self.actual_theta = [0.0, 0.0]   # Initialize to zero
        self.error_integral = [0.0, 0.0]
        
        # ROS interfaces
        self.publisher = self.create_publisher(Float64MultiArray, 'control_law', 10)
        self.target_sub = self.create_subscription(Float64MultiArray, 'desiredJoint', self.target_callback, 10)
        self.joint_state_sub = self.create_subscription(Float32MultiArray, 'joint_states', self.joint_state_callback, 10)
        
        # Time tracking using ROS clock
        self.last_update_time = self.get_clock().now()

    def target_callback(self, msg):
        ''' Handle desired angles and trigger immediate PID update '''
        if len(msg.data) == 2:
            self.target_angles = msg.data
            self.get_logger().info(f"New target received: {self.target_angles}")
            self.updatePID()  # Critical: Trigger PID with initial zero states
        else:
            self.get_logger().error("Invalid desiredJoint message. Expected 2 angles.")

    def joint_state_callback(self, msg):
        ''' Update actual state and trigger PID '''
        if len(msg.data) == 4:
            self.actual_vel = msg.data[0:2]
            self.actual_theta = msg.data[2:4]
            self.updatePID()
        else:
            self.get_logger().error("Invalid joint_states message. Expected 4 elements.")

    def updatePID(self):
        ''' Compute control output using latest available data '''
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds * 1e-9
        
        if dt < 1e-6:  # Avoid division by zero
            return

        # Calculate position errors
        error_pos = [
            self.target_angles[0] - self.actual_theta[0],
            self.target_angles[1] - self.actual_theta[1]
        ]

        # PID computation
        voltages = [0.0, 0.0]
        for i in range(2):
            self.error_integral[i] += error_pos[i] * dt
            
            # Control law: Kp*e + Ki*∫e dt + Kd*(-velocity)
            voltages[i] = (
                self.kp * error_pos[i] +
                self.ki * self.error_integral[i] -
                self.kd * self.actual_vel[i]
            )
    

        # Publish immediately
        output_msg = Float64MultiArray()
        output_msg.data = int(voltages)
        self.publisher.publish(output_msg)
        self.last_update_time = current_time

def main():
    rclpy.init()
    motor_node = PID_Controller(2.29, 0.0, 2.08)
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
