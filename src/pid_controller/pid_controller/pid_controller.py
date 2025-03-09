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
        
        # Desired positions only (theta1, theta2)
        self.target_angles = [0.0, 0.0]  
        # Actual states (vel1, vel2, theta1, theta2)
        self.actual_vel = [0.0, 0.0]          
        self.actual_theta = [0.0, 0.0]        
        
        # PID terms
        self.error_pos = [0.0, 0.0]          
        self.error_integral = [0.0, 0.0]
        
        # ROS interfaces
        self.publisher = self.create_publisher(Float64MultiArray, 'control_law', 10)
        self.target_sub = self.create_subscription(Float64MultiArray, 'desiredJoint', self.target_callback, 10)
        self.joint_state_sub = self.create_subscription(Float32MultiArray, 'joint_states', self.joint_state_callback, 10)
        self.last_update_time = self.get_clock().now()

    def target_callback(self, msg):
        ''' Receive desired ANGLES: [theta1, theta2] '''
        if len(msg.data) == 2:
            self.target_angles = msg.data
        else:
            self.get_logger().error(f"Invalid desiredJoint message. Expected 2 angles, got {len(msg.data)}")

    def joint_state_callback(self, msg):
        ''' Receive current state: [vel1, vel2, theta1, theta2] '''
        if len(msg.data) == 4:
            self.actual_vel = msg.data[0:2]
            self.actual_theta = msg.data[2:4]
            self.updatePID()
        else:
            self.get_logger().error(f"Invalid joint_states message. Expected 4 elements, got {len(msg.data)}")

    def updatePID(self):
        ''' Compute PD control using position error + velocity feedback '''
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds * 1e-9
        
        if dt < 1e-6:  # Minimum time delta check
            return

        for i in range(2):
            # Position error (desired theta - actual theta)
            self.error_pos[i] = self.target_angles[i] - self.actual_theta[i]
            
            # Integral term (anti-windup recommended for real systems)
            self.error_integral[i] += self.error_pos[i] * dt
            
            # Derivative term: uses ACTUAL VELOCITY for damping (assumes desired velocity = 0)
            # This is equivalent to -Kd * actual_vel since desired_vel = 0
            control_output = (self.kp * self.error_pos[i] +
                            self.ki * self.error_integral[i] -
                            self.kd * self.actual_vel[i])

            self.output[i] = int(control_output)

        self.last_update_time = current_time
        
        # Publish voltages
        output_msg = Float64MultiArray()
        output_msg.data = self.output
        self.publisher.publish(output_msg)

def main():
    rclpy.init()
    motor_node = PID_Controller(2.29, 0.0, 2.08)
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
