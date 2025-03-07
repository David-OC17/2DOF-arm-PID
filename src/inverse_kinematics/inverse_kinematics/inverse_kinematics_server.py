'''
Provide the ability to determine if some (x,y) desired end-effector position is reachable, and
calcualte the inverse kinematics to that point, outputing some (theta1, theta2) for the angles
of the actuators in the 2 DOF robotic arm. This works as a service.

service: desiredPosition
SEND
float64 desired_x
float64 desired_y

EXPECT
bool valid_position
'''

# cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2*L1*L2)
# sin_theta2 = (1-cos_theta2**2)**(1/2)

# solucion codo arriba: 
# theta2_EUP = math.degrees(math.atan2(sin_theta2 / cos_theta2))
# solucion codo abajo:
# theta2_EDOWN = math.degrees(math.atan2(-sin_theta2 / cos_theta2))
# theta1 = math.degrees(math.atan2(y/x) - math.atan2((L2*sin_theta2) / (L1 + L2*cos_theta2)))

import rclpy
import math
from typing import Tuple
from rclpy.node import Node
from desired_position_pkg.srv import DesiredPosition
from std_msgs.msg import Float64MultiArray

class JointAngleSolution():
    def __init__(self, theta1_solution1, theta2_solution1, theta1_solution2, theta2_solution2):
        self.theta1_solution1 = theta1_solution1
        self.theta2_solution1 = theta2_solution1

        self.theta1_solution2 = theta1_solution2
        self.theta2_solution2 = theta2_solution2

class InverseKinematicsServer(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_service')
        self.srv = self.create_service(DesiredPosition, 'desiredPosition', self.handle_position_request)
        self.publisher = self.create_publisher(Float64MultiArray, 'desiredAnglesPID', 10)
        self.get_logger().info("Inverse Kinematics Service is ready.")

        # TODO adjust these values to real ones
        self.L1 = 120 # mm
        self.L2 = 88  # mm

        self.min_valid_y_mm = -15

        self.min_valid_theta1_deg = -10
        self.min_valid_theta2_deg = -150

        self.max_valid_theta1_deg = 190
        self.max_valid_theta2_deg = 150
    
    def handle_position_request(self, request, response):
        x, y = request.desired_x, request.desired_y
        self.get_logger().info(f"Received request for position ({x}, {y})")
        
        response.valid_position = self.validate_position(x, y)
        joint_angle_solution = JointAngleSolution(math.inf, math.inf, math.inf, math.inf,)

        if response.valid_position:
            joint_angle_solution = self.calculate_joint_angles(x, y)
            self.get_logger().info(f"Position ({x}, {y}) is valid. Calculating joint angles.")
        else:
            self.get_logger().warn(f"Position ({x}, {y}) is invalid. Request rejected.")

        sol1_validity, sol2_validity = self.validate_angles(joint_angle_solution)

        if sol1_validity:
            self.publish_joint_angles(joint_angle_solution.theta1_solution1, joint_angle_solution.theta2_solution1)
            self.get_logger().info(f"Position ({x}, {y}) is valid. Sending joint angles ({joint_angle_solution.theta1_solution1}, {joint_angle_solution.theta2_solution1}).")
        elif sol2_validity:
            self.publish_joint_angles(joint_angle_solution.theta1_solution2, joint_angle_solution.theta2_solution2)
            self.get_logger().info(f"Position ({x}, {y}) is valid. Sending joint angles ({joint_angle_solution.theta1_solution2}, {joint_angle_solution.theta2_solution2}).")
        else:
            self.get_logger().warn(f"No valid angles can reach, thus ({x}, {y}) is invalid. Request rejected.")
            response.valid_position = False

        return response
    
    def validate_position(self, x, y):
        max_len = (self.L1 + self.L2)
        desired_len = math.sqrt(x**2 + y**2)

        # Validate len & prohibited regions given by arm morphology
        if desired_len > max_len or  y < self.min_valid_y_mm:
            return False
        
        return True
    
    def calculate_joint_angles(self, x, y) -> JointAngleSolution:
        '''
        Calculate both joint angle solution pairs (theta1, theta2) for some position (x, y) by using
        '''
        joint_angle_solution = JointAngleSolution(0, 0, 0, 0)
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2*self.L1*self.L2)
        sin_theta2 = (1-cos_theta2**2)**(1/2)

        joint_angle_solution.theta2_solution1 = math.degrees(math.atan2(sin_theta2 / cos_theta2))
        joint_angle_solution.theta1_solution1 = math.degrees(math.atan2(y/x) - math.atan2((self.L2*math.sin(math.radians(joint_angle_solution.theta2_solution1))) / (self.L1 + self.L2*math.cos(math.radians(joint_angle_solution.theta2_solution1)))))
        joint_angle_solution.theta2_solution2 = math.degrees(math.atan2(-sin_theta2 / cos_theta2))
        joint_angle_solution.theta1_solution2 = math.degrees(math.atan2(y/x) - math.atan2((self.L2*math.sin(math.radians(joint_angle_solution.theta2_solution2))) / (self.L1 + self.L2*math.cos(math.radians(joint_angle_solution.theta2_solution2)))))

        if joint_angle_solution.theta2_solution1 == joint_angle_solution.theta2_solution2:
            joint_angle_solution.theta2_solution2 = math.inf
        

        return joint_angle_solution
    
    def validate_angles(self, joint_angle_solution: JointAngleSolution) -> Tuple[bool, bool]:
        '''
        Return if if each angle pair in the solutions is valid.
        '''
        solution1_validity = (
            joint_angle_solution.theta1_solution1 < self.max_valid_theta1_deg and
            joint_angle_solution.theta1_solution1 > self.min_valid_theta1_deg and
            joint_angle_solution.theta2_solution1 > self.min_valid_theta2_deg and
            joint_angle_solution.theta2_solution1 < self.max_valid_theta2_deg
        )

        solution2_validity = (
            joint_angle_solution.theta1_solution2 < self.max_valid_theta1_deg and
            joint_angle_solution.theta1_solution2 > self.min_valid_theta1_deg and
            joint_angle_solution.theta2_solution2 > self.min_valid_theta2_deg and
            joint_angle_solution.theta2_solution2 < self.max_valid_theta2_deg
        )

        return (solution1_validity, solution2_validity)
    
    def publish_joint_angles(self, theta1, theta2):
        '''
        Publish solution angles (theta1, theta2) to get to position (x, y) to desiredAnglesPID topic.
        '''
        msg = Float64MultiArray()
        msg.data = [theta1, theta2]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint angles: ({theta1}, {theta2})")

def main():
    rclpy.init()
    node = InverseKinematicsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
