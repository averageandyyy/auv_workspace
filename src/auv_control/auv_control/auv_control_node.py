import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from .pid_controller import HeaveController, YawController, SurgeController  # Import your PID controller class
import numpy as np

class AUVControlNode(Node):
    def __init__(self):
        super().__init__('auv_control_node')
        self.declare_parameter('desired_pose', [0.0, 0.0, 0.0])  # Default desired pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            'pose_topic',  # Replace with your actual Pose topic name
            self.pose_callback,
            10)
        self.surge_controller = SurgeController()
        self.heave_controller = HeaveController()
        self.yaw_controller = YawController()
        self.current_pose_msg = None
        self.thruster_config = np.array([
            # Define the matrix based on your AUV's specific configuration
            # Rows correspond to forces and torques (Fx, Fy, Fz, Tx, Ty, Tz)
            # Columns correspond to each thruster's contribution
            # Example:
            [0.1, 0.0, 0.0, 0.0],
            [0.0, 0.1, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0.0],
            [0.0, 0.0, 0.0, 0.05],
            [0.0, 0.0, 0.0, 0.05],
            [0.0, 0.0, 0.0, 0.05]
        ])

        self.forward_1_publisher = self.create_publisher(Float32, "forward_1_pwm", 10)
        self.forward_2_publisher = self.create_publisher(Float32, "forward_2_pwm", 10)
        self.forward_3_publisher = self.create_publisher(Float32, "forward_3_pwm", 10)
        self.heave_1_publisher = self.create_publisher(Float32, "heave_1_pwm", 10)
        self.heave_2_publisher = self.create_publisher(Float32, "heave_2_pwm", 10)
        self.heave_3_publisher = self.create_publisher(Float32, "heave_3_pwm", 10)
        
        self.thruster_publishers = [
            self.forward_1_publisher,
            self.forward_2_publisher,
            self.forward_3_publisher,
            self.heave_1_publisher,
            self.heave_2_publisher,
            self.heave_3_publisher,
        ]


    def pose_callback(self, msg):
        desired_pose = self.get_parameter('desired_pose').get_parameter_value().double_array_value
        self.current_pose_msg = [msg.position.x, msg.position.y, msg.position.z]

        
        # Compute control inputs for different motion aspects
        surge_control_input = self.surge_controller.compute(desired_pose[0], self.current_pose[0])
        heave_control_input = self.heave_controller.compute(desired_pose[1], self.current_pose[1])
        yaw_control_input = self.yaw_controller.compute(desired_pose[2], self.current_pose[2])

        combined_control_input = self.combine_controls(surge_control_input, heave_control_input, yaw_control_input)

        # Map control inputs to PWM values (if needed)
        pwm_values = self.map_to_pwm(combined_control_input)

        # Publish PWM values as thruster commands
        # Implement this part based on your thruster control setup
        self.publish_thruster_commands(pwm_values)
    
    def control_loop(self):
        # Define the control loop logic here
        rate = self.create_rate(10)  # Adjust the loop rate as needed (e.g., 10 Hz)

        while rclpy.ok():
            # Get desired pose from parameters (update if needed)
            desired_pose = self.get_parameter('desired_pose').get_parameter_value().double_array_value

            # Get current pose from the Pose topic (update if needed)
            current_pose = self.get_current_pose()  # Implement the method to fetch current pose

            # Calculate control inputs for different aspects of motion
            surge_control_input = self.surge_controller.compute(desired_pose[0], current_pose[0])
            heave_control_input = self.heave_controller.compute(desired_pose[1], current_pose[1])
            yaw_control_input = self.yaw_controller.compute(desired_pose[2], current_pose[2])

            # Combine control inputs to create an overall control signal
            desired_forces = [surge_control_input, heave_control_input, yaw_control_input, 0.0, 0.0, 0.0]
            thruster_outputs = self.formulate_linear_system(desired_forces)

            # Map control inputs to PWM values (if needed)
            pwm_values = self.map_to_pwm(thruster_outputs)

            # Publish PWM values as thruster commands
            self.publish_thruster_commands(pwm_values)

            rate.sleep()
    
    def get_current_pose(self):
        if self.current_pose_msg is not None:
            current_pose = [self.current_pose_msg.position.x, self.current_pose_msg.position.y, self.current_pose_msg.position.z]
            return current_pose
        else:
            # Handle the case where the current pose message is not available yet
            return [0.0, 0.0, 0.0]  # Default to zero pose if not available
        
    def publish_thruster_commands(self, pwm_values):
        # Publish the PWM values for each thruster
        msg = Float32()
        for i, output in enumerate(pwm_values):
            msg.data = output
            self.thruster_publishers[i].publish(msg)


    
    
    def formulate_linear_system(self, desired_forces):
        thruster_outputs, _, _, _ = np.linalg.lstsq(self.thruster_config, desired_forces, rcond=None)

        return thruster_outputs.tolist()

    
    # def combine_controls(surge_control, heave_control, yaw_control, weights=[1.0, 1.0, 1.0]):
    #     combined_control = (
    #     weights[0] * surge_control +
    #     weights[1] * heave_control +
    #     weights[2] * yaw_control
    #     )   
    #     return combined_control
    
    def map_to_pwm(self, control_inputs):
        # Implement your mapping logic to PWM values here
        # Ensure the values are within the range (-127 to 128)
        pass

def main(args=None):
    rclpy.init(args=args)
    auv_control_node = AUVControlNode()
    auv_control_node.control_loop()  # Start the control loop
    rclpy.spin(auv_control_node)
    auv_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
