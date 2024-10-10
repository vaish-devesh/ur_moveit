# import rclpy
# from rclpy.node import Node
# from visualization_msgs.msg import InteractiveMarkerFeedback
# from geometry_msgs.msg import Pose

# class InteractiveMarkerPublisher(Node):
#     def __init__(self):
#         super().__init__('interactive_marker_publisher')
#         self.publisher_ = self.create_publisher(
#             InteractiveMarkerFeedback,
#             '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',
#             10
#         )
#         timer_period = 1.0  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)

#     def timer_callback(self):
#         feedback_msg = InteractiveMarkerFeedback()

#         # Customize the marker information
#         feedback_msg.marker_name = "my_interactive_marker"
#         feedback_msg.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        
#         # Fill in the new pose for the interactive marker
#         feedback_msg.pose = Pose()
#         feedback_msg.pose.position.x = 0.13003000617027283
#         feedback_msg.pose.position.y = 0.33588409423828125
#         feedback_msg.pose.position.z = 0.587088942527771

#         feedback_msg.pose.orientation.x =  0.9239556789398193
#         feedback_msg.pose.orientation.y = -0.38249948620796204
#         feedback_msg.pose.orientation.z = 1.3249325681724544e-12
#         feedback_msg.pose.orientation.w = 3.200411723830454e-12
        



#         # Publish the feedback message
#         self.publisher_.publish(feedback_msg)
#         self.get_logger().info(f'Publishing updated marker pose: {feedback_msg.pose.position.x}, {feedback_msg.pose.position.y}, {feedback_msg.pose.position.z}')

# def main(args=None):
#     rclpy.init(args=args)

#     interactive_marker_publisher = InteractiveMarkerPublisher()

#     try:
#         rclpy.spin(interactive_marker_publisher)
#     except KeyboardInterrupt:
#         pass

#     interactive_marker_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarkerFeedback
from geometry_msgs.msg import Pose
import pandas as pd
import time

class InteractiveMarkerPublisher(Node):
    def __init__(self):
        super().__init__('interactive_marker_publisher')

        # Load the CSV file
        self.excel_data = pd.read_excel('~/ur_ws2/src/interactive_mark_update/interactive_mark_update/endeffector.xlsx')
        self.current_index = 0  # Start at the first row

        # Get the total number of rows
        self.total_rows = len(self.excel_data)

        self.publisher_ = self.create_publisher(
            InteractiveMarkerFeedback,
            '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',
            10
        )

        # Timer to update pose every 0.1 seconds
        timer_period = 2.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.current_index < self.total_rows:
            feedback_msg = InteractiveMarkerFeedback()

            # Customize the marker information
            feedback_msg.marker_name = "my_interactive_marker"
            feedback_msg.event_type = InteractiveMarkerFeedback.POSE_UPDATE

            # Fill in the new pose for the interactive marker from CSV
            feedback_msg.pose = Pose()
            feedback_msg.pose.position.x = self.excel_data.iloc[self.current_index]['x_position']
            feedback_msg.pose.position.y = self.excel_data.iloc[self.current_index]['y_position']
            feedback_msg.pose.position.z = self.excel_data.iloc[self.current_index]['z_position']

            feedback_msg.pose.orientation.x = self.excel_data.iloc[self.current_index]['x_orientation']
            feedback_msg.pose.orientation.y = self.excel_data.iloc[self.current_index]['y_orientation']
            feedback_msg.pose.orientation.z = self.excel_data.iloc[self.current_index]['z_orientation']
            feedback_msg.pose.orientation.w = self.excel_data.iloc[self.current_index]['w_orientation']

            # Publish the feedback message
            self.publisher_.publish(feedback_msg)

            # Log the current pose being published
            self.get_logger().info(
                f'Publishing updated marker pose: x={feedback_msg.pose.position.x}, '
                f'y={feedback_msg.pose.position.y}, z={feedback_msg.pose.position.z}'
            )

            # Move to the next row
            self.current_index += 1
        else:
            # Stop when all rows have been processed
            self.get_logger().info("All rows processed, stopping.")
            self.destroy_node()  # Shutdown after completing

def main(args=None):
    rclpy.init(args=args)

    interactive_marker_publisher = InteractiveMarkerPublisher()

    try:
        rclpy.spin(interactive_marker_publisher)
    except KeyboardInterrupt:
        pass

    interactive_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
