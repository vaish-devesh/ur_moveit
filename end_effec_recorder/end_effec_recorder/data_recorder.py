import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarkerFeedback
import pandas as pd

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # Define an empty list to store message data
        self.data = []

        # Subscribe to the topic where the InteractiveMarkerFeedback data is being published
        self.subscription = self.create_subscription(
            InteractiveMarkerFeedback,  # Message type
            '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',  # Topic name
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Define the Excel file path
        self.excel_file_path = 'endeffector.xlsx'

    def listener_callback(self, msg):
        # Add received message data to the list
        self.data.append({
            'marker_name': msg.marker_name,
            'event_type': msg.event_type,
            'x_position': msg.pose.position.x,
            'y_position': msg.pose.position.y,
            'z_position': msg.pose.position.z,
            'x_orientation': msg.pose.orientation.x,
            'y_orientation': msg.pose.orientation.y,
            'z_orientation': msg.pose.orientation.z,
            'w_orientation': msg.pose.orientation.w,
            'timestamp': self.get_clock().now().to_msg().sec  # Log timestamp
        })

        # Log received data (optional)
        self.get_logger().info(f'Received: {msg.marker_name}, position: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})')

    def export_to_excel(self):
        # Convert the list of data into a pandas DataFrame
        df = pd.DataFrame(self.data)

        # Export the DataFrame to an Excel file
        df.to_excel(self.excel_file_path, index=False)
        self.get_logger().info(f'Data successfully exported to {self.excel_file_path}')

def main(args=None):
    rclpy.init(args=args)

    data_recorder = DataRecorder()

    try:
        rclpy.spin(data_recorder)
    except KeyboardInterrupt:
        # On shutdown, export the data to Excel
        data_recorder.export_to_excel()
    finally:
        data_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
