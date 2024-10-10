// #include <vector>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// class MoveItNode : public rclcpp::Node
// {
// public:
//     MoveItNode() : Node("moveit_node"), executing_(false)
//     {
//         // Create the MoveIt MoveGroup Interface
//         move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "panda_arm");

//         // Subscribe to feedback topic for new goal states
//         subscription_ = this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
//             "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
//             10,
//             std::bind(&MoveItNode::feedback_callback, this, std::placeholders::_1));
//     }

// private:
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
//     rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr subscription_;
//     bool executing_; // Flag to track if a plan is being executed

//     // Callback to handle new feedback (goal state)
//     void feedback_callback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received feedback! Planning new motion...");

//         // Extract position and orientation from the feedback message
//         double x_position = msg->pose.position.x;
//         double y_position = msg->pose.position.y;
//         double z_position = msg->pose.position.z;
//         double x_orientation = msg->pose.orientation.x;
//         double y_orientation = msg->pose.orientation.y;
//         double z_orientation = msg->pose.orientation.z;
//         double w_orientation = msg->pose.orientation.w;

//         // Set a new pose target
//         geometry_msgs::msg::Pose target_pose;
//         target_pose.position.x = x_position;
//         target_pose.position.y = y_position;
//         target_pose.position.z = z_position;
//         target_pose.orientation.x = x_orientation;
//         target_pose.orientation.y = y_orientation;
//         target_pose.orientation.z = z_orientation;
//         target_pose.orientation.w = w_orientation;

//         // Stop current motion if already executing
//         if (executing_)
//         {
//             RCLCPP_WARN(this->get_logger(), "New goal received. Stopping current motion.");
//             move_group_interface_->stop();
//         }

//         // Plan and execute the new motion asynchronously
//         plan_and_execute_async(target_pose);
//     }

//     // Function to plan and execute the motion asynchronously
//     void plan_and_execute_async(const geometry_msgs::msg::Pose &target_pose)
//     {
//         // Set the pose target
//         move_group_interface_->setPoseTarget(target_pose);

//         // Perform planning
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         bool success = static_cast<bool>(move_group_interface_->plan(plan));

//         if (success)
//         {
//             RCLCPP_INFO(this->get_logger(), "Planning successful. Executing motion...");

//             // Set the executing flag to true and execute asynchronously
//             executing_ = true;
//             moveit::core::MoveItErrorCode exec_result = move_group_interface_->asyncExecute(plan);

//             if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
//             {
//                 RCLCPP_INFO(this->get_logger(), "Motion execution started.");
//             }
//             else
//             {
//                 RCLCPP_WARN(this->get_logger(), "Motion execution failed to start.");
//                 executing_ = false;
//             }
//         }
//         else
//         {
//             RCLCPP_ERROR(this->get_logger(), "Planning failed.");
//         }
//     }
// };

// int main(int argc, char *argv[])
// {
//     // Initialize ROS 2 and the node
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MoveItNode>();

//     // Spin the node to handle callbacks
//     rclcpp::spin(node);

//     // Shutdown ROS 2
//     rclcpp::shutdown();
//     return 0;
// }











// #include <vector>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// class MoveItNode : public rclcpp::Node
// {
// public:
//     MoveItNode() : Node("moveit_node"), executing_(false)
//     {
//         // Create the MoveIt MoveGroup Interface
//         move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "panda_arm");

//         // Subscribe to feedback topic for new goal states
//         subscription_ = this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
//             "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
//             10,
//             std::bind(&MoveItNode::feedback_callback, this, std::placeholders::_1));
//     }

// private:
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
//     rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr subscription_;
//     bool executing_; // Flag to track if a plan is being executed

//     // Callback to handle new feedback (goal state)
//     void feedback_callback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received feedback! Planning new motion...");

//         // Extract position and orientation from the feedback message
//         double x_position = msg->pose.position.x;
//         double y_position = msg->pose.position.y;
//         double z_position = msg->pose.position.z;
//         double x_orientation = msg->pose.orientation.x;
//         double y_orientation = msg->pose.orientation.y;
//         double z_orientation = msg->pose.orientation.z;
//         double w_orientation = msg->pose.orientation.w;

//         // Set a new pose target
//         geometry_msgs::msg::Pose target_pose;
//         target_pose.position.x = x_position;
//         target_pose.position.y = y_position;
//         target_pose.position.z = z_position;
//         target_pose.orientation.x = x_orientation;
//         target_pose.orientation.y = y_orientation;
//         target_pose.orientation.z = z_orientation;
//         target_pose.orientation.w = w_orientation;

//         // Stop current motion if already executing
//         if (executing_)
//         {
//             RCLCPP_WARN(this->get_logger(), "New goal received. Stopping current motion.");
//             move_group_interface_->stop();
//         }

//         // Plan and execute the new motion asynchronously
//         plan_and_execute_async(target_pose);
//     }

//     // Function to plan and execute the motion asynchronously
//     void plan_and_execute_async(const geometry_msgs::msg::Pose &target_pose)
//     {
//         // Set the pose target
//         move_group_interface_->setPoseTarget(target_pose);

//         // Perform planning
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         bool success = static_cast<bool>(move_group_interface_->plan(plan));

//         if (success)
//         {
//             RCLCPP_INFO(this->get_logger(), "Planning successful. Executing motion...");

//             // Set the executing flag to true and execute asynchronously
//             executing_ = true;
//             move_group_interface_->asyncExecute(plan, [this](moveit::core::MoveItErrorCode result) {
//                 if (result == moveit::core::MoveItErrorCode::SUCCESS)
//                 {
//                     RCLCPP_INFO(this->get_logger(), "Motion executed successfully.");
//                 }
//                 else
//                 {
//                     RCLCPP_WARN(this->get_logger(), "Motion execution failed.");
//                 }
//                 // Reset the executing flag when execution is done
//                 executing_ = false;
//             });
//         }
//         else
//         {
//             RCLCPP_ERROR(this->get_logger(), "Planning failed.");
//         }
//     }
// };

// int main(int argc, char *argv[])
// {
//     // Initialize ROS 2 and the node
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MoveItNode>();

//     // Spin the node to handle callbacks
//     rclcpp::spin(node);

//     // Shutdown ROS 2
//     rclcpp::shutdown();
//     return 0;
// }





// This part clears the previous goal state. Works nicely

#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// Function to plan and execute based on given orientation values
bool plan_and_execute_pose_target(
    const std::shared_ptr<rclcpp::Node>& node, 
    const double x_position, 
    const double y_position, 
    const double z_position, 
    const double x_orientation, 
    const double y_orientation, 
    const double z_orientation, 
    const double w_orientation)
{
    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "panda_arm");

    // Set a target Pose with given orientation values
    auto const target_pose = [x_position, y_position, z_position, x_orientation, y_orientation, z_orientation, w_orientation]{
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = x_orientation;
        msg.orientation.y = y_orientation;
        msg.orientation.z = z_orientation;
        msg.orientation.w = w_orientation;
        msg.position.x = x_position; 
        msg.position.y = y_position;
        msg.position.z = z_position;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan if planning was successful
    if (success) {
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Execution successful!");

        // Clear pose targets after execution to avoid re-planning to the same target
        move_group_interface.clearPoseTargets();
        
        return true;
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
        return false;
    }
}

// Callback function to handle feedback messages
void feedback_callback(
    const std::shared_ptr<rclcpp::Node>& node, 
    const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("feedback_callback"), "Received feedback!");

    // Extract position and orientation from the feedback message
    double x_position = msg->pose.position.x;
    double y_position = msg->pose.position.y;
    double z_position = msg->pose.position.z;
    double x_orientation = msg->pose.orientation.x;
    double y_orientation = msg->pose.orientation.y;
    double z_orientation = msg->pose.orientation.z;
    double w_orientation = msg->pose.orientation.w;

    // Call the function to plan and execute the motion
    plan_and_execute_pose_target(node, x_position, y_position, z_position, x_orientation, y_orientation, z_orientation, w_orientation);
}

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a subscription to the feedback topic
    auto subscription = node->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
        "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
        10,
        [node](const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg) {
            feedback_callback(node, msg);
        }
    );

    // Spin to handle callbacks
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}















// THIS PART WORKS FOR MOVING TO A DESIRED GOAL STATE
// DO NOT DELETE





// #include <vector>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// // Function to plan and execute based on given orientation values
// bool plan_and_execute_pose_target(
//     const std::shared_ptr<rclcpp::Node>& node, 
//     const double x_position, 
//     const double y_position, 
//     const double z_position, 
//     const double w_orientation)
// {
//     // Create a ROS logger
//     auto const logger = rclcpp::get_logger("hello_moveit");

//     // Create the MoveIt MoveGroup Interface
//     using moveit::planning_interface::MoveGroupInterface;
//     auto move_group_interface = MoveGroupInterface(node, "panda_arm");

//     // Set a target Pose with given orientation values
//     auto const target_pose = [x_position, y_position, z_position, w_orientation]{
//         geometry_msgs::msg::Pose msg;
//         msg.orientation.x = 0;
//         msg.orientation.y = 0;
//         msg.orientation.z = 0;
//         msg.orientation.w = w_orientation;
//         msg.position.x = x_position; // Set these as per your requirement
//         msg.position.y = y_position;
//         msg.position.z = z_position;
//         return msg;
//     }();
//     move_group_interface.setPoseTarget(target_pose);

//     // Create a plan to that target pose
//     auto const [success, plan] = [&move_group_interface]{
//         moveit::planning_interface::MoveGroupInterface::Plan msg;
//         auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//         return std::make_pair(ok, msg);
//     }();

//     // Execute the plan if planning was successful
//     if (success) {
//         move_group_interface.execute(plan);
//         RCLCPP_INFO(logger, "Execution successful!");
//         return true;
//     } else {
//         RCLCPP_ERROR(logger, "Planning failed!");
//         return false;
//     }
// }

// int main(int argc, char *argv[])
// {
//     // Initialize ROS and create the Node
//     rclcpp::init(argc, argv);
//     auto const node = std::make_shared<rclcpp::Node>(
//         "hello_moveit",
//         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//     );

//     // Call the function with desired orientation values (for example)
//     double x_position = 0.633327;
//     double y_position = 0.0236555;
//     double z_position = 0.221904;
//     double w_orientation = 1.0; // Neutral quaternion

//     bool result = plan_and_execute_pose_target(node, x_position, y_position, z_position, w_orientation);

//     // Shutdown ROS
//     rclcpp::shutdown();
//     return result ? 0 : 1;
// }



// This is also good but kepps doing the task after it is complete and planning is executed.
// after end effector is reached , it tries to execute in several different ways, which is not so useful

// #include <vector>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// // Function to plan and execute based on given orientation values
// bool plan_and_execute_pose_target(
//     const std::shared_ptr<rclcpp::Node>& node, 
//     const double x_position, 
//     const double y_position, 
//     const double z_position, 
//     const double x_orientation, 
//     const double y_orientation, 
//     const double z_orientation, 
//     const double w_orientation)
// {
//     // Create a ROS logger
//     auto const logger = rclcpp::get_logger("hello_moveit");

//     // Create the MoveIt MoveGroup Interface
//     using moveit::planning_interface::MoveGroupInterface;
//     auto move_group_interface = MoveGroupInterface(node, "panda_arm");

//     // Set a target Pose with given orientation values
//     auto const target_pose = [x_position, y_position, z_position, x_orientation, y_orientation, z_orientation, w_orientation]{
//         geometry_msgs::msg::Pose msg;
//         msg.orientation.x = x_orientation;
//         msg.orientation.y = y_orientation;
//         msg.orientation.z = z_orientation;
//         msg.orientation.w = w_orientation;
//         msg.position.x = x_position; 
//         msg.position.y = y_position;
//         msg.position.z = z_position;
//         return msg;
//     }();
//     move_group_interface.setPoseTarget(target_pose);

//     // Create a plan to that target pose
//     auto const [success, plan] = [&move_group_interface]{
//         moveit::planning_interface::MoveGroupInterface::Plan msg;
//         auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//         return std::make_pair(ok, msg);
//     }();

//     // Execute the plan if planning was successful
//     if (success) {
//         move_group_interface.execute(plan);
//         RCLCPP_INFO(logger, "Execution successful!");
//         return true;
//     } else {
//         RCLCPP_ERROR(logger, "Planning failed!");
//         return false;
//     }
// }

// // Callback function to handle feedback messages
// void feedback_callback(
//     const std::shared_ptr<rclcpp::Node>& node, 
//     const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg)
// {
//     RCLCPP_INFO(rclcpp::get_logger("feedback_callback"), "Received feedback!");

//     // Extract position and orientation from the feedback message
//     double x_position = msg->pose.position.x;
//     double y_position = msg->pose.position.y;
//     double z_position = msg->pose.position.z;
//     double x_orientation = msg->pose.orientation.x;
//     double y_orientation = msg->pose.orientation.y;
//     double z_orientation = msg->pose.orientation.z;
//     double w_orientation = msg->pose.orientation.w;

//     // Call the function to plan and execute the motion
//     plan_and_execute_pose_target(node, x_position, y_position, z_position, x_orientation, y_orientation, z_orientation, w_orientation);
// }

// int main(int argc, char *argv[])
// {
//     // Initialize ROS and create the Node
//     rclcpp::init(argc, argv);
//     auto const node = std::make_shared<rclcpp::Node>(
//         "hello_moveit",
//         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//     );

//     // Create a subscription to the feedback topic
//     auto subscription = node->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
//         "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
//         10,
//         [node](const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg) {
//             feedback_callback(node, msg);
//         }
//     );

//     // Spin to handle callbacks
//     rclcpp::spin(node);

//     // Shutdown ROS
//     rclcpp::shutdown();
//     return 0;
// }
