#ifndef DRIFT_EVALUATOR_NODE_HPP
#define DRIFT_EVALUATOR_NODE_HPP

#include <deque>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

class DriftEvaluatorNode : public rclcpp::Node {
public:
    DriftEvaluatorNode();

private:
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void
    plannedTrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg);

    void computeAndPublishADE();
    double computeADE();
    geometry_msgs::msg::Point interpolatePlannedPosition(const rclcpp::Time& time);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr
        planned_traj_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ade_pub_;

    // Timer for periodic ADE computation
    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<std::pair<rclcpp::Time, geometry_msgs::msg::Point>> actual_positions_;
    trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr planned_trajectory_;

    std::mutex data_mutex_;

    double update_rate_;             // Hz
    size_t max_history_size_;
    double interpolation_tolerance_; // seconds
};

#endif
