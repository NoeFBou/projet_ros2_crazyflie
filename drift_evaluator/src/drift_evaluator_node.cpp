#include <drift_evaluator_node.hpp>

#include <cmath>

DriftEvaluatorNode::DriftEvaluatorNode() : Node("drift_evaluator_node") {
    this->declare_parameter("update_rate", 10.0);
    this->declare_parameter("max_history_size", 1000);
    this->declare_parameter("interpolation_tolerance", 0.1);

    update_rate_             = this->get_parameter("update_rate").as_double();
    max_history_size_        = this->get_parameter("max_history_size").as_int();
    interpolation_tolerance_ = this->get_parameter("interpolation_tolerance").as_double();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/crazyflie/odom",
        10,
        std::bind(&DriftEvaluatorNode::odomCallback, this, std::placeholders::_1)
    );

    planned_traj_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
        "/planned_trajectory_final",
        10,
        std::bind(&DriftEvaluatorNode::plannedTrajectoryCallback, this, std::placeholders::_1)
    );

    ade_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/drift/ade", 10);

    auto timer_period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_            = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
        std::bind(&DriftEvaluatorNode::computeAndPublishADE, this)
    );

    RCLCPP_INFO(this->get_logger(), "Drift Evaluator Node initialized");
    RCLCPP_INFO(this->get_logger(), "Update rate: %.2f Hz", update_rate_);
    RCLCPP_INFO(this->get_logger(), "Max history size: %zu", max_history_size_);
}

void DriftEvaluatorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Store the actual position with timestamp
    geometry_msgs::msg::Point position = msg->pose.pose.position;
    rclcpp::Time timestamp             = rclcpp::Time(msg->header.stamp);

    actual_positions_.push_back(std::make_pair(timestamp, position));

    // Limit the history size
    if (actual_positions_.size() > max_history_size_) {
        actual_positions_.pop_front();
    }
}

void DriftEvaluatorNode::plannedTrajectoryCallback(
    const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    planned_trajectory_ = msg;

    RCLCPP_INFO(
        this->get_logger(),
        "Received planned trajectory with %zu points",
        msg->points.size()
    );
}

geometry_msgs::msg::Point DriftEvaluatorNode::interpolatePlannedPosition(const rclcpp::Time& time) {
    geometry_msgs::msg::Point result;
    result.x = result.y = result.z = 0.0;

    if (!planned_trajectory_ || planned_trajectory_->points.empty()) {
        return result;
    }

    // Get the trajectory start time
    rclcpp::Time traj_start_time = rclcpp::Time(planned_trajectory_->header.stamp);

    // Calculate elapsed time since trajectory start
    double elapsed = (time - traj_start_time).seconds();

    // Find the two points to interpolate between
    size_t idx = 0;
    for (size_t i = 0; i < planned_trajectory_->points.size(); ++i) {
        double point_time = planned_trajectory_->points[i].time_from_start.sec
                            + planned_trajectory_->points[i].time_from_start.nanosec * 1e-9;

        if (point_time > elapsed) {
            idx = i;
            break;
        }
    }

    // Handle edge cases
    if (idx == 0) {
        // Before or at first point
        if (planned_trajectory_->points[0].transforms.empty()) {
            return result;
        }
        result.x = planned_trajectory_->points[0].transforms[0].translation.x;
        result.y = planned_trajectory_->points[0].transforms[0].translation.y;
        result.z = planned_trajectory_->points[0].transforms[0].translation.z;
        return result;
    }

    if (idx >= planned_trajectory_->points.size()) {
        // After last point
        size_t last_idx = planned_trajectory_->points.size() - 1;
        if (planned_trajectory_->points[last_idx].transforms.empty()) {
            return result;
        }
        result.x = planned_trajectory_->points[last_idx].transforms[0].translation.x;
        result.y = planned_trajectory_->points[last_idx].transforms[0].translation.y;
        result.z = planned_trajectory_->points[last_idx].transforms[0].translation.z;
        return result;
    }

    // Interpolate between idx-1 and idx
    const auto& p1 = planned_trajectory_->points[idx - 1];
    const auto& p2 = planned_trajectory_->points[idx];

    if (p1.transforms.empty() || p2.transforms.empty()) {
        return result;
    }

    double t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9;
    double t2 = p2.time_from_start.sec + p2.time_from_start.nanosec * 1e-9;

    // Linear interpolation factor
    double alpha = (elapsed - t1) / (t2 - t1);
    alpha        = std::max(0.0, std::min(1.0, alpha)); // Clamp to [0, 1]

    // Interpolate position
    result.x =
        p1.transforms[0].translation.x * (1.0 - alpha) + p2.transforms[0].translation.x * alpha;
    result.y =
        p1.transforms[0].translation.y * (1.0 - alpha) + p2.transforms[0].translation.y * alpha;
    result.z =
        p1.transforms[0].translation.z * (1.0 - alpha) + p2.transforms[0].translation.z * alpha;

    return result;
}

double DriftEvaluatorNode::computeADE() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (actual_positions_.empty() || !planned_trajectory_) {
        return 0.0;
    }

    double total_displacement = 0.0;
    size_t valid_points       = 0;

    for (const auto& [timestamp, actual_pos] : actual_positions_) {
        // Get the corresponding planned position
        geometry_msgs::msg::Point planned_pos = interpolatePlannedPosition(timestamp);

        // Compute Euclidean distance
        double dx = actual_pos.x - planned_pos.x;
        double dy = actual_pos.y - planned_pos.y;
        double dz = actual_pos.z - planned_pos.z;

        double displacement = std::sqrt(dx * dx + dy * dy + dz * dz);
        total_displacement += displacement;
        valid_points++;
    }

    if (valid_points == 0) {
        return 0.0;
    }

    return total_displacement / valid_points;
}

void DriftEvaluatorNode::computeAndPublishADE() {
    double ade = computeADE();

    // Create timestamped message
    geometry_msgs::msg::PointStamped ade_msg;
    ade_msg.header.stamp    = this->now();
    ade_msg.header.frame_id = "world";
    ade_msg.point.x         = ade;
    ade_msg.point.y         = 0.0;
    ade_msg.point.z         = 0.0;

    ade_pub_->publish(ade_msg);

    if (ade > 0.0) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Average Displacement Error: %.4f m (based on %zu points)",
            ade,
            actual_positions_.size()
        );
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriftEvaluatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
