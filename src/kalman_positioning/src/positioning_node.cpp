#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <memory>
#include <cmath>
#include "kalman_positioning/ukf.hpp"
#include "kalman_positioning/landmark_manager.hpp"
#include <Eigen/Dense>


/**
 * @brief Positioning node for UKF-based robot localization (Student Assignment)
 * 
 * This node subscribes to:
 *   - /robot_noisy: Noisy odometry (dead-reckoning)
 *   - /landmarks_observed: Noisy landmark observations
 * 
 * And publishes to:
 *   - /robot_estimated_odometry: Estimated pose and velocity from filter
 * 
 * STUDENT ASSIGNMENT:
 * Implement the Kalman filter logic to fuse odometry and landmark observations
 * to estimate the robot's true position.
 */
class PositioningNode : public rclcpp::Node {
public:
    PositioningNode() : Node("kalman_positioning_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Kalman Positioning Node");
        
        // Create subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_noisy",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::odometryCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_noisy");
        
        landmarks_obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_observed",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::landmarksObservedCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmarks_observed");
        
        // Create publisher
        estimated_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/robot_estimated_odometry", rclcpp::QoS(10)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing to /robot_estimated_odometry");
        
        RCLCPP_INFO(this->get_logger(), "Kalman Positioning Node initialized successfully");


        // ------------------------- Parameters ---------------------------------
        std::string landmarks_csv_path =
            this->declare_parameter<std::string>("landmarks_csv_path");

        
        std::string process_noise_xy_str =
            this->declare_parameter<std::string>("process_noise_xy", "1e-4");

        process_noise_xy_ = std::stod(process_noise_xy_str);

        std::string process_noise_theta_str =
            this->declare_parameter<std::string>("process_noise_theta", "1e-4");
        process_noise_theta_ = std::stod(process_noise_theta_str);

        measurement_noise_xy_ = this->declare_parameter<double>("measurement_noise_xy", 0.01);
        observation_radius_ = this->declare_parameter<double>("observation_radius", 5.0);

        RCLCPP_INFO(this->get_logger(),
                    "Using params: P_xy=%.2e, P_theta=%.2e, R_xy=%.3f, obs_radius=%.2f",
                    process_noise_xy_, process_noise_theta_,
                    measurement_noise_xy_, observation_radius_);

        // ------------------------- Landmarks ----------------------------------
        landmark_manager_.loadFromCSV(landmarks_csv_path);

        // --------------------------- UKF --------------------------------------
        // num_landmarks is not really used inside current UKF implementation, but the
        // constructor expects it.
        int num_landmarks = static_cast<int>(landmark_manager_.getLandmarks().size());

        ukf_ = std::make_unique<UKF>(
            process_noise_xy_,
            process_noise_theta_,
            measurement_noise_xy_,
            num_landmarks
        );

        // Give UKF the static landmark positions
        ukf_->setLandmarks(landmark_manager_.getLandmarks());

        filter_initialized_ = false;
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_noisy",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::odometryCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_noisy");
        
        landmarks_obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_observed",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::landmarksObservedCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmarks_observed");
        
        // Create publisher
        estimated_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/robot_estimated_odometry", rclcpp::QoS(10)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing to /robot_estimated_odometry");
        
        RCLCPP_INFO(this->get_logger(), "Kalman Positioning Node initialized successfully");

    }

private:
    // ============================================================================
    // SUBSCRIBERS AND PUBLISHERS
    // ============================================================================
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_obs_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_pub_;
    
    // ============================================================================
    // PLACEHOLDER: KALMAN FILTER STATE
    // ============================================================================
    // Students should implement a proper Kalman filter (e.g., UKF, EKF) 
    // with the following state:
    //   - Position: x, y (m)
    //   - Orientation: theta (rad)
    //   - Velocity: vx, vy (m/s)
    // And maintain:
    //   - State covariance matrix
    //   - Process noise covariance
    //   - Measurement noise covariance


    // Unscented Kalman Filter handling:
    //   x = [x, y, theta, vx, vy]^T
    //   P, Q, R are stored internally in this class
    nav_msgs::msg::Odometry last_odometry_msg_;
    std::unique_ptr<UKF> ukf_;
    LandmarkManager landmark_manager_;
    //bool filter_initialized_{false};
    //rclcpp::Time last_odom_time_;

    // configuration parameters
    double observation_radius_{5.0};
    double process_noise_xy_{1e-4};
    double process_noise_theta_{1e-4};
    double measurement_noise_xy_{0.01};

    // bookkeeping
    bool filter_initialized_{false};
    double last_odom_x_{0.0}, last_odom_y_{0.0}, last_odom_theta_{0.0};
    rclcpp::Time last_odom_stamp_;
    
    // ============================================================================
    // CALLBACK FUNCTIONS
    // ============================================================================
    
    /**
     * @brief Callback for noisy odometry measurements
     * 
     * STUDENT TODO:
     * 1. Extract position (x, y) and orientation (theta) from the message
     * 2. Update the Kalman filter's prediction step with this odometry
     * 3. Publish the estimated odometry
     */
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), 
            "Odometry received: x=%.3f, y=%.3f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        
        // STUDENT ASSIGNMENT STARTS HERE
        // ========================================================================
        
        // Placeholder: Extract and log the data
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = quaternionToYaw(msg->pose.pose.orientation);
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Parsed: x=%.3f, y=%.3f, theta=%.3f, vx=%.3f, vy=%.3f",
            x, y, theta, vx, vy);

        rclcpp::Time current_stamp(msg->header.stamp);

        if (!filter_initialized_) {
            last_odom_x_ = x;
            last_odom_y_ = y;
            last_odom_theta_ = theta;
            last_odom_stamp_ = current_stamp;
            filter_initialized_ = true;

            // --- NEW: initialize UKF state from first odom ---
            Eigen::VectorXd init_state(5);
            init_state << x, y, theta, vx, vy;
            ukf_->setState(init_state);

            RCLCPP_INFO(this->get_logger(),
                        "UKF odometry prediction initialised at x=%.3f, y=%.3f, theta=%.3f",
                        x, y, theta);

            // On the very first step we don't have a delta yet, so just publish
            // whatever the current filter state is (will typically be at origin).
            publishEstimatedOdometry(msg->header.stamp, *msg);
            return;
        }
        // Compute time step since last odometry
        
        double dt = (current_stamp - last_odom_stamp_).seconds();


        if (dt <= 0.0) {
            RCLCPP_WARN(this->get_logger(),
                        "Non-positive dt in odometryCallback (dt=%.6f), skipping predict.", dt);
            publishEstimatedOdometry(msg->header.stamp, *msg);
            return;
        }

        // 3. Compute odometry increments (delta pose) since last step
        double dx = vx * dt;
        double dy = vy * dt;
        double dtheta = normalizeAngle(theta - last_odom_theta_);

        RCLCPP_DEBUG(this->get_logger(),
                 "Odometry increments: dx=%.3f, dy=%.3f, dtheta=%.3f, dt=%.3f",
                 dx, dy, dtheta, dt);

        // 4. Call UKF prediction step with this odometry
        //    State inside UKF is [x, y, theta, vx, vy].
        ukf_->predict(dt, dx, dy, dtheta);

        // Update stored last odometry for next callback
        last_odom_x_ = x;
        last_odom_y_ = y;
        last_odom_theta_ = theta;
        last_odom_stamp_ = current_stamp;

        // ---------------------------------------------
        // NEW: build odometry from UKF state
        // ---------------------------------------------
        Eigen::VectorXd state = ukf_->getState();
        double x_f     = state(0);
        double y_f     = state(1);
        double theta_f = state(2);
        double vx_f    = state(3);
        double vy_f    = state(4);

        // Start from the incoming message as a template
        nav_msgs::msg::Odometry est_msg = *msg;

        est_msg.header.stamp = msg->header.stamp;

        // Pose
        est_msg.pose.pose.position.x = x_f;
        est_msg.pose.pose.position.y = y_f;

        tf2::Quaternion q_out;
        q_out.setRPY(0.0, 0.0, theta_f);
        est_msg.pose.pose.orientation = tf2::toMsg(q_out);

        // Twist (from UKF velocities)
        est_msg.twist.twist.linear.x = vx_f;
        est_msg.twist.twist.linear.y = vy_f;

        // Finally publish the **filtered** odometry
        last_odometry_msg_=*msg;
        publishEstimatedOdometry(msg->header.stamp, est_msg);
    }
    
    /**
     * @brief Callback for landmark observations
     * 
     * STUDENT TODO:
     * 1. Parse the PointCloud2 data to extract landmark observations
     * 2. Update the Kalman filter's measurement update step with these observations
     * 3. Optionally publish the updated estimated odometry
     */
    void landmarksObservedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // STUDENT ASSIGNMENT STARTS HERE
        // ========================================================================
        // Placeholder: Parse and log the observations
        std::vector<std::tuple<int, double, double, double>> obs;
        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_id(*msg, "id");
            
            int count = 0;
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_id) {
                int landmark_id = static_cast<int>(*iter_id);
                float obs_x = static_cast<float>(*iter_x);
                float obs_y = static_cast<float>(*iter_y);
                
                RCLCPP_DEBUG(this->get_logger(),
                    "Landmark %d observed at (%.3f, %.3f)",
                    landmark_id, obs_x, obs_y);
                
                obs.emplace_back(landmark_id, obs_x, obs_y,0.1);
                count++;
            }
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Processed %d landmark observations", count);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to parse landmark observations: %s", e.what());
        }
        if (!obs.empty()){
            ukf_->update(obs);
        }
        publishEstimatedOdometry(msg->header.stamp,last_odometry_msg_);
    }
    
    // ============================================================================
    // HELPER FUNCTIONS
    // ============================================================================
    
    /**
     * @brief Convert quaternion to yaw angle
     * @param q Quaternion from orientation
     * @return Yaw angle in radians [-pi, pi]
     */
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    /**
     * @brief Normalize angle to [-pi, pi]
     * @param angle Input angle in radians
     * @return Normalized angle in [-pi, pi]
     */
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    /**
     * @brief Publish estimated odometry message
     * @param timestamp Message timestamp
     * @param odom_msg Odometry message to publish
     */
    void publishEstimatedOdometry(const rclcpp::Time& timestamp, 
                                  const nav_msgs::msg::Odometry& odom_msg) {
        nav_msgs::msg::Odometry estimated_odom = odom_msg;
        estimated_odom.header.stamp = timestamp;
        estimated_odom.header.frame_id = "map";
        estimated_odom.child_frame_id = "robot_estimated";
        
        // STUDENT TODO: Replace this with actual filter output
        // Set position, orientation, velocity, and covariance from your Kalman filter
        // Currently using placeholder values (noisy odometry)
        
        // Get state [x, y, theta, vx, vy] from UKF
        Eigen::VectorXd state = ukf_->getState();

        // Position
        estimated_odom.pose.pose.position.x = state(0);
        estimated_odom.pose.pose.position.y = state(1);

        // Orientation (theta -> quaternion)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, state(2));
        estimated_odom.pose.pose.orientation = tf2::toMsg(q);

        // Velocity
        estimated_odom.twist.twist.linear.x = state(3);
        estimated_odom.twist.twist.linear.y = state(4);

        RCLCPP_DEBUG(this->get_logger(),
             "Publishing estimated state: x=%.3f, y=%.3f, theta=%.3f, vx=%.3f, vy=%.3f",
             state(0), state(1), state(2), state(3), state(4));

        auto cov   = ukf_->getCovariance();
        for (int i = 0; i < 36; i++) estimated_odom.pose.covariance[i] = 0.0;
        estimated_odom.pose.covariance[0] = cov(0,0);
        estimated_odom.pose.covariance[7] = cov(1,1);
        estimated_odom.pose.covariance[35] = cov(2,2);
        estimated_odom_pub_->publish(estimated_odom);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositioningNode>());
    rclcpp::shutdown();
    return 0;
}