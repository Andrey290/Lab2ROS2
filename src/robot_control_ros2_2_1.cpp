#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals; // that allows to use hndly 100ms notation

// we inherit RobotControl fron basic class Node
class RobotControl : public rclcpp::Node {
	public:
		// parent class constructor
		RobotControl() : Node("control_node") {
			// initialisation of subscribers
			laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>( // msg type
                             "base_scan", // data source (topic) name (lidar posts here) 
			     10, // queue size
                             std::bind(&RobotControl::laserCallback, this, std::placeholders::_1)
			     // ^ binding laserCallback method to subscriber (this) object
			);
			pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                            "odom",
			    10,
                            std::bind(&RobotControl::poseCallback, this, std::placeholders::_1)
			);
			// and publisher
			cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
			    "cmd_vel",
			    10
			);
                        // Таймер 10 Hz
                        timer_ = create_wall_timer(
                            100ms,
                            std::bind(&RobotControl::timerCallback, this)
			);
		}
	private:
                void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                	RCLCPP_INFO(get_logger(), "(^0w0=^)Laser msg: %f", msg->scan_time);
        
        		const double kMinRange = 0.55;
        		obstacle_ = false;

			size_t start_index = msg->ranges.size() * 0.35;
			size_t end_index   = msg->ranges.size() * 0.65;

        		for (size_t i = start_index; i < end_index; i++) {
            			if (msg->ranges[i] < kMinRange && !std::isinf(msg->ranges[i])) {
                			obstacle_ = true;
                			RCLCPP_INFO(get_logger(), "OBSTACLE DETECTED at ray %zu: distance = %f", i, msg->ranges[i]);
                			break;
            			}
        		}
    		}

    		void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        		auto orientation = msg->pose.pose.orientation;
        		RCLCPP_DEBUG(get_logger(), 
            		"Pose msg: x = %f y = %f theta = %f",
            		msg->pose.pose.position.x,
            		msg->pose.pose.position.y,
            		2 * atan2(orientation.z, orientation.w));
    		}

    		void timerCallback() {
        	    	static int counter = 0;
        		counter++;
        		RCLCPP_INFO(get_logger(), "on timer %d", counter);
			
			auto cmd = geometry_msgs::msg::Twist();
        
        		if (!obstacle_) {
            			RCLCPP_INFO(get_logger(), "go forward");
            			cmd.linear.x = 0.5;
            			cmd.angular.z = 0.0;
        		} else {
            			RCLCPP_INFO(get_logger(), "Spin around!");
            			cmd.linear.x = 0.0;
            			cmd.angular.z = 0.5;
        		}
        
        		cmd_pub_->publish(cmd);
    		}
	// Члены класса
    	bool obstacle_ = false;
    	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControl>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
