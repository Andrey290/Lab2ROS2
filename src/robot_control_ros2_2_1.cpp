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
        
        		obstacle_ = false;

		        //size_t start_index = msg->ranges.size() * 0.35;
			//size_t end_index   = msg->ranges.size() * 0.65;

			// PID parameters //
			err = 7 - msg->ranges[0];
			// P = setpoint - input
			P = err;
			// I = I + (setpoint - input) * dt
        		I += err * 0.1; // 100ms is 0.1s
		        // D = (err - prev_err) / dt
		        D = (err - prev_err) * 10; // *10 is cheaper than /0.1
                        //
			prev_err = err;

			// Correction on this step
			cor = P * kP + I * kI + D * kD;
		}

    		void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        		auto orientation = msg->pose.pose.orientation;
        		
			double current_x = msg->pose.pose.position.x;
			double current_y = msg->pose.pose.position.y;
			double current_theta = 2 * atan2(orientation.z, orientation.w); 
			
			RCLCPP_INFO(get_logger(), 
            		"Pose msg: x = %f y = %f theta = %f",
            		current_x, current_y, current_theta);

			double next_x = current_x + v * 0.1;
			double next_y = current_y + cor;
			rotation = atan2(next_y - current_y , next_x - current_x) - current_theta;
    		}

    		void timerCallback() {
        	    	static int counter = 0;
        		counter++;
        		RCLCPP_INFO(get_logger(), "on timer %d", counter);
			
			auto cmd = geometry_msgs::msg::Twist(); 
            		
			RCLCPP_INFO(get_logger(), "go go go!");
            		cmd.angular.z = rotation;
            		cmd.linear.x = v;
        
        		cmd_pub_->publish(cmd);
    		}
	// Члены класса
	const double kP = 1;
	const double kI = 0;
	const double kD = 0;
	double P = 0;
	double D = 0;
	double I = 0;
	double err = 0;
	double prev_err = 0;
	double cor = 0;
	const double v = 0.5;
	double rotation = 0.0;
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
