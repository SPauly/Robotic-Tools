#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class PointApproach : public rclcpp::Node {
 public:
  PointApproach() : Node("PointApproach") {
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "laserscan", 20,
        std::bind(&PointApproach::LaserCallback, this, std::placeholders::_1));
    amcl_subscriber_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", 100,
        std::bind(&PointApproach::ProcessAmclCB, this, std::placeholders::_1));
    command_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                     std::bind(&PointApproach::mainLoop, this));
  }

 private:
  enum class follow_mode { HitWall, Following, FindWall, SharpLeft };

  struct BasicPos {
    double x, y, z;
    double alpha;
  };

  void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_data) {
    laserscan_ = *scan_data;
  }

  void ProcessAmclCB(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_msg) {
    last_amcl_pos_.x = amcl_msg->pose.pose.position.x;
    last_amcl_pos_.y = amcl_msg->pose.pose.position.y;
    last_amcl_pos_.z = 0.0;

    auto orientation = amcl_msg->pose.pose.orientation;
    tf2::Quaternion tf_quat;
    tf2::fromMsg(orientation, tf_quat);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    last_amcl_pos_.alpha = yaw;
  }

  void readLaser() {
    // No-op: laserscan_ is updated in callback
  }

  void emergencyStop() {
    int danger = 0;
    if (!laserscan_.ranges.empty()) {
      for (auto r : laserscan_.ranges) {
        if (r <= emergency_dist_) danger = 1;
      }
    }
    if (danger == 1) {
      roomba_command_.linear.x = 0.0;
      roomba_command_.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Robot halted by emergency stop");
    }
  }

  void DetermineMode() {
    mode_ = follow_mode::Following;
    if (laserscan_.ranges.size() > 450 &&
        laserscan_.ranges[450] > dist_from_wall + 1.0)
      mode_ = follow_mode::FindWall;
    if (laserscan_.ranges.size() > 320 &&
        Average(267, 320) < dist_from_wall + 0.1)
      mode_ = follow_mode::HitWall;
  }

  double Average(int x, int y) {
    if (laserscan_.ranges.size() <= (size_t)y) return 0.0;
    double avg = 0;
    for (int i = 0; i < (y - x); i++) {
      avg += laserscan_.ranges[x + i];
    }
    avg /= (double)(y - x);
    return avg;
  }

  void ExecuteWallFollow() {
    if (!laserscan_.ranges.empty()) {
      DetermineMode();
      RCLCPP_INFO(this->get_logger(), "Mode: %d", (int)mode_);
      switch (mode_) {
        case follow_mode::FindWall:
          roomba_command_.linear.x =
              robot_fw_speed_ * std::tanh(Average(267, 273) - 0.7);
          roomba_command_.angular.z = 0.1;
          break;
        case follow_mode::Following:
          roomba_command_.angular.z =
              rota_speed_ *
              std::tanh(rotation_scalar *
                        (laserscan_.ranges[449] - laserscan_.ranges[451]));
          roomba_command_.linear.x =
              robot_fw_speed_ * std::tanh(Average(267, 273) - 0.7);
          roomba_command_.angular.z +=
              rota_speed_ / 2 *
              std::tanh(rotation_scalar * (Average(447, 453) - dist_from_wall));
          break;
        case follow_mode::HitWall:
          roomba_command_.angular.z = -2 * rota_speed_;
          break;
        default:
          break;
      }
    }
  }

  void mainLoop() {
    readLaser();
    RCLCPP_INFO(this->get_logger(),
                "robo_03_00 commands: forward speed=%f [m/sec] and turn "
                "speed=%f [rad/sec]",
                roomba_command_.linear.x, roomba_command_.angular.z);
    RCLCPP_INFO(this->get_logger(),
                "robo_03_00 amcl position: x=%f and y=%f alpha=%f",
                last_amcl_pos_.x, last_amcl_pos_.y, last_amcl_pos_.alpha);

    emergencyStop();
    ExecuteWallFollow();
    command_publisher_->publish(roomba_command_);
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      amcl_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan laserscan_;
  geometry_msgs::msg::Twist roomba_command_;
  BasicPos last_amcl_pos_;

  static constexpr double emergency_dist_ = 0.4;
  static constexpr double dist_from_wall = 0.8;
  double robot_fw_speed_ = 0.2, rota_speed_ = .3, rotation_scalar = 10.0;
  bool found = false;
  follow_mode mode_ = follow_mode::FindWall;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointApproach>());
  rclcpp::shutdown();
  return 0;
}