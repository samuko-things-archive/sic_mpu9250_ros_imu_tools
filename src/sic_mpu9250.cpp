#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sic_mpu9250/sic_cppserial_lib.hpp"


void delay_ms(unsigned long milliseconds) {
  usleep(milliseconds*1000);
}

float MicroTeslaToTesla(float mT)
{
  return mT * 1000000;
}


class SicMPU9250Node : public rclcpp::Node
{
public:
    SicMPU9250Node() : Node("sic_mpu9250")
    {
      /*---------------node parameter declaration-----------------------------*/
      this->declare_parameter<std::string>("frame_id", "imu");
      this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
      this->declare_parameter<double>("publish_frequency", 10.0);

      frame_id = this->get_parameter("frame_id").as_string();
      RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());

      port = this->get_parameter("port").as_string();
      RCLCPP_INFO(this->get_logger(), "port: %s", port.c_str());

      publish_frequency = this->get_parameter("publish_frequency").as_double();
      RCLCPP_INFO(this->get_logger(), "publish_frequency: %f", publish_frequency);
      /*---------------------------------------------------------------------*/


      /*----------start connection to sic_mpu9250_driver module---------------*/
      sic_mpu9250.connect(port);
      // wait for the imu to fully setup
      for (int i=1; i<=5; i+=1){ 
        delay_ms(1000);
        RCLCPP_INFO(this->get_logger(), "%d", i);
      }
      /*---------------------------------------------------------------------*/


      /*----------initialize IMU message---------------*/
      messageMag.header.frame_id = frame_id;
      messageImu.header.frame_id = frame_id;

      sic_mpu9250.getGyroVariance(data_x, data_y, data_z);
      messageImu.angular_velocity_covariance = { data_x, 0.0, 0.0, 0.0, data_y, 0.0, 0.0, 0.0, data_z };

      sic_mpu9250.getAccVariance(data_x, data_y, data_z);
      messageImu.linear_acceleration_covariance = { data_x, 0.0, 0.0, 0.0, data_y, 0.0, 0.0, 0.0, data_z };
      /*---------------------------------------------------------------------*/


      /*---------------start imu and mag publishers and timer-----------------------------*/
      imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
      mag_data_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);

      timer_ = this->create_wall_timer(
        std::chrono::microseconds((long)(1000000/publish_frequency)),
        std::bind(&SicMPU9250Node::publish_imu_mag_callback, this));
      /*---------------------------------------------------------------------*/

      RCLCPP_INFO(this->get_logger(), "sic_mpu9250 node has started");
    }

private:
  void publish_imu_mag_callback(){  
    messageImu.header.stamp = rclcpp::Clock().now();    
    messageMag.header.stamp = messageImu.header.stamp;

    sic_mpu9250.getAcc(data_x, data_y, data_z);
    messageImu.linear_acceleration.x = data_x;
    messageImu.linear_acceleration.y = data_y;
    messageImu.linear_acceleration.z = data_z;

    sic_mpu9250.getGyro(data_x, data_y, data_z);
    messageImu.angular_velocity.x = data_x;
    messageImu.angular_velocity.y = data_y;
    messageImu.angular_velocity.z = data_z;

    sic_mpu9250.getMag(data_x, data_y, data_z);
    messageMag.magnetic_field.x = MicroTeslaToTesla(data_x);
    messageMag.magnetic_field.y = MicroTeslaToTesla(data_y);
    messageMag.magnetic_field.z = MicroTeslaToTesla(data_z);

    imu_data_publisher_->publish(messageImu);
    mag_data_publisher_->publish(messageMag);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_data_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Imu messageImu = sensor_msgs::msg::Imu();
  sensor_msgs::msg::MagneticField messageMag = sensor_msgs::msg::MagneticField();

  std::string frame_id;
  std::string port;
  double publish_frequency;

  SIC sic_mpu9250;
  float data_x, data_y, data_z;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SicMPU9250Node>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
