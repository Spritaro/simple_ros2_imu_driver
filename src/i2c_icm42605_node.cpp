#include <chrono>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sensor_msgs/msg/imu.hpp>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>


class I2cIcm42605Node : public rclcpp::Node
{

public:
    /* constructor */
    explicit I2cIcm42605Node(
        const std::string &topic_name,
        const unsigned int period_ms,
        const std::string &device_name,
        const unsigned int address)
    : Node("i2c_icm42605_node")
    {
        /* Open I2C device */
        i2c_fd = open(device_name.c_str(), O_RDWR);
        RCLCPP_INFO(this->get_logger(), "device name %s, device no %d", device_name.c_str(), i2c_fd);
        if(i2c_fd == -1)
        {
            throw std::runtime_error("device open error");
        }
        if (ioctl(i2c_fd, I2C_SLAVE, address) < 0)
        {
            throw std::runtime_error("Failed to talk to device");
        }

        /* start IMU */
        char buffer[16];
        int length;
        // buffer[0] = 0x6b;
        // buffer[1] = 0x00;
        buffer[0] = 0x4e;
        buffer[1] = 0x0f;
        length = 2;
        if(write(i2c_fd, buffer, length) != length)
            throw std::runtime_error("Failed to start IMU");
        usleep(200);
        usleep(45 * 1000);

        /* message */
        message_ = std::make_shared<sensor_msgs::msg::Imu>();

        /* publisher */
        auto qos = rclcpp::QoS( rclcpp::KeepLast(1) );
        publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic_name, qos);

        /* timer */
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&I2cIcm42605Node::timer_callback, this)
        );
    }

private:
    /* timer event handler */
    void timer_callback()
    {
        /* read IMU */
        char buffer[32];
        int length;
    
        buffer[0] = 0x1f;   // register address
        length = 1;
        if(write(i2c_fd, buffer, length) != length)
            RCLCPP_INFO(this->get_logger(), "Failed to write");

        length = 12;     // 6 accelerometer + 6 gyroscope
        if(read(i2c_fd, buffer, length) != length)
            RCLCPP_INFO(this->get_logger(), "Failed to read");
        message_->linear_acceleration.x = static_cast<double>( static_cast<int16_t>( buffer[0] <<8 | buffer[1]  )) * 19.6 / 32768.0;
        message_->linear_acceleration.y = static_cast<double>( static_cast<int16_t>( buffer[2] <<8 | buffer[3]  )) * 19.6 / 32768.0;
        message_->linear_acceleration.z = static_cast<double>( static_cast<int16_t>( buffer[4] <<8 | buffer[5]  )) * 19.6 / 32768.0;
        message_->angular_velocity.x    = static_cast<double>( static_cast<int16_t>( buffer[6] <<8 | buffer[7]  )) * M_PI / 360.0 * 250.0 / 32768.0;
        message_->angular_velocity.y    = static_cast<double>( static_cast<int16_t>( buffer[8] <<8 | buffer[9]  )) * M_PI / 360.0 * 250.0 / 32768.0;
        message_->angular_velocity.z    = static_cast<double>( static_cast<int16_t>( buffer[10]<<8 | buffer[11] )) * M_PI / 360.0 * 250.0 / 32768.0;
        RCLCPP_INFO(this->get_logger(), "accel %f %f %f, gyro %f %f %f",
            message_->linear_acceleration.x,
            message_->linear_acceleration.y,
            message_->linear_acceleration.z,
            message_->angular_velocity.x,
            message_->angular_velocity.y,
            message_->angular_velocity.z);

        publisher_->publish(*message_);
    }

    std::shared_ptr<sensor_msgs::msg::Imu> message_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    /* create node */
    rclcpp::init(argc, argv);
    auto node = std::make_shared<I2cIcm42605Node>("imu", 50, "/dev/i2c-0", 0x68);

    /* spin node */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
