#include <mass_spring_damper/force_publisher.h>
#include <cmath>
#include <numbers>


    Force_Publisher :: Force_Publisher(float angular_frequency, float amplitude) : Node("pub_node"), angular_frequency_(angular_frequency), amplitude_(amplitude) {
        
    // initialize shared ptr and time object
    publisher_ = this->create_publisher<mass_spring_damper::msg::FloatStamped>("/force", 10);
    time_start_ = this->get_clock()->now();
    
    }

    // define publishData()
    void Force_Publisher :: publishData(){
        float pi = 3.141592653589793;
        // find f(t)
        double delta_t = (this->get_clock()->now() - time_start_).seconds();
        // get value and time associated with it
        msg_.value = amplitude_ * std :: sin(2 * pi * angular_frequency_ * delta_t);
        msg_.header.stamp = this->get_clock()->now();
        publisher_->publish(msg_);

    }


    // main function
    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);

    // set arguments equal to constructor params
    float angular_frequency = std::stof(argv[1]);
    float amplitude = std::stof(argv[2]);
    
    auto node = std::make_shared<Force_Publisher>(angular_frequency, amplitude);

    rclcpp::Rate rate = rclcpp::Rate(100);

    while (rclcpp::ok()) {
        node->publishData();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}