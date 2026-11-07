#include <mass_spring_damper/mass_spring_damper_node.h>
#include <iostream>
#include<Eigen/Dense>

    Mass_Spring_Damper_Node :: Mass_Spring_Damper_Node(float mass, float spring_const, float damper_const) : Node("sub_node"), mass_(mass), spring_const_(spring_const), damper_const_(damper_const){

        //initialize shared ptr sub
        subscriber_ = this->create_subscription<mass_spring_damper::msg::FloatStamped>("/force", 10, std::bind(&Mass_Spring_Damper_Node::messageCallback, this, std::placeholders::_1));
        //initialize shared ptr pub
        publisher_ = this->create_publisher<mass_spring_damper::msg::MassStateStamped>("/mass_state", 10);
        y_ << 0, 0;
        A_ << 0, 1, -spring_const_/mass_, -damper_const_/mass_;
        B_ << 0, 1/mass_;
        first_ = true;
        previous_time_ = this->get_clock()->now();
        //start_time_ = 
    }

    void Mass_Spring_Damper_Node :: publishData(){
        msg_.position = y_(0);
        msg_.velocity = y_(1);
        msg_.header.stamp = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Position: %0.6f Velcotiy: %0.6f", msg_.position, msg_.velocity);
        publisher_->publish(msg_);
    }

    void Mass_Spring_Damper_Node::messageCallback(const mass_spring_damper::msg::FloatStamped & msg){

        rclcpp::Time current_time(msg.header.stamp);
        Eigen::Matrix<float, 2, 1>x_dot_;
        //if (first_){
        //    previous_time_ = current_time;
            first_ = false;
        //}

        float f = msg.value;

        double dt = (current_time - previous_time_).seconds();

        x_dot_ = A_ * y_ + B_ * f;

        // Euler integration
        y_ = y_ + x_dot_ * dt;

        // Current becomes previous for next callback
        previous_time_ = current_time;

        this->publishData();
    }

    
    // MAIN FUNCTION
    int main(int argc, char * argv[])
    {

        rclcpp::init(argc, argv);

        // set arguments equal to constructor params
        float mass = std::stof(argv[1]);
        float spring_const = std::stof(argv[2]);
        float damper_const = std::stof(argv[3]);
        auto node = std::make_shared<Mass_Spring_Damper_Node>(mass, spring_const, damper_const);
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }