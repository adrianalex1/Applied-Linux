#ifndef MASS_SPRING_DAMPER_NODE
#define MASS_SPRING_DAMPER_NODE

// PUT INCLUDES HERE
#include <rclcpp/rclcpp.hpp>
#include <mass_spring_damper/msg/float_stamped.hpp>
#include <mass_spring_damper/msg/mass_state_stamped.hpp>
#include<Eigen/Dense>


class Mass_Spring_Damper_Node : public rclcpp :: Node {

    public:

        Mass_Spring_Damper_Node(float mass, float spring_const, float damper_const);
        ~Mass_Spring_Damper_Node() = default;
        void publishData();

    private:

        void messageCallback(const mass_spring_damper::msg::FloatStamped & msg); 
        // subscriber pointer
        rclcpp::Subscription<mass_spring_damper::msg::FloatStamped>::SharedPtr subscriber_;
        // publihser ptr and correspnding msg
        rclcpp::Publisher<mass_spring_damper::msg::MassStateStamped>::SharedPtr publisher_;
        mass_spring_damper::msg::MassStateStamped msg_;
        rclcpp::Time previous_time_;
        //rclcpp::Time start_time_;
        float mass_;
        float spring_const_;
        float damper_const_;
        Eigen::Matrix<float, 2, 2>A_;
        Eigen::Matrix<float, 2, 1>B_;
        // z and z_dot in a matrix
        Eigen::Matrix<float, 2, 1> y_;
        bool first_;
        

};

#endif // MASS_SPRING_DAMPER_NODE