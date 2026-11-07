#ifndef FORCE_PUBLISHER
#define FORCE_PUBLISHER

// PUT INCLUDES HERE 
#include <rclcpp/rclcpp.hpp>
#include <mass_spring_damper/msg/float_stamped.hpp>


// DEFINE CLASS HERE
class Force_Publisher : public rclcpp :: Node{

    public:
        // constructor
        Force_Publisher(float angular_frequency, float amplitude);
        // destructor
        ~Force_Publisher() = default;
        void publishData();
    
    private:
        // make sharedptr for publishers
        rclcpp::Publisher<mass_spring_damper::msg::FloatStamped>::SharedPtr publisher_;
        // make msg for force
        mass_spring_damper::msg::FloatStamped msg_;
        // track time start
        rclcpp::Time time_start_;
        // BMV
        float angular_frequency_;
        float amplitude_;

}; 


#endif // FORCE_PUBLISHER