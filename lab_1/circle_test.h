
#ifndef CIRCLE_TEST_H
#define CIRCLE_TEST_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// DECLARE YOUR CLASS HERE
class CircleTest : public rclcpp::Node{

    public:

    CircleTest();
    ~CircleTest() = default;

  void solveIK(
    double x, double y, double z,
    double qx, double qy, double qz, double qw);

  bool ikSolverInitialized() const
  {
    return ik_solver_ != nullptr;
  }

    private: 

    void robotDescriptionCallback(
        const std_msgs::msg::String& msg);

    // Robot description subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

    // Joint state publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    sensor_msgs::msg::JointState joint_state_msg_;

    // KDL objects
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    KDL::JntArray q_;


};


#endif // CIRCLE_TEST_H