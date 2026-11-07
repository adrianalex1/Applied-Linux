#ifndef ROBOT_KIN_VALIDATION
#define ROBOT_KIN_VALIDATION

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <cmath>


class RobotKinValidation : public rclcpp::Node
{
public:

    RobotKinValidation();
    ~RobotKinValidation() = default;

    // Initialize MoveGroupInterface
    void initializeMoveGroup();

    // Solve IK and command robot
    bool solveIK(
        double x, double y, double z,
        double qx, double qy, double qz, double qw);

    // Check if KDL solver is initialized
    bool ikSolverInitialized() const
    {
        return ik_solver_ != nullptr;
    }

    // Plan and execute joint motion
    bool moveJoints();

 
    /** 
     * Prints the rotation and translation error between the end-effector and both test frames
    */
    void printErrors();


    /**
     * Print the summary for error measurements including maximum, average, trasnlation and rotational errors
    */
    void printErrorSummary();


private:

    // Robot description callback
    void robotDescriptionCallback(
        const std_msgs::msg::String& msg);

    // Robot description subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        robot_description_sub_;


    // KDL objects
    KDL::Tree tree_;
    KDL::Chain chain_;

    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

    KDL::JntArray q_;
    KDL::JntArray q_last_;

    std::vector<double> goal_angles_;

    /**
     * Wrapper function for transformLookup(). Will lookup transform between "J6" and frame_name
     * @param frame_name the name of the frame to look up
     * @param rot If the lookup is successful, this will get set to the looked up frame's rotation
     * @param tran If the lookup is successful, this will get set to the looked up frame's translation
     * @return True if successful, false otherwise
     */
    bool lookupFrame(const std::string & frame_name, Eigen::Quaterniond & rot, Eigen::Vector3d & tran);


    // Frames
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::array<std::string, 2> frame_names_;
    std::array<bool,2> lookup_success_;

    // error
    std::array<Eigen::Quaterniond,2 > quat_;
    std::array<Eigen::Vector3d,2> pos_;
    std::vector<double> error_trans_;
    std::vector<double> error_rot_;
    double max_rot_;
    double max_trans_;
    double avg_rot_;
    double avg_trans_;

    // MoveIt
    moveit::planning_interface::MoveGroupInterface::Plan plan_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
        move_group_;


    

};

#endif // ROBOT_KIN_VALIDATION