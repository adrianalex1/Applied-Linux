#ifndef WORKPIECE_REG
#define WORKPIECE_REG

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <array>
#include <vector>
#include <memory>
#include <cstdint>
#include <std_msgs/msg/string.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

class WorkpieceReg : public rclcpp :: Node
{

    static constexpr uint8_t N_ = 5; // Number of points 

    public:


        WorkpieceReg();
        ~WorkpieceReg() = default;

        // Solve IK and command robot
        bool solveIK(
            double x, double y, double z,
            double qx, double qy, double qz, double qw);

        // Check if KDL solver is initialized
        bool ikSolverInitialized() const
        {
            return ik_solver_ != nullptr;
        }

        /**
        * Computes the Arun registration
        */
        void registerPoints();

        /**
        * Sets up the move group. Must call before moveToNextPose()
        */
        void initializeMoveGroup();

        /**
        * Moves joints to the starting angles 
        */
        bool moveToStart();
        
        /**
        * Main telemanipulation function. Publishes the twist 
        */
        void mainTelemanipulation();

        /**
        * Records and adds a point to the list of points in robot and workspace frame
        */ 
        void addPoint();


    private:

        /**
        * Computes the centroid of a list of points
        * @param points Array of points to compute centroid of
        * @return The centroid 
        */
        Eigen::Vector3d computeCentroid(std::array<Eigen::Vector3d,N_> & points);

        /**
        * Computes the H matrix from Arun registration
        */
        void createH();

        /**
        * Computes the Fiducial Registration Error (FRE)
        * @return The FRE 
        */
        double computeFRE();

        /**
        * Computes the Target Registration Error (TRE)
        * @return The TRE 
        */
        double computeTRE();

        /**
        * Print the rotation matrix, TRE, and FRE
        */
        void printResults();


        std::array<Eigen::Vector3d,N_> p_w_; // points in workpiece frame
        std::array<Eigen::Vector3d,N_> p_r_; // points in robot frame

        std::array<Eigen::Vector3d,N_> q_w_; // points in workpiece frame
        std::array<Eigen::Vector3d,N_> q_r_; // points in robot frame

        Eigen::Matrix3d H_ = Eigen::Matrix3d::Zero(); // H Matrix from Arun Reg
        Eigen::Matrix3d R_rw_ = Eigen::Matrix3d::Zero(); // the rotation matrix of {w} wrt {r}

        Eigen::Vector3d p_rw_; // The position of {w} wrt {r}
        Eigen::Vector3d p_r_new_; // New point for TRE in {r}
        Eigen::Vector3d p_w_new_; // New point for TRE in {w}

        /**
        * Wrapper function for transformLookup(). Will lookup transform between world and end_effector frame name
        */
        void lookupFrame();

        // Robot description callback
        void robotDescriptionCallback(const std_msgs::msg::String& msg);

        // Robot description subscriber
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

        // KDL objects
        KDL::Tree tree_;
        KDL::Chain chain_;

        std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
        KDL::JntArray q_;
        KDL::JntArray q_init_;


        /**
        * Callback for the /joy topic. converts the joystick readings to a twist
        * @param msg the message on the /joy topic
        */
        void joystickCallback(const sensor_msgs::msg::Joy & msg);

        // subscriber for the joystick
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        // the previous value of the A button (prevents multiple registers per press)
        bool a_button_prev_ = false;

        int point_count_ = 0;

        // Frames
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // twist message to publish
        geometry_msgs::msg::TwistStamped twist_msg_;

        // publisher for twist
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;

        // gains
        double lin_gain_ = 0.1; // 100 mm/s max velocity
        double ang_gain_ = 30*M_PI/180.0; // 10 deg/s max velocity

        // MoveIt
        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        // Hard coded start angles to move the end effector down
        std::vector<double> goal_angles_;
        
    
};

#endif // WORKPIECE_REG