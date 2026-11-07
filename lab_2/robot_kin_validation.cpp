#include <robot_kin_validation/robot_kin_validation.h>



RobotKinValidation::RobotKinValidation() : Node("robot_kin_validation")
{
    // Subscribe to robot description
    robot_description_sub_ =
        this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(rclcpp::KeepLast(1))
                .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
            std::bind(
                &RobotKinValidation::robotDescriptionCallback,
                this,
                std::placeholders::_1));

    // Setup TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}


// Construct KDL chain and initialize IK solver
void RobotKinValidation::robotDescriptionCallback(
    const std_msgs::msg::String& msg)
{
    // Construct KDL tree from URDF
    const std::string urdf = msg.data;

    if (!kdl_parser::treeFromString(urdf, tree_))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "KDL tree construction failed");

        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "KDL tree created successfully");

    // Extract kinematic chain
    if (!tree_.getChain("base_link", "J6", chain_))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to extract KDL chain");

        return;
    }

    // Verify six joints
    if (chain_.getNrOfJoints() != 6)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Expected 6 joints, found %u",
            chain_.getNrOfJoints());

        return;
    }
    
    // Initialize joint arrays
    q_ = KDL::JntArray(chain_.getNrOfJoints());

    q_last_ = KDL::JntArray(chain_.getNrOfJoints());

    //set const guesses
    q_last_(0) = 0.0;
    q_last_(1) = 0.0;
    q_last_(2) = 0.0;
    q_last_(3) = 0.0;
    q_last_(4) = 0.0;
    q_last_(5) = 0.0;

    // Initialize IK solver
    ik_solver_ =
        std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);

    // Resize joint target vector
    goal_angles_.resize(chain_.getNrOfJoints());

    RCLCPP_INFO(
        this->get_logger(),
        "IK solver initialized");
}


// Initialize MoveGroupInterface 
void RobotKinValidation::initializeMoveGroup()
{
    // arm maps to denso robot srdf file
    move_group_ =
        std::make_shared<
            moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), //needs to happen outside constructor
                "arm");
}



// Plan and execute joint motion
bool RobotKinValidation::moveJoints()
{

    move_group_->setMaxVelocityScalingFactor(0.05);
    move_group_->setMaxAccelerationScalingFactor(0.05);

    // Set MoveIt joint target
    if (!move_group_->setJointValueTarget(goal_angles_))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Invalid joint target");

        return false;
    }

    // Plan joint-space trajectory
    if (move_group_->plan(plan_) !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Planning failed");

        return false;
    }

    // Execute trajectory
    if (move_group_->execute(plan_) !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Execution failed");

        return false;
    }

    return true;
}


// Solve inverse kinematics for desired end-effector pose
bool RobotKinValidation::solveIK(
    double x, double y, double z,
    double qx, double qy, double qz, double qw)
{
    // Check if IK solver is initialized
    if (ik_solver_ == nullptr)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "IK solver not initialized");

        return false;
    }

    // Use previous successful joint solution as initial guess
    KDL::JntArray q_init = q_last_;

    // Desired end-effector position
    KDL::Vector kdl_position(x, y, z);

    // Desired end-effector orientation
    KDL::Rotation kdl_rotation =
        KDL::Rotation::Quaternion(
            qx, qy, qz, qw);

    // Construct desired end-effector frame
    KDL::Frame desired_ee_frame(
        kdl_rotation,
        kdl_position);

    // Solve IK
    int error =
        ik_solver_->CartToJnt(
            q_init,
            desired_ee_frame,
            q_);

    // Check IK result, it looks for a sepcfiic type of error
    if (error != KDL::ChainIkSolverPos_LMA::E_NOERROR)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Inverse kinematics failed: %d",
            error);

        return false;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Inverse kinematics successful");

    // Copy KDL joint solution into MoveIt joint target
    std::copy(
        q_.data.begin(),
        q_.data.end(),
        goal_angles_.begin());

    // Now we have goal angles plan and execute motion
    if (!moveJoints())
    {
        return false;
    }

    // compute error after reaching waypoint (imma get the circle working first tho)
    printErrors();

    // Save successful solution for next waypoint
    q_last_ = q_;

    return true;
}

bool RobotKinValidation::lookupFrame(
    const std::string & frame_name,
    Eigen::Quaterniond & rot,
    Eigen::Vector3d & tran)
{
    try // try catch block so program can still run if lookup fails
    {
        // Look up transform from J6 to the requested frame
        auto transform = tf_buffer_->lookupTransform(
            "J6",
            frame_name,
            tf2::TimePointZero); // get latest avaiable transform between J6 and frame_name

        // Extract translation
        tran = Eigen::Vector3d(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        // Extract rotation
        rot = Eigen::Quaterniond(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z);

        return true;
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not lookup transform: %s",
            ex.what());

        return false;
    }
}


void RobotKinValidation::printErrors()
{
    // Look up transform between J6 and measured_ee
    Eigen::Quaterniond quat;
    Eigen::Vector3d pos;

    bool lookup_success = lookupFrame("measured_ee", quat, pos);

    // Only calculate errors if lookup succeeded
    if (lookup_success)
    {
        // Convert quaternion to axis-angle
        Eigen::AngleAxisd ax_ang(quat.normalized());

        // Rotation error (degrees)
        double rot_error = ax_ang.angle() * 180.0 / M_PI;

        // Translation error (meters)
        double trans_error = pos.norm();

        // Store errors for each waypoint
        error_rot_.push_back(rot_error);
        error_trans_.push_back(trans_error);

        // Print errors
        RCLCPP_INFO(this->get_logger(),
            "Rotation Error: %0.5f deg", rot_error);

        RCLCPP_INFO(this->get_logger(),
            "Translation Error: %0.5f m", trans_error);

           
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
            "Failed to retrieve end-effector transform");
    }
}

void RobotKinValidation::printErrorSummary()
{
    // have error measurements
    if (error_trans_.empty() || error_rot_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No error measurements recorded");
        return;
    }

    // Calculate average translational error
    double avg_trans = std::accumulate(
        error_trans_.begin(), error_trans_.end(), 0.0)
        / error_trans_.size();

    // Calculate maximum translational error
    double max_trans = *std::max_element(
        error_trans_.begin(), error_trans_.end());

    // Calculate average rotational error
    double avg_rot = std::accumulate(
        error_rot_.begin(), error_rot_.end(), 0.0)
        / error_rot_.size();

    // Calculate maximum rotational error
    double max_rot = *std::max_element(
        error_rot_.begin(), error_rot_.end());

    // Print results
    RCLCPP_INFO(this->get_logger(), "===== ERROR SUMMARY =====");
    RCLCPP_INFO(this->get_logger(), "Measurements: %zu", error_trans_.size());
    RCLCPP_INFO(this->get_logger(), "Average Translation Error: %.5f m", avg_trans);
    RCLCPP_INFO(this->get_logger(), "Maximum Translation Error: %.5f m", max_trans);
    RCLCPP_INFO(this->get_logger(), "Average Rotation Error: %.5f deg", avg_rot);
    RCLCPP_INFO(this->get_logger(), "Maximum Rotation Error: %.5f deg", max_rot);
}



// Main function
int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<RobotKinValidation>();

    // Wait for robot description to initialize KDL solver
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // breaks once IK solver initialized (no longer a null ptr)
        if (node->ikSolverInitialized())
        {
            break;
        }
    }

    // rcpp:node might have shut down in previous while loop termination
    if (!rclcpp::ok())
    {
        rclcpp::shutdown();
        return 1;
    }

    // Initialize MoveGroupInterface
    node->initializeMoveGroup();

    // pi
    const double pi = 3.14159265358979323846;


    // Execute 72 waypoints, increment angle by 5, and all while node is running
    for (int angle_deg = 0;
         angle_deg < 360 && rclcpp::ok();
         angle_deg += 5)
    {
        // Convert degrees to radians
        double theta = angle_deg * pi / 180.0;

        // circle in YZ plane
        double x = 0.35;
        double y = 0.1 * std::cos(theta);
        double z = 0.35 + 0.1 * std::sin(theta);


        // Solve IK and execute motion
        bool success = node->solveIK(
            x, y, z,
            0.0, 0.7071, 0.0, 0.7071); //w constant quaternion params

        if (!success)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Failed at waypoint %d degrees",
                angle_deg);

            break;
        }

        // Process pending ROS callbacks
        rclcpp::spin_some(node);
    }

    node->printErrorSummary();
    rclcpp::shutdown();

    return 0;
}