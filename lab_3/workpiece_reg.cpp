#include <workpiece_reg/workpiece_reg.h>

WorkpieceReg :: WorkpieceReg() : Node("WorkpieceReg")
{
    // setup subscriber for robot description
    robot_description_sub_ =
        this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(rclcpp::KeepLast(1))
                .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
            std::bind(
                &WorkpieceReg::robotDescriptionCallback,
                this,
                std::placeholders::_1));


    // setup joy node subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 
        10,
        std::bind(&WorkpieceReg::joystickCallback, this, std::placeholders::_1));

    // setup publisher for twist commands
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);

    // Setup frame
    twist_msg_.header.frame_id = "base_link";

    // setup transform buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Known target locations in workpiece frame
    p_w_[0] << -0.165, -0.095, 0.0; // Point 1
    p_w_[1] <<  0.165, -0.095, 0.0; // Point 2
    p_w_[2] << -0.165,  0.095, 0.0; // Point 3
    p_w_[3] <<  0.165,  0.095, 0.0; // Point 4
    p_w_[4] << -0.090,  0.000, 0.0; // Point 5
}

void WorkpieceReg::robotDescriptionCallback(const std_msgs::msg::String& msg)
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

    q_init_ = KDL::JntArray(chain_.getNrOfJoints());

    //set const guesses
    q_init_(0) = 0.0;
    q_init_(1) = 0.0;
    q_init_(2) = 0.0;
    q_init_(3) = 0.0;
    q_init_(4) = 0.0;
    q_init_(5) = 0.0;

    // Initialize IK solver
    ik_solver_ =
        std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);

    // Resize joint target vector
    goal_angles_.resize(chain_.getNrOfJoints());

    RCLCPP_INFO(
        this->get_logger(),
        "IK solver initialized");
}

// Solve inverse kinematics for desired end-effector pose
bool WorkpieceReg::solveIK(
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
    KDL::JntArray q_init = q_init_;

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
    if (!moveToStart())
    {
        return false;
    }

    // Save successful solution for next waypoint
    q_init_ = q_;

    return true;
}

// Plan and execute joint motion
bool WorkpieceReg::moveToStart()
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

void WorkpieceReg::registerPoints()
{
    Eigen::Vector3d w_centroid = computeCentroid(p_w_);
    Eigen::Vector3d r_centroid = computeCentroid(p_r_);

    for (int i = 0; i< N_ - 1; ++i)
    {
        q_w_[i] = p_w_[i] -  w_centroid;
        q_r_[i] = p_r_[i] -  r_centroid;
    }

    createH();

    // compute the SVD of H
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d U_transpose = svd.matrixU().transpose();
    
    R_rw_ = V*U_transpose;

    // compute determinant of R
    double det_R = R_rw_.determinant();

    if (std::abs(det_R- 1) < 0.001) // det(R) = +1
    {
        // position of {w} wrt {r}
        p_rw_ = r_centroid - R_rw_*w_centroid;

        printResults(); 
    }
    else if (std::abs(det_R + 1) < 0.001) // det(R) = -1
    {
        // Invert last column of V
        V.col(2) = -V.col(2);

        // Reconstruct R
        R_rw_ = V*U_transpose;

        // position of {w} wrt {r}
        p_rw_ = r_centroid - R_rw_*w_centroid;

        RCLCPP_WARN(this->get_logger(), "Reflection Detected!");

        printResults(); 
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Determinate is incorrect: %0.4f", det_R);
    }
}

Eigen::Vector3d WorkpieceReg::computeCentroid(std::array<Eigen::Vector3d,WorkpieceReg::N_> & points)
{
    // int N = points.length();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i< N_ - 1; ++i)
    {
        centroid = centroid + points[i];
    }
    return centroid/(N_ - 1);
}

void WorkpieceReg::createH()
{
    H_.setZero();
    // int N = points.length();
    for (int i = 0; i< N_ - 1; ++i)
    {
        H_ = H_ + q_w_[i]*(q_r_[i].transpose());
    }
}

double WorkpieceReg::computeFRE()
{
    // int N = points.length();
    double FRE = 0;
    for (int i = 0; i< N_ - 1; ++i)
    {
        FRE = FRE + std::pow((p_r_[i] - (R_rw_*p_w_[i] + p_rw_)).norm(), 2);
    }
    return std::sqrt(FRE/(N_ - 1));
}

double WorkpieceReg::computeTRE()
{
    return (p_r_[4] - (R_rw_ * p_w_[4] + p_rw_)).norm();
}

void WorkpieceReg::printResults()
{
    std::ostringstream oss_r;
    oss_r << R_rw_;

    std::ostringstream oss_p;
    oss_p << p_rw_;

    RCLCPP_INFO(this->get_logger(), 
                "R_rw:\n%s", 
                oss_r.str().c_str());

    RCLCPP_INFO(this->get_logger(), 
                "p_rw:\n%s", 
                oss_p.str().c_str());

    RCLCPP_INFO(this->get_logger(), 
                "FRE: %0.4f, TRE: %0.4f", 
                computeFRE(), computeTRE());
}

void WorkpieceReg::initializeMoveGroup()
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");
}


void WorkpieceReg::mainTelemanipulation()
{
    twist_msg_.header.stamp = this->get_clock()->now();

    twist_publisher_->publish(twist_msg_);
}

void WorkpieceReg::joystickCallback(const sensor_msgs::msg::Joy & msg)
{
    // Move robot in XY plane
    twist_msg_.twist.linear.x = lin_gain_ * msg.axes[0];
    twist_msg_.twist.linear.y = -lin_gain_ * msg.axes[1];
    twist_msg_.twist.linear.z = 0;

    // No rotation
    twist_msg_.twist.angular.x = 0;
    twist_msg_.twist.angular.y = 0;
    twist_msg_.twist.angular.z = 0;

    // Only record once per press of A
    if ((msg.buttons[0] == 1) && !a_button_prev_)
    {
        if (point_count_ < N_)
        {
            addPoint();
        }
    }

    // Update previous A button state
    a_button_prev_ = msg.buttons[0];
}

void WorkpieceReg::addPoint()
{
    // Make sure we haven't recorded all 5 points
    if (point_count_ >= N_)
    {
        return;
    }

    try
    {
        // Get current end effector position in robot base frame
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(
                "base_link",
                "J6",
                tf2::TimePointZero);

        // Record laser position in robot frame
        p_r_[point_count_] << 
            transform.transform.translation.x,
            transform.transform.translation.y,
            -0.0127*2; //CONSIDER ACTUAL ROBOT SETUP FOR THIS (dcould be a laser offset) 12.7 mm is for plate

        // Print recorded point
        RCLCPP_INFO(
            this->get_logger(),
            "Point %d recorded: x = %0.4f, y = %0.4f, z = %0.4f",
            point_count_ + 1,
            p_r_[point_count_](0),
            p_r_[point_count_](1),
            p_r_[point_count_](2));

        // Increment point counter
        point_count_++;

        // Once all 5 points are recorded, perform registration
        if (point_count_ == N_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "All 5 points recorded. Performing registration...");

            registerPoints();
        }
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to record point: %s",
            ex.what());
    }
}

void WorkpieceReg::lookupFrame()
{
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("world", "J6", tf2::TimePointZero);

        RCLCPP_INFO(this->get_logger(), "Position -- x: %0.3f m, y: %0.3f m, z: %0.3f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "Orientation -- qw: %0.3f, qx: %0.3f, qy: %0.3f, qz: %0.3f", transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_ERROR(this->get_logger(), "lookup failed! Reason: %s", ex.what());
    }
}

// IMPLEMENT MAIN() FUNCTION HERE

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<WorkpieceReg>();

    // Wait for robot description to initialize KDL solver
    rclcpp::Rate init_rate(10);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // breaks once IK solver initialized (no longer a null ptr)
        if (node->ikSolverInitialized())
        {
            break;
        }

        init_rate.sleep();
    }

    // rcpp:node might have shut down in previous while loop termination
    if (!rclcpp::ok())
    {
        rclcpp::shutdown();
        return 1;
    }

    // Initialize MoveGroupInterface
    node->initializeMoveGroup();

    // Move Mitsubishi robot to starting pose
    bool success = node->solveIK(
        0.44, 0.0, 0.42,
        0.0, 1.0, 0.0, 0.0);

    if (!success)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Could not reach starting position");

        rclcpp::shutdown();
        return 1;
    }

    // Set loop rate
    rclcpp::Rate rate = rclcpp::Rate(100); // Hz

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->mainTelemanipulation();
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}