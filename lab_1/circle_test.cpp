#include <example_robot/circle_test.h>

CircleTest::CircleTest() : Node("circle_test")
{
  // Setup subscriber for robot description and transient lets you access old messages, you only want the latest robot description so keepLast(1)
  robot_description_sub_ = this->create_subscription<std_msgs::msg::String>("robot_description",
rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
    std::bind(&CircleTest::robotDescriptionCallback,
      this,
      std::placeholders::_1));


      // set first to true
      // set failures to 0


  // Joint state publisher
  joint_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

  // Joint names must match the names in the URDF
  joint_state_msg_.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
  joint_state_msg_.position.resize(joint_state_msg_.name.size());
}


/** for the first initialization of the robot should only take the URDF and create a tree
from the xacro file, and from tree build a chain from base to ee
*/
void CircleTest::robotDescriptionCallback(
    const std_msgs::msg::String& msg)
{
    const std::string urdf = msg.data;

    // make sure tree can actually be built from urdf
    if (!kdl_parser::treeFromString(urdf, tree_))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "KDL Tree build failed");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "KDL tree created successfully");

    // Extract chain from base to end effector
    if (!tree_.getChain("link_0", "ee_link", chain_))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to get KDL chain from link_0 to ee_link");

        // sanity check post compile
        RCLCPP_ERROR(
            this->get_logger(),
            "Tree has %u segments",
            tree_.getNrOfSegments());

        return;
    }


    // Initialize output joint array
    q_ = KDL::JntArray(chain_.getNrOfJoints());
    // initialize q_last_ as same size

    // create solver after you know the chain is valid
    ik_solver_ =
        std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);

    RCLCPP_INFO(
        this->get_logger(),
        "IK solver initialized");
}


void CircleTest::solveIK(
  double x, double y, double z,
  double qx, double qy, double qz, double qw)
{
  // Check if the IK solver has been initialized
  if (ik_solver_ != nullptr)
  {
    // Initial guess: all joint positions are zero as of now
    KDL::JntArray q_init(chain_.getNrOfJoints());


    double pi = 3.14159;


    //set const guesses
    q_init(0) = pi/2;
    q_init(1) = 0.0;
    q_init(2) = 0.2;
    q_init(3) = -pi/2;
    q_init(4) = 0.2;
    q_init(5) = 0.0;
    q_init(6) = 0.0;
  


    // Desired end-effector position
    KDL::Vector kdl_position(x, y, z);

    // Desired end-effector orientation
    KDL::Rotation kdl_rotation =
      KDL::Rotation::Quaternion(qx, qy, qz, qw);

    // Construct desired end-effector frame
    KDL::Frame desired_ee_frame(
      kdl_rotation,
      kdl_position);

    // Solve inverse kinematics
    int error =
      ik_solver_->CartToJnt(
        q_init,
        desired_ee_frame,
        q_);

    // If successful, publish the joint positions
    if (error == KDL::ChainIkSolverPos_LMA::E_NOERROR)
    {
      joint_state_msg_.header.stamp =
        this->get_clock()->now();

        // copy q_ (member var) into the message position for joint pub
      std::copy(
        q_.data.begin(),
        q_.data.end(),
        joint_state_msg_.position.begin());

        //put the q_ data into q_last

      joint_pub_->publish(joint_state_msg_);

      RCLCPP_INFO(
        this->get_logger(),
        "Inverse kinematics successful!");
    }
    else
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Inverse kinematics failed!");

        //increment failures
    }
  }
}


int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<CircleTest>();

    // Wait for /robot_description to initialize KDL solver
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if (node->ikSolverInitialized())
        {
            break;
        }
    }

    rclcpp::Rate rate(100.0);

    // Keep track of position around circle
    int angle_deg = 0;
    double pi = 3.14159;

    while (rclcpp::ok())
    {
        // Convert degrees to radians
        double theta = angle_deg * pi / 180.0;

        // Calculate desired position on circle
        double x = 1.6;
        double y = 0.5 + 0.1 * std::cos(theta);
        double z = 1.8 + 0.1 * std::sin(theta);

        // Solve IK for this desired pose
        node->solveIK(
            x, y, z,
            0.5, 0.5, 0.5, 0.5);

        rclcpp::spin_some(node);

        // Advance one degree
        angle_deg++;

        // Wrap around after one full circle
        if (angle_deg >= 360)
        {
            angle_deg = 0;
        }

        // Maintain 100 Hz
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}