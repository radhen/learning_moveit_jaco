
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace learn_moveit
{

class kinematicModelTutorial
{
private:
  ros::NodeHandle nh_;

public:
  kinematicModelTutorial()
  : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("constructor","starting kinematicModelTutorial...");

    // look up robot description on the ROS parameter server and construct moveit RobotModel
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    // Q: Is this differnt than the robot model that RViz shows when you add 'RobotModel'?
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO_STREAM_NAMED("km_tester","Model frame = " << kinematic_model->getModelFrame().c_str());

    // set robot model to default values
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

    // return a reference to the member variable?
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    std::vector<double> joint_values;
    // Q: didn't we just set these to the default values? What are we doing now?
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); i++)
    {
      ROS_INFO_STREAM_NAMED("km_tester","joint " << joint_names[i].c_str() << ": " << joint_values[i]);
    }

    // Joint limits
    // Q: how would we query what the set bounds are?
    joint_values[0] = 2;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    ROS_INFO_STREAM_NAMED("km_tester","Current state is " << kinematic_state->satisfiesBounds());

    kinematic_state->enforceBounds();
    ROS_INFO_STREAM_NAMED("km_tester","Current state is " << kinematic_state->satisfiesBounds());

    // Forward Kinematics
    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("jaco_6_hand_limb");
    ROS_INFO_STREAM_NAMED("km_tester","Translation = " << end_effector_state.translation());
    ROS_INFO_STREAM_NAMED("km_tester","Rotation = " << end_effector_state.rotation());

    // Inverse Kinematics
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); i++)
      {
        ROS_INFO_STREAM_NAMED("km_tester","Joint " << joint_names[i].c_str() << ": " << joint_values[i]);
      }
    }
    else
    {
      ROS_INFO_STREAM_NAMED("km_tester","Did not find IK solution");
    }

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,
                                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                 reference_point_position, jacobian);
    ROS_INFO_STREAM("Jacobian: " << jacobian);

  }

}; // end class kinematicModelTutorial
} // end namespace learn_moveit


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematic_model_tester");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  int test = 1;
  learn_moveit::kinematicModelTutorial tester;

  return 0;
}
