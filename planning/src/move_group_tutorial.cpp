#include <ros/ros.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>


namespace learn_moveit
{

class moveGroupInterfaceTester
{
private:
  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  // Q: Is this the best way to create and initialize class variables for MoveGroup?
  moveit::planning_interface::MoveGroup* Arm;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


public:

  // Q: Why does constructor need an argument to work? W/o default constructor keeps getting called.
  moveGroupInterfaceTester(int test)
    : nh_("~")
  {

    ROS_INFO_STREAM_NAMED("constructor","starting moveGroupInterfaceTester...");
    moveit::planning_interface::MoveGroup arm("arm");
    Arm = &arm;

    ROS_INFO_STREAM_NAMED("constructor","Reference frame: " << Arm->getPlanningFrame().c_str());

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("/robot_root", "/rviz_visual_tools"));

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1;
    target_pose.position.x = 0.35;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.5;
    Arm->setPoseTarget(target_pose);
    visual_tools_->publishAxisLabeled(target_pose, "target_pose1", rviz_visual_tools::SMALL);

    moveit::planning_interface::MoveGroup::Plan new_plan;
    //Compute a motion plan that takes the group declared in the constructor from the current state to the specified target.
    //No execution is performed. The resulting plan is stored in plan.
    bool success = Arm->plan(new_plan);
    ROS_INFO_STREAM_NAMED("constructor","success = " << success);
    ROS_INFO_STREAM_NAMED("constructor","Visualizing joint space plan for 5 seconds...");
    ros::Duration(5.0).sleep();


    /*==*==*==*==*== PLANNING WITH ORIENTATION CONSTRAINT ==*==*==*==*/

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "jaco_6_hand_limb";
    ocm.header.frame_id = "robot_root";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    Arm->setPathConstraints(test_constraints);

    // set start pose so orientation constraint is satisfied.
    robot_state::RobotState start_state(*Arm->getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.4;
    start_pose2.position.y = 0.2;
    start_pose2.position.z = 0.2;

    const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(Arm->getName());
    start_state.setFromIK(joint_model_group, start_pose2);
    Arm->setStartState(start_state);
    visual_tools_->publishAxisLabeled(start_pose2, "target_pose2", rviz_visual_tools::SMALL);

    Arm->setPoseTarget(target_pose);
    success = Arm->plan(new_plan);
    ROS_INFO_STREAM_NAMED("constructor","success = " << success);

    ROS_INFO_STREAM_NAMED("constructor","Visualizing orientation constrained plan for 5 seconds...");
    ros::Duration(5.0).sleep();


    /*==*==*==*==*== CARTESIAN PATH, FOLLOW A TRAJECTORY (waypoints)==*==*==*==*/

    // std::vector<geometry_msgs::Pose> waypoints;
    //
    // // set start pose so orientation constraint is satisfied.
    // robot_state::RobotState start_state(*Arm->getCurrentState());
    // geometry_msgs::Pose start_pose2;
    // start_pose2.orientation.w = 1.0;
    // start_pose2.position.x = 0.2;
    // start_pose2.position.y = 0.2;
    // start_pose2.position.z = 0.35;
    //
    // geometry_msgs::Pose target_pose3 = start_pose2;
    // target_pose3.position.x -= 0.1;
    // target_pose3.position.z += 0.1;
    // waypoints.push_back(target_pose3);
    //
    // target_pose3.position.y -= 0.1;
    // waypoints.push_back(target_pose3);
    //
    // target_pose3.position.x -= 0.1;
    // target_pose3.position.y += 0.1;
    // target_pose3.position.z += 0.1;
    // waypoints.push_back(target_pose3);
    //
    // moveit_msgs::RobotTrajectory trajectory;
    // // Q: what exactly is fraction?
    // double fraction = Arm->computeCartesianPath(waypoints, 0.05, 0.01, trajectory);
    //
    // ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
    // ros::Duration(15.0).sleep();


    /*==*==*==*==*== Adding/Removing Objects and Attaching/Detaching Objects==*==*==*==*/

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = Arm->getPlanningFrame();

    /* The id of the object is used to identify it. */
    collision_object.id = "box1";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.6;
    box_pose.position.y = -0.2;
    box_pose.position.z = 0.2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO("Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    /* Sleep so we have time to see the object in RViz */
    sleep(2.0);

    Arm->setPlanningTime(10.0);

    Arm->setStartState(*Arm->getCurrentState());
    Arm->setPoseTarget(target_pose);

    success = Arm->plan(new_plan);
    ROS_INFO("Visualizing plan 5 (pose goal move around box) %s", success ? "" : "FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(10.0);

    ROS_INFO("Attach the object to the robot");
    Arm->attachObject(collision_object.id);
    /* Sleep to give Rviz time to show the object attached (different color). */
    sleep(4.0);

    ROS_INFO("Detach the object from the robot");
    Arm->detachObject(collision_object.id);
    /* Sleep to give Rviz time to show the object detached. */
    sleep(4.0);

    ROS_INFO("Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    /* Sleep to give Rviz time to show the object is no longer there. */
    sleep(4.0);

    while(ros::ok())
    {

    }
  }

}; // end class moveGroupInterfaceTester
} // end namespace learn_moveit


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tester");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  int test = 1;
  learn_moveit::moveGroupInterfaceTester tester(test);

  return 0;
}
