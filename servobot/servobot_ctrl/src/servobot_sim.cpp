#include <servobot_sim.h>
#include <tf/tf.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#define PI_CONST 3.141592653589793238462643383279502884
#include <cmath>

struct getTragetRPY
{
  double x, y;
  getTragetRPY():x(0), y(0){};
  getTragetRPY(double _x, double _y):x(_x),y(_y){};
  //when the angle depend on X axis
  double get_axisX(double x_target, double y_target)
  {
    double result;
    result = atan2(y_target, x_target);
    return result;
  };
  //when the angle depend on Y axis
  double get_axisY(double x_target, double y_target)
  {
    double result;
    result = atan2(x_target - x, y_target - y);
    return result;
  };
};

double getAngle(double x0, double y0, double x1, double y1)
{
  double t = ((x0)*(x1) + (y0)*(y1))/ (sqrt(pow(x0, 2) + pow(y0, 2))*sqrt(pow(x1, 2) + pow(y1, 2)));
  t = acos(t);
  return t;
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_ctrl");
    ros::NodeHandle node_hadle;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("arm");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("arm");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    std::cout << "EEF Name: " << joint_model_group->getEndEffectorName() << std::endl;

    group.setPoseReferenceFrame("base_link");
    group.setGoalPositionTolerance(0.001);
    group.setGoalOrientationTolerance(0.01);


    //get the original pose of end effector
    geometry_msgs::PoseStamped defaultPose;
    defaultPose = group.getCurrentPose("link5");     
    // getTragetRPY servobot(defaultPose.pose.position.x,defaultPose.pose.position.y);
    double x0 = defaultPose.pose.position.x;
    double y0 = defaultPose.pose.position.y;





    //官网的方法
    
//     moveit_visual_tools::MoveItVisualTools visual_tool("world");
//     visual_tool.deleteAllMarkers();

//     ros::Publisher planning_scene_diff_publisher = node_hadle.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene",1);
//     ros::WallDuration sleep_t(0.5);

//     while (planning_scene_diff_publisher.getNumSubscribers() < 1)
//     {
//       sleep_t.sleep();
//     }
//     visual_tool.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    


//   // Define the attached object message
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   // We will use this message to add or
//   // subtract the object from the world
//   // and to attach the object to the robot
//   moveit_msgs::AttachedCollisionObject attached_object;
//   attached_object.link_name = "link5";
//   /* The header must contain a valid TF frame*/
//   attached_object.object.header.frame_id = "link5";
//   /* The id of the object */
//   attached_object.object.id = "box";

//   /* A default pose */
//   geometry_msgs::Pose pose;
//   pose.orientation.w = 1.0;

//   /* Define a box to be attached */
//   shape_msgs::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[0] = 0.1;
//   primitive.dimensions[1] = 0.1;
//   primitive.dimensions[2] = 0.1;

//   attached_object.object.primitives.push_back(primitive);
//   attached_object.object.primitive_poses.push_back(pose);

//   // Note that attaching an object to the robot requires
//   // the corresponding operation to be specified as an ADD operation
//   attached_object.object.operation = attached_object.object.ADD;

//   attached_object.touch_links = std::vector<std::string>{ "link5", "link4"};

//   ROS_INFO("Adding the object into the world at the location of the hand.");
//   moveit_msgs::PlanningScene planning_scene;
//   planning_scene.world.collision_objects.push_back(attached_object.object);
//   planning_scene.is_diff = true;
//   planning_scene_diff_publisher.publish(planning_scene);
//   visual_tool.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   /* First, define the REMOVE object message*/
//   moveit_msgs::CollisionObject remove_object;
//   remove_object.id = "box";
//   remove_object.header.frame_id = "world";
//   remove_object.operation = remove_object.REMOVE;

// /* Carry out the REMOVE + ATTACH operation */
//   ROS_INFO("Attaching the object to the hand and removing it from the world.");
//   planning_scene.world.collision_objects.clear();
//   planning_scene.world.collision_objects.push_back(remove_object);
//   planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
//   planning_scene_diff_publisher.publish(planning_scene);




//   visual_tool.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    //古月的方法
    /*
    moveit::planning_interface::PlanningSceneInterface currentScene;
    sleep(5.0);

    moveit_msgs::CollisionObject cylinder;
    cylinder.id = "seven_dof_arm_cylinder";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.6;
    primitive.dimensions[1] = 0.2;

    geometry_msgs::Pose objectPose;
    objectPose.orientation.w = 1.0;
    objectPose.position.x =0.0;
    objectPose.position.y = -0.4;
    objectPose.position.z = 0.4;

    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(objectPose);
    cylinder.operation = cylinder.ADD;

    std::vector<moveit_msgs::CollisionObject>collision_objects;
    collision_objects.push_back(cylinder);

    currentScene.addCollisionObjects(collision_objects);*/



    tfScalar roll, pitch, yaw;

    yaw=0; pitch=0; roll=PI_CONST;

    tf::Quaternion setData;
    setData.setRPY(roll, pitch, yaw);

    ROS_INFO_NAMED("tutorial", "End effector link: %s", group.getEndEffectorLink().c_str());


    geometry_msgs::Pose targetPose;
    targetPose.orientation.w = setData.w();
    targetPose.orientation.x = setData.x();
    targetPose.orientation.y = setData.y();
    targetPose.orientation.z = setData.z();
    targetPose.position.x = 0.17;
    targetPose.position.y = -0.0235;
    targetPose.position.z = 0.1;
    

 
    // group.setPoseTarget(targetPose);
    // group.setGoalTolerance(0.01);
    // std::cout<<group.getGoalJointTolerance()<<std::endl;

    // moveit::planning_interface::MoveGroupInterface::Plan myPlan;
    // bool success = (group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   



  moveit::planning_interface::MoveGroupInterface::Plan myPlan;  
  ROS_INFO("Trying IK.....");
  if(robot_state->setFromIK(joint_model_group, targetPose))
  {
    ROS_INFO("successfully retrieved IK Solution!");
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      group.setJointValueTarget(joint_values);
    }
  }
  else{
    ROS_ERROR("CANNOT SOLVE IK");
  }
  bool success = (group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  group.execute(myPlan);

  //set waypoint
  std::vector<geometry_msgs::Pose> waypoints;
  std::cout<<"点0yaw为:"<<yaw<<std::endl;
  std::cout<<"点0roll为:"<<roll<<std::endl;
  std::cout<<"点0pitch为:"<<pitch<<std::endl;
  waypoints.push_back(targetPose);    //start point

  //point 1
  // double x1 = 0.0798760167608;
  // double y1 = 0.140600191449;
  // targetPose.position.x = x1;
  // targetPose.position.y = y1;
  targetPose.position.x += 0.000600290584;
  targetPose.position.y += 0.041478726929;
  // targetPose.position.y +=0.01;
  // yaw += getAngle(x0,y0,x1,y1);
  double temp = getAngle(x0,y0,targetPose.position.x,targetPose.position.y);
  std::cout<<"111:"<<temp<<std::endl;
  yaw += getAngle(x0,y0,targetPose.position.x,targetPose.position.y);
  // std::cout<<"点1yaw为:"<<yaw<<std::endl;
  // targetPose.position.x = 0.161059703496;
  // targetPose.position.y = -0.0145593918862;

  setData.setRPY(roll, pitch, yaw);
  targetPose.orientation.w = setData.getW();
  targetPose.orientation.x = setData.getX();
  targetPose.orientation.y = setData.getY();
  targetPose.orientation.z = setData.getZ();
  waypoints.push_back(targetPose);

   std::cout<<"点1yaw为:"<<yaw<<std::endl;
   std::cout<<"点1roll为:"<<roll<<std::endl;
   std::cout<<"点1pitch为:"<<pitch<<std::endl;
  if(robot_state->setFromIK(joint_model_group, targetPose))
  {
    ROS_INFO("successfully retrieved IK Solution!");
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      group.setJointValueTarget(joint_values);
    }
  }
  else{
    ROS_ERROR("CANNOT SOLVE IK");
  }

  //point 2


  setData.setRPY(roll, pitch, yaw);  
  // targetPose.orientation.w = setData.getW();
  // targetPose.orientation.x = setData.getX();
  // targetPose.orientation.y = setData.getY();
  // targetPose.orientation.z = setData.getZ();
  waypoints.push_back(targetPose);  
  if(robot_state->setFromIK(joint_model_group, targetPose))
  {
    ROS_INFO("successfully retrieved IK Solution!");
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      group.setJointValueTarget(joint_values);
    }
  }
  else{
    ROS_ERROR("CANNOT SOLVE IK");
  }

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = 0;
  int maxtries = 200;   //最大尝试规划次数
  int attempts = 0;     //已经尝试规划次数

  while(fraction < 1.0 && attempts < maxtries)
  {
    fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    attempts++;
    if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
  }

  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  myPlan.trajectory_ = trajectory;
  group.execute(myPlan);
  
  ros::shutdown(); 
  
  ros::waitForShutdown();
  return 0;

}


