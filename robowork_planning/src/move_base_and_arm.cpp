#include <moveit/move_group/node_name.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit/kinematic_constraints/utils.h>
#include <boost/scoped_ptr.hpp>
#include <tf/transform_listener.h>
#include <octomap_msgs/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>
#include <queue>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <array>
// #include "voxblox_planner.cpp"
#include <random>
#include <math.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <set>


ros::Subscriber base_sub;
ros::Subscriber arm_sub;



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// namespace move_group_interface {
template <typename T>
void println(T item)
{
  std::cout<< item <<std::endl;
}

template <typename T>
void printl(T item)
{
  std::cout<< item ;
}

class MMPoses
{
    public:
    geometry_msgs::PoseArray base_poses;
    geometry_msgs::PoseArray arm_poses;
    
    void base_cb(const geometry_msgs::PoseArray& msg)
    {
        base_poses.poses = msg.poses;
        // println(base_poses.poses.empty());
    }

    void arm_cb(const geometry_msgs::PoseArray& msg)
    {
        arm_poses.poses = msg.poses;
    }
};

void PrintTrajectory(moveit_msgs::RobotTrajectory& traj)
{
  std::vector<int>::size_type size1 = traj.joint_trajectory.points.size();
  ROS_INFO("Number of points = %i", size1);

  // std::vector<float> position;
  float position;
  //position.resize(vector1);
  int k = 0;
  for (unsigned i=0; i<size1; i++)
  {
    ROS_INFO("Trajectory at point %d",i);
    std::vector<int>::size_type size2 = traj.joint_trajectory.points[i].positions.size();
    for (unsigned j=0; j<size2; j++)
    { 
      position = traj.joint_trajectory.points[i].positions[j]; // positions[j]was eerst [0]
      if(j==0)
      {
        ROS_INFO("Joint angle for shoulder_pan_joint = %g", position);
      }
      else if(j==1)
      {
        ROS_INFO("Joint angle for shoulder_lift_joint = %g", position);
      }
      else if(j==2)
      {
        ROS_INFO("Joint angle for elbow_joint = %g", position);
      }
      else if(j==3)
      {
        ROS_INFO("Joint angle for wrist_1_joint = %g", position);
      }
      else if(j==4)
      {
        ROS_INFO("Joint angle for wrist_2_joint = %g", position);
      }
      else if(j==5)
      {
        ROS_INFO("Joint angle for wrist_3_joint = %g", position);
      }
      else
      {
        ROS_INFO("Joint angle for some random joint = %g", position);
      }
    }
  } 
}


// Eigen::Affine3d JointSpaceToCartesianPose(const trajectory_msgs::JointTrajectoryPoint& Traj, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_, const robot_state::JointModelGroup* joint_model_group_)
// {
//   robot_model::RobotModelConstPtr robot_model  = move_group_->getRobotModel();
//   robot_state::RobotState *kinematic_state = new robot_state::RobotState(robot_model);
//   const std::string GRIPPER_NAME = move_group_->getEndEffectorLink();

//   std::vector<double> joint_values;
//   kinematic_state->copyJointGroupPositions(joint_model_group_, joint_values);
//   std::size_t size_joints = joint_values.size();
//   ROS_INFO_STREAM("existing number of joints = "<< size_joints);

//   std::vector<double> home_joint_values;
//   for(auto& x:Traj.positions)
//   {
//     home_joint_values.push_back((double)x);
//   }
//   kinematic_state->setJointGroupPositions(joint_model_group_, home_joint_values);
//   Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform("bvr_SIM/main_arm_SIM/gripper_manipulation_link");
//   println("Name of end effector: "+GRIPPER_NAME);

//   Eigen::Quaterniond q_pos(end_effector_state.rotation());
//   auto tr = end_effector_state.translation();

// //   println("##########################################################");
// //   ROS_INFO_STREAM("Translation:\r\n" << end_effector_state.translation());
// //   // ROS_INFO_STREAM("Rotation:\r\n" << end_effector_state.rotation());
// //   ROS_INFO_STREAM("Qauternion xyzw:\r\n" << q_pos.vec()<< "\n" << q_pos.w());
// //   // ROS_INFO_STREAM("Qauternion xyz:\r\n" );
// //   println("##########################################################");
//   delete kinematic_state;
// }


std::string PLANNING_GROUP_;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
// std::unique_ptr<robot_model::RobotModelPtr> robot_model_;
// std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
std::unique_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
std::unique_ptr<tf::TransformListener> tf_listener_;
std::unique_ptr<MoveBaseClient> ac;

const robot_state::JointModelGroup* joint_model_group_;


std::vector<Eigen::Vector3d> getBoundingBox(Eigen::Vector3d& start, Eigen::Vector3d& end, Eigen::Vector3d& discretization)
{
    // ROS_INFO_STREAM("start = "<<start);
    // ROS_INFO_STREAM("end = "<<end);
    // ROS_INFO_STREAM("discretization = "<<discretization);
    std::vector<Eigen::Vector3d>BBCoordinates;
    for (double i = std::min(start(0),end(0)); i<=std::max(start(0),end(0)); i+=discretization[0])
    {   
        for (double j = std::min(start(1),end(1)); j<=std::max(start(1),end(1)); j+=discretization[1])
        {
            for (double k = std::min(start(2),end(2)); k<=std::max(start(2),end(2)); k+=discretization[2])
            {
                Eigen::Vector3d element;
                // ROS_INFO_STREAM("element added = "<<i<<" "<<j<<" "<<k);
                
                element<<i, j, k;
                BBCoordinates.push_back(element);
            }
        }
    }
    return BBCoordinates;
}

int getCellStatusPoint( const Eigen::Vector3d& point, const octomap::OcTree* octree_)
{
    octomap::OcTreeNode* node = octree_->search(point.x(), point.y(), point.z());
    if (node == NULL) 
    {
        return 1;
    }
    else if (octree_->isNodeOccupied(node))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

double get3DDistance(octomap::point3d& a, octomap::point3d& b)
{
    return sqrt( pow((a.x()-b.x()),2) + pow((a.y()-b.y()),2) + pow((a.z()-b.z()),2) );
}

std::vector<Eigen::Vector3d> getRobotBoundingBox2D(const Eigen::Vector3d& point, double d)
{
    double motionX[] = {0,0, 0,d,d, d,-d,-d,-d};
    double motionY[] = {0,d,-d,0,d,-d, 0, d,-d};
    std::vector<Eigen::Vector3d> RobotBoundingBox2D;
    for(int i=0; i<9; i++)
    {
        Eigen::Vector3d curr_pt;
        curr_pt << point.x()+motionX[i], point.y()+motionY[i], point.z();
        RobotBoundingBox2D.push_back(curr_pt);
    }
    return RobotBoundingBox2D;
}

bool getRobotBBFreeStatus(const Eigen::Vector3d& point, const octomap::OcTree* octree_, double d, double robot_release_thresh)
{
    std::vector<Eigen::Vector3d> RobotBoundingBox2D = getRobotBoundingBox2D(point,d);
    for(auto point : RobotBoundingBox2D)
    {
        if(getCellStatusPoint(point,octree_)==0)     //if point is not occupied
        {
            octomap::point3d origin(point.x(), point.y(), point.z());
            octomap::point3d direction(0.0, 0.0, -1.0);
            octomap::point3d end;
            if(octree_->castRay(origin, direction, end, false))
            {
                auto dist = get3DDistance(origin,end);
                if(dist<robot_release_thresh)
                {
                   return false;        //not free i.e. occupied
                }
            }
            else
            {
                return false;       //not free i.e. occupied
            }
        }
        else
        {
            return false;       //not free i.e. occupied
        }
        
    }
    return true;        //free
}

visualization_msgs::Marker getMarker(const Eigen::Vector3d& position, const Eigen::Vector3d& color, uint action = 0, const std::string& ns = "points")
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    if(action == 0)
    {
        marker.action = visualization_msgs::Marker::ADD;
    }
    else if(action == 3)
    {
        marker.action = visualization_msgs::Marker::DELETEALL;
    }
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();
    marker.lifetime = ros::Duration(0.0);
    return marker;
}

visualization_msgs::MarkerArray getMarkerArray(const std::vector<Eigen::Vector3d>& position_list, const Eigen::Vector3d& color = Eigen::Vector3d(1.0,0.0,0.0), uint action = 0, const std::string& ns = "points")
{
    visualization_msgs::MarkerArray arr;
    for(auto position : position_list)
    {
        visualization_msgs::Marker marker = getMarker(position, color, action, ns);
        arr.markers.push_back(marker);
    }
    return arr;
}


void move_base_sip( move_base_msgs::MoveBaseGoal goal)
{
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    // move_base_msgs::MoveBaseGoal goal;
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("move base goal success");
    else
    ROS_INFO("The base failed to move for some reason");
}

double getDistanceXY(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
    double dist = sqrt( pow(a.position.x-b.position.x, 2) + pow(a.position.y-b.position.y, 2) );
    // printl("distance = ");
    // println(dist);
    return dist;
}

tf::Quaternion armToBaseOrientation(geometry_msgs::Pose arm_pose)
{
    tf::Quaternion q_orig(arm_pose.orientation.x, arm_pose.orientation.y, arm_pose.orientation.z, arm_pose.orientation.w);
    tf::Quaternion q_res;
    tfScalar R,P,Y;
    tf::Matrix3x3 r_res(q_orig);
    r_res.getRPY(R,P,Y);
    q_res.setRPY(0.0,0.0,Y);
    return q_res;
}

// geometry_msgs::Pose generate_base_location(geometry_msgs::Pose arm_pose, YourPlannerVoxblox& voxblox_ob)
// {
//     //Project cluster_center to ground
//     // cluster_center[2] = 0
//     geometry_msgs::Pose final_base_pose;
//     tf::Quaternion q = armToBaseOrientation(arm_pose);
//     final_base_pose.orientation.x = q.x();
//     final_base_pose.orientation.y = q.y();
//     final_base_pose.orientation.z = q.z();
//     final_base_pose.orientation.w = q.w();
//     double projected_circle_radius = 1.5;
//     Eigen::Vector3d base_pose(arm_pose.position.x, arm_pose.position.y, 0.0);
//     std::default_random_engine generator;
//     std::uniform_real_distribution<double> distribution(0.0,projected_circle_radius);
//     std::uniform_real_distribution<double> dist2(0.0,2.0);

//     //  Eigen::Vector3d bp(x,y, 0.5);

//     for(int i =0; i<100;i++)
//     {
//         double length = std::sqrt(distribution(generator));

//         // Sample point for base location in a circle around the projected cluster_center
//         double angle = M_PI * dist2(generator);
//         double x = base_pose(0) + length * cos(angle);
//         double y = base_pose(1) + length * sin(angle);
//         Eigen::Vector3d bp(x,y, 0.75);
//         double distance = voxblox_ob.getMapDistance(bp);
//         final_base_pose.position.x = bp(0);
//         final_base_pose.position.y = bp(1);
//         final_base_pose.position.z = bp(2);
//         if(distance > 0.7)  return final_base_pose;
//     // print('Fatal: Unable to find a possible base location after 100 tries. Maybe try increasing the projected_circle_radius? Voxblox could also be reporting the collision with the ground. This can be fixed by increasing the base_offset')
//     }
//     ROS_INFO_STREAM("Fatal: Unable to find a possible base location after 100 tries. Maybe try increasing the projected_circle_radius? Voxblox could also be reporting the collision with the ground. This can be fixed by increasing the base_offset");
//     exit(0);
// }


void moveArmToHome(moveit_visual_tools::MoveItVisualTools visual_tools)
{
    ROS_INFO_STREAM("Moveing arm to home position");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);
    move_group_->setStartState(*move_group_->getCurrentState());
    // goal_pose.pose.position.x = 0.4478835545670475;
    // goal_pose.pose.position.y = -0.13812854245998615;
    // goal_pose.pose.position.z = 1.0568406575242237;
    // goal_pose.pose.orientation.x = 0.09419626525077829;
    // goal_pose.pose.orientation.y = 0.09419626525077829;
    // goal_pose.pose.orientation.z = 0.09872008286194436;
    // goal_pose.pose.orientation.w = 0.9869694131667913;
    // goal_pose.header.stamp = ros::Time::now();
    // goal_pose.header.frame_id = "map";

    std::vector< std::string > joint_names = move_group_->getJointNames();
    for(auto joint : joint_names)
    {ROS_INFO_NAMED("joints in order = ",joint.c_str());}    
    // filling joint values for goal 
    /*
            - bvr_SIM/main_arm_SIM/shoulder_pan_joint = 0.5374265775041822
            - bvr_SIM/main_arm_SIM/shoulder_lift_joint = -0.4019596255032
            - bvr_SIM/main_arm_SIM/elbow_joint = -1.8751256726187853
            - bvr_SIM/main_arm_SIM/wrist_1_joint = -0.664424544023154
            - bvr_SIM/main_arm_SIM/wrist_2_joint = 1.2225400711188588
            - bvr_SIM/main_arm_SIM/wrist_3_joint = 0.10657869075931803
            */
    joint_group_positions[0] = 0.5374265775041822;
    joint_group_positions[1] = -0.4019596255032;
    joint_group_positions[2] = -1.8751256726187853;
    joint_group_positions[3] = -0.664424544023154;
    joint_group_positions[4] = 1.2225400711188588;
    joint_group_positions[5] = 0.10657869075931803;

    // bvr_SIM/main_arm_SIM/shoulder_pan_joint
    // bvr_SIM/main_arm_SIM/shoulder_lift_joint
    // bvr_SIM/main_arm_SIM/elbow_joint
    // bvr_SIM/main_arm_SIM/wrist_1_joint
    // bvr_SIM/main_arm_SIM/wrist_2_joint
    // bvr_SIM/main_arm_SIM/wrist_3_joint

    move_group_->setGoalOrientationTolerance(0.17);
    move_group_->setGoalPositionTolerance(0.05);
    // move_group_->setPoseTarget(goal_pose);
    move_group_->setJointValueTarget(joint_group_positions);
    geometry_msgs::PoseStamped goal_pose = move_group_->getPoseTarget();

    visual_tools.publishArrow(goal_pose.pose,rviz_visual_tools::colors::CYAN,rviz_visual_tools::scales::LARGE);
    visual_tools.trigger();
    visual_tools.deleteAllMarkers();

    bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success)
    {
        visual_tools.prompt("Found solution Press 'next'to execute trajectory");
        move_group_->execute(my_plan);
        // ros::Duration(1).sleep();
        // visual_tools.prompt("Trajectory executed, Press 'next'to continue");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_launch");
    ros::AsyncSpinner spinner(10);
    spinner.start();
    ros::NodeHandle node_handle("~"),nh;
    MMPoses ArmBasePairs;
    base_sub = nh.subscribe("/base_poses", 10, &MMPoses::base_cb, &ArmBasePairs);
    arm_sub = nh.subscribe("/viewpoints", 10, &MMPoses::arm_cb, &ArmBasePairs);
    PLANNING_GROUP_ = "main_arm_SIM";
    std::string robot_namespace = "bvr_SIM";
    std::string arm_namespace = "main_arm_SIM";
    // node_handle.getParam("robot_namespace", robot_namespace);
    // node_handle.getParam("arm_namespace", arm_namespace);
    std::string bvr_base_interia_frame = robot_namespace+"/bvr_base_inertia";
    std::string gripper_link = robot_namespace + "/main_arm_SIM/gripper_manipulation_link";
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_);
    // planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    robot_model::RobotModelConstPtr robot_model_ = move_group_->getRobotModel();
    robot_state::RobotStatePtr robot_state_(new robot_state::RobotState(robot_model_));
    joint_model_group_ = robot_state_->getJointModelGroup(PLANNING_GROUP_);

    // planning_scene::PlanningScenePtr planning_scene_(new planning_scene::PlanningScene(robot_model_));
    // robot_model_loader::RobotModelLoader robot_model_loader_("bvr_SIM/robot_description");
    planning_scene_monitor_ = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>("bvr_SIM/robot_description");
    planning_scene::PlanningScenePtr planning_scene_ = planning_scene_monitor_->getPlanningScene();
    bool success = planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
    ROS_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));
    //
    planning_scene_monitor_->startSceneMonitor("move_group/monitored_planning_scene");
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_STATE);
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    moveit_msgs::MotionPlanRequest req;
    req.group_name = PLANNING_GROUP_;
    moveit_msgs::MotionPlanResponse res;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model_, node_handle.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                            << "Available plugins: " << ss.str());
    }

    // YourPlannerVoxblox voxblox_ob(nh, node_handle);
    

    move_group_->setMaxVelocityScalingFactor(0.25);
    // Set a scaling factor for optionally reducing the maximum joint acceleration.
    move_group_->setMaxAccelerationScalingFactor(1.0);
    // Planning with constraints can be slow because every sample must call an inverse kinematics solver (default 5 seconds)
    move_group_->setPlanningTime(5.0); //5.0
    // Number of times the motion plan is to be computed from scratch before the shortest solution is returned. 
    move_group_->setNumPlanningAttempts(100); //10	
    // Set the tolerance that is used for reaching the goal. For joint state goals, this will be distance for each joint, in the configuration space (radians or meters depending on joint type). For pose goals this will be the radius of a sphere where the end-effector must reach.
    move_group_->setGoalTolerance(0.02);
    // Pick one of the available configs - see ee ompl_planning<_SIM>.yaml for a complete list
    move_group_->setPlannerId("RRTstarkConfigDefault");

    // Configure a valid robot state
    planning_scene_->getCurrentStateNonConst().setToDefaultValues(joint_model_group_, "ready");
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("map");


    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();
    visual_tools.loadRemoteControl();

    move_group_->setStartState(*move_group_->getCurrentState());
    geometry_msgs::PoseStamped curr_pose = move_group_->getCurrentPose();
    // println("frame of curr pose= ");
    // println(curr_pose.header.frame_id);

    visual_tools.deleteAllMarkers();

    move_base_msgs::MoveBaseGoal goal_base;
    geometry_msgs::PoseStamped goal_pose = curr_pose;

    
    println("waiting for base and arm poses");
    println(ArmBasePairs.arm_poses.poses.empty());
    while(ArmBasePairs.arm_poses.poses.empty() && ros::ok())
    {
        // println("poses empty? 0 is not empty");
        // println(ArmBasePairs.arm_poses.poses.empty());
    }

    geometry_msgs::PoseArray visited_arm_poses;
    if(!ArmBasePairs.arm_poses.poses.empty())
    {
        ROS_INFO_STREAM("Found base and arm poses");
        geometry_msgs::Pose curr_pose;
        curr_pose.position.x = 0.0;
        curr_pose.position.y = 0.0;
        curr_pose.position.z = 0.0;
        // double min_dist = 10000;
        // int ind = -1;
        std::set<int> visited;
        std::set<int> yet_to_visit;

        for( int i = 0; i < ArmBasePairs.base_poses.poses.size(); i++)
        {
            yet_to_visit.insert(i);
        }

        while(!yet_to_visit.empty())
        {
            double min_dist = 10000;
            int ind = -1;
            ROS_INFO_STREAM("size of yet to visit list = "<<yet_to_visit.size());
            for( int i = 0; i < ArmBasePairs.base_poses.poses.size(); i++)
            {
                std::set<int>::iterator ret = visited.find(i);
                if(ret != visited.end())
                {
                    continue;
                }
                else
                {
                    auto base_pose = ArmBasePairs.base_poses.poses[i];
                    double dist = getDistanceXY(curr_pose,base_pose);
                    // ROS_INFO_STREAM("dist inside = "<< dist);
                    if(dist < min_dist)
                    {
                        ind = i;
                        min_dist = dist;
                    }
                }
            }
            // we now have index of next base loc to visit that is closest  
            // to the current position and not yet visited

            // Now go to that point and add it to visited list and
            // remove it from "yet_to_visit" list
            visited.insert(ind);
            yet_to_visit.erase(ind);

            // geometry_msgs::Pose home_arm_pose;
            // home_arm_pose.position.x =  0.38347654341965237;
            // home_arm_pose.position.y =  -0.1320602606905012;
            // home_arm_pose.position.z =  1.0942726556268887;
            // home_arm_pose.orientation.x =  -0.0010899083262423549;
            // home_arm_pose.orientation.y =  0.02130426457749196;
            // home_arm_pose.orientation.z =  0.001094902234892504;
            // home_arm_pose.orientation.w =  0.9997718447724705;
            /*
            - bvr_SIM/main_arm_SIM/elbow_joint = -1.8751256726187853
            - bvr_SIM/main_arm_SIM/shoulder_lift_joint = -0.4019596255032
            - bvr_SIM/main_arm_SIM/shoulder_pan_joint = 0.5374265775041822
            - bvr_SIM/main_arm_SIM/wrist_1_joint = -0.664424544023154
            - bvr_SIM/main_arm_SIM/wrist_2_joint = 1.2225400711188588
            - bvr_SIM/main_arm_SIM/wrist_3_joint = 0.10657869075931803
            */


            moveArmToHome(visual_tools);

            auto base_pose = ArmBasePairs.base_poses.poses[ind];
            curr_pose = base_pose;
            ROS_INFO_STREAM("ind: "<< ind);

            goal_base.target_pose.pose.position.x = base_pose.position.x;
            goal_base.target_pose.pose.position.y = base_pose.position.y;
            ROS_INFO_STREAM("Moving Base to goal: "<< base_pose);

            tf::Quaternion quat_b =  armToBaseOrientation(base_pose);

            goal_base.target_pose.pose.orientation.x = quat_b.x();
            goal_base.target_pose.pose.orientation.y = quat_b.y();
            goal_base.target_pose.pose.orientation.z = quat_b.z();
            goal_base.target_pose.pose.orientation.w = quat_b.w();
            goal_base.target_pose.header.frame_id = "map";

            ROS_INFO_STREAM("Moving Base to goal: "<< goal_base.target_pose.pose);
            visual_tools.publishArrow(goal_base.target_pose.pose,rviz_visual_tools::colors::RED,rviz_visual_tools::scales::LARGE);
            visual_tools.trigger();
            move_base_sip(goal_base);
            // ros::Duration(0.5).sleep();

            // for( auto arm_pose : ArmBasePairs.arm_poses.poses)
            auto arm_pose = ArmBasePairs.arm_poses.poses[ind];
            {
                // if(getDistanceXY(base_pose,arm_pose) < 0.7)
                // {
                //     if(visited_arm_poses.poses.empty())
                //     {
                //         visited_arm_poses.poses.push_back(arm_pose);
                //     }
                //     else
                //     {
                //         for(auto each_arm_pose : visited_arm_poses.poses)
                //         {
                //             if(each_arm_pose == arm_pose) goto next_iteration;
                //         }
                //         visited_arm_poses.poses.push_back(arm_pose);
                //     }
                    
                ROS_INFO_STREAM("Moving arm to goal: "<< arm_pose);
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                move_group_->setStartState(*move_group_->getCurrentState());
                goal_pose.pose = arm_pose;
                goal_pose.header.stamp = ros::Time::now();
                goal_pose.header.frame_id = "map";
                

                // std::vector<double> tolerance_pose(3, 0.05);
                // std::vector<double> tolerance_angle(3, 0.023);
                // moveit_msgs::Constraints pose_goal =
                // kinematic_constraints::constructGoalConstraints(gripper_link, goal_pose, tolerance_pose, tolerance_angle);
                // req.goal_constraints.push_back(pose_goal);

                // planning_interface::PlanningContextPtr context =
                // planner_instance->getPlanningContext(planning_scene_, req, res.error_code);
                // context->solve(res);
                // if (res.error_code.val != res.error_code.SUCCESS)
                // {
                // ROS_ERROR("Could not compute plan successfully");
                // return 0;
                // }

                move_group_->setGoalOrientationTolerance(0.23);
                move_group_->setGoalPositionTolerance(0.05);
                move_group_->setPoseTarget(goal_pose);
                visual_tools.publishArrow(goal_pose.pose,rviz_visual_tools::colors::YELLOW,rviz_visual_tools::scales::XLARGE);
                visual_tools.trigger();
                visual_tools.deleteAllMarkers();

                success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if(success)
                {
                    ROS_INFO_STREAM("plan success for arm pose" << arm_pose);
                    // visual_tools.trigger();
                    // ros::Duration(0.3).sleep();
                    visual_tools.prompt("Found solution Press 'next'to execute trajectory");
                    moveit::planning_interface::MoveItErrorCode exec_success = move_group_->execute(my_plan);
                    ROS_INFO_STREAM("execution success message #"<< exec_success.val);
                    // ros::Duration(1).sleep();
                    // visual_tools.prompt("Trajectory executed, Press 'next'to continue");
                }
                else
                {
                    move_group_->setGoalOrientationTolerance(0.449066);
                    move_group_->setGoalPositionTolerance(0.2);
                    success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    if(success)
                    {
                        ROS_INFO_STREAM("plan success for arm pose" << arm_pose);
                        // visual_tools.trigger();
                        // ros::Duration(0.3).sleep();
                        visual_tools.prompt("Found solution Press 'next'to execute trajectory");
                        moveit::planning_interface::MoveItErrorCode exec_success = move_group_->execute(my_plan);
                        ROS_INFO_STREAM("execution success message #"<< exec_success.val);
                        // ros::Duration(1).sleep();
                        // visual_tools.prompt("Trajectory executed, Press 'next'to continue");
                    }
                }
                
                // }
                // next_iteration: ;
            }
        } // end while
    }
    // visual_tools.prompt("Press 'next' to delete markers");

    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
   

    return 0;
}
