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
#include "voxblox_planner.cpp"
#include <random>
#include <math.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>


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
    printl("distance = ");
    println(dist);
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

geometry_msgs::Pose generate_base_location(geometry_msgs::Pose arm_pose, YourPlannerVoxblox& voxblox_ob)
{
    //Project cluster_center to ground
    // cluster_center[2] = 0
    geometry_msgs::Pose final_base_pose;
    tf::Quaternion q = armToBaseOrientation(arm_pose);
    final_base_pose.orientation.x = q.x();
    final_base_pose.orientation.y = q.y();
    final_base_pose.orientation.z = q.z();
    final_base_pose.orientation.w = q.w();
    double projected_circle_radius = 1.5;
    Eigen::Vector3d base_pose(arm_pose.position.x, arm_pose.position.y, 0.0);
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,projected_circle_radius);
    std::uniform_real_distribution<double> dist2(0.0,2.0);

    //  Eigen::Vector3d bp(x,y, 0.5);

    for(int i =0; i<100;i++)
    {
        double length = std::sqrt(distribution(generator));

        // Sample point for base location in a circle around the projected cluster_center
        double angle = M_PI * dist2(generator);
        double x = base_pose(0) + length * cos(angle);
        double y = base_pose(1) + length * sin(angle);
        Eigen::Vector3d bp(x,y, 0.75);
        double distance = voxblox_ob.getMapDistance(bp);
        final_base_pose.position.x = bp(0);
        final_base_pose.position.y = bp(1);
        final_base_pose.position.z = bp(2);
        if(distance > 0.7)  return final_base_pose;
    // print('Fatal: Unable to find a possible base location after 100 tries. Maybe try increasing the projected_circle_radius? Voxblox could also be reporting the collision with the ground. This can be fixed by increasing the base_offset')
    }
    ROS_INFO_STREAM("Fatal: Unable to find a possible base location after 100 tries. Maybe try increasing the projected_circle_radius? Voxblox could also be reporting the collision with the ground. This can be fixed by increasing the base_offset");
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_launch");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~"),nh;
    MMPoses baseArmPoses;
    // base_sub = nh.subscribe("/base_goal_poses", 10, &MMPoses::base_cb, &baseArmPoses);
    arm_sub = nh.subscribe("/viewpoints", 10, &MMPoses::arm_cb, &baseArmPoses);
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

    YourPlannerVoxblox voxblox_ob(nh, node_handle);
    

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

    // move_base_msgs::MoveBaseGoal goal_base;
    geometry_msgs::PoseStamped goal_pose = curr_pose;

    
    println("waiting for base and arm poses");
    println(baseArmPoses.arm_poses.poses.empty());
    while(baseArmPoses.arm_poses.poses.empty() && ros::ok())
    {
        println("poses empty? 0 is not empty");
        println(baseArmPoses.arm_poses.poses.empty());
    }

    geometry_msgs::PoseArray visited_arm_poses;
    if(!baseArmPoses.arm_poses.poses.empty())
    {
        // for( auto base_pose : baseArmPoses.base_poses.poses)
        // {
        //     goal_base.target_pose.pose.position.x = base_pose.position.x;
        //     goal_base.target_pose.pose.position.y = base_pose.position.y;
        //     goal_base.target_pose.pose.orientation.x = base_pose.orientation.x;
        //     goal_base.target_pose.pose.orientation.y = base_pose.orientation.y;
        //     goal_base.target_pose.pose.orientation.z = base_pose.orientation.z;
        //     goal_base.target_pose.pose.orientation.w = base_pose.orientation.w;

        //     ROS_INFO_STREAM("Moving Base to goal: "<< base_pose);
        //     move_base_sip(goal_base);
        //     // ros::Duration(0.5).sleep();

            for( auto arm_pose : baseArmPoses.arm_poses.poses)
            {
                geometry_msgs::Pose base_pose = generate_base_location(arm_pose,voxblox_ob);
                move_base_msgs::MoveBaseGoal goal_base;
                goal_base.target_pose.header.frame_id = "map";
                goal_base.target_pose.header.stamp = ros::Time::now();
                goal_base.target_pose.pose.position.x = base_pose.position.x;
                goal_base.target_pose.pose.position.y = base_pose.position.y;
                goal_base.target_pose.pose.position.z = 0.0;
                goal_base.target_pose.pose.orientation.x = base_pose.orientation.x;
                goal_base.target_pose.pose.orientation.y = base_pose.orientation.y;
                goal_base.target_pose.pose.orientation.z = base_pose.orientation.z;
                goal_base.target_pose.pose.orientation.w = base_pose.orientation.w;
                visual_tools.publishCuboid(base_pose, 0.05, 0.05, 0.05, rviz_visual_tools::colors::RED);
                visual_tools.trigger();
                visual_tools.deleteAllMarkers();
                ROS_INFO_STREAM("Moving arm to goal: "<< goal_base.target_pose.pose);
                move_base_sip(goal_base);


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

                move_group_->setGoalOrientationTolerance(0.349066);
                move_group_->setGoalPositionTolerance(0.1);
                move_group_->setPoseTarget(goal_pose);
                visual_tools.publishCuboid(goal_pose.pose, 0.05, 0.05, 0.05);
                visual_tools.trigger();
                visual_tools.deleteAllMarkers();

                success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if(success)
                {
                    ROS_INFO_STREAM("plan success for arm pose" << arm_pose);
                    // visual_tools.trigger();
                    // ros::Duration(0.3).sleep();
                    visual_tools.prompt("Found solution Press 'next'to execute trajectory");
                    move_group_->execute(my_plan);
                    // ros::Duration(1).sleep();
                    visual_tools.prompt("Trajectory executed, Press 'next'to continue");
                }
                // }
                // next_iteration: ;
            }
        // }
    }
    // visual_tools.prompt("Press 'next' to delete markers");

    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
   

    return 0;
}
