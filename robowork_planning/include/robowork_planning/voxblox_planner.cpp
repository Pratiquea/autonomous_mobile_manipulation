
#include <ros/ros.h>
#include <iostream>
#include <array>
#include <voxblox_ros/esdf_server.h>
ros::Subscriber base_sub;
ros::Subscriber arm_sub;


class YourPlannerVoxblox 
{
    public:
        YourPlannerVoxblox(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);
        virtual ~YourPlannerVoxblox() {}
        double getMapDistance(const Eigen::Vector3d& position) const;
        private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // Map!
        voxblox::EsdfServer voxblox_server_;
};

YourPlannerVoxblox::YourPlannerVoxblox(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      voxblox_server_(nh_, nh_private_)
{
    // Optionally load a map saved with the save_map service call in voxblox.
    std::string input_filepath;
    nh_private_.param("voxblox_path", input_filepath, input_filepath);
    if (!input_filepath.empty())
    {
        if (!voxblox_server_.loadMap(input_filepath))
        {
            ROS_ERROR("Couldn't load ESDF map!");
        }
    }
    double robot_radius = 1.0;
    voxblox_server_.setTraversabilityRadius(robot_radius);
    voxblox_server_.publishTraversable();
}


int main(int argc, char* argv[])
{

    ros::init(argc, argv, "voxblox_planning");

    ros::AsyncSpinner spinner(2); // TODO: Examples have 1, but move_group fails with: "Didn't received robot state (joint angles) with recent timestamp within 1 seconds." (see: https://github.com/ros-planning/moveit/issues/868)
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    YourPlannerVoxblox vP(nh, pnh);

    ros::waitForShutdown();
    return 0;
}