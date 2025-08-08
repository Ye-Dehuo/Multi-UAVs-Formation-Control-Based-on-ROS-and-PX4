#ifndef FORMATION_CONTROL_H
#define FORMATION_CONTROL_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
// #include <mavros/mavros_plugin.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include <navsat_conversions.h>
#include <std_msgs/Byte.h>
#include <tf/tf.h>
#include <string>

#define KNRM  "\033[0m" // end of color txt set
#define KRED  "\033[31m" // for color txt in console
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;
using namespace RobotLocalization;
using namespace NavsatConversions;

#define TAKEOFF 1
#define MISSION 2
#define RETURN 3
#define LAND 4
#define PI 3.14159

enum VehicleTask
{
    kTakeOff,
    kMission,
    kReturn,
    kLand
};

enum ControlMode
{
    position,
    velocity
};

class FormationController
{
public:
    FormationController(ros::NodeHandle &nodeHandle);
    ~FormationController();

    bool set_offboard();
    
    void initialisation();

    void missionTimer(const ros::TimerEvent &);

    void uavStateCb(const mavros_msgs::State::ConstPtr &msg);

    /**
     * @brief Get UAV current global location
     */
    void globalPosCb(const sensor_msgs::NavSatFix::ConstPtr &msg);

    /**
     * @brief Get leader UAV current global location
     */
    void leaderGlobalPosCb(const sensor_msgs::NavSatFix::ConstPtr &msg);

    /**
     * @brief Get leader UAV current local pose
     */

    void leaderPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    /**
     * @brief Get follower UAV current local pose
     */

    void uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void leaderVelCb(const geometry_msgs::TwistStamped::ConstPtr &msg);

    void userCmdCb(const std_msgs::Byte::ConstPtr &msg);


private:
    ros::NodeHandle _nh; // ros node handle

    ros::Publisher formation_position_pub; // publish desired UAV position in formation, this should be in global coordinate
    ros::Publisher local_position_pub; // publish desired UAV position, this should be in local coordinate
    ros::Publisher setpoint_raw_pub; // for follower's velocity control
    ros::Publisher setpoint_raw_leader_pub;
    
    ros::Subscriber uav_state_sub;
    ros::Subscriber leader_pose_sub; // for all uav to subscribe the pose of leader
    ros::Subscriber uav_pose_sub;
    ros::Subscriber leader_vel_enu_sub; // for followers to subscribe the velocity of leader
    ros::Subscriber uav_global_pos_sub;
    ros::Subscriber leader_global_pos_sub;
    ros::Subscriber user_cmd_sub;

    ros::ServiceClient arming_client; 
    ros::ServiceClient set_mode_client;

    ros::Timer mission_timer;

    mavros_msgs::State uav_current_state;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::PositionTarget setpoint_raw;
    mavros_msgs::PositionTarget setpoint_raw_leader;
    geometry_msgs::TwistStamped leader_vel_enu;
    sensor_msgs::NavSatFix uav_global_pos; // leader global position (lat, long, alt)
    sensor_msgs::NavSatFix leader_global_pos; // leader global position (lat, long, alt)
    geometry_msgs::PoseStamped leader_pose; // leader local pose
    geometry_msgs::PoseStamped uav_pose; // follower local pose
    geometry_msgs::PoseStamped uav_offset; // follower initial position with respect to leader
    geometry_msgs::PoseStamped home; // uav initial position when set offboard control. Compared with takeoff_pos, it's z is 0
    geometry_msgs::PoseStamped takeoff_position; // compared with takeoff_pos, it can be published
    geometry_msgs::PoseStamped home_takeoff_position; // record initial takeoff position, if takeoff more than once
    geometry_msgs::PoseStamped home_takeoff_position_leader; // record leader initial takeoff position
    geometry_msgs::PoseStamped desired_local_pose; // indicates the follower's desired position in local frame with respect to itself
    geometry_msgs::PoseStamped desired_leader_local_pose;
    
    VehicleTask uavTaskState;
    ControlMode ctrlMode;

    bool _initialised;
    bool isLeader;
    bool takeoff_flag;
    bool formation_complete_flag;
    bool goal_flag;
    bool return_flag;
    int mode;

    double _takeoff_height;
    double pub_rate; // mission timer interval
    double home_yaw;
    double roll; double pitch; double yaw;
    double leader_roll; double leader_pitch; double leader_yaw;
    double desired_leader_roll; double desired_leader_pitch; double desired_leader_yaw;
    double offset_angle; // offset angle of the follower's desired position relative to the axis
    double relative_x; // follower desired x position relative to leader
    double relative_y; // follower desired y position relative to leader
    double radius; // formation radius
    double latitude_curr;
    double longitude_curr;
    double leader_latitude_curr;
    double leader_longitude_curr;
    double leader_altitude_curr;
    double leader_utm_ex;
    double leader_utm_ny;
    std::string zone; // function argument holder for LLtoUTM function
    
    Vector3d leader_current_pos; // data of leader_pose is float, data of leader_current_pos is double
    Vector3d current_pos;
    Vector3d takeoff_pos;

    std::string _id; // individual UAV id
    int uav_id;
    int agent_number;

};

#endif
