#include "formation_control.h"

FormationController::FormationController(ros::NodeHandle &nodeHandle) : _nh(nodeHandle) // @suppress("Class members should be properly initialized")
{
    printf("%s[Formation Controller Constructor] Formation Controller is setting up for UAV %d\n", KGRN, uav_id);


    _nh.param<std::string>("agent_id", _id, "uav0");
    _nh.param<double>("publish_rate", pub_rate, 20);
    _nh.param<int>("uav_id", uav_id, 0);
    _nh.param<double>("offset_angle", offset_angle, 0.0);
    _nh.param<int>("agent_number", agent_number, 6);
    _nh.param<double>("radius", radius, 6);
    _nh.param<double>("x_offset", uav_offset.pose.position.x, 0.0);
    _nh.param<double>("y_offset", uav_offset.pose.position.y, 0.0);
    _nh.param<double>("z_offset", uav_offset.pose.position.z, 0.0);
    _nh.param<int>("control_mode", mode, 0); // use velocity control
    _nh.param<double>("home_yaw", home_yaw, 0);
    _nh.param<double>("roll", roll, 0);
    _nh.param<double>("yaw", yaw, 0);
    _nh.param<double>("pitch", pitch, 0);
    _nh.param<double>("leader_roll", leader_roll, 0);
    _nh.param<double>("leader_pitch", leader_pitch, 0);
    _nh.param<double>("leader_yaw", leader_yaw, 0);
    _nh.param<double>("desired_leader_roll", leader_roll, 0);
    _nh.param<double>("desired_leader_pitch", leader_pitch, 0);
    _nh.param<double>("desired_leader_yaw", leader_yaw, 0);
    _nh.param<double>("relative_x", relative_x, 0);
    _nh.param<double>("relative_y", relative_y, 0);
    _nh.param<double>("latitude_curr", latitude_curr, 0);
    _nh.param<double>("longitude_curr", longitude_curr, 0);
    _nh.param<double>("leader_latitude_curr", leader_latitude_curr, 0);
    _nh.param<double>("leader_longitude_curr", leader_longitude_curr, 0);
    _nh.param<double>("leader_altitude_curr", leader_altitude_curr, 0);
    _nh.param<double>("leader_utm_ex", leader_utm_ex, 0);
    _nh.param<double>("leader_utm_ny", leader_utm_ny, 0);
    _nh.param<double>("longitude_curr", longitude_curr, 0);
    _nh.param<double>("longitude_curr", longitude_curr, 0);
    _nh.param<double>("takeoff_height", _takeoff_height, 5.0);
    _nh.param<bool>("initialised", _initialised, false);

    offset_angle = offset_angle*PI/180.0; // DTR

    if(mode == 0){
        ctrlMode = position;
        printf("%suav %d formation mode, position control\n", KGRN, uav_id);
    }
    else if (mode==1){
        ctrlMode = velocity;
        printf("%suav %d following mode, velocity control\n", KGRN, uav_id);
    }

    isLeader = !_id.compare("uav0"); // return 0 if two strings compare equal

    if(isLeader){
        printf("%sI am a leader! My ID is %d\n",KCYN, uav_id);
    }
    else
    {
        printf("%sI am a follower! My ID is %d\n",KYEL,uav_id);
    }


    formation_position_pub = _nh.advertise<geographic_msgs::GeoPoseStamped>(
        "/" + _id + "/mavros/setpoint_position/global", 10);

    /** 
    * @brief Publisher that publishes control position setpoints to Mavros
    */
    local_position_pub = _nh.advertise<geometry_msgs::PoseStamped>(
        "/" + _id + "/mavros/setpoint_position/local", 10);

    setpoint_raw_pub = _nh.advertise<mavros_msgs::PositionTarget>(
        "/" + _id + "/mavros/setpoint_raw/local", 10);
    
    setpoint_raw_leader_pub = _nh.advertise<mavros_msgs::PositionTarget>(
         "/uav0/mavros/setpoint_raw/local", 10);

    /** 
    * @brief Get Mavros State of PX4
    */
    uav_state_sub = _nh.subscribe<mavros_msgs::State>(
        "/" + _id + "/mavros/state", 10, &FormationController::uavStateCb, this);

    leader_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
        "/uav0/mavros/local_position/pose", 10, &FormationController::leaderPoseCb, this); // node /uav0/mavros is publishing to /uav0/mavros/local_position/pose, meanwhile /uav0/mavros and other nodes (/formationX) is subscripting to /uav0/mavros/local_position/pose

    leader_global_pos_sub = _nh.subscribe<sensor_msgs::NavSatFix>(
        "/uav0/mavros/global_position/global", 1, &FormationController::leaderGlobalPosCb, this);// node /uav0/mavros is publishing to /uav0/mavros/global_position/global, meanwhile /uav0/mavros and other nodes (/formationX) is subscripting to /uav0/mavros/global_position/global

    uav_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
        "/" + _id + "/mavros/local_position/pose", 10, &FormationController::uavPoseCb, this);

    leader_vel_enu_sub = _nh.subscribe<geometry_msgs::TwistStamped>(
        "/" + _id + "/mavros/local_position/velocity_local", 10, &FormationController::leaderVelCb, this);

    uav_global_pos_sub = _nh.subscribe<sensor_msgs::NavSatFix>(
        "/" + _id + "/mavros/global_position/global", 1, &FormationController::globalPosCb, this);

    user_cmd_sub = _nh.subscribe<std_msgs::Byte>(
        "/" + _id + "/user", 1, &FormationController::userCmdCb, this);

    /** 
    * @brief Service Client that handles arming in Mavros
    */
    arming_client = _nh.serviceClient<mavros_msgs::CommandBool>(
        "/" + _id + "/mavros/cmd/arming");

    /** 
    * @brief Service Client that handles mode switching in Mavros
    */
    set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>(
        "/" + _id + "/mavros/set_mode");

    mission_timer = _nh.createTimer(ros::Duration(1/pub_rate), &FormationController::missionTimer, this, false, false);


    printf("%s[Formation Controller Constructor] Formation Controller Setup for UAV %d is Ready! \n", KGRN, uav_id);
}

FormationController::~FormationController()
{
}

void FormationController::initialisation()
{
    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now();

    // compute the relative position w.r.t the leader using agent number
    double theta = 2 * PI * (uav_id-1)/(agent_number-1) + offset_angle;
    relative_x = radius * cos(theta);
    relative_y = radius * sin(theta);

    if(!isLeader)
    {
        printf("%sI am UAV %d\n", KGRN, uav_id);
        printf("%sMy relative x is %f\n", KGRN, relative_x);
        printf("%sMy relative y is %f\n\n", KGRN, relative_y);
    }


    // Make Sure FCU is connected, wait for 5s if not connected.
    printf("%s[formation_control.cpp] FCU Connection is %s \n", uav_current_state.connected? KBLU : KRED, uav_current_state.connected? "up" : "down");
    while (ros::ok() && !uav_current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            printf("%s[main.cpp] FCU not connected for 5s. Stop initialisation procedure \n", KRED);
            printf("%s[main.cpp] Check for FCU connection \n", KRED);
            last_request = ros::Time::now();
            return;
        }
    }

    printf("%s[main.cpp] FCU connected! \n", KBLU);
    _initialised = true;
    
    home.pose.position.x = uav_pose.pose.position.x;
    home.pose.position.y = uav_pose.pose.position.y;
    home.pose.position.z = uav_pose.pose.position.z;
    home_yaw = yaw;

    printf("%s[main.cpp] UAV %d Home Pose : \n %s x:%f y:%f z:%f yaw:%f \n",
           KYEL, uav_id, KNRM, home.pose.position.x, home.pose.position.y, home.pose.position.z, home_yaw);

    // Record Initial Takeoff Position
    home_takeoff_position.pose.position.x = uav_pose.pose.position.x;
    home_takeoff_position.pose.position.y = uav_pose.pose.position.y;
    home_takeoff_position.pose.position.z = uav_pose.pose.position.z + _takeoff_height;

    // Record Leader Initial Takeoff Position
    home_takeoff_position_leader.pose.position.x = leader_pose.pose.position.x;
    home_takeoff_position_leader.pose.position.y = leader_pose.pose.position.y;
    home_takeoff_position_leader.pose.position.z = leader_pose.pose.position.z + _takeoff_height;

    return;
}



void FormationController::uavStateCb(const mavros_msgs::State::ConstPtr &msg) // receive uav state before init
{
    uav_current_state = *msg;
}


void FormationController::globalPosCb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    uav_global_pos = *msg;
    latitude_curr = double(uav_global_pos.latitude);
    longitude_curr = double(uav_global_pos.longitude);
}

void FormationController::leaderGlobalPosCb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    leader_global_pos = *msg;
    // leader_latitude_curr = double(leader_global_pos.latitude);
    // leader_longitude_curr = double(leader_global_pos.longitude);
    // leader_altitude_curr = double(leader_global_pos.altitude);
    // LLtoUTM(lat, long, north, east, zone)
    //LLtoUTM(leader_latitude_curr, leader_longitude_curr, leader_utm_ny, leader_utm_ex, zone);
    LLtoUTM(leader_global_pos.latitude, leader_global_pos.longitude, leader_utm_ny, leader_utm_ex, zone);
    // printf("uav %d UTM E is %f\n", uav_id, leader_utm_ex);
    // printf("uav %d UTM N is %f\n", uav_id, leader_utm_ny);
}

void FormationController::leaderPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    leader_pose = *msg;
    leader_current_pos.x() = (double)leader_pose.pose.position.x;
    leader_current_pos.y() = (double)leader_pose.pose.position.y;
    leader_current_pos.z() = (double)leader_pose.pose.position.z;

     // msg received contains attitude info descriped by quaternion, convert it to  rotation matrix
    tf::Quaternion q(
        leader_pose.pose.orientation.x, leader_pose.pose.orientation.y,
        leader_pose.pose.orientation.z, leader_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(leader_roll, leader_pitch, leader_yaw);
}

void FormationController::uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_pose = *msg;

    if (!_initialised)
              FormationController::initialisation();

    current_pos.x() = (double)uav_pose.pose.position.x;
    current_pos.y() = (double)uav_pose.pose.position.y;
    current_pos.z() = (double)uav_pose.pose.position.z;

    tf::Quaternion q(
        uav_pose.pose.orientation.x, uav_pose.pose.orientation.y,
        uav_pose.pose.orientation.z, uav_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

void FormationController::leaderVelCb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    leader_vel_enu = *msg;
}

bool FormationController::set_offboard()
{
    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now();

    takeoff_flag = false;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_position_pub.publish(home);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    bool is_mode_ready = false;
    last_request = ros::Time::now();

    while (!is_mode_ready)
    {
        if (uav_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            printf("%s[main.cpp] UAV %d Try set Offboard \n", KYEL, uav_id);
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                printf("%s[main.cpp] UAV %d Offboard enabled \n", KGRN, uav_id);
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!uav_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                printf("%s[main.cpp] UAV %d Try arm \n", KYEL, uav_id);
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    printf("%s[main.cpp] UAV %d armed! \n", KGRN, uav_id);  
                }
                last_request = ros::Time::now();
            }
        }
        local_position_pub.publish(home); // I think: If the offboard mode cannot be set successfully, continue sending the initial position
        is_mode_ready = (uav_current_state.mode == "OFFBOARD") && uav_current_state.armed;
        ros::spinOnce();
        rate.sleep();
    }

    if (is_mode_ready)
    {
        printf("%s[main.cpp] UAV %d Offboard mode activated! \n", KBLU, uav_id);
    }
    else printf("%s[main.cpp] Offboard mode can't be activated! \n", KRED);

    return is_mode_ready;
}

void FormationController::missionTimer(const ros::TimerEvent &)
{
    switch (uavTaskState)
    {
    case kTakeOff:
        {local_position_pub.publish(takeoff_position);
        //printf("Takeoff height is %f\n", takeoff_position.pose.position.z);
        break;}

    case kMission:
        // compute desired local/global positon if it is follower
        {if(!isLeader)
        {
            // convert leader's current lat and long to UTM positions in meters
            // done in subscirber call back

            // compute ownself's desired UTM position by adding the relative_x, relative_y
            // which was computed during initilization
            // double desired_ex = leader_utm_ex + relative_x;
            // double desired_ny = leader_utm_ny + relative_y;

            // // convert desired current UTM position to lat and long
            // double desired_lat;
            // double desired_long;

            // UTMtoLL(desired_ny, desired_ex, zone, desired_lat,desired_long);

            // // publish the desired global position to /uavX/mavros/setpoint_position/global
            // // using formation_position_pub

            // geographic_msgs::GeoPoseStamped global_pos_desired;
            // global_pos_desired.pose.position.latitude = desired_lat;
            // global_pos_desired.pose.position.longitude = desired_long;
            // global_pos_desired.pose.position.altitude = leader_global_pos.altitude - 47.22; // offset from ellipsoid height to AMSL. ! I think there is a mistake,because global_pos_desired.pose.position.altitude is WGS 84 pos, not ASML pos

            // global_pos_desired.pose.orientation = leader_pose.pose.orientation;
            // formation_position_pub.publish(global_pos_desired);

            switch (ctrlMode)
            {
            case position:
                {

                // followers maintain formation
                double desired_global_pos_x = leader_current_pos.x() + relative_x; // desired_global_pos indicates the follower's desired position in global frame with respect to the leader‘s takeoff_position
                double desired_global_pos_y = leader_current_pos.y() + relative_y;
                desired_local_pose.pose.position.x = desired_global_pos_x - uav_offset.pose.position.x;
                desired_local_pose.pose.position.y = desired_global_pos_y - uav_offset.pose.position.y;
                desired_local_pose.pose.position.z = leader_pose.pose.position.z;
                desired_local_pose.pose.orientation = leader_pose.pose.orientation;
                local_position_pub.publish(desired_local_pose);
                break;}

            case velocity:
                // Simple P-controller, output velocity commands
                {

                // followers maintain formation
                double desired_global_pos_x = leader_current_pos.x() + relative_x;
                double desired_global_pos_y = leader_current_pos.y() + relative_y;
                desired_local_pose.pose.position.x = desired_global_pos_x - uav_offset.pose.position.x;
                desired_local_pose.pose.position.y = desired_global_pos_y - uav_offset.pose.position.y;
                setpoint_raw.position.z = leader_pose.pose.position.z;
                setpoint_raw.velocity.x = (desired_local_pose.pose.position.x - current_pos.x())*1;
                setpoint_raw.velocity.y = (desired_local_pose.pose.position.y - current_pos.y())*1;

                // followers track the leader's movement
                // leader_vel_enu.twist.linear.x*0.8;
                // leader_vel_enu.twist.linear.y*0.8;
                // setpoint_raw.velocity.z = leader_vel_enu.twist.linear.z;

                setpoint_raw.yaw = leader_yaw;
                setpoint_raw.type_mask = 3043; // or 0b101111100011. use position_z, velocity_x,velocity_y and yaw to control
                setpoint_raw.coordinate_frame = 1; // MAV_FRAME_LOCAL_NED
                setpoint_raw_pub.publish(setpoint_raw);
                break;}
            
            default:
                break;
              }
        }

        else if(isLeader)
        {
          if(!formation_complete_flag)
          {
              printf("%s[main.cpp] Leader is waiting for formation to be completed...\n", KCYN);
              ros::Rate rate(20);
              ros::Time start_time = ros::Time::now();
              ros::Duration duration(3.5);

              while (ros::Time::now() - start_time <= duration)
              {
                local_position_pub.publish(takeoff_position);

                ros::spinOnce();
                rate.sleep();
              }
              printf("%s[main.cpp] Formation complete! \n[main.cpp] Move to GOAL... \n", KCYN);
              formation_complete_flag = true;
             }

          else if(formation_complete_flag)
              {
              // Check if its at XY of goal position
              if (!goal_flag && (sqrt(pow(uav_pose.pose.position.x - desired_leader_local_pose.pose.position.x, 2)
                      + pow(uav_pose.pose.position.y - desired_leader_local_pose.pose.position.y, 2))
                      <= 1.0))
              {
                printf("%s[main.cpp] Fomation has arrived GOAL \n", KCYN);
                goal_flag = true;
              }

               setpoint_raw_leader.position.z = desired_leader_local_pose.pose.position.z;
               setpoint_raw_leader.velocity.x = (desired_leader_local_pose.pose.position.x - leader_current_pos.x())*0.14;
               setpoint_raw_leader.velocity.y = (desired_leader_local_pose.pose.position.y - leader_current_pos.y())*0.14;
               setpoint_raw_leader.yaw = desired_leader_yaw;
               setpoint_raw_leader.type_mask = 3043; // or 0b101111100011. use position_z, velocity_x,velocity_y and yaw to control
               setpoint_raw_leader.coordinate_frame = 1; // MAV_FRAME_LOCAL_NED
               setpoint_raw_leader_pub.publish(setpoint_raw_leader);

              }
        }

        break;
        }

    case kReturn:
        {
          if(!isLeader)
          {
            if(!return_flag)
            {
              switch (ctrlMode)
              {
                case position:
                {
                  // Check if its at XY of home position
                  if (!return_flag && (sqrt(pow(leader_pose.pose.position.x - home_takeoff_position_leader.pose.position.x, 2)
                          + pow(leader_pose.pose.position.y - home_takeoff_position_leader.pose.position.y, 2))
                          <= 0.5))
                  {
                    return_flag = true;
                    break;
                  }

                  // followers maintain formation
                  double desired_global_pos_x = leader_current_pos.x() + relative_x;  // desired_global_pos indicates the follower's desired position in global frame with respect to the leader
                  double desired_global_pos_y = leader_current_pos.y() + relative_y;
                  desired_local_pose.pose.position.x = desired_global_pos_x
                      - uav_offset.pose.position.x;
                  desired_local_pose.pose.position.y = desired_global_pos_y
                      - uav_offset.pose.position.y;
                  desired_local_pose.pose.position.z = leader_pose.pose.position.z;
                  desired_local_pose.pose.orientation = leader_pose.pose.orientation;
                  local_position_pub.publish(desired_local_pose);

                  break;
                }

                case velocity:
                  // Simple P-controller, output velocity commands
                {
                  // Check if its at XY of home position
                  if (!return_flag && (sqrt(pow(leader_pose.pose.position.x - home_takeoff_position_leader.pose.position.x, 2)
                          + pow(leader_pose.pose.position.y - home_takeoff_position_leader.pose.position.y, 2))
                          <= 1.0))
                  {
                    return_flag = true;
                    break;
                  }

                  // followers maintain formation
                  double desired_global_pos_x = leader_current_pos.x() + relative_x;
                  double desired_global_pos_y = leader_current_pos.y() + relative_y;
                  desired_local_pose.pose.position.x = desired_global_pos_x
                      - uav_offset.pose.position.x;
                  desired_local_pose.pose.position.y = desired_global_pos_y
                      - uav_offset.pose.position.y;
                  setpoint_raw.position.z = leader_pose.pose.position.z;
                  setpoint_raw.velocity.x = (desired_local_pose.pose.position.x
                      - current_pos.x()) * 1;
                  setpoint_raw.velocity.y = (desired_local_pose.pose.position.y
                      - current_pos.y()) * 1;

                  // followers track the leader's movement
                  // leader_vel_enu.twist.linear.x*0.8;
                  // leader_vel_enu.twist.linear.y*0.8;
                  // setpoint_raw.velocity.z = leader_vel_enu.twist.linear.z;

                  setpoint_raw.yaw = leader_yaw;
                  setpoint_raw.type_mask = 3043;  // or 0b101111100011. use position_z, velocity_x,velocity_y and yaw to control
                  setpoint_raw.coordinate_frame = 1;  // MAV_FRAME_LOCAL_NED
                  setpoint_raw_pub.publish(setpoint_raw);
                  break;
                }

                default:
                  break;
              }
            }

            if(return_flag)
            {
              local_position_pub.publish(home_takeoff_position);
            }

          }

          else if (isLeader)
          {
            // Check if its at XY of home position
            if ((!return_flag && sqrt(pow(uav_pose.pose.position.x- home_takeoff_position.pose.position.x, 2)
                + pow(uav_pose.pose.position.y - home_takeoff_position.pose.position.y, 2))
                <= 1.0))
            {
              printf("%s[main.cpp] Leader uav %d has arrived HOME \n", KBLU, uav_id);
              return_flag = true;
            }

            if (!return_flag)
            {
              setpoint_raw_leader.position.z = home_takeoff_position.pose.position.z;
              setpoint_raw_leader.velocity.x = (home_takeoff_position.pose.position.x - leader_current_pos.x()) * 0.14;
              setpoint_raw_leader.velocity.y = (home_takeoff_position.pose.position.y - leader_current_pos.y()) * 0.14;
              setpoint_raw_leader.yaw = home_yaw;
              setpoint_raw_leader.type_mask = 3043;  // or 0b101111100011. use position_z, velocity_x,velocity_y and yaw to control
              setpoint_raw_leader.coordinate_frame = 1;  // MAV_FRAME_LOCAL_NED
              setpoint_raw_leader_pub.publish(setpoint_raw_leader);
            }

            else if (return_flag)
            {
              local_position_pub.publish(home_takeoff_position);
            }

          }

         break;}

    case kLand:
         {
         break;}

    default:
        break;
   }
}

void FormationController::userCmdCb(const std_msgs::Byte::ConstPtr &msg)
{
    int cmd = msg->data;
    printf("%s[main.cpp] UAV %d command %d received \n", KBLU, uav_id, cmd);

    switch (cmd)
    {
    case TAKEOFF:
    {

        printf("%s[main.cpp] UAV %d Takeoff command received! \n", KYEL, uav_id);
        arm_cmd.request.value = true; // indicates arm cmd has been excuted

        if (set_offboard())
        {
            printf("%s[main.cpp] Offboard mode activated, UAV %d going to run takeoff \n", KBLU, uav_id);
            mission_timer.start();
            printf("%s[main.cpp] UAV %d mission timer started! \n", KGRN, uav_id);
            takeoff_flag = true;
            uavTaskState = kTakeOff;

            // Set Takeoff Position before ever Time Takeoff
            takeoff_pos.x() = uav_pose.pose.position.x;
            takeoff_pos.y() = uav_pose.pose.position.y;
            takeoff_pos.z() = uav_pose.pose.position.z + _takeoff_height;

            takeoff_position.pose.position.x = uav_pose.pose.position.x;
            takeoff_position.pose.position.y = uav_pose.pose.position.y;
            takeoff_position.pose.position.z = uav_pose.pose.position.z + _takeoff_height;

        
            /** @brief Set up Takeoff Waypoints */
            MatrixXd wp = MatrixXd::Zero(3,1);
            wp.col(0) = takeoff_pos;
            printf("%s[main.cpp] [UAV %d Takeoff Waypoint] \n", KBLU, uav_id);
            std::cout << KNRM << wp << std::endl;

            // double clean_buffer = ros::Time::now().toSec();
            // double buffer = 5.0;
            // std::cout << KBLU << "[main.cpp] " << "Takeoff buffer of " << KNRM << buffer << KBLU << "s" << std::endl;
            // double last_interval = ros::Time::now().toSec();
        
            // // while loop to clean out buffer for command for 5s
            // while (abs(clean_buffer - ros::Time::now().toSec()) < buffer)
            // {
            //     // WARNING : Publishing too fast will result in the mavlink bandwidth to be clogged up hence we need to limit this rate
            //     if (ros::Time::now().toSec() - last_interval > _send_desired_interval)  // not define _send_desired_interval in formation_control
            //     {
            //         Vector3d home_pose = {home.pose.position.x, home.pose.position.y, home.pose.position.z};
            //         uavDesiredControlHandler(home_pose,
            //             Vector3d (0,0,0),
            //             Vector3d (0,0,0),
            //             home_yaw);// but there is no defination for uavDesiredControlHandler()
            //         // std::cout << KBLU << "[main.cpp] Publish buffer" << KNRM << home_pose << std::endl;
            //         last_interval = ros::Time::now().toSec();
            //     }

            // }
        }
        else printf("%s[main.cpp] Takeoff command can't excute! \n", KRED);
        break;
    }

    case MISSION:
    {
        if (!takeoff_flag)
        {
            printf("%s[main.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
            break;
        }
        printf("%s[main.cpp] Mission command received! \n", KYEL);
        printf("%s[main.cpp] Loading Trajectory... \n", KBLU);

        formation_complete_flag = false;
        uavTaskState = kMission;

        desired_leader_local_pose.pose.position.x = 6.0;
        desired_leader_local_pose.pose.position.y = 6.0;
        desired_leader_local_pose.pose.position.z = 6.0;
        desired_leader_local_pose.pose.orientation.x = 0.0;
        desired_leader_local_pose.pose.orientation.y = 0.0;
        desired_leader_local_pose.pose.orientation.z = 0.707;
        desired_leader_local_pose.pose.orientation.w= 0.707;

        tf::Quaternion q_goal(
            desired_leader_local_pose.pose.orientation.x, desired_leader_local_pose.pose.orientation.y,
            desired_leader_local_pose.pose.orientation.z, desired_leader_local_pose.pose.orientation.w);
            tf::Matrix3x3 m(q_goal);
            m.getRPY(desired_leader_roll, desired_leader_pitch, desired_leader_yaw);

        break;
    }

    case RETURN:
    {
      if (!takeoff_flag)
      {
          printf("%s[main.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
          break;
       }

      printf("%s[main.cpp] Return command received! \n", KYEL);
      printf("%s[main.cpp] Returning... \n", KBLU);

      return_flag = false;
      goal_flag = false;
      uavTaskState = kReturn;

       break;
    }

    case LAND:
    {
        if (!takeoff_flag)
        {
            printf("%s[main.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
            break;
        }

        if (!return_flag)
        {
            double sqdist_hpos = pow(takeoff_pos.x(),2) + pow(takeoff_pos.y(),2);
            double sqdist_pos = pow(uav_pose.pose.position.x,2) + pow(uav_pose.pose.position.y,2);

            printf("%s[main.cpp] Position not suitable for landing, no RTL enabled, at dist %lf \n. Please return home first \n", KRED, sqrt(sqdist_pos - sqdist_hpos));
            break;
        }

        /** @brief Set up Land Waypoint */
        MatrixXd wp = MatrixXd::Zero(3,1);
        wp.col(0) = Vector3d (home.pose.position.x, home.pose.position.y, home.pose.position.z);
        std::cout << KBLU << "[main.cpp] " << "[Land Waypoint] " << std::endl << KNRM << wp << std::endl;

        printf("%s[main.cpp] UAV %d is landing! \n", KBLU, uav_id);

        uavTaskState = kLand;

        break;
    }

    default:
        break;
    }
}
