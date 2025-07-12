#include "ugv_control.h"
    
void UGV_CONTROL::init(ros::NodeHandle& nh)
{
    // 智能体类型
    agent_type = sunray_msgs::agent_state::UGV;
    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    // 【参数】智能体IP
    nh.param<std::string>("agent_ip", agent_ip, "192.168.1.1");
    // 【参数】智能体的固定高度
    nh.param<float>("agent_height", agent_height, 0.1);
    // 【参数】智能体位置来源（1：代表动捕、2代表SLAM数据）
    nh.param<int>("pose_source", pose_source, 1);
    // 【参数】是否打印
    nh.param<bool>("flag_printf", flag_printf, true);
    // 【参数】设置获取数据源
    nh.param<int>("pose_source", pose_source, 1);
    // 【参数】悬停控制参数 - xy
    nh.param<float>("ugv_control_param/Kp_xy", ugv_control_param.Kp_xy, 1.4);
    // 【参数】悬停控制参数 - yaw
    nh.param<float>("ugv_control_param/Kp_yaw", ugv_control_param.Kp_yaw, 0.9);
    // 【参数】悬停控制参数 - max_vel_xy
    nh.param<float>("ugv_control_param/max_vel_xy", ugv_control_param.max_vel_xy, 0.5);
    // 【参数】悬停控制参数 - max_vel_yaw
    nh.param<float>("ugv_control_param/max_vel_yaw", ugv_control_param.max_vel_yaw, 50.0/180.0*M_PI);
    // 【参数】地理围栏参数（超出围栏自动降落）
    nh.param<float>("ugv_geo_fence/max_x", ugv_geo_fence.max_x, 100.0);
    nh.param<float>("ugv_geo_fence/min_x", ugv_geo_fence.min_x, -100.0);
    nh.param<float>("ugv_geo_fence/max_y", ugv_geo_fence.max_y, 100.0);
    nh.param<float>("ugv_geo_fence/min_y", ugv_geo_fence.min_y, -100.0);

    agent_name = "ugv_" + std::to_string(agent_id);
    // 根据 pose_source 参数选择数据源
    if (pose_source == 1)
    {
        // 【订阅】订阅动捕的数据(位置+速度) vrpn -> 本节点
        mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ agent_name + "/pose", 1, &UGV_CONTROL::mocap_pos_cb, this);
        mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/"+ agent_name + "/twist", 1, &UGV_CONTROL::mocap_vel_cb, this);
        cout << GREEN << "Pose source: Mocap" << TAIL << endl;
    }
    else if (pose_source == 2)
    {
        // 【订阅】订阅VIOBOT Odom数据
        viobot_odom_sub = nh.subscribe("/sunray/robobaton_mini/odom", 1, &UGV_CONTROL::odom_cb,this);
        cout << GREEN << "Pose source: VIOBOT" << TAIL << endl;
    }
    else
    {
        cout << RED << "Pose source: Unknown" << TAIL << endl;
    }   

    // 【订阅】智能体控制指令 地面站/ORCA等上层算法 -> 本节点
    ugv_cmd_sub = nh.subscribe<sunray_msgs::agent_cmd>("/sunray_swarm/ugv/agent_cmd", 50, &UGV_CONTROL::agnet_cmd_cb, this);
    // 【订阅】智能体控制指令 ORCA算法 -> 本节点
    ugv_cmd_sub = nh.subscribe<sunray_msgs::agent_cmd>("/sunray_swarm/" + agent_name + "/agent_cmd", 10, &UGV_CONTROL::agnet_cmd_cb, this);
    // 【订阅】ugv电池的数据 ugv_driver -> 本节点
    battery_sub = nh.subscribe<std_msgs::Float32>("/sunray_swarm/" + agent_name + "/battery", 1, &UGV_CONTROL::battery_cb, this);  
    
    // 【发布】智能体状态 本节点 -> 地面站/其他节点
    agent_state_pub = nh.advertise<sunray_msgs::agent_state>("/sunray_swarm/" + agent_name + "/agent_state", 1); 
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【发布】控制指令（机体系，单位：米/秒，Rad/秒）本节点 -> ugv_driver
    agent_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/sunray_swarm/" + agent_name + "/cmd_vel", 1); 
    // 【发布】led灯 本节点 -> ugv_driver（暂未启用）
    led_pub = nh.advertise<std_msgs::ColorRGBA>("/sunray_swarm/" + agent_name + "/led", 1);
   
    // 【发布】无人车marker 本节点 -> RVIZ(仿真)
    ugv_mesh_pub = nh.advertise<visualization_msgs::Marker>("/sunray_swarm/" + agent_name + "/mesh", 1);
    // 【发布】无人车运动轨迹  本节点 -> RVIZ(仿真)
    ugv_trajectory_pub = nh.advertise<nav_msgs::Path>("/sunray_swarm/" + agent_name + "/trajectory", 1);
    // 【发布】目标点marker 本节点 -> RVIZ(仿真)
    goal_point_pub = nh.advertise<visualization_msgs::Marker>("/sunray_swarm/" + agent_name + "/goal_point_rviz", 1);
    // 【发布】速度方向 本节点 -> RVIZ(仿真)
    vel_rviz_pub = nh.advertise<geometry_msgs::TwistStamped>("/sunray_swarm/" + agent_name + "/vel_rviz", 10);

    // 【定时器】 定时发布agent_state - 10Hz
    timer_state_pub = nh.createTimer(ros::Duration(0.1), &UGV_CONTROL::timercb_state, this);
    // 【定时器】 定时发布RVIZ显示相关话题(仿真) - 10Hz
    timer_rivz = nh.createTimer(ros::Duration(0.1), &UGV_CONTROL::timercb_rviz, this);
    // 【定时器】 定时打印状态
    timer_debug = nh.createTimer(ros::Duration(3.0), &UGV_CONTROL::timercb_debug, this);

    agent_state.header.stamp = ros::Time::now();
    agent_state.header.frame_id = "world";
    agent_state.agent_type = agent_type;
    agent_state.agent_id = agent_id;
    agent_state.agent_ip = agent_ip;  
    agent_state.connected = false;
    agent_state.odom_valid = false;
    agent_state.pos[0] = 0.0;
    agent_state.pos[1] = 0.0;
    agent_state.pos[2] = agent_height;
    agent_state.vel[0] = 0.0;
    agent_state.vel[1] = 0.0;
    agent_state.vel[2] = 0.0;
    agent_state.att[0] = 0.0;
    agent_state.att[1] = 0.0;
    agent_state.att[2] = 0.0;
    agent_state.attitude_q.x = 0.0;
    agent_state.attitude_q.y = 0.0;
    agent_state.attitude_q.z = 0.0;
    agent_state.attitude_q.w = 1.0;
    agent_state.battery = -1.0;
    agent_state.control_state = sunray_msgs::agent_cmd::INIT;
    current_agent_cmd.control_state = sunray_msgs::agent_cmd::INIT;

    // 根据智能体ID来设置仿真时RVIZ中智能体的颜色，与真机无关
    setup_rviz_color();

    // 打印本节点参数，用于检查
    printf_param();

    node_name = ros::this_node::getName();
    text_info.data = node_name + ": ugv_" + to_string(agent_id) + " init!";
    // text_info_pub.publish(text_info);
    cout << BLUE << text_info.data << TAIL << endl;
}

// 主循环函数
void UGV_CONTROL::mainloop()
{
    // 定位数据丢失情况下，不执行控制指令并直接返回，直到动捕恢复
    if(!agent_state.odom_valid)
    {
        desired_vel.linear.x = 0.0;
        desired_vel.linear.y = 0.0;
        desired_vel.linear.z = 0.0;
        desired_vel.angular.z = 0.0;
        agent_cmd_vel_pub.publish(desired_vel);
        return;
    }

    // 每次进入主循环，先检查无人机是否超出地理围栏，超出的话则不发送任何指令并返回
    if(check_geo_fence())
    {
        return;
    }

    // 根据收到的控制指令进行相关计算，并生成对应的底层控制指令到智能体
    switch (current_agent_cmd.control_state)
    {
        // INIT：不执行任何指令
        case sunray_msgs::agent_cmd::INIT:
            // 初始模式
            // do nothing
            break;
        
        // HOLD：悬停模式，切入该模式的瞬间，无人车在当前位置停止，即发送0速度
        case sunray_msgs::agent_cmd::HOLD:
            // 原地停止
            desired_vel.linear.x = 0.0;
            desired_vel.linear.y = 0.0;
            desired_vel.linear.z = 0.0;
            desired_vel.angular.z = 0.0;
            agent_cmd_vel_pub.publish(desired_vel);
            break;
        
        // POS_CONTROL：位置控制模式，无人车移动到期望的位置+偏航（期望位置由外部指令赋值）
        case sunray_msgs::agent_cmd::POS_CONTROL:
            // 位置控制算法
            pos_control(current_agent_cmd.desired_pos, current_agent_cmd.desired_yaw);
            break;

        // VEL_CONTROL_BODY：车体系速度控制，无人车按照期望的速度在车体系移动（期望速度由外部指令赋值）
        case sunray_msgs::agent_cmd::VEL_CONTROL_BODY:
            // 控制指令限幅（防止外部指令给了一个很大的数）
            desired_vel.linear.x = constrain_function(current_agent_cmd.desired_vel.linear.x, ugv_control_param.max_vel_xy, 0.0);
            desired_vel.linear.y = constrain_function(current_agent_cmd.desired_vel.linear.y, ugv_control_param.max_vel_xy, 0.0);
            desired_vel.angular.z = constrain_function(current_agent_cmd.desired_vel.angular.z, ugv_control_param.max_vel_yaw, 0.01);
            agent_cmd_vel_pub.publish(desired_vel);        
            break;

        // VEL_CONTROL_ENU：惯性系速度控制，无人车按照期望的速度在惯性系移动（期望速度由外部指令赋值）
        case sunray_msgs::agent_cmd::VEL_CONTROL_ENU:
            // 由于UGV底层控制指令为车体系，所以需要将收到的惯性系速度转换为车体系速度
            desired_vel = enu_to_body(current_agent_cmd.desired_vel);
            agent_cmd_vel_pub.publish(desired_vel);
            break;  

        default:
            break;
    }

    agent_state_last = agent_state;
}

void UGV_CONTROL::agnet_cmd_cb(const sunray_msgs::agent_cmd::ConstPtr& msg)
{
    // 判断指令ID是否正确，否则不接收该指令
    if(msg->agent_id != agent_id && msg->agent_id != 99)
    {
        return;
    }

    current_agent_cmd = *msg; 

    switch(msg->control_state) 
    {
        // 收到INIT指令
        case sunray_msgs::agent_cmd::INIT:
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get agent_cmd: INIT!";
            cout << BLUE << text_info.data << TAIL << endl;
            break;
        // 收到HOLD指令
        case sunray_msgs::agent_cmd::HOLD:
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get agent_cmd: HOLD!";
            cout << BLUE << text_info.data << TAIL << endl;
            break;
        // 收到POS_CONTROL指令
        case sunray_msgs::agent_cmd::POS_CONTROL:
            desired_position.x = msg->desired_pos.x;
            desired_position.y = msg->desired_pos.y;
            desired_position.z = agent_height;
            desired_yaw = msg->desired_yaw;
            //  text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get agent_cmd: POS_CONTROL!";
            //  cout << BLUE << text_info.data << TAIL << endl;
            break;
        // 收到VEL_CONTROL_BODY指令：此处不做任何处理，在主循环中处理
        case sunray_msgs::agent_cmd::VEL_CONTROL_BODY:  
            //  text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get agent_cmd: VEL_CONTROL_BODY!";
            //  cout << BLUE << text_info.data << TAIL << endl;
            break;
        // 收到VEL_CONTROL_ENU指令：此处不做任何处理，在主循环中处理
        case sunray_msgs::agent_cmd::VEL_CONTROL_ENU:
            //  text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get agent_cmd: VEL_CONTROL_ENU!";
            //  cout << BLUE << text_info.data << TAIL << endl;
            break;
        default:
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get agent_cmd: Wrong!";
            cout << RED << text_info.data << TAIL << endl;
            return;
            break;
    }
    // text_info_pub.publish(text_info);
}

// 惯性系->机体系
geometry_msgs::Twist UGV_CONTROL::enu_to_body(geometry_msgs::Twist enu_cmd)
{
    geometry_msgs::Twist body_cmd;
    // 惯性系 -> body frame 
    float cmd_body[2];
    float cmd_enu[2];
    cmd_enu[0] = enu_cmd.linear.x;
    cmd_enu[1] = enu_cmd.linear.y;
    rotation_yaw(agent_state.att[2], cmd_body, cmd_enu);   
    body_cmd.linear.x = cmd_body[0];
    body_cmd.linear.y = cmd_body[1];
    body_cmd.linear.z = 0.0;
    body_cmd.angular.x = 0.0;
    body_cmd.angular.y = 0.0;
    // 控制指令计算：使用简易P控制 - YAW
    double yaw_error = get_yaw_error(current_agent_cmd.desired_yaw, agent_state.att[2]);
    body_cmd.angular.z = yaw_error * ugv_control_param.Kp_yaw;

    // 控制指令限幅
    body_cmd.linear.x = constrain_function(body_cmd.linear.x, ugv_control_param.max_vel_xy, 0.0);
    body_cmd.linear.y = constrain_function(body_cmd.linear.y, ugv_control_param.max_vel_xy, 0.0);
    body_cmd.angular.z = constrain_function(body_cmd.angular.z, ugv_control_param.max_vel_yaw, 0.01);

    return body_cmd;
}

void UGV_CONTROL::set_desired_position()
{
    desired_position.x = agent_state.pos[0];
    desired_position.y = agent_state.pos[1];
    desired_position.z = agent_height;
}

double UGV_CONTROL::get_yaw_error(double yaw_ref, double yaw_now)
{
    double error = yaw_ref - yaw_now;

    if(error > M_PI)
    {
        error = error - 2*M_PI;
    }else if(error < -M_PI)
    {
        error = error + 2*M_PI;
    }

    return error;
}

// 位置控制算法
void UGV_CONTROL::pos_control(geometry_msgs::Point pos_ref, double yaw_ref)
{
    float cmd_body[2];
    float cmd_enu[2];
    // 控制指令计算：使用简易P控制 - XY
    cmd_enu[0] = (pos_ref.x - agent_state.pos[0]) * ugv_control_param.Kp_xy;
    cmd_enu[1] = (pos_ref.y - agent_state.pos[1]) * ugv_control_param.Kp_xy;
    // 惯性系 -> body frame
    rotation_yaw(agent_state.att[2], cmd_body, cmd_enu);             
    desired_vel.linear.x = cmd_body[0];
    desired_vel.linear.y = cmd_body[1];
    desired_vel.linear.z = 0.0;
    // YAW误差计算
    double yaw_error = get_yaw_error(yaw_ref, agent_state.att[2]);
    // 控制指令计算：使用简易P控制 - YAW
    desired_vel.angular.z = yaw_error * ugv_control_param.Kp_yaw;

    // 控制指令限幅
    desired_vel.linear.x = constrain_function(desired_vel.linear.x, ugv_control_param.max_vel_xy, 0.0);
    desired_vel.linear.y = constrain_function(desired_vel.linear.y, ugv_control_param.max_vel_xy, 0.0);
    desired_vel.angular.z = constrain_function(desired_vel.angular.z, ugv_control_param.max_vel_yaw, 0.01);

    // 发布控制指令
    agent_cmd_vel_pub.publish(desired_vel);
}

// 【坐标系旋转函数】- enu系到body系
void UGV_CONTROL::rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2])
{
    body_frame[0] = enu_frame[0] * cos(yaw_angle) + enu_frame[1] * sin(yaw_angle);
    body_frame[1] = -enu_frame[0] * sin(yaw_angle) + enu_frame[1] * cos(yaw_angle);
}

// 定时器回调函数：定时打印
void UGV_CONTROL::timercb_debug(const ros::TimerEvent &e)
{
    if(!flag_printf)
    {
        return;
    }
    cout << GREEN << ">>>>>>>>>>>>>> UGV [" << agent_id << "] Control ";
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    if(agent_type == 0)
    {
        if(agent_state.battery < 15.0f)
        {
            cout << RED << "Battery: " << agent_state.battery << " [%] <<<<<<<<<<<<<" << TAIL << endl;
        }else if(agent_state.battery < 30.0f)
        {
            cout << YELLOW << "Battery: " << agent_state.battery << " [%] <<<<<<<<<<<<<" << TAIL << endl;
        }else
        {
            cout << GREEN << "Battery: " << agent_state.battery << " [%] <<<<<<<<<<<<<" << TAIL << endl;
        }
    }


    if(agent_type == 1 || agent_type == 2)
    {
        if(11.3f < agent_state.battery < 13.0f)
        {
            cout << GREEN << "Battery: " << agent_state.battery << " [V] <<<<<<<<<<<<<" << TAIL << endl;
        }else if(11.0f < agent_state.battery < 11.3f)
        {
            cout << YELLOW << "Battery: " << agent_state.battery << " [V] <<<<<<<<<<<<<" << TAIL << endl;
        }else
        {
            cout << RED << "Low Battery: " << agent_state.battery << " [V] <<<<<<<<<<<<<" << TAIL << endl;
        }
    }

    cout << GREEN << "UAV_pos [X Y] : " << agent_state.pos[0] << " [ m ] " << agent_state.pos[1] << " [ m ] " << TAIL << endl;
    cout << GREEN << "UAV_vel [X Y] : " << agent_state.vel[0] << " [m/s] " << agent_state.vel[1] << " [m/s] " << TAIL << endl;
    cout << GREEN << "UAV_att [Yaw] : " << agent_state.att[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    // 动捕丢失情况下，不执行控制指令，直到动捕恢复
    if(!agent_state.odom_valid)
    {
        cout << RED << "Odom_valid: [Invalid] <<<<<<<<<<<<<" << TAIL << endl;
        return;
    }

    //集群控制命令状态打印
    cout << GREEN << "CMD_SOURCE : [ " << current_agent_cmd.cmd_source   << " ] " << TAIL << endl;
    if (current_agent_cmd.control_state == sunray_msgs::agent_cmd::INIT)
    {
        cout << GREEN << "CONTROL_STATE : [ INIT ]" << TAIL << endl;
    }
    else if (current_agent_cmd.control_state == sunray_msgs::agent_cmd::HOLD)
    {
        cout << GREEN << "CONTROL_STATE : [ HOLD ]" << TAIL << endl;
    }
    else if (current_agent_cmd.control_state == sunray_msgs::agent_cmd::POS_CONTROL)
    {
        cout << GREEN << "CONTROL_STATE : [ POS_CONTROL ]" << TAIL << endl;
        cout << GREEN << "POS_REF [X Y] : " << desired_position.x << " [ m ] " << desired_position.y << " [ m ] " << TAIL << endl;
        cout << GREEN << "YAW_REF       : " << desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;    
        cout << GREEN << "CMD_PUB [X Y] : " << desired_vel.linear.x   << " [m/s] " << desired_vel.linear.y  << " [m/s] " << TAIL << endl;
        cout << GREEN << "CMD_PUB [Yaw] : " << desired_vel.angular.z * 180 / M_PI << " [deg/s] " << TAIL << endl;
    }
    else if (current_agent_cmd.control_state == sunray_msgs::agent_cmd::VEL_CONTROL_BODY)
    {
        cout << GREEN << "CONTROL_STATE : [ VEL_CONTROL_BODY ]" << TAIL << endl;
        cout << GREEN << "CMD_PUB [X Y] : " << desired_vel.linear.x   << " [m/s] " << desired_vel.linear.y  << " [m/s] " << TAIL << endl;
        cout << GREEN << "CMD_PUB [Yaw] : " << desired_vel.angular.z * 180 / M_PI << " [deg/s] " << TAIL << endl;
    }
    else if (current_agent_cmd.control_state == sunray_msgs::agent_cmd::VEL_CONTROL_ENU)
    {
        cout << GREEN << "CONTROL_STATE : [ VEL_CONTROL_ENU ]" << TAIL << endl;
        cout << GREEN << "CMD_ENU [X Y] : " << current_agent_cmd.desired_vel.linear.x   << " [m/s] " << current_agent_cmd.desired_vel.linear.y  << " [m/s] " << TAIL << endl;
        cout << GREEN << "CMD_PUB [X Y] : " << desired_vel.linear.x   << " [m/s] " << desired_vel.linear.y  << " [m/s] " << TAIL << endl;
        cout << GREEN << "CMD_PUB [Yaw] : " << desired_vel.angular.z * 180 / M_PI << " [deg/s] " << TAIL << endl;
    }
    else
    {
        cout << RED << "CONTROL_STATE : [ ERROR ]" << TAIL << endl;
    }
}

// 定时器回调函数：定时发布agent_state
void UGV_CONTROL::timercb_state(const ros::TimerEvent &e)
{
    // 发布 agent_state
    agent_state.header.stamp = ros::Time::now();
    
    // 如果电池数据获取超时1秒，则认为智能体driver挂了
    if((ros::Time::now() - get_battery_time).toSec() > 1.0)
    {
        agent_state.connected = false;
    }

    // 如果位姿数据获取超时，则认为odom失效了
    if((ros::Time::now() - get_odom_time).toSec() > ODOM_TIMEOUT)
    {
        agent_state.odom_valid = false;
    }

    agent_state.control_state = current_agent_cmd.control_state;
    agent_state_pub.publish(agent_state);
}

// 回调函数：动捕
void UGV_CONTROL::mocap_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    get_odom_time = ros::Time::now(); // 记录时间戳，防止超时
	agent_state.pos[0] = msg->pose.position.x;
    agent_state.pos[1] = msg->pose.position.y;
	agent_state.pos[2] = agent_height;
    agent_state.attitude_q = msg->pose.orientation;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d agent_att = quaternion_to_euler(q_mocap);

	agent_state.att[0] = agent_att.x();
    agent_state.att[1] = agent_att.y();
	agent_state.att[2] = agent_att.z();

    agent_state.odom_valid = true;
}

// 回调函数：动捕
void UGV_CONTROL::mocap_vel_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
	agent_state.vel[0] = msg->twist.linear.x;
    agent_state.vel[1] = msg->twist.linear.y;
	agent_state.vel[2] = msg->twist.linear.z;
}

// 回调函数：VIOBOT ODOM
void UGV_CONTROL::odom_cb(const nav_msgs::OdometryConstPtr& msg)
{
    get_odom_time = ros::Time::now(); // 记录时间戳，防止超时
	agent_state.pos[0] = msg->pose.pose.position.x;
    agent_state.pos[1] = msg->pose.pose.position.y;
	agent_state.pos[2] = agent_height;
	agent_state.vel[0] = msg->twist.twist.linear.x;
    agent_state.vel[1] = msg->twist.twist.linear.y;
	agent_state.vel[2] = msg->twist.twist.linear.z;
    agent_state.attitude_q = msg->pose.pose.orientation;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d agent_att = quaternion_to_euler(q_mocap);

	agent_state.att[0] = agent_att.x();
    agent_state.att[1] = agent_att.y();
	agent_state.att[2] = agent_att.z();

    agent_state.odom_valid = true;
}

// 回调函数：电池电量
void UGV_CONTROL::battery_cb(const std_msgs::Float32ConstPtr& msg)
{
    // 记录获取电池（从驱动）的时间，用于判断智能体驱动是否正常
    get_battery_time = ros::Time::now();
    agent_state.connected = true;
    agent_state.battery = msg->data;
}

// 根据智能体ID来设置仿真时RVIZ中智能体的颜色，与真机无关
void UGV_CONTROL::setup_rviz_color()
{
    led_color.a = 1.0;
    switch(agent_id) 
    {
        case 1:
            led_color.r = 1.0;
            led_color.g = 0.0;
            led_color.b = 0.0;
            break;
        case 2:
            led_color.r = 0.0;
            led_color.g = 1.0;
            led_color.b = 0.0;
            break;
        case 3:
            led_color.r = 0.0;
            led_color.g = 0.0;
            led_color.b = 1.0;
            break;
        case 4:
            led_color.r = 1.0;
            led_color.g = 1.0;
            led_color.b = 0.0;
            break;
        case 5:
            led_color.r = 1.0;
            led_color.g = 0.0;
            led_color.b = 1.0;
            break;
        case 6:
            led_color.r = 0.0;
            led_color.g = 1.0;
            led_color.b = 1.0;
            break;
        case 7:
            led_color.r = 0.5;
            led_color.g = 0.5;
            led_color.b = 0.5;
            break;
        case 8:
            led_color.r = 0.3;
            led_color.g = 0.7;
            led_color.b = 0.2;
            break;
        default:
            led_color.r = 1.0;
            led_color.g = 1.0;
            led_color.b = 1.0;
            break;
    }
    // led_color.r = int(led_color.r * 255);
    // led_color.g = int(led_color.g * 255);
    // led_color.b = int(led_color.b * 255);
}

void UGV_CONTROL::setup_led()
{
    std_msgs::ColorRGBA ugv_led;
    ugv_led.r = int(led_color.r * 255);
    ugv_led.g = int(led_color.g * 255);
    ugv_led.b = int(led_color.b * 255);
    ugv_led.a = led_color.a;
    led_pub.publish(ugv_led);
}

// 定时器回调函数 - 定时发送RVIZ显示数据（仿真）
void UGV_CONTROL::timercb_rviz(const ros::TimerEvent &e)
{
    // 发布智能机位置marker
    visualization_msgs::Marker ugv_marker;
    ugv_marker.header.frame_id = "world";
    ugv_marker.header.stamp = ros::Time::now();
    ugv_marker.ns = "mesh";
    ugv_marker.id = 0;
    ugv_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    ugv_marker.scale.x = 0.05;  
    ugv_marker.scale.y = 0.05;  
    ugv_marker.scale.z = 0.05;  
    ugv_marker.action = visualization_msgs::Marker::ADD;
    ugv_marker.pose.position.x = agent_state.pos[0];
    ugv_marker.pose.position.y = agent_state.pos[1];
    ugv_marker.pose.position.z = agent_state.pos[2];
    ugv_marker.pose.orientation.w = agent_state.attitude_q.w;
    ugv_marker.pose.orientation.x = agent_state.attitude_q.x;
    ugv_marker.pose.orientation.y = agent_state.attitude_q.y;
    ugv_marker.pose.orientation.z = agent_state.attitude_q.z;
    ugv_marker.color = led_color;
    ugv_marker.mesh_use_embedded_materials = false;
    ugv_marker.mesh_resource = std::string("package://sunray_swarm/meshes/wheeltec.dae");
    ugv_mesh_pub.publish(ugv_marker);

    // 发布智能体运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped ugv_pos;
    ugv_pos.header.stamp = ros::Time::now();
    ugv_pos.header.frame_id = "world";
    ugv_pos.pose.position.x = agent_state.pos[0];
    ugv_pos.pose.position.y = agent_state.pos[1];
    ugv_pos.pose.position.z = agent_state.pos[2];
    ugv_pos.pose.orientation = agent_state.attitude_q;
    pos_vector.insert(pos_vector.begin(), ugv_pos);
    if (pos_vector.size() > TRA_WINDOW)
    {
        pos_vector.pop_back();
    }
    nav_msgs::Path ugv_trajectory;
    ugv_trajectory.header.stamp = ros::Time::now();
    ugv_trajectory.header.frame_id = "world";
    ugv_trajectory.poses = pos_vector;
    ugv_trajectory_pub.publish(ugv_trajectory);

    // 发布当前执行速度的方向箭头
    geometry_msgs::TwistStamped vel_rviz;
    vel_rviz.header.stamp = ros::Time::now();
    vel_rviz.header.frame_id = agent_name + "/base_link";
    vel_rviz.twist.linear.x = desired_vel.linear.x;
    vel_rviz.twist.linear.y = desired_vel.linear.y;
    vel_rviz.twist.linear.z = desired_vel.linear.z;
    vel_rviz.twist.angular.x = desired_vel.angular.x;
    vel_rviz.twist.angular.y = desired_vel.angular.y;
    vel_rviz.twist.angular.z = desired_vel.angular.z;
    vel_rviz_pub.publish(vel_rviz);

    // 发布当前目标点marker，仅针对位置控制模式
    if(current_agent_cmd.control_state == sunray_msgs::agent_cmd::POS_CONTROL)
    {
        // 发布目标点mesh
        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = "world";
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.ns = "goal";
        goal_marker.id = agent_id;
        goal_marker.type = visualization_msgs::Marker::SPHERE;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.position.x = current_agent_cmd.desired_pos.x;
        goal_marker.pose.position.y = current_agent_cmd.desired_pos.y;
        goal_marker.pose.position.z = agent_height;
        goal_marker.pose.orientation.x = 0.0;
        goal_marker.pose.orientation.y = 0.0;
        goal_marker.pose.orientation.z = 0.0;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = 0.2;
        goal_marker.scale.y = 0.2;
        goal_marker.scale.z = 0.2;
        goal_marker.color = led_color;
        goal_marker.mesh_use_embedded_materials = false;
        goal_point_pub.publish(goal_marker);
    }


    // 发布TF用于RVIZ显示
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";       //相对于世界坐标系
    tfs.header.stamp = ros::Time::now(); //时间戳
    //  |----坐标系 ID
    tfs.child_frame_id = "ugv_" + std::to_string(agent_id) + "/base_link"; //子坐标系，无人机的坐标系
    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = agent_state.pos[0];
    tfs.transform.translation.y = agent_state.pos[1];
    tfs.transform.translation.z = agent_state.pos[2];
    //  |--------- 四元数设置
    tfs.transform.rotation = agent_state.attitude_q;
    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);  
}

// 限制幅度函数
float UGV_CONTROL::constrain_function(float data, float Max, float Min)
{
    if(abs(data)>Max)
    {
        return (data > 0) ? Max : -Max;
    }else if(abs(data)<Min)
    {
        return 0.0;
    }
    else
    {
        return data;
    }
}

// 地理围栏检查函数
bool UGV_CONTROL::check_geo_fence()
{
    // 安全检查，超出地理围栏自动降落,打印相关位置信息
    if (agent_state.pos[0] > ugv_geo_fence.max_x || agent_state.pos[0] < ugv_geo_fence.min_x || 
        agent_state.pos[1] > ugv_geo_fence.max_y || agent_state.pos[1] < ugv_geo_fence.min_y)
    {
        ROS_WARN_STREAM("ugv [" << agent_id << "] out of geofence land! Position: [" 
                        << agent_state.pos[0] << ", " << agent_state.pos[1] << ", " 
                        << agent_state.pos[2] << "], Geofence: ["
                        << ugv_geo_fence.min_x << ", " << ugv_geo_fence.max_x << ", "
                        << ugv_geo_fence.min_y << ", " << ugv_geo_fence.max_y << "]");
        return 1;
    }
    return 0;
}

// 四元数转欧拉角
Eigen::Vector3d UGV_CONTROL::quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

// 打印参数
void UGV_CONTROL::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>> UGV_CONTROL Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;

    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        cout << GREEN << "agent_type : RMTT" << TAIL << endl;
    }else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        cout << GREEN << "agent_type : UGV" << TAIL << endl;
    }else
    {
        cout << GREEN << "agent_type : UNKONWN" << TAIL << endl;
    }    
    cout << GREEN << "agent_id : " << agent_id << "" << TAIL << endl;
    cout << GREEN << "agent_ip : " << agent_ip << "" << TAIL << endl;
    cout << GREEN << "flag_printf : " << flag_printf << "" << TAIL << endl;
    cout << GREEN << "agent_height : " << agent_height << TAIL << endl;

    // 悬停控制参数
    cout << GREEN << "Kp_xy : " << ugv_control_param.Kp_xy << TAIL << endl;
    cout << GREEN << "Kp_yaw : " << ugv_control_param.Kp_yaw << TAIL << endl;
    cout << GREEN << "max_vel_xy : " << ugv_control_param.max_vel_xy << " [m/s]" << TAIL << endl;
    cout << GREEN << "max_vel_yaw : " << ugv_control_param.max_vel_yaw << " [rad/s]" << TAIL << endl;

    // 地理围栏参数
    cout << GREEN << "geo_fence max_x : " << ugv_geo_fence.max_x << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence min_x : " << ugv_geo_fence.min_x << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence max_y : " << ugv_geo_fence.max_y << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence min_y : " << ugv_geo_fence.min_y << " [m]" << TAIL << endl;
}


