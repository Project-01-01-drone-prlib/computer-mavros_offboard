/**
 * author:kaidi wang
 * mail  :1055080765@qq.com
 * date  :2020.12.8
 * describe: px4 in offboard mode and publish attitude control
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <stdio.h>
#include <math.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main (int argc,char** argv)
{
    ros::init(argc,argv,"pub_setpoints");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);
    
    ros::Publisher pub_att = nh.advertise<geometry_msgs::PoseStamped>
    ("mavros/setpoint_attitude/attitude",100);
    ros::Publisher pub_thr = nh.advertise<mavros_msgs::Thrust>
    ("mavros/setpoint_attitude/thrust",100);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::TwistStamped>
    ("mavros/setpoint_attitude/cmd_vel",100);

    ros::Publisher raw_pub_att = nh.advertise<mavros_msgs::AttitudeTarget>
    ("mavros/setpoint_raw/attitude",100);

    ros::ServiceClient arming_client   = nh.serviceClient<mavros_msgs::CommandBool>
    ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("mavros/set_mode");


    geometry_msgs::PoseStamped cmd_att;
    mavros_msgs::Thrust cmd_thr;
    mavros_msgs::AttitudeTarget raw_att;


    int count = 1;
    double v[3] = {1.0, 0.0, 0.0};
    double lambda = 0.5;

    double v_norm = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    double theta = 0.0;

    ros::Rate rate(100.0);
    while (ros::ok() && !current_state.connected)
    {
        /* wait until px4 connected */
        ROS_INFO_STREAM("wait until px4 connected ");
        ros::spinOnce();
        rate.sleep();
    }
    


    ros::Rate loop_rate(100);

    //set offboard mode and then arm the vechicl
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value=true;

    ros::Time last_request = ros::Time::now();
    while (ros::ok())
    {        
        /* set offboard and then arm the vechile */
        // if (!current_state.armed && current_state.mode != "OFFBOARD")
        // {
        //     if( current_state.mode != "OFFBOARD"
        //      &&(ros::Time::now() - last_request > ros::Duration(5.0)) )
        //     {
        //         if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        //         {
        //             ROS_INFO("Offboard enabled");
        //         }
        //         last_request = ros::Time::now();
        //     } 
        //     else
        //     {
        //         if( !current_state.armed 
        //         &&(ros::Time::now() - last_request > ros::Duration(3.0)))
        //         {
        //             ROS_INFO("arm enabled or not ");
        //             if( arming_client.call(arm_cmd)&&arm_cmd.response.success)
        //             {
        //                 ROS_INFO("Vehicle armed");
        //             }
        //             last_request = ros::Time::now();
        //         }
        //         else
        //         {
        //             //ROS_INFO_STREAM("don't enter the if armed....");
        //         }    
        //     }
        // }
        
        ROS_INFO_STREAM("publish attitude...");
        //Create attitude command message
        cmd_att.header.stamp = ros::Time::now();
        cmd_att.header.seq=count;
        //cmd_att.header.frame_id = 1;
        cmd_att.pose.position.x = 0.0;//0.001*some_object.position_x;
        cmd_att.pose.position.y = 0.0;//0.001*some_object.position_y;
        cmd_att.pose.position.z = 0.0;//0.001*some_object.position_z;

        cmd_att.pose.orientation.x = sin(theta/2.0)*v[0]/v_norm;
        cmd_att.pose.orientation.y = sin(theta/2.0)*v[1]/v_norm;
        cmd_att.pose.orientation.z = sin(theta/2.0)*v[2]/v_norm;
        cmd_att.pose.orientation.w = cos(theta/2.0);

        //Create throttle command message
        cmd_thr.thrust = lambda;


        //publish setpoint raw data 
        raw_att.header.stamp = ros::Time::now();
        raw_att.header.seq = count;
        raw_att.body_rate.x=1;
        raw_att.body_rate.y=2;
        raw_att.body_rate.z=0;
        raw_att.thrust=0.8;
        raw_att.orientation.x=sin(theta/2.0)*v[0]/v_norm;
        raw_att.orientation.y=sin(theta/2.0)*v[1]/v_norm;
        raw_att.orientation.z=sin(theta/2.0)*v[2]/v_norm;
        raw_att.orientation.w=cos(theta/2.0);

        //raw_att.header.frame_id


        pub_att.publish(cmd_att);
        pub_thr.publish(cmd_thr);
        raw_pub_att.publish(raw_att);
        //ROS_INFO_STREAM("publish the attitude target message...");
        
        ros::spinOnce();
        count++;
        theta=0.3*sin(count/300.0);
        // if(count>1000){
        //     count =1;
        // }
        //run ros loop
        loop_rate.sleep();
    }
    
    return 0;
}
