//author  : kaidi wang
//date    : 2020.12.20
//descript: master policy decision node 
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/RCIn.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>

//define two variable to store rate data 
geometry_msgs::Point px4_linear_rate_enu;
geometry_msgs::Point px4_euler_rate_enu;

//state callback function
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}


//get rc data callback function
mavros_msgs::RCIn get_rc_channel;//get the data of rc channels
int switch_yaw_xyz[4];// this used to store the number of rc channels
void get_rc_channel_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	get_rc_channel = *msg;

	for(int i =0 ; i<4; i++)
	{
		switch_yaw_xyz[i]=get_rc_channel.channels[i];
	}

    //print the number of rc channels
	ROS_INFO_STREAM("channel 1: "<<switch_yaw_xyz[0]);
	ROS_INFO_STREAM("channel 2: "<<switch_yaw_xyz[1]);
	ROS_INFO_STREAM("channel 3: "<<switch_yaw_xyz[2]);
	ROS_INFO_STREAM("channel 4: "<<switch_yaw_xyz[3]);

}


//local velocity callback function
geometry_msgs::TwistStamped local_vel;
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	local_vel = *msg;
}


//tran PX4 attitude to controller input, velocity
geometry_msgs::Point make_main_velocity(geometry_msgs::TwistStamped local_vel)
{
	geometry_msgs::Point main_velocity;
	main_velocity.x = -local_vel.twist.linear.y;
	main_velocity.y = local_vel.twist.linear.x;
	main_velocity.z = local_vel.twist.linear.z;
	ROS_INFO_STREAM("linear_rate_enu_x: "<< main_velocity.x);
	ROS_INFO_STREAM("linear_rate_enu_y: "<< main_velocity.y);
	ROS_INFO_STREAM("linear_rate_enu_z: "<< main_velocity.z);

	return main_velocity;
}


//tran PX4 attitude to controller input, body_rates
geometry_msgs::Point make_main_body_rates(geometry_msgs::TwistStamped local_vel)
{
	geometry_msgs::Point main_body_rates;
	main_body_rates.x = -local_vel.twist.angular.y;
	main_body_rates.y = local_vel.twist.angular.x;
	main_body_rates.z = local_vel.twist.angular.z;

	ROS_INFO_STREAM("euler_rate_enu_x: "<< main_body_rates.x);
	ROS_INFO_STREAM("euler_rate_enu_y: "<< main_body_rates.y);
	ROS_INFO_STREAM("euler_rate_enu_z: "<< main_body_rates.z);

	return main_body_rates;
}

//main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_rc_att");
	ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		("mavros/state", 10, state_cb);

    ros::Subscriber get_rc_channel_sub = nh.subscribe<mavros_msgs::RCIn>
		("/mavros/rc/in",100,get_rc_channel_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("mavros/local_position/velocity_body",10,local_vel_cb);

    //due to this function is too simple that we don't use class and timer
    ros::Rate rate(100.0);
	//ROS_INFO("current_state_connected is %d",current_state.connected);
	while (ros::ok() && current_state.connected) {
		ROS_INFO_STREAM("unconnected");
		ros::spinOnce();
		rate.sleep();
	}


    //function block
    while (ros::ok()) {
        //get rate data which transformed to enu coordinate system
        px4_linear_rate_enu=make_main_velocity(local_vel);
        px4_euler_rate_enu =make_main_body_rates(local_vel);
        ros::spinOnce();
		rate.sleep();
    }

    return 0;
}