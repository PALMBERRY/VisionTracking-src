#pragma once
#include <iostream>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <common/point.h>
#include "abstract_safe_adjust.h"
#include "laser_safe_adjust.h"
#include "ultrasonic_safe_adjust.h"
//#include <robot_type.h>

using namespace NJRobot;




struct Vision_Navigation_RobotState
{
	OrientedPoint after_VN_RobotPose;
	double lave_percent;
	bool can_do;
	bool stop_or_not;
	bool near_flag;
	double near_dist;
};


class VisionNavigation
{
public:
	VisionNavigation(void);
	~VisionNavigation(void);



	LaserSafeAdjust				m_safe_laser_adjust;
	UltrasonicSafeAdjust		m_safe_ultra_adjust;   //add by sunqing

	//set current odometer
	void setCurOdom(const OrientedPoint& odom);

	//set(init) current localize pose
	void setCurPose(const OrientedPoint& pose);

	//get control speed
	void getNavResult(double & v,double& w);
	Vision_Navigation_RobotState getVisionNavigation_RobotState();

	//get current robot pose
	OrientedPoint getCurPose();

	//if vision line detected, set this value
	void setVlineInfo(bool haveline,double dist_left,double dist_right,double ang);

	//if vision landmark detected, set this value
	void setVlandmarkInfo(const Point& p);

	//if set target point
	void setTargetPoint( OrientedPoint target_task);

	//if set initial previous target point = the initial point of the robot
	void setInitialPreviousTarget(OrientedPoint pre_target);

	void setFist_flag(bool start_flag);

	void setInitialPreviousOdom();

	bool get_landmark_flag();
	void setInitLandmark();
	double angle_2point(OrientedPoint A,OrientedPoint B);
	bool whichLandmark();

	//interface: process navigation once, after process(), using getNavResult() to get speed command.
	void process()
	{
		localization();
		navigation();
	}

private:
	OrientedPoint cur_odom_; //current odometer pose in ODOM COORDINATE SYS. [m,m,rad]
	OrientedPoint start_pose_;
	OrientedPoint previous_odom_;
	
	OrientedPoint cur_pose_; //robot pose. localization result

	double speed_v_,speed_w_; //robot speed  [m/s,rad/s]

	bool has_vline_info_; //whether vision line detect and vline_dist_&vline_angle_ are available. NOTE:once data used, set this to FALSE
	double vline_dist_left_; //distance from robot center to line.  [m]
	double vline_dist_right_;
	double vline_dist;
	double vline_angle_; //angle between robot orientation and line. [rad]
	double LineDist;
	double line2line_dist;

	bool task_complete; //if task completed set this FALSE, then if get new target point set this TRUE.
	bool first_flag;

	FILE *fp;
	FILE *fp1;
	int landmark_num;



	bool turned_flag;//= false, 
	double need_turn;
	bool pre_vline_flag;// = false, 
	bool lvbo_flag;// = false, 
	bool emergency_flag;// = false;
	bool stop_can_recovery;
	int emergency_num;
	bool need_detect_landmark;// = false; //是否需要检测标记点
	bool rejudge_flag;
	bool speed_flag;
	bool task_num;
	double dist_speed;
	int Turn_flag, Noline_flag, Move_flag, keep_flag;
	double pre_dist, pre_angle, pre_theta;
	double abs_turn;
	double speed_;
	double all_dist;
	double lave_percent_;

	bool has_vlandmark_info_; //whether vision detect landmark and landmark_point_ is available. NOTE:once data used, set this to FALSE
	Point landmark_point_;    //point of landmark in ROBOT COORDINATE SYS. [m,m]
	Point dirt_landmark_point_; 

	OrientedPoint target_point_; //task, the target point in WORLD/MAP COORDINATE SYS. [m,m]
	OrientedPoint previous_target_point_;
	Point temp_keep;
	OrientedPoint landmark_point_global[100];
	OrientedPoint landmark_point_global_;


	double errorNormalize(double angle1, double angle2);



	

	void localization();

	void navigation();


};

