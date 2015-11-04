#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include "common/utils.h"
#include "common/timer.h"
#include "chassis/chassis.h"
#include "messages/MessageInteraction.h"
#include "map/map_convert.h"
#include "map/traval_map.h"
#include "io/param_reader.h"
#include <highgui.h>
#include <time.h>
#include "math.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include "vision_navigation.h"
#include "LineDetect/LineDetect.h"

#define Cam2CenterDist	345
#define Center_Sick 340
#define Smooth_para	0.001
//#define Speed_A	0.01

FILE *fp2;

double Speed_A;
// camera para
//const double para[5] = { 0.6000,   -0.8000,    0.0010,    0.0005,  327.1323 };
//
//const double cameraPara[] = { 494.3439,         0,  		349.0357,
//0,  		491.5964,  		230.9781,
//0, 				0, 			1.0000 };
//const double distorPara[] = { 0,   0,   0,    0 };

//double cameraPara[];
//double para[];
//double distorPara[];

std::vector<double> cameraPara;
std::vector<double> para;
std::vector<double> distorPara;

LineDetector lineDetector;


IplImage* pFrame;//获取摄像头
CvCapture* pCapture;
int index; //当前处理的图像标号，每次循环加一
Result curResult;

//////////////////////////////////////////////////////////////////////////

//HANDLE th;
//unsigned int thread_id;

///////// global variable ///////////////////////////////////
// 进程外部通讯
bool g_flag_enable_all = false; //是否开启当前模块
bool task_completed_flag = false;
bool nothing_flag = false;
bool man_stop_flag = false;

double percent_int = 0;
boost::mutex g_mtx_flag_enable_all;

OrientedPoint g_odom;
boost::mutex g_mtx_odom;

OrientedPoint g_task_point;
boost::mutex g_mtx_task_point;

OrientedPoint g_pose;  //当前机器人的位姿，当循线模块结束时，需要将该位姿告知外部
Vision_Navigation_RobotState State_after_vn;	// 视觉导航结束后发布机器人状态
Vision_Navigation_RobotState State_nothing;
boost::mutex g_mtx_pose;

boost::mutex g_mutex_laser;

OrientedPoint g_init_pose; //开始定位的时候，初始位置

// 进程内部线程通讯
bool g_has_vline = false;   //视觉直线
double g_vline_dist_left, g_vline_dist_right;
double g_vline_angle;
boost::mutex g_mtx_vline;

bool g_has_vlandmark = false; //视觉标记点
Point g_vlandmark_point;
bool g_need_detect_landmark = false; //是否需要检测标记点
boost::mutex g_mtx_vlandmark;


// 停障
bool recovery_motion_flag = false;	// 恢复运动后只需发布一次‘运动恢复’信号
bool _speed_flag = false;	// 减速
bool stop_flag = false;		// 急停
double stop_dist;			// 停障处理距离

// 超声
boost::mutex g_mtx_ultrsonic;
Mesg_StopObstacle m_stop_obs;
ObstacleData Uldata;
PointList Laserdata;
double Uldata_l[5],Uldata_f[5],Uldata_r[5];




// 速度平滑

double pre_v,pub_this_v;
boost::mutex g_speed_smooth;
RobotSpeed cur_speed;
RobotSpeed pre_speed;



OrientedPoint start_point;
OrientedPoint target;

// others
VisionNavigation g_nav;

bool g_vis_thread_created = false;
bool g_nav_thread_created = false;

//////////////////////////////////////////////////////////////////////////

unsigned __stdcall visionThread(void *p); //图像处理线程
unsigned __stdcall navigationThread(void *p); //导航线程


bool moduleEnable();



void Set_VisionNavi_Param(void)
{
	NRF_ParamReader::Instance()->readParams(PATH_PARAM.c_str());	// "./params/NR_AGV_param.xml"
	double VehicleHalfLength = 0.1;
	double VehicleHalfWidth  = 0.2;
	{
		DECLARE_PARAM_READER_BEGIN(Chassic)
		READ_PARAM(VehicleHalfLength)
		READ_PARAM(VehicleHalfWidth)
		DECLARE_PARAM_READER_END
	}
	g_nav.m_safe_laser_adjust.setVehicleShape(VehicleHalfLength*2.0,VehicleHalfWidth*2.0);
	//printf("VisionNavigation setVehicleShape init Length:%lf Width:%lf\n",VehicleHalfLength,VehicleHalfWidth);

	string CamExtrinPara;
	string CamIntrinMat;
	string CamDistorPara;
	{
		DECLARE_PARAM_READER_BEGIN(Sensor)
		READ_PARAM(CamIntrinMat)
		READ_PARAM(CamDistorPara)
		READ_PARAM(CamExtrinPara)
		DECLARE_PARAM_READER_END
	
		cameraPara = str2vec(CamExtrinPara);
		para	   = str2vec(CamIntrinMat);
		distorPara = str2vec(CamDistorPara);
		//printf("VisionNavigation Set_Camera param init\n");
		//for(int p=0; p<cameraPara.size();++p)
		//std::cout<<"camerPara:"<<cameraPara[p]<<std::endl;
	}	
}

// 更新激光数据
void updateLaser(Mesg_Laser ldata)
{ 
	if(g_mutex_laser.try_lock())
	{
		// access laser data
		if(g_flag_enable_all)
			Laserdata = transLaser(ldata);
		Laserdata = NRF_Chassis::Instance()->transform2robotFrame(Laserdata);
		g_mutex_laser.unlock();
	}
}

// 订阅超声数据
void updateUltrsonic(Mesg_StopObstacle stop_obs_data)
{
	if(g_mtx_ultrsonic.try_lock())
	{
		m_stop_obs = stop_obs_data;
		if(stop_obs_data.points_size()>3){
			static std::vector<MedFilter<double> > filter(3,MedFilter<double>(5));
			for(int i=0;i<3;i++){
				filter[i].input(stop_obs_data.points(i).x());
				m_stop_obs.mutable_points(i)->set_x(filter[i].output());
				m_stop_obs.mutable_points(i)->set_y(0);
			}
		}
		Uldata = transStopObs(m_stop_obs);
		g_mtx_ultrsonic.unlock();
	}
}


// 订阅里程计数据
void updateOdometry(Mesg_RobotState state_odata)
{
	//if(g_flag_enable_all && g_mtx_odom.try_lock())
	if(g_mtx_odom.try_lock())
	{
		g_odom.x = state_odata.x();
		g_odom.y = state_odata.y();
		g_odom.theta = state_odata.theta();
		g_mtx_odom.unlock();
	}
}

// 发布速度
void publishSpeed(double v,double w) // m/s  rad/s
{
	if(g_speed_smooth.try_lock())
	{
		if(man_stop_flag)	// 人为的停车
		{
			Mesg_RobotSpeed man_stop_speed;
			man_stop_speed.set_vx(0);	man_stop_speed.set_vy(0);	man_stop_speed.set_w(0);
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(man_stop_speed);
		}

		if(g_flag_enable_all)
		{		
			Mesg_RobotSpeed now;

			if(State_after_vn.near_flag)
			{
				Speed_A = Smooth_para + (1.0-State_after_vn.near_dist)/500;
			}
			else
				Speed_A = Smooth_para;

			if(fabs(v-pre_speed.vx)>Speed_A)
			{
				if(v>pre_speed.vx)
					pub_this_v = pre_speed.vx + Speed_A;
				if(v<pre_speed.vx)
					pub_this_v = pre_speed.vx - Speed_A;
			}
			else
				pub_this_v = v;

			if(g_nav.m_safe_laser_adjust.isRobotStop())
			{
				stop_flag = true;
				cur_speed.vx = 0;	cur_speed.vy = 0;	cur_speed.w = 0;
			}
			else
			{
				stop_flag = false;
				cur_speed.vx = pub_this_v;	cur_speed.vy = 0;	cur_speed.w = w;
			}

			g_nav.m_safe_laser_adjust.safeSpeedAdjust(cur_speed,Laserdata,pre_speed);
			g_nav.m_safe_ultra_adjust.safeSpeedAdjust(cur_speed,Uldata,pre_speed);

			pre_speed = cur_speed;
			now.set_vx(cur_speed.vx);	now.set_vy(cur_speed.vy);	now.set_w(cur_speed.w);
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(now);			
		}
		g_speed_smooth.unlock();
	}
}

// 发布视觉导航后的机器人状态信息
void publishRobotStata(Vision_Navigation_RobotState pubstate)
{
	if(g_mtx_pose.try_lock())
	{
		if(nothing_flag)
		{
			Mesg_CommonData Vision_Navigation_pub_commondata;
			Mesg_DataUnit* unit_data = Vision_Navigation_pub_commondata.add_datas();
			unit_data->set_flag(Mesg_DataUnit::EDUF_MODULE_SLEEP);
			unit_data->add_values_int(3);
			SubPubManager::Instance()->m_commoninfo.GetPublisher()->publish(Vision_Navigation_pub_commondata);
			printf("VisionNavigation sleeping!\n");
			g_flag_enable_all = false;
			nothing_flag = false;
		}

		if(g_flag_enable_all || task_completed_flag)
		{
			Mesg_RobotState Vision_navigation_pub_state;
			// publish robot state message			
			Vision_navigation_pub_state.set_x(pubstate.after_VN_RobotPose.x);
			Vision_navigation_pub_state.set_y(pubstate.after_VN_RobotPose.y);
			Vision_navigation_pub_state.set_theta(pubstate.after_VN_RobotPose.theta);
			//SubPubManager::Instance()->m_robotstate.GetPublisher()->publish(Vision_navigation_pub_state);

			// publish robot common data include rest-percent|stop|recovery-motion			
			//Mesg_DataUnit Vision_Navigation_data_unit;
			Mesg_CommonData Vision_Navigation_pub_commondata;
			Mesg_DataUnit* unit_data = Vision_Navigation_pub_commondata.add_datas();
			//*unit_data = Vision_Navigation_data_unit;

			//if stop
			if(stop_flag)
			{
				recovery_motion_flag = true;
				unit_data->set_flag(Mesg_DataUnit::EDUF_STOP_BY_OBSTACLE);
				unit_data->add_values_int(1);
			}
			else
			{
				if(recovery_motion_flag)	//if recovery motion fist time
				{
					recovery_motion_flag = false;
					unit_data->set_flag(Mesg_DataUnit::EDUF_RESTORE_MOTION);
				}
				else
				{
					unit_data->set_flag(Mesg_DataUnit::EDUF_DEST_REST_PERCENT);
					unit_data->add_values_double(pubstate.lave_percent);
				}
			}			
			

			if(!task_completed_flag && pubstate.lave_percent == 100)
			{
				Mesg_RobotSpeed completed_speed;
				completed_speed.set_vx(0);completed_speed.set_vy(0);completed_speed.set_w(0);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(completed_speed);
				printf("Oh!This task completed\n");
				g_flag_enable_all = false;
				task_completed_flag = true;
			}
			else
			{
				if(pubstate.lave_percent == 100)
					printf("I have pub 100%!!\n");
				SubPubManager::Instance()->m_robotstate.GetPublisher()->publish(Vision_navigation_pub_state);
				SubPubManager::Instance()->m_commoninfo.GetPublisher()->publish(Vision_Navigation_pub_commondata);
			}
		}
		g_mtx_pose.unlock();
	}
}

void openVisionNav(bool open,double cur_x,double cur_y,double cur_theta)  //[m,m,rad]
{
	g_mtx_flag_enable_all.lock();
	g_init_pose = OrientedPoint(cur_x,cur_y,cur_theta);		// 定位与导航部分初始化
	HANDLE th;
	unsigned int thread_id;
	if(g_vis_thread_created==false && g_nav_thread_created==false)
	{
		//static int cont = 0;
		th = (HANDLE)_beginthreadex(NULL, 0, &visionThread, NULL, 0, &thread_id);
		//printf("%d\n",cont++);
		th = (HANDLE)_beginthreadex(NULL, 0, &navigationThread, NULL, 0, &thread_id);
		//printf("%d\n",cont++);
		g_vis_thread_created = g_nav_thread_created = open;
		//std::cout<<"Enable VisionNavigation module. Create vision thread and navigation thread."<<std::endl;
		//printf("---------------------------------------L&N Camera init complete!----------------------------------------------\n");
	}
	printf("---------------------------------------L&N Camera init complete!-------------------------------------------\n");
	g_mtx_flag_enable_all.unlock();
	//return true;
}


// 接收任务指令
void update_Vision_Navigation(Mesg_NavigationTask Vision_Navi_task)
{
	g_mtx_task_point.lock();

	printf("Get Navi_task! It's %d\n",Vision_Navi_task.flag());
	switch (Vision_Navi_task.flag())
	{
		case Mesg_NavigationTask::VISION_NAV:
			man_stop_flag = false; nothing_flag = false; g_flag_enable_all = true;	task_completed_flag = false; break;
		case Mesg_NavigationTask::NOTHING:
			man_stop_flag = false; nothing_flag = true; g_flag_enable_all = false; publishRobotStata(State_nothing); break;
		case Mesg_NavigationTask::STOP_URGENT:
			man_stop_flag = true; nothing_flag = false; g_flag_enable_all = false; publishSpeed(0,0); break;
		default:
			man_stop_flag = false; nothing_flag = false; g_flag_enable_all = false; break;
	}
	if(g_flag_enable_all)
	{
		//printf("go!\n");
		if(Vision_Navi_task.has_initstate())
		{
			start_point.x = Vision_Navi_task.initstate().x();
			start_point.y = Vision_Navi_task.initstate().y();
			start_point.theta = Vision_Navi_task.initstate().theta();
			//printf("get init state\n");
		}
		if(Vision_Navi_task.has_targetstate())
		{
			target.x = Vision_Navi_task.targetstate().x();
			target.y = Vision_Navi_task.targetstate().y();
			target.theta = Vision_Navi_task.targetstate().theta();
			//printf("get target state\n");

		}
		printf("start @ %lf %lf %lf end @ %lf %lf %lf\n",start_point.x,start_point.y,start_point.theta*180/M_PI,target.x,target.y,target.theta*180/M_PI);
		//pre_v = 0;
		pre_speed.vx = 0;	pre_speed.vy = 0;	pre_speed.w = 0;
		percent_int = 0;
		//printf("start openVisionNav\n");
		openVisionNav(g_flag_enable_all,start_point.x,start_point.y,start_point.theta);
	}
	g_mtx_task_point.unlock();
}


int main()
{
	std::cout<<"done\nInitializing SURO...\n";
	NODE.init("NR-Vision_navigation...\n");
	//fp2 = fopen("visiontrack_data.txt","w+");
	//subcribe
	Set_VisionNavi_Param();

	SubPubManager::Instance()->m_navtask.Initialize(Topic::Topic_NavTask,update_Vision_Navigation);	//接收机器人任务指令信息
	//Single_test();
	//subcribe
	SubPubManager::Instance()->m_laser.Initialize(Topic::Topic_Laser,updateLaser);				//接收激光数据
	SubPubManager::Instance()->m_stopobs.Initialize(Topic::Topic_StopObs,updateUltrsonic);		//接收超声数据
	SubPubManager::Instance()->m_odometer.Initialize(Topic::Topic_Odometer,updateOdometry);		//接收里程计数据

	//publish
	SubPubManager::Instance()->m_robotspeed.Initialize(Topic::Topic_Speed,NULL);				//发布机器人的速度
	SubPubManager::Instance()->m_robotstate.Initialize(Topic::Topic_State,NULL);				//发布机器人状态信息
	SubPubManager::Instance()->m_commoninfo.Initialize(Topic::Topic_CommonInfo,NULL);			//发布机器人相关信息
	//service
	//NODE.advertiseService<bool(bool,double,double,double)>(Service::Service_OpenVisionNav,boost::bind(openVisionNav,_1,_2,_3,_4)); //开启整个导航模块，并传入初始位姿
	Sleep(500);
	NODE.spin();
}

// 视觉识别进程
unsigned __stdcall visionThread(void *p)
{
	lineDetector.setCamera(&para[0], &distorPara[0], 480, 640); //初始化部分
	lineDetector.setExtra(&cameraPara[0]);
	printf("---------------------------------------Camera init complete!----------------------------------------------\n");

	IplImage* pFrame = NULL;
	//获取摄像头
	CvCapture* pCapture = cvCreateCameraCapture(-1);
	int index = 0; //当前处理的图像标号，每次循环加一
	Result curResult;
	cv::namedWindow("outPutImage");
	while(moduleEnable())
	{
		//printf(".");
		pFrame = cvQueryFrame(pCapture);
		cv::Mat x2(pFrame);
		//cout<<x2.cols<<" "<<x2.rows<<endl;
		g_need_detect_landmark = g_nav.get_landmark_flag();
		lineDetector.processImage(x2, g_need_detect_landmark, curResult, 20, false, index);
		imshow("outPutImage", lineDetector.outPutImg);
		cv::waitKey(10);
		if(curResult.angle != -1 && curResult.leftDistance<700 && curResult.rightDistance<700)
		{
			g_has_vline = true;
			g_vline_angle = curResult.angle;
			if(curResult.leftDistance==-1)
				g_vline_dist_left = -1;
			else
				g_vline_dist_left = curResult.leftDistance + cos(curResult.angle*M_PI/180.0)*Cam2CenterDist;
			if(curResult.rightDistance==-1)
				g_vline_dist_right = -1;
			else
				g_vline_dist_right = curResult.rightDistance + cos(curResult.angle*M_PI/180.0)*Cam2CenterDist;
			//printf("%lf %lf %lf\n",curResult.angle, curResult.leftDistance, g_vline_dist);
		}
		else
		{
			g_has_vline = false;
			g_vline_angle = -1;
			g_vline_dist_left = -1;
			g_vline_dist_right = -1;

		}
		g_has_vlandmark = curResult.labelValid;
		//printf("%d\n",g_has_vlandmark);
		if(g_has_vlandmark)
		{
			g_vlandmark_point.x = curResult.labelXpos;
			g_vlandmark_point.y = curResult.labelYpos+Cam2CenterDist;
		//	printf("%lf,%lf\n",curResult.labelXpos/1000.0, curResult.labelYpos/1000.0);
		}
		//cout << curResult.angle << " " << curResult.leftDistance << " " << curResult.rightDistance
		//	<< " " << curResult.leftDistance + curResult.rightDistance << endl;
		//imshow("outPutImg", lineDetector.outPutImg);
		//cv::waitKey(10);
		index += 1;
	}
	std::cout<<"End vision thread."<<std::endl;
//	cvReleaseCapture(&pCapture);
	g_vis_thread_created = false;
	cv::destroyWindow("outPutImage");
	ExitThread(0);
	//TerminateThread(th,NULL);
}

// 定位导航进程
unsigned __stdcall navigationThread(void *p)
{
	Vision_Navigation_RobotState task_complete_state;
	g_nav.setCurPose(g_init_pose);
	g_nav.setInitialPreviousTarget(g_init_pose);
	//g_nav.setFist_flag(true);
	g_mtx_odom.lock();
	g_nav.setCurOdom(g_odom);
	g_nav.setInitialPreviousOdom();
	g_mtx_odom.unlock();
	g_nav.setTargetPoint(target);
	printf("-------------------------------------------L&N init complete!-------------------------------------------\n");
	
	while(moduleEnable())
	{
		//printf("Module Enable\n");
		g_mtx_odom.lock();
		OrientedPoint cur_odom = g_odom;
		g_mtx_odom.unlock();
		
		g_mtx_vline.lock();
		bool has_vline = g_has_vline;
		double vline_dist_left = g_vline_dist_left;
		double vline_dist_right = g_vline_dist_right;
		double vline_ang = g_vline_angle;
		g_mtx_vline.unlock();

		g_mtx_vlandmark.lock();
		bool has_vlandmark = g_has_vlandmark;
		Point landmark_point = g_vlandmark_point;
		g_mtx_vlandmark.unlock();
		
		g_mtx_task_point.lock();
		OrientedPoint task_point = g_task_point;
		g_mtx_task_point.unlock();

		//===设置nav参数
		g_nav.setCurOdom(cur_odom);
		g_nav.setVlineInfo(has_vline,vline_dist_left,vline_dist_right,vline_ang);
		if(has_vlandmark) g_nav.setVlandmarkInfo(landmark_point);
		//printf("set nav_pararm\n");

		double send_v,send_w;
		//Timer loop_time;
		//loop_time.start();
		//===处理一次定位导航
		g_nav.process();
		//===获取结果并发布
		g_nav.getNavResult(send_v,send_w);		
		State_after_vn = g_nav.getVisionNavigation_RobotState();
		publishSpeed(send_v,send_w);	
		publishRobotStata(State_after_vn);
		if(State_after_vn.lave_percent==100)
		{
			task_complete_state.lave_percent = State_after_vn.lave_percent;
			task_complete_state.after_VN_RobotPose = State_after_vn.after_VN_RobotPose;
		}
		//loop_time.stop();
		//while(loop_time.getMsecTime()<100)
		//	Sleep(2);
		//loop_time.stop();
	}
	publishRobotStata(task_complete_state);	
	g_nav_thread_created = false;
	std::cout<<"End navigation thread."<<std::endl;
	ExitThread(0);
}

bool moduleEnable()
{
	g_mtx_flag_enable_all.lock();
	bool enable = g_flag_enable_all;
	g_mtx_flag_enable_all.unlock();
	return enable;
}



//bool three_point_one_line(OrientedPoint cur_point, OrientedPoint mid_point, OrientedPoint next_point)
//{
//	double a2 = euclidianDist(cur_point,mid_point) * euclidianDist(cur_point,mid_point);
//	double b2 = euclidianDist(next_point,mid_point) * euclidianDist(next_point,mid_point);
//	double c2 = euclidianDist(cur_point,next_point) * euclidianDist(cur_point,next_point);
//	double theta = acos((a2+b2-c2) / (2.0*sqrt(a2)*sqrt(b2)));	//(a^2+b^2-c^2)/2ab
//	if(theta<0)	theta = theta + M_PI;
//
//	if(theta > 175.0*M_PI/180.0)	// 175-180 认为在一条直线上
//		return true;
//	else
//		return false;
//}
//
//
//
//Point get_Center(OrientedPoint cur_point, OrientedPoint mid_point, OrientedPoint next_point)
//{
//	//p1: cur_point.x;cur_point.y; p2: mid_point.x;mid_point.y; p3: next_point.x;next_point.y;
//	//double k3 = (y2-y1)/(x1-x2);
//	//double b3 = x1 - k3*y1;
//	//double k4 = (y3-y2)/(x2-x3);
//	//double b4 = x3 - k4*y3;
//	//double y = (b4-b3)/(k3-k4);
//	//double x = k3*y + b3;
//
//	double k3,b3,k4,b4;
//	double thershold = 0.01;
//	Point Center;
//	if(fabs(cur_point.x-mid_point.x)<thershold && fabs(mid_point.x-next_point.x)>=thershold)	// k3 wuqiong da 
//	{
//		k4 = (next_point.y-mid_point.y)/(mid_point.x-next_point.x);
//		b4 = next_point.x - k4*next_point.y;
//		Center.Y = cur_point.y;
//		Center.X = k4*Center.Y + b4;
//	}
//
//	if(fabs(cur_point.x-mid_point.x)>=thershold && fabs(mid_point.x-next_point.x)<thershold)	// k4 wuqiong da
//	{
//		k3 = (mid_point.y-cur_point.y)/(cur_point.x-mid_point.x);
//		b3 = cur_point.x - k3*cur_point.y;
//		Center.Y = next_point.y;
//		Center.X = k3*Center.Y + b3;
//	}
//
//	if(fabs(cur_point.x-mid_point.x)>=thershold && fabs(mid_point.x-next_point.x)>=thershold)	// k3 k4 both not wuqiongda
//	{
//		k3 = (mid_point.y-cur_point.y)/(cur_point.x-mid_point.x);
//		b3 = cur_point.x - k3*cur_point.y;
//		k4 = (next_point.y-mid_point.y)/(mid_point.x-next_point.x);
//		b4 = next_point.x - k4*next_point.y;
//		Center.Y = (b4-b3)/(k3-k4);
//		Center.X = k3*Center.Y + b3;
//	}
//	return Center;
//}
//
//
//bool MotionControlFSM::magic_turn( double &v_best,double &w_best,Point center,double R,bool turn_flag )
//{
//	//功能：magic turn ~amazing!
//	double e_vw,e_vx;
//	e_vx = 0.5;
//	if(turn_flag)
//		e_vw = e_vx/R + ((cur_point.x-center.X)*(cur_point.x-center.X)+(cur_point.y-center.Y)*(cur_point.y-center.Y)-R*R)*0.01;
//	else
//		e_vw = -1*(e_vx/R + ((cur_point.x-center.X)*(cur_point.x-center.X)+(cur_point.y-center.Y)*(cur_point.y-center.Y)-R*R)*0.01);
//	v_best = e_vx;
//	w_best = e_vw;
//	return true;
//}