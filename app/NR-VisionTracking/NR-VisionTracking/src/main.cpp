#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include "common/utils.h"
#include "messages/MessageInteraction.h"
#include "map/map_convert.h"
#include "map/traval_map.h"
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
//#define Speed_A	0.01

FILE *fp2;

double Speed_A = 0.00001;



//



const double para[5] = { 0.6000,   -0.8000,    0.0010,    0.0005,  327.1323 };

const double cameraPara[] = { 494.3439,         0,  		349.0357,
0,  		491.5964,  		230.9781,
0, 				0, 			1.0000 };
const double distorPara[] = { 0,   0,   0,    0 };

LineDetector lineDetector;


IplImage* pFrame;//获取摄像头
CvCapture* pCapture;

//////////////////////////////////////////////////////////////////////////

//HANDLE th;
//unsigned int thread_id;

///////// global variable ///////////////////////////////////
// 进程外部通讯
bool g_flag_enable_all = false; //是否开启当前模块
boost::mutex g_mtx_flag_enable_all;

OrientedPoint g_odom;
boost::mutex g_mtx_odom;

OrientedPoint g_task_point;
boost::mutex g_mtx_task_point;

OrientedPoint g_pose;  //当前机器人的位姿，当循线模块结束时，需要将该位姿告知外部
Vision_Navigation_RobotState State_after_vn;	// 视觉导航结束后发布机器人状态
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


// 速度平滑

double pre_v,pub_this_v;
boost::mutex g_speed_smooth;



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

// 更新激光数据
void updateLaser(Mesg_Laser ldata)
{ 
	if(g_mutex_laser.try_lock())
	{
	// access laser data
		if(g_flag_enable_all)
		{
			int _speed_flag_num = 0, stop_flag_num = 0;
			_speed_flag = false;
			stop_flag = false;
			double stop_min_dist = 1000;

			//for(int i=0;i<ldata.laser_data_size();i++)
			for(int i=90;i<450;i++)
			{
				double angle = ldata.laser_data(i).angle()*3.1415926/180;	//deg->rad
				//double dist =  ldata.laser_data(i).dist()/1000;           //mm -> m
				double dist1 =  ldata.laser_data(i).dist();					//mm 所有dist不大于8000
				//fprintf(fp,"%d %lf %lf\n", i, angle, dist);	

				double dist = sqrt(dist1*dist1 + Center_Sick*Center_Sick - 2*dist1*Center_Sick*cos(M_PI+angle));		// a^2 = b^2+c^2-2bccos(A)

				double cosangle = (dist*dist+Center_Sick*Center_Sick-dist1*dist1)/(2*dist*Center_Sick);					// cos(b) = (a^2+c^2-b^2)/(2ac)
				angle = acos(cosangle);

			//	if(i>90 && i<450)
					//if(dist>350 && dist<500)
				if(cosangle*dist>400 && cosangle*dist<500 && sin(angle)*dist<300)
				{
					stop_flag_num ++;
					if(cosangle*dist<stop_min_dist)
						stop_min_dist = cosangle*dist;

					if(stop_flag_num>2)
					{
						stop_flag = true;
						stop_dist = stop_min_dist;
					}
				}
				else
				{
					//if(dist<1000 && dist>=500)
					if(sin(angle)*dist<300 && cosangle*dist<1000 && cosangle*dist>=500)
					{
						_speed_flag_num ++;
						if(cosangle*dist<stop_min_dist)
							stop_min_dist = cosangle*dist;

						if(_speed_flag_num>2)
						{
							_speed_flag = true;
							stop_dist = stop_min_dist;
							Speed_A = 0.00002 + (1000-stop_dist)/500000;
						}
						else
							Speed_A = 0.00002;
					}
				}
			}
		}
		g_mutex_laser.unlock();
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

//void updateTask(Home_RobotTask task) 
//{
//	g_mtx_task_point.lock();
//	g_task_point.x = task.targetpose().x();
//	g_task_point.y = task.targetpose().y();
//	g_task_point.theta = task.targetpose().theta();
//	g_mtx_task_point.unlock();
//}


// 发布速度
void publishSpeed(double v,double w) // m/s  rad/s
{
	if(g_flag_enable_all)
	{
		if(g_speed_smooth.try_lock())
		{
			Mesg_RobotSpeed speed;
			if(stop_flag)
			{
				//printf("stop = %lf\n",stop_dist);
				pub_this_v = 0;
				w = 0;
				//Speed_A = 0.1;
			}
			else
			{
				if(_speed_flag)
				{
					v = (stop_dist-500)/500 * v;
				}

				if(fabs(v-pre_v)>Speed_A)
				{
					//printf("v=%lf pre_v=%lf\n",v,pre_v);
					if(v>pre_v)
						pub_this_v = pre_v + Speed_A;
					if(v<pre_v)
						pub_this_v = pre_v - Speed_A;
				}
				else
					pub_this_v = v;
			}

			speed.set_vx(pub_this_v); // m/s => mm/s
			speed.set_vy(0);
			speed.set_w(w); 
			pre_v = pub_this_v;
			//std::cout<<v<<","<<w<<std::endl;
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(speed);
			//printf("target_v = %lf real_v = %lf",v,pub_this_v);
			g_speed_smooth.unlock();
		}		
	}
}

// 发布视觉导航后的机器人状态信息
void publishRobotStata(Vision_Navigation_RobotState pubstate)
{
	if(g_mtx_pose.try_lock())
	{
		if(g_flag_enable_all)
		{
			// publish robot state message
			Mesg_RobotState Vision_navigation_pub_state;
			Vision_navigation_pub_state.set_x(pubstate.after_VN_RobotPose.x);
			Vision_navigation_pub_state.set_y(pubstate.after_VN_RobotPose.y);
			Vision_navigation_pub_state.set_theta(pubstate.after_VN_RobotPose.theta);
			SubPubManager::Instance()->m_robotstate.GetPublisher()->publish(Vision_navigation_pub_state);

			// publish robot common data include rest-percent|stop|recovery-motion
			Mesg_DataUnit Vision_Navigation_data_unit;
			Mesg_CommonData Vision_Navigation_pub_commondata;
			if(pubstate.stop_or_not)	//if stop
			{
				recovery_motion_flag = true;
				Vision_Navigation_data_unit.set_flag(Mesg_DataUnit::EDUF_STOP_BY_OBSTACLE);
			}
			else
			{
				//if(recovery_motion_flag)	//if recovery motion fist time
				//{
				//	recovery_motion_flag = false;
				//	Vision_Navigation_data_unit.set_flag(Mesg_DataUnit::EDUF_RESTORE_MOTION);
				//}
				//else
				{
					Vision_Navigation_data_unit.set_flag(Mesg_DataUnit::EDUF_DEST_REST_PERCENT);
					Vision_Navigation_data_unit.add_values_double(pubstate.lave_percent);
				}
			}
			Mesg_DataUnit* du = Vision_Navigation_pub_commondata.add_datas();
			*du = Vision_Navigation_data_unit;
			SubPubManager::Instance()->m_commondata.GetPublisher()->publish(Vision_Navigation_pub_commondata);

			//printf("Start-Target:%lf,%lf-%lf,%lf State:%lf %lf %lf Percent:%lf\n",start_point.x,start_point.y,target.x,target.y,
			//	pubstate.after_VN_RobotPose.x,pubstate.after_VN_RobotPose.y,pubstate.after_VN_RobotPose.theta*180/M_PI,
			//	pubstate.lave_percent);
			
			if(pubstate.lave_percent == 100)
			{
				printf("Oh!task_complete\n");
				g_flag_enable_all = false;
			}
		}
		g_mtx_pose.unlock();
	}
}

void openVisionNav(bool open,double cur_x,double cur_y,double cur_theta)  //[m,m,rad]
{
	g_mtx_flag_enable_all.lock();
	g_init_pose = OrientedPoint(cur_x,cur_y,cur_theta);
	HANDLE th;
	unsigned int thread_id;
	if(g_vis_thread_created==false && g_nav_thread_created==false)
	{
		static int cont = 0;
		th = (HANDLE)_beginthreadex(NULL, 0, &visionThread, NULL, 0, &thread_id);
		printf("%d\n",cont++);
		th = (HANDLE)_beginthreadex(NULL, 0, &navigationThread, NULL, 0, &thread_id);
		printf("%d\n",cont++);
		g_vis_thread_created = g_nav_thread_created = open;
		//std::cout<<"Enable VisionNavigation module. Create vision thread and navigation thread."<<std::endl;
		//printf("---------------------------------------L&N Camera init complete!----------------------------------------------\n");
	}
	printf("---------------------------------------L&N Camera init complete!----------------------------------------------\n");
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
			g_flag_enable_all = true;	break;
		default:
			g_flag_enable_all = false;	break;
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
		pre_v = 0;
		//printf("start openVisionNav\n");
		openVisionNav(g_flag_enable_all,start_point.x,start_point.y,start_point.theta);
	}
	g_mtx_task_point.unlock();
}


//void Single_test()
//{
//	g_flag_enable_all = true;
//	printf("请输入起始点与终止点\n");
//	scanf("%lf%lf%lf%lf%lf",&start_point.x,&start_point.y,&start_point.theta,&target.x,&target.y);
//	printf("Start:%lf,%lf,%lf,End:%lf,%lf", start_point.x,start_point.y,start_point.theta,target.x,target.y);
//}


int main()
{
	std::cout<<"done\nInitializing SURO...\n";
	NODE.init("NR-Vision_navigation...\n");
	//fp2 = fopen("visiontrack_data.txt","w+");
	//subcribe


	SubPubManager::Instance()->m_navtask.Initialize(Topic::Topic_NavTask,update_Vision_Navigation);	//接收机器人任务指令信息
	//Single_test();
		//subcribe
		SubPubManager::Instance()->m_laser.Initialize(Topic::Topic_Laser,updateLaser);				//接收激光数据
		SubPubManager::Instance()->m_odometer.Initialize(Topic::Topic_Odometer,updateOdometry);		//接收里程计数据

		//publish
		SubPubManager::Instance()->m_robotspeed.Initialize(Topic::Topic_Speed,NULL);				//发布机器人的速度
		SubPubManager::Instance()->m_robotstate.Initialize(Topic::Topic_State,NULL);				//发布机器人状态信息
		SubPubManager::Instance()->m_commondata.Initialize(Topic::Topic_State,NULL);				//发布机器人的到点信息
	//service
	//NODE.advertiseService<bool(bool,double,double,double)>(Service::Service_OpenVisionNav,boost::bind(openVisionNav,_1,_2,_3,_4)); //开启整个导航模块，并传入初始位姿
	Sleep(500);

	//if(g_flag_enable_all)
	//{
		//printf("start openVisionNav\n");
		//openVisionNav(g_flag_enable_all,start_point.x,start_point.y,start_point.theta);
	//}

	NODE.spin();
}

// 视觉识别进程
unsigned __stdcall visionThread(void *p)
{
	//printf("---------------------------------------Camera init complete!----------------------------------------------\n");

	lineDetector.setCamera(cameraPara, distorPara, 480, 640); //初始化部分
	lineDetector.setExtra(para);
	//printf("---------------------------------------Camera init complete!----------------------------------------------\n");

	IplImage* pFrame = NULL;
	//获取摄像头
	CvCapture* pCapture = cvCreateCameraCapture(-1);
	//printf("打开摄像头成功！！！！！！！！！！！！！！！！！！！！！！！！！！%d\n",moduleEnable());
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
	g_nav.setCurPose(g_init_pose);
	g_nav.setInitialPreviousTarget(g_init_pose);
	//g_nav.setFist_flag(true);
	g_mtx_odom.lock();
	g_nav.setCurOdom(g_odom);
	g_nav.setInitialPreviousOdom();
	g_mtx_odom.unlock();
	g_nav.setTargetPoint(target);

	//printf("-------------------------------------------L&N init complete!-------------------------------------------\n");
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

		//===处理一次定位导航
		g_nav.process();
		//printf("do one time yet!\n");

		//===获取结果并发布
		double send_v,send_w;
		g_nav.getNavResult(send_v,send_w);		
		publishSpeed(send_v,send_w);		
		//printf("Percent:%lf\n",State_after_vn.lave_percent);
		//printf("after:v = %lf m/s,w = %lf rad/s\n", send_v, send_w);
		//fprintf(fp,"POSE:%lf,%lf,%lf  NAVI:%lf m/s,%lf rad/s DIST:%lf\n",cur_odom.x, cur_odom.y, cur_odom.theta*180.0/M_PI,send_v,send_v,vline_dist);
		
		g_mtx_pose.lock();
		State_after_vn = g_nav.getVisionNavigation_RobotState();
		publishRobotStata(State_after_vn);
		//g_pose = g_nav.getCurPose();
		g_mtx_pose.unlock();

	}
	std::cout<<"End navigation thread."<<std::endl;
	g_nav_thread_created = false;
	ExitThread(0);
}

bool moduleEnable()
{
	g_mtx_flag_enable_all.lock();
	bool enable = g_flag_enable_all;
	g_mtx_flag_enable_all.unlock();
	return enable;
}


