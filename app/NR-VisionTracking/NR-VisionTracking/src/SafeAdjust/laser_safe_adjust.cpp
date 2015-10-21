#include "laser_safe_adjust.h"
#include <common/utils.h>
#include <common/timer.h>
#include <common/std_out.h>

namespace NJRobot{


LaserSafeAdjust::LaserSafeAdjust(void):AbstractSafeAdjust<PointList>()
{
}

LaserSafeAdjust::~LaserSafeAdjust(void)
{
}

void LaserSafeAdjust::dangerCheck( const PointList & obs,const RobotSpeed& actual_speed )
{
	double prev_v = actual_speed.vx;
	// check danger
	m_has_obs_front = m_has_obs_left = m_has_obs_right = false;

	m_closest_point = Point(DBL_MAX,DBL_MAX);

	double half_width = m_vehicle_width/2;
	double half_length = m_vehicle_length/2;
	double stop_dist_turn = 0.010; //0.01m
	double danger_dist_turn  = std::max(half_width,half_length)+stop_dist_turn;
	double max_check_dist =  linearEquation(0.8,3, 0.4,2, prev_v);//最大障碍物检测距离，如果速度很快，这个距离要加大  v:0.4m/s -> d=2m   v:0.8m/s -> d=3m
	max_check_dist = clip(max_check_dist,2.0,5.0);

	for(int i=0;i<obs.size();i++)
	{
		Point pt = obs[i];
		double ox = pt.x;
		double oy = pt.y;
		double theta_deg = rad2deg(pt.dir());

		//太在车身里面的点被认为是噪点
		if(ox+0.100<half_length  && abs(oy)+0.100<half_width)
		{
			continue;
		}
		//前方是否有障碍物，有的话机器人需要减速或者停下
		if(abs(oy)<=half_width && ox>0.010 &&ox-half_length<max_check_dist) //max_check_dist以外的就不考虑了
		{
			m_has_obs_front = true;
			if(ox<m_closest_point.x) m_closest_point = pt;
		}
		//车身周围是否有障碍物，有的话机器人不能转动
		if(hypot(ox,oy)<danger_dist_turn)
		{
			if(m_has_obs_left==false && theta_deg>-10)
			{
				m_has_obs_left = true;
				m_obs_point_left = pt;
			}
			if(m_has_obs_right==false && theta_deg<10)
			{
				m_has_obs_right = true;
				m_obs_point_right = pt;
			}
		}
	}
}

void LaserSafeAdjust::safeSpeedAdjust(RobotSpeed& send_speed,const PointList & obs,const RobotSpeed& actual_speed)
{
	m_stop_flag = false;
	// check danger
	dangerCheck(obs,actual_speed);

	double& vx = send_speed.vx;
	double& vy = send_speed.vy;
	double& vw = send_speed.w;

	double prev_v = actual_speed.vx;
	double prev_w = actual_speed.w;

	// handle vw
	if(m_has_obs_left && vw>0)
	{
		COUT_COLOR(getCurTimeStr()+" Laser STOP! Danger LEFT.("<<m_obs_point_left<<")",ColorType::BLUE);
		vw=0;
	}
	if(m_has_obs_right && vw<0)
	{
		vw=0;
		COUT_COLOR(getCurTimeStr()+" Laser STOP! Danger RIGHT.("<<m_obs_point_right<<")",ColorType::BLUE);
	}

	// handle vx
	double stop_dist = 0.1; //0.1m必须停

	if(m_has_obs_front)
	{
		double cache_dist = m_closest_point.x-m_vehicle_length/2;  //缓冲距离，机器人前方与最近障碍物的距离
		if(cache_dist<0) cache_dist=0;

		if(cache_dist<=stop_dist)  //这个是没有余地的，一旦危险距离太小，不管怎么样必须停
		{
			m_stop_flag = true;
			COUT_COLOR(getCurTimeStr()+" Laser STOP! Danger FRONT. ("<<m_closest_point<<")",ColorType::BLUE);
			vx = 0.0;
		}
		else if(vx>0)
		{
			double newV = generalSlow(vx,cache_dist,prev_v);
			if(newV<vx){
			//	COUT_COLOR(getCurTimeStr()+" Laser slow. ("<<m_closest_point<<") Dist:"<<cache_dist,ColorType::DARKBLUE);
			}
			vx = newV;
		}
	}
}



}