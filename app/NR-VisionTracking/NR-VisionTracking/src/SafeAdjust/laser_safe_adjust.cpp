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
	double max_check_dist =  linearEquation(0.8,3, 0.4,2, prev_v);//����ϰ�������룬����ٶȺܿ죬�������Ҫ�Ӵ�  v:0.4m/s -> d=2m   v:0.8m/s -> d=3m
	max_check_dist = clip(max_check_dist,2.0,5.0);

	for(int i=0;i<obs.size();i++)
	{
		Point pt = obs[i];
		double ox = pt.x;
		double oy = pt.y;
		double theta_deg = rad2deg(pt.dir());

		//̫�ڳ�������ĵ㱻��Ϊ�����
		if(ox+0.100<half_length  && abs(oy)+0.100<half_width)
		{
			continue;
		}
		//ǰ���Ƿ����ϰ���еĻ���������Ҫ���ٻ���ͣ��
		if(abs(oy)<=half_width && ox>0.010 &&ox-half_length<max_check_dist) //max_check_dist����ľͲ�������
		{
			m_has_obs_front = true;
			if(ox<m_closest_point.x) m_closest_point = pt;
		}
		//������Χ�Ƿ����ϰ���еĻ������˲���ת��
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
	double stop_dist = 0.1; //0.1m����ͣ

	if(m_has_obs_front)
	{
		double cache_dist = m_closest_point.x-m_vehicle_length/2;  //������룬������ǰ��������ϰ���ľ���
		if(cache_dist<0) cache_dist=0;

		if(cache_dist<=stop_dist)  //�����û����صģ�һ��Σ�վ���̫С��������ô������ͣ
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