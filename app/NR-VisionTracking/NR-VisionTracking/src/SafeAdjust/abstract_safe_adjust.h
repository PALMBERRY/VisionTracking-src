#pragma once
#include <common/robot_type.h>
#include <common/utils.h>

namespace NJRobot
{


template <class ObsType>
class AbstractSafeAdjust
{
public:
	AbstractSafeAdjust(void):m_process_loop(0.1),m_vehicle_width(0),m_vehicle_length(0),m_stop_flag(false){}
	virtual ~AbstractSafeAdjust(void){}

	void setVehicleShape(double length,double width){
		m_vehicle_length = length;
		m_vehicle_width = width;
	}

	void setProcessLoop(double t){
		m_process_loop = t;
	}

	bool isRobotStop(){
		return m_stop_flag;
	}

	virtual void safeSpeedAdjust(RobotSpeed& send_speed,const ObsType & obs,const RobotSpeed& actual_speed)=0;

protected:
	double	m_vehicle_width,m_vehicle_length;
	bool	m_stop_flag;
	double  m_process_loop;  //处理周期 单位s

	//通用减速方法,根据下发的速度、障碍物距离和上一次下发的速度，给出下发速度
	double generalSlow(double send_v,double dist_obs,double prev_v){
		//1. 不需要处理：机器人后退，机器人当前速度为0
		if(send_v<=0 || prev_v<=0) return send_v;
		//2. 障碍物距离小于停障距离，速度强制设成0
		//const double stop_dist = 0.1; 
		//if(dist_obs<stop_dist) return 0;
		////3. 减速策略
		//double max_v = linearEquation(stop_dist,0.03, 2,0.5,dist_obs);  //期望的速度，但是距离远的时候可以适当控制减速度
		//if(send_v>max_v && send_v>0.1)
		//{
		//	double acc_dec;
		//	double cache_t = dist_obs/prev_v; //估计多长时间会撞，根据缓冲时间来调整加速度	
		//	if(cache_t<0.5) {acc_dec=2;}
		//	else if(cache_t<1) {acc_dec=1;}
		//	else if(cache_t<2) {acc_dec=0.80;}
		//	else if(cache_t<3) {acc_dec=0.40;}
		//	else {acc_dec=0.20;}
		//	send_v = clip(max_v,prev_v-acc_dec*m_process_loop,send_v);
		//}
		//return send_v;

		const double stop_dist = 0.1; 
		if(dist_obs<stop_dist) return 0;
		double cache_t = dist_obs/prev_v; //估计多长时间会撞，根据缓冲时间来调整加速度	
		double acc_dec;
		if(cache_t<0.5) {acc_dec=0.200;}
		else if(cache_t<1) {acc_dec=0.100;}
		else if(cache_t<2) {acc_dec=0.080;}
		else if(cache_t<3) {acc_dec=0.040;}
		else {acc_dec=0.020;}
		double max_v = linearEquation(stop_dist,0.03, 2,0.5,dist_obs);  //期望的速度，但是距离远的时候可以适当控制减速度
		//std::cout<<"cache_t:"<<cache_t<<" InSpeed:"<<send_v<<" max_v:"<<max_v<<std::endl;
		if(send_v>max_v && send_v>0.04)  //之间这里的值是0.1
		{
			if((send_v-prev_v)==0.02&&cache_t>3)  //往上加了速度导致减少不明显，额外减去0.02
				send_v = send_v-acc_dec-0.02;
			else if((send_v-prev_v)==0.02&&cache_t<1) //此时速度减少太明显，往上加经验参数0.04
				send_v = send_v-acc_dec+0.04;
			else if(send_v<0)send_v = 0;
			else send_v = send_v-acc_dec;
			//COUT_COLOR(getCurTimeStr()+"cache_t:"<<cache_t<<" InSpeed:"<<send_v<<" max_v:"<<max_v,ColorType::PINK);
		}
		return send_v;
	}
};

}