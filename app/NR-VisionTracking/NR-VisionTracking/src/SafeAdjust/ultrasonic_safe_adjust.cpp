#include "ultrasonic_safe_adjust.h"
#include <common/utils.h>
#include <common/timer.h>
#include <common/std_out.h>
namespace NJRobot{

UltrasonicSafeAdjust::UltrasonicSafeAdjust(void):AbstractSafeAdjust<ObstacleData>()
{
}

UltrasonicSafeAdjust::~UltrasonicSafeAdjust(void)
{
}

void UltrasonicSafeAdjust::safeSpeedAdjust( RobotSpeed& send_speed,const ObstacleData & obs,const RobotSpeed& actual_speed )
{
	if(obs.data.size()<3) {return;}
	using namespace std;
	const static double thres_stop_side = 0.15;
	const static double thres_stop_front = 0.15;

	double& vx = send_speed.vx;

	double left = obs.data[0].mod();
	double front = obs.data[1].mod();
	double right = obs.data[2].mod();
	if(left<thres_stop_side || right<thres_stop_side || front<thres_stop_front){
		send_speed.vx = send_speed.vy = send_speed.w = 0;
		if(left<thres_stop_side){
			COUT_COLOR(getCurTimeStr()+" Ultr stop. LEFT.",ColorType::PINK);
		}else if(right<thres_stop_side){
			COUT_COLOR(getCurTimeStr()+" Ultr stop. RIGHT.",ColorType::PINK);
		}else if(front<thres_stop_front){
			COUT_COLOR(getCurTimeStr()+" Ultr stop. FRONT.",ColorType::PINK);
		}
	}else if(front<1&&vx>0){
		double newV = generalSlow(vx,front,actual_speed.vx);
		if(newV<vx){
			COUT_COLOR(getCurTimeStr()+" Ultr slow. dist:"<<front<<" v:"<<vx,ColorType::DARKPINK);
		}
		vx = newV;
	}
}

}