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

		double getProcessLoop(){
			return m_process_loop;
		}

		bool isRobotStop(){
			return m_stop_flag;
		}

		virtual void safeSpeedAdjust(RobotSpeed& send_speed,const ObsType & obs,const RobotSpeed& actual_speed)=0;

	protected:
		double	m_vehicle_width,m_vehicle_length;
		bool	m_stop_flag;
		double  m_process_loop;  //�������� ��λs

		//ͨ�ü��ٷ���,�����·����ٶȡ��ϰ���������һ���·����ٶȣ������·��ٶ�
		double generalSlow(double send_v,double dist_obs,double prev_v){
			//1. ����Ҫ���������˺��ˣ������˵�ǰ�ٶ�Ϊ0
			if(send_v<=0 || prev_v<=0) return send_v;
			const double stop_dist = 0.1; 
			if(dist_obs<stop_dist) return 0;
			double cache_t = dist_obs/prev_v; //���ƶ೤ʱ���ײ�����ݻ���ʱ�����������ٶ�	
			double acc_dec = 0;
			/**************��δ���ֻ��ͻȻ���ֽ������ϰ����ʱ��Ż���Ч��**********************/
			if(cache_t<0.5) {acc_dec=0.200;}
			else if(cache_t<1) {acc_dec=0.100;}
			else if(cache_t<2) {acc_dec=0.080;}
			else if(cache_t<3) {acc_dec=0.040;}
			else {acc_dec=0.020;}
			/************************************************************************************/
			double max_v = linearEquation(stop_dist,0.05,1,0.5,dist_obs);  //�������ٶȣ����Ǿ���Զ��ʱ������ʵ����Ƽ��ٶ�
			if(send_v>max_v && send_v>0.04)  //֮�������ֵ��0.1
			{
				send_v = clip(0.0,prev_v-acc_dec,prev_v);
			}
			return send_v;
		}

	};

}