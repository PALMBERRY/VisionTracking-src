/************************************************************************/
/* Institute of Cyber-Systems and Control, Nlict-Robot-Fundation		*/
/* HomePage:	http://www.nlict.zju.edu.cn/robot/						*/
/************************************************************************/
/* File:		NRF_AbstractMotionControl.h								*/
/* Purpose: 	Define the interface for motion control					*/
/************************************************************************/

#ifndef NRF_ABSTRACT_MOTION_CONTROL_H
#define NRF_ABSTRACT_MOTION_CONTROL_H

//////////////////////////////////////////////////////////////////////////
// include files
#include <common/types.h>

namespace NJRobot
{

//////////////////////////////////////////////////////////////////////////
// define the abstract interface for motion control
class AbstractMotionControl {
public:
	/// Constructor
	AbstractMotionControl();
	/// Destructor
	virtual ~AbstractMotionControl() {}

	/// Set current laser
	void setObstaclePoints(const PointList & obs);

	void setObstacleData(const ObstacleData& od);

	/// Do motion control with two-boundary states
	// both states are in world coordinate
	// (x,y,theta) -> (mm,mm,rad)
	// is_real_final: Is terminal_state the final point. if TRUE, motion control should control the oriented angle
	void DoMotionControl( const RobotState& initial_state, const RobotState& terminal_state, bool is_real_final = false ){
		// 1. init
		m_initial_state = initial_state;//��ǰ��״̬�����Ի�ȡ����̼Ƶ�ǰ���ٶ�
		m_terminal_state = terminal_state;
		m_is_real_final = is_real_final;

		// 2. generate available velocity   �ó�һ�����ʵ��ٶ�Ȼ���ٴν��е���

		computeCurSpeed();

		// 3. keep safety���Ѿ�������ĺͳ����ķֿ���
		SafeAdjust();
		
		m_prev_speed = m_cur_speed;          //�������˶�����ģ���е�ǰ�ĺ�������ŵ�ǰ����ǰ�ٶ�
		m_obs_points.clear();                //��ձ������ϰ��������
	}

	//�Ƿ���ͣ��
	bool obsStopOccur();

	/// Get current velocity
	// (vx, vy, w) -> (m/s, m/s, rad/s)
	const RobotSpeed& GetCurSpeed() const;

	const RobotSpeed& GetPrevSpeed() const;

	void SetVehicleGeometry(const std::vector< Point >& cur_vehicle_geo);

protected:
	virtual void computeCurSpeed()=0;
	virtual void SafeAdjust()=0;

	PointList						m_obs_points; //��ǰ�����ϰ����,��������������ϵ
	ObstacleData					m_obs_data;  //��ǰ���������ϰ����

	RobotSpeed					    m_cur_speed;    //��ǰ��Ҫ�Ļ������ٶ� [m/s, m/s, rad/s]
	RobotSpeed					    m_prev_speed;   //�˶�����ģ���еĻ����˵�ǰ����ǰ�ٶ�

	double							m_vehicle_length;
	double							m_vehicle_width;
	PointList						m_vehicle_geometry;

	RobotState						m_initial_state;
	RobotState						m_terminal_state;
	bool							m_is_real_final;
	bool							m_obs_stop_flag;  //�������Ƿ���ͣ��

	double PredictCollision(double v , double w , const Point &obstacle , const Line& line);
	double PredictCollision(double v , double w);

};
}
#endif	// ~NRF_ABSTRACT_MOTION_CONTROL_H