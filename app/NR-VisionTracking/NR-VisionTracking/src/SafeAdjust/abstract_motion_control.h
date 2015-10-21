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
		m_initial_state = initial_state;//当前的状态，可以获取到里程计当前的速度
		m_terminal_state = terminal_state;
		m_is_real_final = is_real_final;

		// 2. generate available velocity   得出一个合适的速度然后再次进行调整

		computeCurSpeed();

		// 3. keep safety，已经将激光的和超声的分开了
		SafeAdjust();
		
		m_prev_speed = m_cur_speed;          //这里是运动控制模块中当前的和用来存放当前的先前速度
		m_obs_points.clear();                //清空本周期障碍物的数据
	}

	//是否发生停障
	bool obsStopOccur();

	/// Get current velocity
	// (vx, vy, w) -> (m/s, m/s, rad/s)
	const RobotSpeed& GetCurSpeed() const;

	const RobotSpeed& GetPrevSpeed() const;

	void SetVehicleGeometry(const std::vector< Point >& cur_vehicle_geo);

protected:
	virtual void computeCurSpeed()=0;
	virtual void SafeAdjust()=0;

	PointList						m_obs_points; //当前避障障碍物点,机器人自身坐标系
	ObstacleData					m_obs_data;  //当前超声检测的障碍物点

	RobotSpeed					    m_cur_speed;    //当前需要的机器人速度 [m/s, m/s, rad/s]
	RobotSpeed					    m_prev_speed;   //运动控制模块中的机器人当前和先前速度

	double							m_vehicle_length;
	double							m_vehicle_width;
	PointList						m_vehicle_geometry;

	RobotState						m_initial_state;
	RobotState						m_terminal_state;
	bool							m_is_real_final;
	bool							m_obs_stop_flag;  //机器人是否发生停障

	double PredictCollision(double v , double w , const Point &obstacle , const Line& line);
	double PredictCollision(double v , double w);

};
}
#endif	// ~NRF_ABSTRACT_MOTION_CONTROL_H