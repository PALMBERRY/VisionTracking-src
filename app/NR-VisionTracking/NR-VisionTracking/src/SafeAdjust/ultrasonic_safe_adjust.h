#pragma once
#include "abstract_safe_adjust.h"
#include <common/robot_type.h>

namespace NJRobot{

class UltrasonicSafeAdjust: public AbstractSafeAdjust<ObstacleData>
{
public:
	UltrasonicSafeAdjust(void);
	~UltrasonicSafeAdjust(void);
	
	void safeSpeedAdjust(RobotSpeed& send_speed,const ObstacleData & obs,const RobotSpeed& actual_speed);

};

}
