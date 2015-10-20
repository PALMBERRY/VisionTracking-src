#include "vision_navigation.h"

VisionNavigation::VisionNavigation(void)
{
}

VisionNavigation::~VisionNavigation(void)
{
}

void VisionNavigation::setCurOdom( const OrientedPoint& odom )
{
	previous_odom_ = cur_odom_;
	cur_odom_ = odom;
}

void VisionNavigation::setCurPose( const OrientedPoint& pose )
{
	cur_pose_ = pose;
	start_pose_ = pose;
	//printf("get init_pose\n");
}


// 计算两个角度的差
double VisionNavigation::errorNormalize(double angle1, double angle2)
{
	//当前值 - 目标值
	double error = angle1 - angle2;
	if(abs(error) > M_PI)
	{
		if(error > 0)
			error = error - 2.0*M_PI;
		else
			error = error + 2.0*M_PI;
	}
	return error;
}

// 计算两点连线的角度
double VisionNavigation::angle_2point(OrientedPoint A,OrientedPoint B)
{
	if(B.x-A.x == 0)
	{
		if(B.y<A.y)
			return -1.0*M_PI/2.0;
		else
			return M_PI/2.0;
	}
	else
	{
		if(B.y>A.y)
		{
			if(B.x>A.x)
				return atan((B.y-A.y)/(B.x-A.x));
			else
				return atan((B.y-A.y)/(B.x-A.x))+M_PI;
		}
		else
		{
			if(B.x>A.x)
				return atan((B.y-A.y)/(B.x-A.x));
			else
				return atan((B.y-A.y)/(B.x-A.x))-M_PI;
		}
	}
}

// 获取LandMark
void VisionNavigation::setInitLandmark()
{
	fp=fopen("landmark.txt","r+");
	for(landmark_num=0;!feof(fp);landmark_num++)
		fscanf(fp,"%lf%lf%lf",&landmark_point_global[landmark_num].x,&landmark_point_global[landmark_num].y,&landmark_point_global[landmark_num].theta);
	printf("Read landmark_info complete, get %d landmarks\n",landmark_num);
	fclose(fp);
}

// 找出当前视野里面的landmark的信息
bool VisionNavigation::whichLandmark()
{
	int which_one;
	double real_dist = 10000;
	for(which_one=0;which_one<=landmark_num;which_one++)
	{
	//	if( errorNormalize(landmark_point_global[which_one].theta,vline_angle_)<10/M_PI*180.0 && 
		if(euclidianDist(landmark_point_global[which_one],cur_pose_)<2.0 &&
			euclidianDist(landmark_point_global[which_one],cur_pose_)<real_dist)
		{
			landmark_point_global_.x = landmark_point_global[which_one].x;
			landmark_point_global_.y = landmark_point_global[which_one].y;
			landmark_point_global_.theta = landmark_point_global[which_one].theta;
			real_dist = euclidianDist(landmark_point_global[which_one],cur_pose_);
		}
	}
	if(real_dist == 10000)
		return false;
	else
	{
		//printf("GET THIS LANDMARK\n");
		return true;
	}
}



void VisionNavigation::localization()
{
//===TODO====//
	//里程计航位推算
	//printf("localization begin! %d\n",first_flag);
	if(task_num)
	{
		if(first_flag)
		{
			setInitLandmark();
			printf("------------------------------------localization begin!-------------------------------------------\n");
			//fp1=fopen("motion.txt","w+");
		}
		OrientedPoint motion = absoluteDifference(cur_odom_, previous_odom_); //以old_odm为原点的motion
		cur_pose_ = absoluteSum(cur_pose_,motion);//add motion to cur_pose_，motion以cur_pose_为原点
		//fprintf(fp1,"cur_odom_:%lf %lf %lf previous_odom_:%lf %lf %lfmotion:%lf %lf %lfstart_pose:%lf,%lf,%lf cur_pose:%lf,%lf,%lf\n",
		//	cur_odom_.x,cur_odom_.y,cur_odom_.theta*180/M_PI,previous_odom_.x,previous_odom_.y,previous_odom_.theta*180/M_PI,
		//	motion.x,motion.y,motion.theta*180/M_PI,start_pose_.x,start_pose_.y,start_pose_.theta*180/M_PI,cur_pose_.x,cur_pose_.y,cur_pose_.theta*180/M_PI);
		//printf("start_pose:%lf,%lf,%lf cur_pose:%lf,%lf,%lf\n",start_pose_.x,start_pose_.y,start_pose_.theta*180/M_PI,cur_pose_.x,cur_pose_.y,cur_pose_.theta*180/M_PI);
		//printf("cur_odom_:%lf %lf %lf\n",cur_odom_.x,cur_odom_.y,cur_odom_.theta*180/M_PI);
		//根据检测到的直线角度，修正机器人当前的航向
		if(has_vline_info_ && !turned_flag) 
		{
			double tmp_theta = cur_pose_.theta;
			double e[] = {abs(cur_pose_.theta-M_PI/2.0),abs(cur_pose_.theta-0),abs(cur_pose_.theta+M_PI/2.0),abs(abs(cur_pose_.theta)-M_PI)};
			//求最小值
			int pos_flag = 0;//当前位置标志位，只有四种情况:90度、0度、-90度、180度
			double tmp = e[0];		
			for(int i=1;i<4;i++)
			{
				if(e[i]<tmp)
				{
					pos_flag = i;
					tmp = e[i];
				}
			}
			switch(pos_flag)
			{
				case 0:cur_pose_.theta = M_PI - vline_angle_*M_PI/180.0; break; //当前航向在90度附近
				case 1:cur_pose_.theta = M_PI/2.0 - vline_angle_*M_PI/180.0; break;//当前航向在0度附近
				case 2:cur_pose_.theta = -vline_angle_*M_PI/180.0;break;//当前航向在-90度附近
				case 3:cur_pose_.theta = M_PI*1.5-vline_angle_*M_PI/180.0;break;//当前航向在180度附近
			}
			cur_pose_.normalize();
			// 如果直线检测的角度值偏离里程计的角度值较远，则不信直线的结果
			if(errorNormalize(cur_pose_.theta,tmp_theta)> 15*M_PI/180.0)
			{
				cur_pose_.theta = tmp_theta;
				has_vline_info_ = false;//标志位也设置为false
			}
			//	has_vline_info_ = false; TE:once data used, set this to FALSE
		}

		//根据路标点修正定位

		if(has_vlandmark_info_)
		{

			Point tmp(landmark_point_.y/1000.0,-landmark_point_.x/1000.0); //将landmark点旋转90度->(y,-x)，因为图像检测的坐标系的原因，即坐标系逆时针转90度
			double c = cos(cur_pose_.theta);
			double s = sin(cur_pose_.theta);
			if(whichLandmark())	//确定当前路标点的坐标
			{
				Point tmp_position;
				//tmp_position.x = landmark_point_global_.x - (c*tmp.x - s*tmp.y);
				//tmp_position.y = landmark_point_global_.y - (s*tmp.x + c*tmp.y);
				if(vline_dist_left_!=-1)
				{
					tmp_position.x = landmark_point_global_.x - (c*tmp.x - s*tmp.y);
					tmp_position.y = landmark_point_global_.y - (s*tmp.x + c*tmp.y);
				}
				else
				{
					tmp_position.x = landmark_point_global_.x - (c*tmp.x - s*tmp.y);
					tmp_position.y = landmark_point_global_.y + (s*tmp.x + c*tmp.y);
				}
				//防止误检：误检距离[]米
				double error = euclidianDist(tmp_position,Point(cur_pose_.x,cur_pose_.y));
				if(error < 0.3)
				{
				//	std::cout<<"标记点距离偏差："<<error<<std::endl;
				//	std::cout<<"标记点校正前："<<cur_pose_.x<<","<<cur_pose_.y<<std::endl;
					cur_pose_.x = tmp_position.x;
					cur_pose_.y = tmp_position.y;
				//	std::cout<<"标记点校正后："<<cur_pose_.x<<","<<cur_pose_.y<<std::endl;
					has_vlandmark_info_ = false;
				}
				else
				{
					has_vlandmark_info_ = false;
				//	std::cout<<"不信路标点"<<std::endl;
				//	printf("(%lf, %lf) (%lf, %lf) (%lf,%lf) (%lf,%lf) %lf\n",landmark_point_.x/1000.0, landmark_point_.y/1000.0, tmp_position.x, tmp_position.y, cur_pose_.x, cur_pose_.y, landmark_point_global_.x, landmark_point_global_.y, cur_pose_.theta);
				}
			}
			else
				has_vlandmark_info_ = false;
			//std::cout<<"标记点校正："<<std::endl;
		}
		//导航完成后，修正机器人的位置：当前位置在两个工位点所在直线上
		//if(!task_complete)
		//{
		//	//当前位置点对直线的投影
		//	//matlab计算结果：x = (x3*x1^2 - 2*x3*x1*x2 - x1*y1*y2 + y3*x1*y1 + x1*y2^2 - y3*x1*y2 + x3*x2^2 + x2*y1^2 - x2*y1*y2 - y3*x2*y1 + y3*x2*y2)/(x1^2 - 2*x1*x2 + x2^2 + y1^2 - 2*y1*y2 + y2^2);
		//	//matlab计算结果：y = (x1^2*y2 - x1*x2*y1 - x1*x2*y2 + x3*x1*y1 - x3*x1*y2 + x2^2*y1 - x3*x2*y1 + x3*x2*y2 + y3*y1^2 - 2*y3*y1*y2 + y3*y2^2)/(x1^2 - 2*x1*x2 + x2^2 + y1^2 - 2*y1*y2 + y2^2);
		//	double x1 = previous_target_point_.x;
		//	double y1 = previous_target_point_.y;
		//	double x2 = target_point_.x;
		//	double y2 = target_point_.y;
		//	double x3 = cur_pose_.x;
		//	double y3 = cur_pose_.y;
		//	double tmp = x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2;
		//	cur_pose_.x = (x3*x1*x1 - 2*x3*x1*x2 - x1*y1*y2 + y3*x1*y1 + x1*y2*y2 - y3*x1*y2 + x3*x2*x2 + x2*y1*y1 - x2*y1*y2 - y3*x2*y1 + y3*x2*y2)/tmp;
		//	cur_pose_.y = (x1*x1*y2 - x1*x2*y1 - x1*x2*y2 + x3*x1*y1 - x3*x1*y2 + x2*x2*y1 - x3*x2*y1 + x3*x2*y2 + y3*y1*y1 - 2*y3*y1*y2 + y3*y2*y2)/tmp;
		//}
		lave_percent_ = euclidianDist(cur_pose_,target_point_)/euclidianDist(previous_target_point_,target_point_) * 100.0;
		if(lave_percent_>=100)
			lave_percent_ = 0;
		else
			lave_percent_ = 100 - lave_percent_;
		//printf("After localization Percent: %lf/%lf = %lf\n",euclidianDist(cur_pose_,target_point_),euclidianDist(previous_target_point_,target_point_),lave_percent_);
	}
}

void VisionNavigation::navigation()
{
	//printf("navigation begin! %d %d\n",first_flag,task_complete);
	if(task_num)
	{
		if(!task_complete)
		{
			///////////////////////////Handle Data Module/////////////////////////////////
			if(!first_flag)		// Handle turn
			{
				if(turned_flag)
				{
					Move_flag = 0;
					pre_theta = cur_pose_.theta;
					abs_turn = angle_2point(cur_pose_,target_point_);
					//abs_turn = angle_2point(start_pose_,target_point_);
					if(rejudge_flag)
					{
						//if(errorNormalize(abs_turn, 0)<15*M_PI/180.0 || errorNormalize(abs_turn,M_PI)<15*M_PI/180.0)
						//{
						//	keep_flag = true;	//keep y track x
						//}
						if(fabs(errorNormalize(abs_turn,M_PI/2.0))<30*M_PI/180.0 || fabs(errorNormalize(abs_turn,-1.0*M_PI/2.0))<30*M_PI/180.0)
						{
							keep_flag = false;	//keep x track y
						}
						else
						{
							keep_flag = true;
						}
						rejudge_flag = false;
					}

					need_turn = errorNormalize(cur_pose_.theta,abs_turn);
					//if(need_turn > M_PI)
					//	need_turn = 2.0*M_PI-need_turn;
					//if(need_turn < -1.0*M_PI)
					//	need_turn = -1.0*(2.0*M_PI+need_turn);

					Turn_flag = 1;
					Noline_flag = 0;
					//if(errorNormalize(cur_pose_.theta,0) < 15*M_PI/180.0)
					//{						
					//	//keep_flag = false;	// befor    ^ y		keep x track y
					//							//			|
					//							//	   -----|----> x
					//							//			|

					//	//keep_flag = true;		// now	    ^ x		keep y track x
					//							//		 	|
					//							//	   y<---|----
					//							//			|
					//	if(cur_pose_.y > target_point_.y)
					//		Turn_flag = -1;
					//	else
					//		Turn_flag = 1;
					//}
					//if(fabs(cur_pose_.theta/M_PI*180 - 90) < 15)
					//{
					//	keep_flag = true;	//keep y track x
					//	if(cur_pose_.x < target_point_.x)
					//		Turn_flag = -1;
					//	else
					//		Turn_flag = 1;
					//}
					//if(fabs(cur_pose_.theta/M_PI*180 + 90) < 15)
					//{
					//	keep_flag = true;	//keep y track x
					//	if(cur_pose_.x > target_point_.x)
					//		Turn_flag = -1;
					//	else
					//		Turn_flag = 1;
					//}
					//if(fabs(abs(cur_pose_.theta)/M_PI*180 - 180) < 15)
					//{
					//	keep_flag = false;	//keep x track y
					//	if(cur_pose_.y < target_point_.y)
					//		Turn_flag = -1;
					//	else
					//		Turn_flag = 1;
					//}
					//printf("Turn_flag = %d\n", Turn_flag);	
					//rejudge_flag = false;
				}
				else
				{
					if(fabs(cur_pose_.x-target_point_.x)<3 && fabs(cur_pose_.y-target_point_.y)<3)
					{
						if(fabs(cur_pose_.x-target_point_.x)<1.0 && fabs(cur_pose_.y-target_point_.y)<1.0)	
						{
							speed_flag = true;
							//dist_speed = euclidianDist(cur_pose_,target_point_);
							if(keep_flag)
							{
								dist_speed = fabs(cur_pose_.x-target_point_.x);
								if(fabs(cur_pose_.x-target_point_.x)>0.05)	//task complete
								{
									Turn_flag = 1;
									//keep_flag = mid_flag;
								}
								else
									task_complete = true;
							}
							else
							{
								dist_speed = fabs(cur_pose_.y-target_point_.y);
								if(fabs(cur_pose_.y-target_point_.y)>0.05)	//task complete
								{
									Turn_flag = 1;
									//keep_flag = mid_flag;
								}
								else
									task_complete = true;
							}
							need_detect_landmark = false;
						}
						//landmark detect
						need_detect_landmark = true;
					}

					if(!has_vline_info_)			// tiaozhen
					{
						//printf("tiaozhen\n");
						Noline_flag++;
						if(Noline_flag>3)			// 3zhen
						{
							if(need_detect_landmark)
							{
								//printf("mangzhou\n");
								Move_flag = -1;
								if(keep_flag)
									temp_keep.y = cur_pose_.y;
								else
									temp_keep.x = cur_pose_.x;
							}
							else
							{
								//emergency_flag = true;
								emergency_num ++;
								//Move_flag = 0;
								Move_flag = -1;
								//Turn_flag = 0;
								//stop_can_recovery = true;
								//printf("I am stopped,but i think i can recovery %d!\n", emergency_num);
							}
						}
						else
						{
							//printf("lvbo\n");
							Noline_flag = 0;
							Move_flag = 1;
							if(keep_flag)
									temp_keep.y = cur_pose_.y;
								else
									temp_keep.x = cur_pose_.x;
							lvbo_flag = true;
						}
					}
					else
					{
						//printf("no tiao zhen\n");
						//if(pre_vline_flag)
						//	Move_flag = 1;
						//else
						//{
							//printf("caiji qianyizhen data\n");
						lvbo_flag = false;
						pre_angle = vline_angle_;
						pre_dist = vline_dist;
						Move_flag = 1;
						Turn_flag = 0;
						pre_vline_flag = true;
						//}
					}
				}
			}
			else
			{
				printf("------------------------------------Navigation   begin!-------------------------------------------\n");
				first_flag = false;
				Noline_flag = 0;
				Move_flag = 0;
				Turn_flag = 0;
				//if(fabs(cur_pose_.y-target_point_.y)>1.0)
				//	keep_flag = false;	//keep x  y change
				//else
				//	keep_flag = true;	//keep y  x change
			}


			///////////////////////////Handle Data Module Over////////////////////////////

			//////////////////////////////Navigation Module///////////////////////////////
			if(Move_flag)
			{
				emergency_flag = false;
				stop_can_recovery = false;
				emergency_num = 0;
				if(speed_flag)
				{
					speed_v_ = 0.05 + dist_speed*0.45;
				}
				else
					speed_v_ = 0.5;
				if(Move_flag==1)
				{
					if(lvbo_flag)
					{
						//speed_w_ = (pre_angle-90)*0.01 + (pre_dist-LineDist)*0.001;
						if(keep_flag)
						{
							//speed_w_ = get_param(cur_pose_.theta)*0.01+(cur_pose_.y-temp_keep.y)*0.07;
							speed_w_ = (cur_pose_.y-temp_keep.y)*0.07;
						}
						else
						{
							//speed_w_ = get_param(cur_pose_.theta)*0.01+(cur_pose_.x-temp_keep.x)*0.07;
							speed_w_ = (cur_pose_.x-temp_keep.x)*0.07;
						}
						//lvbo_flag = false;
					}
					else
					{
						//speed_w_ = (vline_angle_-90)*0.01 + (vline_dist-LineDist)*0.001;
						if(vline_dist_left_!=-1)
							speed_w_ = (vline_angle_-90)*0.01 + (vline_dist-LineDist)*0.0007;
						else
							speed_w_ = (vline_angle_-90)*0.01 - (vline_dist-LineDist)*0.0007;

						pre_angle = vline_angle_;
						pre_dist = vline_dist;
					}
				}
				if(Move_flag==-1)
				{
					//printf("I am MANGZOUing\n");
					if(keep_flag)
					{
						//speed_w_ = (cur_pose_.theta*180.0/M_PI-90)*0.01+(cur_pose_.y-temp_keep.y)*0.07;
						speed_w_ = (cur_pose_.y-temp_keep.y)*0.07;
					}
					else
					{
						//speed_w_ = (cur_pose_.theta*180.0/M_PI-90)*0.01+(cur_pose_.x-temp_keep.x)*0.07;
						speed_w_ = (cur_pose_.x-temp_keep.x)*0.07;
					}
				}
				//printf("POSE:%lf,%lf,%lf  NAVI:%lf m/s,%lf rad/s DIST:%lf\n",cur_pose_.x, cur_pose_.y, cur_pose_.theta*180.0/M_PI,speed_v_,speed_w_,vline_dist_);
			}
			else
			{
				speed_v_ = 0;
				if(Turn_flag==0)
				{
					speed_w_ = 0;
					//if(emergency_num>500)
						//emergency_flag = true;
				}
				else
				{
					//emergency_flag = false;
					if(fabs(need_turn)>M_PI/90.0) //2.0 deg
						speed_w_ = 0 - (0.2*need_turn/fabs(need_turn) + need_turn * 0.2);
					else
						turned_flag = false;
				}
				//if(Turn_flag==1)
				//{
				//	if(abs(pre_theta)>150/180.0*M_PI)
				//	{
				//		if(fabs(abs(pre_theta)-abs(cur_pose_.theta))<M_PI/2.0)
				//		{
				//			//printf("pre_theta:%lf---cur_pose_.theta:%lf 目标点：%lf %lf\n", pre_theta, cur_pose_.theta, target_point_.x, target_point_.y);
				//			//speed_w_ = 0.2;
				//			speed_w_ = 0.1 + (M_PI/2.0 - fabs(abs(pre_theta)-abs(cur_pose_.theta))) * 0.2;
				//		}
				//		else
				//		{
				//			turned_flag = false;
				//		}
				//	}
				//	else
				//	{
				//		if(fabs(pre_theta-cur_pose_.theta)<M_PI/2.0)
				//		{
				//			//printf("pre_theta:%lf---cur_pose_.theta:%lf 目标点：%lf %lf\n", pre_theta, cur_pose_.theta, target_point_.x, target_point_.y);
				//			//speed_w_ = 0.2;
				//			speed_w_ = 0.1 + (M_PI/2.0 - fabs(pre_theta-cur_pose_.theta)) * 0.2;
				//		}
				//		else
				//		{
				//			turned_flag = false;
				//		}
				//	}
				//}
				//if(Turn_flag==-1)
				//{
				//	if(fabs(pre_theta)>150/180.0*M_PI)	//处理180-->-180的问题
				//	{
				//		if(fabs(fabs(pre_theta)-fabs(cur_pose_.theta))<M_PI/2.0)
				//		{
				//			//printf("pre_theta:%lf---cur_pose_.theta:%lf 目标点：%lf %lf\n", pre_theta, cur_pose_.theta, target_point_.x, target_point_.y);
				//			//speed_w_ = -0.3;
				//			speed_w_ = -0.1 - (M_PI/2.0 - fabs(fabs(pre_theta)-fabs(cur_pose_.theta))) * 0.2;
				//		}
				//		else
				//		{
				//			turned_flag = false;
				//		}
				//	}
				//	else
				//	{
				//		if(fabs(pre_theta-cur_pose_.theta)<M_PI/2.0)
				//		{
				//			//printf("pre_theta:%lf---cur_pose_.theta:%lf 目标点：%lf %lf\n", pre_theta, cur_pose_.theta, target_point_.x, target_point_.y);
				//			//speed_w_ = -0.3;
				//			speed_w_ = -0.1 - (M_PI/2.0 - fabs(pre_theta-cur_pose_.theta)) * 0.2;
				//		}
				//		else
				//		{
				//			turned_flag = false;
				//		}
				//	}
				//}
				//printf("POSE:%lf,%lf,%lf  NAVI:%lf m/s,%lf rad/s DIST:%lf\n",cur_pose_.x, cur_pose_.y, cur_pose_.theta*180.0/M_PI,speed_v_,speed_w_,vline_dist);
			}
			/////////////////////////////Navigation Module Over/////////////////////////
			//printf("abs:%lf TURN:%lf keep_flag:%d ",abs_turn*180/M_PI,need_turn*180/M_PI,keep_flag);
		//	printf("POSE:%lf,%lf,%lf  NAVI:%lf m/s,%lf rad/s DIST:%lf Theta:%lf RS:%lf lvbo:%d\n",cur_pose_.x, cur_pose_.y, cur_pose_.theta*180.0/M_PI,speed_v_,speed_w_,vline_dist_,vline_angle_,(vline_angle_-90)*0.01 + (vline_dist_-LineDist)*0.001,lvbo_flag);
		}
		else
		{
			speed_v_ = 0;
			speed_w_ = 0;
			need_detect_landmark = false;
			has_vlandmark_info_ = false;
			turned_flag = true;
			rejudge_flag = true;
			speed_flag = false;
			speed_ = 0;
			Move_flag = 0;
			Turn_flag = 0;
			pre_vline_flag = false;
			task_num = false;
			lave_percent_ = 100; //arrive	
			printf("Arrive the Target:%lf %lf\n",target_point_.x, target_point_.y);
		}
	}
}

void VisionNavigation::setVlineInfo(bool haveline,double dist_left,double dist_right,double ang )
{
	has_vline_info_ = haveline;
	vline_dist_left_ = dist_left;
	vline_dist_right_ = dist_right;
	vline_angle_ = ang;

	if(vline_dist_left_!=-1 && vline_dist_right_!=-1)	//both
	{
		LineDist = (vline_dist_left_ + vline_dist_right_)/2.0;
		vline_dist = vline_dist_left_;
		//printf("BOTH!!tracking mid!! vline_dist = %lf LineDist = %lf and line flag = %d\n",vline_dist,LineDist,has_vline_info_);
	}

	if(vline_dist_left_==-1 && vline_dist_right_!=-1) // track right
	{
		vline_dist = vline_dist_right_;
		LineDist = 450;
		//printf("tracking right!! vline_dist = %lf LineDist = %lf and line flag = %d\n",vline_dist,LineDist,has_vline_info_);
	}

	if(vline_dist_left_!=-1 && vline_dist_right_==-1)	// track left
	{
		vline_dist = vline_dist_left_;
		LineDist = 450;	
		//printf("tracking left!! vline_dist = %lf LineDist = %lf and line flag = %d\n",vline_dist,LineDist,has_vline_info_);
	}

	if(vline_dist_left_==-1 && vline_dist_right_==-1)	// no line
	{
		has_vline_info_ = false;
		//printf("NO LINE!! vline_dist = %lf LineDist = %lf and line flag = %d\n",vline_dist,LineDist,has_vline_info_);
	}
}

//void VisionNavigation::setFist_flag( bool start_flag )
//{
//	first_flag = start_flag;	
//	turned_flag= true;
//	//turned_flag= false;
//	pre_vline_flag = false;
//	lvbo_flag = false; 
//	emergency_flag = false;
//	need_detect_landmark = false; //是否需要检测标记点
//	rejudge_flag = true;
//	speed_flag = false;
//	speed_ = 0;
//	task_num = true;
//	//landmark_point_global_.x = -0.35;
//	//landmark_point_global_.y = 9.87;
//}

bool VisionNavigation::get_landmark_flag(void)
{
	return need_detect_landmark;
}

void VisionNavigation::setVlandmarkInfo( const Point& p )
{
	has_vlandmark_info_ = true;
	landmark_point_ = p;
}

//void VisionNavigation::setTargetPoint( double x,double y)
void VisionNavigation::setTargetPoint( OrientedPoint target_task)
{
	target_point_ = target_task;
	lave_percent_ = 0;
	task_complete = false;	
	turned_flag= true;
	pre_vline_flag = false;
	lvbo_flag = false; 
	stop_can_recovery = false;
	emergency_num = 0;
	emergency_flag = false;
	need_detect_landmark = false; //是否需要检测标记点
	rejudge_flag = true;
	speed_flag = false;
	speed_ = 0;
	task_num = true;
	first_flag = true;
	//std::cout<<x<<","<<y<<std::endl;
	//printf("目标地为%lf %lf\n",target_point_.x, target_point_.y);
}

void VisionNavigation::setInitialPreviousTarget( OrientedPoint pre_target )
{
	previous_target_point_ = pre_target;
}

void VisionNavigation::setInitialPreviousOdom()
{
	previous_odom_ = cur_odom_;
}


OrientedPoint VisionNavigation::getCurPose()
{
	return cur_pose_;
}

void VisionNavigation::getNavResult( double & v,double& w )
{
	v = speed_v_;
	w = speed_w_;
}

Vision_Navigation_RobotState VisionNavigation::getVisionNavigation_RobotState()
{
	Vision_Navigation_RobotState pubstate_;
	pubstate_.after_VN_RobotPose = cur_pose_;
	pubstate_.stop_or_not = stop_can_recovery;
	pubstate_.lave_percent = lave_percent_;
	pubstate_.can_do = emergency_flag;
	return pubstate_;
}
