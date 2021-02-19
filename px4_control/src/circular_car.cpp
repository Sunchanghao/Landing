/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2019.12.31
* Description: Autonomous circular trajectory in offboard mode for amov_car
***************************************************************************************************************************/

#include <ros/ros.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
Eigen::Vector3d pos_target;//offboard模式下，发送给飞控的期望值
float desire_Radius = 15;//期望圆轨迹半径
float MoveTimeCnt = 0;
float priod = 2000.0;   //调此数值可改变走圆形的速度
Eigen::Vector3d temp_pos_drone;
Eigen::Vector3d temp_pos_target;
mavros_msgs::SetMode mode_cmd;
ros::Publisher setpoint_raw_local_pub;
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
mavros_msgs::CommandBool arm_cmd;

enum
{
  WAITING,		//等待offboard模式
	CHECKING,		//检查小车状态
	RUN,			  //跑圆形路经
	RUNOVER,		//结束		
}	RunState = WAITING;//初始状态WAITING

//接收来自飞控的当前小车位置
Eigen::Vector3d pos_drone;                     
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone = pos_drone_fcu_enu;
}

//接收来自飞控的当前小车状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
	current_state = *msg;
}

//发送位置期望值至飞控（输入：期望xyz,期望yaw）
void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

//状态机更新
void run_state_update(void)
{

	switch(RunState)
	{
		case WAITING:
			if(current_state.mode != "OFFBOARD" || current_state.armed == false)//等待offboard模式
			{
				pos_target[0] = pos_drone[0];
				pos_target[1] = pos_drone[1];
				pos_target[2] = pos_drone[2];
				temp_pos_drone[0] = pos_drone[0];
				temp_pos_drone[1] = pos_drone[1];
				temp_pos_drone[2] = pos_drone[2];
				send_pos_setpoint(pos_target, 0);
				cout << "current_x"<< pos_target[0]<<endl;
				cout << "current_y"<< pos_target[1]<<endl;
			}
			else
			{
				pos_target[0] = temp_pos_drone[0];
				pos_target[1] = temp_pos_drone[1];
				pos_target[2] = temp_pos_drone[2];
				send_pos_setpoint(pos_target, 0);
				RunState = CHECKING;
			}
			cout << "WAITING" <<endl;
			break;
		case CHECKING:
			if(pos_drone[0] == 0 && pos_drone[1] == 0) 			//没有位置信息则执行上锁
			{
				cout << "Check error, make sure have local location" <<endl;
        arm_cmd.request.value = false;
        arming_client.call(arm_cmd);
				RunState = WAITING;	
			}
			else
			{
				RunState = RUN;
				MoveTimeCnt = 0;
			}
			cout << "CHECKING" <<endl;
			break;
			case RUN:
			{
				MoveTimeCnt++;
				if(MoveTimeCnt <= 1)
				{
					pos_target[0] = temp_pos_drone[0] + 10;
					pos_target[1] = temp_pos_drone[1];
					cout << "step1" <<endl;
				}
				else if(pos_drone[0] <= (temp_pos_drone[0]+10+0.5) && pos_drone[0] >= (temp_pos_drone[0]+10-0.5) && pos_drone[1] <= (temp_pos_drone[1]+0.5) && pos_drone[1] >= (temp_pos_drone[1]-0.5) )
				{
					pos_target[0] = temp_pos_drone[0] + 10;
					pos_target[1] = temp_pos_drone[1] + 10;
					cout << "step2" <<endl;
				}
				else if(pos_drone[0] <= (temp_pos_drone[0]+10+0.5) && pos_drone[0] >= (temp_pos_drone[0]+10-0.5) && pos_drone[1] <= (temp_pos_drone[1]+10+0.5) && pos_drone[1] >= (temp_pos_drone[1]+10-0.5) )
				{
					pos_target[0] = temp_pos_drone[0];
					pos_target[1] = temp_pos_drone[1] + 10;
					cout << "step3" <<endl;
				}
				else if(pos_drone[0] <= (temp_pos_drone[0]+0.5) && pos_drone[0] >= (temp_pos_drone[0]-0.5) && pos_drone[1] <= (temp_pos_drone[1]+10+0.5) && pos_drone[1] >= (temp_pos_drone[1]+10-0.5) )
				{
					pos_target[0] = temp_pos_drone[0];
					pos_target[1] = temp_pos_drone[1];
					cout << "step4" <<endl;
				}

				pos_target[2] = 0;
				cout << "desire_x"<< pos_target[0]<<endl;
				cout << "desire_y"<< pos_target[1]<<endl;
				send_pos_setpoint(pos_target, 0);
				if(current_state.mode != "OFFBOARD" || current_state.armed == false)			//如果在中途中切换到onboard，则跳到WAITING
				{
					RunState = WAITING;
				}
			}
			cout << "RUN" <<endl;
			break;
		case RUNOVER:
			{
		    	arm_cmd.request.value = false;
		    	arming_client.call(arm_cmd);
				RunState = WAITING;
			}
			cout << "RUNOVER" <<endl;
			break;

		default:
			cout << "error" <<endl;
	}

}				


int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_car_offboard");
    ros::NodeHandle nh("~");
    // 频率 [20Hz]
    ros::Rate rate(20.0);

    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 【服务】修改系统模式
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

   	nh.param<float>("desire_Radius", desire_Radius, 1.0);
    arm_cmd.request.value = true;
    while(ros::ok())
    {
			run_state_update();
	 		ros::spinOnce();
      rate.sleep();
    }

    return 0;

}


