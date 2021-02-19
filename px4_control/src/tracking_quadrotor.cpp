/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.03.10
* Description: 实现px4 quadrotor 二维码跟踪
***************************************************************************************************************************/
#include "tracking_quadrotor.h"
#include "FuzzyPID.h"
#include <iostream>
#include <math.h>
#include "geometry_msgs/Twist.h"


using namespace std;
using namespace Eigen;

//fuzzypid param
float e1_max=10;
float e1_min=-10;
float ec1_max=1;
float ec1_min=-1;
float vx=0;

float e2_max=3;
float e2_min=-3;
float ec2_max=1;
float ec2_min=-1;
float vy =0;

float e3_max=10;
float e3_min=-10;
float ec3_max=1;
float ec3_min=-1;
//前后
float kp1_max=0.9;
float kp1_min= 0.6;
float ki1_max= 0.1;
float ki1_min= 0.06;
float kd1_max= 0.06;
float kd1_min=  0.02;
//左右
float kp2_max=1.5;
float kp2_min= 0.9;
float ki2_max= 0.06;
float ki2_min= 0.04;
float kd2_max= 0.08;
float kd2_min= 0.06;
//上下
float kp3_max=1;
float kp3_min=0.1;
float ki3_max=0.04;
float ki3_min=0.02;
float kd3_max=0.06;
float kd3_min=0.04;

float xerro;
float xerro_c;
float xerro_pre=0;
float xerrp_ppre=0;
float dx=0;
float xsumE=0;


float yerro;
float yerro_c;
float yerro_pre=0;
float yerrp_ppre=0;
float dy;
float ysumE=0;

float zerro;
float zerro_c;
float dz;
float zerro_pre=0;
float zerrp_ppre=0;

float Vx;

FuzzyPID fuzzypid;

FuzzyPID::FuzzyPID()  //构造函数
{
	kp = 0;
	ki = 0;
	kd = 0;
	fuzzy_output = 0;
	qdetail_kp = 0;
	qdetail_ki = 0;
	qdetail_kd = 0;
	qfuzzy_output = 0;
	errosum = 0;
}
 
FuzzyPID::~FuzzyPID()//析构函数
{
}
 
//////////////////////输入e与de/dt隶属度计算函数///////////////////////////
void FuzzyPID::Get_grad_membership(float erro,float erro_c)   
{
	if (erro > e_membership_values[0] && erro < e_membership_values[6])
	{
		for (int i = 0; i < num_area - 2; i++)
		{
			if (erro >= e_membership_values[i] && erro <= e_membership_values[i + 1])
			{
				//计算出来的隶属度  是数值
				e_gradmembership[0] = -(erro - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
				e_gradmembership[1] = 1+(erro - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
				e_grad_index[0] = i;
				e_grad_index[1] = i + 1;
				break;
			}
		}
	}
	else
	{
		if (erro <= e_membership_values[0])
		{
			e_gradmembership[0] = 1;
			e_gradmembership[1] = 0;
			e_grad_index[0] = 0;
			e_grad_index[1] = -1;
		}
		else if (erro >= e_membership_values[6])
		{
			e_gradmembership[0] = 1;
			e_gradmembership[1] = 0;
			e_grad_index[0] = 6;
			e_grad_index[1] = -1;
		}
	}
 
	if (erro_c > ec_membership_values[0] && erro_c < ec_membership_values[6])
	{
		for (int i = 0; i < num_area - 2; i++)
		{
			if (erro_c >= ec_membership_values[i] && erro_c <= ec_membership_values[i + 1])
			{
				ec_gradmembership[0] = -(erro_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_gradmembership[1] = 1 + (erro_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_grad_index[0] = i;
				ec_grad_index[1] = i + 1;
				break;
			}
		}
	}
	else
	{
		if (erro_c <= ec_membership_values[0])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 0;
			ec_grad_index[1] = -1;
		}
		else if (erro_c >= ec_membership_values[6])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 6;
			ec_grad_index[1] = -1;
		}
	}
 
}
 
/////////////////////获取输出增量kp,ki,kd的总隶属度/////////////////////////////
void FuzzyPID::GetSumGrad()
{
	for (int i = 0; i <= num_area - 1; i++)
	{
		KpgradSums[i] = 0;
		KigradSums[i] = 0;
        KdgradSums[i] = 0;
 
	}
  for (int i=0;i<2;i++)
  {
	  if (e_grad_index[i] == -1)
	  {
	   continue;
	  }
	  for (int j = 0; j < 2; j++)
	  {
		  if (ec_grad_index[j] != -1)
		  {
			  int indexKp = Kp_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
			  int indexKi = Ki_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
			  int indexKd = Kd_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
			  //gradSums[index] = gradSums[index] + (e_gradmembership[i] * ec_gradmembership[j])* Kp_rule_list[e_grad_index[i]][ec_grad_index[j]];
			  KpgradSums[indexKp]= KpgradSums[indexKp] + (e_gradmembership[i] * ec_gradmembership[j]);
			  KigradSums[indexKi] = KigradSums[indexKi] + (e_gradmembership[i] * ec_gradmembership[j]);
			  KdgradSums[indexKd] = KdgradSums[indexKd] + (e_gradmembership[i] * ec_gradmembership[j]);
		  }
		  else
		  {
		    continue;
		  }
 
	  }
  }
 
}
 
////////////////////计算输出增量kp,kd,ki对应论域值//////////////////////
void FuzzyPID::GetOUT()
{
	for (int i = 0; i < num_area - 1; i++)
	{
		qdetail_kp += kp_menbership_values[i] * KpgradSums[i];
		qdetail_ki   += ki_menbership_values[i] * KigradSums[i];
		qdetail_kd += kd_menbership_values[i] * KdgradSums[i];
		 cout<<"论域中qdetail_kp:"<<qdetail_kp<<"  qdetail_ki:"<<qdetail_ki<<"  qdetail_kd:"<<qdetail_kd<<endl;
	}
}
 
//////////////////模糊PID控制实现函数/////////////////////////
Eigen::Vector3d FuzzyPID::FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float erro, float erro_c,float ki_max,float ki_min,float kd_max,float kd_min,float erro_pre,float errp_ppre)
{
	Eigen::Vector3d PID;
	//最后在输出 kp ki kd三个参数就好 
	errosum += erro;
	//Arear_dipart(e_max, e_min, ec_max, ec_min, kp_max, kp_min,ki_max,ki_min,kd_max,kd_min);
	qerro = Quantization(e_max, e_min, erro);
	   cout<<"e的论域值："<<qerro<<endl;
	qerro_c = Quantization(ec_max, ec_min, erro_c);
	  cout<<"ec的论域值："<<qerro_c<<endl;
	Get_grad_membership(qerro, qerro_c);
	GetSumGrad();
	GetOUT();
	detail_kp = Inverse_quantization(kp_max, kp_min, qdetail_kp);
	detail_ki  = Inverse_quantization(ki_max, ki_min, qdetail_ki);
	detail_kd = Inverse_quantization(kd_max, kd_min, qdetail_kd);
    //cout<<"detail_kp:"<<detail_kp<<"  detail_ki:"<<detail_ki<<"  detail_kd:"<<detail_kd<<endl;
	qdetail_kd = 0;
	qdetail_ki = 0;
	qdetail_kp = 0;
	/*if (qdetail_kp >= kp_max)
		qdetail_kp = kp_max;
	else if (qdetail_kp <= kp_min)
		qdetail_kp = kp_min;
	if (qdetail_ki >= ki_max)
		qdetail_ki = ki_max;
	else if (qdetail_ki <= ki_min)
		qdetail_ki = ki_min;
	if (qdetail_kd >= kd_max)
		qdetail_kd = kd_max;
	else if (qdetail_kd <= kd_min)
		qdetail_kd = kd_min;*/
		
	// kp =  kp+detail_kp;
	// ki  =  ki+detail_ki;
	// kd =  kd+detail_kd;

	PID[0]=kp+detail_kp;
	PID[1]=ki+detail_ki;
	PID[2]=kd+detail_kd;
	
	 if (PID[0] < 0)
	 	PID[0] = 0;
	//  if (PID[1] < 0)
	//  	PID[1] = 0;
	if (PID[2] < 0)
		PID[2] = 0;
	detail_kp = 0;
  	detail_ki  = 0;
  	detail_kd = 0;
	return PID;
}
 
///////////////////////////////区间映射函数///////////////////////////////////////////
float FuzzyPID::Quantization(float maximum,float minimum,float x)
{
	float qvalues= 6.0 *(x-minimum)/(maximum - minimum)-3;
	//float qvalues=6.0*()
	return qvalues;
	
	//qvalues[1] = 3.0 * ecerro / (maximum - minimum);
}
 
//////////////////////////////反区间映射函数////////////////////////////////////////////
float FuzzyPID::Inverse_quantization(float maximum, float minimum, float qvalues)
{
	float x = (maximum - minimum) *(qvalues + 3)/6 + minimum;
	return x;
}


PX4Tracking::PX4Tracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4Tracking::CmdLoopCallback, this); //周期为0.1s
  //订阅二维码相对飞机正前方位置
   ar_pose_sub_= nh_private_.subscribe("/ar_pose_marker", 1, &PX4Tracking::ArPoseCallback, this,ros::TransportHints().tcpNoDelay());
//订阅发布检测框位置的话题
  box_pose_sub_=nh_private_.subscribe("/yolo_pose",1, &PX4Tracking::BoxPoseCallback, this,ros::TransportHints().tcpNoDelay());
  //飞控得到的飞机的位置
  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4Tracking::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());
  //接收来自飞控的当前飞机的状态
  state_sub_ = nh_private_.subscribe("/mavros/state", 1, &PX4Tracking::Px4StateCallback,this,ros::TransportHints().tcpNoDelay());
   //接收来自飞控的当前飞机的pose
  pose_sub_ = nh_private_.subscribe("/mavros/global_position/local", 1, &PX4Tracking::Px4PoseCallback,this,ros::TransportHints().tcpNoDelay());
  // 【服务】修改系统模式
  set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  set_arm_client_= nh_private_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

}

PX4Tracking::~PX4Tracking() {
  //Destructor
}

/**
* @name       S_SETPOINT_VEL PX4Tracking::TRACKINGPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)

* @brief      pid控制程序,机体坐标系下控制无人机
*             
* @param[in]  &currentPos 当前飞机相对二维码的位置
*             
* @param[in]  &expectPos 期望位置 expectPos[0]:相对二维码前后方向距离；expectPos[1]:相对二维码左右方向距离；expectPos[2]:相对二维码上下方向距离
* @param[out] y,z的期望速度,以及yaw方向的期望速度，横向的roll
*
* @param[out] 
**/
Eigen::Vector4d PX4Tracking::TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos,Eigen::Vector4d &orientation)
{
 // Eigen::Vector3d s_PidOut;
	Eigen::Vector4d s_PidOut;
	Eigen::Vector3d xpid;
	Eigen::Vector3d ypid;
	Eigen::Vector3d zpid;

    xerro=currentPos[2] - expectPos[0];
	if (xerro>10)
	xerro = 10;
	xerrp_ppre=xerro_pre;
	xerro_c=xerro-xerro_pre;
	xsumE +=xerro;
	if(xsumE>=50)
	xsumE=50;
    if (xsumE<=-50)
	xsumE = -50;
    xerro_pre=xerro;

	xpid=fuzzypid.FuzzyPIDcontroller(e1_max,e1_min,ec1_max,ec1_min,kp1_max,kp1_min,xerro,xerro_c,ki1_max,ki1_min,kd1_max,kd1_min,xerro_pre,xerrp_ppre);
	/*前后方向的pid控制，输出机体坐标系下y方向的速度控制*/
	  cout<<"xerro:"<<xerro<<"  xsumE:"<<xsumE<<"  xerro_c:"<<xerro_c<<endl;
	  cout<<"xpid[0]:"<<xpid[0]<<" xpid[1]:"<<xpid[1]<<" xpid[2]:"<<xpid[2]<<endl;	
    s_PidOut[3] =xpid[0]*xerro+xpid[2]*xsumE;
	//+xpid[2]*xerro_c*100 
	//s_PidOut[0] =0.8*xerro+0.02*xsumE+0.04*xerro_c ;
	if ( s_PidOut[3]>2.5)
	s_PidOut[0]=2;
	if ( s_PidOut[3]<-2)
	s_PidOut[3]=-2;
    // cout<<"前后速度:"<<s_PidOut[0] <<endl;
   vx = s_PidOut[3];

	/*yaw方向的pid控制，输出yaw方向速度控制*/
	             /*此处用新的方法计算yaw偏航  加入了角速度*/
	desire_angleVel_=30*exp(fabs(orientation[3]))*pow(orientation[3],3);
	s_PidOut[1]=desire_angleVel_;


    //左右方向的pid控制，计算速度时暂时与s_PidY或s_PidZ系数相同
             // 模糊pid部分
    yerro=-currentPos[0] +expectPos[1];
	if (yerro>3)
	yerro= 3;
	if (yerro<-3)
	yerro=-3;
	ysumE+=yerro;
		if(ysumE>=2)
	   ysumE= 2;
	   if (ysumE<=-2)
	   ysumE = -2;
    yerrp_ppre=yerro_pre;
	yerro_c=yerro-yerro_pre;
	//  cout<<"yerro:"<<yerro<<"  yerro_c:"<<yerro_c<<endl;
    yerro_pre=yerro;
	//   cout<<"yerro: "<<yerro<<"  yerrp_ppre:"<<yerrp_ppre<<"  yerro_pre:"<<yerro_pre<<endl;
        // if (yerro_c>=1.0)
	    //      yerro_c=0.5;
      ypid=fuzzypid.FuzzyPIDcontroller(e2_max,e2_min,ec2_max,ec2_min,kp2_max,kp2_min,yerro,yerro_c,ki2_max,ki2_min,kd2_max,kd2_min,yerro_pre,yerrp_ppre);
	 	   cout<<"yerro:"<<yerro<<"  ysumE:"<<ysumE<<"  yerro_c:"<<yerro_c<<endl;
	       cout<<"ypid[0]:"<<ypid[0]<<" ypid[1]:"<<ypid[1]<<" ypid[2]:"<<ypid[2]<<endl;	
	   //s_PidOut[3] = 0.3*(ypid[0]*yerro+ ypid[1]*ysumE+ypid[2]*yerro_c);
		s_PidOut[0] = ypid[0]*yerro+ypid[1]*ysumE;
		//+0.02*ysumE+0.04*yerro_c;
		//s_PidOut[3] = s_PidY.p*yerro + s_PidY.d*yerro_c + s_PidY.i*ysumE;
	  if ( s_PidOut[0]>1)
	     s_PidOut[0]=1;
	 	else if ( s_PidOut[0]<-1)
	  s_PidOut[0]=-1;
	 


  /*输出s_PidOut[3]为飞机坐标系下Y轴也就是飞机左右方向的速度,此处为roll*/

		/*上下方向的pid控制，输出z方向的速度控制*/
	           /*currentPos[1]为二维码的Y轴方向 expectPos[2]为飞机的Z轴方向*/

    	s_PidItemZ.difference = expectPos[2] - currentPos[1];
	s_PidItemZ.intergral += s_PidItemZ.difference;
	if(s_PidItemZ.intergral >=50)		
		s_PidItemZ.intergral = 50;
	else if(s_PidItemZ.intergral <= -50) 
		s_PidItemZ.intergral = -50;
	s_PidItemZ.differential =  s_PidItemZ.difference  - s_PidItemZ.tempDiffer;
  s_PidItemZ.tempDiffer = s_PidItemZ.difference;
  /*s_PidOut[2]为飞机坐标系下Z轴也就是飞机上下方向的速度*/
	s_PidOut[2] = 0.3*(s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + 0.05*s_PidItemZ.intergral);
	//  cout<<"上下速度："<<s_PidOut[2]<<endl;
	//  cout<<"前进速度："<<s_PidOut[0]<<"  左右速度："<< s_PidOut[3]<<"  上下速度："<< s_PidOut[2]<<endl;
	/*返回的是Eigen::Vector3d数据类型是一个三维的向量s_PidOut=(s_PidOut[0],s_PidOut[1],s_PidOut[2]) */
	return s_PidOut;   // 在TrackingStateUpdate()被实例化为desire_vel_ 
	
}
/**
 * @brief 由句柄nh_创建的cmdloop_timer_ 调用PX4Tracking::CmdLoopCallback（）函数 周期为0.1S
 * @brief 每隔0.1s调用一次TrackingStateUpdate()函数
 **/
void PX4Tracking::CmdLoopCallback(const ros::TimerEvent& event)
{
  TrackingStateUpdate();
}


/**
* @name       void PX4Tracking::TrackingStateUpdate()
* @brief      状态机更新函数
*             
* @param[in]  无
*             
* @param[in]  无
* @param[out] 
*
* @param[out] 
**/
void PX4Tracking::TrackingStateUpdate()
{
	switch(TrackingState)   //tracking_quadrotor.h中枚举的TrackingState
	{
	case WAITING:
			if(px4_state_.mode != "OFFBOARD")//等待offboard模式
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			else
			{
				/*如果已经是板外模式，设置追踪状态为CHECKING（检查飞机状态）*/
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				TrackingState = CHECKING;
				cout << "CHECKING" <<endl;
			}
			break;

	case CHECKING:
		    /*如果有问题自动降落 并设置为等待模式重新传入位置 没有问题则设置为PREPARE模式，起飞到制定高度*/
			if(px4_pose_[0] == 0 && px4_pose_[1] == 0) 			//没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";  //由mavros_msgs::SetMode包  请求设置模式为自动着陆
				set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;	
			}
			else
			{
				TrackingState = PREPARE;
				cout << "PREPARE" <<endl;
			}
			break;

	case PREPARE:											//起飞到指定高度
			posxyz_target[0] = temp_pos_drone[0];
			posxyz_target[1] = temp_pos_drone[1];
			posxyz_target[2] = search_alt_;  //X Y上的位置信息不变，Z轴也就是高度上飞到search_alt_，在launch文件中search_alt_设为1.5m
			/*在高度小于一定的误差范围之内，切换为SEARCH搜索状态*/
			xsumE = 0; 
			ysumE = 0;//重置积分项
			if((px4_pose_[2]<=search_alt_+0.1) && (px4_pose_[2]>=search_alt_-0.1))
			{
				TrackingState = SEARCH;
				cout<<"开始搜索目标"<<endl;
			}
			OffboardControl_.send_pos_setpoint(posxyz_target, 0);					
			if(px4_state_.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}
			break;

	case SEARCH:
			if(detect_state == true )
			{
				TrackingState = TRACKING;
			  cout << "ar_TRACKING" <<endl;
			}	
			 else if(yolo_state == true)
			{
					TrackingState = TRACKING;
			  cout << "yolo_TRACKING" <<endl;
			}
			else//这里无人机通过旋转机头搜寻目标
			{  
				// search_target[0] = temp_pos_drone[0];
			    // search_target[1] = temp_pos_drone[1];
			    // search_target[2] = 2.5; 
				// OffboardControl_.send_pos_setpoint(search_target, 0);
				// desire_xyzVel_[0] = 0;  //机体x轴前后
				// desire_xyzVel_[1] = 0;   //机体z轴上下
				// desire_yawVel_    = 0.5;     //机体y轴yaw
				// desire_xyzVel_[2] = 0;  //机体y轴，此处为roll的速度 对应send_body_velyz_setpoint（）函数中的x
				// OffboardControl_.send_body_velyz_setpoint(desire_xyzVel_,desire_yawVel_); 
				OffboardControl_.send_pos_setpoint(posxyz_target, 0);
			}
			if(px4_state_.mode != "OFFBOARD")				//如果在SEARCH途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}
			break;

	case TRACKING:
		    /*search检测到目标，跳到TRACKING*/
			{
				// if(yolo_state == false)
				// {
				// 	TrackingState = SEARCH;
				// 	cout<<"从TRACKING进入搜索模式"<<endl;
				// 	break;
				// }
				/*如果检测到目标，进入pid追踪函数TrackingPidProcess（）*/
				if(detect_state == true )
				{
					printf("基于二维码计算速度\n");
					cout<<"yolo_state:"<<yolo_state<<"  detect_state:"<<detect_state<<endl;
					/**ar_pose为飞机相对于二维码的位置 desire_pose为飞机期望的位置
					 * ar_pose_在ArPoseCallback（）中被赋值，而ArPoseCallback（）函数为订阅器ar_pose_sub_的回调函数
					 * 过程为此cpp文件通过订阅"/ar_pose_marker"话题，调用回调函数ArPoseCallback（）
					 * 然后在ArPoseCallback（）为ar_pose_赋值，得到飞机相对于二维码的位置
					 * desire_pose_在launch文件中做了初始化，目的是飞机始终与二维码在同一轴且与二维码保持6.5m的距离**/
					 desire_vel_ = TrackingPidProcess(ar_pose_,desire_pose_,orientation); 

					//  if(xerro <= 0.1 && yerro<= 0.05)
				    //  {
					//     TrackingState = TRACKOVER;
					//     cout << "TRACKOVER" <<endl;
				    //    }

				}
              else if (yolo_state == true)
			      {
					cout<<"yolo_state:"<<yolo_state<<"  detect_state:"<<detect_state<<endl;
					  printf("基于YOLOv4计算速度\n");
                      desire_vel_ = TrackingPidProcess(yolo_pose_,yolo_desire_pose_,yolo_orientation); 
					//   yolo_state = false;
			      }
			  else
				{
					TrackingState = SEARCH;
				     cout<<"切换回搜索模式"<<endl;
				}

				if(px4_state_.mode != "OFFBOARD")			//如果在TRACKING中途中切换到onboard，则跳到WAITING
				{
					TrackingState = WAITING;
				}

				desire_xyzVel_[0] = desire_vel_[0];  //机体x轴前后
				desire_xyzVel_[1] = desire_vel_[2];   //机体z轴上下
				desire_yawVel_    = desire_vel_[1];     //机体y轴yaw
				desire_xyzVel_[2] = desire_vel_[3];  //机体y轴，此处为roll的速度 对应send_body_velyz_setpoint（）函数中的x
                /*通过将desire_yzVel_和desire_yawVel_传入到OffboardControl模块的send_body_velyz_setpoint进行控制*/
				OffboardControl_.send_body_velyz_setpoint(desire_xyzVel_,desire_yawVel_); 
			}

			break;
		case TRACKOVER:
			{
				cout<<"高度:"<<px4_odom_.pose.pose.position.z<<endl;
				if (px4_odom_.pose.pose.position.z>=1)
				{
				desire_xyzVel_[0] = 1.1*vx;  //机体x轴前后
				desire_xyzVel_[1] = -1;   //机体z轴上下
				desire_yawVel_     =0;     //yaw
				desire_xyzVel_[2] = 0;  //机体y轴，此处为roll的速度 对应send_body_velyz_setpoint（）函数中的x
				OffboardControl_.send_body_velyz_setpoint(desire_xyzVel_,desire_yawVel_); 
				}
				 else if (px4_odom_.pose.pose.position.z<1)
				 {
					       attitude_rate_sp[0] = 0;
                           attitude_rate_sp[1] = 0;
                           attitude_rate_sp[2] = 0;
                           thrust_sp = 0;
                        OffboardControl_.send_attitude_rate_setpoint(attitude_rate_sp, thrust_sp); 
						cout<<"0油门"<<endl;
						 yolo_state = false;
				         detect_state = false;
				 }
			    break;
				// //TrackingState = WAITING;
			}

			break;

		default:
			cout << "error" <<endl;
	}	

}

/*接收降落板相对飞机的位置以及偏航角*/
void PX4Tracking::ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	detect_state = false;
	for(auto &item : msg->markers)
	{
		if(item.id == markers_id_)
		{
			detect_state = true;
      ar_pose_[0] = item.pose.pose.position.x;
      ar_pose_[1] = item.pose.pose.position.y;
      ar_pose_[2] = item.pose.pose.position.z;
	  orientation[0]=item.pose.pose.orientation.x;
	  orientation[1]=item.pose.pose.orientation.y;
	  orientation[2]=item.pose.pose.orientation.z;
	  orientation[3]=item.pose.pose.orientation.w;
		}
	}
}

void PX4Tracking::BoxPoseCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
			yolo_state = true;
      		yolo_pose_[0] = msg->point.x;
     		yolo_pose_[1] = msg->point.y;
      		yolo_pose_[2] = msg->point.z;
	  		yolo_orientation[0]=0;
	  		yolo_orientation[1]=0;
	  		yolo_orientation[2]=0;
	  		yolo_orientation[3]=0;
}


/*接收来自飞控的当前飞机位置*/                  
void PX4Tracking::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    px4_pose_ = pos_drone_fcu_enu;
}
/*接收来自飞控的当前飞机状态*/
void PX4Tracking::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	px4_state_ = *msg;
}
/*接收来自飞控的当前飞机POSE*/
void PX4Tracking::Px4PoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	px4_odom_ = *msg;
	
}

/*初始化*/
/*此处做很多参数在launch文件中也做了初始化*/
void PX4Tracking::Initialize()
{
  //读取offboard模式下飞机的搜索高度
  nh_private_.param<float>("search_alt_", search_alt_, 2);

  nh_private_.param<float>("markers_id_", markers_id_, 4.0);

  nh_private_.param<float>("PidY_p", s_PidY.p, 0.6);
  nh_private_.param<float>("PidY_d", s_PidY.d, 0.01);
  nh_private_.param<float>("PidY_i", s_PidY.i, 0);
  nh_private_.param<float>("PidZ_p", s_PidZ.p, 0.6);
  nh_private_.param<float>("PidZ_d", s_PidZ.d, 0.01);
  nh_private_.param<float>("PidZ_i", s_PidZ.i, 0);
  nh_private_.param<float>("PidYaw_p", s_PidYaw.p, 0.4);
  nh_private_.param<float>("PidYaw_d", s_PidYaw.d, 0.01);
  nh_private_.param<float>("PidYaw_i", s_PidYaw.i, 0);

  //期望的飞机相对二维码的位置
	float desire_pose_x,desire_pose_y,desire_pose_z;
  nh_private_.param<float>("desire_pose_x", desire_pose_x, 6.5);
  nh_private_.param<float>("desire_pose_y", desire_pose_y, 0);
  nh_private_.param<float>("desire_pose_z", desire_pose_z, 0);
  desire_pose_[0] = desire_pose_x;
  desire_pose_[1] = desire_pose_y;
  desire_pose_[2] = desire_pose_z;

    yolo_desire_pose_[0] = desire_pose_x+3;
    yolo_desire_pose_[1] = desire_pose_y;
    yolo_desire_pose_[2] = desire_pose_z;

  detect_state = false;
  yolo_state = false;
  desire_vel_[0] = 0;
  desire_vel_[1] = 0;
  desire_vel_[2] = 0;
	desire_xyzVel_[0]  = 0;
	desire_xyzVel_[1]  = 0;
	desire_xyzVel_[2]  = 0;

  s_PidItemY.tempDiffer = 0;
  s_PidItemYaw.tempDiffer = 0;
  s_PidItemZ.tempDiffer = 0;
  s_PidItemY.intergral = 0;
  s_PidItemYaw.intergral = 0;
  s_PidItemZ.intergral = 0;
	// cout << "search_alt_ = " << search_alt_ << endl;
	// cout << "markers_id_ = " << markers_id_ << endl;
	// cout << "desire_pose_x = " << desire_pose_[0] << endl;
	// cout << "desire_pose_y = " << desire_pose_[1] << endl;
	// cout << "desire_pose_z = " << desire_pose_[2] << endl;
   cout<<"等待切换OFFBOARD模式"<<endl;
}
int main(int argc, char** argv) {
  ros::init(argc,argv,"tracking_quadrotor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  PX4Tracking PX4Tracking(nh, nh_private);
  ros::spin();
  return 0;
}
