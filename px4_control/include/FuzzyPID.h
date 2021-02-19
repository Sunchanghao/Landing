#ifndef FuzzyPID_H
#define FuzzyPID_H
#include <Eigen/Eigen>  //Eigen库支持线性代数，矩阵运算等
#include <Eigen/Dense>  
#include <Eigen/Geometry>  //包括2D图像的平移旋转缩放 3d图像的旋转（四元数、轴角）
#include <Eigen/Eigenvalues>  //特征值特征向量
using namespace Eigen;
class FuzzyPID
{
public:
	FuzzyPID();
	~FuzzyPID();
	void Get_grad_membership(float erro, float erro_c);
	float Quantization(float maximum, float minimum, float x);
	float Inverse_quantization(float maximum, float minimum, float qvalues);
	void GetSumGrad();
	void GetOUT();
	Eigen::Vector3d FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float erro, float erro_c, float ki_max, float ki_min,float kd_max, float kd_min,float erro_pre, float errp_ppre);
	const int  num_area = 8; //划分区域个数
	//float e_max;  //误差做大值
	//float e_min;  //误差最小值
	//float ec_max;  //误差变化最大值
	//float ec_min;  //误差变化最小值
	//float kp_max, kp_min;
	float e_membership_values[7] = {-3,-2,-1,0,1,2,3}; //输入e的隶属值
	float ec_membership_values[7] = { -3,-2,-1,0,1,2,3 };//输入de/dt的隶属值
	float kp_menbership_values[7] = { -3,-2,-1,0,1,2,3 };//输出增量kp的隶属值
	float ki_menbership_values[7] = { -3,-2,-1,0,1,2,3 }; //输出增量ki的隶属值
	float kd_menbership_values[7] = { -3,-2,-1,0,1,2,3 };  //输出增量kd的隶属值
	float fuzzyoutput_menbership_values[7] = { -3,-2,-1,0,1,2,3 };
 
	//int menbership_values[7] = {-3,-};
	float kp;                       //PID参数kp
	float ki;                       //PID参数ki
	float kd;                       //PID参数kd
	float qdetail_kp;               //增量kp对应论域中的值
	float qdetail_ki;               //增量ki对应论域中的值
	float qdetail_kd;               //增量kd对应论域中的值
	float qfuzzy_output;  
	float detail_kp;                //输出增量kp
	float detail_ki;                //输出增量ki
	float detail_kd;                //输出增量kd
	float fuzzy_output;
	float qerro;                    //输入e对应论域中的值
	float qerro_c;                  //输入de/dt对应论域中的值
	float errosum;                  
	float e_gradmembership[2];      //输入e的隶属度
	float ec_gradmembership[2];     //输入de/dt的隶属度
	int e_grad_index[2];            //输入e隶属度在规则表的索引
	int ec_grad_index[2];           //输入de/dt隶属度在规则表的索引
	float gradSums[7] = {0,0,0,0,0,0,0};
	float KpgradSums[7] = { 0,0,0,0,0,0,0 };   //输出增量kp总的隶属度
	float KigradSums[7] = { 0,0,0,0,0,0,0 };   //输出增量ki总的隶属度
	float KdgradSums[7] = { 0,0,0,0,0,0,0 };   //输出增量kd总的隶属度
	int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3; //论域隶属值
 
	int  Kp_rule_list[7][7] ={ {PB,PB,PM,PM,PS,ZO,ZO},     //kp规则表
								{PB,PB,PM,PS,PS,ZO,NS},
								{PM,PM,PM,PS,ZO,NS,NS},
								{PM,PM,PS,ZO,NS,NM,NM},
								{PS,PS,ZO,NS,NS,NM,NM},
								{PS,ZO,NS,NM,NM,NM,NB},
		                        {ZO,ZO,NM,NM,NM,NB,NB} };




 //{                             {PB,PB,PM,PM,PM,PM,PM},     //kp规则表
								//{PB,PB,PM,PM,PM,PM,PB},
								//{PM,PM,PM,PM,PM,PB,PB},
								//{PM,PM,PM,PM,PB,PM,PM},
								//{PB,PB,PM,PB,PB,PM,PM},
								//{PB,PM,PM,PM,PM,PM,PS},
		                                               // {PM,PM,NM,PM,PM,PS,PS} };
 
	int  Ki_rule_list[7][7] = { {NB,NB,NM,NM,NS,ZO,ZO},     //ki规则表
								{NB,NB,NM,NS,NS,ZO,ZO},
								{NB,NM,NS,NS,ZO,PS,PS},
								{NM,NM,NS,ZO,PS,PM,PM},
								{NM,NS,ZO,PS,PS,PM,PB},
								{ZO,ZO,PS,PS,PM,PB,PB},
								{ZO,ZO,PS,PM,PM,PB,PB} };
 
	int  Kd_rule_list[7][7] = { {PS,NS,NB,NB,NB,NM,PS},    //kd规则表
								{PS,NS,NB,NM,NM,NS,ZO},
								{ZO,NS,NM,NM,NS,NS,ZO},
								{ZO,NS,NS,NS,NS,NS,ZO},
								{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
								{PB,NS,PS,PS,PS,PS,PB},
								{PB,PM,PM,PM,PS,PS,PB} };
 
	int  Fuzzy_rule_list[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},  
								   {PB,PB,PB,PM,PM,ZO,ZO},
								   {PB,PM,PM,PS,ZO,NS,NM},
								   {PM,PM,PS,ZO,NS,NM,NM},
								   {PS,PS,ZO,NM,NM,NM,NB},
								   {ZO,ZO,ZO,NM,NB,NB,NB},
								   {ZO,NS,NB,NB,NB,NB,NB}};
 
 
//private:
 
};
#endif
