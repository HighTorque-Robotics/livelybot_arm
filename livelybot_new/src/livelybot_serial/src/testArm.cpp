#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
// #include "../include/eigen/Eigen/Dense"
//#include "eigen/Eigen/Dense"
#include <math.h>
#include <iostream>
#include <thread>
#include <condition_variable>
#include <sensor_msgs/Joy.h>
//#include "6Dof_Modle.h"
#include "6Dof_Modle_dyna.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using namespace std;
using namespace Eigen;


/*遥控器变量*/
float ctrl_z, ctrl_y, ctrl_x;



void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //ROS_INFO("Received joy message!");
    if (joy->buttons[0] == 1) {  // 假设第一个按钮被按下
        ROS_INFO("Button A pressed!");
    }
    if (joy->buttons[1] == 1) {  // 假设第一个按钮被按下
        ROS_INFO("Button B pressed!");
    }
    if (joy->buttons[3] == 1) {  // 假设第一个按钮被按下
        ROS_INFO("Button X pressed!");
    }
    if (joy->buttons[4] == 1) {  // 假设第一个按钮被按下
        ROS_INFO("Button Y pressed!");
    }

    ctrl_z = joy->axes[1];
    ctrl_y = (joy->axes[3]);
    ctrl_x = joy->axes[4];

}
void ptArry(float date[],int len)
{
    for (size_t i = 0; i < len; i++)
    {
        cout<<date[i]<<", ";
        /* code */
    }
    cout<<endl;
}
//过渡
void now2start(float nowq[6], float startq[6], float q1Trace[1000], float q2Trace[1000], float q3Trace[1000], float q4Trace[1000], float q5Trace[1000], float q6Trace[1000])
{
  float dltq[6]={0}, stepq[6]={0};
  for (size_t i = 0; i < 6; i++)
  {
    dltq[i] = startq[i] - nowq[i];
    stepq[i] = dltq[i]/1000.0;
  }
  // cout<<"nowq1； "<<nowq[0]<<endl;
  // cout<<"startq "<<startq[0]<<endl;
  // cout<<"dltq "<<dltq[0]<<endl;
  // cout<<"stepq "<<stepq[0]<<endl;
  

  for (size_t i = 0; i < 1000; i++)
  {
    q1Trace[i] = nowq[0]+i*stepq[0];
    q2Trace[i] = nowq[1]+i*stepq[1];
    q3Trace[i] = nowq[2]+i*stepq[2];
    q4Trace[i] = nowq[3]+i*stepq[3];
    q5Trace[i] = nowq[4]+i*stepq[4];
    q6Trace[i] = nowq[5]+i*stepq[5];
    //cout<<"q1: "<<q1Trace[i]<<endl;
    /* code */
  }
  
  



}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "testArm");
  ros::NodeHandle n;
  ros::Rate r(1000);
  lively_robot::robot rb;
  ROS_INFO("\033[1;32mSTART\033[0m");
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

  int cont = 0;
  size_t n_motors = 6;
  motor_back_t motor;

/*机械臂模型相关变量*/
float X, Y, Z, rotx, roty, rotz;
float dX, dY, dZ, drotx, droty, drotz;
float ddX, ddY, ddZ, ddrotx, ddroty, ddrotz;
float joint[6] = {0.0, 0.0, 0.0, 0.5236, 0.5236, 0.5236}, djoint[6]={0}, ddjoint[6]={0}, efAcc[6];
float realtime=0;

Matrix3f  R, R01, R12, R23, R34, R45, R56, R06, R02, R03, SP;
Matrix3f jacob3x3Left, jacob3x3Right, downJacob3x3Left, downJacob3x3Right;
Vector3f P01, P12, P23, P34, P45, P56, P06, Fq, F,P0, P1, P2, P3, P4, P5, P6;
Vector3f z00, z01, z02, z03, z04, z05, z06;
VectorXf endPos(6), endVel(6), endAcc(6), q(6), dq(6), ddq(6), torque(6);
MatrixXf jacon0(6,6),jacon0te(6,6);
VectorXf testVec;


/*纯运动学控制*/
bool ik=false;
int ikCount=0,ikTime=0;
float ikCurrentTime=0.0;

/*拖动示教相关变量*/
bool teach=false;
int teachCount=0,teachTime=0;
float teachCurrentTime=0.0;

/*动力学控制算法相关变量*/
bool dynamics=false;
int dynamicsCount=0,dynamicsTime=0;
float dynamicsCurrentTime=0.0;

/*定点阻抗控制相关变量*/
bool impedence=false;
int impedenceCount=0,impedenceTime=0;
float impedenceCurrentTime=0.0;

 //机械臂的构型都是不变的，这里只是用了两种方式表示，修改（改进）的DH和标准的DH 
//我比较喜欢用标准的DH写动力学模型，但是论文中的逆运动学是用的修改的DH，所以
//逆运动学用的改进的DH，逆动力学用的标准的DH。
//其实差别不大，仔细观察即可发现规律。
//修改的DH参数
  float a[6] = {0, 0, 0.210, 0.180, 0, 0};
  float alpha[6] = {0, 1.5708, 0.0, 0.0, 1.5708, -1.5708};
  float d[6] = {0.094, 0.0, 0.0, -0.063, 0.060, 0.063};
//标准的DH参数
  float a_sd[6] = {0, 0.210, 0.180, 0, 0, 0};
  float alpha_sd[6] = {1.5708, 0.0, 0.0, 1.5708, -1.5708, 0};
  float d_sd[6] = {0.094, 0.0, 0.0, -0.063, 0.060, 0.063};

  
cout<<"\n======》》》》晴晴的机械臂《《《《《《======\n"<<endl;

UR6DOFARM arm6dof;


  
//逆运动学，逆动力学计算例子
///末端轨迹
realtime =1.0;
X = 0.33 + 0.05*sin(realtime);
Y = 0.03;
Z = 0.1;
rotx = 0;
roty = 1.5708;
rotz = 0;

dX = 0.05*cos(realtime);
dY = 0.0;
dZ = 0.0;
drotx = 0;
droty = 0.0;
drotz = 0;
endVel<< dX, dY, dZ, drotx, droty,drotz;

ddX = -0.05*sin(realtime); 
ddY = -0.00;
ddZ = 0.0;
ddrotx = 0;
ddroty = -0.0;
ddrotz = 0;
endAcc<<ddX, ddY, ddZ, ddrotx, ddroty, ddrotz;
efAcc[0] = ddX; efAcc[1] = ddY; efAcc[2] = ddZ;
efAcc[3] = ddrotx; efAcc[4] = ddroty; efAcc[5] = ddrotz;

arm6dof.SetDH(a, d, alpha);
cout<<"修改的DH参数:=\n"<<endl;
cout<<"a: "; ptArry(a,6);
cout<<"d: "; ptArry(d,6);
cout<<"alpha: "; ptArry(alpha,6);


cout<<"标准的DH参数:=\n"<<endl;
cout<<"a: "; ptArry(a_sd,6);
cout<<"d: "; ptArry(d_sd,6);
cout<<"alpha: "; ptArry(alpha_sd,6);

cout<<"\n验证逆运动学，使用修改的DH："<<endl;
cout<<"q:\n"<<endl;
arm6dof.Ik_6Dof(X, Y, Z, rotx, roty, rotz, joint);
ptArry(joint, 6);

cout<<"\n验证逆速度,使用修改的DH："<<endl;
jacon0 = arm6dof.Get_Jacob0(a, alpha, d, joint);
dq = jacon0.inverse()*endVel;
cout<<"dq\n"<<dq<<endl;

for (size_t i = 0; i < 6; i++) djoint[i] = dq(i);

cout<<"\n验证逆加速度，使用标准的DH："<<endl;
testVec = arm6dof.Get_Jdot(a_sd, alpha_sd, d_sd, joint, djoint, efAcc);
endAcc = endAcc - testVec;
ddq = jacon0.inverse()*(endAcc);
cout<<"ddq\n"<<ddq<<endl;

//动力学惯性参数在‘Get-Tor’里面修改
for (size_t i = 0; i < 6; i++) ddjoint[i] = ddq(i);
cout<<"\n验证逆动力学，使用标准DH："<<endl;
torque = arm6dof.Get_Tor(a_sd, alpha_sd, d_sd, joint, djoint, ddjoint);
cout<<torque<<endl;
cout<<"\n>>>>>>>>>>>>>>>>>>计算结束！！\n"<<endl;


/*
更改相应的标识为“true”即可运行相应的功能。
注意！设置的的期望轨迹不要超过工作空间。
注意！“过渡到期望位置”的期望位置要与实际期望位置相同，不然会产生波动，尤其是pid较大时。
注意！为了不让电机一直运行，机械臂在运行时间结束后，会让电机力矩为0，注意保护！
*/

/*纯运动学控制，输入电机速度位置*/
ikTime = 11000; // 11秒
ik = false;
if (ik)
{
  float rate = 2;
  float inputMotor[6] = {0}, inputMotord[6] = {0};

  /*********过渡到期望位置*********************/
  float nowMotor[6] = {0}, startMotor[6] = {0};
  float m1Trace[1000] = {0}, m2Trace[1000] = {0}, m3Trace[1000] = {0}, m4Trace[1000] = {0}, m5Trace[1000] = {0}, m6Trace[1000] = {0};
  // 获取电机当前状态
  for (size_t i = 0; i < 30; i++)
  {

    rb.Motors[5]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[4]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[3]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[2]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[1]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[0]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.motor_send();
    r.sleep();
    /* code */
  }

  motor = *rb.Motors[0]->get_current_motor_state();
  nowMotor[0] = motor.position;
  motor = *rb.Motors[1]->get_current_motor_state();
  nowMotor[1] = motor.position;
  motor = *rb.Motors[2]->get_current_motor_state();
  nowMotor[2] = motor.position;
  motor = *rb.Motors[3]->get_current_motor_state();
  nowMotor[3] = motor.position;
  motor = *rb.Motors[4]->get_current_motor_state();
  nowMotor[4] = motor.position;
  motor = *rb.Motors[5]->get_current_motor_state();
  nowMotor[5] = motor.position;
  // 储存的拖动轨迹中，0元素是错误值，所以从10号开始
  //这里的坐标需要和被跟踪轨迹的初始位置坐标相同
  X = 0.33 + 0.05 * sin(0.0);
  Y = 0.03;
  Z = 0.12;
  rotx = 0;
  roty = 1.5708;
  rotz = 0;
  arm6dof.Ik_6Dof(X, Y, Z, rotx, roty, rotz, joint); // 逆解关节位置
  startMotor[0] = joint[0];
  startMotor[1] = -joint[1];
  startMotor[2] = joint[2];
  startMotor[3] = -joint[3];
  startMotor[4] = joint[4];
  startMotor[5] = joint[5];

  now2start(nowMotor, startMotor, m1Trace, m2Trace, m3Trace, m4Trace, m5Trace, m6Trace);
  cout << "》》》正在前往目的地！！\n"
       << endl;
  for (size_t i = 0; i < 1000; i++)
  {
    // cout<<"q:"<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<endl;
    rb.Motors[0]->fresh_cmd(m1Trace[i], 0.0, 0, 15.0, 15); // q1
    rb.Motors[1]->fresh_cmd(m2Trace[i], 0.0, 0, 15.0, 15); // q2
    rb.Motors[2]->fresh_cmd(m3Trace[i], 0.0, 0, 15.0, 15); // q3
    rb.Motors[3]->fresh_cmd(m4Trace[i], 0.0, 0, 15.0, 7);  // q4
    rb.Motors[4]->fresh_cmd(m5Trace[i], 0.0, 0, 15.0, 7);  // q5
    rb.Motors[5]->fresh_cmd(m6Trace[i], 0.0, 0, 15.0, 7);  // q6
    rb.motor_send();
    r.sleep();
  }

  /*********e n d*********************/

  for (size_t i = 0; i < ikTime; i++)
  {
    // 1，计算期望轨迹
    ikCurrentTime = i / 1000.0;
    rate = 3;
    // 1,计算期望轨迹
    X = 0.33 + 0.05 * sin(rate * ikCurrentTime);
    Y = 0.03 + 0.05 * sin(1 * ikCurrentTime);
    Z = 0.12;
    rotx = 0;
    roty = 1.5708;
    rotz = 0;

    dX = 0.05 * rate * cos(rate * ikCurrentTime);
    dY = 0.05 * 1 * cos(1 * ikCurrentTime);
    dZ = 0.0;
    drotx = 0;
    droty = 0.0;
    drotz = 0;
    endVel << dX, dY, dZ, drotx, droty, drotz;

    ddX = -0.05 * rate * rate * sin(rate * ikCurrentTime);
    ddY = -0.05 * 1 * 1 * sin(1 * ikCurrentTime);
    ddZ = 0.0;
    ddrotx = 0;
    ddroty = -0.0;
    ddrotz = 0;
    endAcc << ddX, ddY, ddZ, ddrotx, ddroty, ddrotz;
    efAcc[0] = ddX;
    efAcc[1] = ddY;
    efAcc[2] = ddZ;
    efAcc[3] = ddrotx;
    efAcc[4] = ddroty;
    efAcc[5] = ddrotz;

    // 2，逆运动学
    arm6dof.Ik_6Dof(X, Y, Z, rotx, roty, rotz, joint); // 逆解关节位置
    inputMotor[0] = joint[0];
    inputMotor[1] = -joint[1];
    inputMotor[2] = joint[2];
    inputMotor[3] = -joint[3];
    inputMotor[4] = joint[4];
    inputMotor[5] = joint[5];

    jacon0 = arm6dof.Get_Jacob0(a, alpha, d, joint);
    dq = jacon0.inverse() * endVel; // 逆解关节速度mo
    inputMotord[0] = dq(0);
    inputMotord[1] = -dq(1);
    inputMotord[2] = dq(2);
    inputMotord[3] = -dq(3);
    inputMotord[4] = dq(4);
    inputMotord[5] = dq(5);

    // 3，输入电机
    rb.Motors[0]->fresh_cmd(inputMotor[0], inputMotord[0], 0, 15.0, 15); // q1
    rb.Motors[1]->fresh_cmd(inputMotor[1], inputMotord[1], 0, 15.0, 15); // q2
    rb.Motors[2]->fresh_cmd(inputMotor[2], inputMotord[2], 0, 15.0, 15); // q3
    rb.Motors[3]->fresh_cmd(inputMotor[3], inputMotord[3], 0, 8.0, 4);   // q4
    rb.Motors[4]->fresh_cmd(inputMotor[4], inputMotord[4], 0, 8.0, 4);   // q5
    rb.Motors[5]->fresh_cmd(inputMotor[5], inputMotord[5], 0, 8.0, 4);   // q6
    rb.motor_send();

    r.sleep();
    /* code */
  }
//让电机失去力矩
  rb.Motors[5]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[4]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[3]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[2]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[1]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[0]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.motor_send();
  cout << "结束！" << endl;
}

/*重力补偿和拖动示教*/
teachTime = 11000; // 11秒
teach = false;
if (teach)
{
  cout << ">>>>>>>>>拖动示教中>>>>>>>>>>>>" << endl;
  float motor1Save[teachTime] = {0}, motor2Save[teachTime] = {0}, motor3Save[teachTime] = {0};
  float motor4Save[teachTime] = {0}, motor5Save[teachTime] = {0}, motor6Save[teachTime] = {0};
  float motord1Save[teachTime] = {0}, motord2Save[teachTime] = {0}, motord3Save[teachTime] = {0};
  float motord4Save[teachTime] = {0}, motord5Save[teachTime] = {0}, motord6Save[teachTime] = {0};
  float joint[6] = {0}, jointd[6] = {0}, jointdd[6] = {0};
  float nowMotor[6] = {0}, startMotor[6] = {0};
  float m1Trace[1000] = {0}, m2Trace[1000] = {0}, m3Trace[1000] = {0}, m4Trace[1000] = {0}, m5Trace[1000] = {0}, m6Trace[1000] = {0};
  // 获取电机当前状态
  for (size_t i = 0; i < 30; i++)
  {
    rb.Motors[5]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[4]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[3]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[2]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[1]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[0]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.motor_send();
    motor = *rb.Motors[0]->get_current_motor_state();
    motor1Save[0] = motor.position;
    motord1Save[0] = motor.velocity; // q6 and dq6
    // ROS_INFO_STREAM("ID: " << 3 << " Pos: " << JointState_B[2] );
    motor = *rb.Motors[1]->get_current_motor_state();
    motor2Save[0] = motor.position;
    motord2Save[0] = motor.velocity; // q5 and dq5
    // //ROS_INFO_STREAM("ID: " << 2 << " Pos: " << JointState_B[1] << " torque: " << motor.torque);
    motor = *rb.Motors[2]->get_current_motor_state();
    motor3Save[0] = motor.position;
    motord3Save[0] = motor.velocity; // q4 and dq4
    motor = *rb.Motors[3]->get_current_motor_state();
    motor4Save[0] = motor.position;
    motord4Save[0] = motor.velocity; // q3 and dq3
    // ROS_INFO_STREAM("ID: " << 3 << " Pos: " << JointState_B[2] );
    motor = *rb.Motors[4]->get_current_motor_state();
    motor5Save[0] = motor.position;
    motord5Save[0] = motor.velocity; // q2 and dq2
    // //ROS_INFO_STREAM("ID: " << 2 << " Pos: " << JointState_B[1] << " torque: " << motor.torque);
    motor = *rb.Motors[5]->get_current_motor_state();
    motor6Save[0] = motor.position;
    motord6Save[0] = motor.velocity; // q1 and dq1
    r.sleep();
    /* code */
  }

  //*1，拖 动
  teachCount = 0;
  while (teachCount <= teachTime)
  {
    // a，获取电机的实时位置

    motor = *rb.Motors[0]->get_current_motor_state();
    motor1Save[teachCount] = motor.position;
    motord1Save[teachCount] = motor.velocity; // q6 and dq6
    // ROS_INFO_STREAM("ID: " << 3 << " Pos: " << JointState_B[2] );
    motor = *rb.Motors[1]->get_current_motor_state();
    motor2Save[teachCount] = motor.position;
    motord2Save[teachCount] = motor.velocity; // q5 and dq5
    // //ROS_INFO_STREAM("ID: " << 2 << " Pos: " << JointState_B[1] << " torque: " << motor.torque);
    motor = *rb.Motors[2]->get_current_motor_state();
    motor3Save[teachCount] = motor.position;
    motord3Save[teachCount] = motor.velocity; // q4 and dq4
    motor = *rb.Motors[3]->get_current_motor_state();
    motor4Save[teachCount] = motor.position;
    motord4Save[teachCount] = motor.velocity; // q3 and dq3
    // ROS_INFO_STREAM("ID: " << 3 << " Pos: " << JointState_B[2] );
    motor = *rb.Motors[4]->get_current_motor_state();
    motor5Save[teachCount] = motor.position;
    motord5Save[teachCount] = motor.velocity; // q2 and dq2
    // //ROS_INFO_STREAM("ID: " << 2 << " Pos: " << JointState_B[1] << " torque: " << motor.torque);
    motor = *rb.Motors[5]->get_current_motor_state();
    motor6Save[teachCount] = motor.position;
    motord6Save[teachCount] = motor.velocity; // q1 and dq1

    // 实际电机的转向和模型中关节的转向有差异，这里是电机2和电机4的方向与模型关节的方向相反，所以需要加符号。
    joint[0] = motor1Save[teachCount];
    joint[1] = -motor2Save[teachCount];
    joint[2] = motor3Save[teachCount];
    joint[3] = -motor4Save[teachCount];
    joint[4] = motor5Save[teachCount];
    joint[5] = motor6Save[teachCount];
    jointd[0] = motord1Save[teachCount];
    jointd[1] = -motord2Save[teachCount];
    jointd[2] = motord3Save[teachCount];
    jointd[3] = -motord4Save[teachCount];
    jointd[4] = motord5Save[teachCount];
    jointd[5] = motord6Save[teachCount];

    // cout<<"q123:"<<joint[0]<<", "<<joint[1]<<", "<<joint[2]<<", "<<joint[3]<<", "<<joint[4]<<", "<<joint[5]<<endl;

    // b，计算补偿力矩
    // 其输入的是关节的位置速度和加速度
    torque = arm6dof.Get_Tor(a_sd, alpha_sd, d_sd, joint, jointd, jointdd);
    // cout<<"q123:"<<torque[0]<<", "<<torque[1]<<", "<<torque[2]<<", "<<torque[3]<<", "<<torque[4]<<", "<<torque[5]<<endl;

    // c，输入电机力矩
    rb.Motors[0]->fresh_cmd(0.0, 0.0, torque[0], 0.0, 0.0);
    rb.Motors[1]->fresh_cmd(0.0, 0.0, -1 * torque[1], 0.0, 0.0);
    rb.Motors[2]->fresh_cmd(0.0, 0.0, 1 * torque[2], 0.0, 0.0);
    rb.Motors[3]->fresh_cmd(0.0, 0.0, -1 * torque[3], 0.0, 0.0);
    rb.Motors[4]->fresh_cmd(0.0, 0.0, 1 * torque[4], 0.0, 0.0);
    rb.Motors[5]->fresh_cmd(0.0, 0.0, 1 * torque[5], 0.0, 0.0);
    rb.motor_send();

    teachCount++;
    r.sleep();
    // ROS_INFO("ct: %d",teachCount);
  }
  // cout<<"q:"<<motor1Save[6]<<", "<<motor2Save[6]<<", "<<motor3Save[6]<<", "<<motor4Save[6]<<", "<<motor5Save[6]<<", "<<motor6Save[6]<<endl;

  // 获取电机当前状态
  for (size_t i = 0; i < 30; i++)
  {
    rb.Motors[5]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[4]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[3]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[2]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[1]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[0]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.motor_send();
    r.sleep();
    /* code */
  }
  motor = *rb.Motors[0]->get_current_motor_state();
  nowMotor[0] = motor.position;
  motor = *rb.Motors[1]->get_current_motor_state();
  nowMotor[1] = motor.position;
  motor = *rb.Motors[2]->get_current_motor_state();
  nowMotor[2] = motor.position;
  motor = *rb.Motors[3]->get_current_motor_state();
  nowMotor[3] = motor.position;
  motor = *rb.Motors[4]->get_current_motor_state();
  nowMotor[4] = motor.position;
  motor = *rb.Motors[5]->get_current_motor_state();
  nowMotor[5] = motor.position;
  // 储存的拖动轨迹中，0元素是错误值，所以从10号开始
  startMotor[0] = motor1Save[10];
  startMotor[1] = motor2Save[10];
  startMotor[2] = motor3Save[10];
  startMotor[3] = motor4Save[10];
  startMotor[4] = motor5Save[10];
  startMotor[5] = motor6Save[10];

  now2start(nowMotor, startMotor, m1Trace, m2Trace, m3Trace, m4Trace, m5Trace, m6Trace);
  cout << "》》》正在前往目的地！！\n"
       << endl;
  for (size_t i = 0; i < 1000; i++)
  {
    // cout<<"q:"<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<endl;
    rb.Motors[0]->fresh_cmd(m1Trace[i], 0.0, 0, 15.0, 15); // q1
    rb.Motors[1]->fresh_cmd(m2Trace[i], 0.0, 0, 15.0, 15); // q2
    rb.Motors[2]->fresh_cmd(m3Trace[i], 0.0, 0, 15.0, 15); // q3
    rb.Motors[3]->fresh_cmd(m4Trace[i], 0.0, 0, 15.0, 7);  // q4
    rb.Motors[4]->fresh_cmd(m5Trace[i], 0.0, 0, 15.0, 7);  // q5
    rb.Motors[5]->fresh_cmd(m6Trace[i], 0.0, 0, 15.0, 7);  // q6
    rb.motor_send();
    r.sleep();
  }
  cout << "》》》开始重播轨迹！！\n"
       << endl;
  for (size_t i = 10; i < teachTime; i++)
  {
    // cout<<"q:"<<motor1Save[i]<<", "<<motor2Save[i]<<", "<<motor3Save[i]<<", "<<motor4Save[i]<<", "<<motor5Save[i]<<", "<<motor6Save[i]<<endl;
    rb.Motors[0]->fresh_cmd(motor1Save[i], motord1Save[i], 0, 15.0, 15); // q1
    rb.Motors[1]->fresh_cmd(motor2Save[i], motord2Save[i], 0, 15.0, 15); // q2
    rb.Motors[2]->fresh_cmd(motor3Save[i], motord1Save[i], 0, 15.0, 15); // q3
    rb.Motors[3]->fresh_cmd(motor4Save[i], motord4Save[i], 0, 8.0, 4);   // q4
    rb.Motors[4]->fresh_cmd(motor5Save[i], motord5Save[i], 0, 8.0, 4);   // q5
    rb.Motors[5]->fresh_cmd(motor6Save[i], motord6Save[i], 0, 8.0, 4);   // q6
    rb.motor_send();
    r.sleep();
    /* code */
  }
  // 失能电机
  rb.Motors[0]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[1]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[2]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[3]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[4]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[5]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.motor_send();

  cout << "\n拖动结束！！\n"
       << endl;

  teach = false;
} /**结束拖动示教***/


/*动力学控制，计算力矩法*/
dynamicsTime = 12000;
dynamics = false;
if (dynamics)
{
  cout << "》》》动力学控制，轨迹跟踪！" << endl;
  float motor1, motor2, motor3, motor4, motor5, motor6, motorTor[6]; 
  float motor1d, motor2d, motor3d, motor4d, motor5d, motor6d; 
  float motorRecv[6],motordRecv[6], jointRecv[6], jointdRecv[6];
  float Kp[6], Kd[6], e[6] ,de[6], u[6];
  float nowMotor[6] = {0}, startMotor[6] = {0};
  float m1Trace[1000] = {0}, m2Trace[1000] = {0}, m3Trace[1000] = {0}, m4Trace[1000] = {0}, m5Trace[1000] = {0}, m6Trace[1000] = {0};
  float rate = 2;

/*********过渡到期望位置*********************/

  // 获取电机当前状态
  for (size_t i = 0; i < 30; i++)
  {
    rb.Motors[5]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[4]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[3]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[2]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[1]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[0]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.motor_send();
    r.sleep();
    /* code */
  }
  motor = *rb.Motors[0]->get_current_motor_state();
  nowMotor[0] = motor.position;
  motor = *rb.Motors[1]->get_current_motor_state();
  nowMotor[1] = motor.position;
  motor = *rb.Motors[2]->get_current_motor_state();
  nowMotor[2] = motor.position;
  motor = *rb.Motors[3]->get_current_motor_state();
  nowMotor[3] = motor.position;
  motor = *rb.Motors[4]->get_current_motor_state();
  nowMotor[4] = motor.position;
  motor = *rb.Motors[5]->get_current_motor_state();
  nowMotor[5] = motor.position;
  // 储存的拖动轨迹中，0元素是错误值，所以从10号开始
  X = 0.33 + 0.05 * sin(0.0);
  Y = 0.03;
  Z = 0.12;
  rotx = 0;
  roty = 1.5708;
  rotz = 0;
   arm6dof.Ik_6Dof(X, Y, Z, rotx, roty, rotz, joint);//逆解关节位置
  startMotor[0] = joint[0];
  startMotor[1] = -joint[1];
  startMotor[2] = joint[2];
  startMotor[3] = -joint[3];
  startMotor[4] = joint[4];
  startMotor[5] = joint[5];

  now2start(nowMotor, startMotor, m1Trace, m2Trace, m3Trace, m4Trace, m5Trace, m6Trace);
  cout << "》》》正在前往目的地！！\n"
       << endl;
  for (size_t i = 0; i < 1000; i++)
  {
    // cout<<"q:"<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<endl;
    rb.Motors[0]->fresh_cmd(m1Trace[i], 0.0, 0, 15.0, 15); // q1
    rb.Motors[1]->fresh_cmd(m2Trace[i], 0.0, 0, 15.0, 15); // q2
    rb.Motors[2]->fresh_cmd(m3Trace[i], 0.0, 0, 15.0, 15); // q3
    rb.Motors[3]->fresh_cmd(m4Trace[i], 0.0, 0, 15.0, 7);  // q4
    rb.Motors[4]->fresh_cmd(m5Trace[i], 0.0, 0, 15.0, 7);  // q5
    rb.Motors[5]->fresh_cmd(m6Trace[i], 0.0, 0, 15.0, 7);  // q6
    rb.motor_send();
    r.sleep();
  }

/*********e n d*********************/



  Kp[0] = 210; Kd[0] = 40;
  Kp[1] = 180; Kd[1] = 10;
  Kp[2] = 180; Kd[2] = 7;
  Kp[3] = 180; Kd[3] = 7;
  Kp[4] = 100; Kd[4] = 7;
  Kp[5] = 80; Kd[5] = 10;

  for (size_t i = 0; i < dynamicsTime; i++)
  {
    dynamicsCurrentTime = i/1000.0;
    rate = 3;
      // 1,计算期望轨迹
  X = 0.33 + 0.05 * sin(rate*dynamicsCurrentTime);
  Y = 0.03 + 0.05 * sin(rate*dynamicsCurrentTime);;
  Z = 0.12;
  rotx = 0;
  roty = 1.5708;
  rotz = 0;

  dX = 0.05 * rate*cos(rate*dynamicsCurrentTime);
  dY = 0.05 * rate*cos(rate*dynamicsCurrentTime);
  dZ = 0.0;
  drotx = 0;
  droty = 0.0;
  drotz = 0;
  endVel << dX, dY, dZ, drotx, droty, drotz;

  ddX = -0.05 * rate*rate*sin(rate*dynamicsCurrentTime);
  ddY = -0.05 * rate*rate*sin(rate*dynamicsCurrentTime);
  ddZ = 0.0;
  ddrotx = 0;
  ddroty = -0.0;
  ddrotz = 0;
  endAcc << ddX, ddY, ddZ, ddrotx, ddroty, ddrotz;
  efAcc[0] = ddX;
  efAcc[1] = ddY;
  efAcc[2] = ddZ;
  efAcc[3] = ddrotx;
  efAcc[4] = ddroty;
  efAcc[5] = ddrotz;

  // 2，逆运动学
  arm6dof.Ik_6Dof(X, Y, Z, rotx, roty, rotz, joint);//逆解关节位置

  jacon0 = arm6dof.Get_Jacob0(a, alpha, d, joint);
  dq = jacon0.inverse() * endVel; //逆解关节速度
  for (size_t i = 0; i < 6; i++) djoint[i] = dq(i);

  testVec = arm6dof.Get_Jdot(a_sd, alpha_sd, d_sd, joint, djoint, efAcc);
  endAcc = endAcc - testVec;
  ddq = jacon0.inverse() * (endAcc);//逆解关节加速度
  for (size_t i = 0; i < 6; i++) ddjoint[i] = ddq(i);

// cout<<"q"<<joint[0]<<", "<<joint[1]<<endl;


  //3，获取机械臂实时电机状态
    motor = *rb.Motors[0]->get_current_motor_state();
    motorRecv[0] = motor.position;
    motordRecv[0] = motor.velocity; // q6 and dq6
    // ROS_INFO_STREAM("ID: " << 3 << " Pos: " << JointState_B[2] );
    motor = *rb.Motors[1]->get_current_motor_state();
    motorRecv[1] = motor.position;
    motordRecv[1] = motor.velocity; // q5 and dq5
    // //ROS_INFO_STREAM("ID: " << 2 << " Pos: " << JointState_B[1] << " torque: " << motor.torque);
    motor = *rb.Motors[2]->get_current_motor_state();
    motorRecv[2] = motor.position;
    motordRecv[2] = motor.velocity; // q4 and dq4
    motor = *rb.Motors[3]->get_current_motor_state();
    motorRecv[3] = motor.position;
    motordRecv[3] = motor.velocity; // q3 and dq3
    // ROS_INFO_STREAM("ID: " << 3 << " Pos: " << JointState_B[2] );
    motor = *rb.Motors[4]->get_current_motor_state();
    motorRecv[4] = motor.position;
    motordRecv[4] = motor.velocity; // q2 and dq2
    // //ROS_INFO_STREAM("ID: " << 2 << " Pos: " << JointState_B[1] << " torque: " << motor.torque);
    motor = *rb.Motors[5]->get_current_motor_state();
    motorRecv[5] = motor.position;
    motordRecv[5] = motor.velocity; // q1 and dq1
//机械臂电机和模型的关节转动方向有差异，电机2和电机4与关节2关节4转动方向相反
    jointRecv[0]=motorRecv[0]; jointRecv[1]=-motorRecv[1]; jointRecv[2]=motorRecv[2]; 
    jointRecv[3]=-motorRecv[3]; jointRecv[4]=motorRecv[4]; jointRecv[5]=motorRecv[5]; 
    jointdRecv[0]=motordRecv[0]; jointdRecv[1]=-motordRecv[1]; jointdRecv[2]=motordRecv[2]; 
    jointdRecv[3]=-motordRecv[3]; jointdRecv[4]=motordRecv[4]; jointdRecv[5]=motordRecv[5]; 

  //4，计算控制律
  //在关节空间操作

    for (size_t i = 0; i < 6; i++)
    {
       e[i] = joint[i] - jointRecv[i];
       de[i] = djoint[i] - jointdRecv[i];
      u[i] = ddjoint[i] + Kp[i]*e[i] + Kd[i]*de[i];
    }
   // cout<<"e4: "<<e[3]<<", "<<motorTor[3]<<endl;


  //5，带入动力学模型
  torque = arm6dof.Get_Tor(a_sd, alpha_sd, d_sd, jointRecv, jointdRecv, u);
  //同理关节力矩也要转化成电机力矩
    motorTor[0] = torque(0); motorTor[1] = -torque(1); motorTor[2] = torque(2);
    motorTor[3] = -torque(3); motorTor[4] = torque(4); motorTor[5] = torque(5);
  
  

  //6，输入关节力矩
     rb.Motors[0]->fresh_cmd(0.0, 0.0, 1*motorTor[0], 0.0, 0.0);
     rb.Motors[1]->fresh_cmd(0.0, 0.0, 1*motorTor[1], 0.0, 0.0);
     rb.Motors[2]->fresh_cmd(0.0, 0.0, 1*motorTor[2], 0.0, 0.0);
     rb.Motors[3]->fresh_cmd(0.0, 0.0, 1*motorTor[3], 0.0, 0.0);
     rb.Motors[4]->fresh_cmd(0.0, 0.0, 1*motorTor[4], 0.0, 0.0);
     rb.Motors[5]->fresh_cmd(0.0, 0.0, 1*motorTor[5], 0.0, 0.0);
     rb.motor_send();
     //cout<<"tor: "<<motorTor[0]<<", "<<ddjoint[0]<<endl;

    r.sleep();
    /* code */
  }
    // 失能电机
  rb.Motors[0]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[1]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[2]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[3]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[4]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.Motors[5]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
  rb.motor_send();
  cout << "》》》轨迹跟踪结束！" << endl;

}

/*定点阻抗控制*/
impedence = true;
impedenceTime =16000;
if(impedence)
{
  cout<<">>>>>>>定点阻抗控制<<<<<<"<<endl;
  float nowMotor[6] = {0}, startMotor[6] = {0};
  float m1Trace[1000] = {0}, m2Trace[1000] = {0}, m3Trace[1000] = {0}, m4Trace[1000] = {0}, m5Trace[1000] = {0}, m6Trace[1000] = {0};
  float motorTor[6] = {0}, endTor[6] = {0}, grivTor[6] = {0};
  float dampingX = 0, elasticX = 0, dampingY = 0, elasticY = 0, dampingZ = 0, elasticZ = 0, FX,FY,FZ;
  float E[3], dE[3];
  float motorRecv[6],motordRecv[6], jointRecv[6], jointdRecv[6], impMotorTor[6];
  float Kp[6], Kd[6], e[6], de[6], u[6];
  float dltX = 0, dltY = 0.0, dltZ = 0.0;
  float endPos[3]={0};
  float limitTor[6]={0};
  int getEndTor[6] = {0};
  VectorXf motorTor_V(6), endTor_V(6), torRef(6), endVel(6), jointVel(6), impTor(6);

//注意定点时用的参数和轨迹跟踪时用的参数不同，这里更大
  Kp[0] = 210; Kd[0] = 40;
  Kp[1] = 260; Kd[1] = 10;
  Kp[2] = 260; Kd[2] = 7;
  Kp[3] = 250; Kd[3] = 7;
  Kp[4] = 210; Kd[4] = 7;
  Kp[5] = 80; Kd[5] = 10;


/*********过渡到期望位置*********************/

  // 获取电机当前状态
  for (size_t i = 0; i < 30; i++)
  {

    rb.Motors[5]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[4]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[3]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[2]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[1]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[0]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.motor_send();
    r.sleep();
    /* code */
  }
  
  motor = *rb.Motors[0]->get_current_motor_state();
  nowMotor[0] = motor.position;
  motor = *rb.Motors[1]->get_current_motor_state();
  nowMotor[1] = motor.position;
  motor = *rb.Motors[2]->get_current_motor_state();
  nowMotor[2] = motor.position;
  motor = *rb.Motors[3]->get_current_motor_state();
  nowMotor[3] = motor.position;
  motor = *rb.Motors[4]->get_current_motor_state();
  nowMotor[4] = motor.position;
  motor = *rb.Motors[5]->get_current_motor_state();
  nowMotor[5] = motor.position;
  // 储存的拖动轨迹中，0元素是错误值，所以从10号开始
  X = 0.33 + 0.05 * sin(0.0);
  Y = 0.03;
  Z = 0.12;
  rotx = 0;
  roty = 1.5708;
  rotz = 0;
   arm6dof.Ik_6Dof(X, Y, Z, rotx, roty, rotz, joint);//逆解关节位置
  startMotor[0] = joint[0];
  startMotor[1] = -joint[1];
  startMotor[2] = joint[2];
  startMotor[3] = -joint[3];
  startMotor[4] = joint[4];
  startMotor[5] = joint[5];

  now2start(nowMotor, startMotor, m1Trace, m2Trace, m3Trace, m4Trace, m5Trace, m6Trace);
  cout << "》》》正在前往目的地！！\n"
       << endl;
  for (size_t i = 0; i < 1000; i++)
  {
    // cout<<"q:"<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<", "<<m1Trace[i]<<endl;
    rb.Motors[0]->fresh_cmd(m1Trace[i], 0.0, 0, 15.0, 15); // q1
    rb.Motors[1]->fresh_cmd(m2Trace[i], 0.0, 0, 15.0, 15); // q2
    rb.Motors[2]->fresh_cmd(m3Trace[i], 0.0, 0, 15.0, 15); // q3
    rb.Motors[3]->fresh_cmd(m4Trace[i], 0.0, 0, 15.0, 7);  // q4
    rb.Motors[4]->fresh_cmd(m5Trace[i], 0.0, 0, 15.0, 7);  // q5
    rb.Motors[5]->fresh_cmd(m6Trace[i], 0.0, 0, 15.0, 7);  // q6
    rb.motor_send();
    r.sleep();
  }

/*********e n d*********************/


  // 获取电机当前力矩


for (size_t i = 0; i < impedenceTime; i++)
{


  X = 0.33 ;
  Y = 0.03;
  Z = 0.12;
  rotx = 0;
  roty = 1.5708;
  rotz = 0;
  arm6dof.Ik_6Dof(X, Y, Z, rotx, roty, rotz, joint);//逆解关节位置

    motor = *rb.Motors[0]->get_current_motor_state();
    motorRecv[0] = motor.position;
    motordRecv[0] = motor.velocity; // q6 and dq6
    // ROS_INFO_STREAM("ID: " << 3 << " Pos: " << JointState_B[2] );
    motor = *rb.Motors[1]->get_current_motor_state();
    motorRecv[1] = motor.position;
    motordRecv[1] = motor.velocity; // q5 and dq5
    // //ROS_INFO_STREAM("ID: " << 2 << " Pos: " << JointState_B[1] << " torque: " << motor.torque);
    motor = *rb.Motors[2]->get_current_motor_state();
    motorRecv[2] = motor.position;
    motordRecv[2] = motor.velocity; // q4 and dq4
    motor = *rb.Motors[3]->get_current_motor_state();
    motorRecv[3] = motor.position;
    motordRecv[3] = motor.velocity; // q3 and dq3
    // ROS_INFO_STREAM("ID: " << 3 << " Pos: " << JointState_B[2] );
    motor = *rb.Motors[4]->get_current_motor_state();
    motorRecv[4] = motor.position;
    motordRecv[4] = motor.velocity; // q2 and dq2
    // //ROS_INFO_STREAM("ID: " << 2 << " Pos: " << JointState_B[1] << " torque: " << motor.torque);
    motor = *rb.Motors[5]->get_current_motor_state();
    motorRecv[5] = motor.position;
    motordRecv[5] = motor.velocity; // q1 and dq1
//机械臂电机和模型的关节转动方向有差异，电机2和电s机4与关节2关节4转动方向相反
    jointRecv[0]=motorRecv[0]; jointRecv[1]=-motorRecv[1]; jointRecv[2]=motorRecv[2]; 
    jointRecv[3]=-motorRecv[3]; jointRecv[4]=motorRecv[4]; jointRecv[5]=motorRecv[5]; 
    jointdRecv[0]=motordRecv[0]; jointdRecv[1]=-motordRecv[1]; jointdRecv[2]=motordRecv[2]; 
    jointdRecv[3]=-motordRecv[3]; jointdRecv[4]=motordRecv[4]; jointdRecv[5]=motordRecv[5]; 

  for (size_t i = 0; i < 6; i++) {
    ddjoint[i] = 0.0;
    djoint[i] = 0.0;
    jointVel(i) = jointdRecv[i];

  }


    for (size_t i = 0; i < 6; i++)
    {
       e[i] = joint[i] - jointRecv[i];
       de[i] = 0 - jointdRecv[i];
      u[i] = 0 + Kp[i]*e[i] + Kd[i]*de[i];
    }


 jacon0 = arm6dof.Get_Jacob0(a, alpha, d, jointRecv);
 arm6dof.Get_endPos(a, alpha, d, jointRecv, endPos);//获取末端实时位置
 endVel = jacon0*jointVel;//获取末端实时速度

 elasticX = 150; dampingX=9.0;
 elasticY = 10; dampingY=0.5;
 elasticZ = 10; dampingZ=0.5;

E[0] = 0.33 - endPos[0];
E[1] = 0.03 - endPos[1];
E[2] = 0.12 - endPos[2];

dE[0] = - endVel(0);
dE[1] = - endVel(1);
dE[2] = - endVel(2);
 
 FX = elasticX*E[0] + dampingX*dE[0];
 FY = elasticY*E[1] + dampingY*dE[1];
 FZ = elasticZ*E[2] + dampingZ*dE[2];
 endTor_V(0) = FX;
 endTor_V(1) = FY;
 endTor_V(2) = FZ;
 endTor_V(3) = 0;
 endTor_V(4) = 0;
 endTor_V(5) = 0;

 impTor = jacon0.transpose()*endTor_V;//获取末端阻抗对应的关节阻抗
 impMotorTor[0] = impTor[0]; impMotorTor[1] = -impTor[1];
 impMotorTor[2] = impTor[2]; impMotorTor[3] = -impTor[3];
 impMotorTor[4] = impTor[4]; impMotorTor[5] = impTor[5];
 


  torque = arm6dof.Get_Tor(a_sd, alpha_sd, d_sd, jointRecv, jointdRecv, u);
  torRef = arm6dof.Get_Tor(a_sd, alpha_sd, d_sd, joint, djoint, ddjoint);
  motorTor[0] = torque(0);
  motorTor[1] = -torque(1);
  motorTor[2] = torque(2);
  motorTor[3] = -torque(3);
  motorTor[4] = torque(4);
  motorTor[5] = torque(5);

  torRef[1] = -torRef[1];
  torRef[3] = -torRef[3];

  limitTor[0] = 0.8;
  limitTor[1] = 1.6;
  limitTor[2] = 1.5;
  limitTor[3] = 2.1;
  limitTor[4] = 0.5;
  limitTor[5] = 0.5;
for (size_t i = 0; i < 6; i++)
{
  if(motorTor[i]>=(torRef(i)+limitTor[i])) motorTor[i] = torRef(i)+limitTor[i];
  if(motorTor[i]<=(torRef(i)-limitTor[i])) motorTor[i] = torRef(i)-limitTor[i];
  /* code */
}
for (size_t i = 0; i < 6; i++)
{
  motorTor[i] += impMotorTor[i]; 
  /* code */
}


//  cout<<"qTor:"<<motorTor[0]<<", "<<motorTor[1]<<", "<<motorTor[2]<<", "<<motorTor[3]<<", "<<motorTor[4]<<", "<<motorTor[5]<<endl;
    
//cout<<"endVel:"<<endVel(0)<<", "<<endVel(1)<<", "<<endVel(2)<<endl;
  //cout<<"endPos:"<<endPos[0]<<", "<<endPos[1]<<", "<<endPos[2]<<endl;
  // cout<<"F:"<<impTor(0)<<", "<<impTor(1)<<", "<<impTor(2)<<endl;
  
  rb.Motors[0]->fresh_cmd(0, 0.0, motorTor[0], 0.0, 0); // q1
  rb.Motors[1]->fresh_cmd(0, 0.0, motorTor[1], 0.0, 0); // q2
  rb.Motors[2]->fresh_cmd(0, 0.0, motorTor[2], 0.0, 0); // q3
  rb.Motors[3]->fresh_cmd(0, 0.0, motorTor[3], 0.0, 0);  // q4
  rb.Motors[4]->fresh_cmd(0, 0.0, motorTor[4], 0.0, 0);  // q5
  rb.Motors[5]->fresh_cmd(0, 0.0, motorTor[5], 0.0, 0);  // q6
  rb.motor_send();


  r.sleep();
  /* code */
}


    rb.Motors[5]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[4]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[3]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[2]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[1]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.Motors[0]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    rb.motor_send();
    cout<<"结束！"<<endl;



}




for (auto &thread : rb.ser_recv_threads)
{
  thread.join();
}

return 0;
}
