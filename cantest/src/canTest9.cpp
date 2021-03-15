#include "ros/ros.h"

#include <unistd.h>

#include <chrono>
#include <thread>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>

#include <cmath>
#include "Eigen/Dense"
#include <signal.h>


int8_t mode;
int8_t tra;
int flag;//用来判断是否接收到四个电机当前位置信号，并把它们设置为目标位置；只执行一次

int rxCounter;//计算模式下接受数据次数，如果在代码运行时进行模式更改，则重新计数

int limit[4];//这是用来判断是否碰到了障碍物，1X4的数组存对应的电机，初始化为0
int order[5];//在模式6中使用，默认值为0；如果为1，则进行正向行程扫描；如果为2，则进行反向行程扫描；如果为3，则扫描结束；其中第5个数据无效
int countrLimit;//在模式6中使用，用来计算速度差值积分项是否连续100次大于最大值
int t;

ros::Publisher positionPub;
ros::Publisher velocityPub;
ros::Publisher IPub;
ros::Publisher sendIPub;
ros::Publisher targetPositionPub;
ros::Publisher limitPub;
ros::Publisher xyPub;
ros::Publisher xyPub2;

ros::Subscriber PISub;
ros::Subscriber PSub;
ros::Subscriber targetPositionSub;
ros::Subscriber targetVelocitySub;
ros::Subscriber targetPositionAllSub;
ros::Subscriber modeSub;
ros::Subscriber orderSub;
ros::Subscriber targetAngleSub;
ros::Subscriber targetAngleSub2;
ros::Subscriber targetPositionXYSub;
ros::Subscriber traSub;

struct Motor{
	int16_t angle,velocity,I;
	uint8_t temperature;
	int16_t realAngle;
    int32_t position;
    int16_t NumOfTurns;

	int32_t targetPosition;
	int16_t targetVelocity;

	int16_t angleDifference;
	int16_t angleLast;

	int16_t sendI;

	int32_t positionDifference;
	int32_t positionDifferenceSum;

	int16_t velocityDifference;
	int32_t velocityDifferenceSum;
	
	float Ki;
	float Kp;

	float K;

	int16_t limit_0;
	int16_t limit_1;

	int16_t relativePosition;
	int16_t targetRelativePosition;
	int16_t positionIni;

	int16_t testVel;//用来扫描行程时，电机的速度

};
Motor motor[4];

struct PositionXY{
	float x;
	float y;
};
PositionXY positionXY;

struct AngleOfLink{
	float angle1;
	float angle2;
};
AngleOfLink AngleL1,AngleL2;

float	angle1Ini = 105.0/180*M_PI;
float	angle2Ini = -15.0/180*M_PI;	
float	angle3Ini = 75.0/180*M_PI;
float	angle4Ini = 195.0/180*M_PI;
float L1 = 50;
float L2 = 65;
float z1 = 17;
float z2 = 76;



struct PositionXYXY{
	PositionXY positionxy1;
	PositionXY positionxy2;

};
PositionXYXY p;

PositionXY fk(float angle1,float angle2)
{
	PositionXY positionxy;
	positionxy.x = L1*cos(angle1)+L2*cos(angle2);
	positionxy.y = L1*sin(angle1)+L2*sin(angle2);
	return positionxy;
}


AngleOfLink ik2(float x,float y)
{
	float angle0 = atan2(y,x);
	float L0 = sqrt(pow(x,2)+pow(y,2));

	float angle11 = acos((pow(x,2)+pow(y,2)+pow(L1,2)-pow(L2,2))/(2*L0*L1))+angle0;
	float angle12 = -acos((pow(x,2)+pow(y,2)+pow(L1,2)-pow(L2,2))/(2*L0*L1))+angle0;
	float angle21 = -acos((pow(x,2)+pow(y,2)+pow(L2,2)-pow(L1,2))/(2*L0*L2))+angle0;
	float angle22 = acos((pow(x,2)+pow(y,2)+pow(L2,2)-pow(L1,2))/(2*L0*L2))+angle0;
	AngleOfLink Angle1;

	if  (angle11>angle21) {Angle1.angle1 = angle11;Angle1.angle2 = angle21;}
	else {Angle1.angle1 = angle12;Angle1.angle2 = angle22;}
	return Angle1;

}

AngleOfLink ik3(float x,float y)
{
	float angle0 = atan2(y,x);
	float L0 = sqrt(pow(x,2)+pow(y,2));

	float angle11 = acos((pow(x,2)+pow(y,2)+pow(L1,2)-pow(L2,2))/(2*L0*L1))+angle0;
	float angle12 = -acos((pow(x,2)+pow(y,2)+pow(L1,2)-pow(L2,2))/(2*L0*L1))+angle0;
	float angle21 = -acos((pow(x,2)+pow(y,2)+pow(L2,2)-pow(L1,2))/(2*L0*L2))+angle0;
	float angle22 = acos((pow(x,2)+pow(y,2)+pow(L2,2)-pow(L1,2))/(2*L0*L2))+angle0;

	AngleOfLink Angle1;
	if  (angle11<angle21) {Angle1.angle1 = angle11;Angle1.angle2 = angle21;}
	else {Angle1.angle1 = angle12;Angle1.angle2 = angle22;}
	return Angle1;
}

AngleOfLink numIK(float x,float y,float angle1Current,float angle2Current)
{
    Eigen::MatrixXf targetPosition(2,1),jointAngleCurrent(2,1),currentPosition(2,1),jac(2,2),jac_P(2,2),dJointAngle(2,1);
	AngleOfLink Angle1;
	targetPosition(0,0) = y;
	targetPosition(1,0) = y;
	jointAngleCurrent(0,0) = angle1Current;
    jointAngleCurrent(1,0) = angle2Current;
	float L1 = 50;
	float L2 = 65;
	Eigen::MatrixXf positionDif(2,1);
    positionDif(0,0) = 1;
    positionDif(1,0) = 1;
	float limit = 10;
    float angle1,angle2;
	float lambda;
	while (positionDif.norm() > 1e-2)
		angle1 = jointAngleCurrent(1);
		angle2 = jointAngleCurrent(2);
		PositionXY positionXY;
		positionXY = fk(angle1,angle2);
		currentPosition(1) = positionXY.x;
		currentPosition(2) = positionXY.y;

		positionDif = targetPosition-currentPosition;
		jac(0,0)=-L1*sin(angle1);
		jac(0,1)=-L2*sin(angle2);
		jac(1,0)=L1*cos(angle1);
		jac(1,1) = L2*cos(angle2);
		if(positionDif.norm()>limit)
			lambda = 1; 
		else
			lambda = positionDif.norm()/10;
		jac_P = (jac.transpose() * jac + lambda*lambda*Eigen::MatrixXf::Ones(2,2)).inverse()*jac.transpose();
		dJointAngle = jac_P*positionDif;
		jointAngleCurrent = jointAngleCurrent + dJointAngle;
	Angle1.angle1 = jointAngleCurrent(1);
	Angle1.angle2 = jointAngleCurrent(2);
	return Angle1;

}



void motorInit(void){
	flag = 0;
	for(int i = 0;i<4;i++)
	{
		motor[i].targetPosition = 0;
		motor[i].targetVelocity = 0;
		motor[i].Kp = 13;
		motor[i].Ki = 6;
		motor[i].K = 15;
		motor[i].limit_0 = 0;
		motor[i].limit_1 = 0;

		limit[i] = 0;
		order[i] = 0;

	}
	order[4] = 0;
	order[0] = 1;
	countrLimit = 0;
	motor[0].testVel = 150;
	motor[1].testVel = 150;
	motor[2].testVel = -150;
	motor[3].testVel = -150;

}


void PI_Callback(std_msgs::Float32MultiArray PIMessage)
{
	int ID = 0;
    motor[ID].Kp = PIMessage.data[0];
    motor[ID].Ki = PIMessage.data[1];
}

void P_Callback(std_msgs::Float32 PMessage)
{
	int ID = 0;
	motor[ID].K = PMessage.data;
}

void targetPosition_Callback(std_msgs::Float32 targetPositionMessage){
	int ID = 0;
	motor[ID].targetPosition = targetPositionMessage.data;

}

void targetPositionAll_Callback(std_msgs::Float32MultiArray targetPositionAllMessage){
	for(int i = 0;i<4;i++) motor[i].targetPosition = targetPositionAllMessage.data[i];
	
}

void targetVelocity_Callback(std_msgs::Float32 targetVelocityMessage){
	int ID = 0;
	motor[ID].targetVelocity = targetVelocityMessage.data;
}


// void targetVelocity_Callback(std_msgs::Float32MultiArray targetVelocityMessage){

// 	motor[0].targetVelocity = targetVelocityMessage.data[0];
// 	motor[1].targetVelocity = targetVelocityMessage.data[1];
// }

void mode_Callback(std_msgs::Int8 modeMessage)
{
	mode = modeMessage.data;
    motorInit();

	rxCounter = 0;
}

void order_Callback(std_msgs::Int8 orderMessage)
{
	order[0] = 1;
}

void tra_Callback(std_msgs::Int8 traMessage)
{
	tra = traMessage.data;
	t = 0;
}

void targetAngleSub_Callback(std_msgs::Float32MultiArray targetAngleMessage)
{
	AngleL1.angle1 = targetAngleMessage.data[0];
	AngleL1.angle2 = targetAngleMessage.data[1];
	// linkAngle2motorPosition(linksys,motor[0],motor[1]);
	motor[0].targetPosition = motor[0].positionIni - z2/z1*(AngleL1.angle1 - angle1Ini)*8192/(2*M_PI);
	motor[1].targetPosition = motor[1].positionIni + z2/z1*(AngleL1.angle2 - angle2Ini)*8192/(2*M_PI);
}
void targetAngleSub2_Callback(std_msgs::Float32MultiArray targetAngleMessage)
{
	AngleL1.angle1 = targetAngleMessage.data[0];
	AngleL1.angle2 = targetAngleMessage.data[1];
	AngleL2.angle1 = targetAngleMessage.data[2];
	AngleL2.angle2 = targetAngleMessage.data[3];

	motor[0].targetPosition = motor[0].positionIni - z2/z1*(AngleL1.angle1 - angle1Ini)*8192/(2*M_PI);
	motor[1].targetPosition = motor[1].positionIni + z2/z1*(AngleL1.angle2 - angle2Ini)*8192/(2*M_PI);

	motor[2].targetPosition = motor[2].positionIni - z2/z1*(AngleL2.angle1 - angle3Ini)*8192/(2*M_PI);
	motor[3].targetPosition = motor[3].positionIni + z2/z1*(AngleL2.angle2 - angle4Ini)*8192/(2*M_PI);
}

void targetPositionXYSub_Callback(std_msgs::Float32MultiArray targetPositionXYMessage)
{
	float x1,y1,x2,y2;
	x1 = targetPositionXYMessage.data[0];
	y1 = targetPositionXYMessage.data[1];
	x2 = targetPositionXYMessage.data[2];
	y2 = targetPositionXYMessage.data[3];

	AngleL1 = ik2(x1,y1);
	AngleL2 = ik3(x2,y2);

	motor[0].targetPosition = motor[0].positionIni - z2/z1*(AngleL1.angle1 - angle1Ini)*8192/(2*M_PI);
	motor[1].targetPosition = motor[1].positionIni + z2/z1*(AngleL1.angle2 - angle2Ini)*8192/(2*M_PI);

	motor[2].targetPosition = motor[2].positionIni - z2/z1*(AngleL2.angle1 - angle3Ini)*8192/(2*M_PI);
	motor[3].targetPosition = motor[3].positionIni + z2/z1*(AngleL2.angle2 - angle4Ini)*8192/(2*M_PI);
}

void signal_Callback(int signum)
{
	mode = 0;
	sleep(0.5);
	printf("Caught signal %d\n", signum);
}

void xy2motorTargetPosition(float x1,float y1,float x2,float y2)
{
	AngleL1 = ik2(x1,y1);
	AngleL2 = ik3(x2,y2);

	motor[0].targetPosition = motor[0].positionIni - z2/z1*(AngleL1.angle1 - angle1Ini)*8192/(2*M_PI);
	motor[1].targetPosition = motor[1].positionIni + z2/z1*(AngleL1.angle2 - angle2Ini)*8192/(2*M_PI);

	motor[2].targetPosition = motor[2].positionIni - z2/z1*(AngleL2.angle1 - angle3Ini)*8192/(2*M_PI);
	motor[3].targetPosition = motor[3].positionIni + z2/z1*(AngleL2.angle2 - angle4Ini)*8192/(2*M_PI);
}

PositionXYXY motorPosition2xy(void)
{	
	PositionXY positionxy3,positionxy4;
	AngleOfLink anglel1,anglel2;
	PositionXYXY positionxyxy;
	anglel1.angle1 = angle1Ini -(motor[0].position - motor[0].positionIni)*(2*M_PI)/8192.0*(z1/z2);
	anglel1.angle2 = angle2Ini +(motor[1].position - motor[1].positionIni)*(2*M_PI)/8192.0*(z1/z2);
	anglel2.angle1 = angle3Ini -(motor[2].position - motor[2].positionIni)*(2*M_PI)/8192.0*(z1/z2);
	anglel2.angle2 = angle4Ini +(motor[3].position - motor[3].positionIni)*(2*M_PI)/8192.0*(z1/z2);
	positionxy3 = fk(anglel1.angle1,anglel1.angle2);
	positionxy4 = fk(anglel2.angle1,anglel2.angle2);
	positionxyxy.positionxy1 = positionxy3;
	positionxyxy.positionxy2 = positionxy4;
	return positionxyxy;
}
std_msgs::Int32MultiArray positionMessage,velocityMessage,IMessage,sendIMessage,targetPositionMessage;
std_msgs::Int16MultiArray limitMessage;
std_msgs::Float32MultiArray xyMessage,xy2Message;

void rxThread(int s)
{
	int ID;
	int i;
	int j;
	struct can_frame frame;
	int nbytes;
	rxCounter= 0;

	positionMessage.data.resize(4);
	velocityMessage.data.resize(4);
	IMessage.data.resize(4);
	targetPositionMessage.data.resize(4);
	limitMessage.data.resize(8);

    
	for (j = 0;j<4;j++)
	{
		motor[j].angleDifference = 0;
		motor[j].NumOfTurns = 0;
	}

    for (i = 0;; i++)
    {
		ros::spinOnce();
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if (nbytes < 0)
		{
			perror("Read");
			break;
		}

		ID = int(frame.can_id-0x200)-1;

		rxCounter++;

        motor[ID].angle = (frame.data[0] << 8)+ frame.data[1];
		motor[ID].realAngle = motor[ID].angle*360/8191;
		motor[ID].velocity = (frame.data[2] <<8) + frame.data[3];
		motor[ID].I = (frame.data[4] <<8) + frame.data[5];
		motor[ID].temperature = frame.data[6];

        if (i>4){
            motor[ID].angleDifference = motor[ID].angle - motor[ID].angleLast;
        }

        if(motor[ID].angleDifference<-4000)
        {
            motor[ID].NumOfTurns++;
        }
        if(motor[ID].angleDifference>4000)
        {
            motor[ID].NumOfTurns--;
        }

        motor[ID].position = 8192*motor[ID].NumOfTurns+motor[ID].angle;
        motor[ID].angleLast = motor[ID].angle;


		positionMessage.data[ID] = motor[ID].position;
		velocityMessage.data[ID] = motor[ID].velocity;
		IMessage.data[ID] = motor[ID].I;

		if(i%4==0)
		{
			switch (mode)
			{
				case 0:
					printf("mode is %d; ",mode);
					printf("angle is %d,%d,%d,%d\n",motor[0].angle,motor[1].angle,motor[2].angle,motor[3].angle);
					break;
				case 1:
					printf("mode is %d; ",mode);
					printf("kp is %f; ki is %f; vel is %d; tv is %d; vel_d is %d; vel_d_sum = %d; send_I is %d; I is %d\n",
					motor[0].Kp,motor[0].Ki,motor[0].velocity,motor[0].targetVelocity,motor[0].velocityDifference,motor[0].velocityDifferenceSum,
					motor[0].sendI,motor[0].I);
					break;

				case 2:
					printf("mode is %d; ",mode);
					printf("k %f; p is %d; tp is %d; send_I is %d; I is %d\n",
					motor[0].K,motor[0].position,motor[0].targetPosition,
					motor[0].sendI,motor[0].I);

					break;

				case 3:
					printf("mode is %d; ",mode);
					printf("vel is %d; tv is %d; vel_d is %d; vel_d_sum = %d; send_I is %d; I is %d\n",
					motor[0].velocity,motor[0].targetVelocity,motor[0].velocityDifference,motor[0].velocityDifferenceSum,
					motor[0].sendI,motor[0].I);

					break;

				case 4:
					printf("mode is %d; ",mode);
					printf("p is %d; tp is %d; send_I is %d; I is %d\n",
					motor[0].position,motor[0].targetPosition,
					motor[0].sendI,motor[0].I);

					break;
				case 5:
					printf("mode is %d; ",mode);
					printf("position is %d,%d,%d,%d",motor[0].position,motor[1].position,motor[2].position,motor[3].position);
					printf("target position is %d,%d,%d,%d;",motor[0].targetPosition,motor[1].targetPosition,motor[2].targetPosition,motor[3].targetPosition);
					printf("sendI is %d,%d,%d,%d;",motor[0].sendI,motor[1].sendI,motor[2].sendI,motor[3].sendI);
					printf("I is %d,%d,%d,%d;",motor[0].I,motor[1].I,motor[2].I,motor[3].I);
					printf("\n");
					break;
					
				case 6:
					printf("mode is %d; ",mode);
					
					printf("limitmotor0 is %d %d;",motor[0].limit_0,motor[0].limit_1);
					printf("limitmotor1 is %d %d;",motor[1].limit_0,motor[1].limit_1);
					printf("limitmotor2 is %d %d;",motor[2].limit_0,motor[2].limit_1);
					printf("limitmotor3 is %d %d;\n",motor[3].limit_0,motor[3].limit_1);
					break;

				case 7:
			
					printf("mode is %d; ",mode);
					printf("l1 a1 a2 are: %f %f;",AngleL1.angle1,AngleL1.angle2);
					printf("a1_ini a2_ini are :%f %f;",angle1Ini,angle2Ini);
					printf("pos_ini are :%d %d;",motor[0].positionIni,motor[1].positionIni);
					printf("motor0 targetposition is %d;motor1 targetposition is %d;\n",motor[0].targetPosition,motor[1].targetPosition);
					break;

                case 8:
					printf("mode is %d; ",mode);
					printf("l1 a1 a2 a3 a4 are: %f %f %f %f;",AngleL1.angle1,AngleL1.angle2,AngleL2.angle1,AngleL2.angle2);
					printf("a1_ini a2_ini a3_ini a4_ini are :%f %f %f %f;",angle1Ini,angle2Ini,angle3Ini,angle4Ini);
					printf("tp are %d %d %d %d;\n",motor[0].targetPosition,motor[1].targetPosition,motor[2].targetPosition,motor[3].targetPosition);

                    break;
				case 9:
					printf("mode is %d; ",mode);printf("tra is %d; ",tra);
					printf("l1 a1 a2 a3 a4 are: %f %f %f %f;",AngleL1.angle1,AngleL1.angle2,AngleL2.angle1,AngleL2.angle2);
					//printf("a1_ini a2_ini a3_ini a4_ini are :%f %f %f %f;",angle1Ini,angle2Ini,angle3Ini,angle4Ini);
					printf("tp are %d %d %d %d;\n",motor[0].targetPosition,motor[1].targetPosition,motor[2].targetPosition,motor[3].targetPosition);

                    break;
				default:
					mode = 0;
					printf("请设定正常的mode,现在已经更改mode为0\n");
			}
			positionPub.publish(positionMessage);
			velocityPub.publish(velocityMessage);
			IPub.publish(IMessage);
			for(j = 1;j<4;j++) {
				limitMessage.data[2*j] = motor[j].limit_0;
				limitMessage.data[2*j+1] = motor[j].limit_1;
			}
			limitPub.publish(limitMessage);


		}
		if(rxCounter == 32 && ((mode == 5 || mode == 6 || mode == 7 || mode == 8 ||mode == 9)))
		{
			flag = 1;
			for (j = 0;j<4;j++)
			{
				targetPositionMessage.data[j] = motor[j].position;
				motor[j].targetPosition = motor[j].position;
			}
			
		}
		if(rxCounter>32) targetPositionPub.publish(targetPositionMessage);
		std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    }

}

void controlP_calSendI_PI(int ID){
	motor[ID].positionDifference = motor[ID].targetPosition - motor[ID].position;
	motor[ID].positionDifferenceSum = motor[ID].positionDifference + motor[ID].positionDifferenceSum;

	motor[ID].sendI = motor[ID].Kp*motor[ID].positionDifference + motor[ID].Ki*motor[ID].positionDifferenceSum;
}

void controlV_calSendI_PI(int ID){
	int velocityDifferenceSumLimit = 16667;
	limit[ID] = 0;
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	if(motor[ID].velocityDifference > 1000) motor[ID].velocityDifference = 1000;
	if(motor[ID].velocityDifference < -1000) motor[ID].velocityDifference = -1000;
	motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

	if(motor[ID].velocityDifferenceSum>velocityDifferenceSumLimit) {motor[ID].velocityDifferenceSum = velocityDifferenceSumLimit;limit[ID] = 1;}
	if(motor[ID].velocityDifferenceSum<(0-velocityDifferenceSumLimit)) {motor[ID].velocityDifferenceSum = (0-velocityDifferenceSumLimit);limit[ID] = 1;}

	motor[ID].sendI = motor[ID].Kp*motor[ID].velocityDifference + motor[ID].Ki/10*motor[ID].velocityDifferenceSum;
}

void controlP_calSendI_PPI(int ID){
	int velocityDifferenceSumLimit = 16667;
	limit[ID] = 0;

	motor[ID].positionDifference = motor[ID].targetPosition - motor[ID].position;
	if(motor[ID].positionDifference > 4000) motor[ID].positionDifference = 4000;
	if(motor[ID].positionDifference < -4000) motor[ID].positionDifference = -4000;

	motor[ID].targetVelocity = motor[ID].K/100*(motor[ID].positionDifference);
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

	if(motor[ID].velocityDifferenceSum>velocityDifferenceSumLimit) {motor[ID].velocityDifferenceSum = velocityDifferenceSumLimit;limit[ID] = 1;}
	if(motor[ID].velocityDifferenceSum<(0-velocityDifferenceSumLimit)) {motor[ID].velocityDifferenceSum = (0-velocityDifferenceSumLimit);limit[ID] = 1;}

	motor[ID].sendI = motor[ID].Kp*motor[ID].velocityDifference + motor[ID].Ki/10*motor[ID].velocityDifferenceSum;
}

void testLimit(int ID){

	switch (order[ID])
	{
		case 0:
			break;

		case 1:
			motor[ID].targetVelocity = motor[ID].testVel;
			controlV_calSendI_PI(ID);
			
			if (limit[ID] == 1){
				countrLimit++;
				if(countrLimit >100)
				{
					motor[ID].limit_1 = motor[ID].position;
					
					order[ID] = 2;
					countrLimit = 0;
				}
			}
			else{
				countrLimit = 0;
			}
			break;

		case 2:
			motor[ID].targetVelocity = -motor[ID].testVel;
			controlV_calSendI_PI(ID);
			if (limit[ID] == 1){
				countrLimit++;
				if(countrLimit >100)
				{
					motor[ID].limit_0 = motor[ID].position;
					
					order[ID] = 3;
					countrLimit = 0;
				}
			}
			else{
				countrLimit = 0;
			}
			break;

		case 3:
			order[ID+1] = 1;
			order[ID] = 0;
			motor[ID].sendI = 0;
			break;

		default:
		printf("这个不可能被打印出来！");
	}

}

void getLimit(void)
{
	testLimit(0);
	testLimit(1);
	testLimit(3);
	testLimit(2);
	
	motor[0].positionIni = motor[0].limit_1;
	motor[1].positionIni = motor[1].limit_0;
	motor[2].positionIni = motor[2].limit_1;
	motor[3].positionIni = motor[3].limit_0;
}

void txThread(int s)
{
    struct can_frame frame;
	frame.can_id = 0x200;
	frame.can_dlc = 8;
	int j;
	sendIMessage.data.resize(4);
	xyMessage.data.resize(4);
	xy2Message.data.resize(4);
	float x1,y1,x2,y2;

	for (j = 0; j < 4; j++)
	{
		motor[j].sendI = 0;
		frame.data[2*j] = motor[j].sendI << 8;
		frame.data[2*j+1] = motor[j].sendI << 0;
	}

	int nbytes;

    for (int i = 0;; i++)
	{
		ros::spinOnce();
		switch (mode)
		{
			case 0:
				break;
			case 1:
				controlV_calSendI_PI(0);
				break;
			case 2:
				controlP_calSendI_PPI(0);
				break;
			case 3:
				controlV_calSendI_PI(0);
				break;
			case 4:
				controlP_calSendI_PPI(0);
				break;
			case 5:
				if (flag == 1){
					controlP_calSendI_PPI(0);
					controlP_calSendI_PPI(1);
					controlP_calSendI_PPI(2);
					controlP_calSendI_PPI(3);
				}
				break;
			case 6:
				testLimit(0);
				testLimit(1);
				testLimit(3);
                testLimit(2);
				break;
				
			case 7:
				testLimit(0);
				testLimit(1);
				testLimit(3);
                testLimit(2);
				motor[0].positionIni = motor[0].limit_1;
				motor[1].positionIni = motor[1].limit_0;
				if (order[2] == 1) {
					controlP_calSendI_PPI(0);
					controlP_calSendI_PPI(1);
				}
				break;
			case 8:
				getLimit();
				if (order[4] == 1 && flag == 1) 
				{
					controlP_calSendI_PPI(0);
					controlP_calSendI_PPI(1);
					controlP_calSendI_PPI(2);
					controlP_calSendI_PPI(3);
				};
                break;
			case 9:
				getLimit();
				
				if (order[4] == 1 && flag == 1) 
				{
					switch (tra)
					{
						case 0:
							x1 = 0;
							y1 = 60;
							x2 = 0;
							y2 = 60;
							break;
						case 1:
							x1 = 20+30*sin(float(t)/5.0/(2*M_PI));
							y1 = 60;
							x2 = -x1;
							y2 = y1;
							break;

						case 2:
							x1 = 0;
							y1 = 60+15*sin(float(t)/5.0/(2*M_PI));
							x2 = x1;
							y2 = y1;
							break;
						case 3:
							x1 = 10;
							y1 = 60;
							x2 = -x1;
							y2 = 60;
							break;

						case 4:
							x1 = 20;
							y1 = 60;
							x2 = -x1;
							y2 = 60;
							break;
						case 5:
							x1 = 30;
							y1 = 60;
							x2 = -x1;
							y2 = 60;
							break;

						case 6:
							x1 = 35;
							y1 = 60;
							x2 = -x1;
							y2 = 60;
							break;
						case 7:
							x1 = 40;
							y1 = 60;
							x2 = -x1;
							y2 = 60;
							break;
						case 8:
							x1 = 45;
							y1 = 60;
							x2 = -x1;
							y2 = 60;
							break;
						case 9:
							x1 = 50;
							y1 = 60;
							x2 = -x1;
							y2 = 60;
							break;
						default:
							tra = 0;

					}

					xy2motorTargetPosition(x1,y1,x2,y2);
					
					p = motorPosition2xy();
					xyMessage.data[0] = x1;
					xyMessage.data[1] = y1;
					xyMessage.data[2] = x2;
					xyMessage.data[3] = y2;
					xy2Message.data[0] = p.positionxy1.x;
					xy2Message.data[1] = p.positionxy1.y;
					xy2Message.data[2] = p.positionxy2.x;
					xy2Message.data[3] = p.positionxy2.y;
					xyPub2.publish(xy2Message);
					xyPub.publish(xyMessage);
					controlP_calSendI_PPI(0);
					controlP_calSendI_PPI(1);
					controlP_calSendI_PPI(2);
					controlP_calSendI_PPI(3);
					t++;
					sleep(0.0001);	
				};
				
				break;
				

			default:
				mode = 0;
				printf("请设定正常的mode,现在已经更改mode为0\n");

		}

		for (j = 0;j<4;j++)
		{
			if (motor[j].sendI >15000) {
				motor[j].sendI = 15000;
			}
			if (motor[j].sendI <-15000) {
				motor[j].sendI = -15000;
			}
			frame.data[2*j] = motor[j].sendI>>8;
			frame.data[2*j+1] = motor[j].sendI>>0;
			sendIMessage.data[j] = motor[j].sendI;

		}
		sendIPub.publish(sendIMessage);
        nbytes = write(s, &frame, sizeof(struct can_frame));
        if (nbytes == -1) {
			printf("send error\n");
			printf("Please check if the power switch is turned on\n");
			exit (0);

        }
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
    }

}

int main(int argc, char** argv) {

	mode = 0;
	tra = 0;
	t = 0;
	motorInit();
	signal(SIGINT,signal_Callback);
	ros::init(argc,argv,"canTest8");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

    positionPub = n.advertise<std_msgs::Int32MultiArray>("position",100);
    velocityPub = n.advertise<std_msgs::Int32MultiArray>("velocity",100);
    IPub = n.advertise<std_msgs::Int32MultiArray>("I",100);
	sendIPub = n.advertise<std_msgs::Int32MultiArray>("sendI",100);
	targetPositionPub = n.advertise<std_msgs::Int32MultiArray>("targetPositionFromPosition",100);
	limitPub = n.advertise<std_msgs::Int16MultiArray>("limit",100);
	xyPub = n.advertise<std_msgs::Float32MultiArray>("xy",100);
	xyPub2 = n.advertise<std_msgs::Float32MultiArray>("xy2",100);


	int s;
	struct sockaddr_can addr;
	struct ifreq ifr;

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, "can0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifr.ifr_name, ifr.ifr_ifindex);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

	std::thread canRx(rxThread, s);
	sleep(0.5);
	std::thread canTx(txThread, s);

    PISub = n.subscribe("PI", 10, PI_Callback);
	PSub = n.subscribe("P",10,P_Callback);
	targetVelocitySub = n.subscribe("targetVelocity", 10, targetVelocity_Callback);
	targetPositionSub = n.subscribe("targetPosition", 10, targetPosition_Callback);
    targetPositionAllSub = n.subscribe("targetPositionAll", 10, targetPositionAll_Callback);
	targetAngleSub = n.subscribe("targetAngle", 10, targetAngleSub_Callback);
	targetAngleSub2 = n.subscribe("targetAngle2", 10, targetAngleSub2_Callback);
	targetPositionXYSub = n.subscribe("targetPositionXY", 10, targetPositionXYSub_Callback);
	modeSub = n.subscribe("mode", 10, mode_Callback);
	orderSub = n.subscribe("order",10,order_Callback);
	traSub = n.subscribe("tra",10,tra_Callback);

	while (ros::ok())
    {

		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
