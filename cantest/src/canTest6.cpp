
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


int8_t mode;
int flag;//用来判断是否接收到四个电机当前位置信号，并把它们设置为目标位置；只执行一次

int rxCounter;//计算模式下接受数据次数，如果在代码1运行时进行，模式更改，则重新计数

int limit[4];//这是用来判断是否碰到了障碍物，1X4的数组存对应的电机，初始化为0
int order[5];//在模式6中使用，默认值为0；如果为1，则进行正向行程扫描；如果为2，则进行反向行程扫描；如果为3，则扫描结束；其中第5个数据无效
int countrLimit;//在模式6中使用，用来计算速度差值积分项是否连续100次大于最大值

ros::Publisher positionPub;
ros::Publisher velocityPub;
ros::Publisher IPub;
ros::Publisher sendIPub;
ros::Publisher targetPositionPub;
ros::Publisher limitPub;

ros::Subscriber PISub;
ros::Subscriber PSub;
ros::Subscriber targetPositionSub;
ros::Subscriber targetVelocitySub;
ros::Subscriber targetPositionAllSub;
ros::Subscriber modeSub;
ros::Subscriber orderSub;
ros::Subscriber targetAngleSub;
ros::Subscriber targetAngleSub2;

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
	//电机速度环增益
	float Ki;
	float Kp;
	//电机位置环增益
	float K;
	//电机行程
	int16_t limit_0;
	int16_t limit_1;

	//电机行程坐标下的位置
	int16_t relativePosition;
	int16_t targetRelativePosition;
	int16_t positionIni;
	//电机的实际行程
	//这个怎么描述呢？

};
Motor motor[4];

struct LinkSys{
	float L0;
	float L1 = 50;
	float L2 = 65;

	float z1 = 17;
	float z2 = 76;
	float m = 0.6;

	float angle0;
	float angle1;
	float angle2;

	float x;
	float y;

	float angle11, angle12;
	float angle21, angle22;

	float angle1Ini = 105.0/180*M_PI;
	float angle2Ini = -15.0/180*M_PI;
};
LinkSys linksys,linksys2;

void calLinkSysXYFromAngle(LinkSys linksys,float angle1,float angle2)
{
	linksys.angle1 = angle1;
	linksys.angle2 = angle2;
	linksys.x = linksys.L1*cos(linksys.angle1)+linksys.L2*cos(linksys.angle2);
	linksys.y = linksys.L1*sin(linksys.angle1)+linksys.L2*sin(linksys.angle2);
}

void calLinkSysangleFromXY_1(LinkSys linksys,float x,float y)
{
	//TUDO: 
	linksys.angle0 = atan(linksys.y/linksys.x);
	linksys.L0 = sqrt(pow(linksys.x,2)+pow(linksys.y,2));

	linksys.angle11 = acos((pow(linksys.x,2)+pow(linksys.y,2)+pow(linksys.L1,2)-pow(linksys.L2,2))/(2*linksys.L0*linksys.L1))+linksys.angle0;
	linksys.angle12 = -acos((pow(linksys.x,2)+pow(linksys.y,2)+pow(linksys.L1,2)-pow(linksys.L2,2))/(2*linksys.L0*linksys.L1))+linksys.angle0;
	linksys.angle21 = -acos((pow(linksys.x,2)+pow(linksys.y,2)+pow(linksys.L2,2)-pow(linksys.L1,2))/(2*linksys.L0*linksys.L2))+linksys.angle0;
	linksys.angle22 = acos((pow(linksys.x,2)+pow(linksys.y,2)+pow(linksys.L2,2)-pow(linksys.L1,2))/(2*linksys.L0*linksys.L2))+linksys.angle0;

}

void calLinkSysangleFromXY_2(LinkSys linksys,float x,float y,float Angle1Current,float Angle2Current)
{
	//TUDO COMPLETE IT
	// targetPosition = [x;y];
	// jointAngleCurrent = [angle1Current;angle2Current];
	// L1 = 50;
	// L2 = 65;
	// positionDif = 1;%initilize position difference
	// limit = 10;%used to control lambda
	// while (norm(positionDif) > 1e-2)
	// 	angle1 = jointAngleCurrent(1);
	// 	angle2 = jointAngleCurrent(2);
	// 	[currentPosition1,currentPosition2] = fk(jointAngleCurrent(1),jointAngleCurrent(2));
	// 	currentPosition = [currentPosition1;currentPosition2];
	// 	positionDif = targetPosition-currentPosition;
	// 	jac = [-L1*sin(angle1) -L2*sin(angle2);L1*cos(angle1) L2*cos(angle2)];
	// 	if(norm(positionDif)>limit)
	// 		lambda = 1; 
	// 	else
	// 		lambda = norm(positionDif)/10;
	// 	end
	// 	jac_P = (jac'*jac+lambda^2)^(-1)*jac';%pseudo inverse jacobe
	// 	dJointAngle = jac_P*positionDif;
	// 	jointAngleCurrent = jointAngleCurrent + dJointAngle;
	// end
	// jointAngle1 = jointAngleCurrent(1);
	// jointAngle2 = jointAngleCurrent(2);

}


void motorPosition2linkAngle(LinkSys linksys,Motor motor1,Motor motor2)
{
	linksys.angle1 = linksys.angle1Ini - linksys.z1/linksys.z2*(motor1.position - motor1.positionIni)*2*M_PI/8192;
	linksys.angle2 = linksys.angle2Ini + linksys.z1/linksys.z2*(motor2.position - motor2.positionIni)*2*M_PI/8192;
}

void linkAngle2motorPosition(LinkSys linksys,Motor motor1,Motor motor2)
{
	motor1.targetPosition = motor1.positionIni - linksys.z2/linksys.z1*(linksys.angle1 - linksys.angle1Ini)*8192/(2*M_PI);
	motor2.targetPosition = motor2.positionIni + linksys.z2/linksys.z1*(linksys.angle2 - linksys.angle2Ini)*8192/(2*M_PI);
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

void mode_Callback(std_msgs::Int8 modeMessage)
{
	mode = modeMessage.data;
    motorInit();

	rxCounter = 0;
}

void order_Callback(std_msgs::Int8 orderMessage)
{
	//这个变量决定了是否在模式六或7下面执行行程获取
	//因为执行一遍后这个变量会被清0
	order[0] = 1;
}

void targetAngleSub_Callback(std_msgs::Float32MultiArray targetAngleMessage)
{
	linksys.angle1 = targetAngleMessage.data[0];
	linksys.angle2 = targetAngleMessage.data[1];
	// linkAngle2motorPosition(linksys,motor[0],motor[1]);
	motor[0].targetPosition = motor[0].positionIni - linksys.z2/linksys.z1*(linksys.angle1 - linksys.angle1Ini)*8192/(2*M_PI);
	motor[1].targetPosition = motor[1].positionIni + linksys.z2/linksys.z1*(linksys.angle2 - linksys.angle2Ini)*8192/(2*M_PI);
}
void targetAngleSub2_Callback(std_msgs::Float32MultiArray targetAngleMessage)
{
	linksys.angle1 = targetAngleMessage.data[0];
	linksys.angle2 = targetAngleMessage.data[1];
	linksys2.angle1 = targetAngleMessage.data[2];
	linksys2.angle2 = targetAngleMessage.data[3];

	motor[0].targetPosition = motor[0].positionIni - linksys.z2/linksys.z1*(linksys.angle1 - linksys.angle1Ini)*8192/(2*M_PI);
	motor[1].targetPosition = motor[1].positionIni + linksys.z2/linksys.z1*(linksys.angle2 - linksys.angle2Ini)*8192/(2*M_PI);

	motor[2].targetPosition = motor[2].positionIni - linksys2.z2/linksys2.z1*(linksys2.angle1 - linksys2.angle1Ini)*8192/(2*M_PI);
	motor[3].targetPosition = motor[3].positionIni + linksys2.z2/linksys2.z1*(linksys2.angle2 - linksys2.angle2Ini)*8192/(2*M_PI);
}


std_msgs::Int32MultiArray positionMessage,velocityMessage,IMessage,sendIMessage,targetPositionMessage;
std_msgs::Int16MultiArray limitMessage;

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
					// printf("order is %d;%d;%d;%d;%d;",order[0],order[1],order[2],order[3],order[4]);
					printf("limitmotor0 is %d %d;",motor[0].limit_0,motor[0].limit_1);
					printf("limitmotor1 is %d %d;",motor[1].limit_0,motor[1].limit_1);
					printf("limitmotor2 is %d %d;",motor[2].limit_0,motor[2].limit_1);
					printf("limitmotor3 is %d %d\n;",motor[3].limit_0,motor[3].limit_1);
					break;

				case 7:
				//TUDO 完成两个电机在笛卡尔坐标系下运动即，给定笛卡尔坐标系下的位置，然后得到两个电机的角度值，并利用位置控制模式使末端执行器固定在那个位置上
					printf("mode is %d; ",mode);
					printf("l1 a1 a2 are: %f %f;",linksys.angle1,linksys.angle2);
					printf("a1_ini a2_ini are :%f %f;",linksys.angle1Ini,linksys.angle2Ini);
					printf("pos_ini are :%d %d;",motor[0].positionIni,motor[1].positionIni);
					printf("motor0 targetpositon is %d;motor1 targetpositon is %d;\n",motor[0].targetPosition,motor[1].targetPosition);
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
		if(rxCounter == 32&& ((mode == 5 || mode == 6)|| mode == 7))
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
//TUDO 写一个扫描行程的函数
	switch (order[ID])
	{
		case 0:
			break;

		case 1:
			motor[ID].targetVelocity = 100;
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
			motor[ID].targetVelocity = -100;
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


void txThread(int s)
{
    struct can_frame frame;
	frame.can_id = 0x200;
	frame.can_dlc = 8;
	int j;
	sendIMessage.data.resize(4);
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
		// TODO 如何计算发送电流
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
			//TUDO 思考一下这部分怎么写
				testLimit(0);
				testLimit(1);
				testLimit(2);
				testLimit(3);
				motor[0].positionIni = motor[0].limit_1;
				motor[1].positionIni = motor[1].limit_0;
				// if (order[4] == 1) {
				// 	controlP_calSendI_PPI(0);
				// 	controlP_calSendI_PPI(1);
				// 	controlP_calSendI_PPI(2);
				// 	controlP_calSendI_PPI(3);
				// };	
				break;

			case 7:
				testLimit(0);
				testLimit(1);
				motor[0].positionIni = motor[0].limit_1;
				motor[1].positionIni = motor[1].limit_0;
				if (order[2] == 1) {
					controlP_calSendI_PPI(0);
					controlP_calSendI_PPI(1);

				};	
				
				break;
			case 8:
				testLimit(0);
				testLimit(1);
				testLimit(2);
				testLimit(3);
				motor[0].positionIni = motor[0].limit_1;
				motor[1].positionIni = motor[1].limit_0;
				motor[2].positionIni = motor[2].limit_0;
				motor[3].positionIni = motor[3].limit_1;
				if (order[4] == 1) {
					controlP_calSendI_PPI(0);
					controlP_calSendI_PPI(1);
					controlP_calSendI_PPI(2);
					controlP_calSendI_PPI(3);
				};	

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
	motorInit();

	ros::init(argc,argv,"canTest6");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

    positionPub = n.advertise<std_msgs::Int32MultiArray>("position",100);
    velocityPub = n.advertise<std_msgs::Int32MultiArray>("velocity",100);
    IPub = n.advertise<std_msgs::Int32MultiArray>("I",100);
	sendIPub = n.advertise<std_msgs::Int32MultiArray>("sendI",100);
	targetPositionPub = n.advertise<std_msgs::Int32MultiArray>("targetPositionFromPosition",100);
	limitPub = n.advertise<std_msgs::Int16MultiArray>("limit",100);


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

	modeSub = n.subscribe("mode", 10, mode_Callback);
	orderSub = n.subscribe("order",10,order_Callback);

	while (ros::ok())
    {

		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
