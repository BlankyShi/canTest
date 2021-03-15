#include "ros/ros.h"

#include <chrono>
#include <thread>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"


ros::Subscriber pidSUb;
float kp,ki,K,tP,tV;
int32_t err,err_V,err_V_sum;//偏差信号和偏差信号的累加和

int16_t send_I;


ros::Publisher echoPub;//1.这个是用来干啥的
std_msgs::String echoMsg;//2.这个又是用来干啥的
ros::Publisher motorAnglePub;//电机角度发布
ros::Publisher vel_pub;


struct can_frame rxframe;//3.这个结构体还是蒙蔽，

int rxCounter = 0;//4.计数什么用的
int trErrorCounter = 0;//5.计数什么用的
std_msgs::Float32 velData;
int16_t angle, angle_last, angle_d, vel, R, I, temp, init;//6.vel R I temp init




//pidsub的回调函数
// void pidsubCallback(const std_msgs::Float32MultiArray::ConstPtr pidValue){ //const std_msgs::Float32MultiArray::ConstPtr&
//   kp=pidValue->data[0];
//   ki=pidValue->data[1];
//   kd=pidValue->data[2];
//   tP=pidValue->data[3];
//   ROS_INFO("now pid value is %f %f %f %f", kp,ki,kd,tP);
// }
void pidsubCallback(std_msgs::Float32MultiArray arrayTestMsg)
{

  K = arrayTestMsg.data[0];
  tP = arrayTestMsg.data[1];
  ROS_INFO("now pid value is %f %f", K, tP);
}


void rxThread(int s) 
{
  init = 0;
  angle_last = 0;
  R=0;
  for (int i = 0;; i++) 
  {
    int nbytes;
    struct can_frame rxframe;
    if (1) 
    {
      nbytes = read(s, &rxframe, sizeof(struct can_frame));
      //  mutex.unlock();
      if (nbytes < 0) 
      {
        perror("Read");
        break;
      }
      printf("0x%03X [%d] ", rxframe.can_id, rxframe.can_dlc);
      // for (i = 0; i < rxframe.can_dlc; i++)
      //   printf("%02X ", rxframe.data[i]);
      // printf("\r\n");

      angle = ((uint16_t)rxframe.data[0] << 8) + (uint16_t)rxframe.data[1];
      vel = ((uint16_t)rxframe.data[2] << 8) + (uint16_t)rxframe.data[3];
      I = ((uint16_t)rxframe.data[4] << 8) + (uint16_t)rxframe.data[5];
      temp = (int8_t)(uint16_t)rxframe.data[6];
      angle_d = angle - angle_last;
      //计算圈数
      if(angle_d<-4000){
        R++;
      }else if (angle_d>4000) {
      R--;
      }
      angle_last = angle;
      printf("angle is %d ,vel is %d, I is %d, tem is %d, R is %d", angle, vel, I,temp,R);
      rxCounter++;
      printf("   the %d  rx\n", rxCounter);
    } else {
    }
    // printf("    RX    ");
    printf("err = %d ",err);
    // printf("err_V = %d ",err_V);
    printf("发出去的电流：send_I = %ld  ",send_I);
    printf("angle_d = %d  ",angle_d);
    printf("err_V_sum = %d\n",err_V_sum);

    printf("K = %f  ",K);
    printf("tP = %f\n  ",tP);
    printf("vel = %hd\n",vel);
    velData.data = vel;
    vel_pub.publish(velData);

    std::this_thread::sleep_for(
    std::chrono::nanoseconds(100000)); // 10 6600 9 7200   8  7700
  }
}

void txThread(int s) {

  std_msgs::Float64MultiArray motorAngle;
  struct can_frame frame;
  for (int i = 0;; i++) { // i < 50000
    frame.can_id = 0x200;
    frame.can_dlc = 8;
    for (int j = 0; j < 8; j++) {
      frame.data[j] = 0x00;
    }

//下面是位置环的东西，现在先用速度环
    err = tP - (angle+R*8192);
    // err_sum = err_sum + err;
    tV = K/1000*err;
    err_V = tV - vel;
    err_V_sum = err_V + err_V_sum;

    //int16_t I = kp*err;这是最原始的k的位置环
    send_I = kp*err_V + ki*err_V_sum;
    //为电流添加限幅
    if (send_I > 7000){
      send_I = 7000;
    }
    if (send_I <-7000)
    {
      send_I = -7000;
    }
    // send_I = kp*err + ki*err_sum + kd*angle_d;
    motorAngle.data.resize(2);
    frame.data[0] = send_I >> 8;
    frame.data[1] = send_I >> 0;
    
    motorAngle.data[0]=angle;
    motorAngle.data[1]=1122;
    motorAnglePub.publish(motorAngle);
    ros::spinOnce();

    int nbytes;
    if (1) {
      nbytes = write(s, &frame, sizeof(struct can_frame));
      if (nbytes == -1) {
        printf("send error\n");
        trErrorCounter++;
      }
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(1100000)); // 10 6600 9 7200   8  7700
  }
  echoPub.publish(echoMsg);
  printf("tx over \n");
  
}



int main(int argc, char** argv) {

  tP = 0;
  tV = 0;
  err_V_sum = 0;
  kp = 23;
  ki = 0.001;
  K = 0;

  ros::init(argc, argv, "dji_control_test");
  ros::NodeHandle node;


  pidSUb=node.subscribe<std_msgs::Float32MultiArray>("arraryTest",100,pidsubCallback);

  echoPub=node.advertise<std_msgs::String>("dji_echo",10);
  motorAnglePub=node.advertise<std_msgs::Float64MultiArray>("motorAngle",10);
  vel_pub = node.advertise<std_msgs::Float32>("motorVel",10);


  echoMsg.data=std::string("hello !!");
  echoPub.publish(echoMsg);

  ros::Rate loop_rate(10);

	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	angle = 0;
  send_I = 0;

	const char *ifname = "can0";

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}
//这里是什么意思，类似于回调函数吗？有消息传送的话就直接调用吗？
	std::thread canTx(txThread, s);
	std::thread canRx(rxThread, s);


	while (ros::ok())
  {

		echoPub.publish(echoMsg);
		ros::spinOnce();
		loop_rate.sleep();
	}

  // canRx.join();
  // canTx.join();

  printf("send over\n");
  printf(" trErrorCounter is  %d  \r\n", trErrorCounter);
  printf("the final number rx is  %d  \r\n", rxCounter);


  struct can_frame frame_over;

  for (int i = 0;; i++) 
  { // i < 50000
	frame.can_id = 0x200;
	frame.can_dlc = 8;
	for (int j = 0; j < 8; j++) {
		frame.data[j] = 0x00;
    }
  }
// TmotorTest(frame);

    int nbytes2;
      //  mutex.lock();
      nbytes2 = write(s, &frame_over, sizeof(struct can_frame));

  return 0;
}
