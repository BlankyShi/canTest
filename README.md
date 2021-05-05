
# 打开can口(如果有需要，运行代码文件之前需要打开can口)

sudo ip link set can0 type can bitrate 1000000  
sudo ip link set up can0  

# 关闭roscore（因为通过ssh进行调试，会存在roscore难以关闭的情况）

killall -9 roscore  
killall -9 rosmaster  
roscore  

# 请不要在运行代码时按键CREL + Z；可以按ctrl + c来停止代码

---------------------

## canTest9

    mode9:在ubuntu端生成一个正弦目标位置信号（测试单边）  
    创建publisher{xypub和xypub2}用来发布笛卡尔坐标系下目标位置和当前位置对应的末端位置  
    设置模式9的轨迹选项,通过topic{tra}来选择轨迹  
    0单点  
    1上下运动，目标轨迹是正弦信号  
    2左右运动，实现拍手，目标轨迹是正弦信号  
    创建接受ctrl c 信号的函数signalCallback  

## canTest8

    把两个连杆分别存储到两个linsys里面  
    把正逆运动学解算部分写到ubuntu端  
    在matlab端进行目标位置发送  
    优化扫描行程函数，避免奇异突变运动  
    更改模式：模式6之后的如下  
    模式6:扫描得到电机行程  
    模式7:扫描得到电机行程之后，通过topic{targetAngle}对单边进行逆运动学控制  
    模式8：扫描得到电机行程之后，通过topic{targetAngle2,targetPositionXY}对双边电机进行控制  
    存在问题：电机会跳动,电机一直处于位置环控制模式，而且目标位置一直没有变化，就很神奇  

## canTest7

    测试双边的正逆运动学  
    在matlab端进行正逆运动学解算  
    把两个连杆集成到一个linksys里面  

## canTest6

    测试单边的正逆运动学  
    在matlab端进行正逆运动学解算  

## canTest5

    尝试建立二连杆结构体，并尝试建立正逆运动学方程（未成功）  

## canTest4

    整理代码文件，增加模式功能，可以在运行时选择不同的模式  

    mode；初始mode为0  
    default:更改mode参数为0  
    0:只接受电机传来的数据信息，四个电机的发送电流全为0
    1:速度环PI参数调节模式，初始目标速度为0，通过topic{targetVelocity,PI}来进行调参（单个电机，ID为0）  
    2:位置环P参数调节模式，初始位置为0，通过topic{targetPosition,PI}该模式下使用上一步测试得到的速度环PI参数/单个电机，ID为0）  
    3:速度环控制模式，初始目标速度为0；通过topic{targetVelocity}设置目标速度（两个个电机，ID为0）  
    4:位置环控制模式，初始目标位置为0；通过topic{targetPosition}设置目标位置（单个电机，ID为0）  
    5:位置环控制模式，初始获得电机当前位置设定为目标位置；通过topic{targetPositionALL}设定目标位置（四个电机，ID为0,1，2，3）  
    6:扫描行程模式，得到四个电机的行程，建立相对坐标系；然后执行mode0  
    7:暂定  
    完成扫描得到电机行程极限数据  

## canTest3

    读取四个电机当前位置信息，然后把位置信息设定位目标位置，进行位置环控制  

## canTest2

    单个电机速度环 （PI控制）合适的PI参数  
    然后在速度环外放置位置环进行位置控制  
    在matlab端发送控制信息  
    实验结果：PPI 15,13,6  

## canTest1

    读取电机数据  
