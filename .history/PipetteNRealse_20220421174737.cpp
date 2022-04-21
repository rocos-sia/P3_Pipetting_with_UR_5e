#include <stdio.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace ur_rtde;
using namespace std::chrono;

//*自定义校验和函数
char checkSum(unsigned char *sendBuffer)
{
  int sum = 0;

  for (size_t i = 2; i < 8; i++)
  {
    sum = sum + sendBuffer[i];
  }

  sendBuffer[8] = sum;
  sum = 0;
}

//************************************

int main(int argc, char *argv[])
{
  //** 串口初始化 **//
  boost::asio::io_service ioService;
  boost::asio::serial_port serialPort(ioService, "/dev/ttyUSB0");

  serialPort.set_option(boost::asio::serial_port::baud_rate(115200));
  serialPort.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  serialPort.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serialPort.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  serialPort.set_option(boost::asio::serial_port::character_size(8));

  unsigned char sendBuffer[] = {0x55, 0XAA, 0x04, 0x02, 0x03, 0x37, 0x00, 0x00, 0X58};

  //**-------------------------------**//

  //**************************************连接UR***********************************************************************//
  ur_rtde::RTDEControlInterface rtde_control("192.168.3.101");
  ur_rtde::RTDEReceiveInterface rtde_receive("192.168.3.101");
  ur_rtde::RTDEIOInterface rtde_io("192.168.3.101");
  //*---------------------------------------------------------------------------------------------------------------*//

  //****************定义路点*************************/

  //初始点
  std::vector<double> initialPoint = {-0.43212,-0.15122,0.3796,2.294,2.089,0.057};

  // Box路点
  std::vector<double> boxHighPoint1 = {0.66242,-0.23972,0.30066,2.290,2.090,0.058};
  std::vector<double> boxHighPoint2 = {0.67166,-0.24022,0.30066,2.294,2.089,0.057};
  std::vector<double> boxHighPoint3 = {0.68058,-0.24096,0.30066,2.294,2.089,0.057};
  std::vector<double> boxHighPoint4 = {0.68956,-0.24156,0.30066,2.294,2.089,0.057};
  std::vector<double> boxHighPoint5 = {0.69836,-0.24276,0.30066,2.294,2.089,0.057};
  std::vector<double> boxHighPoint6 = {0.70810,-0.24350,0.30066,2.294,2.089,0.057};
  std::vector<double> boxHighPoint7 = {0.71688,-0.24446,0.30066,2.294,2.089,0.057};
  std::vector<double> boxHighPoint8 = {0.72552,-0.24532,0.30066,2.294,2.089,0.057};
  std::vector<double> boxMidPoint1  = {0.66242,-0.23972,0.22040,2.290,2.090,0.058};
  std::vector<double> boxMidPoint2  = {0.67166,-0.24022,0.22216,2.294,2.089,0.057};
  std::vector<double> boxMidPoint3  = {0.68058,-0.24096,0.22116,2.294,2.089,0.057};
  std::vector<double> boxMidPoint4  = {0.68956,-0.24156,0.22212,2.294,2.089,0.057};
  std::vector<double> boxMidPoint5  = {0.69836,-0.24276,0.22276,2.294,2.089,0.057};
  std::vector<double> boxMidPoint6  = {0.70810,-0.24350,0.22316,2.294,2.089,0.057};
  std::vector<double> boxMidPoint7  = {0.71688,-0.24446,0.22146,2.294,2.089,0.057};
  std::vector<double> boxMidPoint8  = {0.72552,-0.24532,0.22210,2.294,2.089,0.057};


  //蒸馏水路点
  std::vector<double> waterHighPoint = {0.55272,0.00036,0.30066,2.294,2.089,0.057};

  std::vector<double> waterLowPoint = {0.55272,0.000036,0.20266,2.294,2.089,0.057};

  //待稀释溶液路点
  std::vector<double> blackHighPoint = {0.52648,-0.19666,0.30066,2.294,2.089,0.057};

  std::vector<double> blackLowPoint = {0.52646,-0.19666,0.20266,2.294,2.089,0.057};

  //工作版路点
  std::vector<double> wellHighPoint1 = {0.66886,-0.00876,0.26666,2.294,2.089,0.057};
  std::vector<double> wellHighPoint2 = {0.67631,-0.00876,0.26666,2.294,2.089,0.057};
  std::vector<double> wellMidPoint1  = {0.66888,-0.00876,0.23888,2.294,2.089,0.057};
  std::vector<double> wellMidPoint2  = {0.67631,-0.00876,0.23672,2.294,2.089,0.057};
  std::vector<double> wellLowPoint1  = {0.66888,-0.00876,0.21666,2.294,2.089,0.057};
  std::vector<double> wellLowPoint2  = {0.67631,-0.00876,0.21666,2.294,2.089,0.057};

  // medical waste路点
  std::vector<double> trashPoint = {0.56286,-0.41600,0.26066,2.294,2.089,0.057};

  //*******************开始操作****************目标：倍比稀释**********************稀释倍数：4*********//

  //**********************************到达初始位置，准备开始工作***************************************/

  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //************取第一排吸头，然后吸取12*50=600微升蒸馏水到工作板第一排，扔掉吸头并回到初始点准备下一个操作******/

  //取第一排吸头
  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint1}, 0.5, 0.2);
  // Parameters
  std::vector<double> task_frame    = {0, 0, 0, 0, 0, 0};
  std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
  std::vector<double> wrench_down   = {0, 0, -20, 0, 0, 0};
  std::vector<double> wrench_up     = {0, 0, 1, 0, 0, 0};
  int    force_type = 2;
  double dt         = 1.0/500;  // 2ms
  std::vector<double> limits = {1, 1, 1, 1, 1, 1};



  // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();


  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);    
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }

  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);


  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);   
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9)); 
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第一排

  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint1}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);

  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
      serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);
  //*****************************************************************************************************/

  //*************取第二排吸头，然后吸取12*50=600微升蒸馏水到工作板第一排，扔掉吸头并回到初始点准备下一个操作************//

  //取第二排吸头
  rtde_control.moveL({boxHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint2}, 0.5, 0.2);




  // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();

  rtde_control.moveL({boxHighPoint2}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
     serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
   

 

  }
  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第一排

  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint1}, 0.5, 0.2);
  {
sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);

  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
    {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //*******************取第三排吸头，然后吸取12*50=600微升蒸馏水到工作板第一排，扔掉吸头并回到初始点准备下一个操作**************/
  //取第三排吸头
  rtde_control.moveL({boxHighPoint3}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint3}, 0.5, 0.2);
   // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();


  rtde_control.moveL({boxHighPoint3}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第一排

  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint1}, 0.5, 0.2);
  {
 sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
   {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //**************取第四排吸头，然后吸取12*50=600微升蒸馏水到工作板第二排，扔掉吸头并回到初始点准备下一个操作
  //取第四排吸头
  rtde_control.moveL({boxHighPoint4}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint4}, 0.5, 0.2);

  // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();

  rtde_control.moveL({boxHighPoint4}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第二排
  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint2}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
   {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);
  //*****************************************************************************************************/

  //************************取第五排吸头，然后吸取12*50=600微升蒸馏水到工作板第二排，扔掉吸头并回到初始点准备下一个操作********//

  //取第五排吸头
  rtde_control.moveL({boxHighPoint5}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint5}, 0.5, 0.2);

  // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();

  rtde_control.moveL({boxHighPoint5}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第二排
  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint2}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************

  //********************取第六排吸头，然后吸取12*50=600微升蒸馏水到工作板第二排，扔掉吸头并回到初始点准备下一个操作***************//

  //取第六排吸头

  rtde_control.moveL({boxHighPoint6}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint6}, 0.5, 0.2);
 
  // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();

  rtde_control.moveL({boxHighPoint6}, 0.5, 0.2);

  //吸取蒸馏水

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);


  //排到工作板第二排

  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint2}, 0.5, 0.2);

  {
 sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);

  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //***********取第七排吸头，然后吸取12*50=600微升待稀释溶液到工作板第一排，扔掉吸头并回到初始点准备下一个操作***********//

  //取第七排吸头
  rtde_control.moveL({boxHighPoint7}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint7}, 0.5, 0.2);
  
  // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();

  rtde_control.moveL({boxHighPoint7}, 0.5, 0.2);

  //吸取待稀释溶液
  rtde_control.moveL({blackHighPoint}, 0.5, 0.2);
   {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  rtde_control.moveL({blackLowPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({blackHighPoint}, 0.5, 0.2);
  //排到工作板第一排

  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint1}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  //混合均匀
  rtde_control.moveL({wellLowPoint1}, 0.5, 0.2);
  
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
     serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
   

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

      sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  //扔掉吸头

  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //****取第八排吸头，然后吸取12*50=600微升蒸馏水到工作板第二排，扔掉吸头并回到初始点，倍比稀释完毕********************/

  //取第八排吸头
  rtde_control.moveL({boxHighPoint8}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint8}, 0.5, 0.2);

  // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();

  rtde_control.moveL({boxHighPoint8}, 0.5, 0.2);

  //吸取待稀释溶液
  rtde_control.moveL({blackHighPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  rtde_control.moveL({blackLowPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  rtde_control.moveL({blackHighPoint}, 0.5, 0.2);
  //排到工作板第二排

  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint2}, 0.5, 0.2);

  {
   sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  //混合均匀
  rtde_control.moveL({wellLowPoint2}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

      sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  //扔掉吸头

  rtde_control.moveL({trashPoint}, 0.5, 0.2);

 {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************

  //****实验完成:)

  std::cout << "倍比稀释实验完成：）";

  //****************************************支线任务完成！************************************************//

  return 0;
}
