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
  std::vector<double> initialPoint = {};

  // Box路点
  std::vector<double> boxHighPoint1 = {};
  std::vector<double> boxHighPoint2 = {};
  std::vector<double> boxHighPoint3 = {};
  std::vector<double> boxHighPoint4 = {};
  std::vector<double> boxHighPoint5 = {};
  std::vector<double> boxHighPoint6 = {};
  std::vector<double> boxHighPoint7 = {};
  std::vector<double> boxHighPoint8 = {};
  std::vector<double> boxMidPoint1 = {};
  std::vector<double> boxMidPoint2 = {};
  std::vector<double> boxMidPoint3 = {};
  std::vector<double> boxMidPoint4 = {};
  std::vector<double> boxMidPoint5 = {};
  std::vector<double> boxMidPoint6 = {};
  std::vector<double> boxMidPoint7 = {};
  std::vector<double> boxMidPoint8 = {};
  std::vector<double> boxLowPoint1 = {};
  std::vector<double> boxLowPoint2 = {};
  std::vector<double> boxLowPoint3 = {};
  std::vector<double> boxLowPoint4 = {};
  std::vector<double> boxLowPoint5 = {};
  std::vector<double> boxLowPoint6 = {};
  std::vector<double> boxLowPoint7 = {};
  std::vector<double> boxLowPoint8 = {};

  //蒸馏水路点
  std::vector<double> waterHighPoint = {};

  std::vector<double> waterLowPoint = {};

  //待稀释溶液路点
  std::vector<double> blackHighPoint = {};

  std::vector<double> blackLowPoint = {};

  //工作版路点
  std::vector<double> wellHighPoint1 = {};
  std::vector<double> wellHighPoint2 = {};
  std::vector<double> wellMidPoint1 = {};
  std::vector<double> wellMidPoint2 = {};
  std::vector<double> wellLowPoint1 = {};
  std::vector<double> wellLowPoint2 = {};

  // medical waste路点
  std::vector<double> trashPoint = {};

  //*******************开始操作****************目标：倍比稀释**********************稀释倍数：4*********//

  //**********************************到达初始位置，准备开始工作***************************************/

  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //************取第一排吸头，然后吸取12*50=600微升蒸馏水到工作板第一排，扔掉吸头并回到初始点准备下一个操作******/

  //取第一排吸头
  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint1}, 0.5, 0.2);
  rtde_control.moveL({boxLowPoint1}, 0.5, 0.2);
  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};

    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第一排

  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint1}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);

  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);
  //*****************************************************************************************************/

  //*************取第二排吸头，然后吸取12*50=600微升蒸馏水到工作板第一排，扔掉吸头并回到初始点准备下一个操作************//

  //取第二排吸头
  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint1}, 0.5, 0.2);
  rtde_control.moveL({boxLowPoint1}, 0.5, 0.2);
  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);
  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第一排

  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint1}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);

  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //*******************取第三排吸头，然后吸取12*50=600微升蒸馏水到工作板第一排，扔掉吸头并回到初始点准备下一个操作**************/
  //取第三排吸头
  rtde_control.moveL({boxHighPoint3}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint3}, 0.5, 0.2);
  rtde_control.moveL({boxLowPoint3}, 0.5, 0.2);
  rtde_control.moveL({boxHighPoint3}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);
  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第一排

  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint1}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //**************取第四排吸头，然后吸取12*50=600微升蒸馏水到工作板第二排，扔掉吸头并回到初始点准备下一个操作
  //取第四排吸头
  rtde_control.moveL({boxHighPoint4}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint4}, 0.5, 0.2);
  rtde_control.moveL({boxLowPoint4}, 0.5, 0.2);
  rtde_control.moveL({boxHighPoint4}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);
  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);
  //排到工作板第二排
  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint2}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);
  //*****************************************************************************************************/

  //************************取第五排吸头，然后吸取12*50=600微升蒸馏水到工作板第二排，扔掉吸头并回到初始点准备下一个操作********//

  //取第五排吸头
  rtde_control.moveL({boxHighPoint5}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint5}, 0.5, 0.2);
  rtde_control.moveL({boxLowPoint5}, 0.5, 0.2);
  rtde_control.moveL({boxHighPoint5}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);
  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);
  //排到工作板第二排
  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint2}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************

  //********************取第六排吸头，然后吸取12*50=600微升蒸馏水到工作板第二排，扔掉吸头并回到初始点准备下一个操作***************//

  //取第六排吸头

  rtde_control.moveL({boxHighPoint6}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint6}, 0.5, 0.2);
  rtde_control.moveL({boxLowPoint6}, 0.5, 0.2);
  rtde_control.moveL({boxHighPoint6}, 0.5, 0.2);

  //吸取蒸馏水

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);
  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第二排

  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint2}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);

  //扔掉吸头
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //***********取第七排吸头，然后吸取12*50=600微升待稀释溶液到工作板第一排，扔掉吸头并回到初始点准备下一个操作***********//

  //取第七排吸头
  rtde_control.moveL({boxHighPoint7}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint7}, 0.5, 0.2);
  rtde_control.moveL({boxLowPoint7}, 0.5, 0.2);
  rtde_control.moveL({boxHighPoint7}, 0.5, 0.2);

  //吸取待稀释溶液
  rtde_control.moveL({blackHighPoint}, 0.5, 0.2);
  rtde_control.moveL({blackLowPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({blackHighPoint}, 0.5, 0.2);
  //排到工作板第一排

  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint1}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  //混合均匀
  rtde_control.moveL({wellLowPoint1}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  //扔掉吸头

  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************/

  //****取第八排吸头，然后吸取12*50=600微升蒸馏水到工作板第二排，扔掉吸头并回到初始点，倍比稀释完毕********************/

  //取第八排吸头
  rtde_control.moveL({boxHighPoint8}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint8}, 0.5, 0.2);
  rtde_control.moveL({boxLowPoint8}, 0.5, 0.2);
  rtde_control.moveL({boxHighPoint8}, 0.5, 0.2);

  //吸取待稀释溶液
  rtde_control.moveL({blackHighPoint}, 0.5, 0.2);
  rtde_control.moveL({blackLowPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }
  rtde_control.moveL({blackHighPoint}, 0.5, 0.2);
  //排到工作板第二排

  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint2}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  //混合均匀
  rtde_control.moveL({wellLowPoint2}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  rtde_control.moveL({wellHighPoint2}, 0.5, 0.2);
  //扔掉吸头

  rtde_control.moveL({trashPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));

    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
  }

  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************

  //****实验完成:)

  std::cout << "倍比稀释实验完成：）";

  //****************************************支线任务完成！************************************************//

  return 0;
}
