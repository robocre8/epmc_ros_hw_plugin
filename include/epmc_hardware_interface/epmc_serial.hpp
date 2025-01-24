#ifndef EPMC_SERIAL_HPP
#define EPMC_SERIAL_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>

#include <chrono>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class EPMC
{

public:
  EPMC() = default;

  void connect(const std::string &serial_device, int32_t baud_rate = 115200, int32_t timeout_ms = 100)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  bool sendTargetVel(float valA = 0.0, float valB = 0.0)
  {
    return send("/tag", valA, valB);
  }

  bool sendPwm(int valA = 0, int valB = 0)
  {
    return send("/pwm", valA, valB);
  }

  bool setCmdTimeout(int timeout_ms)
  {
    return send("/timeout", timeout_ms, 0);
  }

  void getCmdTimeout(int &timeout_ms)
  {
    get("/timeout");

    timeout_ms = val[0];

    val[0] = 0.0;
    val[1] = 0.0;
  }

  void getMotorsPos(float &angPosA, float &angPosB)
  {
    get("/pos");

    angPosA = val[0];
    angPosB = val[1];

    val[0] = 0.0;
    val[1] = 0.0;
  }

  void getMotorsVel(float &angVelA, float &angVelB)
  {
    get("/vel");

    angVelA = val[0];
    angVelB = val[1];

    val[0] = 0.0;
    val[1] = 0.0;
  }

  void getMotorAData(float &angPos, float &angVel)
  {
    get("/dataA");

    angPos = val[0];
    angVel = val[1];

    val[0] = 0.0;
    val[1] = 0.0;
  }

  void getMotorBData(float &angPos, float &angVel)
  {
    get("/dataB");

    angPos = val[0];
    angVel = val[1];

    val[0] = 0.0;
    val[1] = 0.0;
  }

  void getMotorAMaxVel(float &maxVel)
  {
    get("/maxVelA");

    maxVel = val[0];

    val[0] = 0.0;
    val[1] = 0.0;
  }

  void getMotorBMaxVel(float &maxVel)
  {
    get("/maxVelB");

    maxVel = val[0];

    val[0] = 0.0;
    val[1] = 0.0;
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
  float val[2];

  std::string send_msg(const std::string &msg_to_send)
  {
    auto prev_time = std::chrono::system_clock::now();
    std::chrono::duration<double> duration;

    std::string response = "";

    serial_conn_.FlushIOBuffers(); // Just in case

    while (response == "")
    {
      try
      {

        try
        {
          serial_conn_.Write(msg_to_send);
          serial_conn_.ReadLine(response, '\n', timeout_ms_);
          duration = (std::chrono::system_clock::now() - prev_time);
        }
        catch (const LibSerial::ReadTimeout &)
        {
          continue;
        }

        duration = (std::chrono::system_clock::now() - prev_time);
        if (duration.count() > 2.0)
        {
          throw duration.count();
        }
      }
      catch (double x)
      {
        std::cerr << "Error getting response from arduino nano, wasted much time \n";
      }
    }

    return response;
  }

  bool send(std::string cmd_route, float valA, float valB)
  {
    std::stringstream msg_stream;
    msg_stream << cmd_route << "," << valA << "," << valB;

    std::string res = send_msg(msg_stream.str());

    int data = std::stoi(res);
    if (data)
      return true;
    else
      return false;
  }

  void get(std::string cmd_route)
  {
    std::string res = send_msg(cmd_route);

    std::stringstream ss(res);
    std::vector<std::string> v;

    while (ss.good())
    {
      std::string substr;
      getline(ss, substr, ',');
      v.push_back(substr);
    }

    for (size_t i = 0; i < v.size(); i++)
    {
      val[i] = std::atof(v[i].c_str());
    }
  }
};

#endif