/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include "serial_example/serialChar.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <vector>
#include <std_msgs/Int8.h>
#include <ostream>
#include <string>
#include <unistd.h>

std::string Motor1Set(unsigned int _HZ, unsigned int _Pos);
std::string Motor1Stop();
std::string Motor1Run(bool _Dir);
std::string GetASCII(unsigned int _One, unsigned int _Bit4);
char OneToASCII(int _One);
std::string Motor2Set(unsigned int _HZ, unsigned int _Pos);
std::string Motor2Stop();
std::string Motor2Run(bool _Dir);

const char AddrMotor1HZ[4] = { 0x31, 0x30, 0x43, 0x38 };
const char AddrMotor2HZ[4] = { 0x31, 0x31, 0x39, 0x30 };
const char AddrMotor1EN[4] = { 0x31, 0x32, 0x35, 0x38 };
const char AddrMotor2EN[4] = { 0x31, 0x33, 0x32, 0x30 };

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    serial::Serial ser;

    std::string set1;
    std::string run1;
    std::string set2;
    std::string run2;
    std::string stop1;
    std::string stop2;

    set1 = Motor1Set(200000, 2000000);
    // true 靠近，false 远离
    run1 = Motor1Run(true);
    set2 = Motor2Set(200000, 2000000);
    run2 = Motor2Run(false);
    stop1 = Motor1Stop();
    stop2 = Motor2Stop();

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port is Opened");
    }else{
        return -1;
    }
    ser.write(set1);
    sleep(1);
    ser.write(run1);
    sleep(1);
    ser.write(stop1);
    sleep(1);
//    ser.write(set2);
//    sleep(1);
//    ser.write(run2);
//    sleep(3);
//    ser.write(stop2);
    ROS_INFO_STREAM("Move finished.");
}

std::string Motor1Set(unsigned int _HZ, unsigned int _Pos)
{
        std::string _PLCFram;
        _PLCFram.clear();
        _PLCFram = _PLCFram + char(0x02);
        _PLCFram = _PLCFram + char(0x31);

        for (int ii = 0; ii<4; ii++) _PLCFram = _PLCFram + AddrMotor1HZ[ii];

        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x38);

        _PLCFram = _PLCFram + GetASCII(_HZ, 4);
        _PLCFram = _PLCFram + GetASCII(_Pos, 4);

        _PLCFram = _PLCFram + char(0x03);

        unsigned char _CharData = 0;
        for (int i = 1; i<_PLCFram.size(); i++)
        {
                _CharData += _PLCFram[i];
        }
        _PLCFram = _PLCFram + GetASCII(_CharData, 1);

        return _PLCFram;
}

std::string Motor1Stop()
{
        std::string _PLCFram;
        _PLCFram.clear();
        _PLCFram = _PLCFram + char(0x02);
        _PLCFram = _PLCFram + char(0x31);

        for (int ii = 0; ii<4; ii++) _PLCFram = _PLCFram + AddrMotor1EN[ii];

        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x32);

        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x30);

        _PLCFram = _PLCFram + char(0x03);

        unsigned char _CharData = 0;
        for (int i = 1; i<_PLCFram.size(); i++)
        {
                _CharData += _PLCFram[i];
        }
        _PLCFram = _PLCFram + GetASCII(_CharData, 1);

        return _PLCFram;
}

std::string Motor1Run(bool _Dir)
{
        std::string _PLCFram;
        _PLCFram.clear();
        _PLCFram = _PLCFram + char(0x02);
        _PLCFram = _PLCFram + char(0x31);

        for (int ii = 0; ii<4; ii++) _PLCFram = _PLCFram + AddrMotor1EN[ii];

        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x32);

        _PLCFram = _PLCFram + char(0x30);
        if (_Dir) _PLCFram = _PLCFram + char(0x32);// I guess the two ASCII code mean the direction.
        else _PLCFram = _PLCFram + char(0x31);
        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x30);

        _PLCFram = _PLCFram + char(0x03);

        unsigned char _CharData = 0;
        for (int i = 1; i<_PLCFram.size(); i++)
        {
                _CharData += _PLCFram[i];
        }
        _PLCFram = _PLCFram + GetASCII(_CharData, 1);

        return _PLCFram;
}

std::string GetASCII(unsigned int _One, unsigned int _Bit4)
{
        std::string _Result;
        _Result.clear();
        unsigned char _CharData = 0;
        for (unsigned int ii = 0; ii<_Bit4; ii++)
        {
                _CharData = _One;
                _Result = _Result + OneToASCII(_CharData / 16);
                _Result = _Result + OneToASCII(_CharData % 16);
                _One = _One >> 8;
        }
        return _Result;
}

char OneToASCII(int _One)
{
        if (_One >= 0 && _One<10) return _One + 0x30;
        if (_One<16 && _One>9) return _One - 10 + 0x41;
        else return 0;
}

std::string Motor2Set(unsigned int _HZ, unsigned int _Pos)
{
        std::string _PLCFram;
        _PLCFram.clear();
        _PLCFram = _PLCFram + char(0x02);
        _PLCFram = _PLCFram + char(0x31);

        for (int ii = 0; ii<4; ii++) _PLCFram = _PLCFram + AddrMotor2HZ[ii];

        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x38);

        _PLCFram = _PLCFram + GetASCII(_HZ, 4);
        _PLCFram = _PLCFram + GetASCII(_Pos, 4);

        _PLCFram = _PLCFram + char(0x03);

        unsigned char _CharData = 0;
        for (int i = 1; i<_PLCFram.size(); i++)
        {
                _CharData += _PLCFram[i];
        }
        _PLCFram = _PLCFram + GetASCII(_CharData, 1);

        return _PLCFram;
}

std::string Motor2Stop()
{
        std::string _PLCFram;
        _PLCFram.clear();
        _PLCFram = _PLCFram + char(0x02);
        _PLCFram = _PLCFram + char(0x31);

        for (int ii = 0; ii<4; ii++) _PLCFram = _PLCFram + AddrMotor2EN[ii];

        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x32);

        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x30);

        _PLCFram = _PLCFram + char(0x03);

        unsigned char _CharData = 0;
        for (int i = 1; i<_PLCFram.size(); i++)
        {
                _CharData += _PLCFram[i];
        }
        _PLCFram = _PLCFram + GetASCII(_CharData, 1);

        return _PLCFram;
}

std::string Motor2Run(bool _Dir)
{
        std::string _PLCFram;
        _PLCFram.clear();
        _PLCFram = _PLCFram + char(0x02);
        _PLCFram = _PLCFram + char(0x31);

        for (int ii = 0; ii<4; ii++) _PLCFram = _PLCFram + AddrMotor2EN[ii];

        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x32);

        _PLCFram = _PLCFram + char(0x30);
        if (_Dir) _PLCFram = _PLCFram + char(0x32);// I guess the two ASCII code mean the direction.
        else _PLCFram = _PLCFram + char(0x31);
        _PLCFram = _PLCFram + char(0x30);
        _PLCFram = _PLCFram + char(0x30);

        _PLCFram = _PLCFram + char(0x03);

        unsigned char _CharData = 0;
        for (int i = 1; i<_PLCFram.size(); i++)
        {
                _CharData += _PLCFram[i];
        }
        _PLCFram = _PLCFram + GetASCII(_CharData, 1);

        return _PLCFram;
}
