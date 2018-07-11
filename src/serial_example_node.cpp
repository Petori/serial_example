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

serial::Serial ser;

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Publisher char_pub = nh.advertise<serial_example::serialChar>("pub_char", 1000);

    serial_example::serialChar dataSet;
    dataSet.serialChar.push_back(0x02);
    dataSet.serialChar.push_back(0x31);

    std::string set;
    std::string run;
    char *dataset;
    int length;

    set.clear();
    set = Motor1Set(1, 10);
    run = Motor1Run(false);
    set = set + run;

    length = set.length();
    dataset = (char *)malloc(sizeof(char) * length);

    for (int i = 0; i < length; i++)
    {
            dataset[i] = set.at(i);
            dataSet.serialChar.push_back(dataset[i]);
    }

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
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);

    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            char_pub.publish(dataSet);
        }
        loop_rate.sleep();

    }
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

