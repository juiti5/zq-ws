#!/bin/bash
sleep 1m

if [ ! -e "/dev/ucar" ]; then
  zenity --error --text="底层连接失败，请检查机器人是否连接至上位机。"
  exit 0
fi

if [ ! -e "/dev/ulidar_front" ]; then
  zenity --error --text="单线激光雷达（前）连接失败，请检查雷达是否连接至上位机。"
  exit 0
fi

if [ ! -e "/dev/ulidar_rear" ]; then
  zenity --error --text="单线激光雷达（后）连接失败，请检查雷达是否连接至上位机>。"
  exit 0
fi

if [ ! -e "/dev/ufront" ]; then
  zenity --error --text="定位标签（前）连接失败，请检查标签是否连接至上位机>"
  exit 0
fi

if [ ! -e "/dev/urear" ]; then
  zenity --error --text="定位标签（后）连接失败，请检查标签是否连接至上位机>"
  exit 0
fi

if [ ! -e "/dev/ucamera" ]; then
  zenity --error --text="摄像头连接失败，请检查摄像头是否连接至上位机>"
  exit 0
fi

if [ ! -e "/dev/uimu" ]; then
  zenity --error --text="惯导连接失败，请检查惯导是否连接至上位机>"
  exit 0
fi


roslaunch zhongqi_driver start.launch

