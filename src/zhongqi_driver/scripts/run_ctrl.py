#!/usr/bin/env python
#coding: utf-8
import os
devicepath = '/dev'
def checkDevice():
    os.chdir(devicepath)
    devicelist = []
    for i in os.listdir(os.getcwd()):
        if i == 'ucar' or 'uimu' or 'ucamera' or 'ufront' or 'urear' or 'ulidar_front' or 'ulidar_rear':
            devicelist.append(i)
    if 'ucar' in devicelist:
      print ('car_ok!')
    else:
      print ('car_disconnect!')    
    if 'uimu' in devicelist:
      print ('imu_ok!')
    else:
      print ('imu_disconnect!')     
    if 'ucamera' in devicelist:
      print ('camera_ok!')
    else:
      print ('camera_disconnect!') 
    if 'ufront' in devicelist:
      print ('front_ok!')
    else:
      print ('front_disconnect!')       
    if 'urear' in devicelist:
      print ('rear_ok!')
    else:
      print ('rear_disconnect!')       
    if 'ulidar_front' in devicelist:
      print ('lidar_front_ok!')
    else:
      print ('lidar_front_disconnect!')       
    if 'ulidar_rear' in devicelist:
      print ('lidar_rear_ok!')
    else:
      print ('lidar_rear_disconnect!')        
    
                  
checkDevice()
