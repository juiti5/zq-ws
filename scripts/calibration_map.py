#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
import yaml
import xml.etree.ElementTree as ET
import getpass
yaml_name = sys.argv[1]
#yaml_name = "/home/lrd/map/000.trans.yaml"
launch_name = "/home/autolabor/zhongqi_ws/src/position_optimization/launch/trans.launch"
map_server_name = "/home/autolabor/zhongqi_ws/src/zhongqi_navigation/launch/zhongqi_navigation.launch"
user_name = getpass.getuser()
with open(yaml_name, 'r') as rf:
    data = rf.read()
    data_y = yaml.load(data)
    
#    print(type(data_y))
#    print(data_y['x'])
x = "%.40r" % (data_y['x'])
y = "%.40r" % (data_y['y'])
yaw = "%.40r" % (data_y['yaw'])
    
tree = ET.parse(launch_name)
root = tree.getroot()
tree1 = ET.parse(map_server_name)
root1 = tree1.getroot() 
 
for param in root.iter('param'):
    name = param.get('name')
    print(name)
    
    if name == 'trans_x':
        param.set('value',x)
        print(x)
    if name == 'trans_y':
        param.set('value',y)
        print(y)
    if name == 'trans_yaw':
        param.set('value',yaw)
        print(yaw)
        
for child in root1:
    name1 = child.get('name')
   
    if name1 == 'map_server':
#         name2 = child.get('args')
         map_name = yaml_name.split('/')[-1].split('.')[0]
         path = '/home/' + user_name + '/map/' + map_name + '.yaml'
         child.set('args',path)
#         print(path)        
#         print(map_name)

tree.write(launch_name)
tree1.write(map_server_name)
