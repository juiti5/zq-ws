#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
import json

try:
    rf = open("/home/lrd/zq_data/transfrom", 'r')
    data = rf.read()
    
    data = data.replace("'", '"')
    data_t = json.loads(data)
    
    str1 = 'x: ' + str(data_t['x']) + '\r'
    str2 = 'y: ' + str(data_t['y']) + '\r'
    str3 = 'yaw: ' + str(data_t['yaw']) + '\r' 
       
    list = [str1,str2,str3]
finally:
    if rf:
        rf.close() 
             
with open(sys.argv[1], 'wb') as wf:
    wf.writelines(list)
    wf.flush()
