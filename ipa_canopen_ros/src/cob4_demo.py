#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2013 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS package name: 
#
# \author
# Author: Thiago de Freitas Oliveira Araujo, 
# email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
# \author
# Supervised by: Thiago de Freitas Oliveira Araujo, email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
#
# \date Date of creation: January 2014
#
# \brief
# Demo cob4 torso
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################
import roslib; 
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
from ipa_canopen_ros.srv import *

class cob4_demo:
    
    def __init__(self):
       
        rospy.wait_for_service('/ur_connector_controller/ppmode')
        self.ids = [1,2,3]
        self.vels = [400.0, 90000.0,70000.0]
        self.available_poses = [[16000.0, 20000.0,1000.0],[28000.0, 2700000.0,2000000.0]]


    def move_torso(self):
    
        for pose in self.available_poses:    
            try:
                ppmode_srv = rospy.ServiceProxy('/ur_connector_controller/ppmode', PPMode)
                resp1 = ppmode_srv(self.vels, pose, self.ids)
                rospy.sleep(45.)
                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        return resp1.target_reached
        
if __name__=="__main__":

    rospy.init_node("cob4_demo")
    cob4 = cob4_demo()
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        cob4.move_torso()
        r.sleep()

