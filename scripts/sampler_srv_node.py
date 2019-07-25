#!/usr/bin/env python

from water-sampler-ros.srv import *
import rospy
from PumpControl import Sampler
import time

# Initialize pumps object
pumps = Sampler()

def handle_fill_pump(req):
    rospy.loginfo("[Pump service node] Filling pump number %d" % req.number)
    result = pumps.fill_pump(req.number, req.volume)
    
    while True:
        pump_done = pumps.check_pump_state(req.number)
        if pump_done == 0:
            rospy.loginfo("[Pump service node] Filling pump %d complete!" % req.number)
            pumps.sleep_pump(req.number)
            break
        elif pump_done == 1:
            time.sleep(1)
        elif pump_done == -1:
            rospy.logerr("[Pump service node] Error verifying pump %d state." % req.number)
            pass
        else:            
            break

    return FillPumpResponse(result)

def handle_check_pump(req):
    rospy.loginfo("[Pumps service node] Checking state of pump %d" % req.number)
    result = pumps.check_pump_state(req.number)
    return CheckPumpResponse(result)

def handle_empty_pump(req):
    rospy.loginfo("[Pumps service node] Clearing volume of pump %d" % req.number)
    result = pumps.empty_pump(req.number)
    return EmptyPumpResponse(result)

def handle_stop_pump(req):
    rospy.loginfo("[Pumps service node] Stopping pump %d" % req.number)
    result = pumps.stop_pump(req.number)
    pumps.sleep_pump(req.number)
    return StopPumpResponse(result)

def advServices():
    rospy.init_node('pump_srv_node', anonymous=True)
    fillpumpsrv = rospy.Service('sampler/fill_pump', FillPump, handle_fill_pump)
    checkstatesrv = rospy.Service('sampler/check_pump_state', CheckPump, handle_check_pump)
    emptypumpsrv = rospy.Service('sampler/empty_pump', EmptyPump, handle_empty_pump)
    stoppumpsrv = rospy.Service('sampler/stop_pump', StopPump, handle_stop_pump)
    rospy.loginfo("[Pump service node] Advertised sampler services")
    rospy.spin()    

if __name__ == "__main__":
    # Advertise services
    advServices()