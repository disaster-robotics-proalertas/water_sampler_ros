#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from PumpControl import Sampler
import rosservice
import rospy
import time
from water_sampler_ros.srv import *

# Initialize pumps object
pumps = Sampler()

# Diagnostic status for services
fill_pump_status = DiagnosticStatus()
check_pump_status = DiagnosticStatus()
empty_pump_status = DiagnosticStatus()
stop_pump_status = DiagnosticStatus()
fill_pump_status.name = 'sampler fill pump service'
check_pump_status.name = 'sampler check pump service'
empty_pump_status.name = 'sampler empty pump service'
stop_pump_status.name = 'sampler stop pump service'
fill_pump_status.hardware_id = 'sampler'
check_pump_status.hardware_id = 'sampler'
empty_pump_status.hardware_id = 'sampler'
stop_pump_status.hardware_id = 'sampler'

def check_status(status, srv):
    # Check if service exists in ROS services
    if any(srv in s for s in rosservice.get_service_list()):
        status.level = DiagnosticStatus.OK
        status.message = "OK"
        status.values = [KeyValue(key="Update Status", value="OK")]
    else:
        status.level = DiagnosticStatus.ERROR
        status.message = "Error"
        status.values = [KeyValue(key="Update Status", value="Error"),
                         KeyValue(key="%s" % srv, value="Service not found")]

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

    # Diagnostics
    diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        # Check status for services
        check_status(fill_pump_status, 'fill_pump')
        check_status(check_pump_status, 'check_pump')
        check_status(empty_pump_status, 'empty_pump')
        check_status(stop_pump_status, 'stop_pump')

        # Publish diagnostics message
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()
        diag_msg.status.append(fill_pump_status)
        diag_msg.status.append(check_pump_status)
        diag_msg.status.append(empty_pump_status)
        diag_msg.status.append(stop_pump_status)
        diag_pub.publish(diag_msg)

        rate.sleep()


if __name__ == "__main__":
    # Advertise services
    advServices()