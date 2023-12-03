#!/usr/bin/env python3
from __future__ import print_function
import rospy
from gate_mngr.srv import servoCtl, lcdPrint


class GateClient():
    def __init__(self,
                 init_text="Hello World"):
        print("[GateClient] Initialization...")
        # self.lcd_printer(init_text)
        print("[GateClient] Initialization completed")

    def lcd_printer(self, text="Hi"):
        rospy.wait_for_service('lcd_printer')
        try:
            lcd_printer = rospy.ServiceProxy('lcd_printer', lcdPrint)
            status = lcd_printer(text).status
            print("[GateClient] Text '%s' %s on LCD" % (text, status))
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def servo_controller(self, command="open"):
        rospy.wait_for_service('servo_controller')
        try:
            servo_controller = rospy.ServiceProxy('servo_controller', servoCtl)
            status = servo_controller(command).status
            print("[GateClient] Gate %s" % status)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node('gate_client')
    init_text = rospy.get_param("~init_text", "Hello World")
    gc = GateClient(
        init_text
    )
    while not rospy.is_shutdown():
        gate_command = input("[GateClient] What to do with the gate? open/close: ")
        gc.servo_controller(gate_command)
