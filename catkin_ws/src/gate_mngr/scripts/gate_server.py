#!/usr/bin/env python3
from __future__ import print_function
import rospy
from gate_mngr.srv import lcdPrint, lcdPrintResponse, servoCtl, servoCtlResponse
from gate_mngr.msg import ToF
from gpiozero import AngularServo, LED, DistanceSensor, OutputDevice
from time import sleep


class GateServer():
    def __init__(self,
                 dist_pub_rate=30):
        """Gate Server initialization.

        Args:
            tof_pub_rate: int, publishing rate of ToF sensor measurements
        """
        print("[GateServer] Initialization...")
        self.redLED = LED(17)
        self.blueLED = LED(18)
        self.yellowLED = LED(27)
        self.redLED.off()
        self.blueLED.on()
        self.yellowLED.off()

        rospy.Service('servo_controller', servoCtl, self.servo_controller)
        self.servo = AngularServo(4, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0006, max_pulse_width=0.0025)

        self.dist_sensor = DistanceSensor(echo=26, trigger=5)
        self.dist_sensor_pub = rospy.Subscriber("/dist_sensor", ToF, self.dist_sensor_clbk, queue_size=1)
        rospy.Rate(dist_pub_rate)
        
        self.lcd_init()
        rospy.Service('lcd_printer', lcdPrint, self.lcd_printer)

        print("[GateServer] Initialization completed")
        print("[GateServer] Ready to open/close the gate, print text on the LCD screen.")

    def dist_sensor_clbk(self, msg):
        print("[GateServer] Distance sensor measurement = %f" % msg.dist)

    def open_gate(self):
        self.redLED.on()
        self.blueLED.off()
        self.servo.angle = 90
        print("[GateServer] Gate open.")
        return "open"

    def close_gate(self):
        self.redLED.off()
        self.blueLED.on()
        self.servo.angle = 0
        print("[GateServer] Gate closed.")
        return "closed"

    def servo_controller(self, req):
        if req.command == "open":
            status = self.open_gate()
        else:
            status = self.close_gate()
        return servoCtlResponse(status)

    def lcd_init(self):
        """Initialize and clear display.

        Pinout of the LCD:
        1 : GND
        2 : 5V power
        3 : Display contrast - Connect to middle pin potentiometer 
        4 : RS, Register Select, to choose Data or Instruction Registers
        5 : R/W (Read/Write) - Ground this pin to write, otherwise reading from LCD with 5V power might damage Raspberry Pi's input pins
        6 : Enable or Strobe
        7 : Data Bit 0 - data pin 0, 1, 2, 3 are not used
        8 : Data Bit 1 -
        9 : Data Bit 2 -
        10: Data Bit 3 -
        11: Data Bit 4
        12: Data Bit 5
        13: Data Bit 6
        14: Data Bit 7
        15: LCD Backlight +5V
        16: LCD Red Backlight GND
        17: LCD Green Backlight GND
        18: LCD Blue Backlight GND
        """
        # Control Pins
        self.LCD_RS = OutputDevice(23)
        self.LCD_E = OutputDevice(24)
        # Data Pins
        self.LCD_D4 = OutputDevice(12)
        self.LCD_D5 = OutputDevice(16)
        self.LCD_D6 = OutputDevice(20)
        self.LCD_D7 = OutputDevice(21)
        # Background Color Pins
        # self.LCD_BCKGRD_R = OutputDevice(6)
        # self.LCD_BCKGRD_G = OutputDevice(13)
        # self.LCD_BCKGRD_B = OutputDevice(19)

        self.LCD_CHARS = 16  # Characters per line (16 max)
        self.LCD_LINE_1 = 0x80  # LCD memory location for 1st line
        self.LCD_LINE_2 = 0xC0  # LCD memory location for 2nd line

        # Initialize with 8-bit bus mode
        self.lcd_write(0x33, mode="command")
        # Set to 4-bit mode
        self.lcd_write(0x32, mode="command")
        # Set 2 line with 5x8 dots format display
        self.lcd_write(0x28, mode="command")
        # Cursor move right, no shift of entire display in write mode
        self.lcd_write(0x06, mode="command")
        # Turn display on, cursor & blinkin off
        self.lcd_write(0x0C, mode="command")
        self.lcd_clear()  # Clear display
        sleep(0.0005)  # Delay to allow commands to process

    def lcd_write(self, bits, mode):
        """Write data to LCD.
        4-bit bus mode is used, which requires to transfer 4-bit data twice.
        """
        if mode == "data":
            self.LCD_RS.on()
        elif mode == "command":
            self.LCD_RS.off()

        # High Data bits
        self.LCD_D4.off()
        self.LCD_D5.off()
        self.LCD_D6.off()
        self.LCD_D7.off()
        if bits & 0x10 == 0x10:
            self.LCD_D4.on()
        if bits & 0x20 == 0x20:
            self.LCD_D5.on()
        if bits & 0x40 == 0x40:
            self.LCD_D6.on()
        if bits & 0x80 == 0x80:
            self.LCD_D7.on()
        self.lcd_enable_tx()

        # Low Data bits
        self.LCD_D4.off()
        self.LCD_D5.off()
        self.LCD_D6.off()
        self.LCD_D7.off()
        if bits & 0x01 == 0x01:
            self.LCD_D4.on()
        if bits & 0x02 == 0x02:
            self.LCD_D5.on()
        if bits & 0x04 == 0x04:
            self.LCD_D6.on()
        if bits & 0x08 == 0x08:
            self.LCD_D7.on()
        self.lcd_enable_tx()

    def lcd_enable_tx(self):
        """Toggle 'Enable' bit to transfer data."""
        sleep(0.0005)
        self.LCD_E.on()
        sleep(0.0005)
        self.LCD_E.off()
        sleep(0.0005)

    def lcd_clear(self):
        """Clear LCD screen."""
        self.lcd_write(0x01, mode="command")

    def lcd_text(self, text, line):
        text = text.ljust(self.LCD_CHARS, " ")  # Pad with space to adjust text size to 16 chars
        self.lcd_write(line, mode="command")  # Set the cursor to 1st/2nd line
        # Send text by characters
        for i in range(self.LCD_CHARS):
            self.lcd_write(ord(text[i]), mode="data")

    def lcd_printer(self, req):
        text = req.text
        if req.line == 2:
            line = self.LCD_LINE_2
        else:
            line = self.LCD_LINE_1
        self.lcd_text(text, line)

        print("[GateServer] Text '%s' printed on LCD" % text)
        return lcdPrintResponse("printed")

    def lcd_close(self):
        """Return GPIO."""
        self.lcd_clear()
        self.lcd_text("Bye!", 1)
        sleep(1.0)
        self.lcd_clear()
        self.LCD_RS.close()
        self.LCD_E.close()
        self.LCD_D4.close()
        self.LCD_D5.close()
        self.LCD_D6.close()
        self.LCD_D7.close()
        # self.LCD_BCKGRD_R.close()
        # self.LCD_BCKGRD_G.close()
        # self.LCD_BCKGRD_B.close()


if __name__ == "__main__":
    rospy.init_node('gate_server', anonymous=True)

    tof_pub_rate = rospy.get_param("~tof_pub_rate", 30)
    gs = GateServer(
        tof_pub_rate
    )

    try:
        while not rospy.is_shutdown():
            gs.yellowLED.blink()
            print("Distance: %.2f cm" % (gs.dist_sensor.distance * 100))
            gs.lcd_text("Distance (cm):", gs.LCD_LINE_1)
            gs.lcd_text("%.2f" % (gs.dist_sensor.distance * 100), gs.LCD_LINE_2)
            sleep(1.0)
    except KeyboardInterrupt:
        print("Program was interrupted by the user")
    finally:
        print("Ending program...")
        gs.servo.close()
        gs.redLED.close()
        gs.blueLED.close()
        gs.yellowLED.close()
        gs.lcd_close()
