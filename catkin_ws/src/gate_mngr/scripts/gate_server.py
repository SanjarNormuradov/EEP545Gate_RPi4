#!/usr/bin/env python3
from __future__ import print_function  # To use print() as a function
import rospy
import re
import subprocess
from gate_mngr.srv import lcdPrint, lcdPrintResponse, servoCtl, servoCtlResponse
from gpiozero import AngularServo, Buzzer, DistanceSensor, OutputDevice, PWMLED
from random import random
from time import sleep
from threading import Thread


class LCD():
    """16x2 LCD Screen.

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
    def __init__(self,
                 RS=19, RW=None, E=13,
                 D0=None, D1=None, D2=None, D3=None,
                 D4=18, D5=23, D6=24, D7=25,
                 BCKGRD_R=6, BCKGRD_G=20, BCKGRD_B=21,
                 ):
        """Initialize and clear display.

        Args:
            Boardcom(BCM) GPIO pin numbers (int) for
            RS: Register Select
            E: Enable
            D0-7: Data
            BCKGRD_RGB: Background Color
        """
        # Control Pins
        self.RS = OutputDevice(RS)
        self.E = OutputDevice(E)
        # Data Pins
        self.D4 = OutputDevice(D4)
        self.D5 = OutputDevice(D5)
        self.D6 = OutputDevice(D6)
        self.D7 = OutputDevice(D7)
        # Background Color Pins. Default Color is White
        self.BCKGRD_R = PWMLED(BCKGRD_R)
        self.BCKGRD_G = PWMLED(BCKGRD_G)
        self.BCKGRD_B = PWMLED(BCKGRD_B)

        self.CHARS = 16  # Characters per line (16 max)
        self.LINE_1 = 0x80  # LCD memory location for 1st line
        self.LINE_2 = 0xC0  # LCD memory location for 2nd line
        self.COLOR = "white"

        # Initialize with 8-bit bus mode
        self._write(0x33, mode="command")
        # Set to 4-bit mode
        self._write(0x32, mode="command")
        # Set 2 line with 5x8 dots format display
        self._write(0x28, mode="command")
        # Cursor move right, no shift of entire display in write mode
        self._write(0x06, mode="command")
        # Turn display on, cursor & blinkin off
        self._write(0x0C, mode="command")
        self.clear()  # Clear display
        sleep(0.0005)  # Delay to allow commands to process

    def print(self, text, line=1, position="left"):
        if line == 2:
            line = self.LINE_2
        else:
            line = self.LINE_1
        # Pad with space to adjust text size to 16 chars
        if position.lower() == "center":
            text = text.center(self.CHARS)
        elif position.lower() == "right":
            text = text.rjust(self.CHARS)
        else:
            text = text.ljust(self.CHARS)
        # Set the cursor to 1st/2nd line
        self._write(line, mode="command")

        # Send text by characters
        for i in range(self.CHARS):
            self._write(ord(text[i]), mode="data")

    def _write(self, bits, mode):
        """Write data to LCD.
        4-bit bus mode is used, which requires to transfer 4-bit data twice.

        Args:
            bits: int, Data to send in ASCII format
            mode: string, Transfer data type (data, command) 
        """
        if mode == "data":
            self.RS.on()
        elif mode == "command":
            self.RS.off()

        # High Data bits
        self.D4.off()
        self.D5.off()
        self.D6.off()
        self.D7.off()
        if bits & 0x10 == 0x10:
            self.D4.on()
        if bits & 0x20 == 0x20:
            self.D5.on()
        if bits & 0x40 == 0x40:
            self.D6.on()
        if bits & 0x80 == 0x80:
            self.D7.on()
        self._enable()

        # Low Data bits
        self.D4.off()
        self.D5.off()
        self.D6.off()
        self.D7.off()
        if bits & 0x01 == 0x01:
            self.D4.on()
        if bits & 0x02 == 0x02:
            self.D5.on()
        if bits & 0x04 == 0x04:
            self.D6.on()
        if bits & 0x08 == 0x08:
            self.D7.on()
        self._enable()

    def _enable(self):
        """Toggle 'Enable' bit to transfer data."""
        sleep(0.0005)
        self.E.on()
        sleep(0.0005)
        self.E.off()
        sleep(0.0005)

    def clear(self):
        """Clear LCD screen."""
        self._write(0x01, mode="command")

    def color_background(self, color="white"):
        """Change background color."""
        if color.lower() == "white":
            self.BCKGRD_R.off()
            self.BCKGRD_G.off()
            self.BCKGRD_B.off()
        elif color.lower() == "red":
            self.BCKGRD_R.off()
            self.BCKGRD_G.on()
            self.BCKGRD_B.on()
        elif color.lower() == "green":
            self.BCKGRD_R.on()
            self.BCKGRD_G.off()
            self.BCKGRD_B.on()
        elif color.lower() == "blue":
            self.BCKGRD_R.on()
            self.BCKGRD_G.on()
            self.BCKGRD_B.off()
        elif color.lower() == "random":
            self.BCKGRD_R.value = random()
            self.BCKGRD_G.value = random()
            self.BCKGRD_B.value = random()
        else:
            print("[LCD] Unrecognized background color")

    def tick(self, duration=20, line=2, position="left"):
        for i in range(duration, -1, -1):
            self.print("Time Left: %d s" % i, line, position)
            sleep(1.0)
        self.print("", line, position)

    def close(self):
        """Return GPIO."""
        self.clear()
        self.print("Bye!", 1)
        sleep(1.0)
        self.clear()
        self.RS.close()
        self.E.close()
        self.D4.close()
        self.D5.close()
        self.D6.close()
        self.D7.close()
        self.BCKGRD_R.close()
        self.BCKGRD_G.close()
        self.BCKGRD_B.close()


class GateServer():
    def __init__(self,
                 SERVO=4, BUZZER=5,
                 DIST_ECHO=17, DIST_TRIG=22,
                 LCD_RS=19, LCD_E=13,
                 LCD_D4=18, LCD_D5=23, LCD_D6=24, LCD_D7=25,
                 LCD_BCKGRD_R=6, LCD_BCKGRD_G=20, LCD_BCKGRD_B=21
                 ):
        """Gate Server initialization.

        Args:
            tof_pub_rate: int, publishing rate of ToF sensor measurements
        """
        print("[GateServer] Initialization...")
        self.ip_addr = self.get_ip_address("wlan0")
        self.ip_addr = self.ip_addr if self.ip_addr is not None else "No IP Address"
        # Servo
        rospy.Service('servo_controller', servoCtl, self.servo_controller)
        self.servo = AngularServo(SERVO, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0006, max_pulse_width=0.0025)
        # Tonal Buzzer
        # self.buzzer = Buzzer(BUZZER)
        # self.buzzer_notes = ['C', 'D', 'E', 'F', 'G', 'A', 'B']
        # Ultrasound Distance Sensor
        # self.dist_sensor = DistanceSensor(echo=DIST_ECHO, trigger=DIST_TRIG)
        # 16x2 LCD Screen
        self.lcd = LCD(
            RS=LCD_RS, E=LCD_E,
            D4=LCD_D4, D5=LCD_D5, D6=LCD_D6, D7=LCD_D7,
            BCKGRD_R=LCD_BCKGRD_R, BCKGRD_G=LCD_BCKGRD_G, BCKGRD_B=LCD_BCKGRD_B
        )
        rospy.Service('lcd_printer', lcdPrint, self.lcd_printer)
        self.lcd_line2 = None
        self.lcd.print("UW: %s" % self.ip_addr, line=1, position="left")
        print("[GateServer] Initialization completed")
        self.status = "CLOSED"
        print("[GateServer] Ready to open/close the gate, print text on the LCD screen.")

    def get_ip_address(self, interface):
        # Run the 'ip a' command and capture its output
        result = subprocess.run(['ip', 'a'], stdout=subprocess.PIPE, text=True)
        output = result.stdout

        # Regular expression to match the IP address
        ip_regex = r'inet (\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'

        # Flag to indicate if the next line should be checked for an IP address
        check_next_line = False
        # Split the output into lines and iterate over them
        for line in output.split('\n'):
            if check_next_line:
                ip_match = re.search(ip_regex, line)
                if ip_match:
                    return ip_match.group(1)
                check_next_line = False

            if interface in line:
                # Set flag to check the next line for the IP address
                check_next_line = True
        return None

    def open_gate(self):
        # if (self.dist_sensor.distance >= 0.5) and (self.dist_sensor.distance < 1.0):
        self.servo.angle = 90
        self.status = "OPEN"
        self.lcd.print("GATE %s" % self.status, line=1, position="center")
        self.lcd.color_background(color="green")
        # self.buzzer_play(gate_status=self.status)
        self.lcd_line2 = Thread(target=self.lcd.tick(duration=20, line=2, position="center"))
        self.lcd_line2.start()
        print("[GateServer] Gate is open.")
        return "open"
        # elif self.dist_sensor.distance < 0.5:
        #     self.status = "BLOCKED"
        #     self.lcd.print("GATE %s" % self.status, line=1, position="center")
        #     self.lcd.color_background(color="red")
        #     # self.buzzer_play(gate_status=self.status)
        #     print("[GateServer] Gate is blocked, move backwards.")
        #     return "blocked"
        # else:
        #     self.status = "IDLE"
        #     self.lcd.print("GATE %s" % self.status, line=1, position="center")
        #     self.lcd.color_background(color="blue")
        #     # self.buzzer_play(gate_status=self.status)
        #     return "idle"

    def close_gate(self):
        self.status = "CLOSING"
        self.lcd.print("GATE %s" % self.status, line=1, position="center")
        for i in range(90, -1, -1):
            # self.buzzer_play(gate_status=self.status)
            self.lcd.color_background(color="random")
            self.servo.angle = i
            sleep(0.1)
        self.status = "CLOSED"
        self.lcd.print("GATE %s" % self.status, line=1, position="center")
        self.lcd.color_background(color="white")
        # self.buzzer_play(gate_status=self.status)
        sleep(1.0)
        print("[GateServer] Gate is closed.")
        return "closed"

    def servo_controller(self, req):
        if req.command == "open":
            status = self.open_gate()
        elif req.command == "close":
            status = self.close_gate()
        else:
            print("Unrecognized command. Available: open/close")
        return servoCtlResponse(status)

    def lcd_printer(self, req):
        self.lcd.print(req.text, req.line)
        print("[GateServer] Text '%s' printed on LCD" % req.text)
        return lcdPrintResponse("printed")

    # def buzzer_play(self, gate_status="OPEN"):
    #     if gate_status == "OPEN":
    #         for note in self.buzzer_notes:
    #             self.buzzer.play("%s5" % note)
    #             sleep(0.5)
    #     if gate_status == "BLOCKED":
    #         notes = reversed(self.buzzer_notes)
    #         for note in notes:
    #             self.buzzer.play("%s3" % note)
    #             sleep(0.5)
    #     if gate_status == "CLOSE":
    #         for note in self.buzzer_notes[:4]:
    #             self.buzzer.play("%s4" % note)
    #             sleep(0.5)
    #     if gate_status == "CLOSING":
    #         for note in self.buzzer_notes[4:]:
    #             self.buzzer.play("%s4" % note)
    #             sleep(0.5)
    #     if gate_status == "IDLE":
    #         notes = reversed(self.buzzer_notes)
    #         for note1, note2 in zip(self.buzzer_notes, notes):
    #             self.buzzer.play("%s5" % note1)
    #             sleep(0.25)
    #             self.buzzer.play("%s3" % note2)
    #             sleep(0.25)
    #     self.buzzer.stop()


if __name__ == "__main__":
    rospy.init_node('gate_server', anonymous=True)

    SERVO = rospy.get_param("~SERVO", 4)
    BUZZER = rospy.get_param("~BUZZER", 5)
    DIST_ECHO = rospy.get_param("~DIST_ECHO", 17)
    DIST_TRIG = rospy.get_param("~DIST_TRIG", 22)
    LCD_RS = rospy.get_param("~LCD_RS", 19)
    LCD_E = rospy.get_param("~LCD_E", 13)
    LCD_D4 = rospy.get_param("~LCD_D4", 18)
    LCD_D5 = rospy.get_param("~LCD_D5", 23)
    LCD_D6 = rospy.get_param("~LCD_D6", 24)
    LCD_D7 = rospy.get_param("~LCD_D7", 25)
    LCD_BCKGRD_R = rospy.get_param("~LCD_BCKGRD_R", 6)
    LCD_BCKGRD_G = rospy.get_param("~LCD_BCKGRD_G", 20)
    LCD_BCKGRD_B = rospy.get_param("~LCD_BCKGRD_B", 21)
    gs = GateServer(
        SERVO,
        BUZZER,
        DIST_ECHO, DIST_TRIG,
        LCD_RS, LCD_E,
        LCD_D4, LCD_D5, LCD_D6, LCD_D7,
        LCD_BCKGRD_R, LCD_BCKGRD_G, LCD_BCKGRD_B
    )

    try:
        while not rospy.is_shutdown():
            # if gs.lcd_line2 is None or not gs.lcd_line2.is_active():
            #     if gs.status in ["OPEN", "IDLE"]:
            #         if gs.status == "OPEN":
            #             gs.close_gate()
            #             gs.lcd_line2 = None
            #         gs.lcd.print("UW: %s" % gs.ip_addr, line=1, position="left")
            #         gs.status = "CLOSED"
            #     elif gs.status == "BLOCKED":
            #         if gs.dist_sensor.distance > 0.5:
            #             gs.open_gate()
            #         else:
            #             gs.buzzer_play(gate_status=gs.status)
            #     print("Dist: %.2f cm" % (gs.dist_sensor.distance * 100))
            #     gs.lcd.print("Dist: %.2f cm" % (gs.dist_sensor.distance * 100), line=2, position="left")
            sleep(0.1)
    except KeyboardInterrupt:
        print("Program was interrupted by the user")
    finally:
        print("Ending program...")
        gs.servo.close()
        gs.lcd.close()
