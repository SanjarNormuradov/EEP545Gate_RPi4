#!/usr/bin/python
from __future__ import print_function  # To use print() as a function
from gpiozero import OutputDevice, PWMLED
from time import sleep
from random import random


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

    def print(self,
              text, line=1, position="left"):
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

    # def lcd_init(self):
    #     """Initialize and clear display.

    #     Pinout of the LCD:
    #     1 : GND
    #     2 : 5V power
    #     3 : Display contrast - Connect to middle pin potentiometer 
    #     4 : RS, Register Select, to choose Data or Instruction Registers
    #     5 : R/W (Read/Write) - Ground this pin to write, otherwise reading from LCD with 5V power might damage Raspberry Pi's input pins
    #     6 : Enable or Strobe
    #     7 : Data Bit 0 - data pin 0, 1, 2, 3 are not used
    #     8 : Data Bit 1 -
    #     9 : Data Bit 2 -
    #     10: Data Bit 3 -
    #     11: Data Bit 4
    #     12: Data Bit 5
    #     13: Data Bit 6
    #     14: Data Bit 7
    #     15: LCD Backlight +5V
    #     16: LCD Red Backlight GND
    #     17: LCD Green Backlight GND
    #     18: LCD Blue Backlight GND
    #     """
    #     # Control Pins
    #     self.LCD_RS = OutputDevice(23)
    #     self.LCD_E = OutputDevice(24)
    #     # Data Pins
    #     self.LCD_D4 = OutputDevice(12)
    #     self.LCD_D5 = OutputDevice(16)
    #     self.LCD_D6 = OutputDevice(20)
    #     self.LCD_D7 = OutputDevice(21)
    #     # Background Color Pins
    #     # self.LCD_BCKGRD_R = OutputDevice(6)
    #     # self.LCD_BCKGRD_G = OutputDevice(13)
    #     # self.LCD_BCKGRD_B = OutputDevice(19)

    #     self.LCD_CHARS = 16  # Characters per line (16 max)
    #     self.LCD_LINE_1 = 0x80  # LCD memory location for 1st line
    #     self.LCD_LINE_2 = 0xC0  # LCD memory location for 2nd line

    #     # Initialize with 8-bit bus mode
    #     self.lcd_write(0x33, mode="command")
    #     # Set to 4-bit mode
    #     self.lcd_write(0x32, mode="command")
    #     # Set 2 line with 5x8 dots format display
    #     self.lcd_write(0x28, mode="command")
    #     # Cursor move right, no shift of entire display in write mode
    #     self.lcd_write(0x06, mode="command")
    #     # Turn display on, cursor & blinkin off
    #     self.lcd_write(0x0C, mode="command")
    #     self.lcd_clear()  # Clear display
    #     sleep(0.0005)  # Delay to allow commands to process

    # def lcd_write(self, bits, mode):
    #     """Write data to LCD.
    #     4-bit bus mode is used, which requires to transfer 4-bit data twice.
    #     """
    #     if mode == "data":
    #         self.LCD_RS.on()
    #     elif mode == "command":
    #         self.LCD_RS.off()

    #     # High Data bits
    #     self.LCD_D4.off()
    #     self.LCD_D5.off()
    #     self.LCD_D6.off()
    #     self.LCD_D7.off()
    #     if bits & 0x10 == 0x10:
    #         self.LCD_D4.on()
    #     if bits & 0x20 == 0x20:
    #         self.LCD_D5.on()
    #     if bits & 0x40 == 0x40:
    #         self.LCD_D6.on()
    #     if bits & 0x80 == 0x80:
    #         self.LCD_D7.on()
    #     self.lcd_enable_tx()

    #     # Low Data bits
    #     self.LCD_D4.off()
    #     self.LCD_D5.off()
    #     self.LCD_D6.off()
    #     self.LCD_D7.off()
    #     if bits & 0x01 == 0x01:
    #         self.LCD_D4.on()
    #     if bits & 0x02 == 0x02:
    #         self.LCD_D5.on()
    #     if bits & 0x04 == 0x04:
    #         self.LCD_D6.on()
    #     if bits & 0x08 == 0x08:
    #         self.LCD_D7.on()
    #     self.lcd_enable_tx()

    # def lcd_enable_tx(self):
    #     """Toggle 'Enable' bit to transfer data."""
    #     sleep(0.0005)
    #     self.LCD_E.on()
    #     sleep(0.0005)
    #     self.LCD_E.off()
    #     sleep(0.0005)

    # def lcd_clear(self):
    #     """Clear LCD screen."""
    #     self.lcd_write(0x01, mode="command")

    # def lcd_text(self, text, line):
    #     text = text.ljust(self.LCD_CHARS, " ")  # Pad with space to adjust text size to 16 chars
    #     self.lcd_write(line, mode="command")  # Set the cursor to 1st/2nd line
    #     # Send text by characters
    #     for i in range(self.LCD_CHARS):
    #         self.lcd_write(ord(text[i]), mode="data")

    # def lcd_printer(self, req):
    #     text = req.text
    #     if req.line == 2:
    #         line = self.LCD_LINE_2
    #     else:
    #         line = self.LCD_LINE_1
    #     self.lcd_text(text, line)

    #     print("[GateServer] Text '%s' printed on LCD" % text)
    #     return lcdPrintResponse("printed")

    # def lcd_close(self):
    #     """Return GPIO."""
    #     self.lcd_clear()
    #     self.lcd_text("Bye!", 1)
    #     sleep(1.0)
    #     self.lcd_clear()
    #     self.LCD_RS.close()
    #     self.LCD_E.close()
    #     self.LCD_D4.close()
    #     self.LCD_D5.close()
    #     self.LCD_D6.close()
    #     self.LCD_D7.close()
    #     # self.LCD_BCKGRD_R.close()
    #     # self.LCD_BCKGRD_G.close()
    #     # self.LCD_BCKGRD_B.close()