# Brian Lesko
# receive and translate DualSense controller io (PS5 remote) over a usb connection
# Used for controlling animations, a maze game, and a sketchpad app
# 11/27/23

import hid
import numpy as np
import math

class DualSense:

    def __init__(self, vendorID, productID):
        self.vendorID = vendorID
        self.productID = productID
        self.device = hid.device()
        self.data = []

        # Initialize Dpad states
        self.DpadUp = False
        self.DpadDown = False
        self.DpadLeft = False
        self.DpadRight = False

        # Initialize button states
        self.dpad_up = False
        self.dpad_right = False
        self.dpad_down = False
        self.dpad_left = False
        self.L1 = False
        self.L2 = False
        self.R1btn = False
        self.R2btn = False

        # Initialize gyro readings
        self.Pitch = []
        self.Yaw = []
        self.Roll = []

        # Initialize accelerometer readings
        self.X = []
        self.Y = []
        self.Z = []

        # Initialize the Touchpad 
        self.touchpad_x = []
        self.touchpad_y = []
        # Touchpad two finger touch
        self.touchpad1_x = []
        self.touchpad1_y = []

        # Initialize the battery readings
        self.battery_state = "POWER_SUPPLY_STATUS_UNKNOWN"
        self.battery_level = 0

        # Initialize R3 and L3
        self.R3 = False
        self.L3 = False

        # Initialize sending data back to the controller (lights, motors, etc)
        self.outReport = None

    def disconnect(self):
        self.device.close()

    def connect(self):
        self.device.open(self.vendorID, self.productID)

    def receive(self, size=64):
        self.data = self.device.read(size)

    def send(self, command):
        self.device.write(command)

    def updateButtons(self):
        buttonState = self.data[8]
        self.triangle = (buttonState & (1 << 7)) != 0
        self.circle = (buttonState & (1 << 6)) != 0
        self.cross = (buttonState & (1 << 5)) != 0
        self.square = (buttonState & (1 << 4)) != 0

    def updateDpad(self):
        buttonState = self.data[8]
        dpad_state = buttonState & 0x0F
        dpad_map = {
            0: (True, False, False, False),
            1: (True, False, False, True),
            2: (False, False, False, True),
            3: (False, True, False, True),
            4: (False, True, False, False),
            5: (False, True, False, False),
            6: (False, False, True, False),
            7: (True, False, True, False),
        }
        self.DpadUp, self.DpadDown, self.DpadLeft, self.DpadRight = dpad_map.get(dpad_state, (False, False, False, False))

    def updateGyrometer(self, n=20):
        data = self.data
        self.Pitch.append(int.from_bytes(([data[22], data[23]]), byteorder='little', signed=True))
        self.Yaw.append(int.from_bytes(([data[24], data[25]]), byteorder='little', signed=True))
        self.Roll.append(int.from_bytes(([data[26], data[27]]), byteorder='little', signed=True))
        # Remove the oldest reading if the length exceeds n, we do this so that we can use an averaged gyro reading because the raw gyro readings are noisy
        if len(self.Pitch) > n:
            self.Pitch.pop(0), self.Yaw.pop(0), self.Roll.pop(0)
    
    def updateAccelerometer(self, n=20):
        data = self.data
        self.X.append(int.from_bytes(([data[16], data[17]]), byteorder='little', signed=True))
        self.Y.append(int.from_bytes(([data[18], data[19]]), byteorder='little', signed=True))
        self.Z.append(int.from_bytes(([data[20], data[21]]), byteorder='little', signed=True))
        if len(self.X) > n:
            self.X.pop(0), self.Y.pop(0), self.Z.pop(0)

    def updateTriggers(self):
        misc = self.data[9]
        self.R1 = (misc & (1 << 1)) != 0
        self.R2 = self.data[6]
        self.R2btn = (misc & (1 << 3)) != 0

        self.L1 = (misc & (1 << 0)) != 0
        self.L2 = self.data[5]
        self.L2btn = (misc & (1 << 2)) != 0

    def updateTouchpad(self, n=1):
        data = self.data
        def update_touchpad(touchpad_id, offset):
            touchpad_isActive = (data[offset] & 0x80) == 0
            touchpad_x = ((data[offset+2] & 0x0f) << 8) | data[offset+1]
            touchpad_y = ((data[offset+3]) << 4) | ((data[offset+2] & 0xf0) >> 4)
            return touchpad_isActive, touchpad_x, touchpad_y

        self.touchpad_isActive, touchpad_x, touchpad_y = update_touchpad(0, 33)
        self.touchpad1_isActive, touchpad1_x, touchpad1_y = update_touchpad(1, 37)

        self.touchpad_x.append(touchpad_x)
        self.touchpad_y.append(touchpad_y)
        self.touchpad1_x.append(touchpad1_x)
        self.touchpad1_y.append(touchpad1_y)

        if len(self.touchpad_x) > n:
            self.touchpad_x.pop(0), self.touchpad_y.pop(0)
        if len(self.touchpad1_x) > n:
            self.touchpad1_x.pop(0), self.touchpad1_y.pop(0)

    def updateBattery(self):
        battery = self.data[53]
        state = (battery & 0xF0) >> 4
        BATTERY_STATES = {
            0x0: "POWER_SUPPLY_STATUS_DISCHARGING",
            0x1: "POWER_SUPPLY_STATUS_CHARGING",
            0x2: "POWER_SUPPLY_STATUS_FULL",
            0xb: "POWER_SUPPLY_STATUS_NOT_CHARGING",
            0xf: "POWER_SUPPLY_STATUS_ERROR",
            0xa: "POWER_SUPPLY_TEMP_OR_VOLTAGE_OUT_OF_RANGE",
            # Note: 0x0 is repeated, so we only keep one mapping for it
        }
        self.battery_state = BATTERY_STATES.get(state, "POWER_SUPPLY_STATUS_UNKNOWN")
        self.battery_level = min((battery & 0x0F) * 10 + 5, 100)

    def init_outReport(self):
        if self.outReport is None:
            bytes = [0] * 64
            bytes[0] = 0x02 # This byte says "USB"     BT = 0x0? 
            bytes[1] = 0xff # [1]
            bytes[2] = 0x1 | 0x2 | 0x4 | 0x10 | 0x40 # [2] # This is a flag for what is being sent, not sure what the values mean
            # bytes[5] - bytes[8] audio related
            #bytes[9] = self.audio.microphone_led # [9] #set Micrphone LED, setting doesnt effect microphone settings
            #bytes[10] = 0x10 if self.audio.microphone_mute is True else 0x00
            # bytes[11] - bytes[20] Right Trigger
            # bytes[21] - bytes[31] Left Trigger
            # bytes[39] - bytes[47] Touchpad
            self.outReport = bytes
    
    def clear_outReport(self):
        self.outReport = None
        bytes = [0] * 64
        self.send(bytes)

    def send_outReport(self):
        self.send(self.outReport)

    def rumble(self, intensity = 100, L = 100, R = 100):
        if self.outReport is None: self.init_outReport()
        # Rumble values are integers between 0 and 255
        if L != R: 
            self.outReport[3] = R
            self.outReport[4] = L
        else: 
            self.outReport[3] = intensity
            self.outReport[4] = intensity

    def lights(self, brightness = 1, rgb = (0,100,0), mode = 0, pulse = 0 ):
        if self.outReport is None: self.init_outReport()
        self.outReport[39] = mode  # Mode    # Off = 0 # PlayerLedBrightness = 1     # UninterrumpableLed = 2  # Both = 3
        self.outReport[42] = pulse  # Pulse option     # Off = 0     # FadeBlue = 0    # FadeOut = 0
        self.outReport[43] = brightness  # Brightness    low = 2     # Medium = 1     # High = 0
        self.outReport[44] = 4 # playernumber value # 4 10 21 27 
        self.outReport[45:48] = rgb # RGB must be 0 to 255

    def set_trigger(self, mode = 0xFC, intensities = None):# bytes[11] - bytes[20] Right Trigger
        if self.outReport is None: self.init_outReport()
        if intensities is None: intensities = [0,0,0,0,0,0,0]
        # Trigger modes
        #Off = 0x0  # no resistance
        #Rigid = 0x1  # continous resistance
        #Pulse = 0x2  # section resistance
        #Rigid_A = 0x1 | 0x20
        #Rigid_B = 0x1 | 0x04
        #Rigid_AB = 0x1 | 0x20 | 0x04
        #Pulse_A = 0x2 | 0x20
        #Pulse_B = 0x2 | 0x04
        #Pulse_AB = 0x2 | 0x20 | 0x04
        #Calibration = 0xFC

        # Right Trigger
        self.outReport[1] = 0xff | 0x4 | 0x08
        self.outReport[2] = 0x1 | 0x2 | 0x4 | 0x10 | 0x40
        
        self.outReport[11] = mode # Trigger mode Off: 0 Rigid: 1 Pulse: 2 Calibration: 0xFC 
        #self.outReport[12] = mode
        self.outReport[13] = intensities[1]
        self.outReport[14] = intensities[2]
        self.outReport[15] = intensities[3]
        self.outReport[16] = intensities[4]
        self.outReport[17] = intensities[5]
        self.outReport[20] = intensities[6]
       
        #self.outReport[21] = 1 # ??
        self.outReport[22] = mode # Trigger mode
        #self.outReport[23] = mode
        self.outReport[24] = intensities[1]
        self.outReport[25] = intensities[2]
        self.outReport[26] = intensities[3]
        self.outReport[27] = intensities[4]
        self.outReport[28] = intensities[5]
        self.outReport[31] = intensities[6]
        
    def updateThumbStickPress(self):
        misc = self.data[9]
        self.R3 = (misc & (1 << 7)) != 0
        self.L3 = (misc & (1 << 6)) != 0

    def updateThumbsticks(self):
        data = self.data
        # Raw data
        self.LX = data[1] - 127
        self.LY = data[2] - 127
        self.RX = data[3] - 127
        self.RY = data[4] - 127

        # Calculate angle
        thetas = [0,0]
        # Thumbstick control
        if abs(self.LX) > 4:
            thetas[0] = thetas[0] + (self.LX)/200
        if abs(self.LY) > 4:
            thetas[1] = thetas[1] - (self.LY)/200
        # find the angle from x and y (theta0 and theta1)
        theta = 0
        if np.abs(thetas[0]) > 0:
            theta = np.arctan2(-thetas[0],thetas[1])
        self.Ltheta = theta

    def updateMisc(self):
        misc = self.data[9]
        self.options = (misc & (1 << 5)) != 0
        self.share = (misc & (1 << 4)) != 0
        misc2 = self.data[10]
        self.ps = (misc2 & (1 << 0)) != 0
        self.touchButon = (misc2 & 0x02) != 0
        self.micButon = (misc2 & 0x04) != 0

    def updateAll(self):
        self.receive()
        self.updateMisc()
        self.updateThumbsticks()
        self.updateBattery()
        self.updateTouchpad(n=1)
        self.updateTriggers()
        self.updateAccelerometer(n=1)
        self.updateGyrometer(n=1)
        self.updateDpad()
        self.updateButtons()