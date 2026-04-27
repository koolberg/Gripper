# ============================================================
#  pico_transmitter.py — Full Arm Controller
#  Raspberry Pi Pico 2
#
#  Sensors:
#    MPU6500 #1 (hand)    → I2C0 GP16/GP17  addr 0x68
#    MPU6500 #2 (forearm) → I2C0 GP16/GP17  addr 0x69
#    MPU6500 #3 (finger)  → I2C1 GP2/GP3    addr 0x68
#
#  Buttons:
#    GP9  (Pin 12) Blue  → Emergency release (open gripper)
#    GP10 (Pin 14) Red   → Home position
#    GP11 (Pin 15) Green → Grip lock toggle
#
#  UART:
#    GP0 TX (Pin 1) → STM32 PA10 (USART1 RX)
#    GP1 RX (Pin 2) → STM32 PA9  (USART1 TX)
#
#  Packet: 23 bytes
#    0xAA + yaw(4) + pitch(4) + roll(4) +
#    forearm(4) + grip(4) + flags(1) + checksum(1)
# ============================================================

from machine import UART, Pin, I2C
from time import sleep
import struct
import math

# ── UART to STM32 ─────────────────────────────────────────
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# ── I2C Buses ─────────────────────────────────────────────
i2c0 = I2C(0, sda=Pin(16), scl=Pin(17), freq=100000)
i2c1 = I2C(1, sda=Pin(2),  scl=Pin(3),  freq=100000)

# ── Sensor Addresses ──────────────────────────────────────
HAND_ADDR    = 0x68    # MPU6500 #1  AD0 → GND   on I2C0
FOREARM_ADDR = 0x69    # MPU6500 #2  AD0 → 3.3V  on I2C0
FINGER_ADDR  = 0x68    # MPU6500 #3  AD0 → GND   on I2C1

# ── Buttons (pull-up: idle=HIGH, pressed=LOW) ─────────────
btn_emergency = Pin(9,  Pin.IN, Pin.PULL_UP)  # Blue  Pin 12
btn_home      = Pin(10, Pin.IN, Pin.PULL_UP)  # Red   Pin 14
btn_lock      = Pin(11, Pin.IN, Pin.PULL_UP)  # Green Pin 15

# ── Packet Flags ──────────────────────────────────────────
FLAG_NONE      = 0x00
FLAG_GRIP_LOCK = 0x01
FLAG_EMERGENCY = 0x02
FLAG_HOME      = 0x04

# ── State ─────────────────────────────────────────────────
grip_locked   = False
last_lock_btn = True

# ─────────────────────────────────────────────────────────
#  Sensor Functions
# ─────────────────────────────────────────────────────────

def mpu_init(bus, addr):
    try:
        bus.writeto_mem(addr, 0x6B, b'\x00')
        sleep(0.1)
        bus.writeto_mem(addr, 0x1B, b'\x08')
        bus.writeto_mem(addr, 0x1C, b'\x08')
        print("OK: addr=0x{:02X}".format(addr))
        return True
    except Exception as e:
        print("FAIL: addr=0x{:02X} {}".format(addr, e))
        return False

def mpu_read(bus, addr):
    data = bus.readfrom_mem(addr, 0x3B, 14)
    ax = struct.unpack('>h', data[0:2])[0]  / 8192.0
    ay = struct.unpack('>h', data[2:4])[0]  / 8192.0
    az = struct.unpack('>h', data[4:6])[0]  / 8192.0
    gx = struct.unpack('>h', data[8:10])[0] / 65.5
    gy = struct.unpack('>h', data[10:12])[0]/ 65.5
    gz = struct.unpack('>h', data[12:14])[0]/ 65.5
    return ax, ay, az, gx, gy, gz

# ─────────────────────────────────────────────────────────
#  Angle Trackers
# ─────────────────────────────────────────────────────────

class AngleTracker:
    def __init__(self):
        self.pitch = 0.0
        self.roll  = 0.0
        self.yaw   = 0.0

    def update(self, ax, ay, az, gx, gy, gz, dt):
        accel_pitch = math.atan2(ax,
                      math.sqrt(ay*ay + az*az)) * 180 / math.pi
        accel_roll  = math.atan2(ay, az) * 180 / math.pi
        self.pitch  = 0.98*(self.pitch + gx*dt) + 0.02*accel_pitch
        self.roll   = 0.98*(self.roll  + gy*dt) + 0.02*accel_roll
        self.yaw   += gz * dt
        if self.yaw >  180: self.yaw -= 360
        if self.yaw < -180: self.yaw += 360

class FingerTracker:
    def __init__(self):
        self.curl = 0.0

    def update(self, ax, ay, az, gx, gy, gz, dt):
        accel_pitch = math.atan2(ax,
                      math.sqrt(ay*ay + az*az)) * 180 / math.pi
        self.curl   = 0.98*(self.curl + gx*dt) + 0.02*accel_pitch
        if self.curl < 0:  self.curl = 0
        if self.curl > 90: self.curl = 90

# ─────────────────────────────────────────────────────────
#  Checksum and Packet
# ─────────────────────────────────────────────────────────

def calc_checksum(data):
    cs = 0
    for b in data:
        cs ^= b
    return cs

def send_packet(yaw, pitch, roll, forearm, grip, flags):
    payload   = struct.pack('>fffff',
                            yaw, pitch, roll, forearm, grip)
    flag_byte = bytes([flags])
    checksum  = calc_checksum(payload + flag_byte)
    packet    = bytes([0xAA]) + payload + flag_byte + bytes([checksum])
    uart.write(packet)

# ─────────────────────────────────────────────────────────
#  Button Handler
# ─────────────────────────────────────────────────────────

def read_buttons():
    global grip_locked, last_lock_btn
    flags = FLAG_NONE

    # Green button — grip lock toggle (press and release)
    current = btn_lock.value()
    if current == False and last_lock_btn == True:
        grip_locked = not grip_locked
        print("Grip lock: " + ("ON" if grip_locked else "OFF"))
    last_lock_btn = current
    if grip_locked:
        flags |= FLAG_GRIP_LOCK

    # Blue button — emergency release (hold)
    if btn_emergency.value() == False:
        flags |= FLAG_EMERGENCY
        print("Emergency release!")

    # Red button — home position (hold)
    if btn_home.value() == False:
        flags |= FLAG_HOME
        print("Home position!")

    return flags

# ─────────────────────────────────────────────────────────
#  Startup
# ─────────────────────────────────────────────────────────

print("=== Pico 2 Full Arm Transmitter ===")
print("Scanning I2C0:", [hex(d) for d in i2c0.scan()])
print("Scanning I2C1:", [hex(d) for d in i2c1.scan()])

mpu_init(i2c0, HAND_ADDR)
mpu_init(i2c0, FOREARM_ADDR)
mpu_init(i2c1, FINGER_ADDR)

hand    = AngleTracker()
forearm = AngleTracker()
finger  = FingerTracker()

dt    = 0.02
count = 0

print("\nButtons:")
print("  Blue  (GP9)  → Emergency release")
print("  Red   (GP10) → Home position")
print("  Green (GP11) → Grip lock toggle")
print("\nTransmitting...")

# ─────────────────────────────────────────────────────────
#  Main Loop
# ─────────────────────────────────────────────────────────

while True:
    try:
        # Read all three sensors
        ax,  ay,  az,  gx,  gy,  gz  = mpu_read(i2c0, HAND_ADDR)
        ax2, ay2, az2, gx2, gy2, gz2 = mpu_read(i2c0, FOREARM_ADDR)
        ax3, ay3, az3, gx3, gy3, gz3 = mpu_read(i2c1, FINGER_ADDR)

        # Update angle trackers
        hand.update(ax,  ay,  az,  gx,  gy,  gz,  dt)
        forearm.update(ax2, ay2, az2, gx2, gy2, gz2, dt)
        finger.update(ax3, ay3, az3, gx3, gy3, gz3, dt)

        # Read buttons
        flags = read_buttons()

        # Send packet to STM32
        send_packet(hand.yaw,    hand.pitch,
                    hand.roll,   forearm.pitch,
                    finger.curl, flags)

        count += 1
        if count % 25 == 0:
            print("Yaw:{:.1f} Pit:{:.1f} Rol:{:.1f} "
                  "FA:{:.1f} Grip:{:.1f} F:{:02X}".format(
                  hand.yaw,    hand.pitch,
                  hand.roll,   forearm.pitch,
                  finger.curl, flags))

    except Exception as e:
        print("Error: " + str(e))
        sleep(0.1)

    sleep(dt)