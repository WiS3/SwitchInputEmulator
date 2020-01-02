#!/usr/bin/env python3
import argparse
import serial
import select
import struct
import sys
import time
import math

parser = argparse.ArgumentParser('Client for sending Pokemon Sword/Shield command macros to a controller emulator')
parser.add_argument('port', help='Serial port of connected controller emulator. On a mac, check About This Mac > System Report')
parser.add_argument('macro', help='Macro to send to the controller emulator', default='force_sync', choices=['breed_for_shiny', 'force_sync', 'loto_id', 'mash_a', 'next_den_day', 'release_box', 'skip_day'])
parser.add_argument('--iterations', help='Number of iterations of macro to execute. Default varies by macro. Does not apply to all macros.', default=0, type=int)
args = parser.parse_args()

STATE_OUT_OF_SYNC   = 0
STATE_SYNC_START    = 1
STATE_SYNC_1        = 2
STATE_SYNC_2        = 3
STATE_SYNC_OK       = 4

# Actual Switch DPAD Values
A_DPAD_CENTER    = 0x08
A_DPAD_U         = 0x00
A_DPAD_U_R       = 0x01
A_DPAD_R         = 0x02
A_DPAD_D_R       = 0x03
A_DPAD_D         = 0x04
A_DPAD_D_L       = 0x05
A_DPAD_L         = 0x06
A_DPAD_U_L       = 0x07

# Enum DIR Values
DIR_CENTER    = 0x00
DIR_U         = 0x01
DIR_R         = 0x02
DIR_D         = 0x04
DIR_L         = 0x08
DIR_U_R       = DIR_U + DIR_R
DIR_D_R       = DIR_D + DIR_R
DIR_U_L       = DIR_U + DIR_L
DIR_D_L       = DIR_D + DIR_L

BTN_NONE         = 0x0000000000000000
BTN_Y            = 0x0000000000000001
BTN_B            = 0x0000000000000002
BTN_A            = 0x0000000000000004
BTN_X            = 0x0000000000000008
BTN_L            = 0x0000000000000010
BTN_R            = 0x0000000000000020
BTN_ZL           = 0x0000000000000040
BTN_ZR           = 0x0000000000000080
BTN_MINUS        = 0x0000000000000100
BTN_PLUS         = 0x0000000000000200
BTN_LCLICK       = 0x0000000000000400
BTN_RCLICK       = 0x0000000000000800
BTN_HOME         = 0x0000000000001000
BTN_CAPTURE      = 0x0000000000002000

DPAD_CENTER      = 0x0000000000000000
DPAD_U           = 0x0000000000010000
DPAD_R           = 0x0000000000020000
DPAD_D           = 0x0000000000040000
DPAD_L           = 0x0000000000080000
DPAD_U_R         = DPAD_U + DPAD_R
DPAD_D_R         = DPAD_D + DPAD_R
DPAD_U_L         = DPAD_U + DPAD_L
DPAD_D_L         = DPAD_D + DPAD_L

LSTICK_CENTER    = 0x0000000000000000
LSTICK_R         = 0x00000000FF000000 #   0 (000)
LSTICK_U_R       = 0x0000002DFF000000 #  45 (02D)
LSTICK_U         = 0x0000005AFF000000 #  90 (05A)
LSTICK_U_L       = 0x00000087FF000000 # 135 (087)
LSTICK_L         = 0x000000B4FF000000 # 180 (0B4)
LSTICK_D_L       = 0x000000E1FF000000 # 225 (0E1)
LSTICK_D         = 0x0000010EFF000000 # 270 (10E)
LSTICK_D_R       = 0x0000013BFF000000 # 315 (13B)

RSTICK_CENTER    = 0x0000000000000000
RSTICK_R         = 0x000FF00000000000 #   0 (000)
RSTICK_U_R       = 0x02DFF00000000000 #  45 (02D)
RSTICK_U         = 0x05AFF00000000000 #  90 (05A)
RSTICK_U_L       = 0x087FF00000000000 # 135 (087)
RSTICK_L         = 0x0B4FF00000000000 # 180 (0B4)
RSTICK_D_L       = 0x0E1FF00000000000 # 225 (0E1)
RSTICK_D         = 0x10EFF00000000000 # 270 (10E)
RSTICK_D_R       = 0x13BFF00000000000 # 315 (13B)

NO_INPUT       = BTN_NONE + DPAD_CENTER + LSTICK_CENTER + RSTICK_CENTER

# Commands to send to MCU
COMMAND_NOP        = 0x00
COMMAND_SYNC_1     = 0x33
COMMAND_SYNC_2     = 0xCC
COMMAND_SYNC_START = 0xFF

# Responses from MCU
RESP_USB_ACK       = 0x90
RESP_UPDATE_ACK    = 0x91
RESP_UPDATE_NACK   = 0x92
RESP_SYNC_START    = 0xFF
RESP_SYNC_1        = 0xCC
RESP_SYNC_OK       = 0x33

# Compute x and y based on angle and intensity
def angle(angle, intensity):
    # y is negative because on the Y input, UP = 0 and DOWN = 255
    x =  int((math.cos(math.radians(angle)) * 0x7F) * intensity / 0xFF) + 0x80
    y = -int((math.sin(math.radians(angle)) * 0x7F) * intensity / 0xFF) + 0x80
    return x, y

def lstick_angle(angle, intensity):
    return (intensity + (angle << 8)) << 24

def rstick_angle(angle, intensity):
    return (intensity + (angle << 8)) << 44

# Precision wait
def p_wait(waitTime):
    t0 = time.perf_counter()
    t1 = t0
    while (t1 - t0 < waitTime):
        t1 = time.perf_counter()

# Wait for data to be available on the serial port
def wait_for_data(timeout = 1.0, sleepTime = 0.1):
    t0 = time.perf_counter()
    t1 = t0
    inWaiting = ser.in_waiting
    while ((t1 - t0 < sleepTime) or (inWaiting == 0)):
        time.sleep(sleepTime)
        inWaiting = ser.in_waiting
        t1 = time.perf_counter()

# Read X bytes from the serial port (returns list)
def read_bytes(size):
    bytes_in = ser.read(size)
    return list(bytes_in)

# Read 1 byte from the serial port (returns int)
def read_byte():
    bytes_in = read_bytes(1)
    if len(bytes_in) != 0:
        byte_in = bytes_in[0]
    else:
        byte_in = 0
    return byte_in

# Discard all incoming bytes and read the last (latest) (returns int)
def read_byte_latest():
    inWaiting = ser.in_waiting
    if inWaiting == 0:
        inWaiting = 1
    bytes_in = read_bytes(inWaiting)
    if len(bytes_in) != 0:
        byte_in = bytes_in[0]
    else:
        byte_in = 0
    return byte_in

# Write bytes to the serial port
def write_bytes(bytes_out):
    ser.write(bytearray(bytes_out))
    return

# Write byte to the serial port
def write_byte(byte_out):
    write_bytes([byte_out])
    return

# Compute CRC8
# https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1gab27eaaef6d7fd096bd7d57bf3f9ba083.html
def crc8_ccitt(old_crc, new_data):
    data = old_crc ^ new_data

    for i in range(8):
        if (data & 0x80) != 0:
            data = data << 1
            data = data ^ 0x07
        else:
            data = data << 1
        data = data & 0xff
    return data

# Send a raw packet and wait for a response (CRC will be added automatically)
def send_packet(packet=[0x00,0x00,0x08,0x80,0x80,0x80,0x80,0x00], debug=False):
    if not debug:
        bytes_out = []
        bytes_out.extend(packet)

        # Compute CRC
        crc = 0
        for d in packet:
            crc = crc8_ccitt(crc, d)
        bytes_out.append(crc)
        write_bytes(bytes_out)
        # print(bytes_out)

        # Wait for USB ACK or UPDATE NACK
        byte_in = read_byte()
        commandSuccess = (byte_in == RESP_USB_ACK)
    else:
        commandSuccess = True
    return commandSuccess

# Convert DPAD value to actual DPAD value used by Switch
def decrypt_dpad(dpad):
    if dpad == DIR_U:
        dpadDecrypt = A_DPAD_U
    elif dpad == DIR_R:
        dpadDecrypt = A_DPAD_R
    elif dpad == DIR_D:
        dpadDecrypt = A_DPAD_D
    elif dpad == DIR_L:
        dpadDecrypt = A_DPAD_L
    elif dpad == DIR_U_R:
        dpadDecrypt = A_DPAD_U_R
    elif dpad == DIR_U_L:
        dpadDecrypt = A_DPAD_U_L
    elif dpad == DIR_D_R:
        dpadDecrypt = A_DPAD_D_R
    elif dpad == DIR_D_L:
        dpadDecrypt = A_DPAD_D_L
    else:
        dpadDecrypt = A_DPAD_CENTER
    return dpadDecrypt

# Convert CMD to a packet
def cmd_to_packet(command):
    cmdCopy = command
    low              =  (cmdCopy & 0xFF)  ; cmdCopy = cmdCopy >>  8
    high             =  (cmdCopy & 0xFF)  ; cmdCopy = cmdCopy >>  8
    dpad             =  (cmdCopy & 0xFF)  ; cmdCopy = cmdCopy >>  8
    lstick_intensity =  (cmdCopy & 0xFF)  ; cmdCopy = cmdCopy >>  8
    lstick_angle     =  (cmdCopy & 0xFFF) ; cmdCopy = cmdCopy >> 12
    rstick_intensity =  (cmdCopy & 0xFF)  ; cmdCopy = cmdCopy >>  8
    rstick_angle     =  (cmdCopy & 0xFFF)
    dpad = decrypt_dpad(dpad)
    left_x, left_y   = angle(lstick_angle, lstick_intensity)
    right_x, right_y = angle(rstick_angle, rstick_intensity)

    packet = [high, low, dpad, left_x, left_y, right_x, right_y, 0x00]
    # print (hex(command), packet, lstick_angle, lstick_intensity, rstick_angle, rstick_intensity)
    return packet

# Send a formatted controller command to the MCU
def send_cmd(command=NO_INPUT):
    commandSuccess = send_packet(cmd_to_packet(command))
    return commandSuccess

# Briefly send then release a command
def tap_cmd(command=NO_INPUT, pause=0.05):
    ret = send_cmd(command)
    p_wait(0.05)
    ret = ret and send_cmd()
    p_wait(pause)
    return ret

#Test all buttons except for home and capture
def testbench_btn():
    send_cmd(BTN_A) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_B) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_X) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_Y) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_PLUS) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_MINUS) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_LCLICK) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_RCLICK) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)

# Test DPAD U / R / D / L
def testbench_dpad():
    send_cmd(DPAD_U) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_R) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_D) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_L) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)

# Test DPAD Diagonals - Does not register on switch due to dpad buttons
def testbench_dpad_diag():
    send_cmd(DPAD_U_R) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_D_R) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_D_L) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_U_L) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)

# Test Left Analog Stick
def testbench_lstick():
    #Test U/R/D/L
    send_cmd(BTN_LCLICK) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(LSTICK_U) ; p_wait(0.5)
    send_cmd(LSTICK_R) ; p_wait(0.5)
    send_cmd(LSTICK_D) ; p_wait(0.5)
    send_cmd(LSTICK_L) ; p_wait(0.5)
    send_cmd(LSTICK_U) ; p_wait(0.5)
    send_cmd(LSTICK_CENTER) ; p_wait(0.5)

    # 360 Circle @ Full Intensity
    for i in range(0,721):
        cmd = lstick_angle(i + 90, 0xFF)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(LSTICK_CENTER) ; p_wait(0.5)

    # 360 Circle @ Partial Intensity
    for i in range(0,721):
        cmd = lstick_angle(i + 90, 0x80)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(LSTICK_CENTER) ; p_wait(0.5)

# Test Right Analog Stick
def testbench_rstick():
    #Test U/R/D/L
    send_cmd(BTN_RCLICK) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(RSTICK_U) ; p_wait(0.5)
    send_cmd(RSTICK_R) ; p_wait(0.5)
    send_cmd(RSTICK_D) ; p_wait(0.5)
    send_cmd(RSTICK_L) ; p_wait(0.5)
    send_cmd(RSTICK_U) ; p_wait(0.5)
    send_cmd(RSTICK_CENTER) ; p_wait(0.5)

    # 360 Circle @ Full Intensity
    for i in range(0,721):
        cmd = rstick_angle(i + 90, 0xFF)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(RSTICK_CENTER) ; p_wait(0.5)

    # 360 Circle @ Partial Intensity
    for i in range(0,721):
        cmd = rstick_angle(i + 90, 0x80)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(RSTICK_CENTER) ; p_wait(0.5)

# Test Packet Speed
def testbench_packet_speed(count=100, debug=False):
    sum = 0
    min = 999
    max = 0
    avg = 0
    err = 0

    for i in range(0, count + 1):

        # Send packet and check time
        t0 = time.perf_counter()
        status = send_packet()
        t1 = time.perf_counter()

        # Count errors
        if not status:
            err += 1
            print('Packet Error!')

        # Compute times
        delta = t1 - t0
        if delta < min:
            min = delta
        if delta > max:
            max = delta
        sum = sum + (t1 - t0)

    avg = sum / i
    print('Min =', '{:.3f}'.format(min), 'Max =', '{:.3}'.format(max), 'Avg =', '{:.3f}'.format(avg), 'Errors =', err)

def testbench():
    testbench_btn()
    testbench_dpad()
    testbench_lstick()
    testbench_rstick()
    testbench_packet_speed()
    return

# Force MCU to sync
def force_sync():
    # Send 9x 0xFF's to fully flush out buffer on device
    # Device will send back 0xFF (RESP_SYNC_START) when it is ready to sync
    write_bytes([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF])

    # Wait for serial data and read the last byte sent
    wait_for_data()
    byte_in = read_byte_latest()

    # Begin syn..
    inSync = False
    if byte_in == RESP_SYNC_START:
        write_byte(COMMAND_SYNC_1)
        byte_in = read_byte()
        if byte_in == RESP_SYNC_1:
            write_byte(COMMAND_SYNC_2)
            byte_in = read_byte()
            if byte_in == RESP_SYNC_OK:
                inSync = True
    return inSync

# Start MCU syncing process
def sync():
    inSync = False

    # Try sending a packet
    inSync = send_packet()
    if not inSync:
        # Not in sync: force resync and send a packet
        inSync = force_sync()
        if inSync:
            inSync = send_packet()
    return inSync
# ------------------------ Pokemon Sword & Shield -------------------------
def macro_breed_for_shiny ():
    print('''
Assumptions:
 * Near wild area breeder
 * Egg ready at breeder
 * Option Send to Boxes set to Manual
 * Text speed set to Fast
 * Party full, with egg speed booster in position 1
 * Defaults to filling 1 box (30 eggs). Override with --iterations=N
''')
    macro_counter = 0
    box_spaces = 30
    if args.iterations > 0:
        box_spaces = args.iterations
    print ("in macro_breed_for_shiny")
    for i in range(box_spaces):
        time.sleep(.4)
        print("Loop #{} of {}".format(macro_counter + 1, box_spaces))
        teleport_to_current_location()
        gotoBreader()
        getEggFromBreader()
        teleport_to_current_location()
        gotoEmptyArea()
        bike_loop_mashing_a(90)
        macro_counter = macro_counter + 1
    send_cmd() ; p_wait(0.1)

def macro_loto_id():
    print('''
Assumptions:
 * Nintendo Online VS ranked battle glitch active
 * Standing in front of Rotom PC
 * Initializes year to 2000 (Switch only allows years 2000-2060)
 * If you can't play or don't win, the box will open. Don't worry about it.
 * Ignores --iterations
''')
    # Flush controller
    send_cmd()
    p_wait(0.5)

    # Roll year back to 2000
    skip_day(False)

    # Check Loto-ID every year 2000-2060
    for yr in range(2000, 2061):
        print('Play Loto-ID {}'.format(yr))
        # Open PC
        tap_cmd(BTN_A, 0.5)

        # What can I help you with? Pass
        tap_cmd(BTN_A, 0.5)
        # What can I help you with? Move to Try Loto-ID
        tap_cmd(DPAD_D, 0.1)
        # What can I help you with? Select Try Loto-ID
        tap_cmd(BTN_A, 1.0)
        # I've now connected to the servers at the Loto-ID Center! Pass
        tap_cmd(BTN_A, 1.0)
        # We'll draw a number... Pass
        tap_cmd(BTN_A, 0.5)
        # ...win fabulous prizzzes! Pass
        tap_cmd(BTN_A, 1.0)
        # Ready to save your progress and try your luck? Select Yes
        tap_cmd(BTN_A, 1.0)

        # Mash the B button to get through the Loto-ID and back out of the PC
        for sec in range(30):
            send_cmd()
            p_wait(0.4)
            send_cmd(BTN_B)
            p_wait(0.1)

        # Skip the current day
        skip_day()

def macro_mash_a ():
    print('''
Assumptions:
 * Defaults to one second at a rate of 2 taps per second. Override with --iterations=N
''')
    seconds = 1
    if args.iterations > 0:
        seconds = args.iterations
    print ("in macro_mash_a")
    mash_btn(BTN_A, seconds)

def macro_next_den_day():
    print('''
Assumptions:
 * Initial den screen open with "Invite Others" button selected
 * Den spawned using Wishing Stone
 * Defaults to 1 skipped den day. Override with --iterations=N (e.g. shiny den farming loops with --iterations=3)
''')
    skips = 1
    if args.iterations > 0:
        skips = args.iterations

    # Flush controller
    send_cmd()
    p_wait(0.5)

    for i in range(skips):
        print("Skip #{} of {}".format(i + 1, skips))

        # Invite Others
        tap_cmd(BTN_A, 4)

        # Skip the current day
        skip_day()

        # Quit Invite Others
        tap_cmd(BTN_B, 1)
        # Yes
        tap_cmd(BTN_A, 4.5)

        # Open den, collecting watts
        for i in range(3):
            tap_cmd(BTN_A, 0.5)

def macro_release_box():
    print('''
Assumptions:
 * Entire current box will be released
 * All 30 box spaces contain a pokemon
 * Upper left pokemon selected
 * Text Speed set to fastest
 * Selection Mode set to Select (not Multipurpose or Multiselect)
 * Ignores --iterations
''')
    for row in range(5):
        for col in range(6):
            # Snake through the box, alternating direction each row
            if 0 == (row % 2):
                # L to R row
                col_dir = DPAD_R
            else:
                # R to L row
                col_dir = DPAD_L
            print('Releasing pokemon in row #{}, column #{}'.format(row+1, col+1))
            # Open current pokemon's menu
            tap_cmd(BTN_A, 0.5)
            # What do you want to do with it? Move to Release
            for i in range(2):
                tap_cmd(DPAD_U, 0.1)
            # What do you want to do with it? Press Release
            tap_cmd(BTN_A, 1)
            # Do you really want to release this pokemon? Move to Yes
            tap_cmd(DPAD_U, 0.1)
            # Do you really want to release this pokemon? Press Yes
            tap_cmd(BTN_A, 1.5)
            # Pokemon was released. Bye-bye pokemon!
            tap_cmd(BTN_A, 0.5)
            # Move to the next pokemon in this row unless this is the last column
            if 5 != col:
                tap_cmd(col_dir, 0.1)
        # Done with the row, move down to the next one unless this is the last row
        if 4 != row:
            tap_cmd(DPAD_D, 0.1)

def macro_skip_day():
    print('''
Assumptions:
 * Nintendo Online VS ranked battle glitch active
 * Defaults to 1 skipped den day. Override with --iterations=N
''')
    skips = 1
    if args.iterations > 0:
        skips = args.iterations

    # Flush controller
    send_cmd()
    p_wait(0.5)

    for i in range(skips):
        print("Skip #{} of {}".format(i + 1, skips))

        # Skip the current day
        skip_day()

def teleport_to_current_location():
    print("teleport_to_current_location")
    time.sleep(0.5)
    #Bring up menu
    send_cmd(BTN_X) ; time.sleep(0.1) ;  send_cmd(); time.sleep(0.5)
    #Select bottom left
    send_cmd(LSTICK_L); time.sleep(0.8); send_cmd(LSTICK_CENTER) ; time.sleep(0.1) 
    send_cmd(LSTICK_D); time.sleep(0.8); send_cmd(LSTICK_CENTER) ; time.sleep(0.1) 
    send_cmd(BTN_A); time.sleep(0.1) ;  send_cmd();
    #wait for map to load and select location
    time.sleep(4)
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd(); time.sleep(1)
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd(); 
    time.sleep(4)
    return True

def gotoBreader():
    print("gotoBreader")
    send_cmd(RSTICK_R) ; p_wait(2) ;  send_cmd(RSTICK_CENTER) ; time.sleep(0.1)  
    send_cmd(LSTICK_U) ; p_wait(0.5) ;  send_cmd(LSTICK_CENTER) ; time.sleep(0.1)  
    send_cmd(LSTICK_R) ; p_wait(0.5) ;  send_cmd(LSTICK_CENTER) ; time.sleep(0.1)  
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd() ; time.sleep(0.1)  
    return True

def gotoEmptyArea():
    print("gotoEmptyArea")
    send_cmd(RSTICK_R) ; time.sleep(.25) ;  send_cmd(RSTICK_CENTER) ; time.sleep(0.1)  
    send_cmd(LSTICK_U) ; time.sleep(1) ;  send_cmd(LSTICK_CENTER) ; time.sleep(0.1)  
    send_cmd(LSTICK_R) ; time.sleep(1.0) ;  send_cmd(LSTICK_CENTER) ; time.sleep(0.1)  
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd() ; time.sleep(0.1)  
    return True

def gotoEmptyArea2():
    print("gotoEmptyArea")
    send_cmd(RSTICK_R) ; time.sleep(.3) ;  send_cmd(RSTICK_CENTER) ; time.sleep(0.1)  
    send_cmd(LSTICK_U) ; time.sleep(0.4) ;  send_cmd(LSTICK_CENTER) ; time.sleep(0.1)  
    send_cmd(LSTICK_R) ; time.sleep(0.7) ;  send_cmd(LSTICK_CENTER) ; time.sleep(0.1)  
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd() ; time.sleep(0.1)  
    return True

def getEggFromBreader():
    print("getEggFromBreader")
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd();  time.sleep(3)  
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd();  time.sleep(5)  
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd();  time.sleep(4)
    # select pokemon to swap prompt
    time.sleep(1.5)
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd();  time.sleep(1)  
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd();  time.sleep(0.4)    
    # select pokemon to swap box screen
    time.sleep(1.5)
    send_cmd(DPAD_D) ; time.sleep(0.2) ;  send_cmd();  time.sleep(1)  
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd();  time.sleep(1)  
    # egg will be added to party prompt
    time.sleep(2)
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd();  time.sleep(0.4)
    # raise it well prompt
    time.sleep(2)
    send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd();  time.sleep(0.3)
    return True

def bike_loop_mashing_a(seconds):
    print("bike_loop_mashing_a")
    total_seconds = 0

    # Loop on the bike, tapping A every second
    while total_seconds < seconds:
        send_cmd(LSTICK_R + RSTICK_L)
        p_wait(0.9)
        send_cmd(LSTICK_R + RSTICK_L + BTN_A)
        p_wait(0.1)
        total_seconds = total_seconds+1
    print ("Bike loop seconds: #{}".format(total_seconds))
    # Clear controller
    send_cmd()
    p_wait(0.05)

    return True

def move_cursor_l(delta_x, delta_y):
    print("Move left stick cursor by {}, {}".format(delta_x, delta_y))
    # Default x to move positively, support negative movement
    x_btn = LSTICK_R
    if delta_x < 0:
        x_btn = LSTICK_L
        delta_x = abs(delta_x)

    # Default y to move positively, support negative movement
    y_btn = LSTICK_U
    if delta_y < 0:
        y_btn = LSTICK_D
        delta_y = abs(delta_y)

    # Move x
    for i in range(delta_x):
        tap_cmd(x_btn, 0.1)

    # Move y
    for i in range(delta_y):
        tap_cmd(y_btn, 0.1)

def move_cursor_r(delta_x, delta_y):
    print("Move right stick cursor by {}, {}".format(delta_x, delta_y))
    # Default x to move positively, support negative movement
    x_btn = RSTICK_R
    if delta_x < 0:
        x_btn = RSTICK_L
        delta_x = abs(delta_x)

    # Default y to move positively, support negative movement
    y_btn = RSTICK_U
    if delta_y < 0:
        y_btn = RSTICK_D
        delta_y = abs(delta_y)

    # Move x
    for i in range(delta_x):
        tap_cmd(x_btn, 0.1)

    # Move y
    for i in range(delta_y):
        tap_cmd(y_btn, 0.1)

def mash_btn (btn, seconds):
    print ("Mashing button for {} seconds".format(seconds))
    for i in range(seconds * 2):
        send_cmd(btn)
        p_wait(0.1)
        send_cmd()
        p_wait(0.4)
    return True

def mashA (num):
    print ("mashing A")
    for i in range(num):
        send_cmd(BTN_A) ; time.sleep(0.1) ;  send_cmd();  time.sleep(0.5)
    return True

def skip_day(forward=True):
    print("skip_day")

    # Home screen
    tap_cmd(BTN_HOME, 1)
    # Bottom row buttons
    tap_cmd(DPAD_D, 0.1)
    # Over to System Settings
    for i in range(4):
        tap_cmd(DPAD_R, 0.1)
    # Open Sytem Settings
    tap_cmd(BTN_A, 0.5)
    # To Sytem Settings > System
    send_cmd(DPAD_D)
    p_wait(2.0)
    send_cmd()
    p_wait(0.1)
    # Into Sytem Settings > System
    tap_cmd(BTN_A, 0.5)
    # To System > Date and Time
    for i in range(4):
        tap_cmd(DPAD_D, 0.1)
    # Into System > Date and Time
    tap_cmd(BTN_A, 0.5)
    # To System > Date and Time > Date and Time
    for i in range(2):
        tap_cmd(DPAD_D, 0.1)
    # Into System > Date and Time > Date and Time control
    tap_cmd(BTN_A, 0.5)
    # To Year control
    for i in range(2):
        tap_cmd(DPAD_R, 0.1)
    # Move year forward or revert back to 2000 (the lowest)
    if forward:
        # Increase Year (caps at 2060, do something to reset by then)
        tap_cmd(DPAD_U, 0.1)
    else:
        # Decrease year back (lower cap is 2000)
        send_cmd(DPAD_D)
        p_wait(5)
        send_cmd()
        p_wait(0.05)
    # Accept new Year
    for i in range(5):
        tap_cmd(BTN_A, 0.1)
    p_wait(0.5)
    # Home screen
    tap_cmd(BTN_HOME, 0.5)
    # Re-launch Pokemon
    tap_cmd(BTN_A, 1.0)


# -------------------------------------------------------------------------
if __name__ == "__main__":
    ser = serial.Serial(port=args.port, baudrate=19200,timeout=1)
    # ser = serial.Serial(port=args.port, baudrate=31250,timeout=1)
    # ser = serial.Serial(port=args.port, baudrate=40000,timeout=1)
    # ser = serial.Serial(port=args.port, baudrate=62500,timeout=1)
    # ser = serial.Serial(port=args.port, baudrate=115200,timeout=1)

    # Attempt to sync with the MCU
    if not sync():
        print('Could not sync!')
        raise SystemExit(1)
    p_wait(1.5)

    if not send_cmd():
        print('Packet Error!')
        raise SystemExit(1)
    p_wait(1.5)

    try:
        # Python doesn't have a switch/case feature but does allow functions as dictionary values
        # This is our macro argument to function execution not-switch
        arg_macro_functions = {
            'breed_for_shiny': macro_breed_for_shiny,
            'force_sync': force_sync,
            'loto_id': macro_loto_id,
            'mash_a': macro_mash_a,
            'next_den_day': macro_next_den_day,
            'release_box': macro_release_box,
            'skip_day': macro_skip_day
        }
        arg_macro_function = arg_macro_functions.get(args.macro)
        arg_macro_function()

        # testbench()
        # testbench_packet_speed(1000)

    except KeyboardInterrupt:
        pass

    finally:
        # Tidy up after ourselves
        send_cmd()
        p_wait(0.05)
        ser.close
        raise SystemExit(0)


