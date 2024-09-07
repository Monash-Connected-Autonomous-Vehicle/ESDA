import struct
import serial
import serial.tools.list_ports
import math

serial_port = '/dev/ttyUSB0'
baud_rate = 115200
serial_connection = serial.Serial(serial_port, baud_rate, timeout=1, bytesize=8)



data = 69


packet = bytearray()
packet.append(69)
packet.append(data)

## for every byte n, it will carry 2^(n) to 2^(n+8)

# 16 bits of decimal places
# mult each number by 2^16

val = 42069

#1 - [bit 1-8]: 0 - 255

#2 - [bit 9-16]: 256 - 65,535

#3 - [bit 17-24]: 65,536 - 16,777,216

# 2^ of: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,11,12,13,14,15,16,17,18,19,20,21,22,23

# 420   [0, 1, 2, 3, 4, 5, 6, 7][0, 1, 2, 3, 4, 5, 6, 7][0, 1, 2, 3, 4, 5, 6, 7]
#       [      byte 0 x1       ][     byte 1 x256      ][   byte 2 - x65536    ]


# bytes: 
# 0 - 1/65536 - 1/256
# 1 - 1/255 - 1/2
# 2 - 0 - 255 
# 3 - 256 - 65535
# 4 - 65536 - 2^24
# 5 - 2^25 - 2^32 
# 6 - 2^33 - 2^39
# 7 - 2^40 - 2^47


                                    


# 420 2^9 = 512
# 2^(byte_no * 8 - 7)
# 2^(      2 * 8 - 7) = 512

# 420 / 512 < 1 
# drop down one byte
# 

if False:
    curr_val= val

    for byte_num in range(8, -1, -1): #64 bits
        print(byte_num)
        # CHK within byte
        if curr_val / (2**(byte_num*8)) >= 1:
            # rounds to the minimum bit of the current byte, expectation is that figures 
            # get caught in lower bytes
            working_val = math.floor(curr_val / (2**(byte_num*8)))
            packet.append(working_val)
            curr_val = curr_val - (working_val * (2**(byte_num*8)))
            print(working_val)
            print(curr_val)
        else:
            packet.append(0)



    print(packet)



try:
    ##serial_connection.write(binary_out)
    ##serial_connection.write(binary_data.encode('utf-8'))
    serial_connection.write(packet)
except serial.SerialException as e:
    print("bruh: " + e)