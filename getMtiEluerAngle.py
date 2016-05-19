# coding:utf-8

import serial
import struct
import string


def hex_show(argv):
    result = ''
    hLen = len(argv)
    for i in xrange(hLen):
        hvol = ord(argv[i])
        hhex = '%02x' % hvol
        result += hhex
    # print 'hexShow:', result.upper()
    return result.upper()


def Get_MTi_Euler(ser):
    try:
        # set the frame header
        # (refer to the MT Low-Level Communication Protocol Documentation.pdf)
        frame_header = "FAFF36"
        # frame_header = [250, 255, 54]
        while True:
            read_data = ser.read(40)
            read_hex_data = hex_show(read_data)
            location = string.find(read_hex_data, frame_header)
            if location >= 0:
                # beacuse the every data will transfer to a two byte hex
                # the offest after the location is doubled

                # define the roll - pitch - yaw angle
                roll_bytes = read_hex_data[location + 14:location + 14 + 8]
                '''
                # this is the important thing :
                # we can transfer the 8 bits hex data to the single precision float
                # like this:
                # >>> import struct
                # >>> struct.unpack('!f', '41973333'.decode('hex'))[0]
                # 18.899999618530273
                # >>> struct.unpack('!f', '41995C29'.decode('hex'))[0]
                # 19.170000076293945
                '''
                roll = struct.unpack('!f', roll_bytes.decode('hex'))[0]
                # print roll
                pitch_bytes = read_hex_data[
                    location + 14 + 8:location + 14 + 8 + 8]
                pitch = struct.unpack('!f', pitch_bytes.decode('hex'))[0]
                # print pitch
                yaw_bytes = read_hex_data[
                    location + 14 + 8 + 8:location + 14 + 8 + 8 + 8]
                yaw = struct.unpack('!f', yaw_bytes.decode('hex'))[0]
                # print yaw

                return (roll, pitch, yaw)
            else:
                print location
                ser.flushInput()
                ser.flushOutput()

        # ser.close()
    except Exception, e:
        raise e


def main():
    # setting the serial
    try:
        pass
        ser = serial.Serial()
        ser.baudrate = 115200
        ser.port = 'COM3'
        ser.bytesize = 8
        ser.parity = 'N'
        ser.stopbits = 2
        print ser
        ser.open()
        while True:
            pass
            (roll_angle, pitch_angle, yaw_angle) = Get_MTi_Euler(ser)
            print roll_angle
            print pitch_angle
            print yaw_angle
    except Exception, e:
        # raise e
        print e


if __name__ == '__main__':
    main()
