print('GCS DDR2019')

import serial
import time

ser = serial.Serial()
ser.baudrate = 9600
# ser.port = '/dev/rfcomm3'
ser.port = '/dev/ttyACM0'
# ser.timeout = 0.15

ser.open()

def crc8(data):
    polynomial = 0x07
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
    return crc & 0xFF

def send_command(pwm_L_, pwm_R_):
    pwm_L = pwm_L_ & (0b1111111111111111)
    pwm_R = pwm_R_ & (0b1111111111111111)

    pwm_Lh = pwm_L >> 8
    pwm_Ll = pwm_L & (0b11111111)
    pwm_Rh = pwm_R >> 8
    pwm_Rl = pwm_R & (0b11111111)

    tx_pkt = [0x15, 0xEC, 0x00, pwm_Lh, pwm_Ll, pwm_Rh, pwm_Rl, 0x00, 0x04, 0xD2]

    crc = crc8(bytearray(tx_pkt[2:7]))
    tx_pkt[7] = crc

    tx_pkt = bytearray(tx_pkt)

    try:
        ser.write(tx_pkt)
        print(pwm_L_, ',', pwm_R_, '--->')
    except:
        print('XXX, XXX --->')

def receive_command():
    try:
        rx_pkt = ser.readline()
        rx_pkt = list(rx_pkt)
        index_comma = 0
        n = len(rx_pkt)
        for i in range(0,n):
            if(rx_pkt[i]==44):
                index_comma = i
                break
        
        pwl = []
        pwr = []

        for i in range(0,index_comma):
            pwl.append(rx_pkt[i]-48)
        for i in range(index_comma+1, n-1):
            pwr.append(rx_pkt[i]-48)
        
        pl = 0
        pr = 0
        
        for i in range(0,len(pwl)):
            pl = pl*10 + pwl[i]
        
        for i in range(0,len(pwr)):
            pr = pr*10 + pwr[i]

        print(rx_pkt, pwl, pwr, pl , pr, '<---')
    except:
        print('XXX, XXX <---', "RX Error!")

t0 = time.time()

while 1:
    pwm_L = 170*0
    pwm_R = 119*0
    send_command(pwm_L, pwm_R)
    receive_command()

    time.sleep(0.1)

ser.close()
