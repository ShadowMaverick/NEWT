import time
import serial

ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=250000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        timeout=0)
print("Connected to: " + ser.portstr)

time.sleep(5)

msg = "Testing testing 123\r\n"
time.sleep(1)
print("\nSending message: " + msg)
time.sleep(1)
ser.write(msg.encode())

print("\nReceiving data:")
while 1:
    seq = []
    for char in ser.read():
        seq.append(chr(char))
    while len(seq) > 0:
        for char in ser.read():
            seq.append(chr(char))
        if seq[-1] == '\n':
            print(''.join(str(v) for v in seq).strip())
            break
    time.sleep(0.1)
