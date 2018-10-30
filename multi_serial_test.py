import subprocess
import serial
import threading

MAX_READ_SIZE = 256
'''def forward_to_serial():
    while True:
        data = 'hello'#proc.stdout.read1(MAX_READ_SIZE)
        if not data:
            break  # EOF
        ser.write(data)
        input('wait...')'''

def forward_to_cmd():
    while True:
        print(ser2.read())
        #proc.stdin.write(ser.read())
        proc.stdin.flush()

#ser = serial.Serial('/dev/ttyv9', timeout = 100)

ser2 = serial.Serial('/dev/ptyv9', timeout = 100)

proc = subprocess.Popen("cat", stdout=subprocess.PIPE, stdin=subprocess.PIPE)

#fwc = threading.Thread(target = forward_to_serial)
#fwc.start()

input('wait...')
forward_to_cmd()