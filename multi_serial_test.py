import subprocess
import serial
import threading

MAX_READ_SIZE = 256
def forward_to_serial():
    while True:
        data = proc.stdout.read1(MAX_READ_SIZE)
        if not data:
            break  # EOF
        ser.write(data)

def forward_to_cmd():
    while True:
        proc.stdin.write(ser.read())
        proc.stdin.flush()

ser = serial.Serial('COM27', timeout = 100)

proc = subprocess.Popen("cat", stdout=subprocess.PIPE, stdin=subprocess.PIPE)

fwc = threading.Thread(target = forward_to_cmd)
fwc.start()

forward_to_serial()