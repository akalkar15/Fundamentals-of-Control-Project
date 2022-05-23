import numpy as np
import inputs
import serial
import time

#deviceName = "/dev/input/by-id/usb-Logitech_Wireless_Gamepad_F710_717F9001-event-joystick"
"""
Initialize Gamepad and microcontroller
"""

pad = inputs.devices.gamepads

mcu = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)

mcu.write(b'\x45\x41\xbb')
mcu.write(b'\x45\x42\xb7')

"""
Steering PWM Values
"""
STEERING_LEFT_PWM = 126
STEERING_RIGHT_PWM = 238
STEERING_ZERO_PWM = 183

"""
Throttle PWM Values
"""
THROTTLE_FORWARD_PWM = 217
THROTTLE_REVERSE_PWM = 157
THROTTLE_ZERO_PWM = 187

"""
Functions for mapping Steering and Throttle
"""

throttleMapRange = lambda x: int(((THROTTLE_FORWARD_PWM + THROTTLE_REVERSE_PWM)/2 - THROTTLE_ZERO_PWM)*np.square(x) + ((THROTTLE_FORWARD_PWM-THROTTLE_REVERSE_PWM)/2)*x + THROTTLE_ZERO_PWM)
steeringMapRange = lambda x: int(((STEERING_LEFT_PWM + STEERING_RIGHT_PWM)/2 - STEERING_ZERO_PWM)*np.square(-x) + ((STEERING_LEFT_PWM-STEERING_RIGHT_PWM)/2)*(-x) + STEERING_ZERO_PWM)

"""
Steering and Throttle Mapping Commands
"""

def steeringCommand(x):
    pwm = steeringMapRange(x)
    data = b'\x45\x42' + pwm.to_bytes(1, 'big')
    mcu.write(data)

def throttleCommand(x):
    pwm = throttleMapRange(x)
    data = b'\x45\x41' + pwm.to_bytes(1, 'big')
    mcu.write(data)

# Step function
step = lambda t: np.heaviside(t, 1/2)


"""
Functions for later use (do not use for warm-up)
"""

def decodeIMUData(data):
    if data[0] == b'Q':
        acc_raw = data[1:7].astype(np.int16)
        acc = (acc_raw[::2] | (acc_raw[1::2] << 8)) / 2**15 * 16 * g
    if data[0] == b'R':
        gyro_raw = data[1:7].astype(np.int16)
        gyro = (gyro_raw[::2] | (gyro_raw[1::2] << 8)) / 2**15 * 2000
    if data[0] == b'T':
        mag_raw = data[1:7].astype(np.int16)
        mag = (mag_raw[::2] | (mag_raw[1::2] << 8)) / 2**15
    return acc, gyro, mag

def EKF(X_hat, Y, P, U, t):
    F = f_grad(t, X_hat, U)
    H = h_grad(X_hat)
    X_hat_prev = f(t, X_hat, U)
    P_prev = np.matmul(F, np.matmul(P, F.T)) + Q
    Kg = np.matmul(P_prev, np.matmul(H.T, inv(R + np.matmul(H, np.matmul(P_prev, H.T)))))
    X_hat_next = X_hat_prev + np.matmul(Kg, (Y - h(X_hat_prev)))
    P_next = np.matmul((np.eye(len(X_hat)) - np.matmul(Kg, H)), P_prev)
    return X_hat_next, P_next

"""
Initial Accelerate and stop function
"""
# Stop time
T = 3.1
# Throttle value as function of time
initialThrottle = lambda t: 0.25*(1 - step(t - T))
# Steering value as function of time
initialSteering = lambda t: 0.0*np.sin(2*np.pi*t/5)

"""
Your functions for throttle and steering after initial acceleration
"""
PHASE1_END = 0 
PHASE2_END = 0.2 #0.33
PHASE3_END = 0.45 #0.8
PHASE4_END = 0.8 #1.30

def decel_steer(t):
  return 0
def decel_throttle(t):
  #ramp_time = 0.4
  return 0.2 #100 * (ramp_time - t)

def right_steer(t):
  return 0.9
def right_throttle(t):
  return 0
  #changed this from 0.2 to 0.4

def left_steer(t):
    return -0.9
def left_throttle(t):
  return 0.1 # -0.5

def u(t, start, end):
  return (t >= start and t < end)

myInputs = []#pre-planned values

# throttle values after initial acceleration
myThrottle = lambda t: decel_throttle(t) * u(t, 0, PHASE1_END) + right_throttle(t) * u(t, PHASE1_END, PHASE2_END) + left_throttle(t)  * u(t, PHASE2_END, PHASE3_END) + decel_throttle(t) * u(t, PHASE3_END, PHASE4_END)
# steering values after initial acceleration
mySteering = lambda t: decel_steer(t) * u(t, 0, PHASE1_END) + \
                       right_steer(t) * u(t, PHASE1_END, PHASE2_END) + \
                       left_steer(t)  * u(t, PHASE2_END, PHASE3_END) + \
                       decel_steer(t) * u(t, PHASE3_END, PHASE4_END)                         
"""
Final throttle and steering functions
"""
finalThrottle = lambda t: initialThrottle(t) + myThrottle(t - T)*step(t - T)

finalSteering = lambda t: initialSteering(t) + mySteering(t - T)*step(t - (T))

"""
Main System Loop
"""
i = 0
Ts = 0.05
t0 = time.time()
while True:
    t = time.time() - t0
    print(finalThrottle(t))
    throttle = finalThrottle(t)
    steering = finalSteering(t)
    print("time: ", t, "throttle: ", throttle, "steering: ", steering)
    throttleCommand(throttle)
    steeringCommand(steering)
    time.sleep(Ts)

    if t > 10:
        mcu.write(b'\x45\x41\xbb')
        mcu.write(b'\x45\x42\xb7')
        break

'''
Lessons learnt so far:
NORTH button is actually X button: if you want to use our save functionality, 
make sure to use X rather Y.

In driveWithJoystick, input.get_gamepad() WAITS for game_pad input. If no 
command arrives, the function does not run and therefore the function hangs.

Consequence: You cannot record a test run on the joystick in real-time
Potential solution: record timestamps in addition to the game_pad values.
Then, interpolate the values later to match the 0.05s period of the
microcontroller 

Future Work:
I see two ways of completing this project:
1) Do trial and error testing with test steering and throttle values on
carDrive. The car should have about 1 second of time to avoid the pedestrians.
Since the microcontroller ticks at 20Hz, you just need to come up with 20 
steering and throttle values.

2) Try to record a good run on the joystick with timestamps. Record the
timestamps then interpolate the values across time to come up with the 20+
values needed.
'''
