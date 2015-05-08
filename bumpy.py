# Bumpy Motor Test
# Moves: Forward, Reverse, turn Right, turn Left, Stop - then repeat
# Press Ctrl-C to stop
#
# Run using: sudo python motorTest.py

import sys
import tty
import termios
import time
import RPi.GPIO as GPIO
#from Adafruit_PWM_Servo_Driver import PWM

FREQ = 50
SPEED = 10

# Pins 37, 35 Left Motor  (FW, BW)
# Pins 31, 33 Right Motor (FW, BW)
LF, LB = 35, 37
RF, RB = 31, 33
lf, lb, rf, rb = None, None, None, None
 
def init():
  global lf,lb,rf,rb
  #use physical pin numbering
  GPIO.setmode(GPIO.BOARD)
  GPIO.setwarnings(False)

  lf = _init_pin(LF)
  lb = _init_pin(LB)
  rf = _init_pin(RF)
  rb = _init_pin(RB)

def cleanup():
  stop()
  GPIO.cleanup()

def _init_pin(pin):
  GPIO.setup(pin, GPIO.OUT)
  p = GPIO.PWM(pin, FREQ)
  p.start(0)
  return p

def _switch_on(pwm, *signals):
  assert all([lf,lb,rf,rb]), "init() must be called first!"
  for s in signals:
    s.ChangeDutyCycle(pwm)

def _switch_off(*signals):
  assert all([lf,lb,rf,rb]), "init() must be called first!"
  for s in signals:
    s.ChangeDutyCycle(0)

def step(f, steps):
  f()
  time.sleep(steps)

def forward(speed = SPEED):
  _switch_off(lb, rb)
  _switch_on(speed, lf, rf)

def reverse(speed = SPEED):
  _switch_off(lf, rf)
  _switch_on(speed, lb, rb)

def spin_left(speed = SPEED):
  _switch_off(lb, rf)
  _switch_on(speed, lf, rb)

def spin_right(speed = SPEED):
  _switch_off(lf, rb)
  _switch_on(speed, lb, rf)

def left(speed = SPEED):
  _switch_off(lb, rf)
  _switch_on(speed, lf)

def right(speed = SPEED):
  _switch_off(lf, rb)
  _switch_on(speed, rf)

def stop():
  _switch_off(lf,rf,lb,rb)
   

def readkey(getchar_fn=None):
  '''Read single character'''
  def readchar():
    '''Read single char with stdin in raw mode'''
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
      raise KeyboardInterrupt
    return ch
    
  getchar = getchar_fn or readchar
  c1 = getchar()
  if ord(c1) != 0x1b:
    return c1
  c2 = getchar()
  if ord(c2) != 0x5b:
    return c1
  c3 = getchar()
  return chr(0x10 + ord(c3) - 65)  # 16=Up, 17=Down, 18=Right, 19=Left arrows

def motor_loop():
  while True:
    step(forward, 3)
    print('forward')
    step(reverse, 3)
    print('reverse')
    step(spin_right, 3)
    print('right')
    step(spin_left, 3)
    print('left')
    step(stop, 3)
    print('stop')

def f(*args):
  pass

def cursor_loop():
  last_func = f
  speed = SPEED

  while True:
    keyp = readkey()
    if keyp == 'q' or ord(keyp) == 16:
      forward(speed)
      print 'Forward', speed
      last_func = forward
    elif keyp == 'a' or ord(keyp) == 17:
      reverse(speed)
      print 'Reverse', speed
      last_func = reverse
    elif keyp == 'p' or ord(keyp) == 18:
      right(speed)
      print 'Right', speed
      last_func = right
    elif keyp == 'o' or ord(keyp) == 19:
      left(speed)
      print 'Left', speed
      last_func = left
    elif keyp == 'i':
      spin_right(speed)
      print 'Spin Right', speed
      last_func = spin_right
    elif keyp == 'u':
      spin_left(speed)
      print 'Spin Left', speed
      last_func = spin_left
    elif keyp == '.' or keyp == '>':
      speed = min(100, speed+5)
      print 'Speed+', speed
      last_func(speed)
    elif keyp == ',' or keyp == '<':
      speed = max (0, speed-5)
      print 'Speed-', speed
      last_func(speed)
    elif keyp == ' ':
      stop()
      print 'Stop'
    elif ord(keyp) == 3:
      break

if __name__ == '__main__':
  init()
  try:
    cursor_loop()
  except KeyboardInterrupt:
    cleanup()
    sys.exit()
