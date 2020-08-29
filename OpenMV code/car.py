from pyb import Pin, Timer
inverse_left=True  #change it to True to inverse left wheel
inverse_right=False #change it to True to inverse right wheel

ain1 =  Pin('P0', Pin.OUT_PP)#Left back
ain2 =  Pin('P1', Pin.OUT_PP)#Left go
bin1 =  Pin('P2', Pin.OUT_PP)#Right go
bin2 =  Pin('P3', Pin.OUT_PP)#Right back
ain1.low()
ain2.low()
bin1.low()
bin2.low()

pwma = Pin('P7')#ENA
pwmb = Pin('P8')#ENB
tim = Timer(4, freq=1000)
ch1 = tim.channel(1, Timer.PWM, pin=pwma)
ch2 = tim.channel(2, Timer.PWM, pin=pwmb)
ch1.pulse_width_percent(0)
ch2.pulse_width_percent(0)

def run(left_speed, right_speed):
    if inverse_left==True:
        left_speed=(-left_speed)
    if inverse_right==True:
        right_speed=(-right_speed)
    if left_speed < 0:
        ain1.low()
        ain2.high()
    else:
        ain1.high()
        ain2.low()
    ch1.pulse_width_percent(min(abs(left_speed),70))

    if right_speed < 0:
        bin1.low()
        bin2.high()
    else:
        bin1.high()
        bin2.low()
    ch2.pulse_width_percent(min(abs(right_speed),70))
