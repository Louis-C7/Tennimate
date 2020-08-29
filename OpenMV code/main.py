#main.py
# Blob Detection Example
#
# This example shows off how to use the find_blobs function to find color
# blobs in the image. This example in particular looks for dark green objects.

import sensor, image, time
from pid import PID
from my_utils import expanded_roi
import pyb, ustruct
import ujson
from pyb import Pin, Timer
import car
# You may need to tweak the above settings for tracking green things...
# Select an area in the Framebuffer to copy the color settings.

def sent_data(signal):
    # send data to arduino to control robot arm
    # default_pos: 1001, grip_pos_open: 1010,
    # grip_pos_close: 101, release_pos_open: 100,
    # release_pos_close: 011, empty signal: 000.
    data = str(signal)+" "
    try:
        #print(data)
        bus.send(ustruct.pack("<h", len(data)), timeout=10000) # 首先发送长度 (16-bits).
        try:
            bus.send(data, timeout=10000) # 然后发送数据
            print("Sent Data!") # 没有遇到错误时，会显示
        except OSError as err:
            pass # 不用担心遇到错误，会跳过
    except OSError as err:
        pass # 不用担心遇到错误，会跳过

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.
# OpenMV上的硬件I2C总线都是2
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
bus.deinit() # 完全关闭设备
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
print("Waiting for Arduino...")
# For color tracking to work really well you should ideally be in a very, very,
# very, controlled enviroment where the lighting is constant...
green_threshold   = (15, 59, 40, 99, 14, 98)   # 颜色阈值，不同物体需要修改
K = 725
dist_threshold = 10               #小球距离(cm)
time_threshold = 5000  # mode switching checking time(ms)
time_record = None
mode = True             # mode True: ball tracking, False: ball capturing
circle_detection = False
cumulative_x = -1
cumulative_y = -1
cumulative_r = -1
update_ratio = 0.2
pid_scale = 1./255*100
x_pid = PID(p=0.5, i=0.5, d=0.5, imax=50)     # 方向参数p
h_pid = PID(p=10, i=15, d=5, imax=50)    # 速度参数p
sent_data(1)       # set robot arm to default_pos
cur_time = time.ticks()
while time.ticks()-cur_time <= 4000:
    sent_data(0)

while(True):
    #mode = False
    if mode:             # ball tracking mode
        clock.tick() # Track elapsed milliseconds between snapshots().
        img = sensor.snapshot().lens_corr(1.8)
        #img.draw_rectangle(QVGA_roi, color=(90,90,220))

        # Circle objects have four values: x, y, r (radius), and magnitude. The
        # magnitude is the strength of the detection of the circle. Higher is
        # better...

        # `threshold` controls how many circles are found. Increase its value
        # to decrease the number of circles detected...

        # `x_margin`, `y_margin`, and `r_margin` control the merging of similar
        # circles in the x, y, and r (radius) directions.

        # r_min, r_max, and r_step control what radiuses of circles are tested.
        # Shrinking the number of tested circle radiuses yields a big performance boost.

        blobs = img.find_blobs([green_threshold], #roi=QVGA_roi,
                    x_stride=2, y_stride=2,
                    area_threshold=5, pixels_threshold=5, merge=True)
        max_blob = None
        max_bsize = 0
        for blob in blobs:
            size = blob[2]*blob[3]
            if size > max_bsize:
                max_bsize = size
                max_blob = blob

        if max_blob:
            img.draw_cross(max_blob.cx(), max_blob.cy(), color=(128, 255, 128))
            img.draw_rectangle(max_blob.rect(), color=(128, 255, 128))
            new_roi = expanded_roi(max_blob.rect(), (15,15), img.width(), img.height())
            img.draw_rectangle(new_roi, color=(255, 128, 255))
            max_mag = 0
            ball = None # circle of ball (with max magnitude), default None
            x_error = None
            h_error = None
            for c in img.find_circles(
                    roi = new_roi,
                    #roi = blob.rect(),
                    threshold = 2500,
                    x_stride = 2, y_stride = 1,
                    x_margin = 10, y_margin = 10, r_margin = 10,
                    r_min = 2, r_max = 80, r_step = 2):
                # find the circle with maximum magnitude
                if c.magnitude() > max_mag:
                    max_mag = c.magnitude()
                    ball = c
            ###############################
            if circle_detection == False:
                ball = None # for debug purpose
            ###############################
            if ball:
                img.draw_circle(ball.x(), ball.y(), ball.r(), color = (255, 0, 0))
                x_error = ball.x() - img.width()/2
                dist = K/(ball.r()*2)
                h_error = dist - dist_threshold
                if cumulative_x < 0: # initialize cumulative circle
                    cumulative_x = round(ball.x())
                    cumulative_y = round(ball.y())
                    cumulative_r = round(ball.r())
                else: # update cumulative circle
                    cumulative_x = round((1-update_ratio) * cumulative_x + update_ratio * ball.x())
                    cumulative_y = round((1-update_ratio) * cumulative_y + update_ratio * ball.y())
                    cumulative_r = round((1-update_ratio) * cumulative_r + update_ratio * ball.r())
            else:
                x_error = max_blob.cx() - img.width()/2
                Lm = (max_blob[2] + max_blob[3])/2
                h_error = K/Lm - dist_threshold
                if cumulative_x < 0: # initialize cumulative circle
                    cumulative_x = round(max_blob.cx())
                    cumulative_y = round(max_blob.cy())
                    cumulative_r = round(Lm/2)
                else: # update cumulative circle
                    cumulative_x = round((1-update_ratio) * cumulative_x + update_ratio * max_blob.cx())
                    cumulative_y = round((1-update_ratio) * cumulative_y + update_ratio * max_blob.cy())
                    cumulative_r = round((1-update_ratio) * cumulative_r + update_ratio * Lm / 2)
            if cumulative_x >= 0:
                img.draw_circle(cumulative_x, cumulative_y, cumulative_r, color = (0, 0, 255))
                #x_error = cumulative_x - img.width()/2
                #h_error = K/(cumulative_r * 2) - dist_threshold
            x_output = x_pid.get_pid(x_error, 1)
            h_output = h_pid.get_pid(h_error, 1)
            print("x output: ", x_output, ", h output: ", h_output, ", h error: ", h_error)
            #run(h_output-x_output, h_output+x_output)
            #run(-x_output, x_output)
            print("sum x_error: ", cumulative_x-img.width()/2,
                "sum h_error", abs(K/(cumulative_r*2)-dist_threshold))
            if abs(cumulative_x-img.width()/2) < 8 and abs(K/(cumulative_r*2)-dist_threshold) < 2:
                if time_record is None:
                    time_record = time.ticks()
                else:
                    if time.ticks() - time_record > time_threshold:
                        mode = False  # switch to ball capturing mode
                        cumulative_x = -1 # reset
                        cumulative_y = -1
                        cumulative_r = -1
                        time_record = None
                    else:
                        car.run(h_output-x_output, h_output+x_output)
            else:
                time_record = None
                car.run(h_output-x_output, h_output+x_output)
        else:
            car.run(50,-50) # rotate to search ball
        print("FPS %f" % clock.fps())
    else:                # ball capturing mode
        car.run(0,0)     # Stop moving

        sent_data(2)   # grip_pos_open
        cur_time = time.ticks()
        while time.ticks()-cur_time <= 7000:
            sent_data(0)

        sent_data(3)   # grip_pos_close
        cur_time = time.ticks()
        while time.ticks()-cur_time <= 2500:
            sent_data(0)

        sent_data(1)   # default_pos
        cur_time = time.ticks()
        while time.ticks()-cur_time <= 7000:
            sent_data(0)

        sent_data(4)   # release_pos_close
        cur_time = time.ticks()
        while time.ticks()-cur_time <= 7000:
            sent_data(0)

        sent_data(5)   # release_pos_open
        cur_time = time.ticks()
        while time.ticks()-cur_time <= 2500:
            sent_data(0)

        cur_time = time.ticks()
        sent_data(1)   # default_pos
        while time.ticks()-cur_time <= 7000:
            sent_data(0)
        print("Capturing complete")
        mode = True      # reset mode to ball tracking

