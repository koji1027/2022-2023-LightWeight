import sensor, image, time, math, ustruct
from machine import UART
from fpioa_manager import fm
#from pyb import UART
center = [160, 120] #円錐の中心(x,y)
#xの減少方向が後ろ、増加方向が正面
#yの減少方向が右、増加方向が左(上から俯瞰して)
#thresholds = [(60, 85, -10, 5, 10, 60)]
thresholds = [(40, 60, -6, 10, 10, 45)]
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False, gain_db = 9)
sensor.set_auto_exposure(False, exposure_us = 240380)
sensor.set_auto_whitebal(False, rgb_gain_db = [-5, -5, -1.5])
sensor.set_gainceiling(128)
sensor.set_contrast(3)
sensor.set_brightness(1)
sensor.set_saturation(3)
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time = 2000)
clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()
    goal = [0,0]
    max_s = 0;
    a = -1
    for blob in img.find_blobs(thresholds, pixels_threshold=200, area_threshold=200):
        if max_s < blob.area():
            a = blob
            #img.draw_rectangle(blob.rect())
            #img.draw_cross(blob.cx(), blob.cy())
            #goal[0] = blob.cx()
            #goal[1] = blob.cy()
            max_s = blob.area()
    if a != -1:
        img.draw_rectangle(a.rect())
        img.draw_cross(a.cx(), a.cy())
        goal[0] = a.cx()
        goal[1] = a.cy()
    abs_goal = [goal[0] - center[0], goal[1] - center[1]]
    goal_theta = -math.atan2(abs_goal[0], abs_goal[1])
    goal_theta += math.pi * 3 / 4
    if goal_theta > math.pi:
        goal_theta -= math.pi * 2
    goal_theta += math.pi
    if goal_theta > math.pi:
        goal_theta -= math.pi * 2
    print("角度: ", end="")
    print(goal_theta/math.pi*180, end="")
    print("\t面積:", end="")
    print(max_s)
    print("°\tFPS: ", end="")
    print(clock.fps())
    x = int((goal_theta + math.pi) * 100)
    y = int(x) & 0b0000000001111111
    z = int(x) >> 7
    w = 0
    if max_s > 3000:
        w = 1
    try:
        uart.write(ustruct.pack('B',255))
        uart.write(ustruct.pack('B',int(y)))
        uart.write(ustruct.pack('B',int(z)))
    except OSError as err:
        pass
