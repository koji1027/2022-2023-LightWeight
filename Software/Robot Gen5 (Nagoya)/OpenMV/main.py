import sensor, image, time, math, pyb, ustruct
from pyb import UART
center = [201, 94] #円錐の中心(x,y)
#xの減少方向が後ろ、増加方向が正面
#yの減少方向が右、増加方向が左(上から俯瞰して)
#thresholds = [(60, 85, -10, 5, 10, 60)]
goal_thresholds = [(45, 85, -25, 10, 25, 61)]
ball_thresholds = [(30, 100, 0, 40, -20, 10)]
uart = UART(3, 115200)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time = 2000)
clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()
    goal = [0,0]
    max_s = 0
    a = -1
    for blob in img.find_blobs(goal_thresholds, pixels_threshold=200, area_threshold=200):
        if max_s < blob.area():
            max_s = blob.area()
            a = blob
    if a != -1:
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        #goal[0] = blob.cx() + (blob.w() / 2)
        #goal[1] = blob.cy() + (blob.h() / 2)
        goal[0] = blob.cx()
        goal[1] = blob.cy()
    abs_goal = [goal[0] - center[0], goal[1] - center[1]]
    goal_theta = math.atan2(abs_goal[0], abs_goal[1])
    goal_theta += math.pi / 2
    goal_theta += math.pi
    if goal_theta > math.pi:
        goal_theta -= math.pi * 2
    print("角度: ", end="")
    print(goal_theta/math.pi*180, end="")
    print("\t面積", end="")
    print(max_s, end="")
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
        uart.write(ustruct.pack('B', int(w)))
    except OSError as err:
        pass
