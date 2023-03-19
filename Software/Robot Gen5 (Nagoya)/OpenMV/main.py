import sensor, image, time, math, pyb, ustruct
from pyb import UART
center = [197, 78] #円錐の中心(x,y)
#xの減少方向が後ろ、増加方向が正面
#yの減少方向が右、増加方向が左(上から俯瞰して)
#thresholds = [(60, 85, -10, 5, 10, 60)]
goal_thresholds = [(50, 85, -10, 10, 35, 60)]
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
    for blob in img.find_blobs(goal_thresholds, pixels_threshold=200, area_threshold=200):
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        goal[0] = blob.cx() + (blob.w() / 2)
        goal[1] = blob.cy() + (blob.h() / 2)
    abs_goal = [goal[0] - center[0], goal[1] - center[1]]
    goal_theta = math.atan2(abs_goal[0], abs_goal[1])
    goal_theta += math.pi / 2
    goal_theta += math.pi
    if goal_theta > math.pi:
        goal_theta -= math.pi * 2
    print("角度: ", end="")
    print(goal_theta/math.pi*180, end="")
    print("°\tFPS: ", end="")
    print(clock.fps())
    x = int((goal_theta + math.pi) * 100)
    y = int(x) & 0b0000000001111111
    z = int(x) >> 7
    try:
        uart.write(ustruct.pack('B',255))
        uart.write(ustruct.pack('B',int(y)))
        uart.write(ustruct.pack('B',int(z)))
    except OSError as err:
        pass
