import sensor, image, time, math, pyb, ustruct
from pyb import UART
center = [201, 94] #円錐の中心(x,y)
#xの減少方向が後ろ、増加方向が正面
#yの減少方向が右、増加方向が左(上から俯瞰して)
#goal_thresholds = [(69, 88, -14, 1, 27, 79)] #黄色ゴール
.goal_thresholds = [(29, 49, -28, 0, -30, -5)] #青色ゴール
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
    dist = 0
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
    #print("角度: ", end="")
    #print(goal_theta/math.pi*180, end="")
    print("\t面積", end="")
    print(max_s)
    #print("°\tFPS: ", end="")
    #print(clock.fps())
    #print(goal[0], end="")
    #print("\t", end="")
    #print(goal[1])
    dist = math.sqrt(abs_goal[0] ** 2 + abs_goal[1] ** 2)
    x = int((goal_theta + math.pi) * 100)
    y = int(x) & 0b0000000001111111
    z = int(x) >> 7
    w = 0
    if max_s > 700:
        w = 1
    try:
        uart.write(ustruct.pack('B',255))
        uart.write(ustruct.pack('B',int(y)))
        uart.write(ustruct.pack('B',int(z)))
        uart.write(ustruct.pack('B', int(w)))

    except OSError as err:
        pass
