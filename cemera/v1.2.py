# 垂直向上0°，顺时针旋转，水平向右90°
# 线段首尾x1、x2会颠倒
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
import time, os, gc, math

# 分辨率 240x160
DETECT_WIDTH = 240
DETECT_HEIGHT = 160

sensor = None
uart = None

# 状态机定义
STATE_TRACKING = 1     # 巡线
STATE_INTERSECT = 2    # 进入路口
STATE_IDLE = 3         # 空闲
STATE_PARKING = 4      # 车库停车

# 当前全局状态，默认状态 1
current_state = STATE_TRACKING

def uart_init():
    global uart
    # 初始化串口3，波特率 115200
    uart = UART(3, baudrate=115200)

def camera_init():
    global sensor
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=60, to_ide=True)
    sensor.run()

def camera_deinit():
    global sensor
    sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)

# ================= 辅助函数：寻找最长的线 =================
def find_longest_line(lines_list):
    # 替代 lambda 写法，防止旧版固件报语法错误
    if len(lines_list) == 0:
        return None
    longest = lines_list[0]
    max_len = longest.length()
    for l in lines_list:
        if l.length() > max_len:
            longest = l
            max_len = l.length()
    return longest

# ================= 串口数据发送 =================

def send_uart_data(state, main_line, distance1, is_at_intersection):
    ma = 0
    mcx = 0
    mcy = 0
    mx1 = 0
    my1 = 0
    mx2 = 0
    my2 = 0

    if main_line != None:
        ma = main_line.theta()
        mx1 = main_line.x1()
        my1 = main_line.y1()
        mx2 = main_line.x2()
        my2 = main_line.y2()
        mcx = int((mx1 + mx2) / 2)
        mcy = int((my1 + my2) / 2)

    # 使用最基础的 % 字符串格式化，彻底杜绝 f-string 语法错误
    packet = "SA,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,ED\n" % (
        state, ma, mcx, mcy, mx1, my1, mx2, my2, distance1, is_at_intersection
    )

    if uart:
        uart.write(packet.encode('utf-8'))
    print(packet.strip())

# ================= 核心图像处理与状态机 =================

def process_state_machine():
    global current_state, sensor, uart
    fps = time.clock()

    while True:
        fps.tick()
        try:
            os.exitpoint()

            # --- 1. 检测串口输入 ---
            if uart and uart.any():
                data = uart.read().decode('utf-8').strip()
                if '1' in data:
                    current_state = STATE_TRACKING
                elif '2' in data:
                    current_state = STATE_INTERSECT
                elif '3' in data:
                    current_state = STATE_IDLE
                elif '4' in data:
                    current_state = STATE_PARKING

            img = sensor.snapshot()

            # --- 2. 提取有效线段 (基础for循环，防报错) ---
            raw_lines = img.find_line_segments(merge_distance=40, max_theta_diff=10)
            valid_lines = []
            if raw_lines:
                for l in raw_lines:
                    if l.length() >= 40:
                        valid_lines.append(l)

            # --- 3. 线段分类与提取 ---
            horiz_lines = []
            left_lines = []
            right_lines = []

            for l in valid_lines:
                theta = l.theta()
                # 横向线 (60° 到 120°)
                if theta >= 60 and theta <= 120:
                    horiz_lines.append(l)
                # 竖向线 (0-30° 或 150-180°)
                elif theta <= 30 or theta >= 150:
                    center_x = (l.x1() + l.x2()) / 2
                    if center_x < 120:  # 屏幕宽度一半为界
                        left_lines.append(l)
                    else:
                        right_lines.append(l)

            # 找出最长的干线
            main_line = find_longest_line(horiz_lines)

            # --- 4. 【核心逻辑】计算距离1 与 路口判断 ---
            distance1 = 0
            is_at_intersection = 0

            left_line = find_longest_line(left_lines)
            right_line = find_longest_line(right_lines)

            if left_line != None and right_line != None:
                # 计算中心横坐标
                lx = int((left_line.x1() + left_line.x2()) / 2)
                rx = int((right_line.x1() + right_line.x2()) / 2)

                # 【距离1】两条竖直线的水平像素差
                distance1 = abs(rx - lx)

                # 寻找最高点 (Y越小代表在图像中越靠上)
                ly1 = left_line.y1()
                ly2 = left_line.y2()
                left_top_y = ly1 if ly1 < ly2 else ly2

                ry1 = right_line.y1()
                ry2 = right_line.y2()
                right_top_y = ry1 if ry1 < ry2 else ry2

                # IDE 调试可视化展示
                img.draw_line(left_line.x1(), left_line.y1(), left_line.x2(), left_line.y2(), color=150, thickness=2)
                img.draw_line(right_line.x1(), right_line.y1(), right_line.x2(), right_line.y2(), color=150, thickness=2)
                img.draw_line(lx, left_top_y, rx, right_top_y, color=255, thickness=2)
                img.draw_circle(lx, left_top_y, 4, color=255, thickness=2)
                img.draw_circle(rx, right_top_y, 4, color=255, thickness=2)

                # 【是否到达路口】高度差小于等于 15 个像素
                if abs(left_top_y - right_top_y) <= 15:
                    is_at_intersection = 1

            # ================= 状态机分支与发送 =================

            if current_state == STATE_TRACKING:
                if main_line != None:
                    img.draw_line(main_line.x1(), main_line.y1(), main_line.x2(), main_line.y2(), color=255, thickness=3)

                if is_at_intersection == 1:
                    current_state = STATE_INTERSECT

                send_uart_data(current_state, main_line, distance1, is_at_intersection)

            elif current_state == STATE_INTERSECT:
                img.draw_string(10, 10, "STATE: INTERSECT", color=255, scale=2)
                send_uart_data(current_state, None, distance1, is_at_intersection)

            elif current_state == STATE_IDLE:
                img.draw_string(10, 10, "STATE: IDLE", color=255, scale=2)
                send_uart_data(current_state, None, distance1, is_at_intersection)

            elif current_state == STATE_PARKING:
                if main_line != None:
                    img.draw_line(main_line.x1(), main_line.y1(), main_line.x2(), main_line.y2(), color=255, thickness=3)
                send_uart_data(current_state, main_line, distance1, is_at_intersection)

            Display.show_image(img)
            img = None
            gc.collect()

        except KeyboardInterrupt as e:
            print("user stop")
            break
        except BaseException as e:
            import sys
            sys.print_exception(e)
            break

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    try:
        print("uart init")
        uart_init()
        print("camera init")
        camera_init()
        camera_is_init = True
        print("camera capture")
        process_state_machine()
    except Exception as e:
        import sys
        sys.print_exception(e)
    finally:
        if camera_is_init:
            print("camera deinit")
            camera_deinit()

if __name__ == "__main__":
    main()
