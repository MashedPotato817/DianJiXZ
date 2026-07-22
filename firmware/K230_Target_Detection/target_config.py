"""同心圆靶识别参数：须在实际 K230 相机画面上标定。"""

# K230D 当前已验证的默认相机编号；其他板型可改为 None 使用自动检测。
SENSOR_ID = 0

# 显示设备。使用 IDE 预览时仍会启用 to_ide=True。
DISPLAY_MODE = "lcd"

# Hough 圆半径范围（像素）。根据靶距调整，避免把纸张边缘等大圆纳入检测。
MIN_RADIUS = 20
MAX_RADIUS = 500
CIRCLE_THRESHOLD = 3_500
X_STRIDE = 2
Y_STRIDE = 2

# 同一靶子的圆心允许的最大间距（像素）。
CENTER_TOLERANCE_PX = 18

# 红点 LAB 阈值。此为初始值，必须按实际光照用 CanMV IDE 标定。
# 红点优先；当红点过远、反光或被遮挡而未检出时，程序自动回退到同心圆圆心。
RED_DOT_THRESHOLD = (20, 100, 15, 127, 0, 127)
RED_DOT_MIN_PIXELS = 8
RED_DOT_MAX_PIXELS = 2_000
RED_DOT_MIN_ASPECT = 0.55
RED_DOT_MAX_ASPECT = 1.80

# 每隔多少帧输出一条识别结果，避免日志刷屏。
LOG_PERIOD_FRAMES = 15
