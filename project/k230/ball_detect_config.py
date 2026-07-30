# K230 小球检测与 UART 参数。
# 所有带 CALIBRATION 标记的值均为待实机标定初值，不能当作实测结果。

# "cvlite_circle"：使用 CanMV cv_lite 的灰度霍夫圆检测（默认）；
# "blob"：使用 BALL_LAB_THRESHOLD 检测单色球，速度更快且应优先用于实机。
DETECT_MODE = "cvlite_circle"

# 仅 DETECT_MODE == "blob" 时使用。请用 CanMV IDE 阈值工具标定后填写。
BALL_LAB_THRESHOLD = (20, 100, 15, 127, 0, 127)
BALL_MIN_PIXELS = 40
BALL_MAX_PIXELS = 5000
BALL_MIN_ASPECT = 0.65
BALL_MAX_ASPECT = 1.50

# 圆检测参数。横向保留全范围，避免限制小球全行程；
# 由白管预览画面将小球轨迹的纵向中心约束在 y=85–139 px
# 在管子上下保留余量。
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
BALL_ROI = (0, 85, FRAME_WIDTH, 55)
# cv_lite 无 ROI 参数。True 时先复制有效运动带，再在其上执行霍夫圆检测；
# 需在 K230 上与 False 的全图模式比较 FPS，异常时可立即回退。
CIRCLE_USE_ROI_CROP = True
# cv_lite.grayscale_find_circles() 参数。
CIRCLE_DP = 1
CIRCLE_MIN_DISTANCE = 20
CIRCLE_CANNY_HIGH = 80
CIRCLE_ACCUMULATOR = 20
CIRCLE_R_MIN = 6
CIRCLE_R_MAX = 45

# 时序筛选：至少连续检测到 STABLE_FRAMES 帧才置 valid=1；
# 连续 LOST_FRAMES 帧未检测到才置 valid=0；短暂漏检保持最近一次有效位置。
STABLE_FRAMES = 3
LOST_FRAMES = 3
MAX_CENTER_JUMP_PX = 28

# CALIBRATION: 图像横坐标到摆杆横向位置的线性映射。
# 本轮三点静态实测（-50/0/+50 mm）有效帧中位数线性拟合：
# x_mm = (px - 160.365) * 0.540652，图像向右为正。
# 该值仅用于预览复测；补齐五点、每点三次重复测量并评估总误差前，
# 不得将其用于闭环控制。
IMAGE_CENTER_X = 160.365
MM_PER_PIXEL = 0.540652
MAX_POSITION_MM = 150.0

# 相机和摆杆完成固定并通过本 README 的标定步骤前必须保持 False。
# False 时 UART 脚本仍发送标准 BALL 帧用于链路验证，但固定发送
# x_mm=0.0、valid=0，禁止未标定视觉值进入主控闭环。
CALIBRATION_READY = False

# UART1: IO40=TX, IO41=RX，已与 MSPM0 UART2 通信验证。
UART_BAUDRATE = 115200
SEND_PERIOD_MS = 50
LOG_PERIOD_FRAMES = 30

# K230 本地检测日志。仅记录应用输出，不捕获 CanMV 固件启动时的系统日志。
ENABLE_LOG_FILE = True
LOG_FOLDER_PATH = "/data/ball_detect_preview/"
