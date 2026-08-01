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
# 由白管预览画面将小球轨迹的纵向中心约束在 y=80–149 px
# 在管子上下保留余量。
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
BALL_ROI = (0, 80, FRAME_WIDTH, 70)
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
# 仅供“球从相机视野边缘离开”恢复策略使用。连续丢失前，最后可靠球心
# 落在图像左右各 24 px 内时，才报告 edge=-1 / +1；中途漏检保持 edge=0。
EDGE_LOST_PIXEL_MARGIN = 24

# CALIBRATION: 图像横坐标到摆杆横向位置的线性映射。
# 本轮三点静态实测（-50/0/+50 mm）有效帧中位数线性拟合：
# x_mm = (px - 160.365) * 0.540652，图像向右为正。
# 该值仅用于预览复测；补齐五点、每点三次重复测量并评估总误差前，
# 不得将其用于闭环控制。
IMAGE_CENTER_X = 160.365
MM_PER_PIXEL = 0.540652
MAX_POSITION_MM = 150.0

# 相机和摆杆完成固定并通过本 README 的标定步骤前必须保持 False。
# 它表示位置映射尚不能作为正式控制/精度结果使用。
CALIBRATION_READY = False

# 临时联调开关：仅在已确认舵机中位、方向和小幅限位后，才可置 True。
# True 时 UART 发送当前临时映射的 x_mm 与 valid，供低幅度闭环验证；
# 不改变 CALIBRATION_READY，也绝不能作为标定完成或性能达标的依据。
UART_ALLOW_UNCALIBRATED = True

# UART1: IO40=TX, IO41=RX，接 MSPM0 UART1（PB7=RX, PB6=TX）。
UART_BAUDRATE = 115200
SEND_PERIOD_MS = 50
LOG_PERIOD_FRAMES = 30

# 终端默认只保留启动、一次 ACK 和低频摘要；避免 MSPM0 心跳/RX 原始字节刷屏。
# 排查串口异常时才临时打开，测试完成后恢复 False。
ENABLE_UART_RX_DEBUG = False
# 端口、模式等启动配置仅在排查部署配置时显示，日常运行保持 False。
ENABLE_STARTUP_CONFIG_LOG = False
# 日常终端只输出状态变化；TXT 仍保留低频样本供调参。
# 打开后才将同一摘要同步显示到终端。
ENABLE_CONSOLE_SUMMARY = False
ENABLE_LOG_SUMMARY = True
LOG_SUMMARY_PERIOD_MS = 1000
# 检测循环会持续创建图像和候选对象。保留垃圾回收以避免长期运行耗尽内存，
# 但不在每帧执行，避免其占用圆检测与显示的关键路径。
GC_PERIOD_MS = 1000

# K230 本地检测日志。仅记录应用输出，不捕获 CanMV 固件启动时的系统日志。
ENABLE_LOG_FILE = True
LOG_FOLDER_PATH = "/data/ball_detect_preview/"
# 日志会话编号前缀。完整编号 = 前缀 + "_" + 六位序号，序号由脚本启动时
# 自动递增（计数器存于 LOG_FOLDER_PATH 下的 .session_counter），无需手工维护，
# 每次上电不会重复使用旧编号。启动时终端打印 "LOG SESSION: <前缀>_<序号>"，
# 电脑串口助手用同一编号保存 M0 日志（<同一编号>_M0.txt）。
LOG_SESSION_PREFIX = "20260731"
