"""K230 CanMV 黑框定位第 1 阶段：黑色连通域锁定靶面 ROI。"""
import os
import time

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
BLACK_THRESHOLD = (0, 96)
MIN_BLOB_PIXELS = 1_000
MIN_FRAME_BOX_AREA = 6_000
MIN_FRAME_ASPECT = 0.45
MAX_FRAME_ASPECT = 0.95
LOG_PERIOD_FRAMES = 15


def choose_black_frame(img):
    """从黑色连通域中选取与靶面外框尺寸相符、黑色像素最多的候选。"""
    gray_img = img.to_grayscale(copy=True)
    blobs = gray_img.find_blobs([BLACK_THRESHOLD], pixels_threshold=MIN_BLOB_PIXELS,
                                area_threshold=MIN_BLOB_PIXELS)
    candidates = []
    for blob in blobs:
        width = blob.w()
        height = blob.h()
        area = width * height
        if area < MIN_FRAME_BOX_AREA:
            continue
        aspect = min(width, height) / max(width, height) if width and height else 0
        if not MIN_FRAME_ASPECT <= aspect <= MAX_FRAME_ASPECT:
            continue
        candidates.append(blob)
    return max(candidates, key=lambda blob: blob.pixels()) if candidates else None, len(blobs)


sensor = None
try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(w=FRAME_WIDTH, h=FRAME_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    Display.init(Display.ST7701, width=FRAME_WIDTH, height=FRAME_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()
    print("black-frame blob demo started: %dx%d" % (FRAME_WIDTH, FRAME_HEIGHT))

    frame_count = 0
    while True:
        os.exitpoint()
        img = sensor.snapshot()
        frame_blob, blob_count = choose_black_frame(img)
        if frame_blob:
            x, y, width, height = frame_blob.rect()
            img.draw_rectangle(x, y, width, height, color=(255, 255, 0), thickness=2)
            img.draw_cross(frame_blob.cx(), frame_blob.cy(), color=(255, 0, 0),
                           size=20, thickness=2)
            if frame_count % LOG_PERIOD_FRAMES == 0:
                print("blobs=%d, selected=yes, roi=(%d, %d, %d, %d), center=(%d, %d)" %
                      (blob_count, x, y, width, height, frame_blob.cx(), frame_blob.cy()))
        elif frame_count % LOG_PERIOD_FRAMES == 0:
            print("blobs=%d, selected=no" % blob_count)
        frame_count += 1
        Display.show_image(img, 0, 0)

except KeyboardInterrupt as error:
    print("user stop:", error)
except BaseException as error:
    print("Exception:", error)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
