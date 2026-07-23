"""检查 v1.8 固件 bind_layer + dstlayer 是否可用。"""
from media.sensor import (CAM_CHN_ID_0, CAM_CHN_ID_1,
                          PIXEL_FORMAT_YUV_SEMIPLANAR_420, Sensor)
from media.display import Display
from media.media import MediaManager

sensor = None
try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    print("1. sensor reset OK")

    # ch0: YUV420 用于硬件直显
    sensor.set_framesize(w=640, h=480, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420, chn=CAM_CHN_ID_0)
    print("2. ch0 YUV420 OK")

    # ch1: GRAYSCALE 用于检测
    sensor.set_framesize(w=320, h=240, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)
    print("3. ch1 GRAYSCALE OK")

    Display.init(Display.ST7701, osd_num=1, to_ide=True)
    print("4. display init OK (osd_num=1)")

    info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    print("5. bind_info:", info)

    # 按 main.py 的正确调用方式
    Display.bind_layer(**info, dstlayer=Display.LAYER_VIDEO1)
    print("6. bind_layer(**info, dstlayer=VIDEO1) OK ✓")

    import image
    osd = image.Image(640, 480, image.ARGB8888)
    osd.draw_cross(320, 240, color=(255, 0, 0), size=40, thickness=3)
    Display.show_image(osd, 0, 0, Display.LAYER_OSD3)
    print("7. OSD show_image OK ✓")

    MediaManager.init()
    sensor.run()
    print("ALL OK — v1.8 supports bind_layer + dstlayer!")
    print("Check LCD: should see camera preview with red cross in center")

except Exception as e:
    print("FAILED:", e)

finally:
    try:
        if sensor is not None:
            sensor.stop()
    except:
        pass
    try:
        Display.deinit()
    except:
        pass
    try:
        MediaManager.deinit()
    except:
        pass
