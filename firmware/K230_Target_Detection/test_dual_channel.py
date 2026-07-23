"""检查 v1.8 固件双通道 API 支持情况。"""
from media.sensor import (CAM_CHN_ID_0, CAM_CHN_ID_1,
                          PIXEL_FORMAT_YUV_SEMIPLANAR_420, Sensor)
from media.display import Display
from media.media import MediaManager

sensor = None
try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    print("1. sensor reset OK")

    sensor.set_framesize(w=640, h=480, chn=CAM_CHN_ID_0)
    print("2. ch0 framesize OK")

    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420, chn=CAM_CHN_ID_0)
    print("3. ch0 pixformat OK")

    sensor.set_framesize(w=320, h=240, chn=CAM_CHN_ID_1)
    print("4. ch1 framesize OK")

    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)
    print("5. ch1 pixformat OK")

    Display.init(Display.ST7701, width=640, height=480, to_ide=True)
    print("6. display init OK")

    info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    print("7. bind_info OK:", info)

    import image
    osd = image.Image(640, 480, image.ARGB8888)
    print("8. osd image OK")

    # 尝试1：只传 info，不加 layer
    try:
        Display.bind_layer(**info)
        print("9a. bind_layer(**info) OK")
    except Exception as e1:
        print("9a. bind_layer(**info) FAILED:", e1)
        # 尝试2：不传第四个参数
        try:
            Display.bind_layer(info['pix_format'], info['src'], info['rect'])
            print("9b. bind_layer(3 args) OK")
        except Exception as e2:
            print("9b. bind_layer(3 args) FAILED:", e2)
            # 尝试3：直接传入 info
            try:
                Display.bind_layer(info)
                print("9c. bind_layer(info) OK")
            except Exception as e3:
                print("9c. bind_layer(info) FAILED:", e3)

    MediaManager.init()
    sensor.run()
    print("10. sensor run OK")
    print("ALL OK - dual channel works!")

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
