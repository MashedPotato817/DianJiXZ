# -*- coding: utf-8 -*-
"""det_image_1_3.py 调试版——打印每步结果定位问题"""
import os, gc
from libs.PlatTasks import DetectionApp
from libs.Utils import *

print("=== Step 1: 加载图片 ===")
img_chw, img_rgb888 = read_image("/sdcard/test.jpg")
print("   CHW shape:", img_chw.shape)
print("   RGB888 size:", img_rgb888.width, "x", img_rgb888.height)
rgb888p_size = [img_chw.shape[2], img_chw.shape[1]]

print("=== Step 2: 加载配置 ===")
root_path = "/sdcard/mp_deployment_source/"
deploy_conf = read_json(root_path + "/deploy_config.json")
print("   config keys:", list(deploy_conf.keys()))
print("   kmodel:", deploy_conf["kmodel_path"])
print("   categories:", deploy_conf["categories"])
print("   conf_threshold:", deploy_conf["confidence_threshold"])
print("   nncase_version:", deploy_conf.get("nncase_version", "?"))

kmodel_path = root_path + deploy_conf["kmodel_path"]
print("   kmodel exists:", kmodel_path in os.listdir(root_path) if root_path in os.listdir("/sdcard/") else "?")
labels = deploy_conf["categories"]
confidence_threshold = deploy_conf["confidence_threshold"]
nms_threshold = deploy_conf["nms_threshold"]
model_input_size = deploy_conf["img_size"]
nms_option = deploy_conf["nms_option"]
model_type = deploy_conf["model_type"]
anchors = []
if model_type == "AnchorBaseDet":
    anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

print("=== Step 3: 初始化 DetectionApp ===")
try:
    det_app = DetectionApp("image", kmodel_path, labels, model_input_size,
                            anchors, model_type, confidence_threshold,
                            nms_threshold, rgb888p_size, rgb888p_size,
                            debug_mode=0)
    print("   DetectionApp created OK")
except Exception as e:
    print("   FAILED:", e)
    raise

print("=== Step 4: config_preprocess ===")
try:
    det_app.config_preprocess()
    print("   preprocess OK")
except Exception as e:
    print("   FAILED:", e)
    raise

print("=== Step 5: 推理 ===")
try:
    res = det_app.run(img_chw)
    print("   result type:", type(res))
    if isinstance(res, dict):
        print("   keys:", list(res.keys()))
        for k in res:
            v = res[k]
            if hasattr(v, '__len__') and not isinstance(v, str):
                print("   %s: type=%s len=%d" % (k, type(v).__name__, len(v)))
            else:
                print("   %s: %s" % (k, v))
        # try new API: dict with boxes/scores/classes
        if 'scores' in res and len(res['scores']) > 0:
            n = min(5, len(res['scores']))
            for i in range(n):
                print("   [%d] score=%.3f" % (i, res['scores'][i]), end='')
                if 'classes' in res: print(" class=%d" % res['classes'][i], end='')
                if 'boxes' in res: print(" box=%s" % res['boxes'][i], end='')
                print()
    elif res:
        print("   *** 无检测结果——模型可能不兼容或阈值太高 ***")
        print("   *** 尝试降阈值到 0.1 再试...")
except Exception as e:
    print("   FAILED:", e)
    import sys
    sys.print_exception(e)

print("=== Step 6: 画结果 ===")
det_app.draw_result(img_rgb888, res)
img_rgb888.compress_for_ide()
print("=== 完成 ===")
det_app.deinit()
gc.collect()
