"""测试当前固件是否支持 cv_lite 模块。"""
try:
    import cv_lite
    print("cv_lite: OK, dir=", dir(cv_lite))
except ImportError:
    print("cv_lite: NOT AVAILABLE - need daily_build firmware")
