# K230 病房数字识别

智能送药小车的 K230 CanMV 数字目标检测部署资源。当前目录保留最优的 8 类病房号模型，类别顺序为 `4, 5, 3, 6, 7, 8, 1, 2`。

## 目录内容

- `mp_deployment_source/`：部署配置 `deploy_config.json` 与最优 `.kmodel` 模型。
- `det_image_1_3.py`、`det_video_1_3.py`：基于 `DetectionApp` 的单图和实时视频推理脚本。
- `det_image_1_2_2.py`、`det_video_1_2_2.py`：另一套兼容推理实现。
- `libs/PipeLine.py`：相机采集和屏幕显示管线。
- `test.jpg`：单图推理测试图片；`det_results/` 为本地推理结果，已忽略。

模型输入为 320×320，置信度阈值为 0.4，NMS 阈值为 0.5；具体参数以 `mp_deployment_source/deploy_config.json` 为准。

## 部署

1. 将 `mp_deployment_source/` 上传到开发板 `/sdcard/mp_deployment_source/`。
2. 将所用脚本及 `libs/` 上传到开发板，并准备脚本依赖的 CanMV 库。
3. 使用 `det_image_1_3.py` 时，将 `test.jpg` 上传为 `/sdcard/test.jpg`；使用 `det_image_1_2_2.py` 时，将其上传为 `/sdcard/mp_deployment_source/test.jpg`。
4. 上传并运行所选的单图或实时视频推理脚本。`det_video_1_3.py` 中按下 K0 会将检测画面保存至 `/data/pic/`。

请始终配套使用本目录的配置文件和 `.kmodel`，不要混用其他训练版本的文件。
