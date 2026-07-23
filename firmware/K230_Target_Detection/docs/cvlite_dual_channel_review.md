# cvlite_demo 双通道显示架构评审

> 评审人：Codex  
> 日期：2026-07-23 12:47
> 范围：`cvlite_demo.py`、`test_dual_channel.py`、`cvlite_dual_channel.md`  
> 结论：**暂不建议按当前提交说明合入。** 双通道分辨率和 `×2` 坐标映射方向正确，但主程序尚未落实文档所述的“ch0 硬件直显 + OSD 独立层”架构。

## 1. 目标架构与当前实际架构

### 提交说明和设计文档中的目标架构

```text
1280×960 相机/ISP 输入
├─ ch0：640×480 YUV420 → Display.bind_layer(VIDEO1) → LCD 全屏彩色预览
├─ ch1：320×240 GRAYSCALE → cv_lite 靶纸检测
└─ OSD：640×480 ARGB → LAYER_OSD3，按需刷新
```

这个架构是合理的：两路均为 4:3，完整保留视野，检测坐标映射到显示坐标只需 `×2`。

来源：选自 [cvlite_dual_channel.md（第 14～55 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/docs/cvlite_dual_channel.md:14)。

### 当前 `cvlite_demo.py` 的实际架构

```text
ch0：640×480 RGB565 → 每轮 snapshot() → Python 绘制结果 → show_image()
ch1：320×240 GRAYSCALE → 每轮 snapshot() → cv_lite 检测
```

实际主循环仍每帧获取 ch0 图像，并在同一帧上绘制、调用 `Display.show_image()`。它可以达到全屏显示，但这不是 `bind_layer()` 硬件直显，ch0 帧仍进入 Python，显示路径仍存在逐帧图像处理与提交开销。

来源：选自 [cvlite_demo.py（第 79～86、100～104、151～163 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/cvlite_demo.py:79)。

## 2. 评审结论

### 2.1 阻塞问题：主程序没有使用硬件直显

当前 ch0 配置为 `Sensor.RGB565`，并在循环内使用：

```python
display_img = sensor.snapshot(chn=CAM_CHN_ID_0)
Display.show_image(display_img, 0, 0)
```

因此下列提交描述目前不成立：

- “ch0 硬件直显 LCD”；
- “拒绝 `show_image` 每帧贴图”；
- “OSD 独立层按需刷新”。

应使用 YUV420 ch0，调用 `Display.bind_layer()` 绑定 `Display.LAYER_VIDEO1`；主循环不再 `snapshot(chn=CAM_CHN_ID_0)`，也不再对视频帧调用 `show_image()`。

### 2.2 阻塞问题：诊断脚本错误判断 `dstlayer` 兼容性

`test_dual_channel.py` 当前测试：

```python
Display.bind_layer(**info)
```

这里没有传入 `dstlayer`，其失败不能证明 CanMV v1.8 不兼容 `dstlayer`。工程的 `main.py` 已有正确调用：

```python
Display.bind_layer(
    **sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0),
    dstlayer=Display.LAYER_VIDEO1
)
```

来源：选自 [test_dual_channel.py（第 28～52 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/test_dual_channel.py:28)；[main.py（第 79～80 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/main.py:79)。

此外，诊断脚本在所有 `bind_layer` 尝试失败后仍会继续执行，并打印 `ALL OK - dual channel works!`，会产生假阳性。来源：选自 [test_dual_channel.py（第 35～57 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/test_dual_channel.py:35)。

### 2.3 高优先级问题：OSD 按需刷新尚未实现

设计文档要求创建 `640×480 ARGB` OSD，只有识别结果变化时才清除、重绘并提交到 `LAYER_OSD3`。实际代码是在 RGB565 视频帧上每轮绘制圆、角点和十字，并每轮 `show_image()`。

这会使“显示视频刷新率”“算法帧率”“叠加层刷新率”绑定在一个循环中，无法实现视频稳定直显、OSD 降频刷新。

来源：选自 [cvlite_dual_channel.md（第 41～54 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/docs/cvlite_dual_channel.md:41)；[cvlite_demo.py（第 151～163 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/cvlite_demo.py:151)。

### 2.4 中优先级问题：`reject` 日志 ROI 与图形可能不一致

跳变拒绝时，代码会把 `corners` 与 `rect` 回退为上一份有效候选，但 `rx, ry, rw, rh` 已在回退前从新候选赋值。后续日志可能出现“旧中心/旧四角 + 新 ROI”的混合结果，不利于调试误检。

来源：选自 [cvlite_demo.py（第 130～142、165～169 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/cvlite_demo.py:130)。

### 2.5 功能风险：窗格误检尚未根治

新版本将矩形筛选从单纯最大面积改为面积与长宽比评分，这是改进；但仍是全画面矩形候选。背景窗格也可能满足 `0.60~0.80` 的长宽比和面积范围。黑框 ROI、ROI 内四边线、圆环/红点校验尚未接入，因此此前实拍中“窗格被误判为靶纸”的根因仍存在。

来源：选自 [cvlite_demo.py（第 44～70 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/cvlite_demo.py:44)；[cvlite_demo_优化建议报告.md（两阶段几何方案）](../research/cvlite_demo_优化建议报告.md)。

## 3. 推荐修正方案

### 阶段 1：修正诊断脚本

1. ch0 保持 `640×480`，像素格式改为 `PIXEL_FORMAT_YUV_SEMIPLANAR_420`。
2. 调用 `Display.bind_layer(**info, dstlayer=Display.LAYER_VIDEO1)`。
3. 绑定失败立即抛出异常并退出；不得继续打印成功。
4. 运行后实际观察 LCD/IDE 是否出现全屏彩色动态画面，再判定测试通过。
5. 记录 `Display.width()`、`Display.height()`、ch0/ch1 实际图像尺寸和诊断日志。

### 阶段 2：主程序切换至硬件视频层 + OSD 层

初始化：

```text
ch0: 640×480 YUV420 → bind_layer(..., VIDEO1)
ch1: 320×240 GRAYSCALE → sensor.snapshot(chn=CAM_CHN_ID_1) → cv_lite
osd: 640×480 ARGB8888 → show_image(..., LAYER_OSD3)
```

主循环要求：

```text
不得 snapshot(ch0)
不得对视频帧 show_image()
只 snapshot(ch1) 做检测
仅当结果变化或达到 OSD 刷新周期时，clear/re-draw/show OSD
```

坐标映射：

```text
x_osd = 2 × x_detect
y_osd = 2 × y_detect
r_osd = 2 × r_detect
```

### 阶段 3：性能与交互验收

| 指标 | 验收方式 |
| --- | --- |
| 视频显示 | LCD 全屏 640×480、无软件黑边、连续彩色画面 |
| 检测 | 保持 320×240 灰度，不降低完整 4:3 视野 |
| 坐标叠加 | 十字/四角与真实画面目标重合；误差不超过 2 个显示像素 |
| 视频与 OSD | 视频持续刷新；OSD 仅在结果变化或 10～15 FPS 更新 |
| 性能 | 记录检测 FPS、OSD FPS、内存异常和持续运行稳定性 |
| 诊断 | `bind_layer` 成功、ch0/ch1 均有有效画面、失败时明确退出 |

### 阶段 4：靶纸身份校验

显示架构修正后，继续实现“黑框 ROI → ROI 内四边线 → 对角线交点 → 圆环/红点校验”。在此之前，不应将当前 `detect` 结果接入小车闭环控制。

## 4. 可保留的改动

- ch0 `640×480` 与 ch1 `320×240` 的 4:3 双通道分辨率选择；
- `SCALE = 2` 的坐标、半径映射；
- 连续两帧确认后才使用候选的思路；
- 面积与目标长宽比评分替代“直接取最大面积”的尝试；
- 双通道架构与 OSD 独立层的文档方向。

## 5. 合入门槛

仅在以下条件全部满足后，才可将本特性描述为“ch0 硬件直显 + ch1 低分辨率检测 + OSD 按需刷新”：

1. `cvlite_demo.py` 实际使用 `Display.bind_layer(..., dstlayer=Display.LAYER_VIDEO1)`；
2. 主循环没有 `snapshot(chn=CAM_CHN_ID_0)` 和视频帧 `Display.show_image()`；
3. OSD 在独立 ARGB 层绘制，且存在明确刷新条件；
4. `test_dual_channel.py` 能正确报告绑定成功或失败；
5. 实机测得显示、检测和 OSD 指标达到第 3 节验收要求。

---

## 6. 二次评审（13:36）

评审范围：以下提交说明及当前工作区代码。

```text
fix: 修正 reject 日志 ROI 错位 + 新增 fps 实测 + 确认 bind_layer v1.8 不兼容
```

### 6.1 已确认修复：`reject` 后日志 ROI 一致

本次代码已将 `rx, ry, rw, rh = rect` 移到跳变拒绝和 `rect` 回退之后。若候选被拒绝，`corners`、`rect`、中心坐标和后续半径都来自上一份有效状态；日志 ROI 不再沿用被拒绝候选的 ROI。

来源：选自 [cvlite_demo.py（第 131～151 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/cvlite_demo.py:131)。

结论：**此项修复通过代码审查。** 建议补一条可复现的 `source=reject` 日志样本，证明 `roi` 与打印的四角外接范围一致。

### 6.2 已确认：加入了端到端 FPS 统计；应修正指标表述

代码在每轮开始执行 `clock.tick()`，每 15 帧打印 `clock.fps()`。这测得的是当前主循环的端到端速率，包含：ch0 `snapshot`、ch1 `snapshot`、cv_lite、在 ch0 上绘制叠加层、`Display.show_image()`、日志等开销。

来源：选自 [cvlite_demo.py（第 91～104、177～181 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/cvlite_demo.py:91)。

现有 `log.md` 尾部记录约为 `56.3~57.9 FPS`，可表述为“当前测试条件下观测到约 56～58 FPS”。不建议写成无条件的“稳定 58 FPS”，原因是：

1. 日志只展示了这一段运行窗口，未给出最小值、平均值、持续运行时间；
2. 当前配置使用 `to_ide=True`，IDE 传输和中断会影响实测；日志最后出现 `Exception: IDE interrupt`；
3. 该 FPS 是当前 RGB565 `snapshot + show_image` 回退方案的端到端值，不代表未来 `bind_layer + OSD` 方案的检测 FPS 或显示刷新率。

建议后续分别记录：`detect_fps`（完整检测循环）、`osd_fps`（叠加层提交）、测试持续时间、最小/平均 FPS，以及 `to_ide=True/False` 两组结果。

### 6.3 未通过：`bind_layer v1.8 不兼容`尚不能由当前材料确认

当前 `test_dual_channel.py` 已修正为正确形式：

```python
Display.bind_layer(**info, dstlayer=Display.LAYER_VIDEO1)
```

来源：选自 [test_dual_channel.py（第 25～40 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/test_dual_channel.py:25)。

但本次提交未附该脚本在 K230D/CanMV v1.8 上的**完整实际输出**，当前仓库 `log.md` 也没有该测试的 `FAILED: ...` 异常文本。因此不能从可审查证据确认“不兼容”；只能确认“存在待复核的实机失败现象”。

另外，脚本在 `sensor.run()` 后没有持续循环、等待或抓帧验证，会立刻进入 `finally` 停止传感器、反初始化显示和媒体管理器。也就是说，打印“Check LCD”后并没有有效观察窗口，无法可靠验证绑定后 LCD 是否真的出图。

来源：选自 [test_dual_channel.py（第 42～67 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/test_dual_channel.py:42)。

结论：**在补齐失败日志前，不应把“不兼容”写为确定事实。** 若实机错误明确为 `unexpected keyword argument 'dstlayer'` 或同等 API 签名错误，应将报错原文、固件完整版本和执行脚本版本写入本文档；若是层资源、像素格式或绑定时序错误，则不是 `dstlayer` 不兼容，必须按实际错误修正。

#### 13:36 后补充：已收到实机失败日志

实机环境：`CanMV v1.8-8-ge38ca78`、`k230d_canmv_atk_dnk230d with K230D`、GC2093（当前输出 `1280×960@90`）。`test_dual_channel.py` 已成功完成：传感器复位、ch0 `640×480 YUV420`、ch1 `320×240 GRAYSCALE`、`Display.init(..., osd_num=1)`，并取得：

```text
bind_info: {'pix_format': 31, 'src': (6, 0, 0), 'rect': (0, 0, 640, 480)}
FAILED: extra keyword arguments given
```

失败发生在：

```python
Display.bind_layer(**info, dstlayer=Display.LAYER_VIDEO1)
```

因此可以确认：**该固件拒绝上述“以关键字展开 `info`，并传入 `dstlayer`”的调用形式。** 这不是传感器配置、YUV 像素格式或 `sensor.run()` 失败；异常发生在绑定调用本身。

但异常文本是通用的 `extra keyword arguments given`，不能严格区分“只是不支持 `dstlayer` 关键字”与“`bind_layer` 在此固件中整体不接收任意关键字参数”。所以准确表述应为：

> CanMV v1.8-8-ge38ca78（ATK DNK230D/K230D）不兼容 `Display.bind_layer(**info, dstlayer=Display.LAYER_VIDEO1)` 这一关键字调用形式；`bind_layer` 的可用位置参数签名尚待隔离验证。

不要表述为“已确认 `bind_layer` 完全不可用”。

下一次诊断应只测试位置参数形式，并将每次结果独立打印：

```python
Display.bind_layer(
    info['pix_format'], info['src'], info['rect'], Display.LAYER_VIDEO1
)
```

若该调用也失败，保存完整异常文本；若成功，则主程序应使用这一位置参数签名。测试绑定成功后应保持运行至少 5 秒，才能目视确认 LCD 上的视频层和 OSD 层实际出图。

### 6.4 当前 `show_image` 回退方案的定位

若经过可复核测试确认当前固件无法完成 `bind_layer`，保留：

```text
ch0: 640×480 RGB565 → snapshot → 绘制结果 → show_image
ch1: 320×240 GRAYSCALE → cv_lite
```

是合理的功能回退方案，且最新日志显示约 56～58 FPS，交互体验可接受。

但必须同步修改代码文件头、提交说明和 `cvlite_dual_channel.md`：它不是“ch0 硬件直显 + OSD 独立层按需刷新”，而是“ch0 彩色软件显示 + ch1 灰度检测”。当前文件头和设计文档仍声称 YUV `bind_layer`、ARGB OSD、零 CPU，与实际实现不一致。

来源：选自 [cvlite_demo.py（第 1～4、79～84、154～175 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/cvlite_demo.py:1)；[cvlite_dual_channel.md（第 1～55 行）](/C:/Users/zhntd/Desktop/DianJiXZ/firmware/K230_Target_Detection/docs/cvlite_dual_channel.md:1)。

### 6.5 二次评审结论

| 项目 | 结论 |
| --- | --- |
| `reject` 的 ROI 日志一致性 | 通过代码审查 |
| FPS 统计功能 | 通过；结果应表述为当前条件下约 56～58 FPS |
| 320×240 检测 / 640×480 显示 / `×2` 映射 | 可保留 |
| `bind_layer` 关键字调用 | 已确认 `**info, dstlayer=...` 形式失败；位置参数签名待验证 |
| 当前软件显示回退路径 | 可运行，但不能称作硬件直显或独立 OSD |
| 窗格误检根因 | 尚未解决，仍需两阶段靶纸身份校验 |

### 6.6 下一步最小闭环

1. 将 `test_dual_channel.py` 改为绑定后持续运行至少 5 秒，或用循环持续显示；捕获并保存完整串口输出。
2. 若绑定失败，将完整异常文本追加到 `log.md` 或单独的诊断日志，记录 CanMV 版本、板卡型号、显示器型号和脚本哈希。
3. 依据实测结果二选一：
   - 绑定可用：按第 3 节完成 YUV `bind_layer + ARGB OSD`；
   - 绑定确实不可用：保留当前 RGB565 + `show_image`，并更正全部文档和提交描述为“软件显示回退方案”。
4. 在任一显示路径上，继续完成黑框 ROI、四边几何和圆环/红点校验；显示性能不应替代靶纸身份校验。
