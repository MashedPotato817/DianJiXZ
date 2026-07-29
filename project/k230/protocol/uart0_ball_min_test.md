# K230 到 MSPM0 UART2 最小联调协议

## 固定参数

- K230：CanMV `UART1`，IO40 为 TX、IO41 为 RX。
- MSPM0：`UART2`，PB18 为 RX、PB17 为 TX。
- 串口：115200，8N1；两端共地，信号线交叉连接。
- K230 数据帧：`$K230,BALL,<x_mm>,<valid>,<seq>#`。
- MSPM0 心跳：每 500 ms 发送 `$MSPM0,HELLO#\r\n`。
- 主控超时：连续 100 ms 未收到完整合法帧时，将位置 `valid` 置为 `0`。

`x_mm` 为相对摆杆中心的毫米坐标，右正左负；`valid` 只能为 `0` 或 `1`；`seq` 为递增无符号序号。

## 最小实机步骤

1. 向 K230 上传并运行 `project/k230/k230_uart_ball_test.py`。
2. 烧录当前 MSPM0 固件，不接入舵机或 `Ball_Control_Step()`。只保留 K230 IO40 到 MSPM0 PB18 和共地；回传线异常时可暂不连接 IO41。
   收到第一帧合法 `BALL` 数据后，原本呼吸闪烁的板载 LED 将改为常亮。
3. 在 Keil Watch 中观察 `g_diagnostics` 和 `g_ball_position`：
   - `rx_bytes` 持续增加；
   - `valid_frames` 持续增加；
   - `parse_errors` 保持 0；
   - `last_seq` 递增；
   - `g_ball_position.x_mm` 为 `-25.0`、`valid=1`。
   K230 控制台同时应显示一次 `MSPM0 BALL frame acknowledged`；该 ACK 不依赖 5 ms 时基。
4. 停止 K230 脚本或断开 TX 后，最多 100 ms 内确认 `g_ball_position.valid=0`、`timed_out=1`。

通过以上四项仅证明 UART2 收发与解帧可用，不代表视觉识别、舵机或控球闭环已验证。

## 回传线单向验证

1. 暂时断开 K230 IO40 到 MSPM0 PB18 的发送线。
2. 仅连接 MSPM0 PB17 到 K230 IO41，并保持两端共地。
3. 运行 `project/k230/k230_uart_rx_test.py`，MSPM0 保持烧录当前固件。
4. 控制台每 500 ms 应收到 `$MSPM0,HELLO#` 并显示 `MSPM0 UART2 TX: PASS`。

若显示乱码或无数据，不能进入双向联调；先修复 UART2 PB17 到 IO41 的回传线或其接口映射。

联调 LED 含义：慢闪表示链路初始化未执行发送；2 Hz 闪烁表示 MSPM0 已发送但 UART2 未读到字节；0.5 Hz 闪烁表示 UART2 已读到原始字节但未解出有效帧；常亮表示已经收到并解出一帧合法 `BALL` 数据。

## MSPM0 UART2 本地回环验证

断开 K230 后短接 MSPM0 PB17（UART2_TX）到 PB18（UART2_RX），重启当前固件。LED 在最短回环令牌 `$L#` 或首次 `$MSPM0,HELLO#` 回读后会熄灭，表示 PB17 输出、PB18 输入和 UART2 的波特率均正常；保持 2 Hz 闪烁则表示未收到回读数据。
