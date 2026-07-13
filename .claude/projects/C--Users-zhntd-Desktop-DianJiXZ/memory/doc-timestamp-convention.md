---
name: doc-timestamp-convention
description: 文档更新时间戳规范——每次编辑 md 必须同步更新
metadata:
  type: feedback
---

每次修改 `docs_learn/*.md` 或 `docs_2024/*.md` 时，必须同步更新**两处**时间戳：

1. **顶部标题行** — `[← 学习笔记](.) | 最后更新：YYYY-MM-DD HH:MM`
2. **末尾行** — `> 最后更新：YYYY-MM-DD HH:MM`

用 `date "+%Y-%m-%d %H:%M"` 获取当前时间，精确到分钟。

**Why:** 文档频繁更新，没有时间戳无法判断信息时效性。队友看文档时要知道"这个知识点是什么时候写的"。

**How to apply:** 编辑 md 文件前先读顶部和底部确认当前时间戳，编辑完成后用 Edit 工具把两处都更新。不要只改一处。
