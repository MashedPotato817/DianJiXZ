---
name: two-device-workflow
description: 双设备工作——记忆统一放 shared/ 目录，不按设备分裂
metadata:
  type: project
---

两台设备（Mashed Potato + zhntd）通过同一项目文件夹同步，记忆不需要按设备分目录。

## 记忆位置

所有项目记忆统一放在 `.claude/projects/shared/memory/` 下。跨设备唯一目录，不存在迁移问题。

## 为什么不用设备专属目录

Claude Code 默认按机器名创建 `C--Users-<xxx>-<path>/memory/`，但两台设备共享同一个项目文件夹时，各写各的会导致记忆分裂。统一用 `shared/` 解决。

## 注意事项

- 新记忆直接写入 `shared/memory/`
- 如果 Claude Code 自动写入设备目录，事后移入 `shared/memory/` 并更新 MEMORY.md
- `shared/` 随 git 提交，换设备自动同步

**Why:** 两台设备交替工作时，`.claude/projects/C--Users-<device>/memory/` 各有一套，A 设备写的记忆 B 设备看不到。统一 `shared/memory/` 消除这个问题。

**How to apply:** 发现 `.claude/projects/` 下有新设备目录时，检查 `shared/memory/` 是否已有全部记忆——有则忽略设备目录，没有则从设备目录移入。
