---
name: never-push-teammate
description: 严禁向队友仓库推送，merge 仅单向拉取
metadata:
  type: feedback
---

**绝对不要向队友仓库（yangran）推送任何内容。** 只能 `git fetch yangran` + `git merge yangran/main` 单向拉取。origin 的 main 分支也要谨慎——merge 队友代码后先在本地确认无误再 push。

**Why:** 队友有自己的开发节奏，我们的代码不应该写到他们仓库里。

**How to apply:** 任何涉及 `yangran` remote 的操作只做 fetch/merge，不做 push。push 只推 origin。
