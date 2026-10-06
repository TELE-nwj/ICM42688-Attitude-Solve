# 架构图与时序图

本目录放两张描述本仓库自身的图，都是**单文件 HTML**（内联 SVG、可交互、支持明暗主题与导出）。

| 文件 | 是什么 |
|---|---|
| `icm42688-architecture.html` | 系统架构图：8 个组件、1 个边界、9 条关系 |
| `icm42688-architecture.spec.json` | 上面那张图的**唯一数据源**（Archify 规格 JSON） |
| `icm42688-200hz-loop.html` | 200Hz 姿态回路时序图：一次 5ms 中断里发生的事 |
| `icm42688-200hz-loop.spec.json` | 时序图的规格 JSON |
| `preview-architecture.png` / `preview-200hz-loop.png` | 供 GitHub 页面直接显示（GitHub 不会内联渲染 HTML） |

## 怎么打开

双击 HTML，或拖进浏览器。页面自带工具栏：切换明暗主题、缩放、导出 PNG/JPEG/WebP/SVG。

## 图描述的是哪个版本

两张图都对应提交 `81fe68ca655052131e9d590ec43f19a58310c5a4`。

规格 JSON 里的每个节点都带 `sources` 字段，指向本仓库的文件与行号，可以逐条回溯到源码。
架构图还额外标出了"片内固件"与两颗外部芯片的硬件边界。

## 怎么重新生成

图由 [Archify](https://github.com/tt-a1i/archify) 3.0.1 生成（MIT 许可），
在 DeepSeek Harness 中通过社区适配包 `@tt-a1i/archify-dsh` 1.0.0 提供为技能。

```bash
node <archify>/bin/archify.mjs finalize architecture \
  docs/diagrams/icm42688-architecture.spec.json \
  docs/diagrams/icm42688-architecture.html \
  --repo-root . --quality showcase --json
```

`finalize` 会依次跑 validate → deliver → 严格 check → 真实浏览器渲染检查，任一关不过就退出非零。

两点注意：

- 规格里的 `meta.output` 记录的是**当初生成时的路径**（`.archify/...`），这里故意没有改成当前路径，
  以免破坏"规格与产物可追溯到同一次生成"的关系。想原地重生成，用 `--out-dir` 指定输出即可。
- 改图**只改规格 JSON**，不要手改 HTML——HTML 是生成物，重跑会整个覆盖。

## 图上写进去的三个发现

读代码时发现、原 README 未提及的三点，已作为卡片画进架构图：

1. `Fusion/imu_fusion.c` 里 6 路 Biquad 二阶滤波已经初始化，但 `biquad()` 的返回值被丢弃，
   喂给 `MahonyAHRSupdateIMU` 的是原始值——**这层滤波从未生效**。
2. `twoKi = 0`，积分补偿分支永远不执行；芯片没有磁力计，Yaw 只能靠陀螺积分，必然缓慢漂移。
3. `ICM_ReadWHOAMI()` 的返回值在 `ICM_Init()` 中未被检查，芯片失联时固件不会报错，
   只会安静地输出错误角度。

以上都是针对上面那个提交的事实陈述；改动代码后请同步重新生成图，否则图文会不一致。

## 许可

HTML 产物内嵌了 Archify 的查看器运行时（MIT）。图所描述的内容属于本仓库自身。
