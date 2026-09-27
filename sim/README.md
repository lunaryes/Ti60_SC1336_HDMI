# sim/ — 仿真基础设施（验证组负责）

> **本目录不参与综合**，不会被登记进 `Ti60_Demo.xml`。
> 除集成负责人外，所有人的开发都在这一层完成，**不需要安装 Efinity**。

## 目录

```
sim/
├── tb/           测试台：tb_<模块名>.v，与模块一一对应
│   └── tb_lib.vh 公共断言宏、视频流激励/接收 task
├── vectors/      激励向量：*.hex、*.txt（Python 生成的期望输出）
├── scripts/      运行脚本
└── out/          （.gitignore）编译产物、日志、波形
```

## 用法

```bash
# 在仓库根执行
bash sim/scripts/run_iverilog.sh \
    Ti60F225_SC1336_HDMI_Display/src/image/pix_window_3x3.v \
    sim/tb/tb_pix_window_3x3.v
```

## 三条约定

1. **TB 必须断言 `LATENCY`**：数据必须与 `vsync_o`/`de_o` 严格同拍，错一拍也算失败。
2. **用 16×16 这类小图**。`IMG_W` 必须参数化——小图能让仿真从秒级降到毫秒级，
   否则 1280×720 每跑一次都要等很久，开发节奏会被拖垮。
3. **边界必须覆盖**：首行、末行、首列、末列四种情况全都要有激励。
   边界是这类流水线出错最多的地方。

## 计划中的 TB（按交付单元）

| TB | 对应模块 | 负责人 |
| --- | --- | --- |
| `tb_lib.vh`、`video_stream_gen.v`、`video_stream_sink.v` | 公共激励/接收（先做，所有人受益） | 验证组 |
| `tb_pix_window_3x3.v` | U1 窗口发生器 | U1 作者 |
| `tb_sobel_edge.v` | U2 Sobel | U2 作者 |
| `tb_image_mix.v` | U4 显示合成 | U4 作者 |
| `tb_image_pipeline_top.v` | U3 顶层（含旁路一致性） | 集成负责人 |

> **优先级提示**：`video_stream_gen` / `tb_lib.vh` 应该最先做。
> 没有它们，每个人都要自己手搓激励，LATENCY 断言也会各写各的。
