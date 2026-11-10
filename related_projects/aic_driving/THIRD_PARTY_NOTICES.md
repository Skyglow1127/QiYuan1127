# 第三方组件与开发贡献说明

本项目使用开源仿真、数值计算、学习和绘图组件。以下版本及许可证名称来自当前成功运行的 Python 包元数据；实际再分发这些包或其二进制时，应附带对应版本的完整许可文件。源码复现包通过依赖安装使用这些组件，不把它们列为团队原创。

| 组件 | 当前版本 | 本项目用途 | 元数据中的许可 |
|---|---|---|---|
| [highway-env](https://github.com/Farama-Foundation/HighwayEnv) | 1.12.1 | 道路、车辆运动、碰撞、连续动作接口 | MIT |
| [Gymnasium](https://github.com/Farama-Foundation/Gymnasium) | 1.4.0 | 环境接口及空间定义 | MIT |
| [NumPy](https://github.com/numpy/numpy) | 2.5.4 | 数值计算 | BSD-3-Clause、0BSD、MIT、Zlib、CC0-1.0 的组合，见安装包 |
| [PyTorch](https://github.com/pytorch/pytorch) | 2.14.1+cpu | 小型残差网络训练与推理 | 多种 Apache/BSD/MIT 等许可组合，见安装包 |
| [Matplotlib](https://github.com/matplotlib/matplotlib) | 3.11.2 | 曲线和回放画面 | Matplotlib 许可协议，见安装包 |
| [pygame-ce](https://github.com/pygame-community/pygame-ce) | 2.5.8 | highway-env 依赖 | LGPL v2.1 |
| [ImageIO](https://github.com/imageio/imageio) | 2.38.1 | 回放视频写入 | BSD-2-Clause |
| [imageio-ffmpeg](https://github.com/imageio/imageio-ffmpeg) | 0.6.0 | 视频编码接口 | Python 包为 BSD-2-Clause；捆绑 FFmpeg 二进制的许可另见发行包 |

完整 Python 依赖版本记录在 `requirements.lock`、`requirements-torch.lock`，每次正式运行另存环境信息。版本表不替代第三方完整许可证。

项目自身实现包括：三类参数化场景、模拟时间延迟队列、稳定目标 ID 的历史缓存、解析与学习残差预测的组合、经验误差标定、候选加速度规划与制动检查、配对实验和报告流水线。CA/CV、MLP、分位数裕度与有限候选规划均属于已有方法；组合实现不能直接称为新算法发明。

本次实现使用 AI 辅助编写代码、排错、运行实验、整理文档及审查。参赛团队负责问题定义、范围选择、事实核验、理解方案、结果解释与最终材料。提交时按比赛实际要求如实说明 AI 辅助，不虚构团队独立手写代码、真实车辆实验或未进行的验证。
