PiPER ACT 云端训练配置（v5）

云实例：RTX 5090 D 32GB，25 vCPU，92GB 内存。
云端目录：/root/autodl-tmp/lerobot_act
SSH：ssh -p 29596 root@gpu.example.invalid
密码不保存在本配置中，登录时按终端提示输入。

环境使用官方 LeRobot 0.6.1，无框架源码修改。
独立 venv 复用云镜像预装的 PyTorch 2.8.0+cu128 / torchvision 0.23.0+cu128，
其余依赖安装在数据盘的 .venv；TorchCodec 固定 0.7.0 与 PyTorch 2.8 匹配。
不需要重新安装 NVIDIA 驱动或完整 CUDA Toolkit。

已上传完整数据：
  /root/autodl-tmp/lerobot_act/datasets/rad_place_merged_v5
52 条、20488 帧、20 FPS；global + wrist 两路 640x480 RGB。
7 维状态/动作：六关节 deg，夹爪 m。使用全部 52 条，不划分验证集。
传输完整性：dataset.sha256；云端 logs/dataset_integrity.log。

正式配置：
  官方 lerobot-train，ACT，CUDA，FP32（policy.use_amp=false）
  默认 ResNet18 ImageNet 预训练视觉骨干，chunk_size=100
  batch_size=8，num_workers=0，persistent_workers=false，视频解码后端 TorchCodec
  steps=20000，save_freq=2000，log_freq=100
  dataset.eval_split=0.0，eval_steps=0，env_eval_freq=0
  关闭 W&B 和 Hugging Face 自动上传

登录云端后，预览命令（不开始训练）：
  bash /root/autodl-tmp/lerobot_act/train.sh --dry-run

开始正式 2 万步训练（后台 tmux，SSH 断开不停止）：
  bash /root/autodl-tmp/lerobot_act/start.sh

查看训练终端：
  tmux attach -t act_train
退出查看并保留训练：先按 Ctrl+B，再按 D。
运行期间不要关闭或释放云实例。

结果目录：
  /root/autodl-tmp/lerobot_act/outputs/act_rad_place_日期时间/
日志目录：
  /root/autodl-tmp/lerobot_act/logs/
本次正式输出路径会写入 logs/latest_act_rad_place_output.txt。
最后一个 checkpoint 的部署文件位于：
  outputs/act_rad_place_日期时间/checkpoints/last/pretrained_model/
保留完整 checkpoint 以便恢复优化器和随机状态，不能只留 model.safetensors。

断点续训到总计 2 万步（替换为实际绝对路径）：
  bash /root/autodl-tmp/lerobot_act/train.sh --resume /ABS/OUTPUT/checkpoints/last/pretrained_model/train_config.json
需要保持 SSH 会话时可在 tmux 交互终端执行该续训命令。

环境与传输、短测结果位于 verification/ 以及云端 logs/。
100 步为配置与保存功能验证，不代表任务已学会；正式训练尚需手动执行 start.sh。
云镜像当前的多进程读取在退出时出现 worker Aborted 清理告警；已采用单进程读取，
神经网络仍在 CUDA GPU 上训练。FP16 首轮存在非有限梯度，最终配置采用默认 FP32。
最终配置实测稳态约 0.463 秒/步，2 万步计算和读取约 154 分钟；
考虑启动及保存，先预留 2.5–3 小时。该时间由短测外推，不保证最终收敛或实机成功。
已检查完整模型参数有限，并验证从第 100 步恢复到第 105 步；保留全部训练状态。
