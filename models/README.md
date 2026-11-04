# 模型和数据下载说明

本目录提供复现所需的模型身份、配置和处理器统计。神经网络大权重与完整数据集保留在仓库外。

## 选定资产

实际发布状态、模型仓库链接见 [DOWNLOADS.md](DOWNLOADS.md)，精确版本和每个文件的 SHA-256 见 [manifest.json](manifest.json)。

默认发布并下载 ACT 020000（约 207 MB）和 π0.5 best004000（约 9.35 GB）。π0.5 final020000（约 9.35 GB）作为可选对照保留在本机，默认不重复发布。best004000 仓库内包含完整配套 tokenizer、来源记录和上游 Gemma 条款。完整数据集（52 条示范、20488 帧、20 Hz）暂不公开；元信息在 `examples/dataset_metadata/`。

训练基座来源为 [lerobot/pi05_base](https://huggingface.co/lerobot/pi05_base)，受原模型条款约束。专家 B 发布包已经是完整模型，推理不需要为了拼接 adapter 再下载整个基座；若重新训练，则需准备相应基座与数据。

## 自动下载

在仓库根目录执行：

```bash
python -m pip install -r requirements-hub.txt
python scripts/download_models.py --list
python scripts/download_models.py
# 仅需要 ACT 时：
python scripts/download_models.py --models act_020000
```

脚本按清单内不可变的 Hugging Face commit 下载完整目录，校验每个文件的大小和 SHA-256，然后写入 `.local/downloaded-assets.json`。网络中断后可重跑；缓存由 Hugging Face 管理。校验失败不写入该模型的资产映射。未发布的条目不会使用虚构链接下载。

```bash
python scripts/prepare_workspace.py --output .local/workspace-downloaded --assets .local/downloaded-assets.json
```

下载不启动机器人。配置中的历史训练路径及部署动作步数需要按复现说明处理。

## 维护者发布

```bash
hf auth login
python scripts/publish_models.py          # 只准备本机上传目录和计划
python scripts/publish_models.py --upload
# 需要发布最终对照模型时：
python scripts/publish_models.py --models pi05_final020000 --upload
```

原资产位置通过 `--assets` 指定 JSON，默认 `.local/artifact_locations.json`。准备目录 `.local/hf-staging/` 中的大权重只是指向原文件的只读用途链接；上传脚本不会修改原模型。默认使用已登录的 Hugging Face 用户名创建公开模型仓库。脚本会拒绝覆盖远端不同权重，上传后以匿名访问检查远端大小及 SHA-256，验证成功才更新公开清单和下载页面。未验证成功的上传不标记为已发布。

## 获取后如何放置

每个模型应恢复成完整目录：

```text
pretrained_model/
  model.safetensors
  config.json
  train_config.json
  policy_preprocessor.json
  policy_postprocessor.json
  policy_preprocessor_step_3_normalizer_processor.safetensors
  policy_postprocessor_step_0_unnormalizer_processor.safetensors
```

本仓库已包含除大权重外的配套文件，分别位于同名模型 ID 子目录。不能把 ACT 的统计换给 π0.5，也不能把 LoRA adapter 当完整专家 B 加载。

下载文件后可计算 `sha256sum model.safetensors` 并对照清单。完整数据集应保留其 `meta/`、`data/`、`videos/` 结构；仓库中的元信息只是示例，不是训练数据本身。

将资产目录填入 `configs/assets.example.json` 的副本 `.local/assets.json`，再运行 README 中的路径准备命令。tokenizer 需完整目录，不能仅下载一个 tokenizer.model。

## 本机已有资产

整理任务已经生成 `.local/artifact_locations.json`，指向原本机权重、数据、基座和 tokenizer。它被 Git 忽略，不会泄漏为无效的“公开下载地址”。在本机可以直接把它传给 `scripts/prepare_workspace.py --assets`，脚本只建立本地链接，不复制十几 GB 大文件。

## π0.5 + LoRA 本地推理实验的模型

[本地推理实验](../experiments/pi05_lora_local_inference_v10/README.md) 使用另一份 LoRA 020000 适配器，SHA-256 为 `8e064a68c5991f1b1321e726d901968c516ec728971aa4b9eb1fc6d4213f14a1`。本次收录程序和结果，未发布该适配器；上面的专家 B 完整模型下载不能替代它。复现时需准备报告对应的基座、适配器和 tokenizer。
