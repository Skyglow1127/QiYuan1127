> 来源文档快照：下文的“当前”、路径和命令对应原记录时间；本次选定版本与验证边界见 [提交状态](../../../../docs/RESULTS.md)。

# 独立π0.5无损传输服务18814

本目录是尚未部署的候选服务，远端目录为 `/root/autodl-tmp/piper_pi05_lerobot_lossless_v8`。旧 `lerobot_fast_service`/18812 未修改。模型、tokenizer、处理器、highest精度、denoise_step/reduce-overhead编译、10步flow和50步预测均保持原样；`runtime.py` 与 `pi05_low_memory.py` 和旧服务逐字节相同。13项离线验证结果见 `offline_verification.json`、`wire_codec_test_result.log`。离线验证没有连接网络或机器人。

`wire_codec.py` 是客户端与服务端共享的纯传输模块。固定接口：

```python
RAW_CODEC = 'raw-npz-v1'
PNG_CODEC = 'png-rgb-u8-v1'
encode_payload(fields, codec)  # -> bytes
decode_payload(payload, codec)  # -> 原字段dict[np.ndarray]
decoded_fields_sha256(fields)  # -> 所有字段名/dtype/shape/C连续字节的SHA256
```

HTTP仍为 `application/x-npz`，通过 `X-Image-Codec` 选择格式。省略header按raw处理，便于对照；未知codec拒绝。PNG模式仅把两个已经完成原缩放和uint8量化的RGB CHW图像无损编码为PNG，原键名保持不变。OpenCV编码前RGB转BGR，PIL解码显式RGB；网络解码后恢复RGB CHW uint8，state和可选noise保持原始float32字节。两种格式共用同一模型实例、推理锁与持久推理线程。服务响应回传codec及decoded_fields_sha256，客户端可检查实际进入模型的内容。

canonical固定输入：raw 213163字节；PNG 157308字节，减少26.20%。本机10次离线编码中位raw 5.07ms、PNG 8.25ms，解码中位raw 1.49ms、PNG 2.68ms。这不是云端传输或端到端测速。两种格式所有解码字段摘要相同：`0d3d81a840894a14c25cc9609d063404b2a514e48862fd48a73a297dc7bd1415`；逐图像、state及noise字节SHA记录于离线报告。

health.metadata严格提供 `wire_image_codecs=[RAW_CODEC, PNG_CODEC]`、`production_image_codec=PNG_CODEC`；生命周期控制保留原模型/813张量/精度/编译校验并增加codec pins。`GET /ping`只返回ready、startup_id、model_id和codecs，无GPU计算。本目录未启用保活线程。

server_timing_s保留preprocess/model/postprocess/decode/total，并增加header_validate、body_read、input_sha、npz_decode、image_decode、field_validate、decoded_sha、dispatch、worker_queue、worker_execution。decode/total是包含子阶段的汇总，不能把全部字段相加。request_log及handler_to_reply只在HTTP响应中提供，原请求日志本身不包含对这次写日志的自测结果。所有这些都是性能时间，不能作为相机观测时间或刷新旧动作来源。

开始候选部署前应确认没有实机进程，先用18812自身stop释放显存，再启动18814；32GB卡不同时加载两份完整模型及graph。以下由部署负责方执行，当前离线准备并未执行：

```bash
bash /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/lerobot_fast_service/stop.sh
bash /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/lerobot_lossless_service/start.sh
bash /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/lerobot_lossless_service/status.sh
```

同服务、同canonical、同noise交替A/B，默认每种3次预热、20次测量，每次测量前空闲1秒，以覆盖20步队列的真实间隔：

```bash
/home/aup/Lerobot/envs/piper-cpu/bin/python -B /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/lerobot_lossless_service/client.py \
  --config /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/lerobot_lossless_service/cloud_config.json \
  --url http://127.0.0.1:18814 \
  --input /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/lerobot_lossless_service/canonical_input.npz \
  --codec ab --idle-s 1 --repeats 20 \
  --output /tmp/pi05_lossless_ab_new
```

此CLI只发送录制输入，不连接相机、CAN或机器人。每个响应的模型身份、解码字段摘要和固定noise动作元素均须与首个响应精确相等。测速包括每次无损编码/哈希、HTTP、推理和验证，排除采集/反馈/控制和指定空闲；它不能单独证明实机400ms来源预算或抓取成功。旧服务仍可回退；回退应先停18814再起18812，并同步客户端配置和新审阅计划。
