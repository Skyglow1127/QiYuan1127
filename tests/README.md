# 测试入口

运行 `python scripts/run_offline_checks.py --suite core`，或在完整 LeRobot / SDK 环境中运行 `--suite all`。

原测试与对应源码同目录保存，以维持相对导入和小型夹具位置。套件中的具体文件清单定义在 `scripts/run_offline_checks.py`，报告写入 `.local/`，不连接硬件。

不要对整个历史源码树直接执行无范围测试发现；部分其他测试需要 ROS、完整资产或专门板卡环境，不属于本次已验证套件。
