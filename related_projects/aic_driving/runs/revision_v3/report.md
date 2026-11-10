# 控制修订验证结果

若方法名含recovery，负加速度恢复时允许每周期增加至多2m/s²，其余阶段保留0.4m/s²偏好。候选加速度从6档细化到0.25 m/s²间隔；优先从满足全部原有检查的候选中选取每周期动作变化不超过0.4 m/s²的动作。必要时越过舒适限制执行更强制动。解析与学习方法都获得相同改进。
独立测试前预设：碰撞不增加、平均进度损失不超过1%、平均回合jerk RMS至少降低30%。这些是本次工程验收标准，不是道路安全或乘坐舒适性的认证。

## development

| 方法 | 回合 | 碰撞 | 进度m | 回合jerk RMS均值 | 回合P95耗时均值ms |
|---|---:|---:|---:|---:|---:|
| ca_adaptive | 24 | 0 | 653.180 | 16.288 | 0.477 |
| ml_adaptive | 24 | 0 | 653.418 | 16.677 | 1.091 |
| ca_recovery | 24 | 0 | 652.418 | 8.971 | 2.185 |
| ml_recovery | 24 | 0 | 652.238 | 9.114 | 2.884 |

| 配对差（左减右） | 独立种子块 | 进度差m及95%区间 | jerk差及95%区间 |
|---|---:|---|---|
| ca_recovery − ca_adaptive | 2 | -0.7620 [-1.4512,-0.0728] | -7.3169 [-9.7924,-4.8413] |
| ml_recovery − ml_adaptive | 2 | -1.1796 [-1.3450,-1.0143] | -7.5632 [-9.8039,-5.3225] |
| ml_recovery − ca_recovery | 2 | -0.1794 [-0.4217,0.0629] | 0.1426 [0.0584,0.2268] |

验收：{"ca_recovery": {"collision_no_increase": true, "progress_loss_at_most_1pct": true, "jerk_reduction_at_least_30pct": true}, "ml_recovery": {"collision_no_increase": true, "progress_loss_at_most_1pct": true, "jerk_reduction_at_least_30pct": true}}

## main

| 方法 | 回合 | 碰撞 | 进度m | 回合jerk RMS均值 | 回合P95耗时均值ms |
|---|---:|---:|---:|---:|---:|
| ca_adaptive | 90 | 0 | 638.217 | 20.191 | 0.475 |
| ml_adaptive | 90 | 0 | 638.222 | 20.122 | 1.118 |
| ca_recovery | 90 | 0 | 638.309 | 8.782 | 2.068 |
| ml_recovery | 90 | 0 | 638.334 | 8.752 | 2.818 |

| 配对差（左减右） | 独立种子块 | 进度差m及95%区间 | jerk差及95%区间 |
|---|---:|---|---|
| ca_recovery − ca_adaptive | 10 | 0.0919 [-1.0002,1.2167] | -11.4089 [-13.4780,-8.5930] |
| ml_recovery − ml_adaptive | 10 | 0.1114 [-0.9798,1.2325] | -11.3699 [-13.4377,-8.5644] |
| ml_recovery − ca_recovery | 10 | 0.0248 [0.0065,0.0459] | -0.0298 [-0.1346,0.0675] |

验收：{"ca_recovery": {"collision_no_increase": true, "progress_loss_at_most_1pct": true, "jerk_reduction_at_least_30pct": true}, "ml_recovery": {"collision_no_increase": true, "progress_loss_at_most_1pct": true, "jerk_reduction_at_least_30pct": true}}

## stress

| 方法 | 回合 | 碰撞 | 进度m | 回合jerk RMS均值 | 回合P95耗时均值ms |
|---|---:|---:|---:|---:|---:|
| ca_adaptive | 60 | 0 | 623.085 | 20.844 | 0.463 |
| ml_adaptive | 60 | 0 | 623.138 | 20.604 | 1.053 |
| ca_recovery | 60 | 0 | 622.066 | 10.968 | 2.012 |
| ml_recovery | 60 | 0 | 622.134 | 10.985 | 2.687 |

| 配对差（左减右） | 独立种子块 | 进度差m及95%区间 | jerk差及95%区间 |
|---|---:|---|---|
| ca_recovery − ca_adaptive | 5 | -1.0188 [-1.6115,-0.3659] | -9.8766 [-11.5553,-8.1979] |
| ml_recovery − ml_adaptive | 5 | -1.0045 [-1.6214,-0.3382] | -9.6192 [-11.5234,-7.7149] |
| ml_recovery − ca_recovery | 5 | 0.0679 [0.0114,0.1550] | 0.0172 [-0.1833,0.1858] |

验收：{"ca_recovery": {"collision_no_increase": true, "progress_loss_at_most_1pct": true, "jerk_reduction_at_least_30pct": true}, "ml_recovery": {"collision_no_increase": true, "progress_loss_at_most_1pct": true, "jerk_reduction_at_least_30pct": true}}

## 结论边界

bootstrap按同一种子下的所有条件归块，2000次重采样；区间是未作多重校正的探索估计。仅使用一个旧版冻结网络，未据旧测试重新训练。新测试只有10或5个种子块。
守护与预测拒绝计数分别记录，可重叠，不可直接相加；guard_only_rejected表示仅被制动检查拒绝的候选。它描述判定机制，不是关闭安全检查后的因果收益。
新版解析方案是同样细化与平滑后的强对照；不能把两组共有的控制优化收益称为AI独有收益。完整日志、模型哈希、配置和源码快照保留在本目录。
