# FashionStar 库本地修复源码

状态：仅编写源码；按用户最新要求，未运行测试、编译、链接验证、真实串口或部署。没有伪串口测试通过的声明。

原始文件及 SHA256 保存在 `original/` 与 `ORIGINAL_SOURCE_MANIFEST.json`。原始源目录的旧构建库与现役安装库 SHA 不同；本补丁需要未来将私有库、新头文件和新 controller 配套编译使用，不能把当前源码等同于现役二进制。

接口：

```cpp
struct FSUS_AngleQueryResult {
    FSUS_STATUS status;
    bool valid;
    FSUS_SERVO_ID_T servo_id;
    FSUS_SERVO_ANGLE_T angle;
};
FSUS_AngleQueryResult FSUS_Servo::queryRawAngleChecked(uint32_t timeout_ms = FSUS_TIMEOUT_MS);
FSUS_STATUS FSUS_Protocol::recvQueryAngleChecked(FSUS_SERVO_ID_T expected_id,
                                                FSUS_SERVO_ANGLE_T* angle,
                                                uint32_t timeout_ms = FSUS_TIMEOUT_MS);
FSUS_STATUS FSUS_Protocol::recvPack(uint32_t timeout_ms);
```

默认超时 100 ms。旧 `recvPack()` 保留，委派默认超时；旧 `queryRawAngle()` 保留并使用 checked 路径，调用者仍须检查 `responsePack.recv_status`，旧 float 返回类型不能独立表示有效性。新增 checked 结果的 `valid` 是有效性依据；失败时 `angle` 只是保留缓存，不能刷新最后有效测量时刻。构造函数明确初始化缓存和状态，避免动态分配对象首次查询失败时读取未初始化值。

单圈角度反馈必须同时满足接收成功、校验和正确、响应命令 QUERY_ANGLE、长度 3、ID 与请求匹配，才更新输出和 `curRawAngle`；解析错误、超时均保留缓存。原有 uint8_t 状态码保留，新增 RESPONSE_CMD_MISMATCH=9、RESPONSE_SIZE_MISMATCH=10、INVALID_ARGUMENT=11。valid 只证明这一协议响应通过校验；物理关节范围、测量年龄和安全窗口仍由 controller 判断。协议没有请求序列号，不能仅靠 ID/cmd/长度完全识别同轴的迟到旧响应。

Linux 超时改用 CLOCK_MONOTONIC。timeout_ms 限定接收循环预算，不是从排队、锁等待、清缓存到写串口的整个调用实时截止保证；controller 应提前留调度余量并记录实际耗时。完整总线事务的互斥由 controller 持有，本库未添加字段或改变对象成员布局。

`sendPack()` 构造局部完整报文，只调用一次 writeData；返回值必须等于包长，否则抛出 std::runtime_error，并置 responsePack.recv_status=FAIL。绝不自动重发部分写出的报文。发送/时钟异常可从 checked query 上抛，controller 必须捕获。没有把串口写入成功冒充固件已采纳或到位。接收调试缓冲加了界限保护，避免最大载荷加帧头尾超过该调试数组大小。

当前 CMake 只生成私有 `build/lib/libfsuartservo.so` 和 `build/include/FashionStar/UServo/`，没有全局安装规则。尚未执行的未来构建方式：
普通 `sendSetAngle` 的角度按协议 0.1° 分辨率四舍五入编码，避免浮点表示被截断为少一个单位；其余角度接口未改。调用者必须先验证有限值与关节限位。

```sh
cmake -S library -B library/build
cmake --build library/build
```

controller 应使用匹配的 `library/build/include` 和 `library/build/lib`，运行时通过私有 LD_LIBRARY_PATH 指向对应目录；当前未执行上述操作。`original/` 仅保留证据，不能作为补丁构建入口。
