# 项目变更记录

本文档记录后续会话需要了解的开发变更。正式版本发布历史仍记录在 `CHANGELOG.md` 中。

## 未发布

### 2026-10-29

#### 修复

- 修复 P1 M605 读取空 SN 时的解析异常；屏幕固件烧录后返回 `SN:` 将正常返回空字符串 `""`，不再重试或报超时。读取已设置的 SN 也改为保留原始字符串及前导零。

#### 变更

- P1 M604 设置 SN 的格式校验调整为 `ERUAP` 加 11 位数字，共 16 位，例如 `ERUAP12608000001`。
- P1 M604 设置 SN 时自动将小写字母转换为大写后再校验和发送，例如 `eruap12608000001` 会发送为 `ERUAP12608000001`。

#### 验证

- `tests/test_utils.py` 共 23 项测试通过，包含空 SN 响应、M604 SN 格式校验及小写 SN 自动大写的回归测试；`pymycobot/ultraarm_p1_base.py`、`pymycobot/error.py`、`pymycobot/ultraarm_p1_internal.py` 通过语法编译检查。

### 2026-09-28

#### 修复

- `set_zero_calibration()` 发送 M30 后，延后 0.6 秒再开始 M200 运动状态查询，避免固件内部延迟启动期间连续返回 `MainMoving:0` 而被提前判定到位。
- 补充 P1 碰撞错误码 `4` 的 J4 映射；校准时收到 `$CollisionDetectionError:4` 将返回 J4 碰撞及 `collision_unlock()` 解锁提示。

#### 验证

- `tests/test_utils.py` 共 16 项测试通过，包含 M30 校准状态查询延迟和 J4 碰撞错误码的回归测试；`pymycobot/ultraarm_p1_base.py`、`pymycobot/robot_info.py` 通过语法编译检查。

### 2026-09-17

#### 新增

- 新增 ultraArm P1 的运动暂停、运动恢复和关节加速度接口，对应 M6、M7、M8、M9 协议指令。
- 新增 P1 M92 编码器位置更新接口 `update_encoder_data()`，不提供兼容别名。
- 在中英文使用文档中补充新增的 P1 接口说明。
- 新增项目规则，要求 Cursor 和 Codex 会话读取并维护本文档。

#### 变更

- P1 M200 运动到位判断改为连续收到三次停止状态后才确认到位。
- `get_default_sensor_initialize(sensor_type)` 的响应超时时间调整为 500 毫秒，并补充传感器类型 1～7 的说明。

#### 修复

- 修复 P1 M110 对单个及多个浮点传感器数值的解析。
- 修复 RFID 传感器响应解析，例如将 `85 8C D5 AB` 拼接后返回卡号 `858CD5AB`。

#### 验证

- P1 专项单元测试覆盖 M6、M7、M8、M9、M92、M110 传感器解析及 M200 连续三次停止确认。
- `tests/test_utils.py` 共 14 项测试通过；`pymycobot/ultraarm_p1_base.py` 通过语法编译检查。
- 完整测试集仍存在与本次修改无关的收集错误，涉及 `tests/test_generator.py` 和 `tests/test_socket.py`。
