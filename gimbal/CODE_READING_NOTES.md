# Gimbal 工程阅读笔记

记录日期：2026-11-13。依据当前源码静态阅读；未编译、烧录或进行实机验证。参数可能继续变化，修改前应重读配置。

## 主链路

- STM32F407，CMake + ARM GCC，C++20/C11；Core 为 HAL 外设初始化，Resources 为电机及 CAN 驱动，Tasks 为业务，components 为 PID、陀螺零偏及 Mahony/BMI088。
- `Core/Src/main.c`：HAL/时钟 → FaultLogBoot(2) → 外设 → MainInit → 循环 MainPoll、FaultLogPoll。
- `Tasks/main_task.cpp`：软件初始化 → CAN 启动 → TIM6 中断 → IMU 初始化 → IWDG。TIM6 中断执行独立安全检查并置调度标志；前台执行 ImuUpdate、GimbalStep。
- `single_axis_config.hpp::kEnabled=false`，当前为 integrated 双轴；kSelectedAxis=Pitch 只选择旧调试曲线关注轴。

## 通信及电机

- yaw：CAN1，控制 ID3，方向 +1；pitch：CAN2，控制 ID9，DM4340，方向 -1，内部正方向抬头。
- CAN1 FIFO0、CAN2 FIFO1 由 HW_can.cpp 统一读取，再调用 GimbalOnCanRx；不要新增重复 HAL 回调或重复读取 FIFO。
- 下板遥控命令 CAN1/0x300，上板每 10 ms 回传 0x301；CRC8、序号检查；重复样本不刷新有效命令，100 ms 过期。上板不启动本地 USART3 DT7 接收。
- dm_parameters 通过 0x7ff 查询寄存器 7/8/10/21/22/23，读取反馈 ID、ESC ID、模式及 P/V/T 编码范围。必须完整且 MIT 模式正确才允许驱动；不要仅凭控制 ID 猜反馈 ID 或范围。
- 两轴使用绝对位置；yaw 连续展开。MIT 原生量程跨界和机械 2π 回绕分别处理；continuous yaw 当前不因单独位置跳变拒收，pitch 保留跳变拒收。
- torqueFrame 使用 MIT 的纯力矩控制（位置/速度/Kp/Kd 为零），并按方向和传动比换算。

## 控制

- gimbal_runtime：HAL、发送/使能/停机事务、调试量；gimbal_dual_axis：参数发现、反馈和故障锁存；gimbal_task：Dead/Preparing/Working 状态机及控制；components/control/pid：滤波、测量微分、抗饱和及接近目标制动。
- 摇杆积分成角度目标，位置外环 → 速度参考限幅/斜坡及制动 → 速度内环 → 重力/摩擦前馈 → 力矩限制。
- Joint 模式使用电机反馈；Imu 模式 yaw 使用连续 IMU yaw 和欧拉 yaw rate。pitch IMU 已禁用，始终使用电机角/速度；切模式保留 pitch 目标，首帧输出力矩归零。
- 首次 pitch 目标为原始电机角 -17°（内部 +17°）；yaw 捕获当前位置。准备阶段有开机保护、回中计数、两轴零力矩使能反馈握手。
- 当前配置：yaw 外环 675/0/16，内环 0.75/0.1/0；手动速度 2 rad/s，位置修正上限 3 rad/s，力矩配置 7.5 N·m（按实际电机范围下调），超速 14 rad/s。
- pitch 外环 250/0/9.5，内环 8/0.1/0；速度 2 rad/s，力矩 28 N·m，超速 10 rad/s；重力 7.1 N·m，正/反摩擦 0.6/1.6 N·m，回中时关闭方向摩擦前馈。
- pitch 原始电机边界 -30°～+4.5°，内部 -4.5°～+30°；5° 减速区，目标留 0.5° 内侧余量。当前 pitch_limit_shutdown=false，边界为软保持/恢复，保留重力及向内制动力矩。
- yaw 额外保持弹簧开关、pitch basic cascade 开关、pitch neutral velocity quiet 开关当前均关闭；勿把保留的实验分支当作生效路径。

## IMU 与安全

- BMI088 → 2000 静止样本零偏 → Mahony → 连续 yaw；欧拉 yaw rate 由机体系角速度转换，不能简单替换为 gyro Z。
- imu_config：安装矩阵单位阵，mounting confirmed=true，yaw sign=+1，pitch IMU=false。初始校准允许采样间隔 10 ms，校准后和控制允许 19 ms，反馈新鲜度要求 <20 ms。
- 定时中断独立检查指令及前台 20 ms 冻结；IWDG 只在新有效下板指令、RC 在线且前台进展新鲜时刷新，电机反馈不喂狗。
- 停机顺序：中止旧发送 → 等待中止 → 零力矩 → FD 失能 → 等完成 → 周期失能；重新使能必须等停机事务完成。不会发送 FE 永久置零命令。
- HAL_BUSY 为背压并重算重试，实际持续失败阈值使用 max_control_gap_ms（当前 19 ms）；HAL_ERROR 触发发送故障。代码中的部分 5 ms 注释已过期。
- 运行期故障锁存至明确遥控停止/离线帧确认。yaw 超速持续 8 ms 或达到阈值 1.5 倍立即触发；pitch 超速立即触发；堵转 1500 ms。
- fault_log：工作后停机、工作中复位和 CPU 异常快照；停止输出后等 50 ms 再写 Flash；CRC 日志，不自动擦除。关注 g_dbg_reasons、g_dbg_fault_latched、g_dbg_last_fault_reason 和 dbg_fault_log_*。

## 后续工作约定

- 以有效配置及调用路径为准；UPPER_MODULES.md 和少量头文件注释保留历史内容，不能直接认定当前生效行为。
- tests 中有电机、控制、runtime HAL stub、双轴、IMU、零偏时序和日志主机测试。本次只读源码，未重新执行，历史通过记录不代表当前已验证。
- 后续修改先定位对应层，保留 CAN 回调归属、上下板协议、方向/绝对零点、现有调参和保护；未授权前不修改控制代码或刷写硬件。
