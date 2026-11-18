# WL_P311D 轮腿机器人仿真

基于 Webots R2025a 的四轮腿机器人 C++ 控制工程。使用同一个模型演示轮式行驶、低速爬行和对角小跑，支持小跑原地转向与边走边转。场景和模型资源随项目提供。

## 编译并运行

准备 Webots R2025a 和 C++17 编译环境。以下流程在 Windows 上验证。编译依赖 Webots 安装自带的头文件、控制器库和 Makefile。

1. 克隆或解压仓库，保留目录结构。在 Webots 打开 [worlds/first_demo_cpp.wbt](worlds/first_demo_cpp.wbt)。
2. 在 Webots 文本编辑器打开 [wl_drive_cpp.cpp](controllers/wl_drive_cpp/wl_drive_cpp.cpp)，选择 **Build → Build**。也可用下方的 PowerShell 命令编译。[Webots 编辑器说明](https://cyberbotics.com/doc/guide/webots-built-in-editor?version=R2025a)
3. 编译完成后，在 Webots 重载世界并点击运行。等控制台出现 `Stand=HOLD reached=1`，然后单击 3D 视图，再输入按键。

<details>
<summary>Windows PowerShell 编译命令</summary>

在仓库根目录打开 PowerShell。将第一行改为本机的 Webots 安装位置。

```powershell
$env:WEBOTS_HOME = 'C:/Webots'
$env:OS = 'Windows_NT'
$webots = $env:WEBOTS_HOME
$env:Path = "$webots/msys64/mingw64/bin/cpp;$webots/msys64/mingw64/bin;$webots/lib/controller;$webots/msys64/usr/bin;$env:Path"

Push-Location './controllers/wl_drive_cpp'
try {
    & "$webots/msys64/usr/bin/make.exe" -j3 OS=Windows_NT BUILD_TYPE=controllers
    if ($LASTEXITCODE -ne 0) { throw "Webots controller build failed ($LASTEXITCODE)." }
} finally {
    Pop-Location
}
```

编译成功后会生成 `wl_drive_cpp.exe`。开过世界的情况下，回到 Webots 重载世界。

</details>

## 键盘操作

| 模式 | 进入方式 | 移动 | 停止与退出 |
| --- | --- | --- | --- |
| 轮式 | 站姿就绪后默认进入 | `W` / `S` 前进和后退，`A` / `D` 左转和右转 | 松开方向键停车，车轮停稳后按 `H` 保持轮角 |
| 低速爬行 | 停稳后按 `H`，再按 `G` | 点按或长按 `W` / `A` / `S` / `D` | 空格请求停步。按 `R` 请求停止，站稳后再按一次 `R` 退出 |
| 对角小跑 | 停稳后按 `H`，再按 `T` | 长按 `W` 直行，`A` / `D` 原地转向，`W+A` / `W+D` 边走边转 | 松开方向键或按空格停步，按 `R` 回中后退出 |

按空格停步后，先松开方向键再起步。进入步态前，控制台应显示 `Wheel hold ENABLED`。

## 文件结构

```text
WL_P311D-Webots/
├── worlds/
│   └── first_demo_cpp.wbt       # 平地演示，固定 32 ms 仿真步长
├── protos/
│   ├── WL_P311D.proto           # 机器人、关节、电机与传感器
│   └── meshes/                  # PROTO 引用的 17 个 STL
├── controllers/wl_drive_cpp/
│   ├── wl_drive_cpp.cpp         # 入口、主循环和模式管理
│   ├── robot_io.cpp             # 读取设备并下发执行目标
│   ├── rolling.cpp              # 键盘解码、左右轮速和锁轮
│   ├── gait.cpp                 # 爬行步态阶段与调度
│   ├── crawl_controller.cpp     # 低速爬行控制
│   ├── trot_controller.cpp      # 对角小跑、摆腿、落地和回中
│   ├── wheel_ground.cpp         # 两种步态共用的平地接触和轮缘间隙
│   ├── kinematics.cpp           # 单腿正、逆运动学
│   ├── body_control.cpp         # 站姿和支撑平移
│   ├── estimation.cpp          # 模型质心和接地几何
│   ├── recorder.cpp             # 实测反馈与应用指令日志
│   ├── Makefile
│   └── LEARNING.md              # 源代码阅读顺序
├── licenses/
│   └── LimX-Apache-2.0.txt
├── THIRD_PARTY_NOTICES.md
└── README.md
```

主循环按仿真步长读取反馈。模式管理将键盘指令交给轮式控制或步态控制。步态将轮心位置转换为腿关节目标，再与轮角保持目标一起交给设备层。设备层接收的目标统一由 `RobotIO::applyTargets()` 下发。接地判断使用 Webots Supervisor 查询的碰撞接触点与轮缘几何估计。

按实际调用关系阅读代码，可从 [控制器阅读指南](controllers/wl_drive_cpp/LEARNING.md) 开始。

## 仿真验证

发布场景使用 Webots 内建地面与照明，关闭了电机和接触音效。Webots 仍可能提示原始网格面数过高，部分部件因此无法投射阴影。

Windows 上的 Webots R2025a 已完成 85 秒脚本测试，记录 2657 帧有效反馈。控制器运行时，会把反馈与电机指令写到 `controllers/wl_drive_cpp/logs/`。

要复现测试，在 `worlds/` 下另存一份测试世界。在场景树选中 `WL_P311D`，在 `controllerArgs` 中依次添加以下三个字符串，保存后重载运行。

```text
--validation-seconds=85
--scripted-trot
--quit-webots
```

脚本依次执行锁轮、直行小跑、左右原地转向、边走边转、停步回中及退出后的轮式前进。结束时写出 `validation_completed.json`。第三个参数会在测试结束时关闭 Webots，手动演示时保持 `controllerArgs` 为空。

对角小跑脚本的 `completed` 表示计时结束，`passed` 表示动作验收结果。验收检查脚本是否执行到最后、是否出现无效帧或步态故障、是否完成换步、前进与左右转向是否符合指令，以及退出后是否实测到轮式前移。JSON 中的位移单位为米，偏航变化单位为弧度。发生步态故障时可查看 `first_fault`。配合 `--quit-webots` 自动运行时，通过返回退出码 0，失败返回 1。

发布世界通过 `DEF WALKING_FLOOR` 标记具有水平 Box 碰撞体的地面。整体抬高平台时，同时上移地面和机器人的初始位置。两种步态会在进入模式时读取地面顶面高度。旧世界缺少该标记时，沿用零高度平地。

## 许可

机器人模型来源于 LimX Dynamics 的 TRON1 robot description。模型适用 Apache License 2.0，原始许可文本和适配说明随仓库提供。项目自行编写的 C++ 控制代码尚未声明开源许可证。[第三方资产说明](THIRD_PARTY_NOTICES.md)
