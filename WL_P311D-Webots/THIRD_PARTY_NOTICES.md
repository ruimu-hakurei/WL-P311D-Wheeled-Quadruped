# 第三方资产

## LimX Dynamics WL_P311D 模型

- `protos/meshes/` 包含机器人 PROTO 使用的 17 个原始 STL，保留了上游文件名和原始内容。
- `protos/WL_P311D.proto` 由上游 `wheellegged/WL_P311D/urdf/robot.urdf` 转换并适配。适配包括 Webots 传感器及执行器配置、轮子接触材质、相对网格路径与关闭远程电机音效。
- 上游许可证为 Apache License 2.0。[随仓库分发的完整许可文本](licenses/LimX-Apache-2.0.txt)

上游文件和许可来源见 [WL_P311D 模型目录](https://github.com/limxdynamics/tron1-robot-description/tree/master/wheellegged/WL_P311D) 和 [上游 LICENSE](https://github.com/limxdynamics/tron1-robot-description/blob/master/LICENSE)。

## Webots 与转换工具

场景格式、仿真和 C++ 控制器 API 来自 Cyberbotics Webots。模型由 urdf2webots 转换。运行需要本地安装 Webots，构建时使用其自带的控制器库和 Makefile。[Webots](https://github.com/cyberbotics/webots) · [urdf2webots](https://github.com/cyberbotics/urdf2webots)
