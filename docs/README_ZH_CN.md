Language: [English](README.md) | **简体中文**

# Lua微型2D物理引擎 v0.1

<img src="docs/Icon.png" width="120"/>

> [!WARNING] 
> 注：该库还未完成。

## 简介

这是一个使用纯Lua编写的简易2D物理引擎，为受限的开发环境提供简单的2D物理模拟系统。

- 仅包含单一文件，无外部依赖，你可以轻松地将该库嵌入到现有项目中。
- 可在大多数受限/沙箱环境中运行。
- 基于LuaJIT编写，但理论上兼容其他版本
  - Lua 5.1
  - Lua 5.2
  - Lua 5.3
  - Lua 5.4
  - Luau

> [!WARNING] 
> 如果你需要模拟上百个多边形物体，请使用LuaJIT。标准Lua解释器在计算密集的场景很容易出现性能瓶颈，开启JIT能提升性能3倍。

## 代办

### v1.0

- [ ] feature: Support polygon shape & predef polygon shape.
- [ ] feature: Add world serializer.

### 未来 (如果可能)

- [ ] perf: Dynamic-AABB-Tree for static type bodies.
- [ ] perf: Spatial-Hash-Map for dynamic type bodies.
- [ ] stability: Sleep dynamic type bodies.
- [ ] feature: Add more constraints: Joints.
