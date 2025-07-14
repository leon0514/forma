# Forma: 2D 几何计算库

## 描述

Forma 是一个为计算机视觉应用设计的 C++ 库，专注于处理二维几何形状的计算。它提供了一系列工具，用于操作和分析边界框（bounding boxes）、分割掩码（segmentation masks）以及电子围栏（geofences）。

该库利用了 OpenCV 来处理图像相关的任务，并借助 Boost.Geometry 来执行复杂的几何运算，为目标检测、实例分割和空间分析等任务提供了坚实的基础。

## 主要功能

- **边界框运算**:
  - 计算面积
  - 计算交并比 (IoU)
  - 计算交集面积

- **分割掩码运算**:
  - 计算面积
  - 计算交并比 (IoU)
  - 计算交集面积
  - 计算交集与最小面积的比例

- **混合运算**:
  - 计算边界框与分割掩码的 IoU
  - 计算边界框与分割掩码的交集与最小面积的比例

- **电子围栏**:
  - 判断点、边界框或掩码是否在围栏内部
  - 计算边界框或掩码与围栏的 IoU
  - 计算边界框或掩码与围栏的交集面积

## 依赖

- C++14 或更高版本
- OpenCV
- Boost
```
sudo apt install libboost-all-dev
```

## 构建

本项目使用 `Makefile` 进行管理。要编译库和示例，请在项目根目录下运行：

```bash
make
```

可执行文件和库文件将生成在 `workspace` 目录中。

## 使用示例

以下是一个如何使用 Forma 库计算边界框面积和 IoU 的简单示例：

```cpp
#include "forma/forma.hpp"
#include "object.hpp"
#include <iostream>

int main()
{
    // 定义一个 100x100 的边界框
    object::Box box1(0, 0, 100, 100);
    object::Box box2(50, 50, 150, 150);

    // 计算面积
    float area = forma::box_area(box1);
    std::cout << "Box Area: " << area << std::endl;

    // 计算两个边界框的 IoU
    float iou = forma::box_iou(box1, box2);
    std::cout << "Box IoU: " << iou << std::endl;

    return 0;
}
```

## API 参考

### `forma` 命名空间

#### 边界框函数

- `float box_area(const object::Box& box)`
  - **功能**: 计算矩形框的面积。
  - **说明**: 面积计算公式为 `(right - left) * (bottom - top)`。

- `float box_iou(const object::Box& box1, const object::Box& box2)`
  - **功能**: 计算两个矩形框的交并比 (Intersection over Union, IoU)。
  - **说明**: IoU 是两个矩形框交集面积与并集面积的比值，是目标检测中常用的评估指标。

- `float intersection_box_area(const object::Box& box1, const object::Box& box2)`
  - **功能**: 计算两个矩形框的交集面积。

- `bool point_in_box(const object::PosePoint& pose_point, const object::Box& box)`
  - **功能**: 判断一个点是否在矩形框内。
  - **说明**: 通过检查点的坐标是否在矩形框的边界内来确定。

#### 分割掩码函数

- `float mask_area(const object::Segmentation& segmentation)`
  - **功能**: 计算分割掩码的面积。
  - **说明**: 面积等于掩码中非零像素点的数量。

- `float mask_iou(const object::Segmentation& segmentation1, const object::Segmentation& segmentation2)`
  - **功能**: 计算两个分割掩码的交并比 (IoU)。
  - **说明**: IoU 是两个掩码交集面积与并集面积的比值。

- `float intersection_mask_area(const object::Segmentation& segmentation1, const object::Segmentation& segmentation2)`
  - **功能**: 计算两个分割掩码的交集面积。

- `bool point_in_mask(const object::PosePoint& pose_point, const object::Segmentation& segmentation)`
  - **功能**: 判断一个点是否在分割掩码内。
  - **说明**: 通过检查点的坐标对应在掩码中的值是否非零来确定。

- `float intersection_over_min_mask_ratio(const object::Segmentation& segmentation1, const object::Segmentation& segmentation2)`
  - **功能**: 计算两个掩码的交集面积与两者中较小面积的比值。

#### 混合运算函数

- `float box_mask_iou(const object::Box& box, const object::Segmentation& segmentation)`
  - **功能**: 计算一个矩形框和一个分割掩码之间的交并比 (IoU)。

- `float intersection_over_min_box_mask_ratio(const object::Box& box, const object::Segmentation& segmentation)`
  - **功能**: 计算矩形框与掩码的交集面积与两者中较小面积的比值。

#### 电子围栏函数

- `bool point_in_fence(const object::PosePoint& pose_point, const std::vector<std::tuple<float, float>>& fence)`
  - **功能**: 判断一个点是否在由一系列顶点定义的多边形电子围栏内部。

- `bool box_in_fence(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)`
  - **功能**: 判断一个矩形框是否完全位于电子围栏内部。
  - **说明**: 通过检查矩形框的四个顶点是否都在围栏内来确定。

- `bool mask_in_fence(const object::Segmentation& segmentation, const std::vector<std::tuple<float, float>>& fence)`
  - **功能**: 判断一个分割掩码是否完全位于电子围栏内部。
  - **说明**: 通过检查掩码的所有非零像素点是否都在围栏内来确定。

- `float box_fence_iou(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)`
  - **功能**: 计算一个矩形框与电子围栏的交并比 (IoU)。

- `float mask_fence_iou(const object::Segmentation& segmentation, const std::vector<std::tuple<float, float>>& fence)`
  - **功能**: 计算一个分割掩码与电子围栏的交并比 (IoU)。

- `float intersection_box_fence_area(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)`
  - **功能**: 计算一个矩形框与电子围栏的交集面积。

- `float intersection_mask_fence_area(const object::Segmentation& segmentation, const std::vector<std::tuple<float, float>>& fence)`
  - **功能**: 计算一个分割掩码与电子围栏的交集面积。

- `float intersection_over_min_box_fence_ratio(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)`
  - **功能**: 计算矩形框与围栏的交集面积与两者中较小面积的比值。

- `float intersection_over_min_mask_fence_ratio(const object::Segmentation& segmentation, const std::vector<std::tuple<float, float>>& fence)`
  - **功能**: 计算分割掩码与围栏的交集面积与两者中较小面积的比值。
