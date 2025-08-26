#pragma once

#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <string>
#include <vector>
#include <ostream> // 包含 ostream 用于友元函数声明
#include "opencv2/opencv.hpp"

namespace object
{

    enum class ObjectType
    {
        UNKNOW = -1,
        POSITION = 0,
        POSE = 1,
        OBB = 2,
        SEGMENTATION = 3,
        DEPTH_ANYTHING = 4,
        DEPTH_PRO = 5,
        TRACK = 6,
        DETECTION = 7,
    };

    struct Box
    {
        float left = 0.0f;
        float top = 0.0f;
        float right = 0.0f;
        float bottom = 0.0f;

        Box() = default;
        Box(float l, float t, float r, float b);

        // 简单函数，适合内联
        float width() const noexcept { return right - left; }
        float height() const noexcept { return bottom - top; }
        float center_x() const noexcept { return (left + right) / 2; }
        float center_y() const noexcept { return (top + bottom) / 2; }
        float area() const noexcept { return width() * height(); }

        // 声明友元函数
        friend std::ostream &operator<<(std::ostream &os, const Box &box);
    };

    struct PosePoint
    {
        float x = 0.0f;
        float y = 0.0f;
        float vis = 0.0f;

        PosePoint() = default;
        PosePoint(float x, float y, float vis);

        // 声明友元函数
        friend std::ostream &operator<<(std::ostream &os, const PosePoint &point);
    };

    struct Obb
    {
        float cx = 0.0f;
        float cy = 0.0f;
        float w = 0.0f;
        float h = 0.0f;
        float angle = 0.0f;

        Obb() = default;
        Obb(float cx, float cy, float w, float h, float angle);

        // 简单函数，适合内联
        float area() const { return w * h; }

        // 声明友元函数
        friend std::ostream &operator<<(std::ostream &os, const Obb &obb);
    };

    struct Segmentation
    {
        cv::Mat mask;

        Segmentation() = default;
        Segmentation(const Segmentation &other);            // 声明拷贝构造
        Segmentation &operator=(const Segmentation &other); // 声明拷贝赋值
        Segmentation(Segmentation &&other) = default;
        Segmentation &operator=(Segmentation &&other) = default;
    };

    struct Depth
    {
        cv::Mat depth;
        float fog_data = 0.0f;

        Depth() = default;
        Depth(const Depth &other);            // 声明拷贝构造
        Depth &operator=(const Depth &other); // 声明拷贝赋值
        Depth(Depth &&other) = default;
        Depth &operator=(Depth &&other) = default;

        // 声明成员函数
        float point_depth(int x, int y) const;
        float average_depth() const;
        float min_depth() const;
        float max_depth() const;
        float area_average_depth(const cv::Mat &seg) const;
        float area_average_depth(const Box &box) const;
    };

    struct DetectionBox
    {
        ObjectType type = ObjectType::UNKNOW;
        Box box;
        std::vector<PosePoint> pose_points;
        Obb obb;
        Segmentation segmentation;
        Depth depth;

        float score = 0.0f;
        int class_id = -1;
        int track_id = -1;
        std::string class_name;

        // 声明友元函数
        friend std::ostream &operator<<(std::ostream &os, const DetectionBox &box);
    };

    using DetectionBoxArray = std::vector<DetectionBox>;

} // namespace object

#endif // OBJECT_HPP