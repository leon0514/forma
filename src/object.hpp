#pragma once

#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <string>
#include <vector>
#include <tuple>     // 保留 tuple
#include <ostream>
#include <optional>  // C++17, 用于优化内存
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

        float width() const noexcept { return right - left; }
        float height() const noexcept { return bottom - top; }
        float center_x() const noexcept { return (left + right) / 2; }
        float center_y() const noexcept { return (top + bottom) / 2; }
        float area() const noexcept { return width() * height(); }

        friend std::ostream &operator<<(std::ostream &os, const Box &box);
    };

    struct PosePoint
    {
        float x = 0.0f;
        float y = 0.0f;
        float vis = 0.0f;

        PosePoint() = default;
        PosePoint(float x, float y, float vis);

        friend std::ostream &operator<<(std::ostream &os, const PosePoint &point);
    };

    struct Pose
    {
        std::vector<PosePoint> points;
        friend std::ostream &operator<<(std::ostream &os, const Pose &pose);
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

        float area() const { return w * h; }

        friend std::ostream &operator<<(std::ostream &os, const Obb &obb);
    };

    // 改进：遵循“零之法则”，移除不必要的特殊成员函数声明。
    // cv::Mat 已经能很好地管理自己的资源。
    struct Segmentation
    {
        cv::Mat mask;
    };

    // 改进：同样遵循“零之法则”。
    struct Depth
    {
        cv::Mat depth;
        float fog_data = 0.0f;

        // 成员函数声明
        float point_depth(int x, int y) const;
        float average_depth() const;
        float min_depth() const;
        float max_depth() const;
        float area_average_depth(const cv::Mat &seg) const;
        float area_average_depth(const Box &box) const;
    };

    // 保持不变，根据您的要求使用 std::tuple
    struct Track
    {
        int track_id = -1;
        std::vector<std::tuple<float, float>> track_trace;
        friend std::ostream &operator<<(std::ostream &os, const Track &track);
    };

    // 核心改进：使用 std::optional 包装可选成员，极大地节省内存
    // 并明确地表达了哪些数据是可能不存在的。
    struct DetectionBox
    {
        // --- 核心/必须存在的数据 ---
        ObjectType type = ObjectType::UNKNOW;
        Box box;
        float score = 0.0f;
        int class_id = -1;
        std::string class_name;

        // --- 可选的数据 ---
        std::optional<Pose> pose;
        std::optional<Obb> obb;
        std::optional<Segmentation> segmentation;
        std::optional<Depth> depth;
        std::optional<Track> track;

        // 友元函数声明
        friend std::ostream &operator<<(std::ostream &os, const DetectionBox &box);
    };

    using DetectionBoxArray = std::vector<DetectionBox>;

} // namespace object

#endif // OBJECT_HPP