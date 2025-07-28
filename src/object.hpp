#pragma once

#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

namespace object
{

    enum class ObjectType
    {
        UNKNOW = -1,
        DETECTION = 0,
        POSE = 1,
        OBB = 2,
        SEGMENTATION = 3,
        DEPTH_ANYTHING = 4,
        DEPTH_PRO = 5,
        TRACK = 6,
    };

    struct Box
    {
        float left = 0.0f;
        float top = 0.0f;
        float right = 0.0f;
        float bottom = 0.0f;

        Box() = default;
        Box(float l, float t, float r, float b)
            : left(l), top(t), right(r), bottom(b) {}
    };

    struct PosePoint
    {
        float x = 0.0f;
        float y = 0.0f;
        float vis = 0.0f;

        PosePoint() = default;
        PosePoint(float x, float y, float vis) : x(x), y(y), vis(vis) {}
    };

    struct Obb
    {
        float cx = 0.0f;
        float cy = 0.0f;
        float w = 0.0f;
        float h = 0.0f;
        float angle = 0.0f;

        Obb() = default;
        Obb(float cx, float cy, float w, float h, float angle)
            : cx(cx), cy(cy), w(w), h(h), angle(angle) {}
    };

    struct Segmentation
    {
        cv::Mat mask;
        // 默认构造函数
        Segmentation() = default;

        // 拷贝构造函数：强制深拷贝 cv::Mat
        Segmentation(const Segmentation &other) : mask(other.mask.clone()) {}

        // 拷贝赋值运算符：强制深拷贝 cv::Mat
        Segmentation &operator=(const Segmentation &other)
        {
            if (this != &other)
            {
                mask = other.mask.clone();
            }
            return *this;
        }

        // C++11/14/17 最好也添加移动语义，或者使用 = default
        Segmentation(Segmentation &&other) = default;
        Segmentation &operator=(Segmentation &&other) = default;
    };

    struct Depth
    {
        cv::Mat depth;
        float fog_data;
        // 默认构造函数
        Depth() = default;

        // 拷贝构造函数：强制深拷贝 cv::Mat
        Depth(const Depth &other) : depth(other.depth.clone()), fog_data(other.fog_data) {}

        // 拷贝赋值运算符：强制深拷贝 cv::Mat
        Depth &operator=(const Depth &other)
        {
            if (this != &other)
            {
                depth = other.depth.clone();
                fog_data = other.fog_data;
            }
            return *this;
        }
        Depth(Depth &&other) = default;
        Depth &operator=(Depth &&other) = default;
    };

    struct DetectionBox
    {
        ObjectType type = ObjectType::UNKNOW;
        // Bounding box coordinates
        Box box;
        // Pose points for keypoints detection
        std::vector<PosePoint> pose_points;
        // Oriented bounding box for OBB detection
        Obb obb;
        // Segmentation mask for segmentation detection
        Segmentation segmentation;
        Depth depth;

        float score = 0.0f;
        int class_id = -1;
        int track_id = -1;
        std::string class_name;
    };

    using DetectionBoxArray = std::vector<DetectionBox>;

}

#endif // endif OBJECT_HPP