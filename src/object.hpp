#pragma once

#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <string>
#include <vector>
#include "opencv2/opencv.hpp" 


namespace object{

    struct Box
    {
        float left = 0.0f;
        float top = 0.0f;
        float right = 0.0f;
        float bottom = 0.0f;
    
        Box() = default;
        Box(float l, float t, float r, float b)
            : left(l), top(t), right(r), bottom(b){}
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
        Segmentation(const Segmentation& other) : mask(other.mask.clone()) {}
    
        // 拷贝赋值运算符：强制深拷贝 cv::Mat
        Segmentation& operator=(const Segmentation& other) {
            if (this != &other) {
                mask = other.mask.clone();
            }
            return *this;
        }
    
        // C++11/14/17 最好也添加移动语义，或者使用 = default
        Segmentation(Segmentation&& other) = default;
        Segmentation& operator=(Segmentation&& other) = default;
    };
    
    
    struct DetectionBox
    {
        // Bounding box coordinates
        Box box;
        // Pose points for keypoints detection
        std::vector<PosePoint> pose_points;
        // Oriented bounding box for OBB detection
        Obb obb;
        // Segmentation mask for segmentation detection
        Segmentation segmentation;
    
        float score = 0.0f;
        int class_id = -1;
        int track_id = -1;
        std::string class_name;
    };

    using DetectionBoxArray = std::vector<DetectionBox>;

}

#endif // endif OBJECT_HPP