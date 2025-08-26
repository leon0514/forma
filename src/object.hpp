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
        Box(float l, float t, float r, float b)
            : left(l), top(t), right(r), bottom(b) {}

        // 获取矩形的宽度和高度
        float width() const { return right - left; }
        float height() const { return bottom - top; }
        float center_x() const { return (left + right) / 2; }
        float center_y() const { return (top + bottom) / 2; }
        // 获取矩形的面积
        float area() const { return width() * height(); }

        // 重载cout
        friend std::ostream &operator<<(std::ostream &os, const Box &box)
        {
            os << "{ \"left\": " << box.left
               << ", \"top\": " << box.top
               << ", \"right\": " << box.right
               << ", \"bottom\": " << box.bottom
               << " }";
            return os;
        }
    };

    struct PosePoint
    {
        float x = 0.0f;
        float y = 0.0f;
        float vis = 0.0f;

        PosePoint() = default;
        PosePoint(float x, float y, float vis) : x(x), y(y), vis(vis) {}

        friend std::ostream &operator<<(std::ostream &os, const PosePoint &point)
        {
            os << "{ \"x\": " << point.x
               << ", \"y\": " << point.y
               << ", \"vis\": " << point.vis
               << " }";
            return os;
        }
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

        float area() const { return w * h; }

        friend std::ostream &operator<<(std::ostream &os, const Obb &obb)
        {
            os << "{ \"cx\": " << obb.cx
               << ", \"cy\": " << obb.cy
               << ", \"w\": " << obb.w
               << ", \"h\": " << obb.h
               << ", \"angle\": " << obb.angle
               << " }";
            return os;
        }
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
        float point_depth(int x, int y) const { return depth.at<float>(y, x); }
        float average_depth() const
        {
            if (depth.empty())
                return 0.0f;
            cv::Scalar mean_val = cv::mean(depth);
            return static_cast<float>(mean_val[0]);
        }
        float min_depth() const
        {
            if (depth.empty())
                return 0.0f;
            double min_val;
            cv::minMaxLoc(depth, &min_val, nullptr);
            return static_cast<float>(min_val);
        }
        float max_depth() const
        {
            if (depth.empty())
                return 0.0f;
            double max_val;
            cv::minMaxLoc(depth, nullptr, &max_val, nullptr, nullptr);
            return static_cast<float>(max_val);
        }

        float area_average_depth(const cv::Mat& seg) const
        {
            if (depth.empty() || seg.empty())
                return 0.0f;

            cv::Mat masked_depth;
            depth.copyTo(masked_depth, seg);

            float sum_depth = cv::sum(masked_depth)[0];
            float area = cv::countNonZero(seg);
            return area > 0 ? sum_depth / area : 0.0f;
        }

        float area_average_depth(const Box& box) const
        {
            if (depth.empty())
                return 0.0f;

            int left = std::max(0, static_cast<int>(box.left));
            int top = std::max(0, static_cast<int>(box.top));
            int right = std::min(depth.cols - 1, static_cast<int>(box.right));
            int bottom = std::min(depth.rows - 1, static_cast<int>(box.bottom));

            if (left >= right || top >= bottom)
                return 0.0f;

            cv::Rect roi(left, top, right - left, bottom - top);
            cv::Mat depth_roi = depth(roi);

            float sum_depth = cv::sum(depth_roi)[0];
            float area = depth_roi.total();
            return area > 0 ? sum_depth / area : 0.0f;
        }

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

        friend std::ostream &operator<<(std::ostream &os, const DetectionBox &box)
        {
            // 根据 type 输出 JSON 格式
            if (box.type == ObjectType::DETECTION)
            {
                os << "{ \"type\": \"DETECTION\", "
                   << "\"box\": " << box.box
                   << ", \"score\": " << box.score
                   << ", \"class_id\": " << box.class_id
                   << ", \"class_name\": \"" << box.class_name << "\" }";
            }
            else if (box.type == ObjectType::POSE)
            {
                os << "{ \"type\": \"POSE\", "
                   << "\"pose_points\": [";
                for (size_t i = 0; i < box.pose_points.size(); ++i)
                {
                    os << box.pose_points[i];
                    if (i < box.pose_points.size() - 1)
                        os << ", ";
                }
                os << "], \"score\": " << box.score
                   << ", \"class_id\": " << box.class_id
                   << ", \"class_name\": \"" << box.class_name << "\" }";
            }
            else if (box.type == ObjectType::OBB)
            {
                os << "{ \"type\": \"OBB\", "
                   << "\"obb\": " << box.obb
                   << ", \"score\": " << box.score
                   << ", \"class_id\": " << box.class_id
                   << ", \"class_name\": \"" << box.class_name << "\" }";
            }
            else if (box.type == ObjectType::SEGMENTATION)
            {
                os << "{ \"type\": \"SEGMENTATION\", "
                   << "\"box\": " << box.box
                   << ", \"score\": " << box.score
                   << ", \"class_id\": " << box.class_id
                   << ", \"class_name\": \"" << box.class_name << "\" }";
            }
            else if (box.type == ObjectType::DEPTH_ANYTHING || box.type == ObjectType::DEPTH_PRO)
            {
                os << "{ \"type\": \""
                   << (box.type == ObjectType::DEPTH_ANYTHING ? "DEPTH_ANYTHING" : "DEPTH_PRO")
                   << "\" }";
            }
            else if (box.type == ObjectType::TRACK)
            {
                os << "{ \"type\": \"TRACK\", "
                   << "\"box\": " << box.box
                   << ", \"track_id\": " << box.track_id
                   << ", \"score\": " << box.score
                   << ", \"class_id\": " << box.class_id
                   << ", \"class_name\": \"" << box.class_name << "\" }";
            }
            else
            {
                os << "{ \"type\": \"UNKNOW\" }";
            }
            return os;
        }
    };

    using DetectionBoxArray = std::vector<DetectionBox>;

}

#endif // endif OBJECT_HPP