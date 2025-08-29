#include "object.hpp" // 请确保这里的路径与您的项目结构匹配
#include <iostream>
#include <string>
#include <vector>

namespace object
{
    //================================================================================
    // 辅助函数：将 ObjectType 枚举转换为字符串，用于生成清晰的 JSON 输出
    //================================================================================
    std::string ObjectTypeToString(ObjectType type) {
        switch (type) {
            case ObjectType::DETECTION:      return "DETECTION";
            case ObjectType::POSE:           return "POSE";
            case ObjectType::OBB:            return "OBB";
            case ObjectType::SEGMENTATION:   return "SEGMENTATION";
            case ObjectType::TRACK:          return "TRACK";
            case ObjectType::DEPTH_ANYTHING: return "DEPTH_ANYTHING";
            case ObjectType::DEPTH_PRO:      return "DEPTH_PRO";
            case ObjectType::POSITION:       return "POSITION";
            case ObjectType::UNKNOW:
            default:                         return "UNKNOW";
        }
    }

    //================================================================================
    // 结构体实现 (构造函数和简单的操作符重载)
    // 这部分代码设计良好，基本保持不变。
    //================================================================================

    // Box 的构造函数实现
    Box::Box(float l, float t, float r, float b)
        : left(l), top(t), right(r), bottom(b) {}

    // Box 的输出流操作符重载，用于打印
    std::ostream &operator<<(std::ostream &os, const Box &box) {
        os << "{ \"left\": " << box.left
           << ", \"top\": " << box.top
           << ", \"right\": " << box.right
           << ", \"bottom\": " << box.bottom
           << " }";
        return os;
    }

    // PosePoint 的构造函数实现
    PosePoint::PosePoint(float x, float y, float vis) : x(x), y(y), vis(vis) {}

    // PosePoint 的输出流操作符重载
    std::ostream &operator<<(std::ostream &os, const PosePoint &point) {
        os << "{ \"x\": " << point.x
           << ", \"y\": " << point.y
           << ", \"vis\": " << point.vis
           << " }";
        return os;
    }
    
    // Pose 的输出流操作符重载
    std::ostream &operator<<(std::ostream &os, const Pose &pose) {
        os << "[";
        for (size_t i = 0; i < pose.points.size(); ++i) {
            os << pose.points[i];
            if (i < pose.points.size() - 1) os << ", ";
        }
        os << "]";
        return os;
    }

    // Obb 的构造函数实现
    Obb::Obb(float cx, float cy, float w, float h, float angle)
        : cx(cx), cy(cy), w(w), h(h), angle(angle) {}

    // Obb 的输出流操作符重载
    std::ostream &operator<<(std::ostream &os, const Obb &obb) {
        os << "{ \"cx\": " << obb.cx
           << ", \"cy\": " << obb.cy
           << ", \"w\": " << obb.w
           << ", \"h\": " << obb.h
           << ", \"angle\": " << obb.angle
           << " }";
        return os;
    }

    // Track 的输出流操作符重载
    std::ostream &operator<<(std::ostream &os, const Track &track) {
        os << "{ \"track_id\": " << track.track_id
           << ", \"trace\": [";
        for (size_t i = 0; i < track.track_trace.size(); ++i) {
            // std::get<index> 是访问 tuple 元素的正确方式
            os << "{ \"x\": " << std::get<0>(track.track_trace[i]) 
               << ", \"y\": " << std::get<1>(track.track_trace[i]) << " }";
            if (i < track.track_trace.size() - 1) os << ", ";
        }
        os << "] }";
        return os;
    }

    //================================================================================
    // 已移除: Segmentation 和 Depth 的拷贝操作实现。
    // 因为在 .hpp 文件中我们遵循了“零之法则”，移除了它们的声明，
    // 所以这里的定义也必须移除。编译器自动生成的版本已经足够高效和正确。
    //================================================================================
    // Segmentation::Segmentation(const Segmentation &other) ... // 已删除
    // Segmentation &Segmentation::operator=(const Segmentation &other) ... // 已删除
    // Depth::Depth(const Depth &other) ... // 已删除
    // Depth &Depth::operator=(const Depth &other) ... // 已删除


    //================================================================================
    // Depth 成员函数的实现
    // 这部分实现得很好，予以保留。
    //================================================================================

    float Depth::point_depth(int x, int y) const {
        if (depth.empty() || y < 0 || y >= depth.rows || x < 0 || x >= depth.cols) {
            return 0.0f; // 边界检查，防止访问越界
        }
        // 假设深度图是 CV_32F (32位浮点型)
        return depth.at<float>(y, x);
    }

    float Depth::average_depth() const {
        if (depth.empty()) return 0.0f;
        // cv::mean 返回一个 Scalar，我们通常取第一个通道的均值
        return static_cast<float>(cv::mean(depth)[0]);
    }

    float Depth::min_depth() const {
        if (depth.empty()) return 0.0f;
        double min_val;
        cv::minMaxLoc(depth, &min_val, nullptr, nullptr, nullptr);
        return static_cast<float>(min_val);
    }

    float Depth::max_depth() const {
        if (depth.empty()) return 0.0f;
        double max_val;
        cv::minMaxLoc(depth, nullptr, &max_val, nullptr, nullptr);
        return static_cast<float>(max_val);
    }

    float Depth::area_average_depth(const cv::Mat &seg) const {
        if (depth.empty() || seg.empty()) return 0.0f;
        // 使用掩码计算特定区域的平均深度
        cv::Mat masked_depth;
        depth.copyTo(masked_depth, seg); // 只拷贝掩码为非零的区域
        
        float sum_depth = cv::sum(masked_depth)[0];
        int area = cv::countNonZero(seg);
        return area > 0 ? sum_depth / area : 0.0f;
    }

    float Depth::area_average_depth(const Box &box) const {
        if (depth.empty()) return 0.0f;
        // 定义感兴趣区域 (Region of Interest, ROI)
        cv::Rect roi(cv::Point(box.left, box.top), cv::Point(box.right, box.bottom));
        // 将 ROI 与图像边界取交集，确保 ROI 不会超出图像范围
        roi &= cv::Rect(0, 0, depth.cols, depth.rows);
        if (roi.area() == 0) return 0.0f;
        // 直接计算 ROI 区域的均值
        return static_cast<float>(cv::mean(depth(roi))[0]);
    }

    //================================================================================
    // 已修改: DetectionBox 的输出流操作符重载
    // 这是最重要的修改。它不再使用僵化的 switch 结构，
    // 而是先打印所有必需字段，然后逐一检查每个 optional 成员是否存在值。
    // 这种方式更灵活，能准确地反映出任意组合的数据。
    //================================================================================
    std::ostream &operator<<(std::ostream &os, const DetectionBox &box) {
        os << "{";
        
        // --- 打印核心/必需字段 ---
        os << " \"type\": \"" << ObjectTypeToString(box.type) << "\""
           << ", \"class_id\": " << box.class_id
           << ", \"class_name\": \"" << box.class_name << "\""
           << ", \"score\": " << box.score
           << ", \"box\": " << box.box;

        // --- 逐一检查并打印可选字段 ---
        if (box.pose.has_value()) {
            os << ", \"pose\": " << box.pose.value();
        }
        if (box.obb.has_value()) {
            os << ", \"obb\": " << box.obb.value();
        }
        if (box.track.has_value()) {
            os << ", \"track\": " << box.track.value();
        }
        if (box.segmentation.has_value()) {
            // 直接打印整个掩码矩阵不现实，所以我们只打印它的尺寸信息
            const auto& mask = box.segmentation.value().mask;
            os << ", \"segmentation\": { \"width\": " << mask.cols 
               << ", \"height\": " << mask.rows << " }";
        }
        if (box.depth.has_value()) {
            // 同理，只打印深度图的尺寸信息
            const auto& depth = box.depth.value().depth;
            os << ", \"depth\": { \"width\": " << depth.cols
               << ", \"height\": " << depth.rows << " }";
        }

        os << " }";
        return os;
    }

} // namespace object