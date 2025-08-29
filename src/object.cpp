#include "object.hpp"
#include <iostream>

namespace object
{
    Box::Box(float l, float t, float r, float b)
        : left(l), top(t), right(r), bottom(b) {}

    std::ostream &operator<<(std::ostream &os, const Box &box)
    {
        os << "{ \"left\": " << box.left
           << ", \"top\": " << box.top
           << ", \"right\": " << box.right
           << ", \"bottom\": " << box.bottom
           << " }";
        return os;
    }

    PosePoint::PosePoint(float x, float y, float vis) : x(x), y(y), vis(vis) {}

    std::ostream &operator<<(std::ostream &os, const PosePoint &point)
    {
        os << "{ \"x\": " << point.x
           << ", \"y\": " << point.y
           << ", \"vis\": " << point.vis
           << " }";
        return os;
    }

    Obb::Obb(float cx, float cy, float w, float h, float angle)
        : cx(cx), cy(cy), w(w), h(h), angle(angle) {}

    std::ostream &operator<<(std::ostream &os, const Obb &obb)
    {
        os << "{ \"cx\": " << obb.cx
           << ", \"cy\": " << obb.cy
           << ", \"w\": " << obb.w
           << ", \"h\": " << obb.h
           << ", \"angle\": " << obb.angle
           << " }";
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const Track &track)
    {
        os << "{ \"track_id\": " << track.track_id
           << ", \"track_trace\": [";
        for (size_t i = 0; i < track.track_trace.size(); ++i)
        {
            float x, y;
            std::tie(x, y) = track.track_trace[i];
            os << "{ \"x\": " << x << ", \"y\": " << y << " }";
            if (i < track.track_trace.size() - 1)
            {
                os << ", ";
            }
        }
        os << "] }";
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const Pose &pose)
    {
        os << "{ \"Pose points\": [";
        for (size_t i = 0; i < pose.points.size(); ++i)
        {
            os << pose.points[i];
            if (i < pose.points.size() - 1)
            {
                os << ", ";
            }
        }
        os << "] }";
        return os;
    }

    Segmentation::Segmentation(const Segmentation &other) : mask(other.mask.clone()) {}

    Segmentation &Segmentation::operator=(const Segmentation &other)
    {
        if (this != &other)
        {
            mask = other.mask.clone();
        }
        return *this;
    }

    Depth::Depth(const Depth &other) : depth(other.depth.clone()), fog_data(other.fog_data) {}

    Depth &Depth::operator=(const Depth &other)
    {
        if (this != &other)
        {
            depth = other.depth.clone();
            fog_data = other.fog_data;
        }
        return *this;
    }

    float Depth::point_depth(int x, int y) const
    {
        if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
            return 0.0f;
        return depth.at<float>(y, x);
    }

    float Depth::average_depth() const
    {
        if (depth.empty())
            return 0.0f;
        cv::Scalar mean_val = cv::mean(depth);
        return static_cast<float>(mean_val[0]);
    }

    float Depth::min_depth() const
    {
        if (depth.empty())
            return 0.0f;
        double min_val;
        cv::minMaxLoc(depth, &min_val, nullptr);
        return static_cast<float>(min_val);
    }

    float Depth::max_depth() const
    {
        if (depth.empty())
            return 0.0f;
        double max_val;
        cv::minMaxLoc(depth, nullptr, &max_val, nullptr, nullptr);
        return static_cast<float>(max_val);
    }

    float Depth::area_average_depth(const cv::Mat &seg) const
    {
        if (depth.empty() || seg.empty())
            return 0.0f;

        cv::Mat masked_depth;
        depth.copyTo(masked_depth, seg);

        float sum_depth = cv::sum(masked_depth)[0];
        float area = cv::countNonZero(seg);
        return area > 0 ? sum_depth / area : 0.0f;
    }

    float Depth::area_average_depth(const Box &box) const
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

    std::ostream &operator<<(std::ostream &os, const DetectionBox &box)
    {
        auto print_common_fields = [&](const DetectionBox &b)
        {
            os << ", \"score\": " << b.score
               << ", \"class_id\": " << b.class_id
               << ", \"class_name\": \"" << b.class_name << "\"";
        };

        switch (box.type)
        {
        case ObjectType::DETECTION:
            os << "{ \"type\": \"DETECTION\", \"box\": " << box.box;
            print_common_fields(box);
            os << " }";
            break;

        case ObjectType::POSE:
            os << "{ \"type\": \"POSE\", \"box\": " << box.box
               << ", \"pose\": " << box.pose;
            print_common_fields(box);
            os << " }";
            break;

        case ObjectType::OBB:
            os << "{ \"type\": \"OBB\", \"obb\": " << box.obb;
            print_common_fields(box);
            os << " }";
            break;

        case ObjectType::SEGMENTATION:
            os << "{ \"type\": \"SEGMENTATION\", \"box\": " << box.box;
            print_common_fields(box);
            os << " }";
            break;

        case ObjectType::TRACK:
            os << "{ \"type\": \"TRACK\", \"trace\": " << box.track;
            print_common_fields(box);
            os << " }";
            break;

        case ObjectType::DEPTH_ANYTHING:
            os << "{ \"type\": \"DEPTH_ANYTHING\" }";
            break;

        case ObjectType::DEPTH_PRO:
            os << "{ \"type\": \"DEPTH_PRO\" }";
            break;
        case ObjectType::POSITION:
            os << "{ \"type\": \"POSITION\", \"box\": " << box.box;
            print_common_fields(box);
            os << " }";
            break;
        case ObjectType::UNKNOW:
        default:
            os << "{ \"type\": \"UNKNOW\" }";
            break;
        }
        return os;
    }

} // namespace object