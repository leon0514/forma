#include "forma/forma.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <algorithm> // For std::min/max

// --- Boost Geometry 相关依赖 ---
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace
{
    // --- 内部类型别名 (统一使用 float) ---
    using BoostPoint = boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian>;
    using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;
    using BoostMultiPolygon = boost::geometry::model::multi_polygon<BoostPolygon>;

    // --- 内部辅助函数 ---

    BoostMultiPolygon mask_to_polygon(const object::Segmentation &segmentation)
    {
        std::vector<std::vector<cv::Point>> cv_contours;
        cv::findContours(segmentation.mask.clone(), cv_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        BoostMultiPolygon boost_polygons;
        for (const auto &cv_contour : cv_contours)
        {
            if (cv_contour.size() < 3) continue;
            BoostPolygon boost_polygon;
            for (const auto &cv_point : cv_contour)
            {
                boost_polygon.outer().push_back(BoostPoint(static_cast<float>(cv_point.x), static_cast<float>(cv_point.y)));
            }
            boost::geometry::correct(boost_polygon);
            boost_polygons.push_back(boost_polygon);
        }
        return boost_polygons;
    }

    BoostPolygon fence_to_polygon(const std::vector<std::tuple<float, float>> &fence)
    {
        if (fence.size() < 3) return BoostPolygon();
        BoostPolygon polygon;
        for (const auto &fence_point : fence)
        {
            polygon.outer().push_back(BoostPoint(std::get<0>(fence_point), std::get<1>(fence_point)));
        }
        boost::geometry::correct(polygon);
        return polygon;
    }

    BoostPolygon box_to_polygon(const object::Box &box)
    {
        BoostPolygon polygon;
        polygon.outer().push_back(BoostPoint(box.left, box.top));
        polygon.outer().push_back(BoostPoint(box.right, box.top));
        polygon.outer().push_back(BoostPoint(box.right, box.bottom));
        polygon.outer().push_back(BoostPoint(box.left, box.bottom));
        polygon.outer().push_back(BoostPoint(box.left, box.top));
        boost::geometry::correct(polygon);
        return polygon;
    }

    // --- 核心优化逻辑 ---
    
    struct IntersectionMetrics
    {
        float intersection_area = 0.0f;
        float area1 = 0.0f;
        float area2 = 0.0f;
    };

    template<typename Geometry1, typename Geometry2>
    IntersectionMetrics _calculate_intersection_metrics(const Geometry1& geom1, const Geometry2& geom2)
    {
        IntersectionMetrics metrics;
        metrics.area1 = boost::geometry::area(geom1);
        metrics.area2 = boost::geometry::area(geom2);
        // NUMERIC FIX: Only calculate intersection if areas are non-trivial
        if (metrics.area1 > 1e-6f && metrics.area2 > 1e-6f)
        {
            BoostMultiPolygon intersection_geometry;
            boost::geometry::intersection(geom1, geom2, intersection_geometry);
            metrics.intersection_area = boost::geometry::area(intersection_geometry);
        }
        return metrics;
    }
} // 匿名命名空间结束

namespace forma
{
    // --- Box-Box 相关函数实现 ---

    float box_area(const object::Box &box)
    {
        return std::max(0.0f, box.right - box.left) * std::max(0.0f, box.bottom - box.top);
    }

    float intersection_box_area(const object::Box &box1, const object::Box &box2)
    {
        float left = std::max(box1.left, box2.left);
        float top = std::max(box1.top, box2.top);
        float right = std::min(box1.right, box2.right);
        float bottom = std::min(box1.bottom, box2.bottom);
        return std::max(0.0f, right - left) * std::max(0.0f, bottom - top);
    }

    float box_iou(const object::Box &box1, const object::Box &box2)
    {
        float intersection_area = intersection_box_area(box1, box2);
        float union_area = box_area(box1) + box_area(box2) - intersection_area;
        // NUMERIC FIX: Use float epsilon for robust comparison
        return (union_area <= 1e-6f) ? 0.0f : intersection_area / union_area;
    }

    float intersection_over_min_box_ratio(const object::Box &box1, const object::Box &box2)
    {
        float intersection_area = intersection_box_area(box1, box2);
        float min_area = std::min(box_area(box1), box_area(box2));
        // NUMERIC FIX: Use float epsilon for robust comparison
        return (min_area <= 1e-6f) ? 0.0f : intersection_area / min_area;
    }

    // --- Mask-Mask 相关函数实现 ---

    float mask_area(const object::Segmentation &segmentation)
    {
        return boost::geometry::area(mask_to_polygon(segmentation));
    }

    float intersection_mask_area(const object::Segmentation &segmentation1, const object::Segmentation &segmentation2)
    {
        auto mask_polygons1 = mask_to_polygon(segmentation1);
        auto mask_polygons2 = mask_to_polygon(segmentation2);
        BoostMultiPolygon intersection_geometry;
        boost::geometry::intersection(mask_polygons1, mask_polygons2, intersection_geometry);
        return boost::geometry::area(intersection_geometry);
    }

    float mask_iou(const object::Segmentation &segmentation1, const object::Segmentation &segmentation2)
    {
        auto metrics = _calculate_intersection_metrics(mask_to_polygon(segmentation1), mask_to_polygon(segmentation2));
        float union_area = metrics.area1 + metrics.area2 - metrics.intersection_area;
        return (union_area <= 1e-6f) ? 0.0f : metrics.intersection_area / union_area;
    }

    float intersection_over_min_mask_ratio(const object::Segmentation &segmentation1, const object::Segmentation &segmentation2)
    {
        auto metrics = _calculate_intersection_metrics(mask_to_polygon(segmentation1), mask_to_polygon(segmentation2));
        float min_area = std::min(metrics.area1, metrics.area2);
        return (min_area <= 1e-6f) ? 0.0f : metrics.intersection_area / min_area;
    }

    // --- Box-Mask 相关函数实现 ---

    float box_mask_iou(const object::Box &box, const object::Segmentation &segmentation)
    {
        auto box_poly = box_to_polygon(box);
        auto mask_polys = mask_to_polygon(segmentation);
        BoostMultiPolygon intersection_geometry;
        boost::geometry::intersection(box_poly, mask_polys, intersection_geometry);
        float intersection_area = boost::geometry::area(intersection_geometry);
        float union_area = box_area(box) + boost::geometry::area(mask_polys) - intersection_area;
        return (union_area <= 1e-6f) ? 0.0f : intersection_area / union_area;
    }

    float intersection_over_min_box_mask_ratio(const object::Box &box, const object::Segmentation &segmentation)
    {
        auto box_poly = box_to_polygon(box);
        auto mask_polys = mask_to_polygon(segmentation);
        BoostMultiPolygon intersection_geometry;
        boost::geometry::intersection(box_poly, mask_polys, intersection_geometry);
        float intersection_area = boost::geometry::area(intersection_geometry);
        float min_area = std::min(box_area(box), static_cast<float>(boost::geometry::area(mask_polys)));
        return (min_area <= 1e-6f) ? 0.0f : intersection_area / min_area;
    }

    // --- Point 相关函数实现 ---

    bool point_in_box(const object::PosePoint &pose_point, const object::Box &box)
    {
        return pose_point.x >= box.left && pose_point.x <= box.right &&
               pose_point.y >= box.top && pose_point.y <= box.bottom;
    }
    
    bool point_in_box(const std::tuple<float, float> &point, const object::Box &box)
    {
        const float x = std::get<0>(point);
        const float y = std::get<1>(point);
        return x >= box.left && x <= box.right && y >= box.top && y <= box.bottom;
    }
    
    bool point_in_mask(const object::PosePoint &pose_point, const object::Segmentation &segmentation)
    {
        BoostPoint point(pose_point.x, pose_point.y);
        return boost::geometry::covered_by(point, mask_to_polygon(segmentation));
    }
    
    bool point_in_mask(const std::tuple<float, float> &point, const object::Segmentation &segmentation)
    {
        BoostPoint boost_point(std::get<0>(point), std::get<1>(point));
        return boost::geometry::covered_by(boost_point, mask_to_polygon(segmentation));
    }

    bool point_in_fence(const object::PosePoint &pose_point, const std::vector<std::tuple<float, float>> &fence)
    {
        BoostPoint point(pose_point.x, pose_point.y);
        return boost::geometry::covered_by(point, fence_to_polygon(fence));
    }

    bool point_in_fence(const std::tuple<float, float> &point, const std::vector<std::tuple<float, float>> &fence)
    {
        BoostPoint boost_point(std::get<0>(point), std::get<1>(point));
        return boost::geometry::covered_by(boost_point, fence_to_polygon(fence));
    }
    
    // --- Fence 相关函数实现 ---

    bool box_in_fence(const object::Box &box, const std::vector<std::tuple<float, float>> &fence)
    {
        return boost::geometry::covered_by(box_to_polygon(box), fence_to_polygon(fence));
    }

    bool mask_in_fence(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence)
    {
        return boost::geometry::within(mask_to_polygon(segmentation), fence_to_polygon(fence));
    }

    float intersection_box_fence_area(const object::Box &box, const std::vector<std::tuple<float, float>> &fence)
    {
        auto box_poly = box_to_polygon(box);
        auto fence_poly = fence_to_polygon(fence);
        BoostMultiPolygon intersection_geometry;
        boost::geometry::intersection(box_poly, fence_poly, intersection_geometry);
        return boost::geometry::area(intersection_geometry);
    }
    
    float box_fence_iou(const object::Box &box, const std::vector<std::tuple<float, float>> &fence)
    {
        auto metrics = _calculate_intersection_metrics(box_to_polygon(box), fence_to_polygon(fence));
        float union_area = box_area(box) + metrics.area2 - metrics.intersection_area;
        return (union_area <= 1e-6f) ? 0.0f : metrics.intersection_area / union_area;
    }

    float intersection_mask_fence_area(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence)
    {
        auto mask_polys = mask_to_polygon(segmentation);
        auto fence_poly = fence_to_polygon(fence);
        BoostMultiPolygon intersection_geometry;
        boost::geometry::intersection(mask_polys, fence_poly, intersection_geometry);
        return boost::geometry::area(intersection_geometry);
    }

    float mask_fence_iou(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence)
    {
        auto metrics = _calculate_intersection_metrics(mask_to_polygon(segmentation), fence_to_polygon(fence));
        float union_area = metrics.area1 + metrics.area2 - metrics.intersection_area;
        return (union_area <= 1e-6f) ? 0.0f : metrics.intersection_area / union_area;
    }

    float intersection_over_min_box_fence_ratio(const object::Box &box, const std::vector<std::tuple<float, float>> &fence)
    {
        auto metrics = _calculate_intersection_metrics(box_to_polygon(box), fence_to_polygon(fence));
        float min_area = std::min(box_area(box), metrics.area2);
        return (min_area <= 1e-6f) ? 0.0f : metrics.intersection_area / min_area;
    }

    float intersection_over_min_mask_fence_ratio(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence)
    {
        auto metrics = _calculate_intersection_metrics(mask_to_polygon(segmentation), fence_to_polygon(fence));
        float min_area = std::min(metrics.area1, metrics.area2);
        return (min_area <= 1e-6f) ? 0.0f : metrics.intersection_area / min_area;
    }

} // namespace forma