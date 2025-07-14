#include "forma/forma.hpp"
#include "opencv2/opencv.hpp" 
#include <iostream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

// 定义Boost.Geometry中使用的几何类型别名
using BoostPoint = boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;
using BoostMultiPolygon = boost::geometry::model::multi_polygon<BoostPolygon>;

namespace 
{
// 辅助函数，用于内部几何转换

/**
 * @brief 将 OpenCV 的掩码转换为 Boost.Geometry 多边形的集合。
 * @note 此版本使用 cv::RETR_EXTERNAL，因此只会提取最外层的轮廓，无法处理带孔洞的掩码。
 *       如果需要处理孔洞，需要改用 cv::RETR_CCOMP 或 cv::RETR_TREE 并处理层级关系。
 * 
 * @param segmentation 包含 cv::Mat 掩码的对象。
 * @return BoostMultiPolygon 包含从掩码中提取的所有外部轮廓的多边形集合。   
 */
BoostMultiPolygon mask2polygon(const object::Segmentation& segmentation)
{
    std::vector<std::vector<cv::Point>> cv_contours;
    
    // cv::findContours 会修改输入图像，所以传入一个副本
    cv::findContours(segmentation.mask.clone(), cv_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    BoostMultiPolygon boost_polygons;
    for (const auto& cv_contour : cv_contours)
    {
        BoostPolygon boost_polygon;
        for (const auto& cv_point : cv_contour)
        {
            boost_polygon.outer().push_back(BoostPoint(static_cast<float>(cv_point.x), static_cast<float>(cv_point.y)));
        }
        
        // 修正多边形，确保其闭合和顶点顺序正确（外环通常是逆时针）。
        // 这是 Boost.Geometry 的最佳实践。
        boost::geometry::correct(boost_polygon);
        boost_polygons.push_back(boost_polygon);
    }
    
    return boost_polygons;
}

/**
 * @brief 将顶点向量（围栏）转换为一个 Boost.Geometry 多边形。
 * 
 * @param fence 包含多边形顶点的元组向量。
 * @return BoostPolygon 转换后的多边形。
 */
BoostPolygon fence2polygon(const std::vector<std::tuple<float, float>>& fence)
{
    // 至少需要3个点才能构成一个多边形
    if (fence.size() < 3) {
        return BoostPolygon(); // 返回一个空多边形
    }

    BoostPolygon polygon;
    for (const auto& fence_point : fence)
    {
        polygon.outer().push_back(BoostPoint(std::get<0>(fence_point), std::get<1>(fence_point)));
    }
    
    // 修正多边形，确保其闭合和顶点顺序正确
    boost::geometry::correct(polygon);
    
    return polygon;
}

/**
 * @brief 将 object::Box 转换为 Boost.Geometry 多边形。
 * 
 * @param box 边界框对象。
 * @return BoostPolygon 转换后的矩形多边形。
 */
BoostPolygon box2polygon(const object::Box& box)
{
    BoostPolygon polygon;
    polygon.outer().push_back(BoostPoint(box.left, box.top));
    polygon.outer().push_back(BoostPoint(box.right, box.top));
    polygon.outer().push_back(BoostPoint(box.right, box.bottom));
    polygon.outer().push_back(BoostPoint(box.left, box.bottom));
    
    // 修正多边形，确保顶点顺序和闭合
    boost::geometry::correct(polygon);
    return polygon;
}

} // 匿名命名空间结束

namespace forma{

// --- Box-Box 相关函数 (本身就是几何计算，无需修改) ---

float box_area(const object::Box& box)
{
    // 注意：这里的计算是基于坐标的解析几何面积，而不是Boost.Geometry
    // 这对于轴对齐的矩形来说是最高效且准确的。
    return (box.right - box.left) * (box.bottom - box.top);
}

float intersection_box_area(const object::Box& box1, const object::Box& box2)
{
    float left = std::max(box1.left, box2.left);
    float top = std::max(box1.top, box2.top);
    float right = std::min(box1.right, box2.right);
    float bottom = std::min(box1.bottom, box2.bottom);

    float width = std::max(0.0f, right - left);
    float height = std::max(0.0f, bottom - top);

    return width * height;
}

float box_iou(const object::Box& box1, const object::Box& box2)
{
    float box1_area = box_area(box1);
    float box2_area = box_area(box2);
    float intersection_area = intersection_box_area(box1, box2);
    float union_area = box1_area + box2_area - intersection_area;
    if (union_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / union_area;
}

float intersection_over_min_box_ratio(const object::Box& box1, const object::Box& box2)
{
    float intersection_area = intersection_box_area(box1, box2);
    float min_area = std::min(box_area(box1), box_area(box2));
    if (min_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / min_area;
}

// --- Mask-Mask 相关函数 (已重构为几何计算) ---

float mask_area(const object::Segmentation& mask)
{
    // --- REFACTORED: Now uses geometric area calculation ---
    BoostMultiPolygon mask_polygons = mask2polygon(mask);
    return boost::geometry::area(mask_polygons);
}

float intersection_mask_area(const object::Segmentation& segmentation1, const object::Segmentation& segmentation2)
{
    // --- REFACTORED: Now uses geometric area calculation ---
    BoostMultiPolygon mask_polygons1 = mask2polygon(segmentation1);
    BoostMultiPolygon mask_polygons2 = mask2polygon(segmentation2);
    
    BoostMultiPolygon intersection_geometry;
    boost::geometry::intersection(mask_polygons1, mask_polygons2, intersection_geometry);
    
    return boost::geometry::area(intersection_geometry);
}

float mask_iou(const object::Segmentation& segmentation1, const object::Segmentation& segmentation2)
{
    // --- REFACTORED: Now uses geometric area calculation ---
    // Note: Re-calculating areas here can be slightly inefficient if they are already known.
    // For simplicity, we calculate them again.
    BoostMultiPolygon mask_polygons1 = mask2polygon(segmentation1);
    BoostMultiPolygon mask_polygons2 = mask2polygon(segmentation2);

    float mask1_area = boost::geometry::area(mask_polygons1);
    float mask2_area = boost::geometry::area(mask_polygons2);
    
    BoostMultiPolygon intersection_geometry;
    boost::geometry::intersection(mask_polygons1, mask_polygons2, intersection_geometry);
    float intersection_area = boost::geometry::area(intersection_geometry);

    float union_area = mask1_area + mask2_area - intersection_area;
    if (union_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / union_area;
}

float intersection_over_min_mask_ratio(const object::Segmentation& segmentation1, const object::Segmentation& segmentation2)
{
    // --- REFACTORED: Now uses geometric area calculation ---
    BoostMultiPolygon mask_polygons1 = mask2polygon(segmentation1);
    BoostMultiPolygon mask_polygons2 = mask2polygon(segmentation2);

    float intersection_area = intersection_mask_area(segmentation1, segmentation2); // Re-uses the geometric function
    float min_area = std::min(boost::geometry::area(mask_polygons1), boost::geometry::area(mask_polygons2));
    
    if (min_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / min_area;
}

// --- Box-Mask 相关函数 (已重构为几何计算) ---

float box_mask_iou(const object::Box& box, const object::Segmentation& segmentation)
{
    // --- REFACTORED: Now uses geometric area calculation ---
    BoostPolygon box_poly = box2polygon(box);
    BoostMultiPolygon mask_polys = mask2polygon(segmentation);

    // 计算交集几何体及其面积
    BoostMultiPolygon intersection_geometry;
    boost::geometry::intersection(box_poly, mask_polys, intersection_geometry);
    float intersection_area = boost::geometry::area(intersection_geometry);

    // 获取各自的几何面积
    float box_geo_area = box_area(box); // Uses efficient analytical calculation
    float mask_geo_area = boost::geometry::area(mask_polys);
    
    float union_area = box_geo_area + mask_geo_area - intersection_area;
    
    if (union_area <= 0)
    {
        return 0.0f;
    }
    
    return intersection_area / union_area;
}

float intersection_over_min_box_mask_ratio(const object::Box& box, const object::Segmentation& segmentation)
{
    // --- REFACTORED: Now uses geometric area calculation ---
    BoostPolygon box_poly = box2polygon(box);
    BoostMultiPolygon mask_polys = mask2polygon(segmentation);

    // 计算交集几何体及其面积
    BoostMultiPolygon intersection_geometry;
    boost::geometry::intersection(box_poly, mask_polys, intersection_geometry);
    float intersection_area = boost::geometry::area(intersection_geometry);

    // 获取各自的几何面积
    float box_geo_area = box_area(box);
    float mask_geo_area = boost::geometry::area(mask_polys);
    float min_area = std::min(box_geo_area, mask_geo_area);

    if (min_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / min_area;
}

// --- Fence 相关函数 (本身就是几何计算，无需修改) ---

bool point_in_fence(const object::PosePoint& pose_point, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPoint point(pose_point.x, pose_point.y);
    BoostPolygon fence_polygon = fence2polygon(fence);
    return boost::geometry::covered_by(point, fence_polygon);
}

bool box_in_fence(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostPolygon box_polygon = box2polygon(box);
    return boost::geometry::covered_by(box_polygon, fence_polygon);
}

bool mask_in_fence(const object::Segmentation& mask, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostMultiPolygon mask_polygons = mask2polygon(mask);
    return boost::geometry::within(mask_polygons, fence_polygon);
}

float intersection_box_fence_area(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostPolygon box_polygon = box2polygon(box);
    BoostMultiPolygon intersection_geometry;
    boost::geometry::intersection(box_polygon, fence_polygon, intersection_geometry);
    return boost::geometry::area(intersection_geometry);
}

float box_fence_iou(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPolygon fence_polygon = fence2polygon(fence);
    float fence_area = boost::geometry::area(fence_polygon);
    float intersection_area = intersection_box_fence_area(box, fence);
    float union_area = box_area(box) + fence_area - intersection_area;
    if (union_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / union_area;
}

float intersection_mask_fence_area(const object::Segmentation& segmentation, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostMultiPolygon mask_polygons = mask2polygon(segmentation);
    
    BoostMultiPolygon intersection_geometry;
    boost::geometry::intersection(mask_polygons, fence_polygon, intersection_geometry);
    
    return boost::geometry::area(intersection_geometry);
}

float mask_fence_iou(const object::Segmentation& mask, const std::vector<std::tuple<float, float>>& fence)
{
    BoostMultiPolygon mask_polygons = mask2polygon(mask);
    BoostPolygon fence_polygon = fence2polygon(fence);

    float mask_geo_area = boost::geometry::area(mask_polygons);
    float fence_geo_area = boost::geometry::area(fence_polygon);
    float intersection_area = intersection_mask_fence_area(mask, fence);

    float union_area = mask_geo_area + fence_geo_area - intersection_area;
    
    if (union_area <= 0)
    {
        return 0.0f;
    }
    
    return intersection_area / union_area;
}

float intersection_over_min_box_fence_ratio(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)
{
    float intersection_area = intersection_box_fence_area(box, fence);
    float fence_area = boost::geometry::area(fence2polygon(fence));
    float min_area = std::min(box_area(box), fence_area);
    if (min_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / min_area;
}

float intersection_over_min_mask_fence_ratio(const object::Segmentation& segmentation, const std::vector<std::tuple<float, float>>& fence)
{
    float intersection_area = intersection_mask_fence_area(segmentation, fence);
    
    float mask_geo_area = boost::geometry::area(mask2polygon(segmentation));
    float fence_geo_area = boost::geometry::area(fence2polygon(fence));

    float min_area = std::min(mask_geo_area, fence_geo_area);
    if (min_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / min_area;
}

// --- Point 相关函数 ---

bool point_in_box(const object::PosePoint& pose_point, const object::Box& box)
{
    // 这是最高效的判断方式，无需转换为多边形
    return pose_point.x >= box.left && pose_point.x <= box.right && 
           pose_point.y >= box.top && pose_point.y <= box.bottom;
}

bool point_in_mask(const object::PosePoint& pose_point, const object::Segmentation& segmentation)
{
    // --- REFACTORED: Now uses geometric check ---
    // 尽管像素检查可能更快，但为了保持几何计算的一致性，这里也使用Boost.Geometry
    BoostPoint point(pose_point.x, pose_point.y);
    BoostMultiPolygon mask_polygons = mask2polygon(segmentation);
    return boost::geometry::covered_by(point, mask_polygons);
}

} // namespace forma