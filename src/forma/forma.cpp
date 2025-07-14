#include "forma/forma.hpp"
#include "opencv2/opencv.hpp" 
#include <iostream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using BoostPoint = boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;
using BoostMultiPolygon = boost::geometry::model::multi_polygon<BoostPolygon>;

namespace 
{

/**
 * @brief 将 OpenCV 的掩码转换为 Boost.Geometry 多边形的向量。
 * @note 此版本使用 cv::RETR_EXTERNAL，因此只会提取最外层的轮廓，无法处理带孔洞的掩码。
 *       如果需要处理孔洞，需要改用 cv::RETR_CCOMP 或 cv::RETR_TREE 并处理层级关系。
 * 
 * @param segmentation 包含 cv::Mat 掩码的对象。
 * @return std::vector<BoostPolygon> 包含从掩码中提取的所有外部轮廓的多边形。
 */
 BoostMultiPolygon mask2polygon(const object::Segmentation& segmentation)
{
    // 1. 声明正确类型的变量来接收 OpenCV 的轮廓
    std::vector<std::vector<cv::Point>> cv_contours;
    
    // cv::findContours 会修改输入图像，所以最好传入一个副本
    cv::findContours(segmentation.mask.clone(), cv_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 2. 声明正确类型的返回值变量
    BoostMultiPolygon boost_polygons;

    // 3. 遍历 OpenCV 轮廓并转换为 BoostPolygon
    for (const auto& cv_contour : cv_contours)
    {
        BoostPolygon boost_polygon;
        for (const auto& cv_point : cv_contour)
        {
            // 4. 使用 point.x 和 point.y 访问坐标
            boost_polygon.outer().push_back(BoostPoint(static_cast<float>(cv_point.x), static_cast<float>(cv_point.y)));
        }
        
        // 5. 修正多边形，这是 Boost.Geometry 的最佳实践。
        //    它会确保多边形是闭合的，并且顶点顺序正确（外环通常是逆时针）。
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

BoostPolygon box2polygon(const object::Box& box)
{
    BoostPolygon polygon;
    polygon.outer().push_back(BoostPoint(box.left, box.top));
    polygon.outer().push_back(BoostPoint(box.right, box.top));
    polygon.outer().push_back(BoostPoint(box.right, box.bottom));
    polygon.outer().push_back(BoostPoint(box.left, box.bottom));
    boost::geometry::correct(polygon);
    return polygon;
}


}

namespace forma{


float box_area(const object::Box& box)
{
    return (box.right - box.left) * (box.bottom - box.top);
}



float intersection_box_area(const object::Box& box1, const object::Box& box2)
{
    float left = std::max(box1.left, box2.left);
    float top = std::max(box1.top, box2.top);
    float right = std::min(box1.right, box2.right);
    float bottom = std::min(box1.bottom, box2.bottom);
    return (right - left) * (bottom - top);
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


float mask_area(const object::Segmentation& mask)
{
    return cv::countNonZero(mask.mask);
}


float intersection_mask_area(const object::Segmentation& segmentation1, const object::Segmentation& segmentation2)
{
    return cv::countNonZero(segmentation1.mask & segmentation2.mask);
}


float mask_iou(const object::Segmentation& segmentation1, const object::Segmentation& segmentation2)
{
    float mask1_area = mask_area(segmentation1);
    float mask2_area = mask_area(segmentation2);
    float intersection_area = intersection_mask_area(segmentation1, segmentation2);
    float union_area = mask1_area + mask2_area - intersection_area;
    if (union_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / union_area;
}


float intersection_over_min_mask_ratio(const object::Segmentation& segmentation1, const object::Segmentation& segmentation2)
{
    float intersection_area = intersection_mask_area(segmentation1, segmentation2);
    float min_area = std::min(mask_area(segmentation1), mask_area(segmentation2));
    if (min_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / min_area;
}


float box_mask_iou(const object::Box& box, const object::Segmentation& segmentation)
{
    int width = segmentation.mask.cols;
    int height = segmentation.mask.rows;
    cv::Mat box_mask = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Rect box_rect(box.left, box.top, box.right - box.left, box.bottom - box.top);
    box_mask(box_rect) = 255;
    cv::Mat intersection = segmentation.mask & box_mask;
    float intersection_area = cv::countNonZero(intersection);
    float union_area = box_area(box) + mask_area(segmentation) - intersection_area;
    if (union_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / union_area;
}


float intersection_over_min_box_mask_ratio(const object::Box& box, const object::Segmentation& segmentation)
{
    cv::Mat box_mask = cv::Mat::zeros(segmentation.mask.rows, segmentation.mask.cols, CV_8UC1);
    cv::Rect box_rect(box.left, box.top, box.right - box.left, box.bottom - box.top);
    box_mask(box_rect) = 255;
    cv::Mat intersection = segmentation.mask & box_mask;
    float intersection_area = cv::countNonZero(intersection);
    float min_area = std::min(box_area(box), mask_area(segmentation));
    if (min_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / min_area;
}


bool point_in_fence(const object::PosePoint& pose_point, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPoint point(pose_point.x, pose_point.y);
    BoostPolygon fence_polygon;
    for (const auto& fence_point : fence)
    {
        fence_polygon.outer().push_back(boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian>(std::get<0>(fence_point), std::get<1>(fence_point)));
    }
    return boost::geometry::within(BoostPoint(pose_point.x, pose_point.y), fence_polygon);
}


bool box_in_fence(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostPolygon box_polygon = box2polygon(box);
    return boost::geometry::within(box_polygon, fence_polygon);
}


bool mask_in_fence(const object::Segmentation& mask, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostMultiPolygon mask_polygons = mask2polygon(mask);
    for (const auto& mask_polygon : mask_polygons)
    {
        if (!boost::geometry::within(mask_polygon, fence_polygon))
        {
            return false;
        }
    }
    return true;
}


float intersection_box_fence_area(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)
{
    // 1. 将输入转换为 Boost.Geometry 的多边形
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostPolygon box_polygon = box2polygon(box);

    // 2. 创建一个变量来存储交集的结果。
    //    使用 BoostMultiPolygon 是最稳妥的选择，因为交集可能包含多个不相连的区域。
    BoostMultiPolygon intersection_geometry;

    // 3. 调用 boost::geometry::intersection 来计算交集形状。
    //    这个函数将结果填充到 intersection_geometry 中。
    boost::geometry::intersection(box_polygon, fence_polygon, intersection_geometry);

    // 4. 使用 boost::geometry::area 来计算交集形状的面积。
    //    这个函数会正确处理 MultiPolygon 的情况（将所有子多边形的面积相加）。
    float intersection_area = boost::geometry::area(intersection_geometry);
    
    return intersection_area;
}


float box_fence_iou(const object::Box& box, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostPolygon box_polygon = box2polygon(box);
    BoostMultiPolygon intersection_geometry;
    boost::geometry::intersection(box_polygon, fence_polygon, intersection_geometry);
    float intersection_area = boost::geometry::area(intersection_geometry);
    float union_area = box_area(box) + boost::geometry::area(fence_polygon) - intersection_area;
    if (union_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / union_area;
}


float mask_fence_iou(const object::Segmentation& mask, const std::vector<std::tuple<float, float>>& fence)
{
    float sum_area = 0.0f;
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostMultiPolygon mask_polygons = mask2polygon(mask);
    for (const auto& mask_polygon : mask_polygons)
    {
        sum_area += boost::geometry::area(mask_polygon);
    }
    BoostMultiPolygon intersection_polygons;
    boost::geometry::intersection(mask_polygons, fence_polygon, intersection_polygons);
    float intersection_area = 0.0f;
    for (const auto& intersection_polygon : intersection_polygons)
    {
        intersection_area += boost::geometry::area(intersection_polygon);
    }
    return intersection_area / sum_area;
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


float intersection_mask_fence_area(const object::Segmentation& segmentation, const std::vector<std::tuple<float, float>>& fence)
{
    BoostPolygon fence_polygon = fence2polygon(fence);
    BoostMultiPolygon mask_polygons = mask2polygon(segmentation);
    float intersection_area = 0.0f;
    for (const auto& mask_polygon : mask_polygons)
    {
        BoostMultiPolygon intersection_polygons;
        boost::geometry::intersection(mask_polygon, fence_polygon, intersection_polygons);
        for (const auto& intersection_polygon : intersection_polygons)
        {
            intersection_area += boost::geometry::area(intersection_polygon);
        }
    }
    return intersection_area;
}

float intersection_over_min_mask_fence_ratio(const object::Segmentation& segmentation, const std::vector<std::tuple<float, float>>& fence)
{
    float intersection_area = intersection_mask_fence_area(segmentation, fence);
    float fence_area = boost::geometry::area(fence2polygon(fence));
    float min_area = std::min(mask_area(segmentation), fence_area);
    if (min_area <= 0)
    {
        return 0.0f;
    }
    return intersection_area / min_area;
}





}