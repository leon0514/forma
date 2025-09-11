#pragma once

#ifndef FORMA_HPP
#define FORMA_HPP

#include "object.hpp" // 假设 object.hpp 定义了 object::Box, object::Segmentation 等
#include <string>
#include <vector>
#include <tuple>

namespace forma
{

    /*
     * @brief 计算矩形框面积
     */
    float box_area(const object::Box &box);

    /*
     * @brief 计算矩形框IOU
     */
    float box_iou(const object::Box &box1, const object::Box &box2);

    /*
     * @brief 计算矩形框的交集的面积
     */
    float intersection_box_area(const object::Box &box1, const object::Box &box2);

    /*
     * @brief 计算相交矩形框面积和最小面积的比例
     */
    float intersection_over_min_box_ratio(const object::Box &box1, const object::Box &box2);

    /*
     * @brief 计算mask面积
     */
    float mask_area(const object::Segmentation &segmentation);

    /*
     * @brief 计算mask的交集的面积
     */
    float intersection_mask_area(const object::Segmentation &segmentation1, const object::Segmentation &segmentation2);

    /*
     * @brief 计算mask iou
     */
    float mask_iou(const object::Segmentation &segmentation1, const object::Segmentation &segmentation2);

    /*
     * @brief 计算相交mask面积和最小mask面积的比例
     */
    float intersection_over_min_mask_ratio(const object::Segmentation &segmentation1, const object::Segmentation &segmentation2);

    /*
     * @brief 计算矩形框与mask的iou
     */
    float box_mask_iou(const object::Box &box, const object::Segmentation &segmentation);

    /*
     * @brief 计算矩形框与mask相交面积和最小面积的比例
     */
    float intersection_over_min_box_mask_ratio(const object::Box &box, const object::Segmentation &segmentation);

    /*
     * @brief 判断点是否在矩形框内
     */
    bool point_in_box(const object::PosePoint &pose_point, const object::Box &box);

    /*
     * @brief 判断点是否在mask内
     */
    bool point_in_mask(const object::PosePoint &pose_point, const object::Segmentation &segmentation);

    /*
     * @brief 判断点是否在电子围栏内
     */
    bool point_in_fence(const object::PosePoint &pose_point, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 判断点是否在矩形框内 (元组版本)
     */
    bool point_in_box(const std::tuple<float, float>& point, const object::Box &box);

    /*
     * @brief 判断点是否在mask内 (元组版本)
     */
    bool point_in_mask(const std::tuple<float, float>& point, const object::Segmentation &segmentation);

    /*
     * @brief 判断点是否在电子围栏内 (元组版本)
     */
    bool point_in_fence(const std::tuple<float, float>& point, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 判断矩形框是否在电子围栏内
     */
    bool box_in_fence(const object::Box &box, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 判断mask是否在电子围栏内
     */
    bool mask_in_fence(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算矩形框与电子围栏的iou
     */
    float box_fence_iou(const object::Box &box, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算mask与电子围栏的iou
     */
    float mask_fence_iou(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算矩形框与电子围栏的交集的面积
     */
    float intersection_box_fence_area(const object::Box &box, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算矩形框与电子围栏的交集面积和最小面积的比例
     */
    float intersection_over_min_box_fence_ratio(const object::Box &box, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算mask与电子围栏的交集的面积
     */
    float intersection_mask_fence_area(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算mask与电子围栏的交集面积和最小面积的比例
     */
    float intersection_over_min_mask_fence_ratio(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence);

};

#endif // FORMA_HPP