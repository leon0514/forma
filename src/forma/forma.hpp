#pragma once

#ifndef FORMA_HPP
#define FORMA_HPP

#include <string>
#include <vector>
#include "object.hpp"

namespace forma
{

    /*
     * @brief 计算矩形框面积
     * @note 矩形框面积 = (右下角x - 左上角x) * (右下角y - 左上角y)
     * @param box 矩形框
     * @return 矩形框面积
     */
    float box_area(const object::Box &box);

    /*
     * @brief 计算矩形框IOU
     * @note 矩形框IOU = 矩形框的交集的面积 / 矩形框的并集的面积
     * @param box1 矩形框1
     * @param box2 矩形框2
     * @return 矩形框IOU
     */
    float box_iou(const object::Box &box1, const object::Box &box2);

    /*
     * @brief 计算矩形框的交集的面积
     * @note 矩形框的交集的面积 = 矩形框1和矩形框2的交集的面积
     * @param box1 矩形框1
     * @param box2 矩形框2
     * @return 矩形框的交集的面积
     */
    float intersection_box_area(const object::Box &box1, const object::Box &box2);

    /*
     * @brief 计算相交矩形框面积和最小面积的比例
     * @note 相交矩形框面积和最小面积的比例 = 相交矩形框面积 / 最小面积
     * @param box1 矩形框1
     * @param box2 矩形框2
     * @return 相交矩形框面积和最小面积的比例
     */
    float intersection_over_min_box_ratio(const object::Box &box1, const object::Box &box2);

    /*
     * @brief 计算mask面积
     * @note mask面积 = mask中非0像素的个数
     * @param segmentation mask
     * @return mask面积
     */
    float mask_area(const object::Segmentation &segmentation);

    /*
     * @brief 计算mask的交集的面积
     * @note mask的交集的面积 = mask1和mask2的交集的面积
     * @param segmentation1 mask1
     * @param segmentation2 mask2
     * @return mask的交集的面积
     */
    float intersection_mask_area(const object::Segmentation &segmentation1, const object::Segmentation &segmentation2);

    /*
     * @brief 计算mask iou
     * @note mask iou = mask的交集的面积 / mask的并集的面积
     * @param segmentation1 mask1
     * @param segmentation2 mask2
     * @return mask iou
     */
    float mask_iou(const object::Segmentation &segmentation1, const object::Segmentation &segmentation2);

    /*
     * @brief 计算相交mask面积和最小mask面积的比例
     * @note 相交mask面积和最小mask面积的比例 = 相交mask面积 / 最小mask面积
     * @param segmentation1 mask1
     * @param segmentation2 mask2
     * @return 相交mask面积和最小mask面积的比例
     */
    float intersection_over_min_mask_ratio(const object::Segmentation &segmentation1, const object::Segmentation &segmentation2);

    /*
     * @brief 计算矩形框与mask的iou
     * @note 矩形框与mask的iou = 矩形框与mask的交集的面积 / 矩形框与mask的并集的面积
     * @param box 矩形框
     * @param segmentation mask
     * @return 矩形框与mask的iou
     */
    float box_mask_iou(const object::Box &box, const object::Segmentation &segmentation);

    /*
     * @brief 计算相交矩形框面积和最小面积的比例
     * @note 相交矩形框面积和最小面积的比例 = 相交矩形框面积 / 最小面积
     * @param box 矩形框
     * @param segmentation mask
     * @return 相交矩形框面积和最小面积的比例
     */
    float intersection_over_min_box_mask_ratio(const object::Box &box, const object::Segmentation &segmentation);

    /*
     * @brief 判断点是否在矩形框内
     * @note 判断点是否在矩形框内 = 点是否在矩形框的四个顶点内
     * @param point 点
     * @param box 矩形框
     * @return 点是否在矩形框内
     */
    bool point_in_box(const object::PosePoint &pose_point, const object::Box &box);

    /*
     * @brief 判断点是否在mask内
     * @note 判断点是否在mask内 = 点是否在mask的每个非0像素内
     * @param point 点
     * @param segmentation mask
     * @return 点是否在mask内
     */
    bool point_in_mask(const object::PosePoint &pose_point, const object::Segmentation &segmentation);

    /*
     * @brief 判断点是否在电子围栏内
     *
     * @param point 点
     * @param fence 电子围栏
     * @return 点是否在电子围栏内
     */
    bool point_in_fence(const object::PosePoint &pose_point, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 判断矩形框是否在电子围栏内
     * @note 判断矩形框是否在电子围栏内 = 矩形框的每个顶点是否在电子围栏内
     * @param box 矩形框
     * @param fence 电子围栏
     * @return 矩形框是否在电子围栏内
     */
    bool box_in_fence(const object::Box &box, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 判断mask是否在电子围栏内
     * @note 判断mask是否在电子围栏内 = mask的每个非0像素是否在电子围栏内
     * @param segmentation mask
     * @param fence 电子围栏
     * @return mask是否在电子围栏内
     */
    bool mask_in_fence(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence);

    /*
     * 计算矩形框与电子围栏的iou
     * @note 矩形框与电子围栏的iou = 矩形框与电子围栏的交集的面积 / 矩形框与电子围栏的并集的面积
     * @param box 矩形框
     * @param fence 电子围栏
     * @return 矩形框与电子围栏的iou
     */
    float box_fence_iou(const object::Box &box, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算mask与电子围栏的iou
     * @note mask与电子围栏的iou = mask与电子围栏的交集的面积 / mask与电子围栏的并集的面积
     * @param segmentation mask
     * @param fence 电子围栏
     * @return mask与电子围栏的iou
     */
    float mask_fence_iou(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算矩形框与电子围栏的交集的面积
     * @note 矩形框与电子围栏的交集的面积 = 矩形框与电子围栏的交集的面积
     * @param box 矩形框
     * @param fence 电子围栏
     * @return 矩形框与电子围栏的交集的面积
     */
    float intersection_box_fence_area(const object::Box &box, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算矩形框与电子围栏的交集的面积和最小面积的比例
     * @note 矩形框与电子围栏的交集的面积和最小面积的比例 = 矩形框与电子围栏的交集的面积 / 最小面积
     * @param box 矩形框
     * @param fence 电子围栏
     * @return 矩形框与电子围栏的交集的面积和最小面积的比例
     */
    float intersection_over_min_box_fence_ratio(const object::Box &box, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算mask与电子围栏的交集的面积
     * @note mask与电子围栏的交集的面积 = mask与电子围栏的交集的面积
     * @param segmentation mask
     * @param fence 电子围栏
     * @return mask与电子围栏的交集的面积
     */
    float intersection_mask_fence_area(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence);

    /*
     * @brief 计算mask与电子围栏的交集的面积和最小面积的比例
     * @note mask与电子围栏的交集的面积和最小面积的比例 = mask与电子围栏的交集的面积 / 最小面积
     * @param segmentation mask
     * @param fence 电子围栏
     * @return mask与电子围栏的交集的面积和最小面积的比例
     */
    float intersection_over_min_mask_fence_ratio(const object::Segmentation &segmentation, const std::vector<std::tuple<float, float>> &fence);

};

#endif