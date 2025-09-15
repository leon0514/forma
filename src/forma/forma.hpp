#pragma once

#ifndef FORMA_HPP
#define FORMA_HPP

#include "object.hpp" // 假设 object.hpp 定义了 object::Box, object::Segmentation 等
#include <string>
#include <vector>
#include <tuple>

namespace forma
{

    enum class CrossingDirection
    {
        NONE,   // 轨迹未穿越边界
        IN,     // 轨迹从区域外进入区域内
        OUT,    // 轨迹从区域内离开到区域外
        BOTH    // 在同一条轨迹中，既有进入又有离开事件
    };

    // 为方便调试，重载输出操作符
    inline std::ostream &operator<<(std::ostream &os, const CrossingDirection &dir)
    {
        switch (dir)
        {
            case CrossingDirection::NONE: os << "NONE"; break;
            case CrossingDirection::IN:   os << "IN"; break;
            case CrossingDirection::OUT:  os << "OUT"; break;
            case CrossingDirection::BOTH: os << "BOTH"; break;
        }
        return os;
    }

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


    /**
     * @brief 判断轨迹穿越矩形框(Box)的方向
     * @param track 对象轨迹
     * @param box 矩形框
     * @return CrossingDirection 穿越方向
     */
    CrossingDirection track_crossing_direction_box(const object::Track &track, const object::Box &box);

    /**
     * @brief 判断轨迹穿越围栏(Fence)的方向
     * @param track 对象轨迹
     * @param fence 围栏
     * @return CrossingDirection 穿越方向
     */
    CrossingDirection track_crossing_direction_fence(const object::Track &track, const std::vector<std::tuple<float, float>> &fence);

    /**
     * @brief 判断轨迹穿越分割区域(Segmentation)的方向
     * @param track 对象轨迹
     * @param segmentation 分割区域
     * @return CrossingDirection 穿越方向
     */
    CrossingDirection track_crossing_direction_segmentation(const object::Track &track, const object::Segmentation &segmentation);

};

#endif // FORMA_HPP