#include "forma/forma.hpp"
#include "object.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

int main()
{
    object::Box box(0, 0, 100, 100);
    object::Segmentation segmentation;
    segmentation.mask = cv::Mat::zeros(100, 100, CV_8UC1);
    // 1. 定义矩形
    cv::Rect rect_to_fill(0, 0, 100, 100);

    // 2. 调用 cv::rectangle 函数
    cv::rectangle(
        segmentation.mask,        // 目标图像
        rect_to_fill,             // 要填充的矩形
        cv::Scalar(255),          // 颜色 (对于单通道就是像素值)
        cv::FILLED                // 关键参数：表示填充矩形，而不是只画边框
    );
    std::vector<std::tuple<float, float>> fence = {{0, 0}, {100, 0}, {100, 100}, {0, 100}};
    std::cout << "box_area: " << forma::box_area(box) << std::endl;
    std::cout << "box_iou: " << forma::box_iou(box, box) << std::endl;
    std::cout << "intersection_box_area: " << forma::intersection_box_area(box, box) << std::endl;
    return 0;
}