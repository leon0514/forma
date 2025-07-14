// ===================================================================================
//
//  test_forma.cpp
//
//  Unit tests for the 'forma' geometry library.
//  This file contains a simple assertion framework and a comprehensive test suite
//  to validate the functionality of all public functions in the 'forma' library.
//
//  How to use:
//  1. Compile the 'forma' library implementation into an object file or static library.
//  2. Compile this test file and link it with the 'forma' library and its
//     dependencies (OpenCV, Boost.Geometry).
//
//  Example compilation:
//  g++ -c forma.cpp -o forma.o `pkg-config --cflags opencv4`
//  g++ test_forma.cpp forma.o -o run_tests `pkg-config --cflags --libs opencv4` -lboost_system
//
// ===================================================================================

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <tuple>

// Header for the library being tested
#include "forma/forma.hpp"

// Required for test data setup
#include "opencv2/opencv.hpp"

// ====================================================================
//                 Simple Assertion Framework
// ====================================================================

#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << "\n  Assertion failed at " << __FILE__ << ":" << __LINE__ \
                      << "\n  Condition: " #condition " was false." << std::endl; \
            throw std::runtime_error("Test case failed"); \
        } \
    } while (0)

#define ASSERT_FALSE(condition) ASSERT_TRUE(!(condition))

#define ASSERT_NEAR(val1, val2, tolerance) \
    do { \
        if (std::abs((val1) - (val2)) > (tolerance)) { \
            std::cerr << "\n  Assertion failed at " << __FILE__ << ":" << __LINE__ \
                      << "\n  Values are not near. Condition: std::abs(" #val1 " - " #val2 ") <= " #tolerance \
                      << "\n    Actual difference: " << std::abs((val1) - (val2)) \
                      << "\n    Expected value: " << (val2) \
                      << "\n    Actual value:   " << (val1) << std::endl; \
            throw std::runtime_error("Test case failed"); \
        } \
    } while (0)

// ====================================================================
//                 Test Fixture Data Structure
// ====================================================================

struct FormaTestData {
    const float FLOAT_TOLERANCE = 1e-5f;

    // Member variables for test data
    object::Box box1_, box2_, box_no_overlap_, box_contained_, box_intersecting_fence_;
    object::Segmentation seg1_, seg2_, seg_contained_, seg_intersecting_fence_, seg_outside_fence_;
    std::vector<std::tuple<float, float>> fence_;

    // Constructor initializes all test data
    FormaTestData() {
        // --- Box Data ---
        box1_ = object::Box(10, 10, 30, 30);      // Area 20*20 = 400
        box2_ = object::Box(20, 20, 40, 40);      // Area 20*20 = 400
        // Intersection: (20, 20, 30, 30), Area 10*10 = 100
        // Union: 400 + 400 - 100 = 700

        box_no_overlap_ = object::Box(100, 100, 120, 120);
        box_contained_ = object::Box(15, 15, 25, 25);      // Fully inside box1_, Area 10*10=100
        box_intersecting_fence_ = object::Box(40, 40, 60, 60); // Intersects with fence_

        // --- Fence Data ---
        // A 50x50 square, Area 2500
        fence_ = { {0.0f, 0.0f}, {50.0f, 0.0f}, {50.0f, 50.0f}, {0.0f, 50.0f} };

        // --- Mask Data (on a 100x100 canvas) ---
        cv::Mat mat_template = cv::Mat::zeros(100, 100, CV_8UC1);

        // seg1_ corresponds to box1_
        seg1_.mask = mat_template.clone();
        cv::rectangle(seg1_.mask, cv::Point(10, 10), cv::Point(30, 30), cv::Scalar(255), cv::FILLED); // Area 400

        // seg2_ corresponds to box2_
        seg2_.mask = mat_template.clone();
        cv::rectangle(seg2_.mask, cv::Point(20, 20), cv::Point(40, 40), cv::Scalar(255), cv::FILLED); // Area 400

        // seg_contained_ corresponds to box_contained_
        seg_contained_.mask = mat_template.clone();
        cv::rectangle(seg_contained_.mask, cv::Point(15, 15), cv::Point(25, 25), cv::Scalar(255), cv::FILLED); // Area 100

        // seg_intersecting_fence_ corresponds to box_intersecting_fence_
        seg_intersecting_fence_.mask = mat_template.clone();
        cv::rectangle(seg_intersecting_fence_.mask, cv::Point(40, 40), cv::Point(60, 60), cv::Scalar(255), cv::FILLED);

        // seg_outside_fence_ corresponds to box_no_overlap_
        seg_outside_fence_.mask = mat_template.clone();
        cv::rectangle(seg_outside_fence_.mask, cv::Point(100, 100), cv::Point(120, 120), cv::Scalar(255), cv::FILLED);
    }
};

// ====================================================================
//                    Individual Test Case Functions
// ====================================================================

// --- Tests for Basic Area and Intersection ---

void test_BoxArea() {
    FormaTestData data;
    ASSERT_NEAR(forma::box_area(data.box1_), 400.0f, data.FLOAT_TOLERANCE);
}

void test_IntersectionBoxArea() {
    FormaTestData data;
    ASSERT_NEAR(forma::intersection_box_area(data.box1_, data.box2_), 100.0f, data.FLOAT_TOLERANCE);
    ASSERT_NEAR(forma::intersection_box_area(data.box1_, data.box_no_overlap_), 0.0f, data.FLOAT_TOLERANCE);
}

void test_MaskArea() {
    FormaTestData data;
    // Area is (30-10) * (30-10) = 400 pixels
    ASSERT_NEAR(forma::mask_area(data.seg1_), 400.0f, data.FLOAT_TOLERANCE);
}

void test_IntersectionMaskArea() {
    FormaTestData data;
    ASSERT_NEAR(forma::intersection_mask_area(data.seg1_, data.seg2_), 100.0f, data.FLOAT_TOLERANCE);
}

// --- Tests for IoU and Ratios ---

void test_BoxIou_Overlap() {
    FormaTestData data;
    float expected = 100.0f / 700.0f;
    ASSERT_NEAR(forma::box_iou(data.box1_, data.box2_), expected, data.FLOAT_TOLERANCE);
}

void test_BoxIou_Contained() {
    FormaTestData data;
    // Intersection = 100, Union = 400. IoU = 100/400 = 0.25
    float expected = 100.0f / 400.0f;
    ASSERT_NEAR(forma::box_iou(data.box1_, data.box_contained_), expected, data.FLOAT_TOLERANCE);
}

void test_IntersectionOverMinBoxRatio_Contained() {
    FormaTestData data;
    // Intersection 100, min_area(box_contained_) 100. Ratio = 1.0
    ASSERT_NEAR(forma::intersection_over_min_box_ratio(data.box1_, data.box_contained_), 1.0f, data.FLOAT_TOLERANCE);
}

void test_MaskIou_Overlap() {
    FormaTestData data;
    float expected = 100.0f / 700.0f;
    ASSERT_NEAR(forma::mask_iou(data.seg1_, data.seg2_), expected, data.FLOAT_TOLERANCE);
}

void test_IntersectionOverMinMaskRatio_Contained() {
    FormaTestData data;
    // Intersection 100, min_area(seg_contained_) 100. Ratio = 1.0
    ASSERT_NEAR(forma::intersection_over_min_mask_ratio(data.seg1_, data.seg_contained_), 1.0f, data.FLOAT_TOLERANCE);
}

void test_BoxMaskIou_Identical() {
    FormaTestData data;
    // box1 and seg1 are identical. IoU should be 1.0
    ASSERT_NEAR(forma::box_mask_iou(data.box1_, data.seg1_), 1.0f, data.FLOAT_TOLERANCE);
}

// --- Tests for Point-in-Shape Relationships ---

void test_PointInBox() {
    FormaTestData data;
    ASSERT_TRUE(forma::point_in_box(object::PosePoint(20, 20, 0), data.box1_));  // Inside
    ASSERT_TRUE(forma::point_in_box(object::PosePoint(10, 20, 0), data.box1_));  // On Edge
    ASSERT_FALSE(forma::point_in_box(object::PosePoint(9, 20, 0), data.box1_)); // Outside
}

void test_PointInMask() {
    FormaTestData data;
    ASSERT_TRUE(forma::point_in_mask(object::PosePoint(15, 15, 0), data.seg1_)); // Inside
    ASSERT_FALSE(forma::point_in_mask(object::PosePoint(5, 5, 0), data.seg1_));  // Outside
    ASSERT_FALSE(forma::point_in_mask(object::PosePoint(200, 200, 0), data.seg1_)); // Out of bounds
}

void test_PointInFence() {
    FormaTestData data;
    ASSERT_TRUE(forma::point_in_fence(object::PosePoint(25, 25, 0), data.fence_));  // Inside
    ASSERT_TRUE(forma::point_in_fence(object::PosePoint(0, 25, 0), data.fence_));   // On Edge
    ASSERT_FALSE(forma::point_in_fence(object::PosePoint(51, 51, 0), data.fence_)); // Outside
}

// --- Tests for Shape-in-Fence Relationships ---

void test_BoxInFence() {
    FormaTestData data;
    ASSERT_TRUE(forma::box_in_fence(data.box1_, data.fence_));                   // Fully inside
    ASSERT_FALSE(forma::box_in_fence(data.box_intersecting_fence_, data.fence_)); // Intersecting
    ASSERT_FALSE(forma::box_in_fence(data.box_no_overlap_, data.fence_));        // Fully outside
}

void test_MaskInFence() {
    FormaTestData data;
    ASSERT_TRUE(forma::mask_in_fence(data.seg1_, data.fence_));                   // Fully inside
    ASSERT_FALSE(forma::mask_in_fence(data.seg_intersecting_fence_, data.fence_)); // Intersecting
    ASSERT_FALSE(forma::mask_in_fence(data.seg_outside_fence_, data.fence_));     // Fully outside
}

// --- Tests for Shape-Fence Geometric Calculations ---

void test_IntersectionBoxFenceArea() {
    FormaTestData data;
    // Intersection of box(40,40,60,60) and fence(0,0,50,50) is (40,40,50,50), area 100
    ASSERT_NEAR(forma::intersection_box_fence_area(data.box_intersecting_fence_, data.fence_), 100.0f, data.FLOAT_TOLERANCE);
}

void test_IntersectionMaskFenceArea() {
    FormaTestData data;
    // Geometric intersection of mask at (40,40,60,60) and fence (0,0,50,50)
    // is a polygon of area 100. Allow slightly larger tolerance for geometry conversion.
    ASSERT_NEAR(forma::intersection_mask_fence_area(data.seg_intersecting_fence_, data.fence_), 100.0f, 1.0f);
}

void test_BoxFenceIou() {
    FormaTestData data;
    // Box area 400, Fence area 2500, Intersection 100
    // Union = 400 + 2500 - 100 = 2800. IoU = 100 / 2800
    float expected = 100.0f / 2800.0f;
    ASSERT_NEAR(forma::box_fence_iou(data.box_intersecting_fence_, data.fence_), expected, data.FLOAT_TOLERANCE);
}

// ====================================================================
//                 Test Runner
// ====================================================================

struct TestCase {
    std::string name;
    std::function<void()> func;
};

int main() {
    std::vector<TestCase> tests_to_run = {
        // Register all test cases here
        {"BoxArea", &test_BoxArea},
        {"IntersectionBoxArea", &test_IntersectionBoxArea},
        {"MaskArea", &test_MaskArea},
        {"IntersectionMaskArea", &test_IntersectionMaskArea},
        {"BoxIou_Overlap", &test_BoxIou_Overlap},
        {"BoxIou_Contained", &test_BoxIou_Contained},
        {"IntersectionOverMinBoxRatio_Contained", &test_IntersectionOverMinBoxRatio_Contained},
        {"MaskIou_Overlap", &test_MaskIou_Overlap},
        {"IntersectionOverMinMaskRatio_Contained", &test_IntersectionOverMinMaskRatio_Contained},
        {"BoxMaskIou_Identical", &test_BoxMaskIou_Identical},
        {"PointInBox", &test_PointInBox},
        {"PointInMask", &test_PointInMask},
        {"PointInFence", &test_PointInFence},
        {"BoxInFence", &test_BoxInFence},
        {"MaskInFence", &test_MaskInFence},
        {"IntersectionBoxFenceArea", &test_IntersectionBoxFenceArea},
        {"IntersectionMaskFenceArea", &test_IntersectionMaskFenceArea},
        {"BoxFenceIou", &test_BoxFenceIou},
    };

    int tests_passed = 0;
    std::cout << "==================== Running Forma Tests ====================" << std::endl;

    for (const auto& test : tests_to_run) {
        std::cout << "[ RUN      ] " << test.name << std::endl;
        try {
            test.func();
            tests_passed++;
            std::cout << "[       OK ] " << test.name << std::endl;
        } catch (const std::exception& e) {
            // Error details are printed by the assertion macros
            std::cout << "[   FAILED ] " << test.name << std::endl;
        }
        std::cout << "--------------------------------------------------------" << std::endl;
    }

    int total_tests = tests_to_run.size();
    int tests_failed = total_tests - tests_passed;

    std::cout << "==================== Test Summary ====================" << std::endl;
    std::cout << "Total tests run: " << total_tests << std::endl;
    std::cout << "[  PASSED  ] " << tests_passed << " tests." << std::endl;
    if (tests_failed > 0) {
        std::cout << "[  FAILED  ] " << tests_failed << " tests." << std::endl;
    }
    std::cout << "======================================================" << std::endl;

    return (tests_failed > 0) ? 1 : 0; // Return non-zero on failure
}