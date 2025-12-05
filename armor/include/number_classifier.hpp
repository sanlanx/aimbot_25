#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "globalParam.hpp"

class NumberClassifier
{
public:
    NumberClassifier(
        const std::string &model_path, const std::string &label_path, const double threshold,
        const std::vector<std::string> &ignore_classes = {});

    // 使用openvino的GPU进行推理
    void ovClassify(const cv::Mat &src, std::vector<UnsolvedArmor> &armors,)

    double threshold;

private:
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;
};

#endif // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
