// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string.h>
#include <string>
#include <vector>

#include "globalParam.hpp"
#include "number_classifier.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/matx.hpp"

NumberClassifier::NumberClassifier(
    const std::string &model_path, const std::string &label_path, const double threshold,
    const std::vector<std::string> &ignore_classes)
    : threshold(threshold), ignore_classes_(ignore_classes)
{
    this->net_ = cv::dnn::readNetFromONNX(model_path);
    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line))
    {
        class_names_.push_back(line);
    }
}