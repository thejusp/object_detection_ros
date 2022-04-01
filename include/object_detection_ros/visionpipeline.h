#ifndef _VISION_PIPELINE_H
#define _VISION_PIPELINE_H

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <math.h>
#include <array>

class Object{

    public:
    cv::Point p;
    std::string name;
    Object(cv::Point p, std::string name){
        this->p = p;
        this->name = name;
    }
    ~Object(){
        delete this;
    }
};

class VisionPipeline{

    private:

    cv::Mat img;
    torch::jit::script::Module module;
    std::vector<std::string> classnames;
    std::vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh, float iou_thresh);
    std::vector<cv::Point> objects;
    std::vector<std::string> object_names;
    std::array<float, 4> bounding_box;
    std::vector<std::array<float,4>> detected_boxes;
    std::vector<float> scores;
    public: 

    VisionPipeline();
    VisionPipeline run(cv::Mat& color);
    VisionPipeline getObjects(std::vector<cv::Point> &p);
    VisionPipeline getNames(std::vector<std::string> &p);
    VisionPipeline getBoundingBoxes(std::vector<std::array<float,4>> &bboxes);
    VisionPipeline getScore(std::vector<float> &scores);

};
#endif