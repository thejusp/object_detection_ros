#include <object_detection_ros/visionpipeline.h>

VisionPipeline::VisionPipeline(){

    module = torch::jit::load("/proj/object_detection/model/yolov5s.torchscript.pt");
	std::ifstream f("/proj/object_detection/model/coco.names");
	std::string name = "";
	while (std::getline(f, name))
	{
		classnames.push_back(name);
	}
}

VisionPipeline VisionPipeline::getObjects(std::vector<cv::Point> &p){
    p = objects;
    return *this;
};

VisionPipeline VisionPipeline::getNames(std::vector<std::string> &p){

    p = object_names;
    return *this;

};

VisionPipeline VisionPipeline::getScore(std::vector<float> &scores){

    scores = this->scores;
    return *this;

};

VisionPipeline VisionPipeline::getBoundingBoxes(std::vector<std::array<float,4>> &bboxes){

    if(bboxes.size() > 0){
        bboxes.clear();
    }

    for(std::array<float,4> box : detected_boxes){
        bboxes.push_back(box);
    }
    
    return *this;
};



VisionPipeline VisionPipeline::run(cv::Mat& color){

    clock_t start = clock();
    cv::resize(color, img, cv::Size(640, 384));
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    torch::Tensor imgTensor = torch::from_blob(img.data, {img.rows, img.cols,3},torch::kByte);
    imgTensor = imgTensor.permute({2,0,1});
    imgTensor = imgTensor.toType(torch::kFloat);
    imgTensor = imgTensor.div(255);
    imgTensor = imgTensor.unsqueeze(0);

    // preds: [?, 15120, 9]
    torch::Tensor preds = module.forward({imgTensor}).toTuple()->elements()[0].toTensor();
    std::vector<torch::Tensor> dets = non_max_suppression(preds, 0.4, 0.5);
    objects.clear();
    object_names.clear();

    if (dets.size() > 0)
    {
        // Visualize result
        detected_boxes.clear();
        scores.clear();
        for (size_t i=0; i < dets[0].sizes()[0]; ++ i)
        {
            float left      = dets[0][i][0].item().toFloat() * color.cols / 640;
            float top       = dets[0][i][1].item().toFloat() * color.rows / 384;
            float right     = dets[0][i][2].item().toFloat() * color.cols / 640;
            float bottom    = dets[0][i][3].item().toFloat() * color.rows / 384;
            float score     = dets[0][i][4].item().toFloat();
            int classID     = dets[0][i][5].item().toInt();

            cv::rectangle(color, cv::Rect(left, top, (right - left), (bottom - top)), cv::Scalar(255, 0, 0), 2);
            std::array<float, 4> bounding_box{left, left,(bottom - top),(right - left)};
            detected_boxes.push_back(bounding_box);
            scores.push_back(score);
            bounding_box[0] = left;
            bounding_box[1] = top;
            bounding_box[2] = (right - left);
            bounding_box[3] = (bottom - top);

            cv::putText(color,
                cv::format("%d->", classID) + classnames[classID] + ": " + cv::format("%.2f", score),
                cv::Point(left +  (right - left)/2, top +(bottom - top)/2 ),
                cv::FONT_HERSHEY_SIMPLEX, (right - left) / 100, cv::Scalar(255, 0, 0), 3);
            objects.push_back(cv::Point(left +  (right - left)/2, top +(bottom - top)/2 ));
            object_names.push_back(classnames[classID]);
            

        }
    }
    cv::putText(color, "FPS: " + std::to_string(int(1e7 / (clock() - start))),
        cv::Point(50, 50),
        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    
    return *this;
    
}


std::vector<torch::Tensor> VisionPipeline::non_max_suppression(torch::Tensor preds, float score_thresh=0.5, float iou_thresh=0.5)
{
        std::vector<torch::Tensor> output;
        for (size_t i=0; i < preds.sizes()[0]; ++i)
        {
            torch::Tensor pred = preds.select(0, i);
            
            // Filter by scores
            torch::Tensor scores = pred.select(1, 4) * std::get<0>( torch::max(pred.slice(1, 5, pred.sizes()[1]), 1));
            pred = torch::index_select(pred, 0, torch::nonzero(scores > score_thresh).select(1, 0));
            if (pred.sizes()[0] == 0) continue;

            // (center_x, center_y, w, h) to (left, top, right, bottom)
            pred.select(1, 0) = pred.select(1, 0) - pred.select(1, 2) / 2;
            pred.select(1, 1) = pred.select(1, 1) - pred.select(1, 3) / 2;
            pred.select(1, 2) = pred.select(1, 0) + pred.select(1, 2);
            pred.select(1, 3) = pred.select(1, 1) + pred.select(1, 3);

            // Computing scores and classes
            std::tuple<torch::Tensor, torch::Tensor> max_tuple = torch::max(pred.slice(1, 5, pred.sizes()[1]), 1);
            pred.select(1, 4) = pred.select(1, 4) * std::get<0>(max_tuple);
            pred.select(1, 5) = std::get<1>(max_tuple);

            torch::Tensor  dets = pred.slice(1, 0, 6);

            torch::Tensor keep = torch::empty({dets.sizes()[0]});
            torch::Tensor areas = (dets.select(1, 3) - dets.select(1, 1)) * (dets.select(1, 2) - dets.select(1, 0));
            std::tuple<torch::Tensor, torch::Tensor> indexes_tuple = torch::sort(dets.select(1, 4), 0, 1);
            torch::Tensor v = std::get<0>(indexes_tuple);
            torch::Tensor indexes = std::get<1>(indexes_tuple);
            int count = 0;
            while (indexes.sizes()[0] > 0)
            {
                keep[count] = (indexes[0].item().toInt());
                count += 1;

                // Computing overlaps
                torch::Tensor lefts = torch::empty(indexes.sizes()[0] - 1);
                torch::Tensor tops = torch::empty(indexes.sizes()[0] - 1);
                torch::Tensor rights = torch::empty(indexes.sizes()[0] - 1);
                torch::Tensor bottoms = torch::empty(indexes.sizes()[0] - 1);
                torch::Tensor widths = torch::empty(indexes.sizes()[0] - 1);
                torch::Tensor heights = torch::empty(indexes.sizes()[0] - 1);
                for (size_t i=0; i<indexes.sizes()[0] - 1; ++i)
                {
                    lefts[i] = std::max(dets[indexes[0]][0].item().toFloat(), dets[indexes[i + 1]][0].item().toFloat());
                    tops[i] = std::max(dets[indexes[0]][1].item().toFloat(), dets[indexes[i + 1]][1].item().toFloat());
                    rights[i] = std::min(dets[indexes[0]][2].item().toFloat(), dets[indexes[i + 1]][2].item().toFloat());
                    bottoms[i] = std::min(dets[indexes[0]][3].item().toFloat(), dets[indexes[i + 1]][3].item().toFloat());
                    widths[i] = std::max(float(0), rights[i].item().toFloat() - lefts[i].item().toFloat());
                    heights[i] = std::max(float(0), bottoms[i].item().toFloat() - tops[i].item().toFloat());
                }
                torch::Tensor overlaps = widths * heights;

                // FIlter by IOUs
                torch::Tensor ious = overlaps / (areas.select(0, indexes[0].item().toInt()) + torch::index_select(areas, 0, indexes.slice(0, 1, indexes.sizes()[0])) - overlaps);
                indexes = torch::index_select(indexes, 0, torch::nonzero(ious <= iou_thresh).select(1, 0) + 1);
            }
            keep = keep.toType(torch::kInt64);
            output.push_back(torch::index_select(dets, 0, keep.slice(0, 0, count)));
        }
        return output;
}
