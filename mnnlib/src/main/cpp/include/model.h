//
// Created by wcirq on 2023/2/28.
//

#ifndef SMARTONLYCROPPERPRO_MODEL_H
#define SMARTONLYCROPPERPRO_MODEL_H
#include <iostream>
#include <Interpreter.hpp>
#include <MNNDefine.h>
#include <Tensor.hpp>
#include <ImageProcess.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <android/bitmap.h>
#include <ImageProcess.hpp>
#include <expr/Expr.hpp>
#include <expr/ExprCreator.hpp>
#include <AutoTime.hpp>
#include <Interpreter.hpp>

using namespace std;

class Model{
public:
    Model(string modelPath);
    virtual ~Model();
    cv::Mat detect(cv::Mat image);

private:
    std::shared_ptr<MNN::Interpreter> net = nullptr;

    MNN::Session *session = nullptr;

    MNN::Tensor *p_input = nullptr;
    MNN::Tensor *p_output = nullptr;

    int size_w   = 512;
    int size_h   = 512;
    int bpp      = 3;

};


#endif //SMARTONLYCROPPERPRO_MODEL_H
