//
// Created by wcirq on 2023/2/28.
//
#include "model.h"

Model::Model(string modelPath) {
//    modelPath = "/data/user/0/me.pqpo.smartcropper/cache/weights/model_1228.mnn";
    net = std::shared_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(modelPath.c_str()));

    //config
    MNN::ScheduleConfig config;
    int num_thread = 4;
    config.numThread = num_thread;
    MNN::BackendConfig backendConfig;
    backendConfig.precision = (MNN::BackendConfig::PrecisionMode) 2;
    config.backendConfig = &backendConfig;

    //create session
    session = net->createSession(config);

    //create input_tensor
    p_input = net->getSessionInput(session, nullptr);

    auto shape = p_input->shape();
    if (shape[0] != 1) {
        shape[0] = 1;
        net->resizeTensor(p_input, shape);
        net->resizeSession(session);
    }
    bpp = shape[1];
    size_h = shape[2];
    size_w = shape[3];
    if (bpp == 0)
        bpp = 1;
    if (size_h == 0)
        size_h = 1;
    if (size_w == 0)
        size_w = 1;
}

Model::~Model() {
    delete p_input, p_output;
    net->releaseSession(session);
    net->releaseModel();
}

cv::Mat Model::detect(cv::Mat image) {
    if (image.channels()==4)
        cv::cvtColor(image, image, cv::COLOR_BGRA2RGB);

    // 执行预处理
    {
        int width = image.cols;
        int height = image.rows;
        int channel = image.channels();

        MNN::CV::Matrix trans;
        // Set scale, from dst scale to src
        trans.setScale((float) (width - 1) / (size_w - 1), (float) (height - 1) / (size_h - 1));
        MNN::CV::ImageProcess::Config config;
        config.filterType = MNN::CV::BILINEAR;
        float mean[3] = {0.f, 0.0f, 0.0f};
        float normals[3] = {0.003921568627451f, 0.003921568627451f, 0.003921568627451f};
        ::memcpy(config.mean, mean, sizeof(mean));
        ::memcpy(config.normal, normals, sizeof(normals));
        config.sourceFormat = MNN::CV::RGB;
        config.destFormat = MNN::CV::RGB;

        std::shared_ptr<MNN::CV::ImageProcess> pretreat(MNN::CV::ImageProcess::create(config));
        pretreat->setMatrix(trans);
        pretreat->convert(image.data, width, height, image.step[0], p_input);
    }

    // 运行session
    {
        net->runSession(session);
    }

    /*解析结果*/
    MNN::Tensor nchw_input(p_input, p_input->getDimensionType());
    MNN::Tensor* nchw_output = p_output;
    p_input->copyToHostTensor(&nchw_input);

    float *result_input = new float[1*512*512*3];
    for (size_t i = 0; i < nchw_input.elementSize(); i++) {
        result_input[i] = nchw_input.host<float>()[i];
    }

    MNN::Tensor * outputTensor = net->getSessionOutput(session, "output");
    // First Create a Expr, then create Variable by the 0 index of expr
    auto output = MNN::Express::Variable::create(MNN::Express::Expr::create(outputTensor));
    if (nullptr == output->getInfo()) {
        MNN_ERROR("Alloc memory or compute size error\n");
        return cv::Mat::zeros(1, 1, CV_8UC1);
    }

    output = _Convert(output, MNN::Express::NHWC);
    auto width = output->getInfo()->dim[2];
    auto height = output->getInfo()->dim[1];
    auto channel = output->getInfo()->dim[3];

    const int humanIndex = 1;
    output = _Reshape(output, {-1, channel});
    auto kv = _TopKV2(output, MNN::Express::_Scalar<int>(1));
    // Use indice in TopKV2's C axis
    auto index = kv[1];
    // If is human, set 255, else set 0
    auto mask = _Select(_Equal(index, MNN::Express::_Scalar<int>(humanIndex)), MNN::Express::_Scalar<int>(255), MNN::Express::_Scalar<int>(0));

    std::vector<int> shape2 = mask->getTensor()->shape();
    //If need faster, use this code
    //auto mask = _Equal(index, _Scalar<int>(humanIndex)) * _Scalar<int>(255);

    mask = _Cast<uint8_t>(mask);
    const uint8_t *data = mask->readMap<uint8_t>();
    cv::Mat outImage = cv::Mat(512, 512, CV_8UC1, (uint8_t*)data);
    cv::resize(outImage, outImage, cv::Size(image.cols, image.rows), cv::INTER_LINEAR);

    return outImage;
}
