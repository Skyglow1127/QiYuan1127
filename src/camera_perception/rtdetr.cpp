#include "rtdetr.hpp"
#include <onnxruntime_cxx_api.h>
#include <cmath>
#include <stdexcept>

namespace grasp {
struct RTDetector::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING,"grasp_rtdetr"};
    Ort::SessionOptions options;
    std::unique_ptr<Ort::Session> session;
    Config config;
    std::string inputName,outputName;
    int size=0;
    explicit Impl(const Config& c):config(c) {
        options.SetIntraOpNumThreads(c.threads); options.SetInterOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        options.AddConfigEntry("session.intra_op.allow_spinning","0");
        session.reset(new Ort::Session(env,c.rtModel.c_str(),options));
        if(session->GetInputCount()!=1 || session->GetOutputCount()!=1)
            throw std::runtime_error("RT-DETR requires the fixed export from scripts/prepare_rtdetr.py");
        Ort::AllocatorWithDefaultOptions allocator;
        inputName=session->GetInputNameAllocated(0,allocator).get();
        outputName=session->GetOutputNameAllocated(0,allocator).get();
        auto shape=session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if(shape.size()!=4 || shape[0]!=1 || shape[1]!=3 || shape[2]!=shape[3] || shape[2]<128 || shape[2]>1280)
            throw std::runtime_error("RT-DETR input must be fixed [1,3,S,S]");
        size=int(shape[2]);
        if(size!=c.input) throw std::runtime_error("input_size must match the exported RT-DETR ONNX model");
    }
};
RTDetector::RTDetector(const Config& config):impl(new Impl(config)) {}
RTDetector::~RTDetector()=default;
std::vector<Detection> RTDetector::run(const cv::Mat& image,Result& timing) {
    auto start=Clock::now(); auto& m=*impl;
    cv::Mat blob=cv::dnn::blobFromImage(image,1./255,{m.size,m.size},cv::Scalar(),true,false);
    std::array<int64_t,4> shape{1,3,m.size,m.size};
    auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
    auto tensor=Ort::Value::CreateTensor<float>(memory,blob.ptr<float>(),blob.total(),shape.data(),shape.size());
    const char* input=m.inputName.c_str(); const char* output=m.outputName.c_str();
    timing.preprocessMs+=elapsed(start); start=Clock::now();
    auto values=m.session->Run(Ort::RunOptions{nullptr},&input,&tensor,1,&output,1);
    double ms=elapsed(start); timing.inferenceMs+=ms; timing.verifierMs=ms;
    auto info=values[0].GetTensorTypeAndShapeInfo(); auto dims=info.GetShape();
    if(info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || dims.size()!=3 || dims[0]!=1 || dims[2]!=6)
        throw std::runtime_error("RT-DETR output must be float32 [1,N,6]");
    const float* data=values[0].GetTensorData<float>();
    std::vector<Detection> result;
    for(int64_t i=0;i<dims[1];++i) {
        const float* d=data+6*i;
        if(!std::all_of(d,d+6,[](float x){return std::isfinite(x);}) || d[4]<m.config.rtConfidence ||
           d[5]<0 || d[5]>=80 || std::floor(d[5])!=d[5]) continue;
        auto bound=[](double value,int limit){return std::clamp(value,0.,double(limit));};
        int x1=int(bound(std::floor(d[0]*image.cols/m.size),image.cols));
        int y1=int(bound(std::floor(d[1]*image.rows/m.size),image.rows));
        int x2=int(bound(std::ceil(d[2]*image.cols/m.size),image.cols));
        int y2=int(bound(std::ceil(d[3]*image.rows/m.size),image.rows));
        if(x2<=x1 || y2<=y1) continue;
        Detection det{int(d[5]),d[4],{x1,y1,x2-x1,y2-y1}};
        det.source="rtdetr"; result.push_back(std::move(det));
    }
    // Official top-k may emit competing labels for one query. Keep NMS class-aware.
    result=classNms(result,m.config.rtConfidence,m.config.nms);
    std::sort(result.begin(),result.end(),[](const auto& a,const auto& b){return a.score>b.score;});
    if(result.size()>32) result.resize(32);
    return result;
}
}
