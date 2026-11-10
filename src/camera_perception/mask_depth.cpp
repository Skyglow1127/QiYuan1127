// Offline Qwen/SAM appearance masks evaluated against the original measured RGB-D.
// This executable shares the calibrated geometry implementation with detect.
#include "vision_io.hpp"
#include <iostream>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <openssl/evp.h>

using namespace grasp;

static std::string sha256(const std::string& path) {
    std::ifstream file(path,std::ios::binary);
    if(!file) throw std::runtime_error("Cannot hash file: "+path);
    auto ctx=std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)>(EVP_MD_CTX_new(),EVP_MD_CTX_free);
    if(!ctx || EVP_DigestInit_ex(ctx.get(),EVP_sha256(),nullptr)!=1) throw std::runtime_error("SHA256 initialization failed");
    char bytes[65536];
    while(file) {
        file.read(bytes,sizeof(bytes));
        if(EVP_DigestUpdate(ctx.get(),bytes,file.gcount())!=1) throw std::runtime_error("SHA256 update failed");
    }
    if(!file.eof()) throw std::runtime_error("File read failed while hashing");
    unsigned char digest[EVP_MAX_MD_SIZE]; unsigned int length=0;
    if(EVP_DigestFinal_ex(ctx.get(),digest,&length)!=1) throw std::runtime_error("SHA256 finalization failed");
    std::ostringstream result;
    for(unsigned int i=0;i<length;++i) result<<std::hex<<std::setw(2)<<std::setfill('0')<<int(digest[i]);
    return result.str();
}

static void checkHash(const cv::FileNode& node,const char* key,const std::string& path) {
    std::string expected; node[key]>>expected;
    if(expected.size()!=64 || sha256(path)!=expected) throw std::runtime_error(std::string("SHA256 mismatch: ")+key);
}

int main(int argc,char** argv) {
    try {
        std::string replay,configPath,proposals,output;
        for(int i=1;i<argc;++i) {
            std::string a=argv[i];
            if(a=="--help") { std::cout<<"mask_depth --replay FILE --config FILE --masks FILE.json --output DIR\nOffline only; never emits a robot motion command.\n"; return 0; }
            if(i+1>=argc) throw std::runtime_error("Missing argument value");
            if(a=="--replay") replay=argv[++i];
            else if(a=="--config") configPath=argv[++i];
            else if(a=="--masks") proposals=argv[++i];
            else if(a=="--output") output=argv[++i];
            else throw std::runtime_error("Unknown argument: "+a);
        }
        if(replay.empty()||configPath.empty()||proposals.empty()||output.empty()) throw std::runtime_error("All four arguments required");
        auto config=loadConfig(configPath); cv::setNumThreads(config.threads);
        Camera camera; auto frame=loadPacket(replay,camera,config);
        if(!depthUsable(*frame)) throw std::runtime_error("RGB-D pair or visual motion is not trusted; capture after settling");
        cv::FileStorage f(proposals,cv::FileStorage::READ);
        if(!f.isOpened()) throw std::runtime_error("Cannot open mask proposals");
        std::string source; f["source_snapshot"]>>source;
        source=resolve(parent(proposals),source);
        checkHash(f.root(),"source_snapshot_sha256",source);
        cv::FileStorage sourceMetadata(source,cv::FileStorage::READ);
        std::string colorPath,depthPath;
        sourceMetadata["color"]>>colorPath; sourceMetadata["depth"]>>depthPath;
        colorPath=resolve(parent(source),colorPath); depthPath=resolve(parent(source),depthPath);
        checkHash(f.root(),"source_color_sha256",colorPath); checkHash(f.root(),"source_depth_sha256",depthPath);
        Camera originalCamera; auto original=loadPacket(source,originalCamera,config);
        // A proposal is tied to the exact pair and calibration, not merely an image size.
        if(original->color.size()!=frame->color.size()||original->depth.size()!=frame->depth.size()||
           cv::norm(original->color,frame->color,cv::NORM_INF)!=0||cv::norm(original->depth,frame->depth,cv::NORM_INF)!=0||
           cv::norm(cv::Mat(camera.K),cv::Mat(originalCamera.K),cv::NORM_INF)!=0||
           camera.distortion.size()!=originalCamera.distortion.size()||
           cv::norm(camera.distortion,originalCamera.distortion,cv::NORM_INF)!=0||
           original->hostReceivedMs!=frame->hostReceivedMs||original->depthHostReceivedMs!=frame->depthHostReceivedMs||
           original->motion.enabled!=frame->motion.enabled||original->motion.state!=frame->motion.state||
           original->motion.stableMs!=frame->motion.stableMs||original->deviceColorUs!=frame->deviceColorUs||original->deviceDepthUs!=frame->deviceDepthUs||original->pairing!=frame->pairing)
            throw std::runtime_error("Mask proposal snapshot does not match replay RGB-D/calibration/time");
        auto items=f["objects"];
        if(!items.isSeq()||items.size()>8) throw std::runtime_error("Expected at most 8 mask objects");
        std::vector<std::string> names;
        std::vector<Detection> detections;
        for(const auto& item:items) {
            std::string label,path; item["label"]>>label; item["mask"]>>path;
            if(label.empty()||label.size()>240||path.empty()) throw std::runtime_error("Invalid mask label or path");
            path=resolve(parent(proposals),path); checkHash(item,"mask_sha256",path);
            cv::Mat mask=cv::imread(path,cv::IMREAD_UNCHANGED);
            if(mask.type()!=CV_8UC1||mask.size()!=frame->color.size()||cv::countNonZero(mask)<40||
               cv::countNonZero((mask!=0)&(mask!=255))) throw std::runtime_error("Expected aligned binary uint8 mask with >=40 pixels");
            int id=int(names.size()); names.push_back(label);
            detections.push_back({id,1.f,cv::boundingRect(mask),mask,"qwen_sam","semantic_hypothesis_only"});
            checkHash(item,"mask_sha256",path);
        }
        checkHash(f.root(),"source_snapshot_sha256",source);
        checkHash(f.root(),"source_color_sha256",colorPath); checkHash(f.root(),"source_depth_sha256",depthPath);
        Result result; result.frame=frame; result.mode="qwen_sam_depth"; result.tableRoi=config.geometry.tableRoi;
        result.maxAgeMs=config.maxExportAgeMs; auto started=Clock::now(); auto rays=makeRays(camera);
        result.plane=fitTable(frame->depth,rays,{},config.geometry);
        for(const auto& d:detections) {
            auto c=estimate(d,frame->depth,rays,result.plane,config.geometry);
            // A box/crop median may describe the table. Only retain an observed object component.
            if(c.mask.empty()) c.measured=false;
            if(c.measured) {
                double coverage=double(cv::countNonZero(c.mask))/cv::countNonZero(d.mask);
                if(c.missingBoundaryRatio>config.geometry.maxMissingBoundaryRatio) {
                    c.valid=false; c.reason="incomplete_depth_contour";
                } else if(coverage<.70) {
                    c.valid=false; c.reason="appearance_depth_disagreement";
                }
            }
            c.rgbContour=d.mask; c.rgbContourStatus="sam_appearance_estimate_only";
            if(d.box.x<=0||d.box.y<=0||d.box.br().x>=frame->color.cols||d.box.br().y>=frame->color.rows) {
                c.valid=false; c.reason="appearance_cut_by_image_edge";
            }
            if(std::find(config.excludedGraspClasses.begin(),config.excludedGraspClasses.end(),names[d.classId])!=config.excludedGraspClasses.end()) {
                c.valid=false; c.reason="class_excluded_from_grasp";
            }
            result.candidates.push_back(std::move(c));
        }
        result.geometryMs=elapsed(started); result.totalMs=result.geometryMs;
        // Offline semantic masks are never used to attach a robot base pose.
        result.baseReason="offline_semantic_proposal_not_motion_command";
        makeDirectory(output); writeResult(output+"/mask_depth_result.json",result,names,true);
        cv::imwrite(output+"/mask_depth_annotated.png",drawResult(result,names,-1,&camera));
        for(size_t i=0;i<result.candidates.size();++i) if(!result.candidates[i].mask.empty())
            cv::imwrite(output+"/measured_component_"+std::to_string(i)+".png",result.candidates[i].mask);
        std::cout<<"Measured mask analysis: "<<output<<"/mask_depth_result.json\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
