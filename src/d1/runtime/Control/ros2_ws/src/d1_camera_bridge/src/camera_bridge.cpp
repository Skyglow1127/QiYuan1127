// RGB-D capture adapted from the user's Camera/detect.cpp. SDK is read-only.
// Deliberately stop-and-look: no frames are exported while the arm is moving or
// joint feedback is stale. Host receipt pairing is NOT exposure synchronization.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <libobsensor/ObSensor.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>

static bool supported(OBFormat f) {
    return f==OB_FORMAT_BGR || f==OB_FORMAT_RGB || f==OB_FORMAT_MJPG || f==OB_FORMAT_YUYV || f==OB_FORMAT_YUY2 || f==OB_FORMAT_UYVY;
}
static bool depthFormat(OBFormat f) {
    return f==OB_FORMAT_Y16 || f==OB_FORMAT_Z16 || f==OB_FORMAT_Y10 || f==OB_FORMAT_Y11 || f==OB_FORMAT_Y12 || f==OB_FORMAT_Y14;
}
static cv::Mat decode(const std::shared_ptr<ob::ColorFrame>& frame) {
    auto fmt=frame->format(); int w=frame->width(),h=frame->height(); cv::Mat image;
    if(fmt==OB_FORMAT_MJPG) return cv::imdecode(cv::Mat(1,frame->dataSize(),CV_8U,frame->data()),cv::IMREAD_COLOR);
    size_t size=size_t(w)*h*((fmt==OB_FORMAT_RGB || fmt==OB_FORMAT_BGR)?3:2);
    if(frame->dataSize()<size) throw std::runtime_error("Truncated color frame");
    if(fmt==OB_FORMAT_BGR) return cv::Mat(h,w,CV_8UC3,frame->data()).clone();
    if(fmt==OB_FORMAT_RGB) cv::cvtColor(cv::Mat(h,w,CV_8UC3,frame->data()),image,cv::COLOR_RGB2BGR);
    else cv::cvtColor(cv::Mat(h,w,CV_8UC2,frame->data()),image,fmt==OB_FORMAT_UYVY?cv::COLOR_YUV2BGR_UYVY:cv::COLOR_YUV2BGR_YUY2);
    return image;
}

class Bridge : public rclcpp::Node {
    struct State { double stamp; std::array<double,6> q; };
    std::deque<State> history_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_, depth_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    std::vector<std::string> names_;
public:
    std::string frame;
    Bridge():Node("d1_stationary_camera_bridge") {
        frame=declare_parameter<std::string>("frame_id","camera_color_optical_frame");
        names_=declare_parameter<std::vector<std::string>>("arm_joints",{"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"});
        if(names_.size()!=6) throw std::runtime_error("Need six arm joint names");
        rgb_=create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw",rclcpp::SensorDataQoS());
        depth_=create_publisher<sensor_msgs::msg::Image>("/camera/aligned_depth_to_color/image_raw",rclcpp::SensorDataQoS());
        info_=create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info",rclcpp::SensorDataQoS());
        status_=create_publisher<std_msgs::msg::String>("/d1/camera/status",10);
        joints_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr m){
            if(m->name.size()!=m->position.size()) return;
            State s; s.stamp=rclcpp::Time(m->header.stamp).seconds();
            double age=now().seconds()-s.stamp;
            if(s.stamp<=0 || age<0 || age>1 || (!history_.empty() && s.stamp<=history_.back().stamp)) return;
            for(size_t i=0;i<6;++i) {
                auto it=std::find(m->name.begin(),m->name.end(),names_[i]);
                if(it==m->name.end()) return;
                s.q[i]=m->position[size_t(it-m->name.begin())];
                if(!std::isfinite(s.q[i])) return;
            }
            history_.push_back(s);
            while(!history_.empty() && history_.front().stamp<s.stamp-3) history_.pop_front();
        });
    }
    void status(const std::string& text) { std_msgs::msg::String m; m.data=text; status_->publish(m); }
    bool stationary(double captureSeconds) {
        if(history_.size()<3 || now().seconds()-history_.back().stamp>1 ||
           history_.back().stamp-history_.front().stamp<.5 || captureSeconds<history_.front().stamp) return false;
        for(size_t i=0;i<6;++i) {
            double lo=1e9,hi=-1e9;
            for(auto& s:history_) { lo=std::min(lo,s.q[i]);hi=std::max(hi,s.q[i]); }
            if(hi-lo>.001) return false;
        }
        return true;
    }
    void publish(const cv::Mat& color,const cv::Mat& depth,const OBCameraParam& param,double receiptSeconds) {
        std_msgs::msg::Header header;
        header.stamp=rclcpp::Time(int64_t(receiptSeconds*1e9)); header.frame_id=frame;
        sensor_msgs::msg::CameraInfo info; info.header=header; info.width=color.cols;info.height=color.rows;
        auto k=param.rgbIntrinsic;auto d=param.rgbDistortion;
        info.distortion_model="rational_polynomial";
        info.k={k.fx,0,k.cx,0,k.fy,k.cy,0,0,1};
        info.d={d.k1,d.k2,d.p1,d.p2,d.k3,d.k4,d.k5,d.k6};
        info.r={1,0,0,0,1,0,0,0,1};info.p={k.fx,0,k.cx,0,0,k.fy,k.cy,0,0,0,1,0};
        rgb_->publish(*cv_bridge::CvImage(header,"bgr8",color).toImageMsg());
        depth_->publish(*cv_bridge::CvImage(header,"32FC1",depth).toImageMsg());
        info_->publish(info);
    }
};

int main(int argc,char** argv) {
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Bridge>();
    try {
        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_ERROR);
        ob::Context context; auto devices=context.queryDeviceList();
        if(!devices->deviceCount()) throw std::runtime_error("No Orbbec device found");
        if(devices->deviceCount()!=1) throw std::runtime_error("Multiple cameras: explicit serial selection required");
        auto device=devices->getDevice(0);ob::Pipeline pipe(device);
        RCLCPP_INFO(node->get_logger(),"Camera: %s",device->getDeviceInfo()->name());
        auto profiles=pipe.getStreamProfileList(OB_SENSOR_COLOR);
        std::vector<std::shared_ptr<ob::VideoStreamProfile>> colors;
        for(uint32_t i=0;i<profiles->count();++i) {
            auto p=profiles->getProfile(i)->as<ob::VideoStreamProfile>();
            if(supported(p->format())) colors.push_back(p);
        }
        auto score=[](const auto& p){return std::abs(int(p->width())-640)+std::abs(int(p->height())-480)+20*std::abs(int(p->fps())-30);};
        std::stable_sort(colors.begin(),colors.end(),[&](const auto& a,const auto& b){return score(a)<score(b);});
        std::shared_ptr<ob::VideoStreamProfile> colorProfile,depthProfile;
        OBAlignMode alignment=ALIGN_DISABLE;
        for(auto& color:colors) {
            for(auto mode:{ALIGN_D2C_HW_MODE,ALIGN_D2C_SW_MODE}) {
                try {
                    auto depths=pipe.getD2CDepthProfileList(color,mode);
                    for(uint32_t i=0;i<depths->count();++i) {
                        auto d=depths->getProfile(i)->as<ob::VideoStreamProfile>();
                        if(depthFormat(d->format()) && d->fps()==color->fps()) {colorProfile=color;depthProfile=d;alignment=mode;break;}
                    }
                } catch(const ob::Error&) {}
                if(colorProfile) break;
            }
            if(colorProfile) break;
        }
        if(!colorProfile) throw std::runtime_error("No supported D2C RGB-D profile");
        auto config=std::make_shared<ob::Config>();
        config->enableStream(colorProfile);config->enableStream(depthProfile);
        config->setAlignMode(alignment);config->setDepthScaleRequire(true);
        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE);
        bool deviceSync=true;
        try {pipe.enableFrameSync();} catch(const ob::Error&) {deviceSync=false;}
        pipe.start(config);
        struct Stop {ob::Pipeline& pipe;~Stop(){try{pipe.stop();}catch(...){}}} stop{pipe};
        auto params=pipe.getCameraParam();
        if(params.isMirrored) throw std::runtime_error("Mirrored camera calibration unsupported");
        std::deque<std::shared_ptr<ob::ColorFrame>> buffer;
        uint64_t used=0;
        auto last=std::chrono::steady_clock::now();
        while(rclcpp::ok()) {
            rclcpp::spin_some(node);
            auto frames=pipe.waitForFrames(100);
            if(!frames || !frames->colorFrame() || !frames->depthFrame()) {
                if(std::chrono::steady_clock::now()-last>std::chrono::seconds(5)) throw std::runtime_error("Camera frame timeout after 5 seconds");
                continue;
            }
            last=std::chrono::steady_clock::now();
            auto color=frames->colorFrame();auto depth=frames->depthFrame();
            if(!deviceSync) {
                if(buffer.empty() || buffer.back()->systemTimeStamp()!=color->systemTimeStamp()) buffer.push_back(color);
                while(buffer.size()>8) buffer.pop_front();
                double best=1e99;
                for(auto& c:buffer) {
                    double delta=std::abs(double(c->systemTimeStamp())-double(depth->systemTimeStamp()));
                    if(c->systemTimeStamp()>used && delta<best){best=delta;color=c;}
                }
                if(best>25) {node->status("rejected: RGB-D host receipt skew >25ms");continue;}
            } else if(std::abs(double(color->timeStampUs())-double(depth->timeStampUs()))>25000) {
                node->status("rejected: RGB-D device timestamps disagree");continue;
            }
            if(color->systemTimeStamp()<=used) continue;
            used=color->systemTimeStamp();
            double received=color->systemTimeStamp()/1000.,depthReceived=depth->systemTimeStamp()/1000.;
            double age=node->now().seconds()-std::min(received,depthReceived);
            if(age<0 || age>.2 || std::abs(received-depthReceived)>.025) {node->status("rejected: stale frames or system clock mismatch");continue;}
            if(!node->stationary(std::min(received,depthReceived))) {node->status("waiting: three fresh stationary arm feedback samples");continue;}
            cv::Mat image=decode(color);
            if(image.empty() || int(depth->width())!=image.cols || int(depth->height())!=image.rows ||
               params.rgbIntrinsic.width!=image.cols || params.rgbIntrinsic.height!=image.rows)
                throw std::runtime_error("D2C/intrinsic image resolution mismatch");
            if(!depthFormat(depth->format()) || depth->dataSize()<size_t(image.cols)*image.rows*2)
                throw std::runtime_error("Invalid/truncated depth frame");
            float scale=depth->getValueScale();
            if(!std::isfinite(scale) || scale<=0) throw std::runtime_error("Invalid depth unit");
            cv::Mat metres;cv::Mat(image.rows,image.cols,CV_16U,depth->data()).convertTo(metres,CV_32F,scale/1000.);
            node->publish(image,metres,params,received);
            node->status(deviceSync?"ready: stationary; device-paired RGB-D, host-receipt ROS stamp":"ready: stationary; approximate host-receipt pairing, NOT exposure synchronized");
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),"%s",e.what());rclcpp::shutdown();return 2;
    } catch(const ob::Error& e) {
        RCLCPP_ERROR(node->get_logger(),"Orbbec: %s",e.getMessage());rclcpp::shutdown();return 2;
    }
    rclcpp::shutdown();return 0;
}
