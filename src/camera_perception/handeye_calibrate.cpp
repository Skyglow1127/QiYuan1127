#include "handeye.hpp"
#include "vision_io.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <algorithm>
#include <iostream>
#include <cmath>

static cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary(const std::string& name) {
    if(name=="DICT_4X4_50") return cv::aruco::DICT_4X4_50;
    if(name=="DICT_5X5_100") return cv::aruco::DICT_5X5_100;
    throw std::runtime_error("Unsupported aruco_dictionary: "+name);
}

int main(int argc,char** argv) {
    try {
        if(argc!=3 || std::string(argv[1])=="--help") {
            std::cout<<"handeye_calibrate samples.yml output.yml\nEach sample: T_base_end plus either T_camera_target or snapshot path. Units: metres. Fixed ChArUco/chessboard, stationary robot.\n";
            return argc==2?0:1;
        }
        cv::FileStorage input(argv[1],cv::FileStorage::READ);
        if(!input.isOpened()) throw std::runtime_error("Cannot open samples file");
        std::string unit; input["translation_unit"]>>unit;
        if(unit!="m") throw std::runtime_error("translation_unit must be m");
        int cols=0,rows=0,squaresX=0,squaresY=0; double square=0,marker=0;
        std::string boardType="chessboard",dictionaryName="DICT_4X4_50";
        if(!input["board_type"].empty()) input["board_type"]>>boardType;
        input["board_columns"]>>cols; input["board_rows"]>>rows; input["square_size_m"]>>square;
        input["board_squares_x"]>>squaresX; input["board_squares_y"]>>squaresY;
        input["marker_size_m"]>>marker; input["aruco_dictionary"]>>dictionaryName;
        if(boardType!="chessboard" && boardType!="charuco") throw std::runtime_error("board_type must be chessboard or charuco");
        std::vector<cv::Matx44d> baseEnd,cameraTarget;
        std::vector<double> reprojectionErrors; double maxReprojection=0;
        for(auto sample:input["samples"]) {
            cv::Mat robot,target; sample["T_base_end"]>>robot;
            if(sample["T_camera_target"].empty()) {
                if(!std::isfinite(square) || square<=0) throw std::runtime_error("Invalid board square size");
                std::string path; sample["snapshot"]>>path;
                grasp::Camera camera; grasp::Config config;
                auto packet=grasp::loadPacket(grasp::resolve(grasp::parent(argv[1]),path),camera,config);
                cv::Mat gray; cv::cvtColor(packet->color,gray,cv::COLOR_BGR2GRAY);
                std::vector<cv::Point2f> corners;
                std::vector<cv::Point3f> board;
                cv::Mat rvec,tvec,R;
                if(boardType=="charuco") {
                    if(squaresX<4 || squaresY<4 || squaresX>30 || squaresY>30 || !std::isfinite(marker) || marker<=0 || marker>=square)
                        throw std::runtime_error("Invalid ChArUco dimensions");
                    auto dict=cv::aruco::getPredefinedDictionary(dictionary(dictionaryName));
                    auto charuco=cv::aruco::CharucoBoard::create(squaresX,squaresY,float(square),float(marker),dict);
                    std::vector<std::vector<cv::Point2f>> markerCorners; std::vector<int> markerIds;
                    cv::aruco::detectMarkers(gray,dict,markerCorners,markerIds);
                    cv::Mat charucoCorners,charucoIds;
                    int count=cv::aruco::interpolateCornersCharuco(markerCorners,markerIds,gray,charuco,charucoCorners,charucoIds,camera.K,camera.distortion);
                    int minimum=std::max(12,int(std::ceil(.55*(squaresX-1)*(squaresY-1))));
                    if(count<minimum || !cv::aruco::estimatePoseCharucoBoard(charucoCorners,charucoIds,charuco,camera.K,camera.distortion,rvec,tvec))
                        throw std::runtime_error("Insufficient ChArUco IDs in "+path);
                    corners.assign(charucoCorners.begin<cv::Point2f>(),charucoCorners.end<cv::Point2f>());
                    for(int i=0;i<charucoIds.rows;++i) board.push_back(charuco->chessboardCorners.at(size_t(charucoIds.at<int>(i))));
                } else {
                    if(cols<3 || rows<3 || cols>100 || rows>100) throw std::runtime_error("Invalid chessboard inner-corner dimensions");
                    if(!cv::findChessboardCornersSB(gray,{cols,rows},corners,cv::CALIB_CB_NORMALIZE_IMAGE|cv::CALIB_CB_EXHAUSTIVE))
                        throw std::runtime_error("Chessboard not found in "+path);
                    for(int y=0;y<rows;++y) for(int x=0;x<cols;++x) board.emplace_back(x*square,y*square,0);
                    if(!cv::solvePnP(board,corners,camera.K,camera.distortion,rvec,tvec)) throw std::runtime_error("PnP failed: "+path);
                }
                cv::Rodrigues(rvec,R); target=cv::Mat::eye(4,4,CV_64F);
                R.copyTo(target(cv::Rect(0,0,3,3))); tvec.copyTo(target(cv::Rect(3,0,1,3)));
                std::vector<cv::Point2f> projected; cv::projectPoints(board,rvec,tvec,camera.K,camera.distortion,projected);
                double error=cv::norm(corners,projected,cv::NORM_L2)/std::sqrt(corners.size());
                reprojectionErrors.push_back(error);
                maxReprojection=std::max(maxReprojection,error);
                if(error>2) throw std::runtime_error("Board reprojection RMS exceeds 2 pixels: "+path);
            } else { sample["T_camera_target"]>>target; reprojectionErrors.push_back(-1); }
            baseEnd.push_back(grasp::asTransform(robot)); cameraTarget.push_back(grasp::asTransform(target));
        }
        auto result=grasp::solveHandEye(baseEnd,cameraTarget);
        const double maxTargetTranslation=*std::max_element(result.targetTranslationErrors.begin(),result.targetTranslationErrors.end());
        const double maxTargetRotation=*std::max_element(result.targetRotationErrorsDegrees.begin(),result.targetRotationErrorsDegrees.end());
        const bool qualityPass=maxReprojection<=1.0 && result.translationRms<=.01 && result.rotationRmsDegrees<=1.0 &&
                               maxTargetTranslation<=.025 && maxTargetRotation<=2.5;
        cv::FileStorage output(argv[2],cv::FileStorage::WRITE);
        if(!output.isOpened()) throw std::runtime_error("Cannot write calibration output");
        output<<"translation_unit"<<"m"<<"verified"<<0<<"T_end_camera"<<cv::Mat(result.endCamera)
              <<"sample_count"<<int(baseEnd.size())<<"target_consistency_translation_rms_m"<<result.translationRms
              <<"target_consistency_rotation_rms_deg"<<result.rotationRmsDegrees<<"max_reprojection_rms_px"<<maxReprojection
              <<"reprojection_rms_px_per_sample"<<cv::Mat(reprojectionErrors)
              <<"target_translation_error_m_per_sample"<<cv::Mat(result.targetTranslationErrors)
              <<"target_rotation_error_deg_per_sample"<<cv::Mat(result.targetRotationErrorsDegrees)
              <<"max_target_translation_error_m"<<maxTargetTranslation<<"max_target_rotation_error_deg"<<maxTargetRotation
              <<"quality_gate_pass"<<int(qualityPass)<<"quality_review_required"<<1<<"board_type"<<boardType;
        std::cout<<"Wrote "<<argv[2]<<". Consistency RMS: "<<result.translationRms*1000<<" mm, "<<result.rotationRmsDegrees
                 <<" deg. verified=0 until independent physical validation.\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<"Calibration error: "<<e.what()<<"\n"; return 1; }
}
