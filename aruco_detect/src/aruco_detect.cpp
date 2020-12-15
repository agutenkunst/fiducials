/*
 * Copyright (c) 2017-20, Ubiquity Robotics Inc., Austin Hendrix
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
//#include <dynamic_reconfigure/server.h>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "fiducial_msgs/msg/fiducial.hpp"
#include "fiducial_msgs/msg/fiducial_array.hpp"
#include "fiducial_msgs/msg/fiducial_transform.hpp"
#include "fiducial_msgs/msg/fiducial_transform_array.hpp"
//#include "aruco_detect/DetectorParamsConfig.h"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <list>
#include <string>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace cv;

class FiducialsNode {
  private:
    rclcpp::Publisher<fiducial_msgs::msg::FiducialArray>::SharedPtr vertices_pub;
    rclcpp::Publisher<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr pose_pub;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ignore_sub;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;
    tf2_ros::TransformBroadcaster broadcaster;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_enable_detections;

    // if set, we publish the images that contain fiducials
    bool publish_images;
    bool enable_detections;

    double fiducial_len;

    bool doPoseEstimation;
    bool haveCamInfo;
    bool publishFiducialTf;

    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    int frameNum;
    std::string frameId;
    std::vector<int> ignoreIds;
    std::map<int, double> fiducialLens;
    rclcpp::Node::SharedPtr nh;
    rclcpp::Node::SharedPtr pnh = std::make_shared<rclcpp::Node>("~");

    image_transport::Publisher image_pub;

    cv::Ptr<aruco::DetectorParameters> detectorParams;
    cv::Ptr<aruco::Dictionary> dictionary;

    void handleIgnoreString(const std::string& str);

    void estimatePoseSingleMarkers(const vector<int> &ids,
                                   const vector<vector<Point2f > >&corners,
                                   float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                   vector<double>& reprojectionError);


    void ignoreCallback(const std_msgs::msg::String::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    //void configCallback(aruco_detect::DetectorParamsConfig &config, uint32_t level);

    bool enableDetectionsCallback(const std_srvs::srv::SetBool::Request &req,
                                  std_srvs::srv::SetBool::Response &res);

    //dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig> configServer;
    //dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig>::CallbackType callbackType;

  public:
    FiducialsNode();
};


/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(float markerLength, vector<Point3f>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
static double calcFiducialArea(const std::vector<cv::Point2f> &pts)
{
    const Point2f &p0 = pts.at(0);
    const Point2f &p1 = pts.at(1);
    const Point2f &p2 = pts.at(2);
    const Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

// estimate reprojection error
static double getReprojectionError(const vector<Point3f> &objectPoints,
                            const vector<Point2f> &imagePoints,
                            const Mat &cameraMatrix, const Mat  &distCoeffs,
                            const Vec3d &rvec, const Vec3d &tvec) {

    vector<Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(double)objectPoints.size();
    return rerror;
}

void FiducialsNode::estimatePoseSingleMarkers(const vector<int> &ids,
                                const vector<vector<Point2f > >&corners,
                                float markerLength,
                                const cv::Mat &cameraMatrix,
                                const cv::Mat &distCoeffs,
                                vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    vector<Point3f> markerObjPoints;
    int nMarkers = (int)corners.size();
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // for each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {
       double fiducialSize = markerLength;

       std::map<int, double>::iterator it = fiducialLens.find(ids[i]);
       if (it != fiducialLens.end()) {
          fiducialSize = it->second;
       }

       getSingleMarkerObjectPoints(fiducialSize, markerObjPoints);
       cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
                    rvecs[i], tvecs[i]);

       reprojectionError[i] =
          getReprojectionError(markerObjPoints, corners[i],
                               cameraMatrix, distCoeffs,
                               rvecs[i], tvecs[i]);
    }
}

// void FiducialsNode::configCallback(aruco_detect::DetectorParamsConfig & config, uint32_t level)
// {
//     /* Don't load initial config, since it will overwrite the rosparam settings */
//     if (level == 0xFFFFFFFF) {
//         return;
//     }

//     detectorParams->adaptiveThreshConstant = config.adaptiveThreshConstant;
//     detectorParams->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
//     detectorParams->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
//     detectorParams->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
//     detectorParams->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
//     detectorParams->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
//     detectorParams->cornerRefinementWinSize = config.cornerRefinementWinSize;
// #if CV_MINOR_VERSION==2 and CV_MAJOR_VERSION==3
//     detectorParams->doCornerRefinement = config.doCornerRefinement;
// #else
//     if (config.doCornerRefinement) {
//        if (config.cornerRefinementSubpix) {
//          detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
//        }
//        else {
//          detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
//        }
//     }
//     else {
//        detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_NONE;
//     }
// #endif
//     detectorParams->errorCorrectionRate = config.errorCorrectionRate;
//     detectorParams->minCornerDistanceRate = config.minCornerDistanceRate;
//     detectorParams->markerBorderBits = config.markerBorderBits;
//     detectorParams->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
//     detectorParams->minDistanceToBorder = config.minDistanceToBorder;
//     detectorParams->minMarkerDistanceRate = config.minMarkerDistanceRate;
//     detectorParams->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
//     detectorParams->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
//     detectorParams->minOtsuStdDev = config.minOtsuStdDev;
//     detectorParams->perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell;
//     detectorParams->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
//     detectorParams->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;
// }

void FiducialsNode::ignoreCallback(const std_msgs::msg::String::SharedPtr msg)
{
    ignoreIds.clear();
    //pnh.declare_parameter("ignore_fiducials", msg.data); // TODO ROS2 migrate
    handleIgnoreString(msg->data);
}

void FiducialsNode::camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (haveCamInfo) {
        return;
    }

    if (msg->k != std::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) = msg->k[i*3+j];
            }
        }

        for (int i=0; i<5; i++) {
            distortionCoeffs.at<double>(0,i) = msg->d[i];
        }

        haveCamInfo = true;
        frameId = msg->header.frame_id;
    }
    else {
        RCLCPP_WARN(nh->get_logger(), "%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

void FiducialsNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    if (enable_detections == false) {
        return; //return without doing anything
    }

    RCLCPP_INFO(nh->get_logger(), "Got image");
    frameNum++;

    cv_bridge::CvImagePtr cv_ptr;

    fiducial_msgs::msg::FiducialTransformArray fta;
    fta.header.stamp = msg->header.stamp;
    fta.header.frame_id = frameId;

    fiducial_msgs::msg::FiducialArray fva;
    fva.header.stamp = msg->header.stamp;
    fva.header.frame_id = frameId;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        vector <int>  ids;
        vector <vector <Point2f> > corners, rejected;
        vector <Vec3d>  rvecs, tvecs;

        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);
        RCLCPP_INFO(nh->get_logger(), "Detected %d markers", (int)ids.size());

        for (size_t i=0; i<ids.size(); i++) {
	    if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
	        RCLCPP_INFO(nh->get_logger(), "Ignoring id %d", ids[i]);
	        continue;
	    }
            fiducial_msgs::msg::Fiducial fid;
            fid.fiducial_id = ids[i];

            fid.x0 = corners[i][0].x;
            fid.y0 = corners[i][0].y;
            fid.x1 = corners[i][1].x;
            fid.y1 = corners[i][1].y;
            fid.x2 = corners[i][2].x;
            fid.y2 = corners[i][2].y;
            fid.x3 = corners[i][3].x;
            fid.y3 = corners[i][3].y;
            fva.fiducials.push_back(fid);
        }

        vertices_pub->publish(fva);

        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        if (doPoseEstimation) {
            if (!haveCamInfo) {
                if (frameNum > 5) {
                    RCLCPP_ERROR(nh->get_logger(),"No camera intrinsics");
                }
                return;
            }

            vector <double>reprojectionError;
            estimatePoseSingleMarkers(ids, corners, (float)fiducial_len,
                                      cameraMatrix, distortionCoeffs,
                                      rvecs, tvecs,
                                      reprojectionError);

            for (size_t i=0; i<ids.size(); i++) {
                aruco::drawAxis(cv_ptr->image, cameraMatrix, distortionCoeffs,
                                rvecs[i], tvecs[i], (float)fiducial_len);

                RCLCPP_INFO(nh->get_logger(), "Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
                         tvecs[i][0], tvecs[i][1], tvecs[i][2],
                         rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
                    RCLCPP_INFO(nh->get_logger(), "Ignoring id %d", ids[i]);
                    continue;
                }

                double angle = norm(rvecs[i]);
                Vec3d axis = rvecs[i] / angle;
                RCLCPP_INFO(nh->get_logger(), "angle %f axis %f %f %f",
                         angle, axis[0], axis[1], axis[2]);

                fiducial_msgs::msg::FiducialTransform ft;
                ft.fiducial_id = ids[i];

                ft.transform.translation.x = tvecs[i][0];
                ft.transform.translation.y = tvecs[i][1];
                ft.transform.translation.z = tvecs[i][2];

                tf2::Quaternion q;
                q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

                ft.transform.rotation.w = q.w();
                ft.transform.rotation.x = q.x();
                ft.transform.rotation.y = q.y();
                ft.transform.rotation.z = q.z();

                ft.fiducial_area = calcFiducialArea(corners[i]);
                ft.image_error = reprojectionError[i];

                // Convert image_error (in pixels) to object_error (in meters)
                ft.object_error =
                    (reprojectionError[i] / dist(corners[i][0], corners[i][2])) *
                    (norm(tvecs[i]) / fiducial_len);

                fta.transforms.push_back(ft);

                // Publish tf for the fiducial relative to the camera
                if (publishFiducialTf) {
                    geometry_msgs::msg::TransformStamped ts;
                    ts.transform = ft.transform;
                    ts.header.frame_id = frameId;
                    ts.header.stamp = msg->header.stamp;
                    ts.child_frame_id = "fiducial_" + std::to_string(ft.fiducial_id);
                    broadcaster.sendTransform(ts);
                }
            }
            pose_pub->publish(fta);
        }

        if (publish_images) {
	    image_pub.publish(cv_ptr->toImageMsg());
        }
    }
    catch(cv_bridge::Exception & e) {
        RCLCPP_ERROR(nh->get_logger(),"cv_bridge exception: %s", e.what());
    }
    catch(cv::Exception & e) {
        RCLCPP_ERROR(nh->get_logger(),"cv exception: %s", e.what());
    }
}

void FiducialsNode::handleIgnoreString(const std::string& str)
{
    /*
    ignogre fiducials can take comma separated list of individual
    fiducial ids or ranges, eg "1,4,8,9-12,30-40"
    */
    std::vector<std::string> strs;
    boost::split(strs, str, boost::is_any_of(","));
    for (const string& element : strs) {
        if (element == "") {
           continue;
        }
        std::vector<std::string> range;
        boost::split(range, element, boost::is_any_of("-"));
        if (range.size() == 2) {
           int start = std::stoi(range[0]);
           int end = std::stoi(range[1]);
           RCLCPP_INFO(nh->get_logger(), "Ignoring fiducial id range %d to %d", start, end);
           for (int j=start; j<=end; j++) {
               ignoreIds.push_back(j);
           }
        }
        else if (range.size() == 1) {
           int fid = std::stoi(range[0]);
           RCLCPP_INFO(nh->get_logger(), "Ignoring fiducial id %d", fid);
           ignoreIds.push_back(fid);
        }
        else {
           RCLCPP_ERROR(nh->get_logger(), "Malformed ignore_fiducials: %s", element.c_str());
        }
    }
}

bool FiducialsNode::enableDetectionsCallback(const std_srvs::srv::SetBool::Request &req,
                                             std_srvs::srv::SetBool::Response &res)
{
    enable_detections = req.data;
    if (enable_detections){
        res.message = "Enabled aruco detections.";
        RCLCPP_INFO(nh->get_logger(), "Enabled aruco detections.");
    }
    else {
        res.message = "Disabled aruco detections.";
        RCLCPP_INFO(nh->get_logger(), "Disabled aruco detections.");
    }
    
    res.success = true;
    return true;
}


FiducialsNode::FiducialsNode() : it(nh), broadcaster(nh)
{
    frameNum = 0;

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    haveCamInfo = false;
    enable_detections = true;

    int dicno;

    detectorParams = new aruco::DetectorParameters();

    pnh->get_parameter_or("publish_images", publish_images, false);
    pnh->get_parameter_or("fiducial_len", fiducial_len, 0.14);
    pnh->get_parameter_or("dictionary", dicno, 7);
    pnh->get_parameter_or("do_pose_estimation", doPoseEstimation, true);
    pnh->get_parameter_or("publish_fiducial_tf", publishFiducialTf, true);

    std::string str;
    std::vector<std::string> strs;

    pnh->get_parameter_or("ignore_fiducials", str, std::string());
    handleIgnoreString(str);

    /*
    fiducial size can take comma separated list of size: id or size: range,
    e.g. "200.0: 12, 300.0: 200-300"
    */
    pnh->get_parameter_or("fiducial_len_override", str, std::string());
    boost::split(strs, str, boost::is_any_of(","));
    for (const string& element : strs) {
        if (element == "") {
           continue;
        }
        std::vector<std::string> parts;
        boost::split(parts, element, boost::is_any_of(":"));
        if (parts.size() == 2) {
            double len = std::stod(parts[1]);
            std::vector<std::string> range;
            boost::split(range, element, boost::is_any_of("-"));
            if (range.size() == 2) {
               int start = std::stoi(range[0]);
               int end = std::stoi(range[1]);
               RCLCPP_INFO(nh->get_logger(), "Setting fiducial id range %d - %d length to %f",
                        start, end, len);
               for (int j=start; j<=end; j++) {
                   fiducialLens[j] = len;
               }
            }
            else if (range.size() == 1){
               int fid = std::stoi(range[0]);
               RCLCPP_INFO(nh->get_logger(), "Setting fiducial id %d length to %f", fid, len);
               fiducialLens[fid] = len;
            }
            else {
               RCLCPP_ERROR(nh->get_logger(), "Malformed fiducial_len_override: %s", element.c_str());
            }
        }
        else {
           RCLCPP_ERROR(nh->get_logger(), "Malformed fiducial_len_override: %s", element.c_str());
        }
    }

    image_pub = it.advertise("/fiducial_images", 1);

    //vertices_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialArray>("fiducial_vertices", 1));

    //pose_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialTransformArray>("fiducial_transforms", 1));

    dictionary = aruco::getPredefinedDictionary(dicno);

    //img_sub = it.subscribe("camera", 1,
    //                     &FiducialsNode::imageCallback, this);

    caminfo_sub = nh->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 1,
                     std::bind(&FiducialsNode::camInfoCallback, this, std::placeholders::_1));

    // ignore_sub = nh.create_subscription<std_msgs::msg::String>("ignore_fiducials", 1,
    //                                     std::bind(&FiducialsNode::ignoreCallback, this, std::placeholders::_1));

    //service_enable_detections = nh.create_service("enable_detections", &FiducialsNode::enableDetectionsCallback);

    //callbackType = boost::bind(&FiducialsNode::configCallback, this, _1, _2); // TODO migrate to ROS2
    //configServer.setCallback(callbackType); // TODO migrate to ROS2

    pnh->get_parameter_or("adaptiveThreshConstant", detectorParams->adaptiveThreshConstant, 7.0);
    pnh->get_parameter_or("adaptiveThreshWinSizeMax", detectorParams->adaptiveThreshWinSizeMax, 53); /* defailt 23 */
    pnh->get_parameter_or("adaptiveThreshWinSizeMin", detectorParams->adaptiveThreshWinSizeMin, 3);
    pnh->get_parameter_or("adaptiveThreshWinSizeStep", detectorParams->adaptiveThreshWinSizeStep, 4); /* default 10 */
    pnh->get_parameter_or("cornerRefinementMaxIterations", detectorParams->cornerRefinementMaxIterations, 30);
    pnh->get_parameter_or("cornerRefinementMinAccuracy", detectorParams->cornerRefinementMinAccuracy, 0.01); /* default 0.1 */
    pnh->get_parameter_or("cornerRefinementWinSize", detectorParams->cornerRefinementWinSize, 5);
#if CV_MINOR_VERSION==2 and CV_MAJOR_VERSION==3
    pnh->get_parameter_or("doCornerRefinement",detectorParams->doCornerRefinement, true); /* default false */
#else
    bool doCornerRefinement = true;
    pnh->get_parameter_or("doCornerRefinement", doCornerRefinement, true);
    if (doCornerRefinement) {
       bool cornerRefinementSubPix = true;
       pnh->get_parameter_or("cornerRefinementSubPix", cornerRefinementSubPix, true);
       if (cornerRefinementSubPix) {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
       }
       else {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
       }
    }
    else {
       detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_NONE;
    }
#endif
    pnh->get_parameter_or("errorCorrectionRate", detectorParams->errorCorrectionRate , 0.6);
    pnh->get_parameter_or("minCornerDistanceRate", detectorParams->minCornerDistanceRate , 0.05);
    pnh->get_parameter_or("markerBorderBits", detectorParams->markerBorderBits, 1);
    pnh->get_parameter_or("maxErroneousBitsInBorderRate", detectorParams->maxErroneousBitsInBorderRate, 0.04);
    pnh->get_parameter_or("minDistanceToBorder", detectorParams->minDistanceToBorder, 3);
    pnh->get_parameter_or("minMarkerDistanceRate", detectorParams->minMarkerDistanceRate, 0.05);
    pnh->get_parameter_or("minMarkerPerimeterRate", detectorParams->minMarkerPerimeterRate, 0.1); /* default 0.3 */
    pnh->get_parameter_or("maxMarkerPerimeterRate", detectorParams->maxMarkerPerimeterRate, 4.0);
    pnh->get_parameter_or("minOtsuStdDev", detectorParams->minOtsuStdDev, 5.0);
    pnh->get_parameter_or("perspectiveRemoveIgnoredMarginPerCell", detectorParams->perspectiveRemoveIgnoredMarginPerCell, 0.13);
    pnh->get_parameter_or("perspectiveRemovePixelPerCell", detectorParams->perspectiveRemovePixelPerCell, 8);
    pnh->get_parameter_or("polygonalApproxAccuracyRate", detectorParams->polygonalApproxAccuracyRate, 0.01); /* default 0.05 */

    RCLCPP_INFO(nh->get_logger(), "Aruco detection ready");
}

int main(int argc, char ** argv) {
    // ros::init(argc, argv, "aruco_detect"); // TODO ROS2 migrate

    FiducialsNode* node = new FiducialsNode();

    // ros::spin(); //TODO ROS2 migrate

    return 0;
}
