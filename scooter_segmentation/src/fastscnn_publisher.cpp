#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

// #include "NNConverter.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/NNConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

std::tuple<dai::Pipeline, int, int> createPipeline( bool lrcheck,
                                                   bool extended,
                                                   bool subpixel,
                                                   int stereo_fps,
                                                   int confidence,
                                                   int LRchecktresh,
                                                   int previewWidth,
                                                   int previewHeight,
                                                   std::string nnPath) {
    dai::Pipeline pipeline;

    pipeline.setOpenVINOVersion(dai::OpenVINO::Version::VERSION_2022_1);

    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();


    controlIn->setStreamName("control");
    controlIn->out.link(monoRight->inputControl);
    controlIn->out.link(monoLeft->inputControl);

    xoutDepth->setStreamName("depth");

    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    int stereoWidth, stereoHeight, rgbWidth, rgbHeight;

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoLeft->setFps(stereo_fps);

    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    monoRight->setFps(stereo_fps);

    stereoWidth = 640;
    stereoHeight = 360;

    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // stereo->setOutputSize(stereoWidth, stereoHeight);
    
    // stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    dai::node::ColorCamera::Properties::SensorResolution rgbResolution;

    camRgb->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P);
    rgbWidth = 1920;
    rgbHeight = 1080;

    rgbWidth = rgbWidth / 3;
    rgbHeight = rgbHeight / 3;

    camRgb->setIspScale(1, 3);

    camRgb->isp.link(xoutRgb->input);

    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setInterleaved(false);
    camRgb->setPreviewSize(previewWidth, previewHeight);

    auto NN = pipeline.create<dai::node::NeuralNetwork>();
    NN->setBlobPath(nnPath);
    NN->setNumInferenceThreads(2);
    NN->input.setBlocking(false);

    auto xoutNN = pipeline.create<dai::node::XLinkOut>();
    xoutNN->setStreamName("segmentation");

    camRgb->preview.link(NN->input);
    NN->out.link(xoutNN->input);

    stereoWidth = rgbWidth;
    stereoHeight = rgbHeight;
    
    stereo->setRectifyEdgeFillColor(0);
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    
    stereo->depth.link(xoutDepth->input);

    std::cout << stereoWidth << " " << stereoHeight << " " << rgbWidth << " " << rgbHeight << std::endl;
    return std::make_tuple(pipeline, stereoWidth, stereoHeight);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fastscnn_node");

    std::string tfPrefix, mxId, resourceBaseFolder, nnPath;
    int stereo_fps, confidence, LRchecktresh, previewWidth, previewHeight;
    bool lrcheck, extended, subpixel;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("mxId", "x");
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);
    node->declare_parameter("previewWidth", 640);
    node->declare_parameter("previewHeight", 360);

    node->declare_parameter("stereo_fps", 15);
    node->declare_parameter("resourceBaseFolder", resourceBaseFolder);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("mxId", mxId);
    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("extended", extended);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("previewWidth", previewWidth);
    node->get_parameter("previewHeight", previewHeight);


    node->get_parameter("stereo_fps", stereo_fps);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);

    // std::string resourceBaseFolder, nnPath;
    nnPath = resourceBaseFolder + "/campus_360x640.blob";
    std::string monoResolution = "720p", rgbResolution = "1080p";
    std::cout << nnPath << std::endl;

    dai::Pipeline pipeline;
    int width, height;
    bool isDeviceFound = false;

    
    std::tie(pipeline, width, height) = createPipeline( lrcheck,
                                                        extended,
                                                        subpixel,
                                                        stereo_fps,
                                                        confidence,
                                                        LRchecktresh,
                                                        previewWidth,
                                                        previewHeight,
                                                        nnPath);

    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    std::cout << "Listing available devices..." << std::endl;
    for(auto deviceInfo : availableDevices) {
        std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
        if(deviceInfo.getMxId() == mxId) {
            if(deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER) {
                isDeviceFound = true;
                device = std::make_shared<dai::Device>(pipeline, deviceInfo, false);
                break;
            } else if(deviceInfo.state == X_LINK_BOOTED) {
                throw std::runtime_error("\" DepthAI Device with MxId  \"" + mxId + "\" is already booted on different process.  \"");
            }
        } else if(mxId == "x") {
            isDeviceFound = true;
            device = std::make_shared<dai::Device>(pipeline);
        }
    }

    if(!isDeviceFound) {
        throw std::runtime_error("\" DepthAI Device with MxId  \"" + mxId + "\" not found.  \"");
    }

    auto controlQueue = device->getInputQueue("control");
    // Set manual exposure
    dai::CameraControl ctrl;
    ctrl.setManualExposure(20000, 800);
    controlQueue->send(ctrl);

    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    stereoQueue = device->getOutputQueue("depth", 15, false);

    auto calibrationHandler = device->readCalibration();

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    rgbConverter.setUpdateRosBaseTimeOnToRosMsg();

    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    rightconverter.setUpdateRosBaseTimeOnToRosMsg();
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

    auto depthCameraInfo = rightCameraInfo;
    auto depthconverter = rightconverter;
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        stereoQueue,
        node,
        std::string("/oak_d_pro/stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                    &depthconverter,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                    std::placeholders::_1,
                    std::placeholders::_2),
        15,
        depthCameraInfo,
        "/oak_d_pro/stereo");
    depthPublish.addPublisherCallback();

    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);
    auto imgQueue = device->getOutputQueue("rgb", 15, false);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
        imgQueue,
        node,
        std::string("/oak_d_pro/rgb/image_raw"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        15,
        rgbCameraInfo,
        "/oak_d_pro/rgb");
    rgbPublish.addPublisherCallback();

    dai::rosBridge::NNConverter nnConverter(tfPrefix + "_rgb_camera_optical_frame", false, false);
    nnConverter.setUpdateRosBaseTimeOnToRosMsg();
    auto segmentationQueue = device->getOutputQueue("segmentation", 15, false);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::NNData> segmentationPublish(
        segmentationQueue,
        node,
        std::string("/oak_d_pro/seg"),
        std::bind(&dai::rosBridge::NNConverter::toRosMsg,
                &nnConverter,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                std::placeholders::_1,
                std::placeholders::_2),
        15);
    segmentationPublish.addPublisherCallback();
    rclcpp::spin(node);

    return 0;
}

