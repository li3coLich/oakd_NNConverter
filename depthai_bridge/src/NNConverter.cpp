
#include "depthai_bridge/NNConverter.hpp"

#include <sensor_msgs/msg/detail/compressed_image__struct.hpp>

#include "depthai/depthai.hpp"
#include "depthai-shared/datatype/RawEncodedFrame.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"

namespace dai {

namespace ros {

NNConverter::NNConverter(const std::string frameName, bool interleaved, bool getBaseDeviceTimestamp)
    : frameName(frameName), daiInterleaved(interleaved), steadyBaseTime(std::chrono::steady_clock::now()), getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
    rosBaseTime = rclcpp::Clock().now();
}

NNConverter::~NNConverter() = default;

void NNConverter::updateRosBaseTime() {
    updateBaseTime(steadyBaseTime, rosBaseTime, totalNsChange);
}

std::vector<uint8_t> NNConverter::rearrangeData(const std::vector<int32_t>& inputData, int height, int width, int channels) {
    if (inputData.size() != static_cast<size_t>(height * width * channels)) {
        throw std::runtime_error("Input data size does not match the provided dimensions.");
    }
    std::vector<uint8_t> outputData(height * width * channels);
    for (int h = 0; h < height; ++h) {
        for (int w = 0; w < width; ++w) {
            for (int c = 0; c < channels; ++c) {
                // Row-major order: output[h][w][c] = input[c][h][w]
                // Convert int32_t to uint8_t during assignment (ensure valid range [0, 255])
                int32_t value = inputData[c * height * width + h * width + w];
                outputData[h * width * channels + w * channels + c] = static_cast<uint8_t>(std::clamp(value, 0, 255));
            }
        }
    }
    return outputData;
}

void NNConverter::addExposureOffset(dai::CameraExposureOffset& offset) {
    expOffset = offset;
    addExpOffset = true;
}

ImageMsgs::Image NNConverter::toRosMsgRawPtr(std::shared_ptr<dai::NNData> inData, const sensor_msgs::msg::CameraInfo& info, int batch, int channels, int height, int width) {
    if(updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    ImageMsgs::Image outImageMsg;
    outImageMsg.header.stamp = rclcpp::Clock().now(); // Set the current timestamp
    outImageMsg.header.frame_id = info.header.frame_id; // Use frame_id from the provided CameraInfo
    // StdMsgs::Header header;
    // header.frame_id = frameName;
    // header.stamp = getFrameTime(rosBaseTime, steadyBaseTime, tstamp);
    std::string layerName = "output"; // Use the exact layer name
    std::vector<int> data = inData->getLayerInt32(layerName);
    auto rearrangedData = rearrangeData(data, height, width, channels);
    // outImageMsg.header = header;
    outImageMsg.height = height;
    outImageMsg.width = width;
    outImageMsg.encoding = (channels == 1) ? "mono8" : "rgb8"; // Choose encoding based on channels
    outImageMsg.is_bigendian = false; // Assuming little-endian system
    outImageMsg.step = width * channels; // Row length in bytes
    outImageMsg.data = std::move(rearrangedData); // Move the tensor data into the ROS Image
    return outImageMsg;
}

void NNConverter::toRosMsg(std::shared_ptr<dai::NNData> inData, std::deque<ImageMsgs::Image>& outImageMsgs) {
    auto outImageMsg = toRosMsgRawPtr(inData);
    outImageMsgs.push_back(outImageMsg);
    return;
}
}  // namespace ros
}  // namespace dai
