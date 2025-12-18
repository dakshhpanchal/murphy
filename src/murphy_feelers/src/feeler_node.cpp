#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include "murphy_feelers_core/feeler.hpp"

using std::placeholders::_1;

class FeelerNode : public rclcpp::Node {
public:
    FeelerNode() : Node("feeler_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/murphy/vision/fused_mask",
            10,
            std::bind(&FeelerNode::maskCallback, this, _1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/murphy/feelers/debug",
            10
        );

        initFeelers();
        RCLCPP_INFO(this->get_logger(), "Feeler node started");
    }

private:
    void initFeelers() {
        // Fan of feelers: -90° to +90°
        for (int deg = -90; deg <= 90; deg += 15) {
            double r = radians(deg);
            int x = std::cos(r) * 200;
            int y = std::sin(r) * 200;
            feelers_.emplace_back(x, y);
        }
    }

    void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat mask = cv_bridge::toCvCopy(msg, "mono8")->image;
        cv::Mat debug;
        cv::cvtColor(mask, debug, cv::COLOR_GRAY2BGR);

        for (auto &f : feelers_) {
            f.update(mask);
            f.draw(debug);
        }

        auto out =
            cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg();
        pub_->publish(*out);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::vector<Feeler> feelers_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeelerNode>());
    rclcpp::shutdown();
    return 0;
}

