#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

int main(int argc, char** argv)
{
    // Check for image path argument
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image_path>\n";
        return 1;
    }

    // Load image in grayscale
    cv::Mat image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Error: Could not read image\n";
        return 1;
    }

    // Configure SimpleBlobDetector parameters
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 10;
    params.maxThreshold = 200;
    params.filterByArea = true;
    params.minArea = 1500;          // Minimum area of blob
    params.filterByCircularity = true;
    params.minCircularity = 0.1f;   // Allow imperfect circles
    params.filterByConvexity = true;
    params.minConvexity = 0.87f;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01f;
    params.blobColor = 0;            // Detect dark blobs

    // Detect keypoints (blobs)
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(image, keypoints);

    // Filter blobs by average intensity inside each keypoint to ensure they are dark
    std::vector<cv::KeyPoint> filtered;
    for (auto &kp : keypoints) {
        int radius = static_cast<int>(kp.size / 2);
        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8U);
        cv::circle(mask, kp.pt, radius, cv::Scalar(255), -1);

        cv::Scalar mean_val = cv::mean(image, mask);
        if (mean_val[0] < 50) {  // Threshold for dark color
            filtered.push_back(kp);
        }
    }

    // Sort blobs by size (optional, ensures consistent ordering)
    std::sort(filtered.begin(), filtered.end(),
              [](const cv::KeyPoint &a, const cv::KeyPoint &b){ return a.size > b.size; });

    // Keep only the 3 most relevant blobs
    if (filtered.size() > 3) filtered.resize(3);

    // Convert grayscale to color for visualization
    cv::Mat output;
    cv::cvtColor(image, output, cv::COLOR_GRAY2BGR);

    // Draw each blob and annotate coordinates directly on the image
    for (size_t i = 0; i < filtered.size(); ++i) {
        int x = static_cast<int>(filtered[i].pt.x);
        int y = static_cast<int>(filtered[i].pt.y);

        // Draw filled black circle
        cv::circle(output, filtered[i].pt,
                   static_cast<int>(filtered[i].size / 2),
                   cv::Scalar(0, 0, 0), -1);

        // Draw red contour for visibility
        cv::circle(output, filtered[i].pt,
                   static_cast<int>(filtered[i].size / 2),
                   cv::Scalar(0, 0, 255), 2);

        // Prepare annotation text: "Circle 1: (x, y)"
        std::string text = "Circle " + std::to_string(i + 1) +
                           ": (" + std::to_string(x) + ", " + std::to_string(y) + ")";

        // Put text above the circle
        cv::putText(output, text, cv::Point(x - 40, y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }

    // Display the annotated image
    cv::imshow("Detected Circles", output);
    cv::waitKey(0);

    return 0;
}
