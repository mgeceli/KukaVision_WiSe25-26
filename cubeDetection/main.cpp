#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Usage: detect_cube <reference_image> <current_image>" << endl;
        return -1;
    }

    // ================== LOAD IMAGES ==================
    Mat ref = imread(argv[1], IMREAD_GRAYSCALE);
    Mat cur = imread(argv[2], IMREAD_GRAYSCALE);

    if (ref.empty() || cur.empty()) {
        cout << "Error loading images" << endl;
        return -1;
    }

    // ================== CALIBRATION ==================
    vector<Point2f> camPts = {
        Point2f(646, 248), //Point 1 (x, y)px
        Point2f(617, 770), //Point 2 (x, y)px
        Point2f(1395, 815) //Point 3 (x, y)px
    };

    vector<Point2f> robotPts = {
        Point2f(159.02f, 643.85f), //Point 1 (x, y)mm
        Point2f(349.60f, 676.21f), //Point 2 (x, y)mm
        Point2f(398.97f, 391.18f) //Point 3 (x, y)mm
    };

    Mat affineMat = getAffineTransform(camPts, robotPts);

    // ================== PREPROCESS ==================
    GaussianBlur(ref, ref, Size(5,5), 0);
    GaussianBlur(cur, cur, Size(5,5), 0);

    // ================== DIFFERENCE ==================
    Mat diff;
    absdiff(ref, cur, diff);

    // ================== ADAPTIVE THRESHOLD ==================
    Mat bin;
    adaptiveThreshold(
        diff, bin,
        255,
        ADAPTIVE_THRESH_MEAN_C,
        THRESH_BINARY,
        21,
        -5
    );

    // ================== MORPHOLOGY ==================
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5));
    morphologyEx(bin, bin, MORPH_OPEN, kernel);
    morphologyEx(bin, bin, MORPH_CLOSE, kernel);

    // ================== CONTOURS ==================
    vector<vector<Point>> contours;
    findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat output;
    cvtColor(cur, output, COLOR_GRAY2BGR);

    cout << "Contours found: " << contours.size() << endl;

    // ================== FILTER + POSE ==================
    for (size_t i = 0; i < contours.size(); i++) {

        double area = contourArea(contours[i]);
        if (area < 600) continue;

        Rect bbox = boundingRect(contours[i]);
        double solidity = area / bbox.area();
        if (solidity < 0.7) continue;

        RotatedRect box = minAreaRect(contours[i]);
        float w = box.size.width;
        float h = box.size.height;
        float ratio = max(w,h) / min(w,h);
        if (ratio > 1.4) continue;

        // ---- Cube accepted ----
        Point2f center = box.center;
        float angle = box.angle;
        if (w < h) angle += 90.0f;

        // ================== PIXEL → ROBOT ==================
        Mat camPoint = (Mat_<double>(3,1) << center.x, center.y, 1.0);
        Mat robotPoint = affineMat * camPoint;

        double x_robot = robotPoint.at<double>(0,0);
        double y_robot = robotPoint.at<double>(1,0);

        // ================== DRAW ==================
        Point2f vertices[4];
        box.points(vertices);
        for (int j = 0; j < 4; j++)
            line(output, vertices[j], vertices[(j+1)%4],
                 Scalar(0,255,0), 2);

        circle(output, center, 4, Scalar(0,0,255), -1);

        // ================== PRINT ==================
        cout << "Cube detected" << endl;
        cout << " Pixel X: " << center.x << endl;
        cout << " Pixel Y: " << center.y << endl;
        cout << " Robot X (mm): " << x_robot << endl;
        cout << " Robot Y (mm): " << y_robot << endl;
        cout << " Rotation (deg): " << angle << endl;
        cout << "-------------------------" << endl;
    }

    // ================== DISPLAY ==================
    imshow("Detection Result", output);

    waitKey(0);
    return 0;
}
