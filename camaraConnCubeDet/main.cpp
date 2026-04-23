#include <iostream>
#include <opencv2/opencv.hpp>
#include "neoapi/neoapi.hpp"
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace NeoAPI;

// --- NETWORK CONFIGURATION ---
#define ROBOT_IP "192.168.41.64"
#define ROBOT_PORT 54600
#define BUFFER_SIZE 64

// --- OPERATIONAL CONSTANTS (Professional variables) ---

const float offset_x = 15.0f;
const float offset_y = 35.0f;


const float Z_APPROACH_PICK  = 286.0f;
const float Z_PICK           = 155.0f;
const float Z_APPROACH_PLACE = 660.0f;

// --- TARGET PLACE COORDINATES ---
const float PLACE_X = 749.00f;
const float PLACE_Y = -467.73f;
const float PLACE_Z = 400.0f;
const float PLACE_A = -85.61f;
const float PLACE_B = 51.74f;
const float PLACE_C = 3.00f;

// Global variables for data persistence
float g_x = 0, g_y = 0, g_a = 0;
bool g_cuboLocalizado = false;

/**
 * Sends string-based commands to the KUKA controller and waits for confirmation.
 * Used for "home", "open", and "close" routines.
 */
void mandarComandoBasico(int sock, const char* comando) {
    char buffer[BUFFER_SIZE];
    memset(buffer, 0, BUFFER_SIZE);
    strcpy(buffer, comando);
    send(sock, buffer, BUFFER_SIZE, 0);
    memset(buffer, 0, BUFFER_SIZE);
    read(sock, buffer, BUFFER_SIZE);
    cout << "[ROBOT] Confirmed: " << comando << endl;
}

/**
 * Sends binary coordinate data (6 floats) to the robot.
 * Protocol: [4 bytes "move"] + [24 bytes for X,Y,Z,A,B,C]
 */
void mandarMovimiento(int sock, float x, float y, float z, float a, float b, float c) {
    char buffer[BUFFER_SIZE];
    memset(buffer, 0, BUFFER_SIZE);
    memcpy(buffer, "move", 4);
    int offset = 4;
    memcpy(buffer + offset, &x, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &y, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &z, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &a, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &b, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &c, sizeof(float));
    send(sock, buffer, BUFFER_SIZE, 0);
    memset(buffer, 0, BUFFER_SIZE);
    read(sock, buffer, BUFFER_SIZE);
}

/**
 * Handles image Point2f(402.78f, 200.61f)acquisition and target identification.
 * Uses adaptive thresholding and contour filtering to locate the black cube.
 */
bool faseVision(const string& refPath) {
    Mat ref = imread(refPath, IMREAD_GRAYSCALE);
    if (ref.empty()) return false;

    Cam camera;
    string camIdentifier = "192.168.41.65";

    try {
        while(!camera.IsConnected()) {
            camera.Connect(NeoString(camIdentifier.c_str()));
        }

        // Camera Initialization Settings
        camera.StopStreaming();
        camera.f().TriggerMode = TriggerMode::Off;
        camera.f().AcquisitionMode.Set(AcquisitionMode::Continuous);
        camera.f().ExposureAuto.Set(ExposureAuto::Continuous);
        camera.f().AcquisitionFrameRateEnable.Set(true);
        camera.f().AcquisitionFrameRate.Set(1);
        camera.SetImageBufferCount(1);
        camera.SetImageBufferCycleCount(1);
        camera.SetUserBufferMode(false);

        int type = CV_8U;
        bool isColor = true;
        if (camera.f().PixelFormat.GetEnumValueList().IsReadable("BGR8")) {
            camera.f().PixelFormat.SetString("BGR8");
            type = CV_8UC3;
        } else if (camera.f().PixelFormat.GetEnumValueList().IsReadable("Mono8")) {
            camera.f().PixelFormat.SetString("Mono8");
            type = CV_8UC1;
            isColor = false;
        }

        Image image = camera.GetImage();
        if (image.IsEmpty()) { camera.Disconnect(); return false; }

        Mat cur_raw(Size((int)camera.f().Width, (int)camera.f().Height), type, image.GetImageData(), Mat::AUTO_STEP);
        Mat cur = cur_raw.clone();
        if (isColor) cvtColor(cur, cur, COLOR_BGR2GRAY);

        camera.Disconnect();

        // Image Processing for detection
        vector<Point2f> camPts = { Point2f(646, 248), Point2f(617, 770), Point2f(1395, 815), Point2f(1441, 286)};
        vector<Point2f> robotPts = { Point2f(688.00f, 164.12f), Point2f(688.00f, 356.29f), Point2f(396.70f, 356.29f), Point2f(396.70f, 161.66f)};
        Mat H = findHomography(camPts, robotPts, RANSAC);

        GaussianBlur(ref, ref, Size(9,9), 0);
        GaussianBlur(cur, cur, Size(9,9), 0);
        Mat diff, bin;
        absdiff(ref, cur, diff);
        threshold(diff, bin, 30, 255, THRESH_BINARY | THRESH_OTSU);
        Mat kernel = getStructuringElement(MORPH_RECT, Size(9,9));
        morphologyEx(bin, bin, MORPH_OPEN, kernel);
        morphologyEx(bin, bin, MORPH_CLOSE, kernel);

        vector<vector<Point>> contours;
        findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Anzahl gefundener Konturen: " << contours.size() << endl;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            cout << "Kontur " << i << " - Area: " << area << endl;
            if (area < 2000) continue;

            imshow("Binärbild (Maske)", bin);
            imshow("Diff", diff);
            waitKey(30);

            Mat mask = Mat::zeros(cur.size(), CV_8UC1);
            drawContours(mask, contours, (int)i, Scalar(255), -1);
            if (mean(cur, mask)[0] > 120) continue;

            Rect bbox = boundingRect(contours[i]);
            if ((area / bbox.area()) < 0.5) continue;

            RotatedRect box = minAreaRect(contours[i]);
            float ratio = max(box.size.width, box.size.height) / min(box.size.width, box.size.height);
            if (ratio > 2.0) continue;

            // Target DataPoint2f(402.78f, 200.61f) Extraction

            vector<Point2f> src(1), dst;
            src[0] = box.center;

            perspectiveTransform(src, dst, H);

            g_x = dst[0].x;
            g_y = dst[0].y;

            g_x += offset_x;
            g_y += offset_y;


            float angle_A = box.angle;
            if (box.size.width < box.size.height) angle_A += 180.0f;
            if (angle_A > 180) angle_A -= 90;


            g_a = 0.0f;
            g_cuboLocalizado = true;

            cout << "[SUCCESS] Cube identified at X: " << g_x << " Y: " << g_y << endl;
            imshow("Deteccion", cur);
            waitKey(1000);
            return true;
        }
    } catch (...) { return false; }
    return false;
}

int main(int argc, char** argv) {
    if (argc < 2) return -1;
    int opcion = 0;

    while (opcion != 3) {
        cout << "\n--- MAIN CONTROL MENU ---" << endl;
        cout << "1. Run Detection" << endl;
        if (g_cuboLocalizado) cout << "2. EXECUTE FULL PICK & PLACE" << endl;
        cout << "3. Exit" << endl;
        cout << "Selection: ";
        cin >> opcion;

        if (opcion == 1) {
            if (faseVision(argv[1])) cout << "Target acquired." << endl;
            else cout << "[FAIL] Detection failed." << endl;
        }
        else if (opcion == 2 && g_cuboLocalizado) {
            int sock = socket(AF_INET, SOCK_STREAM, 0);
            struct sockaddr_in serv_addr;
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(ROBOT_PORT);
            inet_pton(AF_INET, ROBOT_IP, &serv_addr.sin_addr);

            if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) >= 0) {
                cout << "Connected. Handshaking (3s)..." << endl;
                sleep(3);

                // --- FULL SEQUENCE START ---
                mandarComandoBasico(sock, "home");
                sleep(2);
                mandarComandoBasico(sock, "open");
                sleep(2);

                // PICK SEQUENCE
                mandarMovimiento(sock, g_x, g_y, Z_APPROACH_PICK, g_a, 90.0, 0.0);
                sleep(2);
                mandarMovimiento(sock, g_x, g_y, Z_PICK, g_a, 90.0, 0.0);
                sleep(2);
                mandarComandoBasico(sock, "close");
                sleep(2);
                mandarMovimiento(sock, g_x, g_y, Z_APPROACH_PICK, g_a, 90.0, 0.0);
                sleep(2);
                mandarComandoBasico(sock, "home");
                sleep(2);


                // PLACE SEQUENCE
                cout << "Initiating Place sequence..." << endl;
                mandarMovimiento(sock, PLACE_X, PLACE_Y, Z_APPROACH_PLACE, PLACE_A, PLACE_B, PLACE_C);
                sleep(2);
                mandarMovimiento(sock, PLACE_X, PLACE_Y, PLACE_Z, PLACE_A, PLACE_B, PLACE_C);
                sleep(2);
                mandarComandoBasico(sock, "open");
                sleep(1);
                mandarComandoBasico(sock, "home");

                close(sock);
                cout << "Pick & Place cycle completed." << endl;
            }
        }
    }
    return 0;
}

