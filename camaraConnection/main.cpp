#include <iostream>
#include <opencv2/opencv.hpp>
#include "neoapi/neoapi.hpp"

using namespace std;
using namespace cv;
using namespace NeoAPI;

int main() {
    Cam camera;
    string camIdentifier = "192.168.41.65"; // Reemplaza con la IP de tu cámara si es necesario

    try {
        // Intentar conectar la cámara
        while(!camera.IsConnected()) {
            cout << "Intentando conectar cámara..." << endl;
            camera.Connect(NeoString(camIdentifier.c_str()));
        }

        cout << "Cámara conectada exitosamente!" << endl;

        // Configuración mínima
        camera.StopStreaming(); // Nos aseguramos que no haga streaming
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
            isColor = true;
        } else if (camera.f().PixelFormat.GetEnumValueList().IsReadable("Mono8")) {
            camera.f().PixelFormat.SetString("Mono8");
            type = CV_8UC1;
            isColor = false;
        } else {
            cerr << "Formato de pixel no válido." << endl;
            return 1;
        }

        int width = camera.f().Width;
        int height = camera.f().Height;

        // Tomar una sola imagen
        Image image = camera.GetImage();
        if (image.IsEmpty()) {
            cerr << "No se pudo capturar la imagen." << endl;
            return 1;
        }

        Mat img(Size(width, height), type, image.GetImageData(), Mat::AUTO_STEP);

        // Mostrar imagen
        imshow("Imagen de la cámara", img);
        cout << "Presiona cualquier tecla para salir..." << endl;
        waitKey(0); // Espera hasta que el usuario presione una tecla

        camera.Disconnect();
        cout << "Cámara desconectada. Fin del programa." << endl;

    } catch (NoAccessException &e) {
        cerr << "NoAccessException: No se puede acceder a la cámara." << endl;
    } catch (NotConnectedException &e) {
        cerr << "NotConnectedException: La cámara no está conectada." << endl;
    } catch (exception &e) {
        cerr << "Error inesperado: " << e.what() << endl;
    }

    return 0;
}
