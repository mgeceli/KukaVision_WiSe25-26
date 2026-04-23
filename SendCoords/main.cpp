#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define ROBOT_IP "192.168.41.64"
#define ROBOT_PORT 54600
#define BUFFER_SIZE 64

using namespace std;

// Funcion para comandos simples (open, close, home)
void mandarComandoBasico(int sock, const char* comando) {
    char buffer[BUFFER_SIZE];
    memset(buffer, 0, BUFFER_SIZE);
    strcpy(buffer, comando);

    cout << "[PC] Enviando: " << comando << endl;
    send(sock, buffer, BUFFER_SIZE, 0);

    memset(buffer, 0, BUFFER_SIZE);
    read(sock, buffer, BUFFER_SIZE);

    cout << "[ROBOT] Confirma: " << comando << endl;
}

// --- FUNCION PARA ENVIAR COORDENADAS ---
void mandarMovimiento(int sock, float x, float y, float z, float a, float b, float c) {
    char buffer[BUFFER_SIZE];
    memset(buffer, 0, BUFFER_SIZE); // Limpiamos todo con ceros

    // 1. Escribimos la palabra "move" al principio del buffer (ocupa 4 bytes)
    memcpy(buffer, "move", 4);

    // 2. A partir del byte 4, pegamos los floats (coordenadas) en binario
    int offset = 4;
    memcpy(buffer + offset, &x, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &y, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &z, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &a, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &b, sizeof(float)); offset += sizeof(float);
    memcpy(buffer + offset, &c, sizeof(float));

    cout << "[PC] Enviando Movimiento XYZ: (" << x << ", " << y << ", " << z << ")" << endl;

    // 3. Enviamos los 64 bytes
    send(sock, buffer, BUFFER_SIZE, 0);

    // 4. Esperamos la confirmacion
    memset(buffer, 0, BUFFER_SIZE);
    read(sock, buffer, BUFFER_SIZE);
    cout << "[ROBOT] Movimiento finalizado." << endl;
}

int main() {
    int sock = 0;
    struct sockaddr_in serv_addr;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) return -1;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(ROBOT_PORT);
    inet_pton(AF_INET, ROBOT_IP, &serv_addr.sin_addr);

    cout << "Conectando al KUKA..." << endl;
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        cout << "Error de conexion." << endl;
        return -1;
    }
    cout << "--- CONECTADO ---" << endl;
    sleep(3);

    // --- SECUENCIA DE TRABAJO ---

    // 1. Asegurar que estamos en Home
    mandarComandoBasico(sock, "home");
    sleep(2);

    // 2. Abrir el gripper
    mandarComandoBasico(sock, "open");
    sleep(1);

    // 3. ¡Mover a una coordenada!
    // ⚠️ ATENCION: Cambia estos numeros por coordenadas reales y seguras de tu robot.
    // Ejemplo: X=400.0, Y=0.0, Z=500.0, A=180.0, B=90.0, C=0.0
    float posX = 643.48;
    float posY = -184.64;
    float posZ = 845.26;
    float angA = -18.43;
    float angB = 0;
    float angC = 0;

    mandarMovimiento(sock, posX, posY, posZ, angA, angB, angC);

    // Le damos tiempo extra para terminar el movimiento fisico
    sleep(3);

    // 4. Cerrar el gripper en esa nueva posicion
    mandarComandoBasico(sock, "close");
    sleep(1);

    // 5. Volver a Home
    mandarComandoBasico(sock, "home");
    sleep(3);

    close(sock);
    cout << "--- FIN DEL PROGRAMA ---" << endl;
    return 0;
}
