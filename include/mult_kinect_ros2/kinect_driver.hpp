#ifndef KINECT_DRIVER_HPP_
#define KINECT_DRIVER_HPP_

#include <opencv2/opencv.hpp>
#include <libfreenect/libfreenect.hpp>
#include <vector>
#include <unordered_map>  // Necessário para armazenar relação entre dispositivos e instâncias

class KinectManager {
public:
    KinectManager();
    ~KinectManager();
    
    bool initialize();  // Inicializa os dispositivos Kinect conectados
    bool captureFrame(int device_index, cv::Mat &image);  // Captura um frame de um Kinect específico
    int getNumDevices() const;  // Retorna o número de Kinects conectados

private:
    freenect_context *f_ctx_;  // Contexto global do Kinect
    std::vector<freenect_device*> kinect_devices_;  // Lista de Kinects conectados
    static std::vector<uint8_t*> videoBuffers_;  // Buffers de imagem de cada Kinect

    static std::unordered_map<freenect_device*, KinectManager*> deviceMap_;  // Mapa de dispositivos para a instância correta

    static void videoCallback(freenect_device *dev, void *video, uint32_t timestamp);
};

#endif  // KINECT_DRIVER_HPP_
