#include "mult_kinect_ros2/kinect_driver.hpp"
#include <iostream>

std::vector<uint8_t*> KinectManager::videoBuffers_;
std::unordered_map<freenect_device*, KinectManager*> KinectManager::deviceMap_;  // Inicializando o mapa

KinectManager::KinectManager() : f_ctx_(nullptr) {}

KinectManager::~KinectManager() {
    for (auto dev : kinect_devices_) {
        freenect_stop_video(dev);
        freenect_close_device(dev);
        deviceMap_.erase(dev);  // Removendo do mapa ao destruir
    }
    freenect_shutdown(f_ctx_);
}

bool KinectManager::initialize() {
    if (freenect_init(&f_ctx_, NULL) < 0) {
        std::cerr << "Erro ao inicializar o contexto do Kinect" << std::endl;
        return false;
    }

    int num_devices = freenect_num_devices(f_ctx_);
    if (num_devices < 2) {
        std::cerr << "Menos de 2 dispositivos Kinect conectados" << std::endl;
        return false;
    }

    videoBuffers_.resize(num_devices, nullptr);
    
    std::cout << "Número de dispositivos Kinect encontrados: " << num_devices << std::endl;

    for (int i = 0; i < num_devices; ++i) {
        freenect_device* dev;
        if (freenect_open_device(f_ctx_, &dev, i) < 0) {
            std::cerr << "Não foi possível abrir o dispositivo Kinect " << i << std::endl;
            continue;
        }
        std::cout << "Kinect aberto no índice: " << i << std::endl;

        freenect_set_video_mode(dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
        freenect_set_video_callback(dev, videoCallback);
        freenect_start_video(dev);  // Certificando-se de que o vídeo inicia para cada Kinect

        kinect_devices_.push_back(dev);
        deviceMap_[dev] = this;  // Associa o dispositivo ao mapa
    }

    return true;
}


void KinectManager::videoCallback(freenect_device *dev, void *video, uint32_t timestamp) {
    if (deviceMap_.find(dev) == deviceMap_.end()) {
        std::cerr << "Erro: dispositivo Kinect não encontrado no mapa!" << std::endl;
        return;
    }

    KinectManager* instance = deviceMap_[dev];  // Obtém a instância correta

    for (size_t i = 0; i < instance->kinect_devices_.size(); ++i) {
        if (instance->kinect_devices_[i] == dev) {
            instance->videoBuffers_[i] = static_cast<uint8_t*>(video);
            std::cout << "Frame atualizado para Kinect " << i << std::endl;
            break;
        }
    }
}

bool KinectManager::captureFrame(int device_index, cv::Mat &image) {
    freenect_process_events(f_ctx_);  // Processar eventos ANTES de tentar capturar

    if (device_index >= kinect_devices_.size() || videoBuffers_[device_index] == nullptr) {
        std::cerr << "Erro ao capturar frame do Kinect " << device_index << std::endl;
        return false;
    }

    image = cv::Mat(480, 640, CV_8UC3, videoBuffers_[device_index]);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    return true;
}

int KinectManager::getNumDevices() const {
    return kinect_devices_.size();
}
