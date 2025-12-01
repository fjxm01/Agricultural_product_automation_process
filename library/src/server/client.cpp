#include "client.h"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>

size_t Client::WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

Client::Client(const std::string& server_url)
    : server_url_(server_url) {
    curl_ = curl_easy_init();
    if (!curl_) {
        std::cerr << "[ERROR] curl_easy_init() failed during Client initialization." << std::endl;
    }

    curl_easy_setopt(curl_, CURLOPT_URL, server_url_.c_str());
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, Client::WriteCallback);
    
    std::cout << "Client initialized with URL: " << server_url_ << std::endl;
}

Client::~Client() {
    if (curl_) {
        curl_easy_cleanup(curl_);
    }
}

std::string Client::sendImageForDetection(const std::string& image_path) {
    if (!curl_) {
        std::cerr << "[ERROR] CURL handle is invalid." << std::endl;
        return "";
    }
    
    std::string response_data;
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    
    if (image.empty()) {
        std::cerr << "[ERROR] Could not read the image: " << image_path << std::endl;
        return "";
    }

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80); 

    std::vector<uchar> buffer;
    if (!cv::imencode(".jpg", image, buffer, compression_params)) {
        std::cerr << "[ERROR] Image encoding failed." << std::endl;
        return "";
    }

    struct curl_httppost *formpost = NULL;
    struct curl_httppost *lastptr = NULL;
    CURLcode res;

    curl_formadd(&formpost, &lastptr,
                 CURLFORM_COPYNAME, "file",
                 CURLFORM_BUFFER, "image.jpg",
                 CURLFORM_BUFFERPTR, buffer.data(),
                 CURLFORM_BUFFERLENGTH, buffer.size(),
                 CURLFORM_END);

    curl_easy_setopt(curl_, CURLOPT_HTTPPOST, formpost);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response_data); 
    
    std::cout << "Sending " << image_path << " (" << buffer.size() << " bytes) to " << server_url_ << std::endl;
    
    res = curl_easy_perform(curl_);
    
    if (res != CURLE_OK) {
        std::cerr << "[ERROR] curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        response_data = "";
    } else {
        long http_code = 0;
        curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
        
        if (http_code == 200) {
            std::cout << "[SUCCESS] Client responded with 200 OK." << std::endl;
        } else {
            std::cerr << "[ERROR] Server returned status code: " << http_code << std::endl;
            response_data = "";
        }
    }

    curl_formfree(formpost);
    curl_easy_setopt(curl_, CURLOPT_HTTPPOST, NULL); 
    
    return response_data;
}