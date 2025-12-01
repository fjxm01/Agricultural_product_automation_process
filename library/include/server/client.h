#ifndef CLIENT_H
#define CLIENT_H

#include <string>
#include <curl/curl.h>

class Client
{
public:
    Client(const std::string &server_url);
    ~Client(); 

    std::string sendImageForDetection(const std::string &image_path);

private:
    std::string server_url_;
    CURL *curl_; 

    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
};

#endif 