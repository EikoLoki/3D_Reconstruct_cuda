#ifndef CONFIG_H
#define CONFIG_H

#include <common.h>

class Config{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;
    Config(){} //singleton

public:
    ~Config();
    static void getParameterFile(const std::string& filename);

    template <typename T>
    static T get(const std::string& key){
        T data;
        config_->file_[key] >> data;
        return data;
    }
};



#endif
