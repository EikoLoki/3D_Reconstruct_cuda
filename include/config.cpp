#include <config.h>

std::shared_ptr<Config> Config::config_ = nullptr;

Config::~Config(){
    if(config_->file_.isOpened()){
        config_->file_.release();
    }
}

void Config::getParameterFile(const std::string& filename){
    if ( config_ == nullptr ){
        config_ = std::shared_ptr<Config>(new Config);
    }
    config_->file_ = cv::FileStorage( filename, cv::FileStorage::READ );
    if ( config_->file_.isOpened() == false ){
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();
        return;
    }
}
