//
// Created by D1456 on 2022/7/5.
//

#include "img_loader.h"
#include "camera_model.h"

namespace sfm {
    ImgLoader::ImgLoader(const std::string &file_path,
                         std::vector<std::shared_ptr<CameraModel>> &_cameras,
                         bool use_sql) {
        std::vector<int> image_keys;
        if (use_sql) {
            sfm::SQLHandle::getAllImageKey(image_keys);
            for (int i = 0; i < image_keys.size(); ++i) {
                _cameras.emplace_back(std::make_shared<sfm::CameraModel>(image_keys[i]));
                std::cout << "Have successfully read " << this->image_num_ << " images" << std::endl;

            }
        }
        int number{};

        std::string image_path(file_path, 0, file_path.size() - 1);
        std::string temp_path{};

        DIR *pDIR{};
        struct dirent *ptr{};

        cv::Mat descriptors{};

        std::vector<cv::KeyPoint> key_points{};

        cv::Mat image{};
        cv::Mat down_sample_2{};
        cv::Mat down_sample_4{};
        cv::Mat down_sample_8{};
        cv::Mat grey_image{};

        if (!(pDIR = opendir(file_path.c_str()))) {
            std::cerr << "error occurred!!! unable to find the target file" << std::endl;
            std::cout << file_path << std::endl;
        }
        temp_path.assign(file_path);
        while ((ptr = readdir(pDIR)) != nullptr && number < image_num_) {
            if (strcmp(ptr->d_name, "..") == 0 || strcmp(ptr->d_name, ".") == 0) {
                continue;
            }
            temp_path.append(ptr->d_name);
            image = cv::imread(temp_path);

            if (image.empty()) {
                cerr << "unable to load the target image" << endl;
                return;
            }
            /** 下采样 提升运行速度**/
            cv::pyrDown(image,
                        down_sample_2,
                        cv::Size{image.cols / 2, image.rows / 2});
            cv::pyrDown(down_sample_2,
                        down_sample_4,
                        cv::Size{down_sample_2.cols / 2, down_sample_2.rows / 2});
            cv::pyrDown(down_sample_4,
                        down_sample_8,
                        cv::Size{down_sample_4.cols / 2, down_sample_4.rows / 2});
            cv::cvtColor(down_sample_2, grey_image, cv::COLOR_BGR2GRAY);

            _cameras.emplace_back(std::make_shared<CameraModel>(grey_image, number));
//            sfm::SQLHandle::addImage(number);
            number++;
            temp_path.erase(temp_path.length() - strlen(ptr->d_name), strlen(ptr->d_name));
        }
        std::cout << "Have successfully read " << this->image_num_ << " images" << std::endl;
    }

    int ImgLoader::ComputeKey(int index1, int index2) {
        int key = (index1 > index2 ? index1 : index2);
        key = key * 100 + (index1 + index2 - key);
        return key;
    }
}
