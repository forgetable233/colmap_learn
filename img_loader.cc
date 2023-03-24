//
// Created by D1456 on 2022/7/5.
//

#include "img_loader.h"

namespace sfm {
    ImgLoader::ImgLoader(const std::string &file_path,
                         std::vector<std::shared_ptr<CameraModel>> &_cameras,
                         bool use_sql) {
        std::vector<int> image_keys;
        std::vector<int> row_sizes;
        std::vector<int> col_sizes;
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
        int i = 0;
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
//            cv::pyrDown(image,
//                        down_sample_2,
//                        cv::Size{image.cols / 2, image.rows / 2});
//            cv::pyrDown(down_sample_2,
//                        down_sample_4,
//                        cv::Size{down_sample_2.cols / 2, down_sample_2.rows / 2});
//            cv::pyrDown(down_sample_4,
//                        down_sample_8,
//                        cv::Size{down_sample_4.cols / 2, down_sample_4.rows / 2});

            if (use_sql) {
                sfm::SQLHandle::getAllImage(image_keys, row_sizes, col_sizes);
                _cameras.emplace_back(std::make_shared<sfm::CameraModel>(image_keys[i], row_sizes[i], col_sizes[i], grey_image));
//                std::cout << "Have successfully read " << this->image_num_ << " images" << std::endl;
                i++;
            } else {
                _cameras.emplace_back(std::make_shared<CameraModel>(image, number));
                sfm::SQLHandle::addImage(number, image.rows, image.cols);
            }
            number++;
            temp_path.erase(temp_path.length() - strlen(ptr->d_name), strlen(ptr->d_name));
        }
        for (auto &temp : _cameras) {
            std::cout << temp->key_points_.size() << std::endl;
        }
        std::cout << "Have successfully read " << this->image_num_ << " images" << std::endl;
    }

    int ImgLoader::ComputeKey(int index1, int index2) {
        int key = (index1 > index2 ? index1 : index2);
        key = key * 100 + (index1 + index2 - key);
        return key;
    }
}
