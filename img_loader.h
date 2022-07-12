//
// Created by D1456 on 2022/7/5.
//

#ifndef TEST_IMG_LOADER_H
#define TEST_IMG_LOADER_H

#include <iostream>
#include <streambuf>
#include <vector>
#include <map>
#include <cstring>
#include <sys/types.h>
#include <dirent.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Eigen>

#include "threholds.h"
#include "camera_model.h"


namespace sfm {
    class ImgLoader {

    private:
    public:
        int image_num_{10};

        std::vector<CameraModel> image_features_;

//        std::vector<sfm::ImagePair> edges_;

        std::map<int, sfm::ImagePair> edges_;

        explicit ImgLoader(const std::string &file_path, std::vector<sfm::CameraModel> &_cameras);

        ~ImgLoader() = default;

        static int ComputeKey(int index1, int index2);
    };
}

#endif //TEST_IMG_LOADER_H
