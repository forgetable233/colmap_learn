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

#include "thresholds.h"
#include "camera_model.h"
#include "mysql.h"


namespace sfm {
    class ImgLoader {

    private:
    public:
        int image_num_{IMAGE_NUMBER};

        std::vector<CameraModel> image_features_;

        explicit ImgLoader(const std::string &file_path, std::vector<std::shared_ptr<CameraModel>> &_cameras);

        ~ImgLoader() = default;

        static int ComputeKey(int index1, int index2);
    };
}

#endif //TEST_IMG_LOADER_H
