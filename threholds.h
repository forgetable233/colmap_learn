//
// Created by D1456 on 2022/7/5.
//

#ifndef TEST_THREHOLDS_H
#define TEST_THREHOLDS_H

#include <vector>
#include <pcl/pcl_base.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Eigen>

#define ESSENTIAL_MATRIX_INLIERS_THRESHOLD 0.5
#define FUNDAMENTAL_MATRIX_INLIERS_THRESHOLD 0.53
#define ESSENTIAL_INLIER_THRESHOLD 1.0
#define FUNDAMENTAL_INLIER_THRESHOLD 1.0

namespace sfm {
    struct ImagePair {
        std::vector<cv::DMatch> matches{};

        cv::Mat_<double> essential_matrix{3, 3, 0};
        cv::Mat_<double> fundamental_matrix{3, 3, 0};
        cv::Mat_<double> R{3, 3, 0};

        cv::Mat_<double> t{3, 1, 0};

        ImagePair(std::vector<cv::DMatch> _matches,
                  const cv::Mat_<double>& e_m,
                  const cv::Mat_<double>& f_m,
                  cv::Mat_<double> _R,
                  cv::Mat_<double> _t) {
            matches.assign(_matches.begin(), _matches.end());

            e_m.copyTo(essential_matrix);
            f_m.copyTo(fundamental_matrix);
            _R.copyTo(R);
            _t.copyTo(t);
        }
    };

    struct FeaturePoint {
        int key;

    };
}


#endif //TEST_THREHOLDS_H
