//
// Created by dcr on 22-7-25.
//

#include "point3d.h"

sfm::Point3d::Point3d(int key, Eigen::Vector3d _point) {

}

sfm::Point3d::Point3d(Eigen::Vector3d _point) {
    world_point_ = std::move(_point);
}
