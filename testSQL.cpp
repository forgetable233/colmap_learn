//
// Created by dcr on 23-3-9.
//
#include "mysql.h"

int main() {
//    sfm::SQLHandle::addImage(2);
//    sfm::SQLHandle::addPoint2d(0, 1, 1, 1, 1, 1, 1, 1);
//    std::cout << sfm::SQLHandle::getPoint2dKey(1, 1);
//    sfm::SQLHandle::addEdge(1, 2);
    std::vector<int> image_index;
    std::vector<int> match_index;
    std::vector<int> r;
    std::vector<int> g;
    std::vector<int> b;
    std::vector<double> x;
    std::vector<double> y;
    for (int i = 0; i < 100000; ++i) {
        image_index.push_back(1);
        match_index.push_back(i);
        r.push_back(i);
        g.push_back(i);
        b.push_back(i);
        x.push_back(i);
        y.push_back(i);
    }
    sfm::SQLHandle::addPoint2d(image_index, match_index, x, y, r, g, b);
}