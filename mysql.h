//
// Created by dcr on 23-3-8.
//

#ifndef TEST_MYSQL_H
#define TEST_MYSQL_H

#include <jni.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <mysql/mysql.h>
#include <iostream>
#include <cstring>
#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <prepared_statement.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>


#include "camera_model.h"
//#include "edge.h"
#include "point2d.h"

#define USER "dcr"
#define PWD "d21700499"
//#define URL ""
#define HOST "localhost"
#define DBNAME "SFM"
#define PORT 3306

/** 还得实现连接池
 * 目前的查询速度很慢**/
namespace sfm {
    struct Point {
        Point(double x, double y, int r, int g, int b) {
            point(0, 0) = x;
            point(1, 0) = y;
            color(0, 0) = r;
            color(1, 0) = g;
            color(2, 0) = b;
        }
        Eigen::Vector2d point{};
        Eigen::Vector3i color{};
    };
    class SQLHandle {
    private:
        static std::string user;

        static std::string password;

        static std::string url;

    public:
        SQLHandle();

        ~SQLHandle();

        static bool
        addPoint2d(std::vector<int> &image_index, std::vector<int> &match_index, std::vector<double> &x,
                   std::vector<double> &y, std::vector<int> &r, std::vector<int> &g, std::vector<int> &b);

        static int getPoint2dKey(int image_index, int match_index);

        static int getEdgeKey(int image1, int image2);

        static bool
        addPointMatch(std::vector<int> &match_index1,
                      std::vector<int> &match_index2,
                      int image1,
                      int image2,
                      int edge_key);

        static bool addEdge(int image1, int image2, int edge_key);

        static bool addImage(int image_index, int row_size, int col_size);

        static bool addKeyPoint(int image_num,
                                std::vector<float> &x,
                                std::vector<float> &y,
                                std::vector<float> &size,
                                std::vector<float> &angle,
                                std::vector<int> &response,
                                std::vector<int> &octave,
                                std::vector<int> &class_id,
                                std::vector<int> &r,
                                std::vector<int> &g,
                                std::vector<int> &b);

        static bool
        getAllKeyPoints(int &image_num,
                        std::vector<float> &x,
                        std::vector<float> &y,
                        std::vector<float> &size,
                        std::vector<float> &angle,
                        std::vector<int> &response,
                        std::vector<int> &octave,
                        std::vector<int> &class_id,
                        std::vector<int> &r,
                        std::vector<int> &g,
                        std::vector<int> &b);

        static bool getAllImage(std::vector<int> &image_key, std::vector<int> &row_size, std::vector<int> &col_size);

        static bool getEdges(std::vector<Eigen::Vector2i> &edges);

        static bool getMatchPoint(std::vector<cv::Point2i> &temp_point1,
                                  std::vector<cv::Point2i> &temp_point2,
                                  std::vector<Eigen::Vector3d> &color1,
                                  std::vector<Eigen::Vector3d> &color2,
                                  int edge_key);
    };
}


#endif //TEST_MYSQL_H
