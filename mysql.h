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


#include "edge.h"

#define USER "dcr"
#define PWD "d21700499"
//#define URL ""
#define HOST "localhost"
#define DBNAME "SFM"
#define PORT 3306

/** 还得实现连接池
 * 目前的查询速度很慢**/
namespace sfm {
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
                   std::vector<double> &y,
                   std::vector<int> &r, std::vector<int> &g, std::vector<int> &b, int edge_key);

        static int getPoint2dKey(int image_index, int match_index);

        static int getEdgeKey(int image1, int image2);

        static bool addPointMatch(std::vector<int> &match1_, std::vector<int> &match2_, int image1, int image2);

        static bool addEdge(int image1, int image2);

        static bool addImage(int image_index);

        static bool getAllImageKey(std::vector<int> &image_key);

        static bool getEdges(std::vector<Eigen::Vector2i> &edges);

        static bool getMatchPoint(std::vector<sfm::Point> &temp_point);
    };
}


#endif //TEST_MYSQL_H
