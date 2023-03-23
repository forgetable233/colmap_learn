//
// Created by dcr on 23-3-8.
//

#include "mysql.h"

sfm::SQLHandle::SQLHandle() {

}

sfm::SQLHandle::~SQLHandle() {

}

bool
sfm::SQLHandle::addPoint2d(std::vector<int> &image_index,
                           std::vector<int> &match_index,
                           std::vector<double> &x,
                           std::vector<double> &y,
                           std::vector<int> &r,
                           std::vector<int> &g,
                           std::vector<int> &b) {

    std::string add_point2d =
            "insert into point2d (image_index, match_index, x, y, r, g, b, point_key) VALUES (?, ?, ?, ?, ?, ?, ?, ?);";
    std::string image_match_count =
            "select count(*) from image_match;";
    std::string add_image_match =
            "insert into image_match (image_match_key, image_index, point_key) VALUES (?, ?, ?);";
    try {
        sql::Driver *driver;
        sql::Connection *connection;

        sql::ResultSet *result;
        int image_match_index;
        int point_match_index;
        int size = image_index.size();
        int final_edge_key;
        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");

        sql::Statement *count = connection->createStatement();
        result = count->executeQuery(image_match_count);
        while (result->next()) {
            image_match_index = result->getInt(1);
        }
        count->close();
        delete count;
        sql::PreparedStatement *preparedStatement = connection->prepareStatement(add_point2d);
        sql::PreparedStatement *match_prepareStatement = connection->prepareStatement(add_image_match);

        for (int i = 0; i < size; ++i) {
            if (sfm::SQLHandle::getPoint2dKey(image_index[i], match_index[i]) != -1) {
                continue;
            }
            int index = sfm::Point2d::ComputePointKey(image_index[i], match_index[i]);
            preparedStatement->setInt(1, image_index[i]);
            preparedStatement->setInt(2, match_index[i]);
            preparedStatement->setDouble(3, x[i]);
            preparedStatement->setDouble(4, y[i]);
            preparedStatement->setInt(5, r[i]);
            preparedStatement->setInt(6, g[i]);
            preparedStatement->setInt(7, b[i]);
            preparedStatement->setInt(8, index);
            preparedStatement->execute();
//            std::cout << "Have added a point the index is " << index << std::endl;

            match_prepareStatement->setInt(1, image_match_index);
            match_prepareStatement->setInt(2, image_index[i]);
            match_prepareStatement->setInt(3, index);
            match_prepareStatement->execute();

            image_match_index++;

            preparedStatement->clearParameters();
            match_prepareStatement->clearParameters();
        }
        connection->close();
        delete preparedStatement;
        delete match_prepareStatement;
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "error in sfm::SQLHandle::addPoint2d " << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "error in sfm::SQLHandle::addPoint2d " << e.what() << std::endl;
        return false;
    }
    return false;
}

int sfm::SQLHandle::getPoint2dKey(int image_index, int match_index) {
    int point_key = -1;
    std::string sql = "select point_key from point2d where image_index = ? and  match_index = ?;";
    try {
        sql::Driver *driver;
        sql::Connection *connection;
        sql::PreparedStatement *preparedStatement;
        sql::ResultSet *result;
        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");
        preparedStatement = connection->prepareStatement(sql);
        preparedStatement->setInt(1, image_index);
        preparedStatement->setInt(2, match_index);
        result = preparedStatement->executeQuery();
        while (result->next()) {
            point_key = result->getInt(1);
            break;
        }
        preparedStatement->close();
        connection->close();
        delete preparedStatement;
        delete connection;
        return point_key;
    } catch (sql::SQLException &e) {
        std::cerr << "sql error in sfm::SQLHandle::getPoint2dKey " << ' ' << e.what() << std::endl;
        return -1;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error in sfm::SQLHandle::getPoint2dKey " << e.what() << std::endl;
        return -1;
    }
    return -1;
}

bool sfm::SQLHandle::addEdge(int image1, int image2, int edge_key) {
    if (image2 == image1) {
        return false;
    }
    std::string sql = "insert into edge (edge_key, image_key1, image_key2) values (?, ?, ?);";
    try {
        sql::Driver *driver;
        sql::Connection *connection;
        sql::PreparedStatement *preparedStatement;
        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");
        if (sfm::SQLHandle::getEdgeKey(image1, image2) != -1) {
            return false;
        }
        preparedStatement = connection->prepareStatement(sql);
        preparedStatement->setInt(1, edge_key);
        preparedStatement->setInt(2, image1);
        preparedStatement->setInt(3, image2);
        preparedStatement->execute();
        preparedStatement->close();
        connection->close();
        delete preparedStatement;
        delete connection;
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "sql error in sfm::SQLHandle::addEdge " << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error in sfm::SQLHandle::addEdge " << e.what() << std::endl;
        return false;
    }
    return false;
}

bool
sfm::SQLHandle::addPointMatch(std::vector<int> &match_index1,
                              std::vector<int> &match_index2,
                              int image1, int image2,
                              int edge_key) {

    if (match_index1.size() != match_index2.size()) {
        std::cerr << "The size is not equal" << std::endl;
        return false;
    }
    const unsigned size = match_index1.size();
    int index = -1;
    std::string count = "select count(*) from point_match;";
    std::string insert_sql = "insert into point_match values (?, ?, ?, ?);";
    try {
        sql::Driver *driver = get_driver_instance();
        sql::Connection *connection;
        sql::ResultSet *resultSet;
        sql::Statement *num;
        sql::PreparedStatement *insert;
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");
        insert = connection->prepareStatement(insert_sql);
        num = connection->createStatement();
        resultSet = num->executeQuery(count);
        while (resultSet->next()) {
            index = resultSet->getInt(1);
        }
        num->close();
        delete num;

        for (int i = 0; i < size; ++i) {
//            int key1 = sfm::Point2d::ComputePointKey(image1, match_index1[i]);
//            int key2 = sfm::Point2d::ComputePointKey(image2, match_index2[i]);
            int key1 = sfm::SQLHandle::getPoint2dKey(image1, match_index1[i]);
            int key2 = sfm::SQLHandle::getPoint2dKey(image2, match_index2[i]);
            if (key1 == key2) {
                std::cerr << "error occurred the key of the two points are equal" << std::endl;
                continue;
            }
            insert->setInt(1, index++);
            insert->setInt(2, key1);
            insert->setInt(3, key2);
            insert->setInt(4, edge_key);
            insert->execute();
            insert->clearParameters();

//            insert->setInt(1, index++);
//            insert->setInt(2, key2);
//            insert->setInt(3, key1);
//            insert->execute();
//            insert->clearParameters();
        }
        insert->close();
        connection->close();
        delete insert;
        delete connection;
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "sql error in sfm::SQLHandle::addPointMatch" << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error in sfm::SQLHandle::addPointMatch " << e.what() << std::endl;
        return false;
    }
    return false;
}

bool sfm::SQLHandle::addImage(int image_index, int row_size, int col_size) {
    MYSQL *sql = nullptr;
    sql = mysql_init(sql);
    if (!sql) {
        std::cerr << "Mysql connect error " << std::endl;
        return false;
    }
    sql = mysql_real_connect(sql, HOST, USER, PWD, DBNAME, PORT, nullptr, 0);
    MYSQL_STMT *stmt = mysql_stmt_init(sql);
    MYSQL_BIND bind[3];
    const char *sql_query = "INSERT INTO image VALUES (?, ?, ?);";
    if (mysql_stmt_prepare(stmt, sql_query, strlen(sql_query))) {
        std::cerr << "initial stmt failed" << std::endl;
        std::cerr << mysql_error(sql) << std::endl;
        std::cerr << mysql_stmt_error(stmt) << std::endl;
        return false;
    }
    bind[0].buffer_type = MYSQL_TYPE_LONG;
    bind[0].buffer = (void *) &image_index;
    bind[0].is_null = nullptr;
    bind[0].length = nullptr;

    bind[1].buffer_type = MYSQL_TYPE_LONG;
    bind[1].buffer = (void *) &row_size;
    bind[1].is_null = nullptr;
    bind[1].length = nullptr;

    bind[2].buffer_type = MYSQL_TYPE_LONG;
    bind[2].buffer = (void *) &col_size;
    bind[2].is_null = nullptr;
    bind[2].length = nullptr;
    if (mysql_stmt_bind_param(stmt, bind)) {
        std::cerr << "Bind value failed" << std::endl;
        std::cerr << mysql_error(sql) << std::endl;
        std::cerr << mysql_stmt_error(stmt) << std::endl;
        return false;
    }
    if (mysql_stmt_execute(stmt)) {
        std::cerr << "Execute failed" << std::endl;
        std::cerr << mysql_error(sql) << std::endl;
        std::cerr << mysql_stmt_error(stmt) << std::endl;
        return false;
    }
    mysql_stmt_close(stmt);
    mysql_close(sql);
    return true;
}

int sfm::SQLHandle::getEdgeKey(int image1, int image2) {
    int edge_key = -1;
    std::string sql = "select edge_key from edge where image_key1 = ? and image_key2 = ?;";
    try {
        sql::Driver *driver;
        sql::Connection *connection;
        sql::PreparedStatement *preparedStatement;
        sql::ResultSet *result;
        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");
        preparedStatement = connection->prepareStatement(sql);
        preparedStatement->setInt(1, image1);
        preparedStatement->setInt(2, image2);
        result = preparedStatement->executeQuery();
        while (result->next()) {
            edge_key = result->getInt(1);
            break;
        }
        preparedStatement->close();
        connection->close();
        delete preparedStatement;
        delete connection;
        return edge_key;
    } catch (sql::SQLException &e) {
        std::cerr << "sql error in int sfm::SQLHandle::getEdgeKey" << ' ' << e.what() << std::endl;
        return -1;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error in int sfm::SQLHandle::getEdgeKey " << e.what() << std::endl;
        return -1;
    }
    return -1;
}

bool sfm::SQLHandle::getAllImage(std::vector<int> &image_key,
                                 std::vector<int> &row_size,
                                 std::vector<int> &col_size) {
    std::string sql = "select image_index, row_size, col_size from image;";
    try {
        sql::Driver *driver;
        sql::Connection *connection;
        sql::ResultSet *resultSet;
        sql::Statement *statement;
        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");

        statement = connection->createStatement();
        resultSet = statement->executeQuery(sql);
        while (resultSet->next()) {
            image_key.push_back(resultSet->getInt(1));
            row_size.push_back(resultSet->getInt(2));
            col_size.push_back(resultSet->getInt(3));
        }
        statement->close();
        connection->close();
        delete statement;
        delete connection;
        delete resultSet;
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "sql error in sfm::SQLHandle::getAllImage" << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error in sfm::SQLHandle::getAllImage " << e.what() << std::endl;
        return false;
    }
    return false;
}

bool sfm::SQLHandle::getEdges(std::vector<Eigen::Vector2i> &edges) {
    std::string sql = "select image_key1, image_key2 from edge;";
    try {
        sql::Driver *driver;
        sql::Connection *connection;
        sql::ResultSet *resultSet;
        sql::Statement *statement;
        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");

        statement = connection->createStatement();
        resultSet = statement->executeQuery(sql);
        while (resultSet->next()) {
            edges.emplace_back(resultSet->getInt(1), resultSet->getInt(2));
        }
        statement->close();
        connection->close();
        delete statement;
        delete connection;
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "sql error in sfm::SQLHandle::getEdges" << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error in sfm::SQLHandle::getEdges " << e.what() << std::endl;
        return false;
    }
    return false;
}

bool sfm::SQLHandle::getMatchPoint(std::vector<cv::Point2i> &temp_point1, std::vector<cv::Point2i> &temp_point2,
                                   std::vector<Eigen::Vector3d> &color1, std::vector<Eigen::Vector3d> &color2,
                                   int edge_key) {
    std::string point1_sql =
            "select x, y, r, g, b from point2d, point_match where point_match.edge_key = ? and point_match.point_key1 = point2d.point_key;";
    std::string point2_sql =
            "select x, y, r, g, b from point2d, point_match where point_match.edge_key = ? and point_match.point_key2 = point2d.point_key;";

    try {
        sql::Driver *driver;
        sql::Connection *connection;
        sql::PreparedStatement *point1;
        sql::PreparedStatement *point2;
        sql::ResultSet *resultSet;

        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");

        point1 = connection->prepareStatement(point1_sql);
        point2 = connection->prepareStatement(point2_sql);
        point1->setInt(1, edge_key);
        point2->setInt(1, edge_key);

        resultSet = point1->executeQuery();
        while (resultSet->next()) {
            temp_point1.emplace_back(resultSet->getDouble(1),
                                     resultSet->getDouble(2));
            color1.emplace_back(resultSet->getInt(3),
                                resultSet->getInt(4),
                                resultSet->getInt(5));
        }

        resultSet = point2->executeQuery();
        while (resultSet->next()) {
            temp_point2.emplace_back(resultSet->getDouble(1),
                                     resultSet->getDouble(2));
            color2.emplace_back(resultSet->getInt(3),
                                resultSet->getInt(4),
                                resultSet->getInt(5));
        }
        resultSet->close();
        point2->close();
        point1->close();
        connection->close();
        delete resultSet;
        delete point1;
        delete point2;
        delete connection;
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "sql error in sfm::SQLHandle::getMatchPoint" << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error in sfm::SQLHandle::getMatchPoint " << e.what() << std::endl;
        return false;
    }
    return false;
}

bool sfm::SQLHandle::addKeyPoint(int image_num,
                                 std::vector<float> &x,
                                 std::vector<float> &y,
                                 std::vector<float> &size,
                                 std::vector<float> &angle,
                                 std::vector<int> &response,
                                 std::vector<int> &octave,
                                 std::vector<int> &class_id,
                                 std::vector<int> &r,
                                 std::vector<int> &g,
                                 std::vector<int> &b) {
    int point_key_size;

    try {
        sql::Driver *driver;
        sql::Connection *connection;
        sql::Statement *size_stmt;
        sql::PreparedStatement *insert_stmt;
        sql::ResultSet *resultSet;

        std::string num_sql = "select count(*) from key_point;";
        std::string insert_sql = "insert into key_point values (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";

        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");
        size_stmt = connection->createStatement();
        insert_stmt = connection->prepareStatement(insert_sql);

        resultSet = size_stmt->executeQuery(num_sql);
        resultSet->next();
        point_key_size = resultSet->getInt(1);

        size_stmt->close();
        delete size_stmt;
        delete resultSet;

        for (int i = 0; i < x.size(); ++i) {
            insert_stmt->setInt(1, point_key_size++);
            insert_stmt->setDouble(2, x[i]);
            insert_stmt->setDouble(3, y[i]);
            insert_stmt->setDouble(4, size[i]);
            insert_stmt->setDouble(5, angle[i]);
            insert_stmt->setDouble(6, response[i]);
            insert_stmt->setInt(7, octave[i]);
            insert_stmt->setInt(8, class_id[i]);
            insert_stmt->setInt(9, image_num);
            insert_stmt->setInt(10, r[i]);
            insert_stmt->setInt(11, g[i]);
            insert_stmt->setInt(12, b[i]);
            insert_stmt->execute();
            insert_stmt->clearParameters();
        }
//        std::cout << x.size() << std::endl;
        insert_stmt->close();
        connection->close();
        delete insert_stmt;
        delete connection;
        std::cout << "add key points succeed" << std::endl;
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "sql error in sfm::SQLHandle::addKeyPoint" << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error in sfm::SQLHandle::addKeyPoint " << e.what() << std::endl;
        return false;
    }
    return false;
}

bool
sfm::SQLHandle::getAllKeyPoints(int &image_num,
                                std::vector<float> &x,
                                std::vector<float> &y,
                                std::vector<float> &size,
                                std::vector<float> &angle,
                                std::vector<int> &response,
                                std::vector<int> &octave,
                                std::vector<int> &class_id,
                                std::vector<int> &r,
                                std::vector<int> &g,
                                std::vector<int> &b) {
    std::string sql =
            "select x, y, size, angle, response, octave, class_id, r, g, b from key_point where image_number = ?;";

    try {
        sql::Driver *driver;
        sql::Connection *connection;
        sql::PreparedStatement *stmt;
        sql::ResultSet *resultSet;

        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");

        stmt = connection->prepareStatement(sql);
        stmt->setInt(1, image_num);

        resultSet = stmt->executeQuery();
        while (resultSet->next()) {
            x.emplace_back(resultSet->getDouble(1));
            y.emplace_back(resultSet->getDouble(2));
            size.emplace_back(resultSet->getDouble(3));
            angle.emplace_back(resultSet->getDouble(4));
            response.emplace_back(resultSet->getInt(5));
            octave.emplace_back(resultSet->getInt(6));
            class_id.emplace_back(resultSet->getInt(7));
            r.emplace_back(resultSet->getInt(8));
            g.emplace_back(resultSet->getInt(9));
            b.emplace_back(resultSet->getInt(10));
        }

        resultSet->close();
        stmt->close();
        connection->close();
        delete resultSet;
        delete stmt;
        delete connection;
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "sql error in sfm::SQLHandle::getAllKeyPoints" << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error in sfm::SQLHandle::getAllKeyPoints " << e.what() << std::endl;
        return false;
    }
    return false;
}


