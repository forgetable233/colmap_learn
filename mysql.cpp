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

    std::string sql =
            "insert into point2d (image_index, match_index, x, y, r, g, b, point_key) VALUES (?, ?, ?, ?, ?, ?, ?, ?);";
    std::string count_sql = "select count(*) from point2d;";
    std::string image_match_count = "select count(*) from image_match;";
    std::string add_image_match = "insert into image_match (image_match_key, image_index, point_key) VALUES (?, ?, ?);";
    try {
        sql::Driver *driver;
        sql::Connection *connection;

        sql::ResultSet *result;
        int index;
        int image_match_index;
        int size = image_index.size();
        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");

        sql::Statement *count = connection->createStatement();
        result = count->executeQuery(count_sql);
        while (result->next()) {
            index = result->getInt(1);
        }
        result = count->executeQuery(image_match_count);
        while (result->next()) {
            image_match_index = result->getInt(1);
        }
        count->close();

        for (int i = 0; i < size; ++i) {
            if (sfm::SQLHandle::getPoint2dKey(image_index[i], match_index[i]) != -1) {
                continue;
            }
            sql::PreparedStatement *preparedStatement = connection->prepareStatement(sql);
            sql::PreparedStatement *match_prepareStatement = connection->prepareStatement(add_image_match);
            preparedStatement->setInt(1, image_index[i]);
            preparedStatement->setInt(2, match_index[i]);
            preparedStatement->setDouble(3, x[i]);
            preparedStatement->setDouble(4, y[i]);
            preparedStatement->setInt(5, r[i]);
            preparedStatement->setInt(6, g[i]);
            preparedStatement->setInt(7, b[i]);
            preparedStatement->setInt(8, index);
            preparedStatement->execute();

            match_prepareStatement->setInt(1, image_match_index);
            match_prepareStatement->setInt(2, image_index[i]);
            match_prepareStatement->setInt(3, index);
            match_prepareStatement->execute();
            index++;
            image_match_index++;

//            preparedStatement->clearParameters();
//            match_prepareStatement->clearParameters();
            preparedStatement->close();
            match_prepareStatement->close();
        }
        connection->close();
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "链接异常" << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error: " << e.what() << std::endl;
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
            connection->close();
            return point_key;
        }
        preparedStatement->close();
        connection->close();
    } catch (sql::SQLException &e) {
        std::cerr << "链接异常" << ' ' << e.what() << std::endl;
        return -1;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error: " << e.what() << std::endl;
        return -1;
    }
    return -1;
}

bool sfm::SQLHandle::addEdge(int image1, int image2) {
    if (image2 == image1) {
        return false;
    }
    int index = -1;
    std::string sql = "insert into edge (edge_key, image_key1, image_key2) values (?, ?, ?);";
    std::string count_sql = "select count(*) from edge;";
    try {
        sql::Driver *driver;
        sql::Connection *connection;
        sql::PreparedStatement *preparedStatement;
        sql::ResultSet *result;
        sql::Statement *count;
        driver = get_driver_instance();
        connection = driver->connect(HOST, USER, PWD);
        connection->setSchema("SFM");
        count = connection->createStatement();
        result = count->executeQuery(count_sql);
        while (result->next()) {
            index = result->getInt(1);
        }
        preparedStatement = connection->prepareStatement(sql);
        preparedStatement->setInt(1, index);
        preparedStatement->setInt(2, image1);
        preparedStatement->setInt(3, image2);
        preparedStatement->execute();
        preparedStatement->close();
        connection->close();
        return true;
    } catch (sql::SQLException &e) {
        std::cerr << "链接异常" << ' ' << e.what() << std::endl;
        return false;
    } catch (std::runtime_error &e) {
        std::cerr << "runtime error: " << e.what() << std::endl;
        return false;
    }
    return false;
}

bool sfm::SQLHandle::addImage(int image_index) {
    MYSQL *sql = nullptr;
    sql = mysql_init(sql);
    if (!sql) {
        std::cerr << "Mysql connect error " << std::endl;
        return false;
    }
    sql = mysql_real_connect(sql, HOST, USER, PWD, DBNAME, PORT, nullptr, 0);
    MYSQL_STMT *stmt = mysql_stmt_init(sql);
    MYSQL_BIND bind{nullptr};
    const char *sql_query = "INSERT INTO image VALUES (?);";
    if (mysql_stmt_prepare(stmt, sql_query, strlen(sql_query))) {
        std::cerr << "initial stmt failed" << std::endl;
        std::cerr << mysql_error(sql) << std::endl;
        std::cerr << mysql_stmt_error(stmt) << std::endl;
        return false;
    }
    bind.buffer_type = MYSQL_TYPE_LONG;
    bind.buffer = (void *) &image_index;
    bind.is_null = nullptr;
    bind.length = nullptr;
    if (mysql_stmt_bind_param(stmt, &bind)) {
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
