#include <mapping/map_io.h>

Map_IO::Map_IO()
{
}

bool Map_IO::saveMap(const std::string &path_img,
                     const std::string &path_metadata,
                           nav_msgs::OccupancyGrid &map)
{
    // ** Write metadata
    std::ofstream file;
    file.open(path_metadata);
    file << map.info.height<<" "
         <<map.info.width <<" "
         <<map.info.resolution << " "
         <<map.info.origin.position.x << " "
         <<map.info.origin.position.y << " "
         <<map.header.frame_id;

    file.close();

    // ** Write image
    std::size_t rows = map.info.height;
    std::size_t cols = map.info.width;
//    std::vector<int8_t> v(map.data); // Avoid big copy but not sure if the initial map gets affected. Test!!
    const cv::Mat img_map (rows, cols, CV_8UC1, &map.data[0]);

    return cv::imwrite(path_img, img_map);
}

bool Map_IO::loadMap(const std::string &path_img,
                     const std::string &path_metadata,
                     nav_msgs::OccupancyGrid &map)
{
    // ** Read metadata
    std::ifstream file;
    file.open(path_metadata);
    int rows, cols, px, py;
    double resolution;
    std::string frame_id;

    file >> rows >> cols >> resolution >> px >> py >> frame_id;
    file.close();

    // ** Read image and get data as array
    cv::Mat img = cv::imread(path_img);
    cv::Mat img2 = img.reshape(0,1);
    cv::imshow("IMG",img);
    cv::waitKey();
    v_.reserve(img2.cols);
    memcpy(&(v_[0]), img2.data, img2.cols*sizeof(int8_t));

    map.data.resize(rows*cols);
    map.info.height = rows;
    map.info.width = cols;
    map.info.resolution = resolution;
    map.info.origin.position.x = px;
    map.info.origin.position.y = py;
    map.header.frame_id = frame_id;

    return true;
}
