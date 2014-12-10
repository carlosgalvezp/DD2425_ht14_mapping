#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include <geometry_msgs/Pose2D.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_srv_msgs/LaserScanner.h>
#include <ras_srv_msgs/IRData.h>
#include <ras_utils/ras_utils.h>
#include <ras_utils/ras_sensor_utils.h>

#include <mapping/cell.h>
#include <mapping/map.h>

#include <math.h>
#include <vector>
#include <string>
#include <algorithm>

#define MAX_SHORT_SENSOR_DISTANCE   25
#define MAX_LONG_SENSOR_DISTANCE    25

#define SHORT_SENSOR_DISTANCE_FROM_CENTER           10.68
#define SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD -0.53839

#define LONG_SENSOR_DISTANCE_FROM_CENTER        1.75
#define LONG_FRONT_SENSOR_ANGLE_FROM_FORWARD    0       //TODO: Not entirely true...

#define METRIC_CONVERTER    100.0 // To convert meters from odometry to cm in map (might redo this)

#define FREE_AREA_LIMIT 8

#define SENSOR_READING_PART_DISTANCE_BLOCK_SIZE 0.5 // Value 0.1 means that when working through the linear function "Sensor start to sensor reading", we look 0.1 cm at a time. Lower value leads to more computational costs but better precission in the sense of not missing a "hit node"

#define SENSOR_OLD_VALUE_DISTANCE_LIMIT 4.0 // 10 cm means only fill the wall cells if the last 2 values were within 10 cm from eachother.

#define FILL_FREE_CELL_TOP_LIMIT 12 // How big "unkonwn area", that we fill as known, if it is enclosed by wall cells or free cells

#define SENSOR_FREE_AREA_OFFSET_FROM_WALL 4 // To make sure we do not overwrite blocked cells right after setting them

#define MAX_SENSOR_WALL_TO_FREE_PER_ITERATION 1

class MapHandler {
public:

    MapHandler(){}

    MapHandler(Map map) :
        map_(map),
        robot_angle_(0),
        robot_x_pos_(0),
        robot_y_pos_(0),
        robot_x_pos_offset_(map.getWidth() / 2),
        robot_y_pos_offset_(map.getHeight() / 2),
        prev_value_short_sensors_(4, PrevValue(-10000, -10000)),
        prev_value_long_sensors_(2, PrevValue(-10000, -10000))
    {
    }

    void update(const geometry_msgs::Pose2D::ConstPtr &odo_data,
                const ras_srv_msgs::IRData::ConstPtr &adc_data,
                const ras_srv_msgs::LaserScanner::ConstPtr &las_data,
                bool new_adc_recieved,
                bool new_laser_recieved)
    {

        // Retrieve the data
        robot_x_pos_ = odo_data->x * METRIC_CONVERTER + robot_x_pos_offset_;
        robot_y_pos_ = odo_data->y * METRIC_CONVERTER + robot_y_pos_offset_;
        robot_angle_ = odo_data->theta;



        SensorDistances sd(adc_data->front,
           adc_data->back,
           adc_data->front_right,
           adc_data->back_right,
           adc_data->front_left,
           adc_data->back_left);


        // Update where we detect walls based on sensor readings
//        updateOccupiedAreaLongSensor(sd.front_+12, true);
       // updateOccupiedAreaLongSensor(dist_back_large_range, false);


        if(new_adc_recieved){

            updateOccupiedAreaShortSensor(sd.right_front_, true, true);
            updateOccupiedAreaShortSensor(sd.right_back_, true, false);
            updateOccupiedAreaShortSensor(sd.left_front_, false, true);
            updateOccupiedAreaShortSensor(sd.left_back_, false, false);




            updateFreeAreaShortSensor(sd.right_front_, true, true);
            updateFreeAreaShortSensor(sd.right_back_, true, false);
            updateFreeAreaShortSensor(sd.left_front_, false, true);
            updateFreeAreaShortSensor(sd.left_back_, false, false);


            updateFreeAreaBetweenSensors(sd.right_front_, sd.right_back_, true);
            updateFreeAreaBetweenSensors(sd.left_front_, sd.left_back_, false);

        }

        if(new_laser_recieved){
          //  updateAreaLaser(*las_data, false);
            updateAreaLaser(*las_data, true);
        }

        updateFreeAreaUsingRobotPos();

       // updateFreeAreaLongSensor(sd.front_, true);

//        std::vector<std::string> names = {"robot_x_pos", "robot_y_pos", "robot_angle"};
//        std::vector<double> values = {robot_x_pos_, robot_y_pos_, robot_angle_};
//        RAS_Utils::print(names, values);

    }

    std::vector<int8_t> & getMap()
    {
        return map_.getSimpleMapVector();
    }


    std::vector<int8_t> & getThickMap()
    {
        return map_.getSimpleThickMapVector();
    }

    std::vector<long> & getCostMap()
    {
        return map_.getCostVector();
    }

    int getHeight()
    {
        return map_.getHeight();
    }

    int getWidth()
    {
        return map_.getWidth();
    }

    double getCellSize()
    {
        return map_.getCellSize();
    }


private:
    struct SensorDistances
    {
        double back_;
        double front_;
        double right_front_;
        double right_back_;
        double left_front_;
        double left_back_;
        SensorDistances(){}
        SensorDistances(double front_reading, double back_reading, double right_front_reding, double right_back_reading, double left_front_reading, double left_back_reading)
            : front_(front_reading*100.0),
              back_(back_reading*100.0),
              right_front_(right_front_reding*100.0),
              right_back_(right_back_reading*100.0),
              left_front_(left_front_reading*100.0),
              left_back_(left_back_reading*100.0)
        {
        }
    };


    struct PrevValue {
        double x;
        double y;

        PrevValue(double x, double y) : x(x), y(y){}
    };

    std::vector<PrevValue> prev_value_short_sensors_;
    std::vector<PrevValue> prev_value_long_sensors_;

    Map map_;

    int height_;
    int width_;

    double robot_angle_;
    double robot_x_pos_;
    double robot_y_pos_;

    double robot_x_pos_offset_;
    double robot_y_pos_offset_;

    typedef std::vector< std::vector<Cell> > CellMatrix;
    typedef std::vector<Cell> CellVector;


    void updateAreaLaser(const ras_srv_msgs::LaserScanner & laser_scanner, bool paint_wall)
    {
        for(int i = 0; i < laser_scanner.scan.size(); i++)
        {
            const ras_srv_msgs::LaserLine & line = laser_scanner.scan[i];
            double from_x = line.from.x * METRIC_CONVERTER + robot_x_pos_offset_;
            double from_y = line.from.y * METRIC_CONVERTER + robot_y_pos_offset_;
            double to_x = line.to.x * METRIC_CONVERTER + robot_x_pos_offset_;
            double to_y = line.to.y * METRIC_CONVERTER + robot_y_pos_offset_;
            bool is_wall = line.is_wall;

            if(is_wall && paint_wall || !is_wall && !paint_wall)
            {
                // Only calculate if we currently want to paint wall and it actually is wall or vice versa

                double line_dist = getPointDistance(from_x , from_y, to_x, to_y);
                double laser_dist_from_center = getPointDistance(robot_x_pos_, robot_y_pos_, from_x, from_y);
                double laser_angle_center_offset = getAngleFromPoints(robot_x_pos_, robot_y_pos_, from_x, from_y) - robot_angle_;
                if(is_wall)
                {
                    updateOccupiedArea(line_dist, 0, laser_angle_center_offset, 1000, laser_dist_from_center);

                } else if(!is_wall)
                {
                     updateFreeArea(line_dist, 0, laser_angle_center_offset, 1000 , laser_dist_from_center);
                }
            }
        }
    }

    void updateOccupiedAreaLongSensor(double sensor_reading_distance, bool front)
    {
        double sensor_angle = (front) ? 0 : M_PI;
        double sensor_angle_center_offset = (front) ? LONG_FRONT_SENSOR_ANGLE_FROM_FORWARD : M_PI + LONG_FRONT_SENSOR_ANGLE_FROM_FORWARD;
        PrevValue & prev_value = getPrevValueLongSensors(front);

        updateOccupiedArea(sensor_reading_distance, sensor_angle, sensor_angle_center_offset, MAX_LONG_SENSOR_DISTANCE, LONG_SENSOR_DISTANCE_FROM_CENTER);
    }

    void updateOccupiedAreaShortSensor(double sensor_reading_distance, bool right_side, bool front)
    {
        double sensor_angle = getShortSensorAngle(right_side);
        double sensor_angle_center_offset = getShortSensorAngleCenterOffset(right_side, front);
        PrevValue & prev_value = getPrevValueShortSensors(right_side, front);

        updateOccupiedArea(sensor_reading_distance, sensor_angle, sensor_angle_center_offset, MAX_SHORT_SENSOR_DISTANCE, SHORT_SENSOR_DISTANCE_FROM_CENTER);
    }

    void updateOccupiedArea(double sensor_reading_distance, double sensor_angle, double sensor_angle_center_offset, int max_sensor_distance, double sensor_distance_from_center)
    {


        double x, y;
        bool acceptedSensorPos = getSensorReadingPos(x, y, sensor_reading_distance, sensor_angle, max_sensor_distance, sensor_distance_from_center, sensor_angle_center_offset);

        if(acceptedSensorPos) {
            map_.setBlockedIfNotBlockedAllready(x, y);
        }
    }


    /*
        Given the sensor reading, will update a wall cell unless max_distance > sensor_reading_distance
    */
    bool getSensorReadingPos(double & x,
                            double & y,
                            double sensor_reading_distance,         // The distance that the sensor show
                            double sensor_angle,                    // The angle in respect to the robot that the sensor points towards
                            double max_distance,                    // The max distance that the sensor can give acceptable readings
                            double sensor_distance_center_offset,   // The distance the sensor is away from the robot center
                            double sensor_angle_center_offset)      // The angle in respect to the robot that the sensor is positioned
    {
        if(getSensorReadingPosOffset(x, y, sensor_reading_distance, sensor_angle, max_distance,sensor_distance_center_offset, sensor_angle_center_offset, robot_angle_))
        {
            x += robot_x_pos_;
            y += robot_y_pos_;
            return true;
        }
        return false;

    }

    bool getSensorReadingPosOffset(double & x_offset,
                            double & y_offset,
                            double sensor_reading_distance,         // The distance that the sensor show
                            double sensor_angle,                    // The angle in respect to the plane
                            double max_distance,                    // The max distance that the sensor can give acceptable readings
                            double sensor_distance_center_offset,   // The distance the sensor is away from the robot center
                            double sensor_angle_center_offset,      // The angle in respect to the robot that the sensor is positioned
                            double plane_angle)                     // Normally the angle the robot is facing, can be used to set the angle of the plane
    {
        if(sensor_reading_distance > max_distance)
        {
            // We can not trust the information, to far away reading
            return false;
        }

        typedef std::function<double (double)> CosSinFunction;

        std::function<double (CosSinFunction)> getSensorReadingPosOffset = [plane_angle, sensor_angle_center_offset, sensor_distance_center_offset, sensor_angle, sensor_reading_distance]
                (CosSinFunction cosSinFunction)
            {
            double sensor_pos_offset = cosSinFunction(sensor_angle_center_offset + plane_angle) * sensor_distance_center_offset;

            double sensor_reading_pos_offset = cosSinFunction(sensor_angle + plane_angle) * sensor_reading_distance;

            return sensor_pos_offset + sensor_reading_pos_offset;

            };

        x_offset = getSensorReadingPosOffset([](double angle){ return cos(angle);});
        y_offset = getSensorReadingPosOffset([](double angle){ return sin(angle);});

        return true;
    }

    void updateFreeAreaLongSensor(double sensor_reading_distance, bool front)
    {
        double sensor_angle = (front) ? 0 : M_PI;
        double sensor_angle_center_offset = (front) ? LONG_FRONT_SENSOR_ANGLE_FROM_FORWARD : M_PI + LONG_FRONT_SENSOR_ANGLE_FROM_FORWARD;

        updateFreeArea(sensor_reading_distance, sensor_angle, sensor_angle_center_offset, MAX_LONG_SENSOR_DISTANCE, LONG_SENSOR_DISTANCE_FROM_CENTER);
    }

    void updateFreeAreaShortSensor(double sensor_reading_distance, bool right_side, bool front)
    {
        double sensor_angle = getShortSensorAngle(right_side);

        double sensor_angle_center_offset = getShortSensorAngleCenterOffset(right_side, front);

        updateFreeArea(sensor_reading_distance, sensor_angle, sensor_angle_center_offset, MAX_SHORT_SENSOR_DISTANCE, SHORT_SENSOR_DISTANCE_FROM_CENTER);

    }
    void updateFreeArea(double sensor_reading_distance, double sensor_angle, double sensor_angle_center_offset, int max_sensor_distance, double sensor_distance_from_center)
    {
        double x,y;
        int write_over_wall_counter = 0;
        for(double sensor_reading_part_distance = 0; sensor_reading_part_distance < (sensor_reading_distance - SENSOR_FREE_AREA_OFFSET_FROM_WALL) && sensor_reading_part_distance < max_sensor_distance; sensor_reading_part_distance += SENSOR_READING_PART_DISTANCE_BLOCK_SIZE)
        {
            getSensorReadingPos(x, y, sensor_reading_part_distance, sensor_angle, sensor_reading_part_distance + 1, sensor_distance_from_center, sensor_angle_center_offset);
            if(map_.getCell(x,y).isBlocked() && write_over_wall_counter < MAX_SENSOR_WALL_TO_FREE_PER_ITERATION){
                write_over_wall_counter++;
                map_.setFreeIfNotFreeAllready(x, y);
            } else {
                map_.setFreeIfUnknown(x, y);
            }
        }
    }

    double getShortSensorAngle(bool right_side) {
        return (right_side) ? (3*M_PI/2) : M_PI / 2.0;
    }

    double getShortSensorAngleCenterOffset(bool right_side, bool front)
    {
        double angle = (front) ? SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD : M_PI - SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD;
        angle *= (right_side) ? 1 : -1;
        return angle;
    }

    void updateFreeAreaUsingRobotPos() {
        double x_pos, y_pos;
        for(int x = -FREE_AREA_LIMIT; x <= FREE_AREA_LIMIT; x++) {
            for(int y = -FREE_AREA_LIMIT; y <= FREE_AREA_LIMIT; y++) {
                x_pos = robot_x_pos_ + x;
                y_pos = robot_y_pos_ + y;
                map_.setFreeIfNotFreeAllready(x_pos, y_pos);
                map_.setThickTempToWall(x_pos, y_pos);
            }
        }
    }

    void fillSmallUnknownAreaAroundPointWithFree(Cell & free_cell)
    {
        std::vector<Cell> closestCells = get8ClosestCells(free_cell);
        for(Cell possible_unknown_cell : closestCells)
        {
            if(possible_unknown_cell.isUnknown())
            {
                // Lets see how big the unknown area is
                std::vector<std::vector<bool> > visited (FILL_FREE_CELL_TOP_LIMIT * 3, std::vector<bool>(FILL_FREE_CELL_TOP_LIMIT * 3, false));
                std::vector<Cell> gathered_cells;
                int visited_i = visited.size()/2;
                int visited_j = visited.size()/2;
                bool small_enough_to_fill = getConnectedUnknown(gathered_cells, possible_unknown_cell, visited, visited_i, visited_j);
                if(small_enough_to_fill) {
                    for(Cell cell : gathered_cells) {
                        // It is small enough, lets fill it.
                        map_.setFreeIfNotFreeAllready(cell.getI(), cell.getJ());
                    }
                }

            }
        }
    }

    void fillcloses4UnknownWithFree(Cell & free_cell)
    {
        std::vector<Cell> closestCells = get4ClosestCells(free_cell);
        for(Cell possible_unknown_cell : closestCells)
        {
            if(possible_unknown_cell.isUnknown())
            {
                map_.setFreeIfNotFreeAllready(possible_unknown_cell.getI(), possible_unknown_cell.getJ());
            }
        }
    }

    bool getConnectedUnknown(std::vector<Cell> & gathered_cells, Cell & cell, std::vector<std::vector<bool> > & visited, int visited_i, int visited_j)
    {
        if(gathered_cells.size() == FILL_FREE_CELL_TOP_LIMIT)
        {
            // To many unknowns, abort!

            return false;
        }
        visited[visited_i][visited_j] = true;
        gathered_cells.push_back(cell);
        std::vector<Cell> closestCells = get4ClosestCells(cell);
        for(Cell close_cell : closestCells)
        {
            int close_cell_visited_i = visited_i + (cell.getI() - close_cell.getI());
            int close_cell_visited_j = visited_j + (cell.getJ() - close_cell.getJ());
            if(!visited[close_cell_visited_i][close_cell_visited_j] && close_cell.isUnknown())
            {
                // Cell can be added
                if(!getConnectedUnknown(gathered_cells, close_cell, visited, close_cell_visited_i, close_cell_visited_j))
                {
                    return false;
                }
            }
        }
        return true;
    }

    std::vector<Cell> get4ClosestCells(Cell & cell)
    {
        int i = cell.getI();
        int j = cell.getJ();
        std::vector<Cell> returnList;
        returnList.push_back(map_.getCell(i + 1, j));
        returnList.push_back(map_.getCell(i - 1, j));
        returnList.push_back(map_.getCell(i, j + 1));
        returnList.push_back(map_.getCell(i, j - 1));
        return returnList;
    }

    std::vector<Cell> get8ClosestCells(Cell & cell)
    {
        int i = cell.getI();
        int j = cell.getJ();
        std::vector<Cell> returnList = get4ClosestCells(cell);
        returnList.push_back(map_.getCell(i + 1, j + 1));
        returnList.push_back(map_.getCell(i - 1, j + 1));
        returnList.push_back(map_.getCell(i + 1, j - 1));
        returnList.push_back(map_.getCell(i - 1, j - 1));

        return returnList;
    }

    bool canBlock(double x, double y, bool write_over = false)
    {
         return (write_over || !map_.getCell(x, y).isFree());
    }


    PrevValue & getPrevValueShortSensors(bool right_side, bool front)
    {
        int index =  (right_side) ? 0 : 1 ;
        index |= (front) ? 0 : 2;
        return prev_value_short_sensors_[index];
    }

    PrevValue & getPrevValueLongSensors(bool front)
    {
        int index =  (front) ? 0 : 1 ;
        return prev_value_long_sensors_[index];
    }

    double getPointDistance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    double getAngleFromPoints(double x1, double y1, double x2, double y2)
    {
        return atan2(y2 - y1, x2 - x1);
    }

    struct Point
    {
        double x;
        double y;
    };

    std::vector<Point> getSideSensorPoints(double sensor_angle, double d_front, double d_back, double sensor_angle_center_offset_front, double sensor_angle_center_offset_back)
    {
        std::vector<Point> polygon(4);
        getSensorReadingPos(polygon[0].x, polygon[0].y, std::min(d_front - SENSOR_FREE_AREA_OFFSET_FROM_WALL, (double) MAX_SHORT_SENSOR_DISTANCE - SENSOR_FREE_AREA_OFFSET_FROM_WALL), sensor_angle, MAX_SHORT_SENSOR_DISTANCE + 1, SHORT_SENSOR_DISTANCE_FROM_CENTER, sensor_angle_center_offset_front);
        getSensorReadingPos(polygon[1].x, polygon[1].y, std::min(d_back - SENSOR_FREE_AREA_OFFSET_FROM_WALL, (double) MAX_SHORT_SENSOR_DISTANCE - SENSOR_FREE_AREA_OFFSET_FROM_WALL), sensor_angle, MAX_SHORT_SENSOR_DISTANCE, SHORT_SENSOR_DISTANCE_FROM_CENTER, sensor_angle_center_offset_back);
        getSensorReadingPos(polygon[2].x, polygon[2].y, 0, sensor_angle, MAX_SHORT_SENSOR_DISTANCE, SHORT_SENSOR_DISTANCE_FROM_CENTER, sensor_angle_center_offset_back);
        getSensorReadingPos(polygon[3].x, polygon[3].y, 0, sensor_angle, MAX_SHORT_SENSOR_DISTANCE, SHORT_SENSOR_DISTANCE_FROM_CENTER, sensor_angle_center_offset_front);
        return polygon;
    }

    void updateFreeAreaBetweenSensors(double d_front, double d_back, bool right_side)
    {
        double sensor_angle = getShortSensorAngle(right_side);

        double sensor_angle_center_offset_front = getShortSensorAngleCenterOffset(right_side, true);
        double sensor_angle_center_offset_back = getShortSensorAngleCenterOffset(right_side, false);

        std::vector<Point> polygon = getSideSensorPoints(sensor_angle, d_front, d_back, sensor_angle_center_offset_front, sensor_angle_center_offset_back);

        double smallest_x = polygon[0].x;
        double smallest_y = polygon[0].y;
        double biggest_x = polygon[0].x;
        double biggest_y = polygon[0].y;

        for(Point poly_point : polygon)
        {
            smallest_x = std::min(smallest_x, poly_point.x);
            smallest_y = std::min(smallest_y, poly_point.y);
            biggest_x = std::max(biggest_x, poly_point.x);
            biggest_y = std::max(biggest_y, poly_point.y);
            map_.setFreeIfUnknown(poly_point.x, poly_point.y);
        }



        Point point;
        for(int i = smallest_x; i <= biggest_x; i++)
        {
            for(int j = smallest_y; j <= biggest_y; j++)
            {
                point.x = i;
                point.y = j;
                if(pointInPolygon(point, polygon))
                {
                    map_.setFreeIfUnknown(point.x, point.y);
                }
            }
        }

    }


    bool pointInPolygon(const Point & point, const std::vector<Point> & polygon) {
      int i, j, nvert = polygon.size();
      bool c = false;

        for(i = 0, j = nvert - 1; i < nvert; j = i++) {
            if( ( (polygon[i].y) >= point.y ) != (polygon[j].y >= point.y) &&
                (point.x <= (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)
              )
              c = !c;
          }

          return c;
        }
};


#endif // MAP_HANDLER
