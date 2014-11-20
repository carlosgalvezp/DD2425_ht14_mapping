#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include <geometry_msgs/Pose2D.h>
#include <ras_arduino_msgs/ADConverter.h>

#include <ras_utils/ras_utils.h>

#include <robot_maps/cell.h>
#include <robot_maps/map.h>

#include <math.h>
#include <vector>
#include <string>

#define MAX_SHORT_SENSOR_DISTANCE   30
#define MAX_LONG_SENSOR_DISTANCE    80

#define SHORT_SENSOR_DISTANCE_FROM_CENTER           10.68
#define SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD -0.53839

#define LONG_SENSOR_DISTANCE_FROM_CENTER        1.75
#define LONG_FRONT_SENSOR_ANGLE_FROM_FORWARD    0       //TODO: Not entirely true...

#define METRIC_CONVERTER    100.0 // To convert meters from odometry to cm in map (might redo this)

#define FREE_AREA_LIMIT 12

#define SENSOR_READING_PART_DISTANCE_BLOCK_SIZE 0.5 // Value 0.1 means that when working through the linear function "Sensor start to sensor reading", we look 0.1 cm at a time. Lower value leads to more computational costs but better precission in the sense of not missing a "hit node"

#define SENSOR_OLD_VALUE_DISTANCE_LIMIT 10.0 // 10 cm means only fill the wall cells if the last 2 values were within 10 cm from eachother.

class MapHandler {
public:

    MapHandler(Map map) :
        map(map),
        robot_angle_(0),
        robot_x_pos_(0),
        robot_y_pos_(0),
        robot_x_pos_offset_(map.getWidth() / 2),
        robot_y_pos_offset_(map.getHeight() / 2),
        prev_value_short_sensors_(4, PrevValue(-10000, -10000))
    {
    }

    void update(const geometry_msgs::Pose2D::ConstPtr &odo_data, const ras_arduino_msgs::ADConverter::ConstPtr &adc_data)
    {
        ROS_INFO("[ROBOT_MAPS");

        // Retrieve the data
        robot_x_pos_ = odo_data->x * METRIC_CONVERTER + robot_x_pos_offset_;
        robot_y_pos_ = odo_data->y * METRIC_CONVERTER + robot_y_pos_offset_;
        robot_angle_ = odo_data->theta;

        double dist_front_large_range = RAS_Utils::longSensorToDistanceInCM(adc_data->ch8);
        double dist_back_large_range = RAS_Utils::longSensorToDistanceInCM(adc_data->ch7);  //TODO Check if it is really ch7

        double d_right_front = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch4);
        double d_right_back  = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch3);
        double d_left_front  = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch1);
        double d_left_back   = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch2);

        // Update where we detect walls based on sensor readings
       // updateOccupiedAreaLongSensor(dist_front_large_range, true);
       // updateOccupiedAreaLongSensor(dist_back_large_range, false);

        updateOccupiedAreaShortSensor(d_right_front, true, true);
        updateOccupiedAreaShortSensor(d_right_back, true, false);
        updateOccupiedAreaShortSensor(d_left_front, false, true);
        updateOccupiedAreaShortSensor(d_left_back, false, false);

        updateFreeAreaUsingRobotPos();

        updateFreeAreaShortSensor(d_right_front, true, true);
        updateFreeAreaShortSensor(d_right_back, true, false);
        updateFreeAreaShortSensor(d_left_front, false, true);
        updateFreeAreaShortSensor(d_left_back, false, false);

        std::vector<std::string> names = {"robot_x_pos", "robot_y_pos", "robot_angle"};
        std::vector<double> values = {robot_x_pos_, robot_y_pos_, robot_angle_};
        RAS_Utils::print(names, values);

    }

    std::vector<int8_t> & getMap() {
        return map.getSimpleMapVector();
    }

    int getHeight()
    {
        return map.getHeight();
    }

    int getWidth()
    {
        return map.getWidth();
    }

    double getCellSize()
    {
        return map.getCellSize();
    }


private:
    struct PrevValue {
        double x;
        double y;

        PrevValue(double x, double y) : x(x), y(y){}
    };

    std::vector<PrevValue> prev_value_short_sensors_;

    Map map;

    int height_;
    int width_;

    double robot_angle_;
    double robot_x_pos_;
    double robot_y_pos_;

    double robot_x_pos_offset_;
    double robot_y_pos_offset_;

    typedef std::vector< std::vector<Cell> > CellMatrix;
    typedef std::vector<Cell> CellVector;

    void updateOccupiedAreaLongSensor(double sensor_reading_distance, bool front)
    {
        double sensor_angle = (front) ? 0 : M_PI;
        double sensor_angle_center_offset = (front) ? LONG_FRONT_SENSOR_ANGLE_FROM_FORWARD : M_PI + LONG_FRONT_SENSOR_ANGLE_FROM_FORWARD;

        double x, y;
        getSensorReadingPos(x, y, sensor_reading_distance, sensor_angle, MAX_LONG_SENSOR_DISTANCE, LONG_SENSOR_DISTANCE_FROM_CENTER, sensor_angle_center_offset);
        setBlocked(x, y);
    }

    void updateOccupiedAreaShortSensor(double sensor_reading_distance, bool right_side, bool front)
    {
        double sensor_angle = getShortSensorAngle(right_side);

        double sensor_angle_center_offset = getShortSensorAngleCenterOffset(right_side, front);

        double x, y;
        bool acceptedSensorPos = getSensorReadingPos(x, y, sensor_reading_distance, sensor_angle, MAX_SHORT_SENSOR_DISTANCE, SHORT_SENSOR_DISTANCE_FROM_CENTER, sensor_angle_center_offset);
        bool acceptedBlocking = false;
        if(acceptedSensorPos) {
            acceptedBlocking = setBlocked(x, y, true);
        }

        if(acceptedBlocking)
        {
            PrevValue & prev_value = getPrevValueShortSensors(right_side, front);

            double pointDistance = getPointDistance(x, y, prev_value.x, prev_value.y);

            std::vector<std::string> names = {"x", "y", "prev_x", "prev_y", "pointDistance"};
            std::vector<double> values = {x, y, prev_value.x, prev_value.y, pointDistance };
            RAS_Utils::print(names, values);

            if (pointDistance < SENSOR_OLD_VALUE_DISTANCE_LIMIT)
            {
                // The points are close enough to "fill in the blanks" with wall cells

                double function_angle = getAngleFromPoints(prev_value.x, prev_value.y, x, y);
                double x_offset, y_offset;
                for(double distance_chunk = SENSOR_READING_PART_DISTANCE_BLOCK_SIZE; distance_chunk < pointDistance; distance_chunk += SENSOR_READING_PART_DISTANCE_BLOCK_SIZE)
                {
                    // Here we start filling the function given by the point_distance and angle as (r, theta)
                    getSensorReadingPosOffset(x_offset, y_offset, distance_chunk, function_angle, pointDistance, 0, 0, 0);
                    setBlocked(prev_value.x + x_offset, prev_value.y + y_offset, true);
                }
            }
            // Finally update the prev Value
            prev_value.x = x;
            prev_value.y = y;
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

    void updateFreeAreaShortSensor(double sensor_reading_distance, bool right_side, bool front)
    {
        double sensor_angle = getShortSensorAngle(right_side);

        double sensor_angle_center_offset = getShortSensorAngleCenterOffset(right_side, front);

        double x,y;
        for(double sensor_reading_part_distance = 0; sensor_reading_part_distance < sensor_reading_distance && sensor_reading_part_distance < MAX_SHORT_SENSOR_DISTANCE; sensor_reading_part_distance += SENSOR_READING_PART_DISTANCE_BLOCK_SIZE)
        {
            getSensorReadingPos(x, y, sensor_reading_part_distance, sensor_angle, sensor_reading_part_distance + 1, SHORT_SENSOR_DISTANCE_FROM_CENTER, sensor_angle_center_offset);
            setFree(x, y);
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
                if(map.getCell(x_pos, y_pos).isUnknown()) {
                    map.setFree(robot_x_pos_ + x, robot_y_pos_ + y);
                }
            }
        }
    }

    bool setBlocked(double x, double y, bool write_over = false)
    {
        if(write_over || map.getCell(x, y).isUnknown()) {
            map.setBlocked(x, y);
            return true;
        }
        return false;
    }

    bool setFree(double x, double y, bool write_over = false)
    {
        if(write_over || map.getCell(x, y).isUnknown()) {
            map.setFree(x, y);
            return true;
        }
        return false;
    }

    PrevValue & getPrevValueShortSensors(bool right_side, bool front)
    {
        int index =  (right_side) ? 0 : 1 ;
        index |= (front) ? 0 : 2;
        return prev_value_short_sensors_[index];
    }

    double getPointDistance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    double getAngleFromPoints(double x1, double y1, double x2, double y2)
    {
        return atan2(y2 - y1, x2 - x1);
    }
};


#endif // MAP_HANDLER
