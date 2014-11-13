#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include <geometry_msgs/Pose2D.h>
#include <ras_arduino_msgs/ADConverter.h>

#include <ras_utils/ras_utils.h>

#include <robot_maps/cell.h>
#include <robot_maps/map.h>

#include <math.h>


#define MAX_SHORT_SENSOR_DISTANCE   30
#define MAX_LONG_SENSOR_DISTANCE    80

#define SHORT_SENSOR_DISTANCE_FROM_CENTER   10.68
#define SHORT_SENSOR_ANGLE_FROM_FORWARD

#define LONG_SENSOR_DISTANCE_FROM_CENTER
#define LONG_SENSOR_ANGLE_FROM_FORWARD

class MapHandler {
public:

    MapHandler(Map map) : map(map)
    {
    }

    void update(const geometry_msgs::Pose2D::ConstPtr &odo_data, const ras_arduino_msgs::ADConverter::ConstPtr &adc_data)
    {
        // Retrieve the data
        robot_x_pos_ = odo_data->x;
        robot_y_pos_ = odo_data->y;

        double dist_front_large_range = RAS_Utils::longSensorToDistanceInCM(adc_data->ch7); //TODO Check if it is really ch7
        double dist_back_large_range = RAS_Utils::longSensorToDistanceInCM(adc_data->ch8);

        double d_right_front = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch4);
        double d_right_back  = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch3);
        double d_left_front  = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch1);
        double d_left_back   = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch2);

        //updateOccupiedArea(d_right_front);

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

    int getCellSize()
    {
        return map.getCellSize();
    }


private:
    Map map;

    int height_;
    int width_;

    double robot_angle_;
    double robot_x_pos_;
    double robot_y_pos_;

    typedef std::vector< std::vector<Cell> > CellMatrix;
    typedef std::vector<Cell> CellVector;

    void updateOccupiedArea(double sensor_distance,                 // The distance that the sensor show
                            double sensor_angle,                    // The angle in respect to the robot that the sensor points towards
                            double max_distance,                    // The max distance that the sensor can give acceptable readings
                            double sensor_distance_center_offset,   // The distance the sensor is away from the robot center
                            double sensor_angle_center_offset)      // The angle in respect to the robot that the sensor is positioned
    {
        if(sensor_distance > max_distance)
        {
            // We can not trust the information, to far away reading
            return;
        }

        typedef std::function<double (double)> CosSinFunction;

        std::function<double (double, CosSinFunction)> getSensorReadingPosOffset = [robot_angle_, sensor_angle_center_offset, sensor_distance_center_offset, sensor_angle, sensor_distance]
                (double robot_pos_, CosSinFunction cosSinFunction)
            {
            double sensor_pos_offset = cosSinFunction(sensor_angle_center_offset + robot_angle_) * sensor_distance_center_offset;

            double sensor_reading_pos_offset = cosSinFunction(sensor_angle + robot_angle_) * sensor_distance;

            return sensor_pos_offset + sensor_reading_pos_offset;

            };

        double sensor_reading_x_pos_ = getSensorReadingPosOffset(robot_x_pos_, [](double angle){ return cos(angle);});
        double sensor_reading_y_pos_ = getSensorReadingPosOffset(robot_y_pos_, [](double angle){ return sin(angle);});

        // TODO: finnish this function

    }
};


#endif // MAP_HANDLER
