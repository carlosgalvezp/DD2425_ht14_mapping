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

#define SHORT_SENSOR_DISTANCE_FROM_CENTER           10.68
#define SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD 0.53839

#define LONG_SENSOR_DISTANCE_FROM_CENTER        1.75
#define LONG_FRONT_SENSOR_ANGLE_FROM_FORWARD    0       //TODO: Not entirely true...

#define METRIC_CONVERTER    100.0 // To convert meters from odometry to cm in map (might redo this)

class MapHandler {
public:

    MapHandler(Map map) :
        map(map),
        robot_angle_(0),
        robot_x_pos_(0),
        robot_y_pos_(0),
        robot_x_pos_offset_(map.getWidth() / 2),
        robot_y_pos_offset_(map.getHeight() / 2)
    {
    }

    void update(const geometry_msgs::Pose2D::ConstPtr &odo_data, const ras_arduino_msgs::ADConverter::ConstPtr &adc_data)
    {
        // Retrieve the data
        robot_x_pos_ = odo_data->x * METRIC_CONVERTER + robot_x_pos_offset_;
        robot_y_pos_ = odo_data->y * METRIC_CONVERTER + robot_y_pos_offset_;

        double dist_front_large_range = RAS_Utils::longSensorToDistanceInCM(adc_data->ch8);
        double dist_back_large_range = RAS_Utils::longSensorToDistanceInCM(adc_data->ch7);  //TODO Check if it is really ch7

        double d_right_front = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch4);
        double d_right_back  = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch3);
        double d_left_front  = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch1);
        double d_left_back   = RAS_Utils::shortSensorToDistanceInCM(adc_data->ch2);

        // Update where we detect walls based on sensor readings
        updateOccupiedAreaLongSensor(dist_front_large_range, true);
        updateOccupiedAreaLongSensor(dist_back_large_range, false);

        updateOccupiedAreaShortSensor(d_right_front, true, true);
        updateOccupiedAreaShortSensor(d_right_back, true, false);
        updateOccupiedAreaShortSensor(d_left_front, false, true);
        updateOccupiedAreaShortSensor(d_left_back, false, false);
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

        updateOccupiedArea(sensor_reading_distance, sensor_angle, MAX_LONG_SENSOR_DISTANCE, LONG_SENSOR_DISTANCE_FROM_CENTER, sensor_angle_center_offset);
    }

    void updateOccupiedAreaShortSensor(double sensor_reading_distance, bool right_side, bool front)
    {
        double sensor_angle = (right_side) ? M_PI / 2.0 : (3*M_PI/2);

        double sensor_angle_center_offset = (front) ? SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD : M_PI - SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD;
        sensor_angle_center_offset *= (right_side) ? 1 : -1;

        updateOccupiedArea(sensor_reading_distance, sensor_angle, MAX_SHORT_SENSOR_DISTANCE, SHORT_SENSOR_DISTANCE_FROM_CENTER, sensor_angle_center_offset);
    }
    /*
        Given the sensor reading, will update a wall cell unless max_distance > sensor_reading_distance
    */
    void updateOccupiedArea(double sensor_reading_distance,         // The distance that the sensor show
                            double sensor_angle,                    // The angle in respect to the robot that the sensor points towards
                            double max_distance,                    // The max distance that the sensor can give acceptable readings
                            double sensor_distance_center_offset,   // The distance the sensor is away from the robot center
                            double sensor_angle_center_offset)      // The angle in respect to the robot that the sensor is positioned
    {
        if(sensor_reading_distance > max_distance)
        {
            // We can not trust the information, to far away reading
            return;
        }

        typedef std::function<double (double)> CosSinFunction;

        std::function<double (double, CosSinFunction)> getSensorReadingPos = [robot_angle_, sensor_angle_center_offset, sensor_distance_center_offset, sensor_angle, sensor_reading_distance]
                (double robot_pos_, CosSinFunction cosSinFunction)
            {
            double sensor_pos_offset = cosSinFunction(sensor_angle_center_offset + robot_angle_) * sensor_distance_center_offset;

            double sensor_reading_pos_offset = cosSinFunction(sensor_angle + robot_angle_) * sensor_reading_distance;

            return robot_pos_ + sensor_pos_offset + sensor_reading_pos_offset;

            };

        double sensor_reading_x_pos = getSensorReadingPos(robot_x_pos_, [](double angle){ return cos(angle);});
        double sensor_reading_y_pos = getSensorReadingPos(robot_y_pos_, [](double angle){ return sin(angle);});

        RAS_Utils::print({"sensor_angle", "sensor_distance_center_offset", "sensor_angle_center_offset", "sensor_reading_distance", "max_distance", "sensor_reading_x_pos", "sensor_reading_y_pos"}, {sensor_angle, sensor_distance_center_offset, sensor_angle_center_offset, sensor_reading_distance, max_distance, sensor_reading_x_pos, sensor_reading_y_pos});

        map.setBlocked(sensor_reading_x_pos, sensor_reading_y_pos);
    }
};


#endif // MAP_HANDLER
