enum move {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

enum direction {
    GO_FOWARD,
    GO_FOWARD_RIGHT,
    GO_FOWARD_LEFT,
    GO_RIGHT,
    GO_LEFT,
    GO_BACKWARD
};

movement generate_output(int move_output, float advance, float twist) {

    movement output;

    switch(move_output) {
        case STOP:
            output.advance = 0.0f;
            output.twist = 0.0f;
            break;

        case FORWARD:
            output.advance = advance;
            output.twist = 0.0f;
            break;

        case BACKWARD:
            output.advance = -advance;
            output.twist = 0.0f;
            break;

        case LEFT:
            output.advance = 0.0f;
            output.twist = twist;
            break;

        case RIGHT:
            output.advance = 0.0f;
            output.twist = -twist;
            break;

        default:
            printf("OUTPUT %d NOT DEFINED ", move_output);
            output.advance = 0.0f;
            output.twist = 0.0f;
            break;
    }

    return(output);
}

int get_light_direction(float *light_readings){
    int sensor = 0;

    std::cout << "i->" << 0 << " value->" << light_readings[0] << std::endl;

    for(int i = 1; i < 8; i+=2 ) {
        std::cout << "i->" << i << " value->" << light_readings[i] << std::endl;
        if(light_readings[i] > light_readings[sensor])
            sensor = i;
    }

    std::cout << "Sensor->" << sensor << std::endl;

    if(sensor == 0)      return GO_FOWARD;
    else if(sensor == 1) return GO_FOWARD;
    else if(sensor == 3) return GO_LEFT;
    else if(sensor == 5) return GO_RIGHT;
    else if(sensor == 7) return GO_FOWARD;
    else return GO_RIGHT;
}

int obstacle_detection(float* observations, int size, float laser_value) {
    int right_sensors, left_sensors;
    int obstacle_left = 0, obstacle_right = 0;
    if( size % 2 != 0) {
        right_sensors = ( size - 1 ) / 2;
        left_sensors = right_sensors + 1;
    } else {
        right_sensors = left_sensors = size / 2;
    }

    for (int i = left_sensors; i < size ; i++ ) {
        if( observations[i] < laser_value  ) {
            obstacle_left = 2;
            break;
        }
    }

    for (int i = 0; i < right_sensors ; i++ ) {
        if( observations[i] < laser_value  ) {
            obstacle_right = 1;
            break;
        }
    }

    return obstacle_left + obstacle_right ;
}