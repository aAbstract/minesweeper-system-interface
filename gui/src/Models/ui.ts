export interface sensor_reading {
    reading_key: string,
    reading_value: string,
};

export interface sensor_reading_group {
    group_name: string,
    readings: sensor_reading[],
};

export interface state_pair {
    pair_name: string,
    pair_state: string,
};

export interface ros_log {
    ros_log_level: string,
    ros_log_text: string,
};