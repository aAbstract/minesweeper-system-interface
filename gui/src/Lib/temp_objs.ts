import { sensor_reading_group, state_pair, ros_log } from '../Models/ui';

export const robot_readings = [
    {
        group_name: 'Lateral',
        readings: [
            {
                reading_key: 'Low',
                reading_value: '50',
            },
            {
                reading_key: 'Normal',
                reading_value: '200',
            },
            {
                reading_key: 'High',
                reading_value: '400',
            },
        ],
    },
    {
        group_name: 'Rotetional',
        readings: [
            {
                reading_key: 'Low',
                reading_value: '50',
            },
            {
                reading_key: 'Normal',
                reading_value: '200',
            },
            {
                reading_key: 'High',
                reading_value: '400',
            },
        ],
    },
    {
        group_name: 'Vertical',
        readings: [
            {
                reading_key: 'Low',
                reading_value: '50',
            },
            {
                reading_key: 'Normal',
                reading_value: '200',
            },
            {
                reading_key: 'High',
                reading_value: '400',
            },
        ],
    },
] as sensor_reading_group[];

export const rpi_state = {
    group_name: 'RPI State',
    readings: [
        {
            reading_key: 'CPU',
            reading_value: '20',
        },
        {
            reading_key: 'MEM',
            reading_value: '50',
        },
        {
            reading_key: 'NET',
            reading_value: '70',
        },
    ],
} as sensor_reading_group;

export const cu_state = {
    group_name: 'Control Unit State',
    readings: [
        {
            reading_key: 'CPU',
            reading_value: '35',
        },
        {
            reading_key: 'MEM',
            reading_value: '60',
        },
        {
            reading_key: 'NET',
            reading_value: '10',
        },
    ],
} as sensor_reading_group;

export const net_state = [
    {
        pair_name: 'Control Unit',
        pair_state: 'ONLINE',
    },
    {
        pair_name: 'RPI',
        pair_state: 'COFFLINE',
    },
] as state_pair[];

export const ros_nodes_states = [
    {
        pair_name: 'ROS Master',
        pair_state: 'ONLINE',
    },
    {
        pair_name: 'Joystick',
        pair_state: 'OFFLINE',
    },
    {
        pair_name: 'GUI',
        pair_state: 'ONLINE',
    },
    {
        pair_name: 'Mapping Vision',
        pair_state: 'OFFLINE',
    },
    {
        pair_name: 'Mines Vision',
        pair_state: 'OFFLINE',
    },
    {
        pair_name: 'Camera Adapter',
        pair_state: 'OFFLINE',
    },
    {
        pair_name: 'Motors Controller',
        pair_state: 'OFFLINE',
    },
    {
        pair_name: 'Arm Controller',
        pair_state: 'OFFLINE',
    },
    {
        pair_name: 'Serial Interface',
        pair_state: 'OFFLINE',
    },
    {
        pair_name: 'Auto Pilot',
        pair_state: 'OFFLINE',
    },
] as state_pair[];

export const ros_logs = [
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Connecting to ROS Network',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Initializing Serial Interface',
    },
    {
        ros_log_level: 'ERROR',
        ros_log_text: 'Faild to Start Mapping Vision',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Starting Minesweeper in Maunual Mode, Starting Minesweeper in Maunual Mode',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Connecting to ROS Network',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Initializing Serial Interface',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Starting Minesweeper in Maunual Mode',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Connecting to ROS Network',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Initializing Serial Interface',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Starting Minesweeper in Maunual Mode',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Connecting to ROS Network',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Initializing Serial Interface',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Starting Minesweeper in Maunual Mode',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Connecting to ROS Network',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Initializing Serial Interface',
    },
    {
        ros_log_level: 'ERROR',
        ros_log_text: 'Faild to Start Mapping Vision',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Starting Minesweeper in Maunual Mode',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Connecting to ROS Network',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Initializing Serial Interface',
    },
    {
        ros_log_level: 'INFO',
        ros_log_text: 'Starting Minesweeper in Maunual Mode',
    },
] as ros_log[];