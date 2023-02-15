<script setup lang="ts">

// framework
import { ref, onMounted } from 'vue';

// lib
import { rpi_state, cu_state, net_state, ros_nodes_states } from '../Lib/temp_objs';
import { server_url } from '../Lib/networking';

// UI components
import SensorsGroup from '../Components/SensorsGroup.vue';
import ConfigSection from '../Components/ConfigSection.vue'
import StateDisplay from '../Components/StateDisplay.vue';

// API
import { get_sensors_readings } from '../API/sensors';
import { get_controls_readings } from '../API/joystick_controls';

// models
import { sensor_reading_group, ros_log } from '../Models/ui';

// global vars
const ros_log_tag_map = {
    'INFO': 'green_ros_log_tag',
    'ERROR': 'red_ros_log_tag',
} as any;

// refs
const acc_readings_group = ref({
    group_name: 'Accelerometer',
    readings: [
        {
            reading_key: 'X',
            reading_value: '0',
        },
        {
            reading_key: 'Y',
            reading_value: '0',
        },
        {
            reading_key: 'Z',
            reading_value: '0',
        },
    ],
} as sensor_reading_group);

const coil_readings_group = ref({
    group_name: 'Coil',
    readings: [{
        reading_key: 'Trigger',
        reading_value: '0',
    }],
} as sensor_reading_group);

const ros_logs = ref([
    {
        ros_log_level: 'INFO',
        ros_log_text: 'ZRI Online',
    },
] as ros_log[]);

const right_analog = ref('0,0')
const left_analog = ref('0,0')
const cmd = ref('X')

// utils
function add_log(_log: ros_log) {
    ros_logs.value.push(_log);
}

function update_sensors() {
    get_sensors_readings().then((resp) => {
        if (!resp.success) {
            add_log({
                ros_log_level: 'ERROR',
                ros_log_text: resp.msg,
            });
            return;
        }

        // set acc reaings
        const acc_readings = {
            group_name: 'Accelerometer',
            readings: [
                {
                    reading_key: 'X',
                    reading_value: resp.data.sensors_readings.acc_x,
                },
                {
                    reading_key: 'Y',
                    reading_value: resp.data.sensors_readings.acc_y,
                },
                {
                    reading_key: 'Z',
                    reading_value: resp.data.sensors_readings.acc_z,
                },
            ],
        } as sensor_reading_group;
        acc_readings_group.value = acc_readings;

        // set coil trigger
        const coil_readings = {
            group_name: 'Coil',
            readings: [{
                reading_key: 'Trigger',
                reading_value: String(resp.data.sensors_readings.coil * 100),
            }],
        };
        coil_readings_group.value = coil_readings;

    });
}

function update_controls_feedback() {
    get_controls_readings().then((resp) => {
        if (!resp.success) {
            add_log({
                ros_log_level: 'ERROR',
                ros_log_text: resp.msg,
            });
            return;
        }

        cmd.value = resp.data.joystick_controls.cmd;

        let lax = Number(resp.data.joystick_controls.left_analog.split(',')[0]);
        let lay = Number(resp.data.joystick_controls.left_analog.split(',')[1]);
        let rax = Number(resp.data.joystick_controls.right_analog.split(',')[0]);
        let ray = Number(resp.data.joystick_controls.right_analog.split(',')[1]);

        left_analog.value = `${lax.toFixed(2)},${lay.toFixed(2)}`;
        right_analog.value = `${rax.toFixed(2)},${ray.toFixed(2)}`;
    });
}

onMounted(() => {
    // slow update loop
    setInterval(() => {
        update_sensors();
    }, 1000);

    // quick update loop
    setInterval(() => {
        update_controls_feedback();
    }, 500);
});

</script>

<template>
    <div id="main_cu_cont">
        <div id="control_cont">
            <div id="readings_cont">
                <h3 class="box_style sec_header">Readings</h3>
                <SensorsGroup :sensors_group="acc_readings_group" :apply_colors="false" />
                <SensorsGroup :sensors_group="coil_readings_group" :apply_colors="true" />
                <SensorsGroup :sensors_group="rpi_state" :apply_colors="true" />
            </div>
            <div id="config_cont">
                <h3 class="box_style sec_header">Config</h3>
                <ConfigSection />
            </div>
        </div>
        <div id="mid_cont">
            <div id="map_cont">
                <h3 class="small_sec_header">Mapping</h3>
                <div id="map_body"></div>
            </div>
            <div id="cams_cont">
                <h3 class="small_sec_header">Live Feed</h3>
                <div id="cams_inner_cont">
                    <div class="cam_view">
                        <img :src="`${server_url}/api/cameras/cam_1`" alt="OFFLINE" style="color: #DD2C00;">
                    </div>
                    <div class="cam_view">
                        <img :src="`${server_url}/api/cameras/cam_1`" alt="OFFLINE" style="color: #DD2C00;">
                    </div>
                </div>
            </div>
            <div id="control_feed_cont">
                <h3 class="small_sec_header">Feed Back</h3>
                <div class="control_feed_body_line">
                    <span>Left Analog</span>
                    <span>Button</span>
                    <span>Right Analog</span>
                </div>
                <div class="control_feed_body_line">
                    <span>{{ left_analog }}</span>
                    <span>{{ cmd }}</span>
                    <span>{{ right_analog }}</span>
                </div>
                <div id="tags_cont">
                    <div id="ros_conn_tag" class="mode_tag">ROS ONLINE</div>
                    <div id="control_mode_tag" class="mode_tag">MANUAL</div>
                </div>
            </div>
        </div>
        <div id="tech_cont">
            <div id="cu_state" class="tech_window">
                <SensorsGroup :sensors_group="cu_state" :apply_colors="true" />
            </div>
            <div id="net_state_cont" class="tech_window">
                <h3 class="tech_sec_header">Network State</h3>
                <StateDisplay :state_pairs="net_state" />
            </div>
            <div id="ros_state_cont" class="tech_window">
                <h3 class="tech_sec_header">ROS Nodes State</h3>
                <StateDisplay :state_pairs="ros_nodes_states" />
            </div>
            <div id="logs_cont" class="tech_window">
                <h3 class="tech_sec_header">Logs</h3>
                <div v-for="ros_log_obj in ros_logs" class="ros_log_item">
                    <span :class="`ros_log_tag ${ros_log_tag_map[ros_log_obj.ros_log_level]}`">{{
                        ros_log_obj.ros_log_level
                    }}</span>
                    <div class="ros_log_text">{{ ros_log_obj.ros_log_text }}</div>
                </div>
            </div>
        </div>
    </div>
</template>

<style scoped>
.ros_log_item {
    width: fit-content;
}

.ros_log_text {
    white-space: nowrap;
}

.ros_log_item {
    display: flex;
    flex-direction: row;
    justify-content: flex-start;
}

.ros_log_tag {
    width: 80px;
}

.tech_sec_header {
    margin: 0px;
    margin-bottom: 8px;
    font-size: 16px;
}

.tech_window {
    margin: 4px;
    padding: 4px;
    border: 1px solid var(--font-color-light);
}

.tech_window .sensors_group_cont {
    margin: 0px;
    width: 100%;
}

#tags_cont {
    display: flex;
    flex-direction: row;
    justify-content: space-around;
}

.control_feed_body_line span {
    width: 200px;
    font-weight: bold;
}

.control_feed_body_line {
    display: flex;
    flex-direction: row;
    justify-content: space-around;
    padding: 4px;
}

.mode_tag {
    font-weight: bold;
    width: 200px;
    padding: 8px;
    margin: auto;
    text-align: center;
    margin-top: 8px;
}

.red_ros_log_tag {
    color: #DD2C00;
}

.green_ros_log_tag {
    color: #64DD17;
}

#ros_conn_tag {
    color: #64DD17;
    border: 2px solid #64DD17;
}

#control_mode_tag {
    color: #FFD600;
    border: 2px solid #FFD600;
}

#map_body {
    margin: 0px 8px;
    border: 1px solid var(--font-color-light);
    flex-grow: 1;
}

.cam_view {
    flex-grow: 1;
    border: 1px solid var(--font-color-light);
    aspect-ratio: 1;
    margin: 8px;
    display: flex;
    flex-direction: row;
    justify-content: center;
    align-items: center;
    min-width: 470px;
}

#logs_cont {
    flex-grow: 1;
    overflow: scroll;
}

#map_cont {
    width: 100%;
    flex-grow: 1;
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
}

#control_feed_cont {
    height: calc(100% / 6);
    width: 100%;
}

#cams_cont {
    height: fit-content;
    width: 100%;
}

#cams_inner_cont {
    display: flex;
    flex-direction: row;
    justify-content: space-around;
}

.sec_header {
    margin: 8px;
    margin-left: 0px;
    width: 70%;
}

.small_sec_header {
    font-size: 16px;
    margin: 8px 0px 16px 8px;
}

#main_cu_cont {
    display: flex;
    flex-direction: row;
    justify-content: flex-start;
}

#control_cont {
    width: calc(100% / 4);
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
}

#readings_cont {
    flex-grow: 1;
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
    align-items: flex-start;
}

#config_cont {
    flex-grow: 1;
}

#mid_cont {
    flex-grow: 1;
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
}

#tech_cont {
    width: calc(100% / 4);
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
}

.box_style {
    background-color: var(--font-color-light);
    color: var(--background-color);
    padding: 12px 32px;
    --notchSize: 16px;
    clip-path: polygon(var(--notchSize) 0, 100% 0, 100% calc(100% - var(--notchSize)), calc(100% - var(--notchSize)) 100%, 0 100%, 0 var(--notchSize));
}
</style>