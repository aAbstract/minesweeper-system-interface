<script setup lang="ts">

// models
import { sensor_reading_group } from '../Models/ui';

// props
defineProps<{
    sensors_group: sensor_reading_group,
    apply_colors: boolean,
}>();

// utils
function compute_color_class(value: string) {
    const num_value = Number(value);

    if (num_value < 30)
        return 'green_span';
    else if (num_value < 60)
        return 'yellow_span';
    else if (num_value < 100)
        return 'red_span';
    else
        return '';
}

</script>

<template>
    <div class="sensors_group_cont">
        <div class="group_header">{{ sensors_group.group_name }}</div>
        <div class="group_body">
            <div class="group_labels">
                <span v-for="reading in sensors_group.readings"
                    :class="`reading_text ${apply_colors ? compute_color_class(reading.reading_value) : ''}`">{{
                        reading.reading_key
                    }}</span>
            </div>
            <div class="group_vals">
                <span v-for="reading in sensors_group.readings"
                    :class="`reading_text ${apply_colors ? compute_color_class(reading.reading_value) : ''}`">{{
                        reading.reading_value
                    }}</span>
            </div>
        </div>
    </div>
</template>

<style scoped>
.reading_text {
    width: 150px;
}

.red_span {
    color: #DD2C00;
}

.yellow_span {
    color: #FFD600;
}

.green_span {
    color: #64DD17;
}

.sensors_group_cont {
    width: 96%;
    height: fit-content;
    margin-bottom: 24px;
}

.group_header {
    font-weight: bold;
    border: 1px solid var(--font-color-light);
    border-left: 6px solid var(--font-color-light);
    padding: 8px;
}

.group_body {
    width: 100%;
}

.group_labels,
.group_vals {
    padding: 4px;
    display: flex;
    flex-direction: row;
    justify-content: flex-start;
    font-weight: bold;
}
</style>