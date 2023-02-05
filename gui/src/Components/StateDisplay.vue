<script setup lang="ts">

import { state_pair } from '../Models/ui';

defineProps<{
    state_pairs: state_pair[],
}>();

// global vars
const class_map = {
    'ONLINE': 'green_state',
    'OFFLINE': 'yellow_state',
    'COFFLINE': 'red_state',
} as any;

</script>

<template>
    <div class="state_display_cont">
        <div v-for="state in state_pairs" :class="`state_display_item ${class_map[state.pair_state]}`">{{
            state.pair_name
        }}</div>
    </div>
</template>

<style scoped>
.red_state {
    color: #DD2C00;
    border: 1px solid #DD2C00;
    border-left: 4px solid #DD2C00;
}

.green_state {
    color: #64DD17;
    border: 1px solid #64DD17;
    border-left: 4px solid #64DD17;
}

.yellow_state {
    color: #FFD600;
    border: 1px solid #FFD600;
    border-left: 4px solid #FFD600;
}

.state_display_cont {
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
}

.state_display_item {
    margin-bottom: 8px;
    padding: 4px;
}
</style>