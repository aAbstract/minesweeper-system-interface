<script setup lang="ts"></script>

<template>
    <div id="config_body">
        <div class="config_control_cont">
            <span>Config 1:</span>
            <input type="text">
        </div>
        <div class="config_control_cont">
            <span>Config 2:</span>
            <input type="text">
        </div>
        <div class="config_control_cont">
            <span>Config 3:</span>
            <input type="text">
        </div>
        <div id="config_footer">
            <button class="config_button">Apply Configs</button>
        </div>
        <div id="config_shortcuts">
            <div class="shortcut_row">
                <button class="config_button">ROS UP</button>
                <button class="config_button">ROS DOWN</button>
                <button class="config_button">ROS SHELL</button>
            </div>
            <div class="shortcut_row">
                <button class="config_button">RPI SHELL</button>
                <button class="config_button">RPI CODE</button>
                <button class="config_button">CU CODE</button>
            </div>
        </div>
    </div>
</template>

<style scoped>
.shortcut_row button {
    width: 120px;
}

.shortcut_row {
    margin-bottom: 16px;
    display: flex;
    flex-direction: row;
    justify-content: space-around;
}

#config_body {
    width: 100%;
    height: fit-content;
}

.config_control_cont {
    width: 90%;
    display: flex;
    flex-direction: row;
    justify-content: flex-start;
    align-items: center;
    margin-bottom: 16px;
}

.config_control_cont span {
    width: 100px;
    font-weight: bold;
}

.config_control_cont input {
    font-family: "Orbitron", sans-serif;
    flex-grow: 1;
    background-color: transparent;
    color: var(--font-color-light);
    border: 1px solid var(--font-color-light);
    font-size: 16px;
}

#config_footer {
    display: flex;
    flex-direction: row;
    justify-content: flex-end;
    margin-top: 16px;
    margin-bottom: 36px;
    width: 90%;
}

.config_button {
    font-family: "Orbitron", sans-serif;
    width: 150px;
    height: 32px;
    background-color: transparent;
    color: var(--font-color-light);
    border: 1px solid var(--font-color-light);
    font-weight: bold;
    cursor: pointer;
    transition: 0.3s ease;
}

.config_button:hover {
    background-color: var(--font-color-light);
    color: var(--background-color);
}
</style>