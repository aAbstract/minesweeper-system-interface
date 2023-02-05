<script setup lang="ts">
</script>

<template>
  <router-view style="height: 100%;" />
</template>

<style>
:root {
  /* dark theme */
  --background-color: #000000;
  --font-color-light: #bdecf6;
  --font-color-dark: #112b3d;
}

body,
html {
  width: 100%;
  height: 100%;
  margin: 0;
  background-color: var(--background-color);
}

#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  font-family: "Orbitron", sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  color: var(--font-color-light);
  width: 100%;
  height: 100%;
}

img[alt] {
  text-indent: 8px;
}
</style>
