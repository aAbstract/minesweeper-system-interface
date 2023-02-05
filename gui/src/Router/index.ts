// framework
import { createRouter, createWebHashHistory } from 'vue-router';

import MainCU from '../Views/MainCU.vue';

const routes = [
  {
    path: '/',
    name: 'MainCu',
    component: MainCU,
  },
]

const router = createRouter({
  history: createWebHashHistory(),
  routes,
});

export default router;
