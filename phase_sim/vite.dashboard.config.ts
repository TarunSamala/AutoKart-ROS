import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// AutoKart Vehicle Dashboard: real-vehicle monitoring only. Separate build, port and
// process from the Algorithm Lab (vite.lab.config.ts). It never talks to the Lab API.
const PERCEPTION = process.env.AUTOKART_PERCEPTION_URL ?? 'http://127.0.0.1:8768';
const ROS_GATEWAY = process.env.AUTOKART_ROS_GATEWAY_URL ?? 'http://127.0.0.1:8765';

export default defineConfig({
  root: 'apps/dashboard',
  plugins: [react()],
  server: {
    host: '0.0.0.0',
    port: Number(process.env.AUTOKART_DASHBOARD_PORT ?? 5180),
    strictPort: true,
    fs: { allow: ['../..'] },
    proxy: {
      '/api/snapshot': PERCEPTION,
      '/api/image': PERCEPTION,
      '/api/v1': PERCEPTION,
      '/health': PERCEPTION,
      '/ready': PERCEPTION,
      '^/mission/': { target: ROS_GATEWAY, ws: true },
    },
  },
  preview: { port: Number(process.env.AUTOKART_DASHBOARD_PORT ?? 5180), strictPort: true },
  build: { outDir: '../../dist-dashboard', emptyOutDir: true },
});
