import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// AutoKart Algorithm Lab: simulation, replay and algorithm inspection. Separate build,
// port and process from the Vehicle Dashboard. Its ROS gateway is a separate
// SIMULATION-source process on its own port and ROS domain, never the live 8765 gateway.
const LAB_CORE = process.env.AUTOKART_LAB_CORE_URL ?? 'http://127.0.0.1:8767';
const LAB_ROS_GATEWAY = process.env.AUTOKART_LAB_ROS_GATEWAY_URL ?? 'http://127.0.0.1:8775';

export default defineConfig({
  root: 'apps/lab',
  plugins: [react()],
  server: {
    host: '0.0.0.0',
    port: Number(process.env.AUTOKART_LAB_PORT ?? 5181),
    strictPort: true,
    fs: { allow: ['../..'] },
    proxy: {
      '/api/core': LAB_CORE,
      '^/mission/': { target: LAB_ROS_GATEWAY, ws: true },
    },
  },
  preview: { port: Number(process.env.AUTOKART_LAB_PORT ?? 5181), strictPort: true },
  build: { outDir: '../../dist-lab', emptyOutDir: true },
});
