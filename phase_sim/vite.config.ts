import { defineConfig } from 'vitest/config';
import react from '@vitejs/plugin-react';

export default defineConfig({
  plugins: [react()],
  server: { proxy: {
    '/mission': { target: 'http://127.0.0.1:8765', ws: true },
    '/api/core': 'http://127.0.0.1:8767',
    // Stack B's HP60C-owning process (autokart_perception.live_mission), feeds /live.
    // Diagnosed root cause of the "Failed to fetch" report: this rule was missing,
    // so /api/snapshot fell through to Vite's own SPA-fallback index.html (200, text/html)
    // instead of reaching port 8768 -- see docs/ui_split/AS_IS_MAP.md Section 4.
    '/api/snapshot': 'http://127.0.0.1:8768',
    '/api/image': 'http://127.0.0.1:8768',
  } },
  test: {
    environment: 'jsdom',
    globals: true,
    setupFiles: ['./src/test/setup.ts'],
  },
});
