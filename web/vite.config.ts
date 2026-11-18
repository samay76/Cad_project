import { defineConfig } from 'vite';

// Development: `npm run dev` serves the web app on :3000 and forwards /api to the cell server
// (python -m server, default 127.0.0.1:8000). Production: `npm run build`, then the server serves web/dist.
const API = process.env.CELL_API || 'http://127.0.0.1:8000';

export default defineConfig({
  server: {
    port: 3000,
    open: false,
    proxy: { '/api': { target: API, changeOrigin: false } }
  },
  optimizeDeps: {
    exclude: ['occt-import-js']
  },
  assetsInclude: ['**/*.wasm', '**/*.stp', '**/*.step', '**/*.stl', '**/*.obj']
});
