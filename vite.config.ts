import { defineConfig } from 'vite';

export default defineConfig({
  server: {
    port: 3000,
    open: false
  },
  optimizeDeps: {
    exclude: ['occt-import-js']
  },
  assetsInclude: ['**/*.wasm', '**/*.stp', '**/*.step', '**/*.stl', '**/*.obj']
});
