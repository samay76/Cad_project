import { defineConfig, Plugin } from 'vite';
import fs from 'fs';
import path from 'path';

function localFileServerPlugin(): Plugin {
  return {
    name: 'local-file-server',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        if (!req.url) return next();
        const urlObj = new URL(req.url, 'http://localhost:3000');
        const rawPath = decodeURIComponent(urlObj.pathname);

        // Do not intercept vite internal or client files
        if (
          rawPath.startsWith('/@') ||
          rawPath.startsWith('/node_modules') ||
          rawPath.startsWith('/src') ||
          rawPath === '/' ||
          rawPath === '/index.html'
        ) {
          return next();
        }

        // 1. Direct absolute filesystem path (e.g. /home/human/...)
        if (fs.existsSync(rawPath)) {
          try {
            const stat = fs.statSync(rawPath);
            if (stat.isFile()) {
              res.writeHead(200, {
                'Content-Type': 'application/octet-stream',
                'Content-Length': stat.size,
                'Access-Control-Allow-Origin': '*'
              });
              fs.createReadStream(rawPath).pipe(res);
              return;
            }
          } catch {
            // pass to next
          }
        }

        // 2. Relative to workspace directory
        const stripped = rawPath.replace(/^\//, '');
        const workspacePath = path.resolve(process.cwd(), stripped);
        if (fs.existsSync(workspacePath)) {
          try {
            const stat = fs.statSync(workspacePath);
            if (stat.isFile()) {
              res.writeHead(200, {
                'Content-Type': 'application/octet-stream',
                'Content-Length': stat.size,
                'Access-Control-Allow-Origin': '*'
              });
              fs.createReadStream(workspacePath).pipe(res);
              return;
            }
          } catch {
            // pass to next
          }
        }

        next();
      });
    }
  };
}

export default defineConfig({
  server: {
    port: 3000,
    open: false,
    fs: {
      strict: false,
      allow: ['..', '/home/human']
    }
  },
  plugins: [localFileServerPlugin()],
  optimizeDeps: {
    exclude: ['occt-import-js']
  },
  assetsInclude: ['**/*.wasm', '**/*.stp', '**/*.step', '**/*.stl', '**/*.obj']
});
