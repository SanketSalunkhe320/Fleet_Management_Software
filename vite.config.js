// import { defineConfig } from 'vite'
// import react from '@vitejs/plugin-react'

// export default defineConfig({
//   plugins: [react()],
// })

import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    host: true,
    proxy: {
      '/api/hiwonder': {
        target: 'http://localhost:3002',
        changeOrigin: true,
        secure: false,
        ws: false, // Set to false if not using WebSocket through proxy
      }
    }
  }
})