import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  base: process.env.GITHUB_ACTIONS ? '/PAI2026-Advanced1/' : '/',
  build: {
    target: 'es2022',
    sourcemap: true,
  },
})
