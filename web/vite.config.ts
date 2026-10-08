import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// Dev server proxies the API, WebSocket stream and MCP to the Rust server (:8787).
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      "/api": { target: "http://localhost:8787", ws: true },
      "/mcp": { target: "http://localhost:8787" },
    },
  },
});
