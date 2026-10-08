import { defineConfig } from "vitest/config";

// Unit tests live next to the code in src/; Playwright specs in e2e/ are run by `just e2e`.
export default defineConfig({
  test: { include: ["src/**/*.test.{ts,tsx}"] },
});
