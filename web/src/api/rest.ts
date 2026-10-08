import type { SimState } from "./types";

async function post(path: string, body?: unknown): Promise<SimState> {
  const res = await fetch(path, {
    method: "POST",
    headers: body === undefined ? undefined : { "content-type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  if (!res.ok) throw new Error(`${path}: HTTP ${res.status}`);
  return (await res.json()) as SimState;
}

export const api = {
  play: () => post("/api/sim/play"),
  pause: () => post("/api/sim/pause"),
  reset: () => post("/api/sim/reset"),
  setTimeScale: (value: number) => post("/api/sim/time_scale", { value }),
  setTargetSpeed: (value: number) => post("/api/ctrl/target_speed", { value }),
};
