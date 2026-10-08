import { create } from "zustand";
import type { SimState } from "../api/types";
import { RingBuffer } from "../plots/ringBuffer";

/** Plot history (omega, i_q, omega_ref); the plot shows the last PLOT_WINDOW_S of sim time. */
export const history = new RingBuffer(4000, 3);
export const PLOT_WINDOW_S = 10;

interface SimStore {
  connected: boolean;
  state: SimState | null;
  /** Last user-visible error (failed request, bad frame, no WebGL…). */
  error: string | null;
  setConnected: (c: boolean) => void;
  setState: (s: SimState) => void;
  setError: (e: string | null) => void;
}

export const useSim = create<SimStore>((set) => ({
  connected: false,
  state: null,
  error: null,
  setConnected: (connected) => set({ connected }),
  setState: (state) => {
    history.push(state["sim.t"], [
      state["motor.omega"],
      state["motor.i_q"],
      state["ctrl.omega_ref"],
    ]);
    set({ state });
  },
  setError: (error) => set({ error }),
}));
