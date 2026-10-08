import { create } from "zustand";
import type { SimState } from "../api/types";
import { RingBuffer } from "../plots/ringBuffer";

/** ~10 s of history at the 30 Hz stream rate: omega, i_q, omega_ref. */
export const history = new RingBuffer(300, 3);

interface SimStore {
  connected: boolean;
  state: SimState | null;
  setConnected: (c: boolean) => void;
  setState: (s: SimState) => void;
}

export const useSim = create<SimStore>((set) => ({
  connected: false,
  state: null,
  setConnected: (connected) => set({ connected }),
  setState: (state) => {
    history.push(state.t, [state.omega, state.i_q, state.omega_ref]);
    set({ state });
  },
}));
