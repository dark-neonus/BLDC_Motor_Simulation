/** Live state frame from `/api/stream` (skeleton protocol v1). Units: SI. */
export interface SimState {
  t: number;
  omega: number;
  theta: number;
  i_d: number;
  i_q: number;
  i_a: number;
  i_b: number;
  i_c: number;
  omega_ref: number;
  running: boolean;
  time_scale: number;
  sim_real_ratio: number;
}

export interface StateFrame extends SimState {
  type: "state";
  v: number;
}
