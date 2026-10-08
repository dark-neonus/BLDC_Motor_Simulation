/**
 * Live state frame from `/api/stream` (skeleton protocol v1). Keys are the dotted
 * signal paths of CONVENTIONS §3; units SI.
 * TODO(P12.T04): replace with types generated from the OpenAPI spec (`just gen-api`).
 */
export interface SimState {
  "sim.t": number;
  "motor.omega": number;
  "motor.theta": number;
  "motor.i_d": number;
  "motor.i_q": number;
  "motor.i_a": number;
  "motor.i_b": number;
  "motor.i_c": number;
  "ctrl.omega_ref": number;
  "sim.running": boolean;
  "sim.time_scale": number;
  "sim.real_ratio": number;
}

export interface StateFrame extends SimState {
  type: "state";
  v: number;
}
