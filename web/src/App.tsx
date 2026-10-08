import { useEffect, useState } from "react";
import { api } from "./api/rest";
import { connectStream } from "./api/ws";
import { SpeedPlot } from "./plots/SpeedPlot";
import { useSim } from "./state/sim";
import { RotorView } from "./viz/RotorView";

const fmt = (v: number | undefined, digits: number) => (v === undefined ? "–" : v.toFixed(digits));

/** Walking-skeleton UI (P01.T07): controls, live readouts, one plot and the rotor view. */
export default function App() {
  const { connected, state, error, setConnected, setState, setError } = useSim();
  const [target, setTarget] = useState("20");

  useEffect(
    () => connectStream({ onState: setState, onConnection: setConnected, onError: setError }),
    [setState, setConnected, setError],
  );

  /** Run an API call; surface failures in the status line instead of dropping them. */
  const run = (label: string, call: () => Promise<unknown>) => {
    call().then(
      () => setError(null),
      (err) => setError(`${label} failed: ${String(err)}`),
    );
  };

  const applyTarget = () => {
    const v = Number(target);
    if (Number.isFinite(v)) run("set target speed", () => api.setTargetSpeed(v));
    else setError(`"${target}" is not a number`);
  };

  return (
    <main style={{ maxWidth: 900, margin: "0 auto", padding: 16 }}>
      <h1>BLDC Motor Simulator</h1>
      <p role="status" data-agent-id="signal:sim.connected" aria-label="Connection status">
        {connected ? "● connected" : "○ disconnected"}
      </p>
      {error && (
        <p
          role="alert"
          data-agent-id="signal:ui.error"
          aria-label="Last error"
          style={{ color: "#c0392b" }}
        >
          {error}
        </p>
      )}
      <div style={{ display: "flex", gap: 8, alignItems: "center", flexWrap: "wrap" }}>
        <button
          type="button"
          data-agent-id="action:sim.play"
          aria-label="Play"
          onClick={() => run("play", api.play)}
        >
          Play
        </button>
        <button
          type="button"
          data-agent-id="action:sim.pause"
          aria-label="Pause"
          onClick={() => run("pause", api.pause)}
        >
          Pause
        </button>
        <button
          type="button"
          data-agent-id="action:sim.reset"
          aria-label="Reset"
          onClick={() => run("reset", api.reset)}
        >
          Reset
        </button>
        <label>
          Target speed [rad/s]{" "}
          <input
            data-agent-id="param:ctrl.omega_ref"
            aria-label="Target speed in rad/s"
            value={target}
            size={6}
            onChange={(e) => setTarget(e.target.value)}
            onKeyDown={(e) => e.key === "Enter" && applyTarget()}
          />
        </label>
        <button
          type="button"
          data-agent-id="action:ctrl.omega_ref.apply"
          aria-label="Apply target speed"
          onClick={applyTarget}
        >
          Apply
        </button>
      </div>
      <p>
        t ={" "}
        <span role="status" data-agent-id="signal:sim.t" aria-label="Sim time in seconds">
          {fmt(state?.["sim.t"], 3)}
        </span>{" "}
        s · ω ={" "}
        <span role="status" data-agent-id="signal:motor.omega" aria-label="Motor speed in rad/s">
          {fmt(state?.["motor.omega"], 2)}
        </span>{" "}
        rad/s · i_q ={" "}
        <span role="status" data-agent-id="signal:motor.i_q" aria-label="q-axis current in A">
          {fmt(state?.["motor.i_q"], 3)}
        </span>{" "}
        A · sim/real ={" "}
        <span
          role="status"
          data-agent-id="signal:sim.real_ratio"
          aria-label="Sim to real time ratio"
        >
          {fmt(state?.["sim.real_ratio"], 2)}
        </span>
      </p>
      <RotorView />
      <SpeedPlot />
    </main>
  );
}
