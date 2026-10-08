import { useEffect, useState } from "react";
import { api } from "./api/rest";
import { connectStream } from "./api/ws";
import { SpeedPlot } from "./plots/SpeedPlot";
import { useSim } from "./state/sim";
import { RotorView } from "./viz/RotorView";

/** Walking-skeleton UI (P01.T07): controls, live readouts, one plot and the rotor view. */
export default function App() {
  const { connected, state, setConnected, setState } = useSim();
  const [target, setTarget] = useState("20");

  useEffect(
    () => connectStream({ onState: setState, onConnection: setConnected }),
    [setState, setConnected],
  );

  const applyTarget = () => {
    const v = Number(target);
    if (Number.isFinite(v)) void api.setTargetSpeed(v);
  };

  return (
    <main style={{ maxWidth: 900, margin: "0 auto", padding: 16 }}>
      <h1>BLDC Motor Simulator</h1>
      <p role="status" data-agent-id="signal:sim.connected" aria-label="Connection status">
        {connected ? "● connected" : "○ disconnected"}
      </p>
      <div style={{ display: "flex", gap: 8, alignItems: "center", flexWrap: "wrap" }}>
        <button
          type="button"
          data-agent-id="action:sim.play"
          aria-label="Play"
          onClick={() => void api.play()}
        >
          Play
        </button>
        <button
          type="button"
          data-agent-id="action:sim.pause"
          aria-label="Pause"
          onClick={() => void api.pause()}
        >
          Pause
        </button>
        <button
          type="button"
          data-agent-id="action:sim.reset"
          aria-label="Reset"
          onClick={() => void api.reset()}
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
          data-agent-id="action:ctrl.set_target"
          aria-label="Apply target speed"
          onClick={applyTarget}
        >
          Apply
        </button>
      </div>
      <p>
        t ={" "}
        <span role="status" data-agent-id="signal:sim.t" aria-label="Sim time in seconds">
          {state?.t.toFixed(3) ?? "–"}
        </span>{" "}
        s · ω ={" "}
        <span role="status" data-agent-id="signal:motor.omega" aria-label="Motor speed in rad/s">
          {state?.omega.toFixed(2) ?? "–"}
        </span>{" "}
        rad/s · i_q ={" "}
        <span role="status" data-agent-id="signal:motor.i_q" aria-label="q-axis current in A">
          {state?.i_q.toFixed(3) ?? "–"}
        </span>{" "}
        A · sim/real ={" "}
        <span
          role="status"
          data-agent-id="signal:sim.real_ratio"
          aria-label="Sim to real time ratio"
        >
          {state?.sim_real_ratio.toFixed(2) ?? "–"}
        </span>
      </p>
      <RotorView />
      <SpeedPlot />
    </main>
  );
}
