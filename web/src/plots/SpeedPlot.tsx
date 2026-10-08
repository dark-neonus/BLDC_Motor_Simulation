import { useEffect, useRef } from "react";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";
import { history, PLOT_WINDOW_S } from "../state/sim";

/** Rolling plot of motor.omega, ctrl.omega_ref and motor.i_q (redrawn each animation frame). */
export function SpeedPlot() {
  const host = useRef<HTMLElement>(null);

  useEffect(() => {
    const el = host.current;
    if (!el) return;
    const opts: uPlot.Options = {
      width: el.clientWidth || 600,
      height: 260,
      scales: { x: { time: false } },
      axes: [{ label: "t [s]" }, { label: "ω [rad/s]" }, { side: 1, scale: "A", label: "i_q [A]" }],
      series: [
        {},
        { label: "motor.omega", stroke: "#D97757", width: 2 },
        { label: "motor.i_q", stroke: "#6A9BCC", scale: "A" },
        { label: "ctrl.omega_ref", stroke: "#888", dash: [6, 4] },
      ],
    };
    const plot = new uPlot(opts, [[], [], [], []], el);
    let raf = 0;
    let lastT = Number.NaN;
    const draw = () => {
      const last = history.lastTime;
      if (last !== lastT) {
        const arrays = history.toArrays(PLOT_WINDOW_S);
        plot.setData([arrays[0], arrays[1], arrays[2], arrays[3]]);
        lastT = last;
      }
      raf = requestAnimationFrame(draw);
    };
    raf = requestAnimationFrame(draw);
    const ro = new ResizeObserver(() => plot.setSize({ width: el.clientWidth, height: 260 }));
    ro.observe(el);
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      plot.destroy();
    };
  }, []);

  return <section ref={host} data-agent-id="panel:plots" aria-label="Speed and current plot" />;
}
