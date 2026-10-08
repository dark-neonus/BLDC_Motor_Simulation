import { useEffect, useRef } from "react";
import { useSim } from "../state/sim";
import { createRotorView } from "./rotor";

export function RotorView() {
  const host = useRef<HTMLElement>(null);

  useEffect(() => {
    const el = host.current;
    if (!el) return;
    let dispose: (() => void) | undefined;
    let cancelled = false;
    createRotorView(el, () => useSim.getState().state?.theta ?? 0).then((d) => {
      if (cancelled) d();
      else dispose = d;
    });
    return () => {
      cancelled = true;
      dispose?.();
    };
  }, []);

  return (
    <section
      ref={host}
      data-agent-id="panel:viz"
      aria-label="Motor visualization"
      style={{ width: "100%", height: 320 }}
    />
  );
}
