import { Application, Container, Graphics } from "pixi.js";

/** Pole pairs of the skeleton motor (P01); real geometry comes from the API in P14. */
const POLE_PAIRS = 14;

/**
 * Minimal motor cross-section: stator ring and an outer rotor with 2·p alternating
 * magnets, rotated to the latest mechanical angle every animation frame.
 */
export async function createRotorView(
  host: HTMLElement,
  getTheta: () => number,
): Promise<() => void> {
  const app = new Application();
  await app.init({ background: "#262624", antialias: true, resizeTo: host });
  host.appendChild(app.canvas);

  const scene = new Container();
  app.stage.addChild(scene);

  const stator = new Graphics().circle(0, 0, 70).fill(0x5a5a55).circle(0, 0, 20).fill(0x262624);
  const rotor = new Container();
  const magnets = new Graphics();
  const n = 2 * POLE_PAIRS;
  for (let k = 0; k < n; k++) {
    const a0 = (2 * Math.PI * k) / n;
    const a1 = (2 * Math.PI * (k + 1)) / n - 0.02;
    magnets
      .moveTo(78 * Math.cos(a0), 78 * Math.sin(a0))
      .arc(0, 0, 78, a0, a1)
      .lineTo(92 * Math.cos(a1), 92 * Math.sin(a1))
      .arc(0, 0, 92, a1, a0, true)
      .closePath()
      .fill(k % 2 === 0 ? 0xd97757 : 0x6a9bcc);
  }
  const marker = new Graphics().rect(92, -3, 14, 6).fill(0xf5f4ee);
  rotor.addChild(magnets, marker);
  scene.addChild(stator, rotor);

  const tick = () => {
    scene.position.set(app.screen.width / 2, app.screen.height / 2);
    const s = Math.min(app.screen.width, app.screen.height) / 230;
    scene.scale.set(s);
    // Screen y points down, so negate for counter-clockwise-positive rotation.
    rotor.rotation = -getTheta();
  };
  app.ticker.add(tick);

  return () => {
    app.ticker.remove(tick);
    app.destroy(true, { children: true });
  };
}
