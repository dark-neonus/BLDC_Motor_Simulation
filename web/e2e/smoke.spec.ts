import { expect, test } from "@playwright/test";

const aid = (id: string) => `[data-agent-id="${id}"]`;

test("play spins the motor and the UI shows it", async ({ page }) => {
  await page.goto("/");
  await expect(page.locator(aid("signal:sim.connected"))).toContainText("connected");

  await page.locator(aid("param:ctrl.omega_ref")).fill("20");
  await page.locator(aid("action:ctrl.set_target")).click();
  await page.locator(aid("action:sim.play")).click();

  // Speed readout must rise above zero within a few seconds.
  await expect
    .poll(async () => Number(await page.locator(aid("signal:motor.omega")).textContent()), {
      timeout: 5_000,
    })
    .toBeGreaterThan(5);

  await page.locator(aid("panel:viz")).locator("canvas").waitFor();
  await page.screenshot({ path: "test-results/smoke.png", fullPage: true });

  await page.locator(aid("action:sim.pause")).click();
});
