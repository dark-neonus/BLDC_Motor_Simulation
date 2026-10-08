import { describe, expect, it } from "vitest";
import { RingBuffer } from "./ringBuffer";

describe("RingBuffer", () => {
  it("keeps the newest samples in order when full", () => {
    const b = new RingBuffer(3, 1);
    for (let i = 0; i < 5; i++) b.push(i, [i * 10]);
    expect(b.toArrays()).toEqual([
      [2, 3, 4],
      [20, 30, 40],
    ]);
  });

  it("clears when time jumps backwards (sim reset)", () => {
    const b = new RingBuffer(4, 2);
    b.push(1, [1, 2]);
    b.push(2, [3, 4]);
    b.push(0, [5, 6]);
    expect(b.length).toBe(1);
    expect(b.toArrays()).toEqual([[0], [5], [6]]);
  });

  it("ignores repeated frames with the same time (paused sim keeps history)", () => {
    const b = new RingBuffer(3, 1);
    b.push(1, [1]);
    b.push(2, [2]);
    for (let i = 0; i < 10; i++) b.push(2, [2]);
    expect(b.toArrays()).toEqual([
      [1, 2],
      [1, 2],
    ]);
  });

  it("windows by sim time", () => {
    const b = new RingBuffer(10, 1);
    for (let t = 0; t <= 5; t++) b.push(t, [t]);
    expect(b.toArrays(2)[0]).toEqual([3, 4, 5]);
  });
});
