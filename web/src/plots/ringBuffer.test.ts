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
});
