/** Fixed-capacity time-series buffer (oldest samples are dropped). Lives outside React state. */
export class RingBuffer {
  readonly capacity: number;
  readonly series: number;
  private t: Float64Array;
  private y: Float64Array[];
  private start = 0;
  private count = 0;

  constructor(capacity: number, series: number) {
    this.capacity = capacity;
    this.series = series;
    this.t = new Float64Array(capacity);
    this.y = Array.from({ length: series }, () => new Float64Array(capacity));
  }

  get length(): number {
    return this.count;
  }

  push(t: number, values: number[]): void {
    // A time jump backwards (sim reset) clears the buffer.
    if (this.count > 0 && t < this.t[(this.start + this.count - 1) % this.capacity]) this.clear();
    const idx = (this.start + this.count) % this.capacity;
    this.t[idx] = t;
    for (let s = 0; s < this.series; s++) this.y[s][idx] = values[s] ?? Number.NaN;
    if (this.count < this.capacity) this.count++;
    else this.start = (this.start + 1) % this.capacity;
  }

  clear(): void {
    this.start = 0;
    this.count = 0;
  }

  /** Samples in chronological order as plain arrays: [t, y0, y1, ...] (uPlot data layout). */
  toArrays(): number[][] {
    const out: number[][] = [[], ...this.y.map(() => [])];
    for (let i = 0; i < this.count; i++) {
      const idx = (this.start + i) % this.capacity;
      out[0].push(this.t[idx]);
      for (let s = 0; s < this.series; s++) out[s + 1].push(this.y[s][idx]);
    }
    return out;
  }
}
