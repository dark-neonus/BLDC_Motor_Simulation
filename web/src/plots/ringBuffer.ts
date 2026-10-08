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

  /** Time of the newest sample, or NaN when empty. */
  get lastTime(): number {
    return this.count > 0 ? this.t[(this.start + this.count - 1) % this.capacity] : Number.NaN;
  }

  push(t: number, values: number[]): void {
    const last = this.lastTime;
    // Same time as the newest sample (sim paused, frames keep arriving): nothing new to store.
    if (t === last) return;
    // A time jump backwards (sim reset) clears the buffer.
    if (t < last) this.clear();
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

  /**
   * Samples in chronological order as plain arrays: [t, y0, y1, ...] (uPlot data layout).
   * With `window`, only samples within the last `window` seconds of *sim* time are returned.
   */
  toArrays(window = Number.POSITIVE_INFINITY): number[][] {
    const out: number[][] = [[], ...this.y.map(() => [])];
    const tMin = this.lastTime - window;
    for (let i = 0; i < this.count; i++) {
      const idx = (this.start + i) % this.capacity;
      if (this.t[idx] < tMin) continue;
      out[0].push(this.t[idx]);
      for (let s = 0; s < this.series; s++) out[s + 1].push(this.y[s][idx]);
    }
    return out;
  }
}
