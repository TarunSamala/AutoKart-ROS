export class ReplayController extends EventTarget {
  constructor(store) {
    super();
    this.store = store;
    this.playing = false;
    this.speed = 1;
    this.loop = false;
    this.time = 0;
    this.lastWallMs = performance.now();
    this.animationFrame = null;
  }

  bounds() {
    return {
      start: this.store.records[0]?.timestamp_s ?? 0,
      end: this.store.records.at(-1)?.timestamp_s ?? 0,
    };
  }

  start() {
    if (this.animationFrame != null) return;
    this.lastWallMs = performance.now();
    const tick = now => {
      const dt = Math.min(0.1, (now - this.lastWallMs) / 1000);
      this.lastWallMs = now;
      if (this.playing) {
        const bounds = this.bounds();
        this.time += dt * this.speed;
        if (this.time > bounds.end) {
          if (this.loop) this.time = bounds.start;
          else {
            this.time = bounds.end;
            this.playing = false;
            this.dispatchEvent(new CustomEvent("play", {detail: false}));
          }
        }
        this.seek(this.time, true);
      }
      this.animationFrame = requestAnimationFrame(tick);
    };
    this.animationFrame = requestAnimationFrame(tick);
  }

  setPlaying(playing) {
    this.playing = Boolean(playing);
    this.lastWallMs = performance.now();
    this.dispatchEvent(new CustomEvent("play", {detail: this.playing}));
  }

  toggle() { this.setPlaying(!this.playing); }

  seek(time, interpolated = false) {
    const bounds = this.bounds();
    this.time = Math.max(bounds.start, Math.min(bounds.end, Number(time)));
    this.store.selectTime(this.time, interpolated);
    this.dispatchEvent(new CustomEvent("seek", {detail: this.time}));
  }

  restart() {
    this.seek(this.bounds().start);
  }

  step(direction) {
    if (!this.store.records.length) return;
    const currentIndex = this.store.records.findIndex(record => record.timestamp_s >= this.time);
    const index = Math.max(0, Math.min(this.store.records.length - 1, currentIndex + direction));
    this.seek(this.store.records[index].timestamp_s);
  }

  jumpToFault() {
    const event = this.store.faultEvents.find(item => item.timestamp_s > this.time + 1e-9)
      ?? this.store.faultEvents[0];
    if (event) this.seek(event.timestamp_s);
  }
}
