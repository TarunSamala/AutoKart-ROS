function fit(canvas) {
  const ratio = Math.min(2, window.devicePixelRatio || 1);
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.floor(rect.width * ratio);
  canvas.height = Math.floor(rect.height * ratio);
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return {context, width: rect.width, height: rect.height};
}

export class FrictionCircle {
  constructor(canvas) { this.canvas = canvas; }
  draw(wheel) {
    const {context: ctx, width, height} = fit(this.canvas);
    ctx.fillStyle = "#0a1016"; ctx.fillRect(0, 0, width, height);
    const cx = width / 2, cy = height / 2, radius = Math.min(width, height) * .34;
    ctx.strokeStyle = "#283641"; ctx.lineWidth = 1;
    for (const fraction of [.25, .5, .75, 1]) {
      ctx.beginPath(); ctx.arc(cx, cy, radius * fraction, 0, Math.PI * 2); ctx.stroke();
    }
    ctx.beginPath(); ctx.moveTo(cx - radius - 15, cy); ctx.lineTo(cx + radius + 15, cy);
    ctx.moveTo(cx, cy - radius - 15); ctx.lineTo(cx, cy + radius + 15); ctx.stroke();
    ctx.fillStyle = "#6f7e8a"; ctx.font = "8px IBM Plex Mono, monospace";
    ctx.fillText("Fx", cx + radius + 18, cy + 3); ctx.fillText("Fy", cx + 3, cy - radius - 18);
    const load = Math.max(1, Number(wheel?.vertical_load_n ?? 1));
    const fx = Number(wheel?.longitudinal_force_n ?? 0) / load;
    const fy = Number(wheel?.lateral_force_n ?? 0) / load;
    const px = cx + Math.max(-1.2, Math.min(1.2, fx)) * radius;
    const py = cy - Math.max(-1.2, Math.min(1.2, fy)) * radius;
    ctx.strokeStyle = Number(wheel?.utilisation ?? 0) > .95 ? "#ef5d68" : "#5bc0d8";
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(px, py); ctx.stroke();
    ctx.fillStyle = ctx.strokeStyle; ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI * 2); ctx.fill();
  }
}
