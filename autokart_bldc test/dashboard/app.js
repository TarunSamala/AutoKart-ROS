const $ = (id) => document.getElementById(id);
const palette = { blue: '#56a6ff', yellow: '#ffe166', orange: '#ff9b54',
  large_orange: '#ff704d', unknown: '#b9bdc7' };
let latest = null;
let lastFrameId = null;
let lastReplyAt = 0;

function num(value, digits = 1) {
  return Number.isFinite(value) ? value.toFixed(digits) : '—';
}
function set(id, value) { $(id).textContent = value; }
function stage(id, kind, headline, detail) {
  const root = $('stage-' + id);
  root.className = 'stage ' + kind;
  root.querySelector('strong').textContent = headline;
  root.querySelector('small').textContent = detail;
}
function badge(id, value, kind) {
  const node = $(id);
  node.textContent = value;
  node.className = 'badge ' + kind;
}
function dotColor(label) { return palette[label] || palette.unknown; }

function renderDetections(items) {
  const body = $('detections-body');
  body.replaceChildren();
  if (!items.length) {
    const row = document.createElement('tr');
    const cell = document.createElement('td');
    cell.colSpan = 5;
    cell.className = 'muted';
    cell.textContent = 'No detections in this frame';
    row.append(cell); body.append(row);
    return;
  }
  for (const item of items.slice(0, 30)) {
    const row = document.createElement('tr');
    const values = [
      '#' + item.id,
      item.label,
      num(item.confidence * 100, 0) + '%',
      Number.isFinite(item.candidate_distance_m) ? '~' + num(item.candidate_distance_m, 2) + ' m candidate' : 'rejected',
      (item.localization_status || 'UNAVAILABLE') +
        ((item.validation_reasons || []).length ? ' · UNVERIFIED' : ''),
    ];
    values.forEach((value, index) => {
      const cell = document.createElement('td');
      if (index === 4 && item.validation_reasons) {
        cell.title = item.validation_reasons.join(', ');
      }
      if (index === 1) {
        const dot = document.createElement('span');
        dot.className = 'cone-dot';
        dot.style.backgroundColor = dotColor(item.label);
        cell.append(dot);
      }
      cell.append(document.createTextNode(value));
      row.append(cell);
    });
    body.append(row);
  }
}

function renderStatus(d) {
  const p = d.pipeline || {};
  const detections = d.detections || [];
  const cones = d.birds_eye_cones || [];
  const camera = !!d.camera_ok;
  const age = p.frame_age_ms;
  const stale = Number.isFinite(age) && age > 250;
  badge('stream-badge', camera ? (stale ? 'CAMERA FRAME STALE' : 'CAMERA LIVE') : 'CAMERA UNAVAILABLE',
        camera && !stale ? '' : 'warn');
  badge('safety-badge', d.vehicle_authorized_by_perception ? 'PERCEPTION AUTHORIZED' :
        'NO MOTION AUTHORITY', d.vehicle_authorized_by_perception ? '' : 'bad');
  set('camera-meta', camera ? 'Frame ' + d.frame_count + ' · ' + num(p.rgb_fps, 1) + ' RGB fps' : 'No frame');
  set('bev-meta', cones.length + ' diagnostic candidate' + (cones.length === 1 ? '' : 's'));
  set('detector-label', 'Detector: ' + (p.model || 'unavailable') + (d.perception_enabled ? '' : ' · PAUSED'));
  set('detection-count', detections.length + ' detection' + (detections.length === 1 ? '' : 's'));
  set('path-label', d.centerline_xy && d.centerline_xy.length > 1 ?
      'CANDIDATE CENTERLINE · NOT AUTHORIZED' : 'NO QUALIFIED PATH · ' + (d.corridor_status || 'WAITING'));
  set('metric-detections', String(detections.length));
  set('metric-detections-sub', num(p.inference_ms, 0) + ' ms inference');
  set('metric-depth', String(cones.length));
  set('metric-corridor', d.corridor_status || '—');
  set('metric-corridor-sub', (p.left_boundary_cones || 0) + ' left · ' +
      (p.right_boundary_cones || 0) + ' right · ' + (p.candidate_gates || 0) + ' gates');
  set('metric-steering', Number.isFinite(d.steering_angle_rad) ?
      num(d.steering_angle_rad * 180 / Math.PI, 1) + '°' : 'UNAVAILABLE');
  set('metric-curvature', Number.isFinite(d.steering_angle_rad) ? 'Preview only' : 'No qualified path profile');
  set('metric-dac', String(d.dac_mv_preview || 0) + ' mV');
  set('metric-stop', d.decision === 0 ? 'STOP' : 'OBSERVE');
  set('metric-stop-sub', d.reason || 'Software decision only');
  set('frame-age', Number.isFinite(age) ? num(age, 0) + ' ms frame age' : 'No frame age');

  stage('camera', camera && !stale ? 'ok' : 'bad', camera ? (stale ? 'STALE' : 'LIVE') : 'NO FRAME',
        num(p.rgb_fps, 1) + ' RGB fps · ' + num(p.depth_fps, 1) + ' depth fps');
  stage('detector', !d.perception_enabled ? 'warn' : d.detector_error ? 'bad' : 'ok',
        !d.perception_enabled ? 'PAUSED' : d.detector_error ? 'MODEL ERROR' : detections.length + ' BOXES',
        num(p.inference_ms, 0) + ' ms · ' + (p.detector_kind || '—'));
  stage('depth', cones.length ? 'warn' : 'bad', cones.length + ' CANDIDATES',
        num(p.valid_depth_pct, 0) + '% valid pixels · scale unverified');
  stage('graph', p.delaunay_edges ? 'warn' : 'bad', p.graph_status || 'NO GRAPH',
        (p.delaunay_edges || 0) + ' Delaunay · ' + (p.gabriel_edges || 0) + ' Gabriel');
  stage('corridor', d.centerline_xy && d.centerline_xy.length > 1 ? 'warn' : 'bad',
        d.corridor_status || 'NO PATH', (p.candidate_gates || 0) + ' gates · ' +
        (p.left_boundary_cones || 0) + ' L / ' + (p.right_boundary_cones || 0) + ' R');
  stage('steering', Number.isFinite(d.steering_angle_rad) ? 'warn' : 'bad',
        Number.isFinite(d.steering_angle_rad) ? num(d.steering_angle_rad * 180 / Math.PI, 1) + '°' : 'BLOCKED',
        'Preview; no steering output');
  stage('obstacle', d.obstacle_blocked ? 'warn' : 'warn',
        d.obstacle_blocked ? 'CONE IN PATH' : 'UNQUALIFIED',
        Number.isFinite(d.obstacle_forward_m) ? '~' + num(d.obstacle_forward_m, 2) + ' m candidate' :
        'No qualified clearance');
  stage('stop', d.decision === 0 ? 'ok' : 'warn', d.decision === 0 ? 'STOP PREVIEW' : 'OBSERVE',
        d.stop_controller_state || '—');
  stage('pico', d.vcu_identity_verified ? 'warn' : 'bad',
        d.vcu_identity_verified ? 'PICO VERIFIED' : 'NO VERIFIED LINK',
        d.vcu_link_fresh ? 'Command link fresh' : 'Command link stale · read only');

  const reasons = [...new Set([...(p.safety_reasons || []), d.reason].filter(Boolean))];
  const reasonBox = $('reasons');
  reasonBox.replaceChildren();
  if (!reasons.length) reasonBox.textContent = 'No current reasons';
  for (const reason of reasons) {
    const item = document.createElement('span');
    item.className = 'reason';
    item.textContent = reason;
    reasonBox.append(item);
  }
  renderDetections(detections);
  const t = d.vcu_telemetry || {};
  set('telemetry', 'Pico: ' + (d.vcu_hello ? d.vcu_hello[0] : 'no HELLO') +
      ' · state ' + (t.state || 'unknown') + ' · EN ' + (t.en ?? '—') +
      ' · BRK ' + (t.brk ?? '—') + ' · DAC ' + (t.dac_mv ?? '—') +
      ' mV · commands sent: none');
  set('raw-status', JSON.stringify(d, null, 2));
  const img = $('camera-image');
  const placeholder = $('camera-empty');
  if (camera && d.frame_id) {
    img.style.display = 'block';
    placeholder.style.display = 'none';
    if (lastFrameId !== d.frame_id) {
      lastFrameId = d.frame_id;
      img.src = '/snapshot.jpg?frame=' + encodeURIComponent(d.frame_id);
    }
  } else {
    img.style.display = 'none';
    placeholder.style.display = 'grid';
    placeholder.textContent = d.camera_error || 'Waiting for camera frame';
    lastFrameId = null;
  }
}

async function poll() {
  try {
    const response = await fetch('/api/status', {cache: 'no-store'});
    if (!response.ok) throw new Error('HTTP ' + response.status);
    const data = await response.json();
    latest = data;
    lastReplyAt = performance.now();
    renderStatus(data);
  } catch (error) {
    badge('stream-badge', 'DASHBOARD DISCONNECTED', 'bad');
    set('camera-meta', String(error));
  }
  setTimeout(poll, 350);
}

function drawBirdsEye(time) {
  const canvas = $('bev-canvas');
  const ctx = canvas.getContext('2d');
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const ratio = window.devicePixelRatio || 1;
  if (w && h && (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio))) {
    canvas.width = Math.round(w * ratio); canvas.height = Math.round(h * ratio);
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#081724'; ctx.fillRect(0, 0, w, h);
  const scale = Math.min((w - 70) / 7, (h - 55) / 6);
  const originX = w / 2, originY = h - 32;
  const xy = (forward, left) => [originX - left * scale, originY - forward * scale];
  ctx.strokeStyle = '#193442'; ctx.lineWidth = 1;
  for (let m = 0; m <= 6; m++) {
    const y = originY - m * scale;
    ctx.beginPath(); ctx.moveTo(30, y); ctx.lineTo(w - 20, y); ctx.stroke();
    ctx.fillStyle = '#63818f'; ctx.font = '11px ui-monospace, monospace'; ctx.fillText(m + ' m', 6, y - 3);
  }
  for (let m = -3; m <= 3; m++) {
    const x = originX - m * scale;
    ctx.beginPath(); ctx.moveTo(x, 16); ctx.lineTo(x, originY); ctx.stroke();
  }
  ctx.strokeStyle = '#3798a7'; ctx.lineWidth = 2; ctx.setLineDash([6, 5]);
  ctx.beginPath(); ctx.moveTo(originX, 16); ctx.lineTo(originX, originY); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = '#42c4bf'; ctx.beginPath();
  ctx.moveTo(originX, originY - 24); ctx.lineTo(originX - 13, originY + 9);
  ctx.lineTo(originX + 13, originY + 9); ctx.closePath(); ctx.fill();
  ctx.fillStyle = '#97c9ca'; ctx.font = '11px ui-monospace, monospace';
  ctx.fillText('KART', originX + 17, originY + 4);
  const fresh = performance.now() - lastReplyAt < 1300;
  const points = fresh && latest ? (latest.birds_eye_cones || []) : [];
  const path = fresh && latest ? (latest.centerline_xy || []) : [];
  if (path.length > 1) {
    ctx.strokeStyle = '#eee9b0'; ctx.lineWidth = 3; ctx.setLineDash([9, 6]);
    ctx.beginPath();
    path.forEach((point, index) => {
      const [x, y] = xy(point[0], point[1]);
      if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke(); ctx.setLineDash([]);
  }
  for (const cone of points) {
    if (!Number.isFinite(cone.x_forward_m) || !Number.isFinite(cone.y_left_m)) continue;
    const [x, y] = xy(cone.x_forward_m, cone.y_left_m);
    if (x < 0 || x > w || y < 0 || y > h) continue;
    const color = dotColor(cone.label);
    const pulse = 11 + 3 * Math.sin(time / 250 + cone.id);
    ctx.strokeStyle = color; ctx.globalAlpha = .45; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(x, y, pulse, 0, Math.PI * 2); ctx.stroke();
    ctx.globalAlpha = 1; ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#eaf5f8'; ctx.font = 'bold 11px ui-monospace, monospace';
    ctx.fillText('#' + cone.id, x + 8, y - 7);
  }
  if (!fresh) {
    ctx.fillStyle = '#081724bf'; ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = '#f3c27e'; ctx.font = 'bold 15px ui-sans-serif, sans-serif';
    ctx.fillText('WAITING FOR LIVE PERCEPTION', Math.max(20, w / 2 - 125), h / 2);
  }
  requestAnimationFrame(drawBirdsEye);
}

document.querySelectorAll('button[data-action]').forEach((button) => {
  button.addEventListener('click', async () => {
    try {
      const response = await fetch('/api/control', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: button.dataset.action}),
      });
      if (!response.ok) throw new Error('HTTP ' + response.status);
      set('action-message', button.textContent + ' · applied to preview only');
    } catch (error) {
      set('action-message', 'Control request failed: ' + error);
    }
  });
});
poll();
requestAnimationFrame(drawBirdsEye);
