// Loads the shared MY_TRACK layout (Rev 7), the single source of truth for both
// the Python Layer A simulator and this Lab -- see src/mission/data/README.md.
// Never hand-duplicate cone coordinates; this file only shapes the JSON into the
// Lab's Cone/LabScenario types.
import layoutData from './data/my_track_layout.json';
import type { Cone, Point } from './schema';
import type { LabScenario } from './labScenarios';

interface RawCone {
  id: string;
  color: 'BLUE' | 'YELLOW' | 'ORANGE';
  role: string;
  station: number;
  lat: number;
  x: number;
  y: number;
}

interface RawLayout {
  rev: number;
  corridor_m: number;
  length_m: number;
  spacing_m: number;
  sections: Record<string, number[]>;
  start_pose: { x: number; y: number; heading_deg: number; note: string };
  cones: RawCone[];
}

const raw = layoutData as RawLayout;

export const MY_TRACK_LAYOUT = raw;

export function validateMyTrackLayout(layout: RawLayout = raw): void {
  if (layout.cones.length !== 120) {
    throw new Error(`MY_TRACK layout must have exactly 120 cones, found ${layout.cones.length}`);
  }
  const counts: Record<string, number> = { BLUE: 0, YELLOW: 0, ORANGE: 0 };
  for (const cone of layout.cones) counts[cone.color] = (counts[cone.color] ?? 0) + 1;
  const expected = { BLUE: 20, YELLOW: 20, ORANGE: 80 };
  if (counts.BLUE !== expected.BLUE || counts.YELLOW !== expected.YELLOW || counts.ORANGE !== expected.ORANGE) {
    throw new Error(`MY_TRACK layout colour counts ${JSON.stringify(counts)} do not match ${JSON.stringify(expected)}`);
  }
}

/** Builds a LabScenario from the shared layout. Role metadata (finish line,
 * obstacle, etc.) is NOT passed through as cone class -- same as the Python
 * side, the algorithm must discover obstacle/boundary/finish from geometry,
 * not from ground-truth labels. */
export function myTrackScenario(): LabScenario {
  validateMyTrackLayout(raw);
  const cones: Cone[] = raw.cones.map((c) => ({ id: c.id, class: c.color, confidence: 1, x_m: c.x, y_m: c.y }));
  const guide: Point[] = [];
  for (let s = 0; s <= raw.length_m; s += 1) guide.push({ x_m: s, y_m: 0 });
  const start = { x_m: raw.start_pose.x, y_m: raw.start_pose.y, heading: (raw.start_pose.heading_deg * Math.PI) / 180 };
  const finishCones = raw.cones.filter((c) => c.role === 'finish line');
  const markers = finishCones.map((c) => ({ id: c.id, label: 'FINISH', x_m: c.x, y_m: c.y }));
  return { name: 'MY_TRACK', cones, guide, start, markers };
}
