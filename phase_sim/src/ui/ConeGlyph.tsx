/**
 * Small cone-silhouette SVG (a teardrop/triangle with a base), used everywhere a cone needs to
 * be drawn as a cone rather than a plain circle: the MY_TRACK Track overview, Follow view and
 * HP60C camera panel (Tasks 4/5). `size` is the glyph's full height in the SVG's own user
 * units; callers position it with a wrapping `<g transform="translate(...)">`.
 *
 * className drives fill color (mt-cone-blue/yellow/orange, matching the existing dot classes)
 * so no new color logic is needed -- only the shape changes.
 */
export function ConeGlyph({ size = 7, className, opacity = 1 }: { size?: number; className?: string; opacity?: number }) {
  const halfBase = size * 0.42;
  // Teardrop-ish silhouette: narrow rounded apex, flared base -- not a plain triangle.
  const path = `M 0 ${-size * 0.62} C ${size * 0.22} ${-size * 0.3}, ${halfBase} ${size * 0.18}, ${halfBase} ${size * 0.38}` +
    ` L ${-halfBase} ${size * 0.38} C ${-halfBase} ${size * 0.18}, ${-size * 0.22} ${-size * 0.3}, 0 ${-size * 0.62} Z`;
  return <path d={path} className={className} opacity={opacity} />;
}

/** A small hollow ring around a point, used for "detected" tracking markers. */
export function TrackingRing({ r = 5, className }: { r?: number; className?: string }) {
  return <circle r={r} cx={0} cy={0} fill="none" className={className} />;
}

/** A short tick mark, used for "visible but missed" cones in the camera panel. */
export function MissTick({ size = 5, className }: { size?: number; className?: string }) {
  return <line x1={-size / 2} y1={size / 2} x2={size / 2} y2={-size / 2} className={className} />;
}
