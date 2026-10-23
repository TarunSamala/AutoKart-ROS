import type { ReactNode } from 'react';
import './panel.css';

/**
 * Small-caps-titled instrument panel, matching kart/viewer/index.html's `.card` convention:
 * a bordered box with a compact uppercase title and optional right-aligned meta text.
 */
export function Panel({ title, meta, children, className = '' }: { title: string; meta?: ReactNode; children: ReactNode; className?: string }) {
  return (
    <section className={`ui-panel ${className}`} aria-label={title}>
      <header className="ui-panel-head">
        <h3 className="ui-panel-title">{title}</h3>
        {meta && <span className="ui-panel-meta">{meta}</span>}
      </header>
      <div className="ui-panel-body">{children}</div>
    </section>
  );
}
