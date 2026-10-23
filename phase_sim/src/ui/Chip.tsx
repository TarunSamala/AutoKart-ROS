import type { ReactNode } from 'react';
import './chip.css';

/** Small state pill, e.g. a mission-state or reachability indicator. */
export function Chip({ children, tone = 'idle', title }: { children: ReactNode; tone?: 'good' | 'warn' | 'crit' | 'idle' | 'accent'; title?: string }) {
  return <span className="ui-chip" data-tone={tone} title={title}>{children}</span>;
}
