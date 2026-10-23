import { useEffect, useState } from 'react';

export type Theme = 'light' | 'dark';
const STORAGE_KEY = 'autokart-lab-theme';

/** Reads the persisted manual choice, or null if the user has never picked one (system default applies). */
function storedTheme(): Theme | null {
  try {
    const value = localStorage.getItem(STORAGE_KEY);
    return value === 'light' || value === 'dark' ? value : null;
  } catch {
    return null;
  }
}

function systemTheme(): Theme {
  try {
    return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
  } catch {
    return 'light';
  }
}

function applyTheme(theme: Theme | null) {
  if (theme) document.documentElement.setAttribute('data-theme', theme);
  else document.documentElement.removeAttribute('data-theme');
}

/**
 * App-wide light/dark toggle. On first load (no stored choice) the page follows
 * `prefers-color-scheme` via tokens.css's media query -- no `data-theme` attribute is set, so
 * the system can still flip it live. The first manual click pins a choice to localStorage and
 * sets `data-theme` on `<html>`, which is read by every token consumer (the whole app, not just
 * one page, since tokens.css defines these variables at `:root`).
 */
export function useTheme(): [Theme, () => void] {
  const [manual, setManual] = useState<Theme | null>(() => storedTheme());
  const [system, setSystem] = useState<Theme>(() => systemTheme());

  useEffect(() => {
    applyTheme(manual);
  }, [manual]);

  useEffect(() => {
    let mq: MediaQueryList | null = null;
    try {
      mq = window.matchMedia('(prefers-color-scheme: dark)');
      const onChange = () => setSystem(mq!.matches ? 'dark' : 'light');
      mq.addEventListener('change', onChange);
      return () => mq!.removeEventListener('change', onChange);
    } catch {
      return undefined;
    }
  }, []);

  const effective = manual ?? system;
  const toggle = () => {
    const next: Theme = effective === 'dark' ? 'light' : 'dark';
    setManual(next);
    try { localStorage.setItem(STORAGE_KEY, next); } catch { /* ignore */ }
  };
  return [effective, toggle];
}

export function ThemeToggle() {
  const [theme, toggle] = useTheme();
  const isDark = theme === 'dark';
  return (
    <button type="button" className="lab-theme-toggle" onClick={toggle} aria-pressed={isDark}
      title={`Switch to ${isDark ? 'light' : 'dark'} theme`} aria-label={`Switch to ${isDark ? 'light' : 'dark'} theme`}>
      <span aria-hidden="true">{isDark ? '🌙' : '☀️'}</span>
      <span className="lab-theme-toggle-label">{isDark ? 'Dark' : 'Light'}</span>
    </button>
  );
}
