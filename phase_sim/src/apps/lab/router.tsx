import { useEffect, useSyncExternalStore, type AnchorHTMLAttributes, type MouseEvent } from 'react';
import { normalizePath } from './routes';

/**
 * Minimal History-API router (no dependency). Paths are real URLs, so every page is a
 * deep link that survives reload: the Lab dev/preview server falls back to index.html
 * and this module resolves `location.pathname` on start-up.
 */
const NAV_EVENT = 'autokart-lab:navigate';

function subscribe(callback: () => void) {
  window.addEventListener('popstate', callback);
  window.addEventListener(NAV_EVENT, callback);
  return () => {
    window.removeEventListener('popstate', callback);
    window.removeEventListener(NAV_EVENT, callback);
  };
}

const snapshot = () => window.location.pathname;

export function navigate(to: string, options: { replace?: boolean } = {}) {
  if (to === window.location.pathname) return;
  if (options.replace) window.history.replaceState(null, '', to);
  else window.history.pushState(null, '', to);
  window.dispatchEvent(new Event(NAV_EVENT));
  if (!options.replace) window.scrollTo?.(0, 0);
}

/** Current canonical path. Aliases ('/', '/shared', trailing slashes) are rewritten in place. */
export function usePath(): string {
  const raw = useSyncExternalStore(subscribe, snapshot, snapshot);
  const path = normalizePath(raw);
  useEffect(() => {
    if (path !== raw) navigate(path + window.location.search + window.location.hash, { replace: true });
  }, [path, raw]);
  return path;
}

type LinkProps = AnchorHTMLAttributes<HTMLAnchorElement> & { to: string };

/** In-app link: a real <a href> (open-in-new-tab, copy link work) with client-side navigation. */
export function Link({ to, onClick, children, ...rest }: LinkProps) {
  const handle = (event: MouseEvent<HTMLAnchorElement>) => {
    onClick?.(event);
    if (event.defaultPrevented || event.button !== 0 || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
    event.preventDefault();
    navigate(to);
  };
  return <a href={to} onClick={handle} {...rest}>{children}</a>;
}
