const roots: Array<() => void> = [];

export function registerCleanup(callback: () => void) {
  roots.push(callback);
}

export function cleanup() {
  while (roots.length) roots.pop()?.();
  document.body.innerHTML = '';
}
