import { useEffect, useState } from 'react';
import type { Diagnostic } from '../../shared/labStages';

/**
 * Network allow-list for code written for the Lab. Only two same-origin prefixes exist:
 *   /api/core/*  -> Lab CORE API (autokart_perception.lab_api, 127.0.0.1:8767)
 *   /mission/*   -> Lab-only SIMULATION ROS gateway (127.0.0.1:8775, separate ROS domain)
 * Absolute URLs, the live gateway (:8765) and live perception (:8768, /api/snapshot,
 * /api/image) are refused before any request is made.
 */
export const LAB_ALLOWED_PREFIXES = ['/api/core/', '/mission/'] as const;

export function assertLabUrl(path: string): void {
  if (!LAB_ALLOWED_PREFIXES.some((prefix) => path.startsWith(prefix))) {
    throw new Error(`Lab network policy: refusing request to ${path}`);
  }
}

export async function labJson<T>(path: string, body?: object): Promise<T> {
  assertLabUrl(path);
  const response = await fetch(path, body ? {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  } : { cache: 'no-store' });
  if (!response.ok) throw new Error(`${path}: HTTP ${response.status}`);
  const type = response.headers.get('content-type') ?? '';
  if (!type.includes('application/json')) throw new Error(`${path}: expected JSON, got ${type || 'unknown content'}`);
  return response.json() as Promise<T>;
}

export type CoreStatus = { status: string; vehicle_authorized: boolean; algorithm_version: string; configuration_sha256: string };
export type CoreCone = { id: number; label: string; confidence: number; x_m: number; y_m: number };
export type CoreSnapshot = { source: string; tick: number; cones: CoreCone[];
  kart: { x_m: number; y_m: number; yaw_rad: number; speed_mps: number };
  odometry: { x_m: number; y_m: number; yaw_rad: number }; diagnostic: Diagnostic | null };

export type ServiceState =
  | { state: 'CHECKING' }
  | { state: 'REACHABLE'; checkedAt: number; detail: string; payload: Record<string, unknown> }
  | { state: 'UNREACHABLE'; checkedAt: number; detail: string }
  | { state: 'REFUSED'; checkedAt: number; detail: string };

/**
 * Poll one status endpoint. REACHABLE only means the endpoint answered with JSON; it is
 * never rendered as green/"healthy". `expect` lets a caller refuse a payload whose
 * self-reported source is wrong (e.g. a gateway claiming REAL_KART).
 */
export function useServiceStatus(path: string, intervalMs: number, expect?: (payload: Record<string, unknown>) => string | null): ServiceState {
  const [state, setState] = useState<ServiceState>({ state: 'CHECKING' });
  useEffect(() => {
    let active = true;
    const check = async () => {
      try {
        const payload = await labJson<Record<string, unknown>>(path);
        if (!active) return;
        const problem = expect?.(payload) ?? null;
        setState(problem
          ? { state: 'REFUSED', checkedAt: Date.now(), detail: problem }
          : { state: 'REACHABLE', checkedAt: Date.now(), detail: 'answered with JSON', payload });
      } catch (error) {
        if (active) setState({ state: 'UNREACHABLE', checkedAt: Date.now(), detail: error instanceof Error ? error.message : String(error) });
      }
    };
    void check();
    const timer = window.setInterval(() => void check(), intervalMs);
    return () => { active = false; window.clearInterval(timer); };
    // `expect` is a pure validator; re-subscribing on identity change would hammer the endpoint.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [path, intervalMs]);
  return state;
}

/** The sim gateway must describe itself as SIMULATION; anything else is refused, not displayed. */
export function expectSimulationGateway(payload: Record<string, unknown>): string | null {
  const source = payload.source;
  if (source === 'REAL_KART' || source === 'LIVE') return `gateway reports source ${String(source)} — the Lab only accepts SIMULATION`;
  if (payload.vehicle_authorized === true) return 'gateway reports vehicle_authorized=true — refused';
  return null;
}

export function expectCore(payload: Record<string, unknown>): string | null {
  if (payload.vehicle_authorized === true) return 'lab_api reports vehicle_authorized=true — refused';
  return null;
}

export type Services = { core: ServiceState; simGateway: ServiceState };
