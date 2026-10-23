import { createContext, useCallback, useContext, useEffect, useMemo, useState, type ReactNode } from 'react';
import { DEFAULT_LAB_SETTINGS, type LabSettings, type ScenarioName } from '../../mission/labScenarios';
import { decodeReplay } from '../../mission/replay';
import type { MissionFrame } from '../../mission/schema';

/**
 * Lab run state shared by every page.
 *
 * Provenance is one of three labels. LIVE READ-ONLY is part of the vocabulary but cannot be
 * selected in this build: the Lab dev/preview server has no proxy route to the live gateway
 * (:8765) or the live perception service (:8768), so there is no live feed to read.
 */
export type Provenance = 'SYNTHETIC' | 'REPLAY' | 'LIVE READ-ONLY';
export type RunMode = 'SYNTHETIC' | 'REPLAY';
export const LIVE_READ_ONLY_REASON = 'Not available in this build: the Lab has no network route to the live vehicle gateway or live perception service.';

export type Recording = { name: string; file: File; frames: MissionFrame[]; loadedAt: string };
export type RunConfig = { mode: RunMode; runId: string; scenario: ScenarioName; settings: LabSettings };
/** What the current page is actually displaying (null = page shows no data). */
export type PageProvenance = { label: Provenance; detail: string } | null;

type RunContextValue = {
  run: RunConfig;
  setRun: (next: Partial<RunConfig>) => void;
  newRunId: () => void;
  recording: Recording | null;
  openRecording: (file: File) => Promise<Recording>;
  clearRecording: () => void;
  pageProvenance: PageProvenance;
  setPageProvenance: (value: PageProvenance) => void;
};

const STORAGE_KEY = 'autokart-lab-run-v1';
const SCENARIOS: ScenarioName[] = ['SKIDPAD', 'AUTOCROSS', 'BRAKE', 'MY_TRACK'];

export function makeRunId(seed: number, now = new Date()): string {
  const stamp = now.toISOString().replace(/[-:]/g, '').replace(/\..*$/, '').replace('T', '-');
  return `lab-${stamp}Z-s${seed}`;
}

function sanitizeSettings(value: unknown): LabSettings {
  const result = { ...DEFAULT_LAB_SETTINGS };
  if (value && typeof value === 'object') {
    for (const key of Object.keys(result) as Array<keyof LabSettings>) {
      const candidate = (value as Record<string, unknown>)[key];
      if (typeof candidate === 'number' && Number.isFinite(candidate)) result[key] = candidate;
    }
  }
  return result;
}

function loadStored(): RunConfig {
  const fallback: RunConfig = { mode: 'SYNTHETIC', runId: makeRunId(DEFAULT_LAB_SETTINGS.seed), scenario: 'SKIDPAD', settings: { ...DEFAULT_LAB_SETTINGS } };
  try {
    const raw = window.sessionStorage.getItem(STORAGE_KEY);
    if (!raw) return fallback;
    const parsed = JSON.parse(raw) as Partial<RunConfig>;
    return {
      mode: parsed.mode === 'REPLAY' ? 'REPLAY' : 'SYNTHETIC',
      runId: typeof parsed.runId === 'string' && parsed.runId.trim() ? parsed.runId.slice(0, 120) : fallback.runId,
      scenario: SCENARIOS.includes(parsed.scenario as ScenarioName) ? parsed.scenario as ScenarioName : 'SKIDPAD',
      settings: sanitizeSettings(parsed.settings),
    };
  } catch { return fallback; }
}

const RunContext = createContext<RunContextValue | null>(null);

export function RunProvider({ children }: { children: ReactNode }) {
  const [run, setRunState] = useState<RunConfig>(loadStored);
  const [recording, setRecording] = useState<Recording | null>(null);
  const [pageProvenance, setPageProvenance] = useState<PageProvenance>(null);
  useEffect(() => {
    try { window.sessionStorage.setItem(STORAGE_KEY, JSON.stringify(run)); } catch { /* storage unavailable: run config lives in memory only */ }
  }, [run]);
  const setRun = useCallback((next: Partial<RunConfig>) => setRunState((current) => ({ ...current, ...next, settings: next.settings ? sanitizeSettings(next.settings) : current.settings })), []);
  const newRunId = useCallback(() => setRunState((current) => ({ ...current, runId: makeRunId(current.settings.seed) })), []);
  const openRecording = useCallback(async (file: File) => {
    const frames = decodeReplay(await file.text());
    if (!frames.length) throw new Error(`${file.name}: no valid MissionFrame (schema_version 1) records found`);
    const next: Recording = { name: file.name, file, frames, loadedAt: new Date().toISOString() };
    setRecording(next);
    setRunState((current) => ({ ...current, mode: 'REPLAY', runId: String(frames[0].run_id || current.runId) }));
    return next;
  }, []);
  const clearRecording = useCallback(() => setRecording(null), []);
  const value = useMemo(() => ({ run, setRun, newRunId, recording, openRecording, clearRecording, pageProvenance, setPageProvenance }),
    [run, setRun, newRunId, recording, openRecording, clearRecording, pageProvenance]);
  return <RunContext.Provider value={value}>{children}</RunContext.Provider>;
}

export function useRun(): RunContextValue {
  const value = useContext(RunContext);
  if (!value) throw new Error('useRun must be used inside <RunProvider>');
  return value;
}

/** Declare what the current page is displaying; cleared automatically when the page unmounts. */
export function useReportProvenance(label: Provenance | null, detail: string) {
  const { setPageProvenance } = useRun();
  useEffect(() => {
    setPageProvenance(label ? { label, detail } : null);
  }, [label, detail, setPageProvenance]);
  useEffect(() => () => setPageProvenance(null), [setPageProvenance]);
}
