import React from 'react';
import ReactDOM from 'react-dom/client';
import '@fontsource-variable/manrope';
import '@fontsource-variable/jetbrains-mono';
import { App } from './App';
import { MissionControl } from './mission/MissionControl';
import { MissionControlLive } from './mission/MissionControlLive';
import { SharedLab } from './shared/SharedLab';
import { LiveMissionControl } from './live/LiveMissionControl';
import './styles.css';

// Route map:
//  /live  -> LiveMissionControl   (HP60C camera + perception diagnostic dashboard; real gateway only)
//  /ops   -> MissionControlLive   (full Mission Control world/ROS view, hard-locked to REAL_KART; no sim/lab/replay)
//  /shared -> SharedLab           (shared lab tooling)
//  /demo  -> App                  (pure simulation demo)
//  (default) -> MissionControl    (development/simulation tool: LAB/SIMULATION/REPLAY/REAL_KART switcher)
ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    {location.pathname === '/live' ? <LiveMissionControl />
      : location.pathname === '/ops' ? <MissionControlLive />
      : location.pathname === '/shared' ? <SharedLab />
      : location.pathname === '/demo' ? <App />
      : <MissionControl />}
  </React.StrictMode>,
);
