import React from 'react';
import ReactDOM from 'react-dom/client';
import '@fontsource-variable/manrope';
import '@fontsource-variable/jetbrains-mono';
// Base styles of the reused client-side workbench (/demo). lab.css (imported by LabApp)
// overrides its desktop-only body min-width for the Lab shell.
import '../../styles.css';
import { LabApp } from './LabApp';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode><LabApp /></React.StrictMode>,
);
