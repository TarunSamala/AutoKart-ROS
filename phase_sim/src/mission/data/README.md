This directory holds a byte-identical copy of `docs/my_track_spec/my_track_layout.json`,
the single source of truth for the MY_TRACK cone layout (shared between the Python
Layer A simulator and this TypeScript Lab). Do not hand-edit this copy — re-copy it
from `docs/my_track_spec/my_track_layout.json` if that file changes. Vite's dev-server
`fs.allow` doesn't reach outside `phase_sim/`, so a static copy (not independently
authored data) is simpler than a cross-directory fetch.
