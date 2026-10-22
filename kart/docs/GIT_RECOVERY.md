# Git Metadata Recovery

## Observed failure

Command:

```bash
git status --short
```

Result:

```text
fatal: not a git repository (or any of the parent directories): .git
```

Inspection with `find .git -maxdepth 3 -printf '%y %p\n'` reports only:

```text
d .git
```

`.git` is therefore an empty directory with missing repository metadata. It is
not a worktree pointer or submodule pointer (those would be files containing a
`gitdir:` path), and it is not a usable Git directory. No commit hash is
available.

## Non-destructive recovery options

1. Recover the original `.git` directory from a trusted backup of this exact
   working tree.
2. Clone the authoritative repository into a separate directory, compare it to
   this checkpoint using `diff --no-dereference --recursive`, and deliberately
   transfer the changed files listed in `docs/CHANGE_MANIFEST.md`.
3. Apply the portable source archive or a separately generated patch to a
   verified clone.
4. Reinitialization would create unrelated history and must be an explicit
   user decision only after preserving this tree; it was not performed.

No `.git` content was deleted, replaced, reset, or reinitialized.
