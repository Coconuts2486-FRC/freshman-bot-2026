#!/usr/bin/env python3
"""
Simple template sync (cross-platform) + --atomic:

- Applies template changes onto the *current* branch of the target repo.
- Default template branch is "main" (override with --template-branch).
- No PR creation, no labels.

Reads TEMPLATE_ORIGIN.txt for:
  Template: owner/repo
  Template Commit: <sha>

Options you asked for:
- --template-branch <name>
- --skip-workflows true/false (default true)
- --no-origin-update (skip updating TEMPLATE_ORIGIN.txt)

Safety / QoL:
- Refuse if working tree dirty (unless --auto-stash).
- --auto-stash stashes before sync and pops afterward.
- --dry-run

NEW: --atomic
- Runs the sync on a temporary branch starting at your current HEAD.
- If (and only if) everything succeeds, fast-forwards your current branch to the temp branch result.
- If anything fails, your current branch is left untouched.

----

Template Sync CLI (Simple + Atomic)
==================================

A lightweight, cross-platform command-line tool for syncing a repository with its GitHub template.

Unlike the full GitHub Action workflow, this tool:

- Works locally (Linux / macOS / WSL / Windows PowerShell)
- Applies updates to the **current branch**
- Does **not** create PRs or labels
- Supports safe atomic updates
- Supports auto-stashing
- Can optionally skip updating ``TEMPLATE_ORIGIN.txt``

Overview
--------

This script reads ``TEMPLATE_ORIGIN.txt`` from the root of your repository and:

1. Fetches the template repository
2. Finds commits since the last recorded template commit
3. Filters out:

   - Merge commits
   - (Optionally) commits modifying ``.github/workflows/``

4. Cherry-picks the remaining commits
5. Squashes them into a single commit (default)
6. Updates ``TEMPLATE_ORIGIN.txt`` (unless disabled)

With ``--atomic``, all changes are applied to a temporary branch and only fast-forwarded into your working branch if everything succeeds.

Requirements
------------

- ``git`` installed and available on PATH
- Python 3.8+
- A valid ``TEMPLATE_ORIGIN.txt`` file in the repo root

Required File: ``TEMPLATE_ORIGIN.txt``
--------------------------------------

The script expects::

  Template: owner/repository
  Template Commit: <sha>

Optional fields (ignored unless updating origin)::

  Template Branch: main
  Recorded At (UTC): <timestamp>

If the file is missing or malformed, the script exits cleanly.

Basic Usage
-----------

Default (safe mode)
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  python3 sync_template_simple.py

- Refuses to run if working tree is dirty
- Applies changes directly to current branch
- Squashes into one commit
- Updates ``TEMPLATE_ORIGIN.txt``

Recommended Usage
-----------------

Atomic Mode (Safest)
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  python3 sync_template_simple.py --atomic

This:

1. Creates a temporary branch at current HEAD
2. Applies all template changes there
3. If successful:

   - Fast-forwards your branch to the updated commit

4. If anything fails:

   - Your original branch remains untouched

Use this mode if you want strong safety guarantees.

Options
-------

``--template-branch <name>``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Specify template branch (default: ``main``).

.. code-block:: bash

  python3 sync_template_simple.py --template-branch develop

``--skip-workflows true|false``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Default: ``true``

Skips commits modifying ``.github/workflows/``.

.. code-block:: bash

  python3 sync_template_simple.py --skip-workflows false

``--squash true|false``
~~~~~~~~~~~~~~~~~~~~~~~

Default: ``true``

Squashes all applied commits into a single commit.

``--no-origin-update``
~~~~~~~~~~~~~~~~~~~~~~

Do not update ``TEMPLATE_ORIGIN.txt``.

.. code-block:: bash

  python3 sync_template_simple.py --no-origin-update

Useful for testing or dry-runs without changing recorded state.

``--auto-stash``
~~~~~~~~~~~~~~~~

If working tree is dirty:

- Automatically runs ``git stash push -u``
- Applies template sync
- Runs ``git stash pop`` afterward

.. code-block:: bash

  python3 sync_template_simple.py --auto-stash

Without this flag, the script aborts if the working tree is dirty.

``--dry-run``
~~~~~~~~~~~~~

Show what would happen without making changes.

.. code-block:: bash

  python3 sync_template_simple.py --dry-run

``--atomic``
~~~~~~~~~~~~

Enable atomic mode (recommended for production use).

.. code-block:: bash

  python3 sync_template_simple.py --atomic

Guarantees your branch is only modified if the sync completes successfully.

Behavior Details
----------------

Merge Commit Handling
~~~~~~~~~~~~~~~~~~~~~

- All merge commits from the template are skipped.
- Only non-merge commits are considered.

Workflow Commit Handling
~~~~~~~~~~~~~~~~~~~~~~~~

By default, commits touching::

  .github/workflows/

are skipped to prevent automation conflicts.

Commit Strategy
~~~~~~~~~~~~~~~

By default:

- All cherry-picked changes are staged using ``--no-commit``
- A single squash commit is created with the message::

  Template Sync Updates

You may disable squashing if desired.

No-Op Protection
~~~~~~~~~~~~~~~~

If applying the template results in **no net changes**, the script:

- Aborts cleanly
- Leaves branch untouched
- Does not create a commit

Atomic Mode Internals
---------------------

When ``--atomic`` is enabled:

1. A temporary branch is created::

     template-sync-tmp-YYYYMMDDTHHMMSSZ

2. All sync operations happen on that branch
3. If successful:

   - Original branch is fast-forwarded using ``git merge --ff-only``

4. Temporary branch is deleted

If any error occurs:

- The temporary branch is cleaned up
- Your original branch remains unchanged

Failure Modes
-------------

Cherry-Pick Conflict
~~~~~~~~~~~~~~~~~~~~

If a conflict occurs:

- Script stops
- Instructions are printed::

    git cherry-pick --continue
    git cherry-pick --abort

In atomic mode, your main branch remains untouched.

Stash Pop Conflict
~~~~~~~~~~~~~~~~~~

If ``--auto-stash`` was used and ``stash pop`` conflicts:

- The stash remains intact
- Manual resolution instructions are printed

Recommended Workflow
--------------------

For most teams:

.. code-block:: bash

  python3 sync_template_simple.py --atomic --auto-stash

This gives:

- Clean working branch handling
- Safe atomic updates
- Automatic stash management
- Updated template tracking

Example Full Session
--------------------

.. code-block:: bash

  git checkout feature/my-work
  python3 sync_template_simple.py --atomic --auto-stash

Example output::

  Target branch (atomic): feature/my-work
  Atomic temp branch: template-sync-tmp-20260216T213000Z
  Template repo: owner/Az-RBSI
  Template branch: main
  Last applied: 1a2b3c4

  To apply:
    - 8f9d12e: Fix CAN timeout
    - 4ac8912: Update logging format

  Applied template updates successfully.
  Updated TEMPLATE_ORIGIN.txt to commit 4ac8912.

  Atomic: fast-forwarded feature/my-work to 9abcde1

Philosophy
----------

This script is intentionally:

- Minimal
- Deterministic
- Infrastructure-free
- Local-first
- Safe by default

It is designed to complement (not replace) the full GitHub Action template sync workflow.
"""






from __future__ import annotations

import argparse
import datetime as dt
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


TEMPLATE_ORIGIN = "TEMPLATE_ORIGIN.txt"


class CmdError(RuntimeError):
    pass


def run(args: List[str], *, check: bool = True) -> subprocess.CompletedProcess:
    try:
        cp = subprocess.run(args, check=False, capture_output=True, text=True)
    except FileNotFoundError as e:
        raise CmdError(f"Command not found: {args[0]}") from e
    if check and cp.returncode != 0:
        out = (cp.stdout or "").strip()
        err = (cp.stderr or "").strip()
        msg = f"Command failed ({cp.returncode}): {' '.join(args)}"
        if out:
            msg += f"\n--- stdout ---\n{out}"
        if err:
            msg += f"\n--- stderr ---\n{err}"
        raise CmdError(msg)
    return cp


def out(args: List[str]) -> str:
    return (run(args).stdout or "").strip()


def ensure_git() -> None:
    run(["git", "--version"])


def utc_timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def current_branch() -> str:
    return out(["git", "rev-parse", "--abbrev-ref", "HEAD"])


def head_sha() -> str:
    return out(["git", "rev-parse", "HEAD"])


def working_tree_is_dirty() -> bool:
    s = out(["git", "status", "--porcelain"])
    return bool(s.strip())


def has_unmerged_paths() -> bool:
    cp = run(["git", "diff", "--name-only", "--diff-filter=U"], check=False)
    return bool((cp.stdout or "").strip())


def stash_push(message: str) -> bool:
    cp = run(["git", "stash", "push", "-u", "-m", message], check=False)
    txt = (cp.stdout or "") + (cp.stderr or "")
    if "No local changes to save" in txt:
        return False
    if cp.returncode != 0:
        raise CmdError(f"git stash push failed:\n{txt.strip()}")
    return True


def stash_pop() -> None:
    cp = run(["git", "stash", "pop"], check=False)
    if cp.returncode != 0:
        txt = ((cp.stdout or "") + "\n" + (cp.stderr or "")).strip()
        raise CmdError(
            "git stash pop failed (likely conflicts).\n"
            "Your changes are still in the stash.\n"
            "Resolve manually, then run:\n"
            "  git stash list\n"
            "  git stash show -p stash@{0}\n"
            "  git stash apply stash@{0}\n\n"
            f"Details:\n{txt}"
        )


@dataclass
class OriginInfo:
    template_repo: str
    last_applied_commit: str


def parse_template_origin(path: Path) -> Optional[OriginInfo]:
    if not path.exists():
        return None
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()

    def field(prefix: str) -> Optional[str]:
        for line in lines:
            if line.startswith(prefix):
                return line[len(prefix) :].strip()
        return None

    repo = field("Template: ")
    commit = field("Template Commit: ")

    if not repo or not commit:
        return None
    if repo == "none" or commit == "none":
        return None
    return OriginInfo(template_repo=repo, last_applied_commit=commit)


def ensure_remote(name: str, url: str) -> None:
    remotes = set(out(["git", "remote"]).splitlines())
    if name in remotes:
        run(["git", "remote", "set-url", name, url])
    else:
        run(["git", "remote", "add", name, url])


def fetch(remote: str, branch: str) -> None:
    run(["git", "fetch", remote, branch])


def is_merge_commit(sha: str) -> bool:
    line = out(["git", "rev-list", "--parents", "-n", "1", sha])
    return len(line.split()) > 2


def list_commits_between(old: str, new_ref: str) -> List[str]:
    cp = run(["git", "rev-list", "--reverse", f"{old}..{new_ref}"], check=False)
    s = (cp.stdout or "").strip()
    if cp.returncode != 0 or not s:
        return []
    return [x.strip() for x in s.splitlines() if x.strip() and x.strip() != old]


def touches_workflows(sha: str) -> bool:
    files = (
        run(["git", "diff-tree", "--no-commit-id", "--name-only", "-r", sha]).stdout or ""
    ).splitlines()
    return any(f.startswith(".github/workflows/") for f in files)


def short_sha(sha: str) -> str:
    return out(["git", "rev-parse", "--short=7", sha])


def first_line(sha: str) -> str:
    msg = out(["git", "log", "-1", "--pretty=%B", sha])
    return (msg.splitlines()[0].strip() if msg else "")


def cherry_pick_no_commit(sha: str) -> None:
    cp = run(["git", "cherry-pick", "--no-commit", sha], check=False)
    if cp.returncode != 0:
        raise CmdError(
            "Cherry-pick conflict occurred.\n"
            f"Commit: {sha}\n\n"
            "Resolve conflicts, then run:\n"
            "  git cherry-pick --continue\n\n"
            "Or abort:\n"
            "  git cherry-pick --abort\n"
        )


def index_has_changes() -> bool:
    cp = run(["git", "diff", "--cached", "--name-only"], check=False)
    return bool((cp.stdout or "").strip())


def write_template_origin(path: Path, repo: str, branch: str, commit: str) -> None:
    path.write_text(
        f"Template: {repo}\n"
        f"Template Branch: {branch}\n"
        f"Template Commit: {commit}\n"
        f"Recorded At (UTC): {utc_timestamp()}\n",
        encoding="utf-8",
    )


def create_temp_branch(base_branch: str) -> str:
    # Unique enough for humans; deterministic-ish with timestamp
    name = f"template-sync-tmp-{dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    run(["git", "checkout", "-b", name, base_branch])
    return name


def delete_branch(name: str) -> None:
    run(["git", "branch", "-D", name], check=False)


def fast_forward(dst_branch: str, src_branch: str) -> None:
    run(["git", "checkout", dst_branch])
    # FF-only merge to guarantee no surprises
    run(["git", "merge", "--ff-only", src_branch])


def apply_template_sync(
    *,
    template_repo: str,
    template_branch: str,
    remote_name: str,
    last_applied_commit: str,
    skip_workflows: bool,
    squash: bool,
    no_origin_update: bool,
    dry_run: bool,
) -> int:
    """Applies template commits onto the *current checked out branch*."""
    template_url = f"https://github.com/{template_repo}.git"
    ensure_remote(remote_name, template_url)
    fetch(remote_name, template_branch)

    template_ref = f"{remote_name}/{template_branch}"
    all_commits = list_commits_between(last_applied_commit, template_ref)
    non_merge = [c for c in all_commits if not is_merge_commit(c)]

    if not non_merge:
        print("No new template commits to apply.")
        return 0

    to_apply: List[str] = []
    skipped: List[Tuple[str, str]] = []
    for sha in non_merge:
        if skip_workflows and touches_workflows(sha):
            skipped.append((short_sha(sha), first_line(sha)))
        else:
            to_apply.append(sha)

    print(f"Template repo:      {template_repo}")
    print(f"Template branch:    {template_branch}")
    print(f"Last applied:       {last_applied_commit}")
    print("\nTo apply:")
    for sha in to_apply:
        print(f"  - {short_sha(sha)}: {first_line(sha)}")

    if skipped:
        print("\nSkipped (workflow-touching):")
        for s, line in skipped:
            print(f"  - {s}: {line}")

    if not to_apply:
        print("\nNo applicable commits after filtering. Exiting.")
        return 0

    if dry_run:
        print("\n--dry-run: not applying changes.")
        return 0

    summary_lines: List[str] = []
    newest = ""
    for sha in to_apply:
        summary_lines.append(f"- {short_sha(sha)}: {first_line(sha)}")
        cherry_pick_no_commit(sha)
        newest = sha

    if not index_has_changes():
        print("\nNo changes staged after applying commits (net no-op). Exiting cleanly.")
        run(["git", "cherry-pick", "--abort"], check=False)
        return 0

    if squash:
        msg = "Template Sync Updates\n\n" + "\n".join(summary_lines) + "\n"
        run(["git", "commit", "-m", msg])
    else:
        run(["git", "commit", "-m", "Template Sync Updates"])

    if not no_origin_update:
        write_template_origin(Path(TEMPLATE_ORIGIN), template_repo, template_branch, newest)
        run(["git", "add", TEMPLATE_ORIGIN])
        run(["git", "commit", "--amend", "--no-edit"])

    print("\nApplied template updates successfully.")
    if no_origin_update:
        print(f"Did NOT update {TEMPLATE_ORIGIN} (--no-origin-update).")
    else:
        print(f"Updated {TEMPLATE_ORIGIN} to commit {newest}.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Sync template commits into current branch (simple).")
    ap.add_argument("--template-branch", default="main", help="Template branch (default: main).")
    ap.add_argument("--remote-name", default="template_repo", help="Remote name (default: template_repo).")
    ap.add_argument(
        "--skip-workflows",
        default="true",
        choices=["true", "false"],
        help="Skip commits that touch .github/workflows/ (default: true).",
    )
    ap.add_argument(
        "--squash",
        default="true",
        choices=["true", "false"],
        help="Squash applied commits into one commit (default: true).",
    )
    ap.add_argument("--dry-run", action="store_true", help="Show what would happen without changing anything.")
    ap.add_argument("--auto-stash", action="store_true", help="Stash (-u) before sync and pop afterward if dirty.")
    ap.add_argument("--no-origin-update", action="store_true", help=f"Do not update {TEMPLATE_ORIGIN}.")
    ap.add_argument(
        "--atomic",
        action="store_true",
        help="Run sync on a temporary branch and fast-forward current branch only on success.",
    )

    args = ap.parse_args()

    ensure_git()
    if has_unmerged_paths():
        raise CmdError("Repo has unmerged paths (conflicts). Resolve/abort before running sync.")

    origin = parse_template_origin(Path(TEMPLATE_ORIGIN))
    if origin is None:
        print(f"{TEMPLATE_ORIGIN} missing or not from a template; nothing to do.")
        return 0

    # Dirty tree handling
    did_stash = False
    if working_tree_is_dirty():
        if not args.auto_stash:
            raise CmdError("Working tree is dirty. Commit/stash changes, or rerun with --auto-stash.")
        if args.dry_run:
            print("--dry-run: would stash current changes.")
        else:
            did_stash = stash_push("template-sync: auto-stash")
            print(f"Auto-stash: {'created' if did_stash else 'nothing to stash'}")

    skip_workflows = args.skip_workflows.lower() == "true"
    squash = args.squash.lower() == "true"

    start_branch = current_branch()
    start_head = head_sha()
    tmp_branch: Optional[str] = None

    try:
        if not args.atomic:
            print(f"Target branch:      {start_branch}")
            return apply_template_sync(
                template_repo=origin.template_repo,
                template_branch=args.template_branch,
                remote_name=args.remote_name,
                last_applied_commit=origin.last_applied_commit,
                skip_workflows=skip_workflows,
                squash=squash,
                no_origin_update=args.no_origin_update,
                dry_run=args.dry_run,
            )

        # --atomic path
        print(f"Target branch (atomic): {start_branch}")
        if args.dry_run:
            print("--dry-run: would create a temp branch, apply changes, then ff-only merge back.")
            return apply_template_sync(
                template_repo=origin.template_repo,
                template_branch=args.template_branch,
                remote_name=args.remote_name,
                last_applied_commit=origin.last_applied_commit,
                skip_workflows=skip_workflows,
                squash=squash,
                no_origin_update=args.no_origin_update,
                dry_run=True,
            )

        tmp_branch = create_temp_branch(start_branch)
        print(f"Atomic temp branch: {tmp_branch}")

        # Apply onto temp branch
        rc = apply_template_sync(
            template_repo=origin.template_repo,
            template_branch=args.template_branch,
            remote_name=args.remote_name,
            last_applied_commit=origin.last_applied_commit,
            skip_workflows=skip_workflows,
            squash=squash,
            no_origin_update=args.no_origin_update,
            dry_run=False,
        )

        # If no changes (rc==0 but might still be no-op), fast-forward only if temp moved
        tmp_head = head_sha()
        if tmp_head == start_head:
            print("Atomic: no new commit created; leaving branch unchanged.")
            run(["git", "checkout", start_branch])
            delete_branch(tmp_branch)
            tmp_branch = None
            return rc

        # Fast-forward original branch to temp
        fast_forward(start_branch, tmp_branch)
        print(f"Atomic: fast-forwarded {start_branch} to {tmp_head}")

        # Cleanup
        delete_branch(tmp_branch)
        tmp_branch = None
        return rc

    finally:
        # Best-effort: return to original branch if we got stranded
        try:
            if current_branch() != start_branch:
                run(["git", "checkout", start_branch], check=False)
        except Exception:
            pass

        # Best-effort: delete temp branch if it still exists (and we’re back on start branch)
        if tmp_branch:
            try:
                delete_branch(tmp_branch)
            except Exception:
                pass

        # Restore stashed work (if any)
        if did_stash:
            try:
                stash_pop()
                print("Auto-stash popped successfully.")
            except CmdError as e:
                print(f"\nWARNING: {e}\n", file=sys.stderr)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except CmdError as e:
        print(f"\nERROR: {e}\n", file=sys.stderr)
        raise SystemExit(2)
    