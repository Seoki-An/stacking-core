#!/usr/bin/env python3
"""Export legacy diffsim motion results from a stacking-planner plan.

The output is a deliberately small, dependency-free token format consumed by
the stacking-core C++ parity test. Run this script inside stacking-planner's
direnv so NumPy and the pickle's application modules are available.
"""

from __future__ import annotations

import argparse
import hashlib
import pickle
from pathlib import Path
from typing import Any, Iterable

import numpy as np


STAGE_ROLES = {
    4: ("free", "attached", "attached", "free"),
    8: (
        "free",
        "attached",
        "attached",
        "free",
        "free",
        "attached",
        "attached",
        "free",
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path, help="stacking-planner plan directory")
    parser.add_argument("output", type=Path, help="fixture file to write")
    parser.add_argument(
        "--case",
        type=int,
        action="append",
        dest="cases",
        required=True,
        help="action/motion index to export; repeat for multiple cases",
    )
    return parser.parse_args()


def load_pickle(path: Path) -> Any:
    with path.open("rb") as stream:
        return pickle.load(stream)


def archive_digest(paths: Iterable[Path]) -> str:
    digest = hashlib.sha256()
    for path in paths:
        digest.update(path.name.encode("utf-8"))
        digest.update(path.read_bytes())
    return digest.hexdigest()


def numbers(values: Any) -> str:
    array = np.asarray(values, dtype=np.float64)
    return " ".join(format(float(value), ".17g") for value in array.reshape(-1))


def checked_path(value: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have finite shape {shape}, got {array.shape}")
    return array


def main() -> None:
    args = parse_args()
    source_paths = [
        args.plan / "planning_params.pkl",
        args.plan / "action_sequence.pkl",
        args.plan / "motion_result_sequence.pkl",
    ]
    params, actions, motions = (load_pickle(path) for path in source_paths)

    q_home_raw = np.asarray(params["q_home"], dtype=np.float64)
    q_home = checked_path(params["q_home"], (q_home_raw.size,), "q_home")
    if q_home.size == 0:
        raise ValueError("q_home must not be empty")
    lines = [
        "stacking_planner_plan_state 1",
        f"source {args.plan.as_posix()}",
        f"archive_sha256 {archive_digest(source_paths)}",
        f"dof {q_home.size}",
        f"q_home {numbers(q_home)}",
        f"case_count {len(args.cases)}",
    ]

    for case_index in args.cases:
        action = actions[case_index]
        motion = motions[case_index]
        if not isinstance(action, dict) or not isinstance(motion, dict):
            raise ValueError(f"case {case_index} is not a saved planner result")
        if not motion.get("is_feasible", False):
            raise ValueError(f"case {case_index} is not feasible")

        q_paths = motion.get("q_path_sequence") or []
        target_paths = motion.get("target_path_sequence") or []
        roles = STAGE_ROLES.get(len(q_paths))
        if roles is None:
            raise ValueError(
                f"case {case_index} has unsupported stage count {len(q_paths)}"
            )
        if len(target_paths) != len(q_paths):
            raise ValueError(f"case {case_index} path sequence sizes differ")

        mode = "direct" if len(q_paths) == 4 else "regrasp"
        scores = motion.get("scores") or []
        score = float(scores[0]) if scores else 0.0
        lines.append(
            f"case {case_index} {mode} {int(action['stone_id'])} "
            f"{format(score, '.17g')} {len(q_paths)}"
        )

        for stage_index, (role, q_path, target_path) in enumerate(
            zip(roles, q_paths, target_paths)
        ):
            if len(q_path) != len(target_path) or not q_path:
                raise ValueError(
                    f"case {case_index} stage {stage_index} has invalid sample counts"
                )
            lines.append(f"stage {stage_index} {role} {len(q_path)}")
            for sample_index, (q, target) in enumerate(zip(q_path, target_path)):
                q = checked_path(
                    q,
                    (q_home.size,),
                    f"case {case_index} stage {stage_index} q[{sample_index}]",
                )
                target = checked_path(
                    target,
                    (4, 4),
                    f"case {case_index} stage {stage_index} target[{sample_index}]",
                )
                lines.append(f"sample {numbers(q)} {numbers(target)}")
            lines.append("end_stage")
        lines.append("end_case")
    lines.append("end_fixture")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
