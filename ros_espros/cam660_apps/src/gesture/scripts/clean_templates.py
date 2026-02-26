#!/usr/bin/env python3
"""
Remove gesture templates with fewer than MIN_SAMPLES frames.
Accidental micro-movements before/after a gesture produce very short recordings
that degrade DTW matching. This script finds and removes them.

Usage:
    python3 clean_templates.py [--threshold 30] [--dry-run] [--templates-dir <path>]
"""

import argparse
import os
import glob
import numpy as np


def find_short_templates(templates_dir, threshold):
    files = sorted(glob.glob(os.path.join(templates_dir, "**/*.npy"), recursive=True))
    if not files:
        print(f"No .npy files found in '{templates_dir}'")
        return []

    short = []
    for f in files:
        n_samples = np.load(f).shape[0]
        if n_samples < threshold:
            short.append((f, n_samples))

    return short


def print_summary(short, threshold, total):
    print(f"\nTotal templates scanned : {total}")
    print(f"Threshold               : {threshold} samples")
    print(f"Templates to remove     : {len(short)}\n")

    if short:
        by_class = {}
        for f, n in short:
            cls = os.path.basename(os.path.dirname(f))
            by_class.setdefault(cls, []).append((f, n))

        for cls, items in sorted(by_class.items()):
            print(f"  [{cls}]")
            for f, n in items:
                print(f"    {n:3d} samples  {os.path.basename(f)}")
        print()


def main():
    parser = argparse.ArgumentParser(description="Clean short gesture templates")
    parser.add_argument(
        "--threshold", type=int, default=30,
        help="Remove templates with fewer than this many samples (default: 30)"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Show what would be deleted without actually deleting"
    )
    parser.add_argument(
        "--templates-dir",
        default=os.path.join(os.path.dirname(__file__), "..", "templates"),
        help="Path to the templates directory"
    )
    args = parser.parse_args()

    templates_dir = os.path.realpath(args.templates_dir)
    if not os.path.isdir(templates_dir):
        print(f"Error: templates directory not found: '{templates_dir}'")
        return

    all_files = glob.glob(os.path.join(templates_dir, "**/*.npy"), recursive=True)
    short = find_short_templates(templates_dir, args.threshold)

    print_summary(short, args.threshold, len(all_files))

    if not short:
        print("Nothing to remove.")
        return

    if args.dry_run:
        print("Dry-run mode: no files were deleted.")
        return

    answer = input(f"Delete {len(short)} file(s)? [y/N] ").strip().lower()
    if answer != "y":
        print("Aborted.")
        return

    removed = 0
    for f, n in short:
        os.remove(f)
        print(f"  Deleted  {f}  ({n} samples)")
        removed += 1

    print(f"\nDone. {removed} template(s) removed.")


if __name__ == "__main__":
    main()
