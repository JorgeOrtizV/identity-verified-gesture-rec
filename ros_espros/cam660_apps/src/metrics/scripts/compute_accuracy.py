#!/usr/bin/env python
"""
Offline accuracy computation.

Reads a ground-truth JSON and the per-bag JSON results produced by metrics_node to compute per-bag and aggregate metrics for authorization,
system behaviour, and gesture recognition.

Ground truth format (ground_truth.json):
{
  "single_bag_001": {
    "authorized_agents": [1],
    "expected_gestures": {                        <- optional
      "1": {"circle": 2, "arm_up_down": 1}
    }
  },
  "multi_bag_001": {
    "authorized_agents": [1, 2],
    "expected_gestures": {
      "1": {"circle": 1},
      "2": {"arm_up_down": 2}
    }
  }
}

Bag name prefixes:
  "single_"  -> single-agent scenario
  "multi_"   -> multi-agent scenario

Usage:
    python compute_accuracy.py <ground_truth.json>
"""
import argparse
import glob
import json
import os
import sys

SEP_MAJOR = "=" * 72
SEP_MINOR = "-" * 52


# Utilities

def _div(num, den):
    return num / float(den) if den > 0 else 0.0

def _f1(prec, rec):
    return 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0.0

def _pct(num, den):
    return _div(num, den) * 100


# Authorization metrics

def auth_metrics(data, gt_entry):
    gt_auth   = set(gt_entry.get("authorized_agents", []))
    pred_auth = set(data.get("authorized_agent_ids", []))
    all_ids   = set(data.get("all_agent_ids", [])) | gt_auth | pred_auth

    tp = len(pred_auth & gt_auth)
    fp = len(pred_auth - gt_auth)
    fn = len(gt_auth - pred_auth)
    tn = len(all_ids - pred_auth - gt_auth)

    prec = _div(tp, tp + fp)
    rec  = _div(tp, tp + fn)
    acc  = _div(tp + tn, tp + tn + fp + fn)
    return {"tp": tp, "fp": fp, "fn": fn, "tn": tn,
            "precision": prec, "recall": rec, "f1": _f1(prec, rec), "accuracy": acc}


# Gesture accuracy

def gesture_agent_accuracy(detected_counts, expected_counts):
    """
    Per-agent TP/FP/FN comparing detected gesture type counts vs GT.
      TP = correctly detected (min of expected and detected per type)
      FN = missed detections  (expected - detected, when expected > detected)
      FP = extra detections   (detected - expected, when detected > expected)
    """
    all_types = set(detected_counts) | set(expected_counts)
    tp = fp = fn = 0
    per_type = {}
    for gtype in sorted(all_types):
        exp = expected_counts.get(gtype, 0)
        det = detected_counts.get(gtype, 0)
        tp += min(exp, det)
        fp += max(0, det - exp)
        fn += max(0, exp - det)
        per_type[gtype] = {"expected": exp, "detected": det}

    prec = _div(tp, tp + fp)
    rec  = _div(tp, tp + fn)
    return {"tp": tp, "fp": fp, "fn": fn,
            "precision": prec, "recall": rec, "f1": _f1(prec, rec),
            "per_type": per_type}


# Per-bag report

def report_bag(bag_name, data, gt_entry):
    """Print the per-bag section. Returns a metrics dict for later aggregation."""
    scene_dur         = data.get("scene_duration", 0.0)
    total_auth_events = data.get("total_auth_events", 0)
    total_gestures    = data.get("total_gestures", 0)
    auth_gestures     = data.get("authorized_gestures", 0)
    lat               = data.get("gesture_latency", {})
    per_agent         = data.get("per_agent", {})

    am = auth_metrics(data, gt_entry)

    total_auth_time = sum(v.get("total_time_authorized", 0.0) for v in per_agent.values())
    scene_coverage  = _div(total_auth_time, scene_dur)
    auth_stability  = _div(total_auth_events, scene_dur)  # auth events per second
    false_det       = total_gestures - auth_gestures

    print(SEP_MAJOR)
    print("BAG: %-40s [scene: %.1fs]" % (bag_name, scene_dur))
    print(SEP_MAJOR)

    # Authorization
    print("\n[Authorization]")
    print("  Accuracy=%.3f  Precision=%.3f  Recall=%.3f  F1=%.3f"
          % (am["accuracy"], am["precision"], am["recall"], am["f1"]))
    print("  TP=%-3d  FP=%-3d  FN=%-3d  TN=%-3d"
          % (am["tp"], am["fp"], am["fn"], am["tn"]))

    # System
    print("\n[System]")
    print("  Auth events : %d" % total_auth_events)
    print("  Stability   : %.2f evt/s  (lower = more stable)" % auth_stability)
    print("  Scene auth coverage : %.1f%%  (time any agent was authorized)" % (scene_coverage * 100))

    # Per-agent
    print("\n[Per-Agent]")
    for aid_str, ag in sorted(per_agent.items()):
        presence  = ag.get("time_in_scene", 0.0)
        auth_t    = ag.get("total_time_authorized", 0.0)
        n_auth    = ag.get("times_authorized", 0)
        print("  Agent %-3s  presence=%5.1fs  auth=%5.1fs (%4.1f%%)  "
              "auth_events=%d/%d (%4.1f%%)"
              % (aid_str, presence, auth_t, _pct(auth_t, presence),
                 n_auth, total_auth_events, _pct(n_auth, total_auth_events)))
        print("             avg_auth_conf=%.3f  avg_scene_conf=%.3f"
              % (ag.get("avg_auth_confidence", 0.0), ag.get("avg_scene_confidence", 0.0)))

    # Gestures
    print("\n[Gestures]")
    print("  Detected      : %d total" % total_gestures)
    print("  Authorized    : %d/%d (%.1f%%)"
          % (auth_gestures, total_gestures, _pct(auth_gestures, total_gestures)))
    print("  False detected: %d/%d (%.1f%%)  (gesture with no authorized agent)"
          % (false_det, total_gestures, _pct(false_det, total_gestures)))

    lat_mean = lat.get("mean", 0.0)
    if lat_mean > 0:
        print("  Recognition latency: mean=%.3fs  min=%.3fs  max=%.3fs"
              % (lat_mean, lat.get("min", 0.0), lat.get("max", 0.0)))

    # Gesture accuracy vs ground truth (optional — only if GT provides it)
    gt_gestures = gt_entry.get("expected_gestures", {})
    gest_agg = {"tp": 0, "fp": 0, "fn": 0}
    if gt_gestures:
        print("  Gesture accuracy (vs ground truth):")
        for aid_str_gt, expected in sorted(gt_gestures.items()):
            detected = per_agent.get(str(aid_str_gt), {}).get("gesture_counts", {})
            ga = gesture_agent_accuracy(detected, expected)
            gest_agg["tp"] += ga["tp"]
            gest_agg["fp"] += ga["fp"]
            gest_agg["fn"] += ga["fn"]
            type_str = "  ".join(
                "%s=%d/%d" % (t, v["detected"], v["expected"])
                for t, v in ga["per_type"].items()
            )
            print("    Agent %-3s  %-40s  Prec=%.3f  Recall=%.3f  F1=%.3f"
                  % (str(aid_str_gt), type_str,
                     ga["precision"], ga["recall"], ga["f1"]))

    print()

    return {
        "am":               am,
        "scene_dur":        scene_dur,
        "total_auth_events": total_auth_events,
        "total_auth_time":  total_auth_time,
        "total_gestures":   total_gestures,
        "auth_gestures":    auth_gestures,
        "lat_mean":         lat.get("mean", 0.0),
        "lat_min":          lat.get("min", 0.0),
        "lat_max":          lat.get("max", 0.0),
        "has_lat":          lat.get("mean", 0.0) > 0,
        "gest_tp":          gest_agg["tp"],
        "gest_fp":          gest_agg["fp"],
        "gest_fn":          gest_agg["fn"],
        "has_gest_gt":      bool(gt_gestures),
    }


# Aggregate report

def report_aggregate(label, bag_metrics):
    if not bag_metrics:
        return

    # Authorization — micro-average (sum counts, recompute rates)
    tp = sum(m["am"]["tp"] for m in bag_metrics)
    fp = sum(m["am"]["fp"] for m in bag_metrics)
    fn = sum(m["am"]["fn"] for m in bag_metrics)
    tn = sum(m["am"]["tn"] for m in bag_metrics)
    prec = _div(tp, tp + fp)
    rec  = _div(tp, tp + fn)
    acc  = _div(tp + tn, tp + tn + fp + fn)

    # System
    total_scene     = sum(m["scene_dur"] for m in bag_metrics)
    total_auth_ev   = sum(m["total_auth_events"] for m in bag_metrics)
    total_auth_time = sum(m["total_auth_time"] for m in bag_metrics)
    stability       = _div(total_auth_ev, total_scene)
    coverage        = _div(total_auth_time, total_scene)

    # Gestures
    total_gest  = sum(m["total_gestures"] for m in bag_metrics)
    auth_gest   = sum(m["auth_gestures"] for m in bag_metrics)
    false_det   = total_gest - auth_gest

    # Latency — average of per-bag means, global min/max
    lat_bags = [m for m in bag_metrics if m["has_lat"]]
    lat_mean = _div(sum(m["lat_mean"] for m in lat_bags), len(lat_bags)) if lat_bags else 0.0
    lat_min  = min((m["lat_min"] for m in lat_bags), default=0.0)
    lat_max  = max((m["lat_max"] for m in lat_bags), default=0.0)

    # Gesture accuracy — micro-average across bags that have GT
    gest_tp = sum(m["gest_tp"] for m in bag_metrics)
    gest_fp = sum(m["gest_fp"] for m in bag_metrics)
    gest_fn = sum(m["gest_fn"] for m in bag_metrics)
    gest_prec = _div(gest_tp, gest_tp + gest_fp)
    gest_rec  = _div(gest_tp, gest_tp + gest_fn)
    has_gest_gt = any(m["has_gest_gt"] for m in bag_metrics)

    n = len(bag_metrics)
    print(SEP_MAJOR)
    print("AGGREGATE: %-30s (%d bag%s  |  total scene: %.1fs)"
          % (label, n, "s" if n != 1 else "", total_scene))
    print(SEP_MAJOR)

    print("\n[Authorization]")
    print("  Accuracy=%.3f  Precision=%.3f  Recall=%.3f  F1=%.3f"
          % (acc, prec, rec, _f1(prec, rec)))
    print("  TP=%-3d  FP=%-3d  FN=%-3d  TN=%-3d" % (tp, fp, fn, tn))

    print("\n[System]")
    print("  Auth events : %d" % total_auth_ev)
    print("  Stability   : %.2f evt/s" % stability)
    print("  Scene auth coverage : %.1f%%" % (coverage * 100))

    print("\n[Gestures]")
    print("  Detected      : %d total" % total_gest)
    print("  Authorized    : %d/%d (%.1f%%)"
          % (auth_gest, total_gest, _pct(auth_gest, total_gest)))
    print("  False detected: %d/%d (%.1f%%)"
          % (false_det, total_gest, _pct(false_det, total_gest)))
    if lat_mean > 0:
        print("  Recognition latency: mean=%.3fs  min=%.3fs  max=%.3fs"
              % (lat_mean, lat_min, lat_max))
    if has_gest_gt and (gest_tp + gest_fp + gest_fn) > 0:
        print("  Gesture accuracy:  Prec=%.3f  Recall=%.3f  F1=%.3f"
              % (gest_prec, gest_rec, _f1(gest_prec, gest_rec)))

    print()


# Entry point

def compute_accuracy(gt_path):
    with open(gt_path) as f:
        ground_truth = json.load(f)

    if not ground_truth:
        print("Ground truth file is empty.")
        return

    pkg_dir     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    results_dir = os.path.join(pkg_dir, "results")

    if not os.path.isdir(results_dir):
        print("Results directory not found: %s" % results_dir)
        return

    result_files = sorted(glob.glob(os.path.join(results_dir, "*.json")))
    if not result_files:
        print("No result files found in %s" % results_dir)
        return

    all_metrics    = []
    single_metrics = []
    multi_metrics  = []

    for rfile in result_files:
        bag_name = os.path.splitext(os.path.basename(rfile))[0]
        if bag_name not in ground_truth:
            continue

        with open(rfile) as f:
            data = json.load(f)

        m = report_bag(bag_name, data, ground_truth[bag_name])
        all_metrics.append(m)
        if bag_name.startswith("single_"):
            single_metrics.append(m)
        elif bag_name.startswith("multi_"):
            multi_metrics.append(m)

    if not all_metrics:
        print("No matching result files found for the provided ground truth.")
        return

    if single_metrics:
        report_aggregate("single_*", single_metrics)
    if multi_metrics:
        report_aggregate("multi_*", multi_metrics)
    report_aggregate("TOTAL", all_metrics)


class _Tee(object):
    """Writes to both stdout and a file simultaneously."""
    def __init__(self, f):
        self._file = f
        self._stdout = sys.stdout
    def write(self, data):
        self._stdout.write(data)
        self._file.write(data)
    def flush(self):
        self._stdout.flush()
        self._file.flush()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compute system accuracy from metrics results."
    )
    parser.add_argument(
        "ground_truth",
        help="Path to ground truth JSON file",
    )
    parser.add_argument(
        "-o", "--output",
        help="Optional path to save the report as a text file",
        default=None,
    )
    args = parser.parse_args()

    if not os.path.isfile(args.ground_truth):
        print("File not found: %s" % args.ground_truth)
        sys.exit(1)

    if args.output:
        with open(args.output, "w") as out_f:
            sys.stdout = _Tee(out_f)
            compute_accuracy(args.ground_truth)
            sys.stdout = sys.stdout._stdout
        print("Report saved to %s" % args.output)
    else:
        compute_accuracy(args.ground_truth)
