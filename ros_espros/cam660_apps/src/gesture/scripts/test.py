#!/usr/bin/env python3

import sys
import os

import numpy as np
from scipy.spatial.distance import cdist

class TestGesture:
    def __init__(self, test_dir):
        self.templates = {}
        self.template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "templates")
        self.test_dir = os.path.join(self.template_dir, test_dir)

        # Calibration
        self.dtw_band_ratio = 0.5
        self.dtw_threshold = 0.75


    def test(self):
        # Check if test dictionary exists
        if not os.path.isdir(self.test_dir):
            print(f"Given path {self.test_dir} is not a valid path")
            return

        # Load templates
        self.load_templates()
        if self.templates:
            for name, tmpls in self.templates.items():
                print(f"Loaded {len(tmpls)} templates for '{name}'")
        else:
            print("No templates loaded - recognition will not work")


        # For each template in test_dir classify against all stored templates
        print() # Just formatting output
        for fname in os.listdir(self.test_dir):
            if fname.endswith(".npy"):
                print(f"Evaluating {fname}")
                gesture = np.load(os.path.join(self.test_dir, fname))
                print(self.classify(gesture))
                


    def load_templates(self):
        if not os.path.isdir(self.template_dir):
            print(f"Template directory not found: {self.template_dir}")
            return

        for gesture_name in sorted(os.listdir(self.template_dir)):
            if gesture_name == "ood":
                continue
            gesture_path = os.path.join(self.template_dir, gesture_name)
            if not os.path.isdir(gesture_path):
                continue

            templates = []
            for fname in sorted(os.listdir(gesture_path)):
                if fname.endswith(".npy"):
                    tmpl = np.load(os.path.join(gesture_path, fname))
                    templates.append(tmpl)

            if templates:
                self.templates[gesture_name] = templates

    # DTW classification
    def classify(self, gesture):
        best_name = None
        best_dist = float('inf')
        results = []
        output = ""
        band = max(1, int(self.dtw_band_ratio * max(len(gesture), 1)))

        for name, templates in self.templates.items():
            if name == "ood":
                continue
            dists = []
            for template in templates:
                # Skip templates where length mismatch exceeds band (DTW would return inf)
                if abs(len(gesture) - len(template)) > band:
                    continue
                d = self.dtw_distance(gesture, template)
                dists.append(d)
            if not dists:
                continue
            avg_dist = np.mean(dists)
            min_dist = np.min(dists)
            results.append((name, min_dist, avg_dist))

            if min_dist < best_dist:
                best_dist = min_dist
                best_name = name

        # Log all distances for debugging
        for name, min_d, avg_d in results:
            output+=f"  DTW '{name}': min={min_d:.2f}, avg={avg_d:.2f}\n"

        if best_name is not None and best_dist < self.dtw_threshold:
            output+=f"RECOGNIZED: '{best_name}' (distance={best_dist:.2f})\n"
        else:
            output+=f"No match (best='{best_name}', dist={best_dist:.2f}, thresh={self.dtw_threshold})\n"

        return output


    def dtw_distance(self, s1, s2):
        # Multivariate DTW with Sakoe-Chiba band. Distance is normalized by path length so it's comparable across gestures of different durations.
        n, m = len(s1), len(s2)
        band = max(1, int(self.dtw_band_ratio * max(n, m)))

        # Precompute all pairwise Euclidean distances in one C-level call
        D = cdist(s1, s2, 'euclidean')

        cost = np.full((n + 1, m + 1), np.inf)
        cost[0, 0] = 0.0

        for i in range(1, n + 1):
            j_start = max(1, i - band)
            j_end = min(m, i + band)
            for j in range(j_start, j_end + 1):
                cost[i, j] = D[i - 1, j - 1] + min(cost[i-1, j], cost[i, j-1], cost[i-1, j-1])

        return cost[n, m] / (n + m)





if __name__ == "__main__":
    test_dir = sys.argv[1]
    test = TestGesture(test_dir)
    test.test()

