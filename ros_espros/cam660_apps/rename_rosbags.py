import sys
import os

def rename_rosbag(path):
    # Check if path (relative to the location of this file) exists
    rosbag_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), path)
    if not os.path.isdir(rosbag_dir):
        print(f"Path {rosbag_dir} is not valid")
        return

    # Obtain all files in dir
    files = os.listdir(rosbag_dir)
    files = [file for file in files if file.endswith(".bag")]
    rosbags_rename = [file for file in files if file.startswith("_2026")]
    rosbags_rename.sort() # Sort files by ascending order, first recorded first
    # Obtain bags already numbered
    numbered = [
        int(f.split(".")[0])
        for f in files
        if f[0].isdigit()
    ]

    next_ = max(numbered, default=0)+1

    # Change file names
    for rosbag in rosbags_rename:
        fname = os.path.join(rosbag_dir, rosbag)
        dest_fname = os.path.join(rosbag_dir, f"{(next_):03d}.bag")
        if os.path.exists(dest_fname):
            print(f"Target file name already exists: {dest_fname}")
            return
        os.rename(fname, dest_fname)
        next_ += 1

if __name__ == "__main__":
    path = sys.argv[1]
    rename_rosbag(path)