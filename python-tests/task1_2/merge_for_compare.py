#!/usr/bin/env python3
"""
Merge several step-test bags into ONE bag so Foxglove can overlay them on a single plot.

Each input bag's topics are re-published under a prefix (e.g. /ff/odom, /tuned/odom) and its
timestamps are shifted so that the first non-zero command happens at the same instant in every
run.  Plot with timestampMethod = "receive time" in Foxglove (the default); header.stamp is left
untouched.

Usage:
  python3 merge_for_compare.py ~/bags/task1_2/ff_2026... ~/bags/task1_2/tuned_2026... \
          --labels ff tuned -o ~/bags/task1_2/compare_ff_vs_tuned

Then in Foxglove plot, e.g.
  /ff/step_test/cmd.twist.linear.x   /ff/odom.twist.twist.linear.x   /tuned/odom.twist.twist.linear.x
"""
import argparse
import os
import sys

import rosbag2_py
from bag_tools import open_reader, find_step_onset_ns

DEFAULT_TOPICS = ["/step_test/cmd", "/step_test/cmd_wheel", "/step_test/meas",
                  "/cmd_vel", "/odom", "/motor_vel"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+", help="input bag directories (or .mcap files)")
    ap.add_argument("--labels", nargs="+", help="prefix per bag (default: ff, tuned, run3, ...)")
    ap.add_argument("-o", "--output", required=True, help="output bag directory (must not exist)")
    ap.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS, help="topics to copy")
    ap.add_argument("--pre", type=float, default=1.5, help="seconds kept before the step onset")
    ap.add_argument("--no-align", action="store_true", help="keep original timestamps")
    args = ap.parse_args()

    labels = args.labels or (["ff", "tuned"] + [f"run{i}" for i in range(3, len(args.bags) + 1)])[:len(args.bags)]
    if len(labels) != len(args.bags):
        sys.exit("number of --labels must match number of bags")
    if os.path.exists(args.output):
        sys.exit(f"output already exists: {args.output}")

    # Common time base: the onset of the first bag (so Foxglove still shows a sane absolute time).
    onsets = []
    for b in args.bags:
        on = None if args.no_align else find_step_onset_ns(b)
        if on is None and not args.no_align:
            print(f"warning: no step onset found in {b}; keeping its original timestamps")
        onsets.append(on)
    base_ns = next((o for o in onsets if o is not None), 0)

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.output, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))

    created = set()
    next_id = 0
    pre_ns = int(args.pre * 1e9)
    total = 0
    for bag, label, onset in zip(args.bags, labels, onsets):
        reader, types = open_reader(bag)
        wanted = [t for t in args.topics if t in types]
        missing = [t for t in args.topics if t not in types]
        if missing:
            print(f"[{label}] topics not in bag (skipped): {missing}")
        reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
        shift = 0 if onset is None else base_ns - onset
        n = 0
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            if onset is not None and t_ns < onset - pre_ns:
                continue
            new_topic = f"/{label}{topic}"
            if new_topic not in created:
                writer.create_topic(rosbag2_py.TopicMetadata(
                    id=next_id, name=new_topic, type=types[topic], serialization_format="cdr"))
                next_id += 1
                created.add(new_topic)
            writer.write(new_topic, data, t_ns + shift)
            n += 1
        print(f"[{label}] {n} messages from {bag} (shift {shift / 1e9:+.3f} s)")
        total += n
    del writer
    print(f"\nwrote {total} messages to {args.output}")
    print("step onset is at t = %.3f s (bag time) for every run" % (base_ns / 1e9))


if __name__ == "__main__":
    main()
