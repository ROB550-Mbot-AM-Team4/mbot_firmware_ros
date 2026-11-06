"""Small helpers shared by merge_for_compare.py and analyze_step.py (rosbag2_py, mcap or sqlite3)."""
import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def open_reader(path):
    """Return (reader, {topic: type_string}) for a bag directory or a single .mcap/.db3 file."""
    storage_id = ""
    if os.path.isfile(path):
        storage_id = "mcap" if path.endswith(".mcap") else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def iter_messages(path, topics=None, deserialize=True):
    """Yield (topic, t_ns, msg_or_bytes) in time order. `topics` filters to that set."""
    reader, types = open_reader(path)
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    classes = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if deserialize:
            if topic not in classes:
                classes[topic] = get_message(types[topic])
            data = deserialize_message(data, classes[topic])
        yield topic, t_ns, data


def find_step_onset_ns(path, threshold=0.05):
    """Receive time of the first non-zero body-velocity command in the bag.

    Prefers /step_test/cmd (TwistStamped); falls back to /cmd_vel (Twist).
    Returns None when no command exceeds `threshold`.
    """
    reader, types = open_reader(path)
    topic = "/step_test/cmd" if "/step_test/cmd" in types else "/cmd_vel"
    if topic not in types:
        return None
    for tp, t_ns, msg in iter_messages(path, [topic]):
        tw = msg.twist if tp == "/step_test/cmd" else msg
        if abs(tw.linear.x) > threshold or abs(tw.angular.z) > threshold:
            return t_ns
    return None
