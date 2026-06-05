#!/usr/bin/env python3
"""
Merge /odom from the source bag into a SLAM output bag.
Usage:
  python3 merge_bags.py <source_bag_dir> <slam_bag_dir> <output_bag_dir>
"""
import sqlite3
import shutil
import os
import sys
import yaml

def find_db3(bag_dir):
    for f in sorted(os.listdir(bag_dir)):
        if f.endswith('.db3'):
            return os.path.join(bag_dir, f)
    raise FileNotFoundError(f"No .db3 file found in {bag_dir}")

def merge(source_dir, slam_dir, output_dir, ref_topic="/odom"):
    source_db = find_db3(source_dir)
    slam_db   = find_db3(slam_dir)

    os.makedirs(output_dir, exist_ok=True)
    output_db = os.path.join(output_dir, os.path.basename(slam_db).replace(
        os.path.splitext(os.path.basename(slam_db))[0],
        os.path.basename(output_dir) + "_0"
    ))
    # Simpler name
    output_db = os.path.join(output_dir, os.path.basename(output_dir) + "_0.db3")

    shutil.copy2(slam_db, output_db)
    print(f"Copied slam bag → {output_db}")

    conn_out = sqlite3.connect(output_db)
    cur_out  = conn_out.cursor()

    cur_out.execute("SELECT MAX(id) FROM topics")
    max_topic_id = cur_out.fetchone()[0] or 0
    cur_out.execute("SELECT MAX(id) FROM messages")
    max_msg_id = cur_out.fetchone()[0] or 0

    conn_src = sqlite3.connect(source_db)
    cur_src  = conn_src.cursor()

    cur_src.execute(
        "SELECT id, name, type, serialization_format, offered_qos_profiles "
        "FROM topics WHERE name=?", (ref_topic,)
    )
    row = cur_src.fetchone()
    if row is None:
        print(f"ERROR: topic '{ref_topic}' not found in {source_db}")
        sys.exit(1)

    src_topic_id, name, ttype, ser_fmt, qos = row
    new_topic_id = max_topic_id + 1
    cur_out.execute(
        "INSERT INTO topics (id, name, type, serialization_format, offered_qos_profiles) "
        "VALUES (?,?,?,?,?)",
        (new_topic_id, name, ttype, ser_fmt, qos)
    )

    cur_src.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (src_topic_id,)
    )
    msgs = cur_src.fetchall()
    cur_out.executemany(
        "INSERT INTO messages (id, topic_id, timestamp, data) VALUES (?,?,?,?)",
        [(max_msg_id + i + 1, new_topic_id, ts, data) for i, (ts, data) in enumerate(msgs)]
    )
    conn_out.commit()
    conn_out.close()
    conn_src.close()
    print(f"Inserted {len(msgs)} messages of '{ref_topic}'")

    # Build metadata.yaml
    slam_meta_path = os.path.join(slam_dir, "metadata.yaml")
    with open(slam_meta_path) as f:
        meta = yaml.safe_load(f)

    # Read /odom topic type from source metadata
    src_meta_path = os.path.join(source_dir, "metadata.yaml")
    with open(src_meta_path) as f:
        src_meta = yaml.safe_load(f)

    odom_topic_meta = None
    for t in src_meta["rosbag2_bagfile_information"]["topics_with_message_count"]:
        if t["topic_metadata"]["name"] == ref_topic:
            odom_topic_meta = t
            break

    if odom_topic_meta:
        odom_entry = {
            "topic_metadata": odom_topic_meta["topic_metadata"],
            "message_count": len(msgs)
        }
        meta["rosbag2_bagfile_information"]["topics_with_message_count"].append(odom_entry)
        meta["rosbag2_bagfile_information"]["message_count"] += len(msgs)
        meta["rosbag2_bagfile_information"]["relative_file_paths"] = [
            os.path.basename(output_dir) + "_0.db3"
        ]

    out_meta_path = os.path.join(output_dir, "metadata.yaml")
    with open(out_meta_path, "w") as f:
        yaml.dump(meta, f, default_flow_style=False)
    print(f"Wrote {out_meta_path}")
    print("Done.")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: merge_bags.py <source_bag_dir> <slam_bag_dir> <output_bag_dir>")
        sys.exit(1)
    merge(sys.argv[1], sys.argv[2], sys.argv[3])
