"""Write the live ROS 2 graph (nodes, topics, services, actions) as Graphviz DOT.

    bash scripts/ros.sh python3 scripts/graph_dot.py > docs/ros_graph.dot
    dot -Tpng docs/ros_graph.dot -o docs/ros_graph.png
"""
import time

import rclpy
from rclpy.node import Node

SKIP_NODES = {"graph_dot", "rqt_gui_py_node", "_ros2cli_daemon"}
SKIP_TOPICS = {"/parameter_events", "/rosout"}

rclpy.init()
me = Node("graph_dot")
time.sleep(2.0)  # let discovery settle

lines = ["digraph ros {", "  rankdir=LR; node [fontname=Helvetica, fontsize=11];"]
topics = set()
for name, ns in me.get_node_names_and_namespaces():
    full = (ns.rstrip("/") + "/" + name) if ns != "/" else "/" + name
    if name in SKIP_NODES or name.startswith("_") or "rqt" in name or "transform_listener" in name:
        continue
    lines.append(f'  "{full}" [shape=ellipse, style=filled, fillcolor="#dbe9f6"];')
    for t, _ in me.get_publisher_names_and_types_by_node(name, ns):
        if t not in SKIP_TOPICS and not t.startswith("/Ros2Supervisor"):
            topics.add(t)
            lines.append(f'  "{full}" -> "{t}";')
    for t, _ in me.get_subscriber_names_and_types_by_node(name, ns):
        if t not in SKIP_TOPICS and not t.startswith("/Ros2Supervisor"):
            topics.add(t)
            lines.append(f'  "{t}" -> "{full}";')
    for s, _ in me.get_service_names_and_types_by_node(name, ns):
        if s.startswith("/quad/") and "/_action/" not in s:
            lines.append(f'  "{s}" [shape=octagon, style=filled, fillcolor="#fde9c9"];')
            lines.append(f'  "{s}" -> "{full}" [style=dashed, label="service"];')
for t in topics:
    if "/_action/" in t:
        continue
    lines.append(f'  "{t}" [shape=box, style=filled, fillcolor="#e8f3e3"];')
lines.append('  "/quad/go_to" [shape=doubleoctagon, style=filled, fillcolor="#f6d6d6"];')
lines.append('  "/quad/go_to" -> "/mission" [style=dashed, label="action"];')
lines.append("}")
print("\n".join(l for l in lines if "/_action/" not in l))
rclpy.shutdown()
