# Batched snapshot of RSU-detected objects. Array wrapper is our own
# assumption (not confirmed with the teammate) — a single-object-per-message
# stream is also plausible; revisit once their publisher exists.

std_msgs/Header header
DetectedObject[] objects
