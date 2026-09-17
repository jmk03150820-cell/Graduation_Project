# GENERATED from ../../../schema/avva_phase1.idl by generate_ros2_msgs.py. DO NOT EDIT.
CommonHeader header
uint64 state_tick_id
NativeFrameRef native_frame_ref
ControlSetMeta control_set_meta
string<=63[<=8] expected_ego_ids
string<=63[<=1200] expected_traffic_actor_ids
ActorState[<=1200] actors
uint8 backend_health
