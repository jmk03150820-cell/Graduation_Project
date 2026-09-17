# GENERATED from ../../../schema/avva_phase1.idl by generate_ros2_msgs.py. DO NOT EDIT.
CommonHeader header
uint64 based_on_tick_id
uint64 target_tick_id
ControlSetMeta control_set_meta
NpcControlItem[<=1200] items
ActorLifecycleIntent[<=1200] lifecycle_intents
uint64 engine_step_id
uint64 engine_sim_time_ns
uint64 engine_step_duration_ns
