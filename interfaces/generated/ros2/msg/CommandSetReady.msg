# GENERATED from ../../../schema/avva_phase1.idl by generate_ros2_msgs.py. DO NOT EDIT.
CommonHeader header
uint64 target_tick_id
CommandCounts counts
ControlSetMeta control_set_meta
uint8 completeness
uint8 validity
string<=63[<=32] missing_actor_ids
ActorReason[<=32] actor_reasons
bool reports_truncated
HealthSummary health_summary
uint8[32] snapshot_hash
