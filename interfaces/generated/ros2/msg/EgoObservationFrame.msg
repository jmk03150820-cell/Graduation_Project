# GENERATED from ../../../schema/avva_phase1.idl by generate_ros2_msgs.py. DO NOT EDIT.
CommonHeader header
uint64 state_tick_id
ActorState ego_state
ObservedObject[<=1200] gt_objects
ControlSetMeta control_set_meta
bool perfect_prediction_enabled
PredictedPath[<=1200] predicted_paths
uint64 deadline_budget_ns
