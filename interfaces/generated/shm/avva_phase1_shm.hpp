// GENERATED from schema/avva_phase1.idl. DO NOT EDIT.
#pragma once
#include <array>
#include <cstdint>
#include <type_traits>
namespace avva::shm {
template<std::size_t N> struct FixedString { std::uint16_t length{}; std::array<std::uint8_t,N> data{}; };
template<class T,std::size_t N> struct FixedSequence { std::uint32_t count{}; std::array<T,N> data{}; };
using BoundedId = FixedString<63>; using BoundedText255 = FixedString<255>;
using Uuid128 = std::array<std::uint8_t,16>;
using Hash256 = std::array<std::uint8_t,32>;
using MessageKind = std::uint16_t; using ReasonCode = std::uint16_t;
inline constexpr MessageKind RUN_MANIFEST = 1;
inline constexpr MessageKind COMPONENT_READY = 2;
inline constexpr MessageKind RUN_CONTROL_COMMAND = 3;
inline constexpr MessageKind RUN_CONTROL_ACK = 4;
inline constexpr MessageKind WORLD_STATE_FRAME = 10;
inline constexpr MessageKind EGO_OBSERVATION_FRAME = 11;
inline constexpr MessageKind EGO_CONTROL_COMMAND = 12;
inline constexpr MessageKind NPC_CONTROL_BATCH = 13;
inline constexpr MessageKind COMMAND_SET_READY = 20;
inline constexpr MessageKind ADVANCE_FRAME = 21;
inline constexpr MessageKind FRAME_COMPLETE = 22;
inline constexpr MessageKind FRAME_COMPLETE_ACK = 23;
inline constexpr MessageKind COMPONENT_HEARTBEAT = 30;
inline constexpr MessageKind REJECT_NOTICE = 40;
inline constexpr MessageKind STATUS_EVENT = 41;
inline constexpr MessageKind TIMING_EVENT = 42;
inline constexpr ReasonCode UNKNOWN_REASON = 0;
inline constexpr ReasonCode SCHEMA_INCOMPATIBLE = 1;
inline constexpr ReasonCode RUN_MISMATCH = 2;
inline constexpr ReasonCode EPOCH_MISMATCH = 3;
inline constexpr ReasonCode MANIFEST_MISMATCH = 4;
inline constexpr ReasonCode INVALID_HEADER_SCOPE = 5;
inline constexpr ReasonCode STALE_TICK = 1000;
inline constexpr ReasonCode FUTURE_TICK = 1001;
inline constexpr ReasonCode TICK_RELATION_INVALID = 1002;
inline constexpr ReasonCode DUPLICATE_IDEMPOTENT = 1003;
inline constexpr ReasonCode PAYLOAD_CONFLICT = 1004;
inline constexpr ReasonCode DECISION_MISMATCH = 1005;
inline constexpr ReasonCode SNAPSHOT_HASH_MISMATCH = 1006;
inline constexpr ReasonCode INVALID_STATE = 1007;
inline constexpr ReasonCode ACTOR_SET_MISMATCH = 2000;
inline constexpr ReasonCode MISSING_ACTOR_RESPONSE = 2001;
inline constexpr ReasonCode DUPLICATE_ACTOR_ID = 2002;
inline constexpr ReasonCode UNKNOWN_ACTOR_ID = 2003;
inline constexpr ReasonCode SPAWN_FAILED = 2004;
inline constexpr ReasonCode REMOVE_FAILED = 2005;
inline constexpr ReasonCode COMPUTE_FAILED = 2006;
inline constexpr ReasonCode LIFECYCLE_INVALID = 2007;
inline constexpr ReasonCode INVALID_COMMAND_COMBINATION = 2008;
inline constexpr ReasonCode CAN_WRITE_FAILED = 3000;
inline constexpr ReasonCode EMERGENCY_STOP_TRIGGERED = 3001;
inline constexpr ReasonCode CORE_ABORTED = 3002;
inline constexpr ReasonCode SAFETY_GATE_REJECTED = 3003;
inline constexpr ReasonCode TRANSPORT_ERROR = 4000;
inline constexpr ReasonCode EVIDENCE_PERSIST_FAILED = 4001;
inline constexpr ReasonCode QUEUE_TIMEOUT = 4002;
inline constexpr ReasonCode PUBLISH_TIMEOUT = 4003;
inline constexpr ReasonCode DEADLINE_EXCEEDED = 5000;
inline constexpr ReasonCode TIMEOUT = 5001;
inline constexpr ReasonCode HEARTBEAT_LOST = 5002;
inline constexpr ReasonCode SEQUENCE_ANOMALY = 5003;
inline constexpr ReasonCode NATIVE_APPLY_ERROR = 5004;
inline constexpr ReasonCode NATIVE_TICK_ERROR = 5005;
inline constexpr ReasonCode NATIVE_SESSION_LOST = 5006;
inline constexpr ReasonCode INVALID_FIELD_MASK = 6000;
inline constexpr ReasonCode NON_FINITE_VALUE = 6001;
inline constexpr ReasonCode INVALID_QUATERNION = 6002;
inline constexpr ReasonCode OUT_OF_RANGE = 6003;
inline constexpr ReasonCode UNSUPPORTED_CONTROL_MODE = 6004;
inline constexpr ReasonCode CONVERSION_FAILED = 6005;
inline constexpr ReasonCode INTERNAL_ERROR = 9000;
enum class ScopeKind : std::uint8_t {
  SCOPE_UNSPECIFIED = 0,
  GLOBAL = 1,
  SIM = 2,
  EGO = 3,
};
enum class ComponentId : std::uint8_t {
  COMPONENT_UNSPECIFIED = 0,
  CORE = 1,
  SIM_BACKEND = 2,
  TRAFFIC_ADAPTER = 3,
  EGO_ADAPTER = 4,
  RECORDER = 5,
  RC_SAFETY_GATE = 6,
  NATIVE_ADAPTER = 7,
};
enum class ExecutionMode : std::uint8_t {
  EXECUTION_UNSPECIFIED = 0,
  SYNC_FIXED_STEP = 1,
};
enum class ActorRole : std::uint8_t {
  ACTOR_ROLE_UNKNOWN = 0,
  ACTOR_ROLE_EGO = 1,
  NPC = 2,
  PEDESTRIAN = 3,
  STATIC_OBJECT = 4,
};
enum class ActorClass : std::uint8_t {
  ACTOR_CLASS_UNKNOWN = 0,
  PASSENGER_CAR = 1,
  TRUCK = 2,
  BUS = 3,
  MOTORCYCLE = 4,
  BICYCLE = 5,
  ACTOR_CLASS_PEDESTRIAN = 6,
  ACTOR_CLASS_STATIC_OBJECT = 7,
};
enum class ControlOwner : std::uint8_t {
  CONTROL_OWNER_UNSPECIFIED = 0,
  EGO_STACK = 1,
  TRAFFIC_ENGINE = 2,
  SIMULATOR_NATIVE = 3,
  CONTROL_OWNER_NONE = 4,
};
enum class Lifecycle : std::uint8_t {
  LIFECYCLE_UNSPECIFIED = 0,
  SPAWN_REQUESTED = 1,
  ACTIVE = 2,
  DESPAWN_REQUESTED = 3,
  REMOVED = 4,
};
enum class RepresentationLevel : std::uint8_t {
  REPRESENTATION_UNSPECIFIED = 0,
  LOGICAL = 1,
  ACTIVE_PROXY = 2,
  FULL_PHYSICS = 3,
};
enum class LifecycleIntent : std::uint8_t {
  LIFECYCLE_INTENT_UNSPECIFIED = 0,
  INTENT_SPAWN_REQUESTED = 1,
  INTENT_DESPAWN_REQUESTED = 2,
};
enum class CommandAction : std::uint8_t {
  COMMAND_ACTION_UNSPECIFIED = 0,
  APPLY = 1,
  HOLD = 2,
  NO_OP = 3,
};
enum class ItemStatus : std::uint8_t {
  ITEM_STATUS_UNSPECIFIED = 0,
  ITEM_OK = 1,
  ITEM_FAILED = 2,
  DESPAWNED = 3,
};
enum class CommandStatus : std::uint8_t {
  COMMAND_STATUS_UNSPECIFIED = 0,
  COMMAND_OK = 1,
  COMMAND_FAILED = 2,
};
enum class ControlMode : std::uint8_t {
  CONTROL_MODE_UNSPECIFIED = 0,
  VELOCITY_TARGET = 1,
  ACCELERATION_TARGET = 2,
  DIRECT_ACTUATION = 3,
};
enum class Completeness : std::uint8_t {
  COMPLETENESS_UNSPECIFIED = 0,
  FULL = 1,
  PARTIAL = 2,
};
enum class Validity : std::uint8_t {
  VALIDITY_UNSPECIFIED = 0,
  VALID = 1,
  INVALID = 2,
};
enum class StateValidity : std::uint8_t {
  STATE_UNKNOWN = 0,
  STATE_VALID = 1,
  ESTIMATED = 2,
  STATE_INVALID = 3,
};
enum class SourceStack : std::uint8_t {
  SOURCE_UNSPECIFIED = 0,
  AUTOWARE = 1,
  RL = 2,
  MODULE_CHAIN = 3,
};
enum class Gear : std::uint8_t {
  GEAR_UNKNOWN = 0,
  PARK = 1,
  REVERSE = 2,
  NEUTRAL = 3,
  DRIVE = 4,
};
enum class HealthStatus : std::uint8_t {
  HEALTH_UNKNOWN = 0,
  HEALTH_OK = 1,
  DEGRADED = 2,
  HEALTH_ERROR = 3,
  LOST = 4,
};
enum class ReadyStatus : std::uint8_t {
  READY_UNSPECIFIED = 0,
  READY = 1,
  READY_REJECTED = 2,
};
enum class AckStatus : std::uint8_t {
  ACK_UNSPECIFIED = 0,
  ACCEPTED = 1,
  ACK_REJECTED = 2,
};
enum class AdvanceAction : std::uint8_t {
  ADVANCE_UNSPECIFIED = 0,
  ADVANCE = 1,
  ADVANCE_ABORT = 2,
};
enum class RunControlAction : std::uint8_t {
  RUN_CONTROL_UNSPECIFIED = 0,
  PREPARE = 1,
  START = 2,
  PAUSE = 3,
  RESUME = 4,
  RUN_ABORT = 5,
};
enum class RunState : std::uint8_t {
  RUN_UNKNOWN = 0,
  IDLE = 1,
  CONFIGURING = 2,
  WARMING_UP = 3,
  RUNNING = 4,
  PAUSED = 5,
  ABORTING = 6,
  COMPLETED = 7,
  RUN_ERROR = 8,
};
enum class ComponentState : std::uint8_t {
  COMPONENT_UNKNOWN = 0,
  STARTING = 1,
  COMPONENT_READY_STATE = 2,
  COMPONENT_RUNNING = 3,
  COMPONENT_PAUSED = 4,
  STOPPING = 5,
  STOPPED = 6,
  COMPONENT_ERROR = 7,
};
enum class Severity : std::uint8_t {
  SEVERITY_UNKNOWN = 0,
  DEBUG = 1,
  INFO = 2,
  WARN = 3,
  ERROR = 4,
  FATAL = 5,
};
enum class ActionTaken : std::uint8_t {
  ACTION_NONE = 0,
  ACKED = 1,
  RESENT = 2,
  ACTION_REJECTED = 3,
  ABORTED = 4,
  EMERGENCY_STOPPED = 5,
  LOGGED_ONLY = 6,
};
enum class EventType : std::uint8_t {
  EVENT_UNKNOWN = 0,
  STATE_TRANSITION = 1,
  WARMUP_PROGRESS = 2,
  VALIDATION_REJECTED = 3,
  TIMEOUT_DETECTED = 4,
  NATIVE_FAILURE = 5,
  SAFETY_EVENT = 6,
  EVIDENCE_FAILURE = 7,
};
enum class DetailKind : std::uint8_t {
  DETAIL_NONE = 0,
  VALIDATION_DETAIL = 1,
  TIMEOUT_DETAIL = 2,
  NATIVE_DETAIL = 3,
  TRANSPORT_DETAIL = 4,
  SAFETY_DETAIL = 5,
  WARMUP_PROGRESS_DETAIL = 6,
};
enum class TimingSegment : std::uint8_t {
  TIMING_UNKNOWN = 0,
  RELAY = 1,
  STACK = 2,
  CONVERT = 3,
  ASSEMBLY = 4,
  PUBLISH_CALL = 5,
  FRAME_INPUT_WAIT = 6,
  NATIVE_APPLY = 7,
  NATIVE_TICK = 8,
  STATE_PUBLISH = 9,
  COMPLETE_ACK_WAIT = 10,
};
enum class TransportProfile : std::uint8_t {
  TRANSPORT_UNSPECIFIED = 0,
  INPROC_TEST = 1,
  LOCAL_HYBRID = 2,
  ROS2_REFERENCE = 3,
};
enum class PlantType : std::uint8_t {
  PLANT_UNSPECIFIED = 0,
  VIRTUAL = 1,
  CHAMELEON = 2,
  SPARK = 3,
  THOR = 4,
};
enum class NativeAdapterType : std::uint8_t {
  NATIVE_ADAPTER_UNSPECIFIED = 0,
  CARLA = 1,
  MORAI = 2,
  AURELION = 3,
};
enum class TrafficEngineType : std::uint8_t {
  TRAFFIC_ENGINE_UNSPECIFIED = 0,
  SUMO_TRACI = 1,
  TERASIM = 2,
  MOSS = 3,
};
enum class DeterminismCapability : std::uint8_t {
  DETERMINISM_UNKNOWN = 0,
  BIT_EXACT = 1,
  NUMERIC_TOLERANCE = 2,
  NON_DETERMINISTIC = 3,
};
struct OptionalBoundedId {
  bool has_value{};
  BoundedId value{};
};
struct OptionalUuid128 {
  bool has_value{};
  Uuid128 value{};
};
struct OptionalUint64 {
  bool has_value{};
  std::uint64_t value{};
};
struct OptionalHash256 {
  bool has_value{};
  Hash256 value{};
};
struct SemanticVersion {
  std::uint16_t major{};
  std::uint16_t minor{};
  std::uint16_t patch{};
};
struct ContentRef {
  BoundedId id{};
  Hash256 sha256{};
};
struct Vector3d {
  double x{};
  double y{};
  double z{};
};
struct Quaterniond {
  double x{};
  double y{};
  double z{};
  double w{};
};
struct Dimensions3d {
  double length_m{};
  double width_m{};
  double height_m{};
};
struct OptionalDimensions3d {
  bool has_value{};
  Dimensions3d value{};
};
struct Pose3d {
  Vector3d position_m{};
  Quaterniond orientation_xyzw{};
};
struct Twist3d {
  Vector3d linear_mps{};
  Vector3d angular_rad_s{};
};
struct Covariance6x6 {
  std::array<double,36> values{};
};
struct NativeFrameRef {
  Uuid128 native_session_id{};
  std::uint32_t native_generation{};
  BoundedId native_frame_id{};
};
struct ControlSetMeta {
  std::uint64_t version{};
  Hash256 digest{};
  std::uint16_t expected_ego_count{};
  std::uint32_t expected_traffic_count{};
};
struct CommonHeader {
  std::uint16_t schema_major{};
  std::uint16_t schema_minor{};
  Uuid128 run_id{};
  std::uint64_t run_epoch{};
  ScopeKind scope_kind{};
  OptionalBoundedId sim_id{};
  OptionalBoundedId ego_id{};
  ComponentId producer_id{};
  Uuid128 producer_instance_id{};
  std::uint64_t event_seq{};
  OptionalUuid128 correlation_id{};
  OptionalUint64 sim_time_ns{};
  std::int64_t wall_time_unix_ns{};
  Hash256 payload_hash{};
};
struct AdapterCapability {
  std::uint32_t control_mode_mask{};
  bool supports_physics{};
  bool supports_spawn{};
  bool supports_despawn{};
  std::uint32_t max_actor_count{};
};
struct SimInstanceProfile {
  BoundedId sim_id{};
  NativeAdapterType adapter_type{};
  BoundedId adapter_version{};
  AdapterCapability capability{};
  DeterminismCapability determinism{};
};
struct EgoProfile {
  BoundedId ego_id{};
  SourceStack source_stack{};
  PlantType plant_type{};
  BoundedId vehicle_profile_id{};
};
struct TrafficProfile {
  TrafficEngineType engine_type{};
  BoundedId binary_version{};
  std::uint64_t fixed_step_ns{};
  std::uint64_t derived_seed{};
  Hash256 demand_hash{};
};
struct TimeoutConfig {
  std::uint64_t ready_timeout_ns{};
  std::uint64_t apply_timeout_ns{};
  std::uint64_t tick_timeout_ns{};
  std::uint64_t heartbeat_timeout_ns{};
  std::uint64_t queue_timeout_ns{};
  std::uint64_t publish_ack_timeout_ns{};
};
struct LoggingProfile {
  std::uint16_t critical_depth{};
  std::uint16_t general_depth{};
  std::uint64_t flush_period_ns{};
  std::uint16_t retention_days{};
};
struct SoftwareVersion {
  ComponentId component_id{};
  SemanticVersion semantic_version{};
  BoundedId git_commit{};
  BoundedId build_id{};
  OptionalHash256 image_digest{};
};
struct DetailData {
  DetailKind kind{};
  std::int64_t numeric0{};
  std::int64_t numeric1{};
  OptionalBoundedId id0{};
  OptionalHash256 related_hash{};
};
struct ActorState {
  BoundedId actor_id{};
  ActorRole actor_role{};
  ControlOwner control_owner{};
  Lifecycle lifecycle{};
  RepresentationLevel representation_level{};
  ActorClass class_id{};
  Vector3d position_m{};
  Quaterniond orientation_xyzw{};
  Vector3d velocity_mps{};
  Vector3d acceleration_mps2{};
  Vector3d angular_velocity_rad_s{};
  OptionalDimensions3d dimensions_m{};
  StateValidity state_validity{};
};
struct OptionalActorState {
  bool has_value{};
  ActorState value{};
};
struct OptionalRepresentationLevel {
  bool has_value{};
  RepresentationLevel value{};
};
struct ObservedObject {
  BoundedId actor_id{};
  ActorClass class_id{};
  Pose3d pose{};
  Twist3d twist{};
  Covariance6x6 pose_covariance{};
  Covariance6x6 twist_covariance{};
  StateValidity state_validity{};
};
struct PredictionPoint {
  std::uint64_t relative_time_ns{};
  Pose3d pose{};
  Vector3d velocity_mps{};
};
struct PredictedPath {
  BoundedId actor_id{};
  std::uint16_t path_id{};
  double probability{};
  FixedSequence<PredictionPoint,32> points{};
};
struct TickRefs {
  bool has_state_tick_id{};
  std::uint64_t state_tick_id{};
  bool has_based_on_tick_id{};
  std::uint64_t based_on_tick_id{};
  bool has_target_tick_id{};
  std::uint64_t target_tick_id{};
  bool has_decision_id{};
  Uuid128 decision_id{};
};
struct NeutralControl {
  ControlMode control_mode{};
  std::uint32_t valid_fields_mask{};
  double steering_tire_angle_rad{};
  double steering_tire_rotation_rate_rad_s{};
  double velocity_mps{};
  double acceleration_mps2{};
  double jerk_mps3{};
  double throttle{};
  double brake{};
  Gear gear{};
  bool hand_brake{};
};
struct OptionalNeutralControl {
  bool has_value{};
  NeutralControl value{};
};
struct NpcControlItem {
  BoundedId actor_id{};
  CommandAction action{};
  ItemStatus status{};
  OptionalNeutralControl control{};
  ReasonCode failure_reason{};
};
struct ActorLifecycleIntent {
  BoundedId actor_id{};
  LifecycleIntent intent{};
  OptionalActorState initial_state{};
  OptionalRepresentationLevel representation_level{};
  std::uint16_t intent_retry_count{};
  ReasonCode reason_context{};
};
struct CommandCounts {
  std::uint16_t expected_ego{};
  std::uint16_t received_ego{};
  std::uint16_t valid_ego{};
  std::uint16_t failed_ego{};
  std::uint32_t expected_traffic{};
  std::uint32_t received_traffic{};
  std::uint32_t valid_traffic{};
  std::uint32_t failed_traffic{};
};
struct ActorReason {
  BoundedId actor_id{};
  ReasonCode reason_code{};
};
struct ComponentHealthIssue {
  ComponentId component_id{};
  HealthStatus health_status{};
  ReasonCode reason_code{};
};
struct HealthSummary {
  HealthStatus overall_health{};
  FixedSequence<ComponentHealthIssue,32> issues{};
  bool truncated{};
};
struct Warning {
  ReasonCode code{};
  DetailData data{};
  BoundedText255 detail_message{};
};
struct RunManifest {
  CommonHeader header{};
  SemanticVersion manifest_version{};
  ExecutionMode execution_mode{};
  std::uint64_t fixed_step_ns{};
  std::uint32_t warmup_ticks{};
  std::uint64_t seed{};
  ContentRef map_ref{};
  ContentRef scenario_ref{};
  FixedSequence<SimInstanceProfile,8> sim_instances{};
  FixedSequence<BoundedId,8> required_sim_ids{};
  FixedSequence<EgoProfile,8> ego_profiles{};
  TrafficProfile traffic_profile{};
  std::uint32_t max_actor_count{};
  std::uint16_t max_report_count{};
  std::uint16_t max_prediction_points{};
  std::uint64_t deadline_budget_ns{};
  TimeoutConfig timeouts{};
  TransportProfile transport_profile{};
  FixedSequence<SoftwareVersion,32> software_versions{};
  LoggingProfile logging_profile{};
};
struct ComponentReady {
  CommonHeader header{};
  Hash256 manifest_hash{};
  ReadyStatus ready_status{};
  Hash256 capability_digest{};
  SoftwareVersion software_version{};
  FixedSequence<ReasonCode,32> reason_codes{};
};
struct RunControlCommand {
  CommonHeader header{};
  Uuid128 command_id{};
  RunControlAction action{};
  ReasonCode reason_code{};
};
struct RunControlAck {
  CommonHeader header{};
  Uuid128 command_id{};
  RunControlAction action{};
  AckStatus ack_status{};
  RunState current_run_state{};
  ReasonCode reason_code{};
};
struct WorldStateFrame {
  CommonHeader header{};
  std::uint64_t state_tick_id{};
  NativeFrameRef native_frame_ref{};
  ControlSetMeta control_set_meta{};
  FixedSequence<BoundedId,8> expected_ego_ids{};
  FixedSequence<BoundedId,1200> expected_traffic_actor_ids{};
  FixedSequence<ActorState,1200> actors{};
  HealthStatus backend_health{};
};
struct EgoObservationFrame {
  CommonHeader header{};
  std::uint64_t state_tick_id{};
  ActorState ego_state{};
  FixedSequence<ObservedObject,1200> gt_objects{};
  ControlSetMeta control_set_meta{};
  bool perfect_prediction_enabled{};
  FixedSequence<PredictedPath,1200> predicted_paths{};
  std::uint64_t deadline_budget_ns{};
};
struct EgoControlCommand {
  CommonHeader header{};
  std::uint64_t based_on_tick_id{};
  std::uint64_t target_tick_id{};
  SourceStack source_stack{};
  std::uint64_t control_time_sim_ns{};
  CommandStatus command_status{};
  OptionalNeutralControl control{};
  ReasonCode failure_reason{};
};
struct NpcControlBatch {
  CommonHeader header{};
  std::uint64_t based_on_tick_id{};
  std::uint64_t target_tick_id{};
  ControlSetMeta control_set_meta{};
  FixedSequence<NpcControlItem,1200> items{};
  FixedSequence<ActorLifecycleIntent,1200> lifecycle_intents{};
  std::uint64_t engine_step_id{};
  std::uint64_t engine_sim_time_ns{};
  std::uint64_t engine_step_duration_ns{};
};
struct CommandSetReady {
  CommonHeader header{};
  std::uint64_t target_tick_id{};
  CommandCounts counts{};
  ControlSetMeta control_set_meta{};
  Completeness completeness{};
  Validity validity{};
  FixedSequence<BoundedId,32> missing_actor_ids{};
  FixedSequence<ActorReason,32> actor_reasons{};
  bool reports_truncated{};
  HealthSummary health_summary{};
  Hash256 snapshot_hash{};
};
struct AdvanceFrame {
  CommonHeader header{};
  Uuid128 decision_id{};
  std::uint64_t target_tick_id{};
  Hash256 snapshot_hash{};
  AdvanceAction action{};
  ReasonCode abort_reason_code{};
};
struct FrameComplete {
  CommonHeader header{};
  std::uint64_t state_tick_id{};
  Uuid128 decision_id{};
  Hash256 applied_snapshot_hash{};
  NativeFrameRef native_frame_ref{};
  std::uint64_t tick_duration_ns{};
  ControlSetMeta control_set_meta{};
  FixedSequence<Warning,32> warnings{};
};
struct FrameCompleteAck {
  CommonHeader header{};
  Uuid128 decision_id{};
  std::uint64_t state_tick_id{};
  Hash256 applied_snapshot_hash{};
  AckStatus ack_status{};
  ReasonCode reason_code{};
};
struct ComponentHeartbeat {
  CommonHeader header{};
  ComponentState component_state{};
  HealthStatus health_status{};
  OptionalUint64 last_state_tick_id{};
  OptionalUint64 last_target_tick_id{};
  OptionalUuid128 native_session_id{};
  std::uint64_t monotonic_time_ns{};
};
struct RejectNotice {
  CommonHeader header{};
  ComponentId target_component_id{};
  MessageKind rejected_message_kind{};
  Hash256 related_hash{};
  TickRefs tick_refs{};
  ReasonCode reason_code{};
  bool retryable{};
  DetailData detail_data{};
  BoundedText255 detail_message{};
};
struct StatusEvent {
  CommonHeader header{};
  ComponentId subject_component_id{};
  EventType event_type{};
  Severity severity{};
  ReasonCode reason_code{};
  ActionTaken action_taken{};
  RunState run_state{};
  TickRefs tick_refs{};
  DetailData detail_data{};
  BoundedText255 detail_message{};
};
struct TimingEvent {
  CommonHeader header{};
  TimingSegment segment{};
  std::uint64_t start_monotonic_ns{};
  std::uint64_t end_monotonic_ns{};
  std::uint64_t duration_ns{};
  BoundedId monotonic_clock_domain_id{};
  TickRefs tick_refs{};
};
static_assert(std::is_trivially_copyable_v<OptionalBoundedId>);
static_assert(std::is_trivially_copyable_v<OptionalUuid128>);
static_assert(std::is_trivially_copyable_v<OptionalUint64>);
static_assert(std::is_trivially_copyable_v<OptionalHash256>);
static_assert(std::is_trivially_copyable_v<SemanticVersion>);
static_assert(std::is_trivially_copyable_v<ContentRef>);
static_assert(std::is_trivially_copyable_v<Vector3d>);
static_assert(std::is_trivially_copyable_v<Quaterniond>);
static_assert(std::is_trivially_copyable_v<Dimensions3d>);
static_assert(std::is_trivially_copyable_v<OptionalDimensions3d>);
static_assert(std::is_trivially_copyable_v<Pose3d>);
static_assert(std::is_trivially_copyable_v<Twist3d>);
static_assert(std::is_trivially_copyable_v<Covariance6x6>);
static_assert(std::is_trivially_copyable_v<NativeFrameRef>);
static_assert(std::is_trivially_copyable_v<ControlSetMeta>);
static_assert(std::is_trivially_copyable_v<CommonHeader>);
static_assert(std::is_trivially_copyable_v<AdapterCapability>);
static_assert(std::is_trivially_copyable_v<SimInstanceProfile>);
static_assert(std::is_trivially_copyable_v<EgoProfile>);
static_assert(std::is_trivially_copyable_v<TrafficProfile>);
static_assert(std::is_trivially_copyable_v<TimeoutConfig>);
static_assert(std::is_trivially_copyable_v<LoggingProfile>);
static_assert(std::is_trivially_copyable_v<SoftwareVersion>);
static_assert(std::is_trivially_copyable_v<DetailData>);
static_assert(std::is_trivially_copyable_v<ActorState>);
static_assert(std::is_trivially_copyable_v<OptionalActorState>);
static_assert(std::is_trivially_copyable_v<OptionalRepresentationLevel>);
static_assert(std::is_trivially_copyable_v<ObservedObject>);
static_assert(std::is_trivially_copyable_v<PredictionPoint>);
static_assert(std::is_trivially_copyable_v<PredictedPath>);
static_assert(std::is_trivially_copyable_v<TickRefs>);
static_assert(std::is_trivially_copyable_v<NeutralControl>);
static_assert(std::is_trivially_copyable_v<OptionalNeutralControl>);
static_assert(std::is_trivially_copyable_v<NpcControlItem>);
static_assert(std::is_trivially_copyable_v<ActorLifecycleIntent>);
static_assert(std::is_trivially_copyable_v<CommandCounts>);
static_assert(std::is_trivially_copyable_v<ActorReason>);
static_assert(std::is_trivially_copyable_v<ComponentHealthIssue>);
static_assert(std::is_trivially_copyable_v<HealthSummary>);
static_assert(std::is_trivially_copyable_v<Warning>);
static_assert(std::is_trivially_copyable_v<RunManifest>);
static_assert(std::is_trivially_copyable_v<ComponentReady>);
static_assert(std::is_trivially_copyable_v<RunControlCommand>);
static_assert(std::is_trivially_copyable_v<RunControlAck>);
static_assert(std::is_trivially_copyable_v<WorldStateFrame>);
static_assert(std::is_trivially_copyable_v<EgoObservationFrame>);
static_assert(std::is_trivially_copyable_v<EgoControlCommand>);
static_assert(std::is_trivially_copyable_v<NpcControlBatch>);
static_assert(std::is_trivially_copyable_v<CommandSetReady>);
static_assert(std::is_trivially_copyable_v<AdvanceFrame>);
static_assert(std::is_trivially_copyable_v<FrameComplete>);
static_assert(std::is_trivially_copyable_v<FrameCompleteAck>);
static_assert(std::is_trivially_copyable_v<ComponentHeartbeat>);
static_assert(std::is_trivially_copyable_v<RejectNotice>);
static_assert(std::is_trivially_copyable_v<StatusEvent>);
static_assert(std::is_trivially_copyable_v<TimingEvent>);
} // namespace avva::shm
