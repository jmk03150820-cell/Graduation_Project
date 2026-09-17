# GENERATED from schema/avva_phase1.idl. DO NOT EDIT.
from __future__ import annotations
from dataclasses import dataclass
from enum import IntEnum
from typing import TypeAlias

Uuid128: TypeAlias = bytes
Hash256: TypeAlias = bytes
BoundedId: TypeAlias = str
BoundedText255: TypeAlias = str
MessageKind: TypeAlias = int
ReasonCode: TypeAlias = int

RUN_MANIFEST: MessageKind = 1
COMPONENT_READY: MessageKind = 2
RUN_CONTROL_COMMAND: MessageKind = 3
RUN_CONTROL_ACK: MessageKind = 4
WORLD_STATE_FRAME: MessageKind = 10
EGO_OBSERVATION_FRAME: MessageKind = 11
EGO_CONTROL_COMMAND: MessageKind = 12
NPC_CONTROL_BATCH: MessageKind = 13
COMMAND_SET_READY: MessageKind = 20
ADVANCE_FRAME: MessageKind = 21
FRAME_COMPLETE: MessageKind = 22
FRAME_COMPLETE_ACK: MessageKind = 23
COMPONENT_HEARTBEAT: MessageKind = 30
REJECT_NOTICE: MessageKind = 40
STATUS_EVENT: MessageKind = 41
TIMING_EVENT: MessageKind = 42
UNKNOWN_REASON: ReasonCode = 0
SCHEMA_INCOMPATIBLE: ReasonCode = 1
RUN_MISMATCH: ReasonCode = 2
EPOCH_MISMATCH: ReasonCode = 3
MANIFEST_MISMATCH: ReasonCode = 4
INVALID_HEADER_SCOPE: ReasonCode = 5
STALE_TICK: ReasonCode = 1000
FUTURE_TICK: ReasonCode = 1001
TICK_RELATION_INVALID: ReasonCode = 1002
DUPLICATE_IDEMPOTENT: ReasonCode = 1003
PAYLOAD_CONFLICT: ReasonCode = 1004
DECISION_MISMATCH: ReasonCode = 1005
SNAPSHOT_HASH_MISMATCH: ReasonCode = 1006
INVALID_STATE: ReasonCode = 1007
ACTOR_SET_MISMATCH: ReasonCode = 2000
MISSING_ACTOR_RESPONSE: ReasonCode = 2001
DUPLICATE_ACTOR_ID: ReasonCode = 2002
UNKNOWN_ACTOR_ID: ReasonCode = 2003
SPAWN_FAILED: ReasonCode = 2004
REMOVE_FAILED: ReasonCode = 2005
COMPUTE_FAILED: ReasonCode = 2006
LIFECYCLE_INVALID: ReasonCode = 2007
INVALID_COMMAND_COMBINATION: ReasonCode = 2008
CAN_WRITE_FAILED: ReasonCode = 3000
EMERGENCY_STOP_TRIGGERED: ReasonCode = 3001
CORE_ABORTED: ReasonCode = 3002
SAFETY_GATE_REJECTED: ReasonCode = 3003
TRANSPORT_ERROR: ReasonCode = 4000
EVIDENCE_PERSIST_FAILED: ReasonCode = 4001
QUEUE_TIMEOUT: ReasonCode = 4002
PUBLISH_TIMEOUT: ReasonCode = 4003
DEADLINE_EXCEEDED: ReasonCode = 5000
TIMEOUT: ReasonCode = 5001
HEARTBEAT_LOST: ReasonCode = 5002
SEQUENCE_ANOMALY: ReasonCode = 5003
NATIVE_APPLY_ERROR: ReasonCode = 5004
NATIVE_TICK_ERROR: ReasonCode = 5005
NATIVE_SESSION_LOST: ReasonCode = 5006
INVALID_FIELD_MASK: ReasonCode = 6000
NON_FINITE_VALUE: ReasonCode = 6001
INVALID_QUATERNION: ReasonCode = 6002
OUT_OF_RANGE: ReasonCode = 6003
UNSUPPORTED_CONTROL_MODE: ReasonCode = 6004
CONVERSION_FAILED: ReasonCode = 6005
INTERNAL_ERROR: ReasonCode = 9000

class ScopeKind(IntEnum):
    SCOPE_UNSPECIFIED = 0
    GLOBAL = 1
    SIM = 2
    EGO = 3

class ComponentId(IntEnum):
    COMPONENT_UNSPECIFIED = 0
    CORE = 1
    SIM_BACKEND = 2
    TRAFFIC_ADAPTER = 3
    EGO_ADAPTER = 4
    RECORDER = 5
    RC_SAFETY_GATE = 6
    NATIVE_ADAPTER = 7

class ExecutionMode(IntEnum):
    EXECUTION_UNSPECIFIED = 0
    SYNC_FIXED_STEP = 1

class ActorRole(IntEnum):
    ACTOR_ROLE_UNKNOWN = 0
    ACTOR_ROLE_EGO = 1
    NPC = 2
    PEDESTRIAN = 3
    STATIC_OBJECT = 4

class ActorClass(IntEnum):
    ACTOR_CLASS_UNKNOWN = 0
    PASSENGER_CAR = 1
    TRUCK = 2
    BUS = 3
    MOTORCYCLE = 4
    BICYCLE = 5
    ACTOR_CLASS_PEDESTRIAN = 6
    ACTOR_CLASS_STATIC_OBJECT = 7

class ControlOwner(IntEnum):
    CONTROL_OWNER_UNSPECIFIED = 0
    EGO_STACK = 1
    TRAFFIC_ENGINE = 2
    SIMULATOR_NATIVE = 3
    CONTROL_OWNER_NONE = 4

class Lifecycle(IntEnum):
    LIFECYCLE_UNSPECIFIED = 0
    SPAWN_REQUESTED = 1
    ACTIVE = 2
    DESPAWN_REQUESTED = 3
    REMOVED = 4

class RepresentationLevel(IntEnum):
    REPRESENTATION_UNSPECIFIED = 0
    LOGICAL = 1
    ACTIVE_PROXY = 2
    FULL_PHYSICS = 3

class LifecycleIntent(IntEnum):
    LIFECYCLE_INTENT_UNSPECIFIED = 0
    INTENT_SPAWN_REQUESTED = 1
    INTENT_DESPAWN_REQUESTED = 2

class CommandAction(IntEnum):
    COMMAND_ACTION_UNSPECIFIED = 0
    APPLY = 1
    HOLD = 2
    NO_OP = 3

class ItemStatus(IntEnum):
    ITEM_STATUS_UNSPECIFIED = 0
    ITEM_OK = 1
    ITEM_FAILED = 2
    DESPAWNED = 3

class CommandStatus(IntEnum):
    COMMAND_STATUS_UNSPECIFIED = 0
    COMMAND_OK = 1
    COMMAND_FAILED = 2

class ControlMode(IntEnum):
    CONTROL_MODE_UNSPECIFIED = 0
    VELOCITY_TARGET = 1
    ACCELERATION_TARGET = 2
    DIRECT_ACTUATION = 3

class Completeness(IntEnum):
    COMPLETENESS_UNSPECIFIED = 0
    FULL = 1
    PARTIAL = 2

class Validity(IntEnum):
    VALIDITY_UNSPECIFIED = 0
    VALID = 1
    INVALID = 2

class StateValidity(IntEnum):
    STATE_UNKNOWN = 0
    STATE_VALID = 1
    ESTIMATED = 2
    STATE_INVALID = 3

class SourceStack(IntEnum):
    SOURCE_UNSPECIFIED = 0
    AUTOWARE = 1
    RL = 2
    MODULE_CHAIN = 3

class Gear(IntEnum):
    GEAR_UNKNOWN = 0
    PARK = 1
    REVERSE = 2
    NEUTRAL = 3
    DRIVE = 4

class HealthStatus(IntEnum):
    HEALTH_UNKNOWN = 0
    HEALTH_OK = 1
    DEGRADED = 2
    HEALTH_ERROR = 3
    LOST = 4

class ReadyStatus(IntEnum):
    READY_UNSPECIFIED = 0
    READY = 1
    READY_REJECTED = 2

class AckStatus(IntEnum):
    ACK_UNSPECIFIED = 0
    ACCEPTED = 1
    ACK_REJECTED = 2

class AdvanceAction(IntEnum):
    ADVANCE_UNSPECIFIED = 0
    ADVANCE = 1
    ADVANCE_ABORT = 2

class RunControlAction(IntEnum):
    RUN_CONTROL_UNSPECIFIED = 0
    PREPARE = 1
    START = 2
    PAUSE = 3
    RESUME = 4
    RUN_ABORT = 5

class RunState(IntEnum):
    RUN_UNKNOWN = 0
    IDLE = 1
    CONFIGURING = 2
    WARMING_UP = 3
    RUNNING = 4
    PAUSED = 5
    ABORTING = 6
    COMPLETED = 7
    RUN_ERROR = 8

class ComponentState(IntEnum):
    COMPONENT_UNKNOWN = 0
    STARTING = 1
    COMPONENT_READY_STATE = 2
    COMPONENT_RUNNING = 3
    COMPONENT_PAUSED = 4
    STOPPING = 5
    STOPPED = 6
    COMPONENT_ERROR = 7

class Severity(IntEnum):
    SEVERITY_UNKNOWN = 0
    DEBUG = 1
    INFO = 2
    WARN = 3
    ERROR = 4
    FATAL = 5

class ActionTaken(IntEnum):
    ACTION_NONE = 0
    ACKED = 1
    RESENT = 2
    ACTION_REJECTED = 3
    ABORTED = 4
    EMERGENCY_STOPPED = 5
    LOGGED_ONLY = 6

class EventType(IntEnum):
    EVENT_UNKNOWN = 0
    STATE_TRANSITION = 1
    WARMUP_PROGRESS = 2
    VALIDATION_REJECTED = 3
    TIMEOUT_DETECTED = 4
    NATIVE_FAILURE = 5
    SAFETY_EVENT = 6
    EVIDENCE_FAILURE = 7

class DetailKind(IntEnum):
    DETAIL_NONE = 0
    VALIDATION_DETAIL = 1
    TIMEOUT_DETAIL = 2
    NATIVE_DETAIL = 3
    TRANSPORT_DETAIL = 4
    SAFETY_DETAIL = 5
    WARMUP_PROGRESS_DETAIL = 6

class TimingSegment(IntEnum):
    TIMING_UNKNOWN = 0
    RELAY = 1
    STACK = 2
    CONVERT = 3
    ASSEMBLY = 4
    PUBLISH_CALL = 5
    FRAME_INPUT_WAIT = 6
    NATIVE_APPLY = 7
    NATIVE_TICK = 8
    STATE_PUBLISH = 9
    COMPLETE_ACK_WAIT = 10

class TransportProfile(IntEnum):
    TRANSPORT_UNSPECIFIED = 0
    INPROC_TEST = 1
    LOCAL_HYBRID = 2
    ROS2_REFERENCE = 3

class PlantType(IntEnum):
    PLANT_UNSPECIFIED = 0
    VIRTUAL = 1
    CHAMELEON = 2
    SPARK = 3
    THOR = 4

class NativeAdapterType(IntEnum):
    NATIVE_ADAPTER_UNSPECIFIED = 0
    CARLA = 1
    MORAI = 2
    AURELION = 3

class TrafficEngineType(IntEnum):
    TRAFFIC_ENGINE_UNSPECIFIED = 0
    SUMO_TRACI = 1
    TERASIM = 2
    MOSS = 3

class DeterminismCapability(IntEnum):
    DETERMINISM_UNKNOWN = 0
    BIT_EXACT = 1
    NUMERIC_TOLERANCE = 2
    NON_DETERMINISTIC = 3

@dataclass(slots=True)
class OptionalBoundedId:
    has_value: bool
    value: str

@dataclass(slots=True)
class OptionalUuid128:
    has_value: bool
    value: bytes

@dataclass(slots=True)
class OptionalUint64:
    has_value: bool
    value: int

@dataclass(slots=True)
class OptionalHash256:
    has_value: bool
    value: bytes

@dataclass(slots=True)
class SemanticVersion:
    major: int
    minor: int
    patch: int

@dataclass(slots=True)
class ContentRef:
    id: str
    sha256: bytes

@dataclass(slots=True)
class Vector3d:
    x: float
    y: float
    z: float

@dataclass(slots=True)
class Quaterniond:
    x: float
    y: float
    z: float
    w: float

@dataclass(slots=True)
class Dimensions3d:
    length_m: float
    width_m: float
    height_m: float

@dataclass(slots=True)
class OptionalDimensions3d:
    has_value: bool
    value: Dimensions3d

@dataclass(slots=True)
class Pose3d:
    position_m: Vector3d
    orientation_xyzw: Quaterniond

@dataclass(slots=True)
class Twist3d:
    linear_mps: Vector3d
    angular_rad_s: Vector3d

@dataclass(slots=True)
class Covariance6x6:
    values: tuple[float, ...]

@dataclass(slots=True)
class NativeFrameRef:
    native_session_id: bytes
    native_generation: int
    native_frame_id: str

@dataclass(slots=True)
class ControlSetMeta:
    version: int
    digest: bytes
    expected_ego_count: int
    expected_traffic_count: int

@dataclass(slots=True)
class CommonHeader:
    schema_major: int
    schema_minor: int
    run_id: bytes
    run_epoch: int
    scope_kind: ScopeKind
    sim_id: OptionalBoundedId
    ego_id: OptionalBoundedId
    producer_id: ComponentId
    producer_instance_id: bytes
    event_seq: int
    correlation_id: OptionalUuid128
    sim_time_ns: OptionalUint64
    wall_time_unix_ns: int
    payload_hash: bytes

@dataclass(slots=True)
class AdapterCapability:
    control_mode_mask: int
    supports_physics: bool
    supports_spawn: bool
    supports_despawn: bool
    max_actor_count: int

@dataclass(slots=True)
class SimInstanceProfile:
    sim_id: str
    adapter_type: NativeAdapterType
    adapter_version: str
    capability: AdapterCapability
    determinism: DeterminismCapability

@dataclass(slots=True)
class EgoProfile:
    ego_id: str
    source_stack: SourceStack
    plant_type: PlantType
    vehicle_profile_id: str

@dataclass(slots=True)
class TrafficProfile:
    engine_type: TrafficEngineType
    binary_version: str
    fixed_step_ns: int
    derived_seed: int
    demand_hash: bytes

@dataclass(slots=True)
class TimeoutConfig:
    ready_timeout_ns: int
    apply_timeout_ns: int
    tick_timeout_ns: int
    heartbeat_timeout_ns: int
    queue_timeout_ns: int
    publish_ack_timeout_ns: int

@dataclass(slots=True)
class LoggingProfile:
    critical_depth: int
    general_depth: int
    flush_period_ns: int
    retention_days: int

@dataclass(slots=True)
class SoftwareVersion:
    component_id: ComponentId
    semantic_version: SemanticVersion
    git_commit: str
    build_id: str
    image_digest: OptionalHash256

@dataclass(slots=True)
class DetailData:
    kind: DetailKind
    numeric0: int
    numeric1: int
    id0: OptionalBoundedId
    related_hash: OptionalHash256

@dataclass(slots=True)
class ActorState:
    actor_id: str
    actor_role: ActorRole
    control_owner: ControlOwner
    lifecycle: Lifecycle
    representation_level: RepresentationLevel
    class_id: ActorClass
    position_m: Vector3d
    orientation_xyzw: Quaterniond
    velocity_mps: Vector3d
    acceleration_mps2: Vector3d
    angular_velocity_rad_s: Vector3d
    dimensions_m: OptionalDimensions3d
    state_validity: StateValidity

@dataclass(slots=True)
class OptionalActorState:
    has_value: bool
    value: ActorState

@dataclass(slots=True)
class OptionalRepresentationLevel:
    has_value: bool
    value: RepresentationLevel

@dataclass(slots=True)
class ObservedObject:
    actor_id: str
    class_id: ActorClass
    pose: Pose3d
    twist: Twist3d
    pose_covariance: Covariance6x6
    twist_covariance: Covariance6x6
    state_validity: StateValidity

@dataclass(slots=True)
class PredictionPoint:
    relative_time_ns: int
    pose: Pose3d
    velocity_mps: Vector3d

@dataclass(slots=True)
class PredictedPath:
    actor_id: str
    path_id: int
    probability: float
    points: list[PredictionPoint]

@dataclass(slots=True)
class TickRefs:
    has_state_tick_id: bool
    state_tick_id: int
    has_based_on_tick_id: bool
    based_on_tick_id: int
    has_target_tick_id: bool
    target_tick_id: int
    has_decision_id: bool
    decision_id: bytes

@dataclass(slots=True)
class NeutralControl:
    control_mode: ControlMode
    valid_fields_mask: int
    steering_tire_angle_rad: float
    steering_tire_rotation_rate_rad_s: float
    velocity_mps: float
    acceleration_mps2: float
    jerk_mps3: float
    throttle: float
    brake: float
    gear: Gear
    hand_brake: bool

@dataclass(slots=True)
class OptionalNeutralControl:
    has_value: bool
    value: NeutralControl

@dataclass(slots=True)
class NpcControlItem:
    actor_id: str
    action: CommandAction
    status: ItemStatus
    control: OptionalNeutralControl
    failure_reason: int

@dataclass(slots=True)
class ActorLifecycleIntent:
    actor_id: str
    intent: LifecycleIntent
    initial_state: OptionalActorState
    representation_level: OptionalRepresentationLevel
    intent_retry_count: int
    reason_context: int

@dataclass(slots=True)
class CommandCounts:
    expected_ego: int
    received_ego: int
    valid_ego: int
    failed_ego: int
    expected_traffic: int
    received_traffic: int
    valid_traffic: int
    failed_traffic: int

@dataclass(slots=True)
class ActorReason:
    actor_id: str
    reason_code: int

@dataclass(slots=True)
class ComponentHealthIssue:
    component_id: ComponentId
    health_status: HealthStatus
    reason_code: int

@dataclass(slots=True)
class HealthSummary:
    overall_health: HealthStatus
    issues: list[ComponentHealthIssue]
    truncated: bool

@dataclass(slots=True)
class Warning:
    code: int
    data: DetailData
    detail_message: str

@dataclass(slots=True)
class RunManifest:
    header: CommonHeader
    manifest_version: SemanticVersion
    execution_mode: ExecutionMode
    fixed_step_ns: int
    warmup_ticks: int
    seed: int
    map_ref: ContentRef
    scenario_ref: ContentRef
    sim_instances: list[SimInstanceProfile]
    required_sim_ids: list[str]
    ego_profiles: list[EgoProfile]
    traffic_profile: TrafficProfile
    max_actor_count: int
    max_report_count: int
    max_prediction_points: int
    deadline_budget_ns: int
    timeouts: TimeoutConfig
    transport_profile: TransportProfile
    software_versions: list[SoftwareVersion]
    logging_profile: LoggingProfile

@dataclass(slots=True)
class ComponentReady:
    header: CommonHeader
    manifest_hash: bytes
    ready_status: ReadyStatus
    capability_digest: bytes
    software_version: SoftwareVersion
    reason_codes: list[int]

@dataclass(slots=True)
class RunControlCommand:
    header: CommonHeader
    command_id: bytes
    action: RunControlAction
    reason_code: int

@dataclass(slots=True)
class RunControlAck:
    header: CommonHeader
    command_id: bytes
    action: RunControlAction
    ack_status: AckStatus
    current_run_state: RunState
    reason_code: int

@dataclass(slots=True)
class WorldStateFrame:
    header: CommonHeader
    state_tick_id: int
    native_frame_ref: NativeFrameRef
    control_set_meta: ControlSetMeta
    expected_ego_ids: list[str]
    expected_traffic_actor_ids: list[str]
    actors: list[ActorState]
    backend_health: HealthStatus

@dataclass(slots=True)
class EgoObservationFrame:
    header: CommonHeader
    state_tick_id: int
    ego_state: ActorState
    gt_objects: list[ObservedObject]
    control_set_meta: ControlSetMeta
    perfect_prediction_enabled: bool
    predicted_paths: list[PredictedPath]
    deadline_budget_ns: int

@dataclass(slots=True)
class EgoControlCommand:
    header: CommonHeader
    based_on_tick_id: int
    target_tick_id: int
    source_stack: SourceStack
    control_time_sim_ns: int
    command_status: CommandStatus
    control: OptionalNeutralControl
    failure_reason: int

@dataclass(slots=True)
class NpcControlBatch:
    header: CommonHeader
    based_on_tick_id: int
    target_tick_id: int
    control_set_meta: ControlSetMeta
    items: list[NpcControlItem]
    lifecycle_intents: list[ActorLifecycleIntent]
    engine_step_id: int
    engine_sim_time_ns: int
    engine_step_duration_ns: int

@dataclass(slots=True)
class CommandSetReady:
    header: CommonHeader
    target_tick_id: int
    counts: CommandCounts
    control_set_meta: ControlSetMeta
    completeness: Completeness
    validity: Validity
    missing_actor_ids: list[str]
    actor_reasons: list[ActorReason]
    reports_truncated: bool
    health_summary: HealthSummary
    snapshot_hash: bytes

@dataclass(slots=True)
class AdvanceFrame:
    header: CommonHeader
    decision_id: bytes
    target_tick_id: int
    snapshot_hash: bytes
    action: AdvanceAction
    abort_reason_code: int

@dataclass(slots=True)
class FrameComplete:
    header: CommonHeader
    state_tick_id: int
    decision_id: bytes
    applied_snapshot_hash: bytes
    native_frame_ref: NativeFrameRef
    tick_duration_ns: int
    control_set_meta: ControlSetMeta
    warnings: list[Warning]

@dataclass(slots=True)
class FrameCompleteAck:
    header: CommonHeader
    decision_id: bytes
    state_tick_id: int
    applied_snapshot_hash: bytes
    ack_status: AckStatus
    reason_code: int

@dataclass(slots=True)
class ComponentHeartbeat:
    header: CommonHeader
    component_state: ComponentState
    health_status: HealthStatus
    last_state_tick_id: OptionalUint64
    last_target_tick_id: OptionalUint64
    native_session_id: OptionalUuid128
    monotonic_time_ns: int

@dataclass(slots=True)
class RejectNotice:
    header: CommonHeader
    target_component_id: ComponentId
    rejected_message_kind: int
    related_hash: bytes
    tick_refs: TickRefs
    reason_code: int
    retryable: bool
    detail_data: DetailData
    detail_message: str

@dataclass(slots=True)
class StatusEvent:
    header: CommonHeader
    subject_component_id: ComponentId
    event_type: EventType
    severity: Severity
    reason_code: int
    action_taken: ActionTaken
    run_state: RunState
    tick_refs: TickRefs
    detail_data: DetailData
    detail_message: str

@dataclass(slots=True)
class TimingEvent:
    header: CommonHeader
    segment: TimingSegment
    start_monotonic_ns: int
    end_monotonic_ns: int
    duration_ns: int
    monotonic_clock_domain_id: str
    tick_refs: TickRefs
