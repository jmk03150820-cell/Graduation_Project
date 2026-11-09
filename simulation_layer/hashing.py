"""Per-message canonical payload encoders (§4.6) on top of avva_hash_v1.

Field order follows interfaces/schema/avva_phase1.idl exactly, CommonHeader
excluded. Enums are u8, MessageKind/ReasonCode u16, per the golden vectors.

Only the 8 messages on the Simulation Layer data path are implemented.

NeutralControl: a numeric field whose valid_fields_mask bit is 0 is not
hashed. Confirmed (not just README rule 5) by the 구현기준서 §6.6 표34:
"mask가 유일한 숫자 필드 존재 판정이다. bit=0 값은 0-normalized하고 읽거나
hash하지 않는다" — the mask itself is the presence flag, so no extra Bool8
is written for these fields. This is a documented exception to the general
"모든 application field 포함" rule (§4.6), scoped specifically to
NeutralControl's masked fields. No golden vector covers this yet, but the
behavior itself is settled, not an open question.
"""
from __future__ import annotations

from avva_hash_v1 import CanonicalWriter, payload_bytes, sha256
import avva_phase1 as m

# valid_fields_mask bit -> NeutralControl numeric/enum field, in IDL order
_NC_MASKED_FIELDS = (
    (0, "steering_tire_angle_rad", "f64"),
    (1, "steering_tire_rotation_rate_rad_s", "f64"),
    (2, "velocity_mps", "f64"),
    (3, "acceleration_mps2", "f64"),
    (4, "jerk_mps3", "f64"),
    (5, "throttle", "f64"),
    (6, "brake", "f64"),
    (7, "gear", "u8"),
    (8, "hand_brake", "bool8"),
)


def _vec3(w: CanonicalWriter, v: m.Vector3d) -> None:
    w.f64(v.x).f64(v.y).f64(v.z)


def _quat(w: CanonicalWriter, q: m.Quaterniond) -> None:
    w.f64(q.x).f64(q.y).f64(q.z).f64(q.w)


def _pose(w: CanonicalWriter, p: m.Pose3d) -> None:
    _vec3(w, p.position_m); _quat(w, p.orientation_xyzw)


def _cov36(w: CanonicalWriter, c: m.Covariance6x6) -> None:
    if len(c.values) != 36:
        raise ValueError("Covariance6x6 must have 36 values")
    for v in c.values:
        w.f64(v)


def _native_frame_ref(w: CanonicalWriter, r: m.NativeFrameRef) -> None:
    w.uuid128(r.native_session_id).u32(r.native_generation).bounded_id(r.native_frame_id)


def _control_set_meta(w: CanonicalWriter, c: m.ControlSetMeta) -> None:
    w.u64(c.version).hash256(c.digest).u16(c.expected_ego_count).u32(c.expected_traffic_count)


def _dimensions(w: CanonicalWriter, d: m.Dimensions3d) -> None:
    w.f64(d.length_m).f64(d.width_m).f64(d.height_m)


def _actor_state(w: CanonicalWriter, a: m.ActorState) -> None:
    w.bounded_id(a.actor_id).u8(a.actor_role).u8(a.control_owner).u8(a.lifecycle)
    w.u8(a.representation_level).u8(a.class_id)
    _vec3(w, a.position_m); _quat(w, a.orientation_xyzw)
    _vec3(w, a.velocity_mps); _vec3(w, a.acceleration_mps2); _vec3(w, a.angular_velocity_rad_s)
    w.bool8(a.dimensions_m.has_value)
    if a.dimensions_m.has_value:
        _dimensions(w, a.dimensions_m.value)
    w.u8(a.state_validity)


def _neutral_control(w: CanonicalWriter, c: m.NeutralControl) -> None:
    w.u8(c.control_mode).u32(c.valid_fields_mask)
    for bit, name, kind in _NC_MASKED_FIELDS:
        if not c.valid_fields_mask & (1 << bit):
            continue
        v = getattr(c, name)
        if kind == "f64":
            w.f64(v)
        elif kind == "u8":
            w.u8(v)
        else:
            w.bool8(v)


def _opt_neutral_control(w: CanonicalWriter, o: m.OptionalNeutralControl) -> None:
    w.bool8(o.has_value)
    if o.has_value:
        _neutral_control(w, o.value)


def _npc_item(w: CanonicalWriter, i: m.NpcControlItem) -> None:
    w.bounded_id(i.actor_id).u8(i.action).u8(i.status)
    _opt_neutral_control(w, i.control)
    w.u16(i.failure_reason)


def _lifecycle_intent(w: CanonicalWriter, li: m.ActorLifecycleIntent) -> None:
    w.bounded_id(li.actor_id).u8(li.intent)
    w.bool8(li.initial_state.has_value)
    if li.initial_state.has_value:
        _actor_state(w, li.initial_state.value)
    w.bool8(li.representation_level.has_value)
    if li.representation_level.has_value:
        w.u8(li.representation_level.value)
    w.u16(li.intent_retry_count).u16(li.reason_context)


def _observed_object(w: CanonicalWriter, o: m.ObservedObject) -> None:
    w.bounded_id(o.actor_id).u8(o.class_id)
    _pose(w, o.pose)
    _vec3(w, o.twist.linear_mps); _vec3(w, o.twist.angular_rad_s)
    _cov36(w, o.pose_covariance); _cov36(w, o.twist_covariance)
    w.u8(o.state_validity)


def _prediction_point(w: CanonicalWriter, p: m.PredictionPoint) -> None:
    w.u64(p.relative_time_ns); _pose(w, p.pose); _vec3(w, p.velocity_mps)


def _predicted_path(w: CanonicalWriter, p: m.PredictedPath) -> None:
    w.bounded_id(p.actor_id).u16(p.path_id).f64(p.probability)
    w.sequence(p.points, _prediction_point, 32)


def _software_version(w: CanonicalWriter, sv: m.SoftwareVersion) -> None:
    w.u8(sv.component_id)
    w.u16(sv.semantic_version.major).u16(sv.semantic_version.minor).u16(sv.semantic_version.patch)
    w.bounded_id(sv.git_commit).bounded_id(sv.build_id)
    w.bool8(sv.image_digest.has_value)
    if sv.image_digest.has_value:
        w.hash256(sv.image_digest.value)


def _command_counts(w: CanonicalWriter, c: m.CommandCounts) -> None:
    w.u16(c.expected_ego).u16(c.received_ego).u16(c.valid_ego).u16(c.failed_ego)
    w.u32(c.expected_traffic).u32(c.received_traffic).u32(c.valid_traffic).u32(c.failed_traffic)


def _actor_reason(w: CanonicalWriter, r: m.ActorReason) -> None:
    w.bounded_id(r.actor_id).u16(r.reason_code)


def _health_summary(w: CanonicalWriter, h: m.HealthSummary) -> None:
    w.u8(h.overall_health)
    w.sequence(h.issues, lambda ww, i: ww.u8(i.component_id).u8(i.health_status).u16(i.reason_code), 32)
    w.bool8(h.truncated)


def _detail_data(w: CanonicalWriter, d: m.DetailData) -> None:
    w.u8(d.kind).i64(d.numeric0).i64(d.numeric1)
    w.bool8(d.id0.has_value)
    if d.id0.has_value:
        w.bounded_id(d.id0.value)
    w.bool8(d.related_hash.has_value)
    if d.related_hash.has_value:
        w.hash256(d.related_hash.value)


def _warning(w: CanonicalWriter, wa: m.Warning) -> None:
    # detail_message is excluded from hashes by contract (§4.6)
    w.u16(wa.code); _detail_data(w, wa.data)


# ---- message payload hashes (header excluded) ------------------------------

def world_state_frame_hash(msg: m.WorldStateFrame) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.u64(msg.state_tick_id)
        _native_frame_ref(w, msg.native_frame_ref)
        _control_set_meta(w, msg.control_set_meta)
        w.sequence(msg.expected_ego_ids, lambda ww, s: ww.bounded_id(s), 8)
        w.sequence(msg.expected_traffic_actor_ids, lambda ww, s: ww.bounded_id(s), 1200)
        w.sequence(msg.actors, _actor_state, 1200)
        w.u8(msg.backend_health)
    return sha256(payload_bytes(m.WORLD_STATE_FRAME, body))


def ego_observation_frame_hash(msg: m.EgoObservationFrame) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.u64(msg.state_tick_id)
        _actor_state(w, msg.ego_state)
        w.sequence(msg.gt_objects, _observed_object, 1200)
        _control_set_meta(w, msg.control_set_meta)
        w.bool8(msg.perfect_prediction_enabled)
        w.sequence(msg.predicted_paths, _predicted_path, 1200)
        w.u64(msg.deadline_budget_ns)
    return sha256(payload_bytes(m.EGO_OBSERVATION_FRAME, body))


def ego_control_command_hash(msg: m.EgoControlCommand) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.u64(msg.based_on_tick_id).u64(msg.target_tick_id)
        w.u8(msg.source_stack).u64(msg.control_time_sim_ns).u8(msg.command_status)
        _opt_neutral_control(w, msg.control)
        w.u16(msg.failure_reason)
    return sha256(payload_bytes(m.EGO_CONTROL_COMMAND, body))


def npc_control_batch_hash(msg: m.NpcControlBatch) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.u64(msg.based_on_tick_id).u64(msg.target_tick_id)
        _control_set_meta(w, msg.control_set_meta)
        w.sequence(msg.items, _npc_item, 1200)
        w.sequence(msg.lifecycle_intents, _lifecycle_intent, 1200)
        w.u64(msg.engine_step_id).u64(msg.engine_sim_time_ns).u64(msg.engine_step_duration_ns)
    return sha256(payload_bytes(m.NPC_CONTROL_BATCH, body))


def command_set_ready_hash(msg: m.CommandSetReady) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.u64(msg.target_tick_id)
        _command_counts(w, msg.counts)
        _control_set_meta(w, msg.control_set_meta)
        w.u8(msg.completeness).u8(msg.validity)
        w.sequence(msg.missing_actor_ids, lambda ww, s: ww.bounded_id(s), 32)
        w.sequence(msg.actor_reasons, _actor_reason, 32)
        w.bool8(msg.reports_truncated)
        _health_summary(w, msg.health_summary)
        w.hash256(msg.snapshot_hash)
    return sha256(payload_bytes(m.COMMAND_SET_READY, body))


def advance_frame_hash(msg: m.AdvanceFrame) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.uuid128(msg.decision_id).u64(msg.target_tick_id).hash256(msg.snapshot_hash)
        w.u8(msg.action).u16(msg.abort_reason_code)
    return sha256(payload_bytes(m.ADVANCE_FRAME, body))


def frame_complete_hash(msg: m.FrameComplete) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.u64(msg.state_tick_id).uuid128(msg.decision_id).hash256(msg.applied_snapshot_hash)
        _native_frame_ref(w, msg.native_frame_ref)
        w.u64(msg.tick_duration_ns)
        _control_set_meta(w, msg.control_set_meta)
        w.sequence(msg.warnings, _warning, 32)
    return sha256(payload_bytes(m.FRAME_COMPLETE, body))


def frame_complete_ack_hash(msg: m.FrameCompleteAck) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.uuid128(msg.decision_id).u64(msg.state_tick_id).hash256(msg.applied_snapshot_hash)
        w.u8(msg.ack_status).u16(msg.reason_code)
    return sha256(payload_bytes(m.FRAME_COMPLETE_ACK, body))


def component_ready_hash(msg: m.ComponentReady) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.hash256(msg.manifest_hash).u8(msg.ready_status).hash256(msg.capability_digest)
        _software_version(w, msg.software_version)
        w.sequence(msg.reason_codes, lambda ww, v: ww.u16(v), 32)
    return sha256(payload_bytes(m.COMPONENT_READY, body))


def run_control_ack_hash(msg: m.RunControlAck) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.uuid128(msg.command_id).u8(msg.action).u8(msg.ack_status)
        w.u8(msg.current_run_state).u16(msg.reason_code)
    return sha256(payload_bytes(m.RUN_CONTROL_ACK, body))


def component_heartbeat_hash(msg: m.ComponentHeartbeat) -> bytes:
    def body(w: CanonicalWriter) -> None:
        w.u8(msg.component_state).u8(msg.health_status)
        w.bool8(msg.last_state_tick_id.has_value)
        if msg.last_state_tick_id.has_value:
            w.u64(msg.last_state_tick_id.value)
        w.bool8(msg.last_target_tick_id.has_value)
        if msg.last_target_tick_id.has_value:
            w.u64(msg.last_target_tick_id.value)
        w.bool8(msg.native_session_id.has_value)
        if msg.native_session_id.has_value:
            w.uuid128(msg.native_session_id.value)
        w.u64(msg.monotonic_time_ns)
    return sha256(payload_bytes(m.COMPONENT_HEARTBEAT, body))


def _tick_refs(w: CanonicalWriter, t: m.TickRefs) -> None:
    # TickRefs 필드는 OptionalXxx 타입이 아니라 평범한 has_X+X 쌍이지만, has_X가
    # presence flag 역할인 건 동일하다(§4.6 Optional 패턴과 동일 취급) — 4개 필드
    # 전부 동일하게 조건부 인코딩해야 한다. has_decision_id만 조건부, 나머지 3개는
    # 무조건 인코딩하던 이전 버전은 팀 다른 컴포넌트의 인코더와 canonical bytes가
    # 어긋날 수 있는 버그였음.
    w.bool8(t.has_state_tick_id)
    if t.has_state_tick_id:
        w.u64(t.state_tick_id)
    w.bool8(t.has_based_on_tick_id)
    if t.has_based_on_tick_id:
        w.u64(t.based_on_tick_id)
    w.bool8(t.has_target_tick_id)
    if t.has_target_tick_id:
        w.u64(t.target_tick_id)
    w.bool8(t.has_decision_id)
    if t.has_decision_id:
        w.uuid128(t.decision_id)


def reject_notice_hash(msg: m.RejectNotice) -> bytes:
    # detail_message is excluded from hashes by contract (§4.6), same as Warning
    def body(w: CanonicalWriter) -> None:
        w.u8(msg.target_component_id).u16(msg.rejected_message_kind).hash256(msg.related_hash)
        _tick_refs(w, msg.tick_refs)
        w.u16(msg.reason_code).bool8(msg.retryable)
        _detail_data(w, msg.detail_data)
    return sha256(payload_bytes(m.REJECT_NOTICE, body))
