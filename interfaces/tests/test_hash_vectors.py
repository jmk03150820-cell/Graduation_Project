import hashlib, json, sys, uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "generated/python"))
from avva_hash_v1 import CanonicalWriter, control_set_digest_bytes, frame_complete_ack_payload, snapshot_bytes

vectors = {v["name"]: v for v in json.loads((ROOT / "golden/avva_hash_v1.json").read_text())["vectors"]}

def check(name, data):
    v = vectors[name]
    assert data.hex() == v["canonical_hex"], (name, data.hex())
    assert hashlib.sha256(data).hexdigest() == v["sha256"]

run_id = uuid.UUID("550e8400-e29b-41d4-a716-446655440000")
check("bounded_id_ego_0", CanonicalWriter().bounded_id("ego_0").finish())
check("frame_complete_ack_payload", frame_complete_ack_payload(run_id, 1, bytes(range(32)), 1, 0))
control = control_set_digest_bytes([("ego_0",1,1,2,3), ("npc_1",2,2,2,2)])
check("control_set_digest", control)
check("snapshot_empty_inputs", snapshot_bytes(run_id,1,"carla_0",1,hashlib.sha256(control).digest(),[]))
print("4 golden vectors: OK")

