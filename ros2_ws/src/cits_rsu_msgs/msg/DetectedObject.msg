# Predicted C-ITS RSU detected-object schema, based on the field table the
# C-ITS teammate shared (object_id/object_type/position_x/position_y/
# velocity/timestamp/confidence/heading). Field names are snake_case per
# ROS 2 message style but otherwise map 1:1 to that table. Expect this to
# change once the teammate's real implementation solidifies.

# Known object_type codes. Only PERSON=1 was confirmed by the teammate;
# the rest are placeholder guesses pending confirmation.
uint8 UNKNOWN=0
uint8 PERSON=1
uint8 CAR=2
uint8 BICYCLE=3
uint8 MOTORCYCLE=4
uint8 TRUCK=5
uint8 BUS=6

uint32 object_id
uint8 object_type
float32 position_x      # BEV frame, meters, origin/axes TBD (assumed RSU-centered)
float32 position_y      # BEV frame, meters
float32 velocity        # m/s, scalar speed
uint64 timestamp        # assumed Unix epoch milliseconds
float32 confidence      # 0.0-1.0
float32 heading         # degrees, assumed BEV-frame angle (not compass bearing)
