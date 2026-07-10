"""Pure-Python generators for simplified fake C-ITS messages.

No rclpy dependency here on purpose: this module can be unit-tested with a
plain ``python3`` interpreter, independent of a ROS 2 install.

Message schemas are deliberately simplified (not ASN.1/UPER-encoded ETSI
messages) but keep field names close to the real CAM/DENM/SPAT/MAP concepts
so a downstream Adapter node has an easy, well-known mapping to Autoware
Perception/Planning inputs.
"""

import math


EARTH_RADIUS_M = 6371000.0


def meters_to_deg_lat(meters):
    return meters / 111320.0


def meters_to_deg_lon(meters, at_lat_deg):
    return meters / (111320.0 * math.cos(math.radians(at_lat_deg)))


def destination_point(lat_deg, lon_deg, bearing_deg, distance_m):
    """Small-scale flat-earth approximation, adequate for a local intersection."""
    bearing_rad = math.radians(bearing_deg)
    d_lat = meters_to_deg_lat(distance_m * math.cos(bearing_rad))
    d_lon = meters_to_deg_lon(distance_m * math.sin(bearing_rad), lat_deg)
    return lat_deg + d_lat, lon_deg + d_lon


def bearing_between(lat1, lon1, lat2, lon2):
    return math.degrees(math.atan2(lon2 - lon1, lat2 - lat1)) % 360.0


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (math.sin(d_phi / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(d_lambda / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


# Approach directions for the fake 4-way intersection, as compass bearings
# pointing outward from the intersection reference point.
_APPROACH_BEARINGS = {
    'north': 0.0,
    'east': 90.0,
    'south': 180.0,
    'west': 270.0,
}

# signal_group_id 1 controls north/south vehicle movement, 2 controls east/west.
_APPROACH_SIGNAL_GROUP = {
    'north': 1,
    'south': 1,
    'east': 2,
    'west': 2,
}


class RsuSimulator:
    """Holds fake-RSU configuration and produces C-ITS message dicts.

    All generate_* methods are pure functions of a caller-supplied
    ``now`` (unix seconds, e.g. from ``time.time()``); no hidden internal
    clock, so behavior is deterministic and easy to unit test.
    """

    def __init__(
        self,
        station_id=9000,
        intersection_id=1,
        ref_lat=37.5665,
        ref_lon=126.9780,
        lane_length_m=60.0,
        vehicle_speed_mps=8.0,
        vehicle_length_m=4.5,
        vehicle_width_m=1.9,
        green_s=15.0,
        yellow_s=3.0,
        all_red_s=2.0,
        denm_interval_s=30.0,
        denm_duration_s=10.0,
        denm_cause='roadworks',
        denm_approach='north',
        denm_distance_m=25.0,
        cam_approach='south',
        cam_station_id=42,
    ):
        self.station_id = station_id
        self.intersection_id = intersection_id
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.lane_length_m = lane_length_m
        self.vehicle_speed_mps = vehicle_speed_mps
        self.vehicle_length_m = vehicle_length_m
        self.vehicle_width_m = vehicle_width_m
        self.green_s = green_s
        self.yellow_s = yellow_s
        self.all_red_s = all_red_s
        self.denm_interval_s = denm_interval_s
        self.denm_duration_s = denm_duration_s
        self.denm_cause = denm_cause
        self.denm_approach = denm_approach
        self.denm_distance_m = denm_distance_m
        self.cam_approach = cam_approach
        self.cam_station_id = cam_station_id

        self._map_cache = None

    # ------------------------------------------------------------------ MAP
    def generate_map(self):
        if self._map_cache is not None:
            return self._map_cache

        lanes = []
        lane_id = 1
        for approach, bearing in _APPROACH_BEARINGS.items():
            outer_lat, outer_lon = destination_point(
                self.ref_lat, self.ref_lon, bearing, self.lane_length_m)
            # inbound lane: from outer point toward the intersection
            lanes.append({
                'lane_id': lane_id,
                'approach': approach,
                'direction': 'inbound',
                'signal_group_id': _APPROACH_SIGNAL_GROUP[approach],
                'type': 'vehicle',
                'points': [[outer_lat, outer_lon], [self.ref_lat, self.ref_lon]],
            })
            lane_id += 1
            # outbound lane: from the intersection back out (opposite approach)
            lanes.append({
                'lane_id': lane_id,
                'approach': approach,
                'direction': 'outbound',
                'signal_group_id': None,
                'type': 'vehicle',
                'points': [[self.ref_lat, self.ref_lon], [outer_lat, outer_lon]],
            })
            lane_id += 1

        self._map_cache = {
            'msg_type': 'MAP',
            'intersection_id': self.intersection_id,
            'ref_position': {'lat': self.ref_lat, 'lon': self.ref_lon},
            'lanes': lanes,
        }
        return self._map_cache

    # ----------------------------------------------------------------- SPAT
    def _phase_schedule(self):
        """Six-step cycle: NS green/yellow/all-red, then EW green/yellow/all-red."""
        return [
            (self.green_s, {1: 'green', 2: 'red'}),
            (self.yellow_s, {1: 'yellow', 2: 'red'}),
            (self.all_red_s, {1: 'red', 2: 'red'}),
            (self.green_s, {1: 'red', 2: 'green'}),
            (self.yellow_s, {1: 'red', 2: 'yellow'}),
            (self.all_red_s, {1: 'red', 2: 'red'}),
        ]

    def _phase_at(self, now):
        schedule = self._phase_schedule()
        cycle_len = sum(duration for duration, _ in schedule)
        t = now % cycle_len
        for duration, states in schedule:
            if t < duration:
                return states, duration - t
            t -= duration
        # Should not happen, but fall back to the last phase.
        duration, states = schedule[-1]
        return states, duration

    def generate_spat(self, now):
        states, remaining_s = self._phase_at(now)
        signal_groups = []
        for group_id, state in states.items():
            signal_groups.append({
                'signal_group_id': group_id,
                'state': state,
                'min_end_time_s': round(remaining_s, 1),
                'max_end_time_s': round(remaining_s, 1),
                'likely_end_time_s': round(remaining_s, 1),
            })
        return {
            'msg_type': 'SPAT',
            'intersection_id': self.intersection_id,
            'timestamp_unix': now,
            'signal_groups': signal_groups,
        }

    # ------------------------------------------------------------------ CAM
    def generate_cam(self, now):
        bearing = _APPROACH_BEARINGS[self.cam_approach]
        outer_lat, outer_lon = destination_point(
            self.ref_lat, self.ref_lon, bearing, self.lane_length_m)
        travel_time_s = self.lane_length_m / self.vehicle_speed_mps
        cycle = 2 * travel_time_s
        t = now % cycle
        if t <= travel_time_s:
            # inbound: outer point -> intersection
            frac = t / travel_time_s
            lat = outer_lat + (self.ref_lat - outer_lat) * frac
            lon = outer_lon + (self.ref_lon - outer_lon) * frac
            heading = bearing_between(outer_lat, outer_lon, self.ref_lat, self.ref_lon)
        else:
            # outbound: intersection -> outer point
            frac = (t - travel_time_s) / travel_time_s
            lat = self.ref_lat + (outer_lat - self.ref_lat) * frac
            lon = self.ref_lon + (outer_lon - self.ref_lon) * frac
            heading = bearing_between(self.ref_lat, self.ref_lon, outer_lat, outer_lon)

        return {
            'msg_type': 'CAM',
            'station_id': self.cam_station_id,
            'station_type': 'vehicle',
            'gen_time_unix': now,
            'position': {'lat': lat, 'lon': lon, 'altitude': 0.0},
            'heading_deg': round(heading, 1),
            'speed_mps': self.vehicle_speed_mps,
            'vehicle_length_m': self.vehicle_length_m,
            'vehicle_width_m': self.vehicle_width_m,
        }

    # ----------------------------------------------------------------- DENM
    def generate_denm(self, now):
        """Returns None while no hazard is active, matching real DENM
        semantics of only transmitting while an event is in effect."""
        cycle_index = int(now // self.denm_interval_s)
        t_in_cycle = now % self.denm_interval_s
        if t_in_cycle >= self.denm_duration_s:
            return None

        bearing = _APPROACH_BEARINGS[self.denm_approach]
        lat, lon = destination_point(
            self.ref_lat, self.ref_lon, bearing, self.denm_distance_m)

        return {
            'msg_type': 'DENM',
            'station_id': self.station_id,
            'event_id': cycle_index,
            'detection_time_unix': now,
            'cause_code': self.denm_cause,
            'position': {'lat': lat, 'lon': lon},
            'event_radius_m': 15.0,
            'validity_duration_s': int(self.denm_duration_s - t_in_cycle),
            'is_active': True,
        }
