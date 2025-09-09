from opendbc.can import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car import Bus


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.cnt = 0
    self.cycle = 0

  def _acc_cnt(self):
    self.cnt = (self.cnt + 1) % 16
    return self.cnt

  def _acc_cycle(self):
    # DBC shows a cycle_count; assume 2-bit rollover until specified otherwise
    self.cycle = (self.cycle + 1) % 4
    return self.cycle

  def _acc_crc(self, data_bytes: bytes) -> int:
    # TODO: implement real CRC. Placeholder returns 0.
    return 0

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    lat_active = bool(CC.latActive)
    desired_angle = float(actuators.steeringAngleDeg)

    if self.frame % 2 == 0:
      # Full ACC payload with defaults
      values = {
        "cycle_count": 1,
        "crc1": 0,
        "cnt1": 0,
        "always_0x9": 9,
        "steering_angle_req": desired_angle,
        "steer_torque_req": 0.0,
        "TJA_ready": 0,
        "assist_mode": 1 if lat_active else 0,
        "wayback_en1_lane_keeping_trigger": 0,
        "lane_keeping_triggered": 0,
        "like_assist_torque_reserve": 0xA0,
        "constants": 0x03ff17fe,
        "wayback_en_2": 0,
        "steering_engaged": 2,
        "maybe_assist_force_enhance": 0xA2,
        "maybe_assist_force_weaken": 0xFA,
      }

      # simple check on pico, will change later.
      frame_id = self.packer.dbc.name_to_msg.get("ACC").address
      values["cycle_count"] = 1 # use as cycle base
      values["crc1"] = frame_id & 0xFF
      values["cnt1"] = (frame_id >> 8) & 0b111

      msg = self.packer.make_can_msg("ACC", 0, values)
      can_sends.append(msg)

    self.frame += 1
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = desired_angle
    return new_actuators, can_sends


