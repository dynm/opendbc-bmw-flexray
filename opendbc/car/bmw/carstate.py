from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase


class CarState(CarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    # Use CarStateBase.out / out_sp as rolling previous-state buffers
  @staticmethod
  def get_can_parsers(CP, CP_SP):
    # External panda is index 1 -> buses 4-7. Use bus 4 for main traffic.
    cp_main = CANParser("bmw_sp2018", [("vehicle_speed", float("nan")), ("EPS_Angle", float("nan")), ("ACC", float("nan")), ("NEW_MSG_38", float("nan")), ("steer_torque", float("nan"))], bus=4)
    # One-time DBC config; avoid doing this in the control loop
    cp_main.dbc.name_to_msg["vehicle_speed"].ignore_checksum = True
    cp_main.dbc.name_to_msg["EPS_Angle"].ignore_checksum = True
    cp_main.dbc.name_to_msg["vehicle_speed"].ignore_counter = True
    cp_main.dbc.name_to_msg["EPS_Angle"].ignore_counter = True
    # ACC RX is currently synthetic; ignore checks until real CRC/counter implemented
    cp_main.dbc.name_to_msg["ACC"].ignore_checksum = True
    cp_main.dbc.name_to_msg["ACC"].ignore_counter = True
    # Yaw and driver torque parsing; ignore checks for now
    cp_main.dbc.name_to_msg["NEW_MSG_38"].ignore_checksum = True
    cp_main.dbc.name_to_msg["NEW_MSG_38"].ignore_counter = True
    cp_main.dbc.name_to_msg["steer_torque"].ignore_checksum = True
    cp_main.dbc.name_to_msg["steer_torque"].ignore_counter = True

    return {
      Bus.main: cp_main,
      # Bus.adas: CANParser("bmw_sp2018", [], bus=1),
    }

  def _demux_last(self, cp: CANParser, msg: str, cc_sig: str, val_sig: str, cycle_base: int) -> tuple[bool, float]:
    cc_list = cp.vl_all[msg].get(cc_sig, [])
    val_list = cp.vl_all[msg].get(val_sig, [])
    for i in range(len(cc_list) - 1, -1, -1):
      if int(cc_list[i]) == cycle_base:
        return True, float(val_list[i])
    return False, 0.0

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.main]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Previous state snapshot (avoids extra allocations and getattr fallback)
    prev = self.out

    # fl = cp.vl["wheel_speed"].get("FL", 0.0)
    # fr = cp.vl["wheel_speed"].get("FR", 0.0)
    # rl = cp.vl["wheel_speed"].get("RL", 0.0)
    # rr = cp.vl["wheel_speed"].get("RR", 0.0)
    # self.parse_wheel_speeds(ret, fl, fr, rl, rr, CV.KPH_TO_MS)

    # Batch demux using helper; BMW DBC uses fixed cycle codes
    veh_found, veh_speed_kph = self._demux_last(cp, "vehicle_speed", "cycle_count", "veh_speed", cycle_base=3)

    if veh_found:
      ret.vEgoRaw = veh_speed_kph * CV.KPH_TO_MS
      ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
      ret.vEgoCluster = ret.vEgoRaw
    else:
      ret.vEgoRaw = prev.vEgoRaw
      ret.vEgo = prev.vEgo
      ret.aEgo = prev.aEgo
      ret.vEgoCluster = float(prev.vEgoCluster)

    # Steering angle: choose cycle_count == 0 if present
    eps_found, eps_angle = self._demux_last(cp, "EPS_Angle", "cycle_count", "steering_angle", cycle_base=0)
    if eps_found:
      ret.steeringAngleDeg = float(eps_angle)
    else:
      ret.steeringAngleDeg = float(prev.steeringAngleDeg)

    ret.standstill = ret.vEgoRaw < 0.01

    ret.gearShifter = structs.CarState.GearShifter.drive
    # ACC assist_mode demux with cycle base 1
    acc_found, acc_assist_mode = self._demux_last(cp, "ACC", "cycle_count", "assist_mode", cycle_base=1)
    ret.cruiseState.enabled = bool(int(acc_assist_mode)) if acc_found else bool(prev.cruiseState.enabled)
    ret.cruiseState.available = True

    # Yaw rate (deg/s -> rad/s), demux cycle base 0
    yaw_found, yaw_deg_s = self._demux_last(cp, "NEW_MSG_38", "cycle_count", "yaw", cycle_base=0)
    if yaw_found:
      ret.yawRate = float(yaw_deg_s) * CV.DEG_TO_RAD
    else:
      ret.yawRate = float(prev.yawRate)

    # Driver steering torque (native units from CAN)
    steering_torque_found, steering_torque = self._demux_last(cp, "steer_torque", "cycle_count", "steering_torque", cycle_base=0)
    if steering_torque_found:
      ret.steeringTorque = float(steering_torque)
    else:
      ret.steeringTorque = float(prev.steeringTorque)

    return ret, ret_sp


