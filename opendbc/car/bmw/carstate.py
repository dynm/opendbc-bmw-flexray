from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.sunnypilot.car.bmw.mads import MadsCarState


class CarState(CarStateBase, MadsCarState):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
    # Use CarStateBase.out / out_sp as rolling previous-state buffers
  @staticmethod
  def get_can_parsers(CP, CP_SP):
    cp_main = CANParser("bmw_sp2018", [("vehicle_speed", float("nan")), ("EPS_Angle", float("nan"))], bus=0)
    # One-time DBC config; avoid doing this in the control loop
    cp_main.dbc.name_to_msg["vehicle_speed"].ignore_checksum = True
    cp_main.dbc.name_to_msg["EPS_Angle"].ignore_checksum = True
    cp_main.dbc.name_to_msg["vehicle_speed"].ignore_counter = True
    cp_main.dbc.name_to_msg["EPS_Angle"].ignore_counter = True

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
    ret.cruiseState.enabled = True
    ret.cruiseState.available = True

    # Update MADS state (exposes cruise availability for lateral-only enable)
    MadsCarState.update_mads(self, ret, can_parsers)

    return ret, ret_sp


