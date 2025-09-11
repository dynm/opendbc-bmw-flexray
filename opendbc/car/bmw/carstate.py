from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.sunnypilot.car.bmw.mads import MadsCarState


class CarState(CarStateBase, MadsCarState):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
    self._last_state = structs.CarState()
    self._last_state_sp = structs.CarStateSP()
  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return {
      Bus.main: CANParser("bmw_sp2018", [("vehicle_speed", float("nan")), ("EPS_Angle", float("nan"))], bus=0),
      # Bus.adas: CANParser("bmw_sp2018", [], bus=1),
    }

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.main]
    cp.dbc.name_to_msg["vehicle_speed"].ignore_checksum = True
    cp.dbc.name_to_msg["EPS_Angle"].ignore_checksum = True
    cp.dbc.name_to_msg["vehicle_speed"].ignore_counter = True
    cp.dbc.name_to_msg["EPS_Angle"].ignore_counter = True

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # fl = cp.vl["wheel_speed"].get("FL", 0.0)
    # fr = cp.vl["wheel_speed"].get("FR", 0.0)
    # rl = cp.vl["wheel_speed"].get("RL", 0.0)
    # rr = cp.vl["wheel_speed"].get("RR", 0.0)
    # self.parse_wheel_speeds(ret, fl, fr, rl, rr, CV.KPH_TO_MS)

    # Per-signal multiplexing gates
    veh_cc = cp.vl["vehicle_speed"].get("cycle_count", -1)
    eps_cc = cp.vl["EPS_Angle"].get("cycle_count", -1)
    veh_ok = (veh_cc >= 0) and (veh_cc % 4 == 3)
    eps_ok = (eps_cc >= 0) and (eps_cc % 2 == 0)

    # Speed update: only when vehicle_speed is on its valid cycle; otherwise reuse last values
    if veh_ok:
      ret.vEgoRaw = cp.vl["vehicle_speed"].get("veh_speed", 0.0) * CV.KPH_TO_MS
      ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    else:
      ret.vEgoRaw = getattr(self._last_state, "vEgoRaw", 0.0)
      ret.vEgo = getattr(self._last_state, "vEgo", 0.0)
      ret.aEgo = getattr(self._last_state, "aEgo", 0.0)

    # Steering angle update: only when EPS_Angle is on its valid cycle; otherwise reuse last value
    if eps_ok:
      ret.steeringAngleDeg = float(cp.vl["EPS_Angle"].get("steering_angle", 0.0))
    else:
      ret.steeringAngleDeg = float(getattr(self._last_state, "steeringAngleDeg", 0.0))

    ret.standstill = ret.vEgoRaw < 0.01

    ret.gearShifter = structs.CarState.GearShifter.drive
    ret.cruiseState.enabled = True
    ret.cruiseState.available = True

    # Update MADS state (exposes cruise availability for lateral-only enable)
    MadsCarState.update_mads(self, ret, can_parsers)

    self._last_state = ret
    self._last_state_sp = ret_sp
    return ret, ret_sp


