from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.sunnypilot.car.bmw.mads import MadsCarState


class CarState(CarStateBase, MadsCarState):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return {
      Bus.main: CANParser("bmw_sp2018", [], bus=0),
      Bus.adas: CANParser("bmw_sp2018", [], bus=1),
    }

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.main]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # fl = cp.vl["wheel_speed"].get("FL", 0.0)
    # fr = cp.vl["wheel_speed"].get("FR", 0.0)
    # rl = cp.vl["wheel_speed"].get("RL", 0.0)
    # rr = cp.vl["wheel_speed"].get("RR", 0.0)
    # self.parse_wheel_speeds(ret, fl, fr, rl, rr, CV.KPH_TO_MS)

    ret.vEgoRaw = cp.vl["NEW_MSG_37"]["veh_speed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.steeringAngleDeg = float(cp.vl["EPS_Angle"]["steering_angle"])

    if "NEW_MSG_37" in cp.vl and "standstill" in cp.vl["NEW_MSG_37"]:
      ret.standstill = bool(cp.vl["NEW_MSG_37"]["standstill"])
    else:
      ret.standstill = ret.vEgo < 0.01

    # Update MADS state (exposes cruise availability for lateral-only enable)
    MadsCarState.update_mads(self, ret, can_parsers)

    return ret, ret_sp


