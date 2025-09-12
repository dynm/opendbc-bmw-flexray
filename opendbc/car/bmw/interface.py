from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.bmw.carcontroller import CarController
from opendbc.car.bmw.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "bmw"

    # Safe default during bring-up; replace with proper BMW safety once ready
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput)]

    # Angle-only lateral; keep stock longitudinal
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = True

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = True
    ret.radarUnavailable = True

    # Basic PID for angle; tune later
    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kf = 0.00006
    ret.lateralTuning.pid.kpV = [0.6]
    ret.lateralTuning.pid.kiV = [0.2]
    return ret


