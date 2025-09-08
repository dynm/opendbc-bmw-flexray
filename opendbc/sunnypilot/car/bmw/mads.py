"""
Minimal MADS integration for BMW.

This enables MADS (lateral-only enable/disable flow) by ensuring
cruise availability is exposed to the stack. HUD/icons and
brand-specific toggles can be added later when signals are known.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.mads_base import MadsCarStateBase


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

  @staticmethod
  def get_parser(CP, CP_SP, pt_messages, cam_messages) -> None:
    # Placeholder for future BMW-specific MADS signals (e.g., LKAS button, HUD)
    # When signals are identified, append to pt_messages/cam_messages here.
    pass

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    # For lateral-only usage, exposing cruise availability is sufficient.
    # If a proper ACC main toggle signal is found later, set available based on that.
    if ret.cruiseState is None:
      ret.cruiseState = structs.CarState.CruiseState()

    # Default to available so MADS can be enabled; refine when ACC MAIN signal is mapped.
    ret.cruiseState.available = True


