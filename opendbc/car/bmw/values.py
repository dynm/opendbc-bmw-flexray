from dataclasses import dataclass, field
from opendbc.car.docs_definitions import CarDocs

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms


@dataclass(frozen=True, kw_only=True)
class BMWCarSpecs(CarSpecs):
  mass: float = 1600.
  wheelbase: float = 2.82
  steerRatio: float = 15.0
  centerToFrontRatio: float = 0.45


@dataclass
class BMWPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'bmw_sp2018'})

@dataclass
class BMWCarDocs(CarDocs):
  name: str = "BMW SP2018"
  package: str = "5AU"
class CAR(Platforms):
  BMW_SP2018 = BMWPlatformConfig(
    [BMWCarDocs()],
    BMWCarSpecs(),
  )


DBC = CAR.create_dbc_map()


