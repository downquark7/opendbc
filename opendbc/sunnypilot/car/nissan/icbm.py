"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: "RES_BUTTON",
  SendButtonState.decrease: "SET_BUTTON",
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self._spam_end_frame: int = 0
    self._spam_button: str | None = None
    self._spam_counter_offset: int | None = None

  def create_can_mock_button_messages(self, packer, CS, send_button: str) -> list[CanData]:
    can_sends: list[CanData] = []

    if (self.frame - self.last_button_frame) * DT_CTRL > 0.05:

      for button_counter_offset in range(0, 4):
        from opendbc.car.nissan import nissancan
        can_sends.append(nissancan.create_cruise_throttle_button(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, send_button, button_counter_offset))

      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
        self.last_button_frame = self.frame

    return can_sends

  def update(self, CS, CC_SP, packer, frame, last_button_frame) -> list[CanData]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none:
      send_button = BUTTONS[self.ICBM.sendButton]
      can_sends.extend(self.create_can_mock_button_messages(packer, CS, send_button))

    return can_sends
