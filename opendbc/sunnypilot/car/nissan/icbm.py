"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase
from opendbc.car.nissan import nissancan

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: "RES_BUTTON",
  SendButtonState.decrease: "SET_BUTTON",
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    # Keep track of the Counter field in the cruise throttle message to decide when to send the button
    self.cc_counter = 0
    self.last_cc_counter = 0

  def update(self, CS, CC_SP, packer, frame, last_button_frame) -> list[CanData]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame
    self.cc_counter = CS.cruise_throttle_msg["COUNTER"]

    if self.ICBM.sendButton != SendButtonState.none:
        send_button = BUTTONS[self.ICBM.sendButton]

        if (self.frame - self.last_button_frame) * DT_CTRL >= 0.13:

          if self.cc_counter == self.last_cc_counter:
            can_sends.append(nissancan.create_cruise_throttle_button(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, send_button, 1))
          else:
            can_sends.append(nissancan.create_cruise_throttle_button(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, send_button, 0))

          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.2:
            self.last_button_frame = self.frame
        else: # Maybe this will sync the car and the panda better
          if self.cc_counter == self.last_cc_counter:
            can_sends.append(nissancan.create_cruise_throttle_button(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, "NO_BUTTON_PRESSED", 1))
          else:
            can_sends.append(nissancan.create_cruise_throttle_button(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, "NO_BUTTON_PRESSED", 0))

    self.last_cc_counter = self.cc_counter
    return can_sends
