import numpy as np

from common.realtime import DT_CTRL
from selfdrive.car.body import bodycan
from selfdrive.car.body.body_mpc_lib.body_mpc import BodyMpc
from opendbc.can.packer import CANPacker
from selfdrive.car.body.values import SPEED_FROM_RPM
from selfdrive.controls.lib.pid import PIDController


MAX_TORQUE = 500
MAX_TORQUE_RATE = 50
MAX_ANGLE_ERROR = np.radians(7)
MAX_POS_INTEGRATOR = 0.2   # meters
MAX_TURN_INTEGRATOR = 0.1  # meters


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.frame = 0
    self.packer = CANPacker(dbc_name)

    # Speed, balance and turn PIDs
    self.speed_pid = PIDController(0.115, k_i=0.23, rate=1/DT_CTRL)
    self.body_mpc = BodyMpc()
    self.body_mpc.set_weights(1.0, 1.0, 1.0)

    self.turn_pid = PIDController(110, k_i=11.5, rate=1/DT_CTRL)

    self.torque_r_filtered = 0.
    self.torque_l_filtered = 0.

  @staticmethod
  def deadband_filter(torque, deadband):
    if torque > 0:
      torque += deadband
    else:
      torque -= deadband
    return torque

  def update(self, CC, CS):

    torque_l = 0
    torque_r = 0

    llk_valid = len(CC.orientationNED) > 0 and len(CC.angularVelocity) > 0
    if CC.enabled and llk_valid:
      # Read these from the joystick
      # TODO: this isn't acceleration, okay?
      #speed_desired = CC.actuators.accel / 5.
      speed_diff_desired = -CC.actuators.steer

      speed_measured = SPEED_FROM_RPM * (CS.out.wheelSpeeds.fl + CS.out.wheelSpeeds.fr) / 2.
      x0 = np.array([0.0, speed_measured, -CC.orientationNED[1], -CC.angularVelocity[1]])
      self.body_mpc.run(x0)


      # Clip angle error, this is enough to get up from stands
      torque_nm = self.body_mpc.u_sol[0,0]
      #torque = interp(torque_nm, TORQUE_BP, TORQUE_VAL)
      torque = 0.5 * (torque_nm * 50 / 0.44)
      print(torque)

      speed_diff_measured = SPEED_FROM_RPM * (CS.out.wheelSpeeds.fl - CS.out.wheelSpeeds.fr)
      turn_error = speed_diff_measured - speed_diff_desired
      freeze_integrator = ((turn_error < 0 and self.turn_pid.error_integral <= -MAX_POS_INTEGRATOR) or
                           (turn_error > 0 and self.turn_pid.error_integral >= MAX_POS_INTEGRATOR))
      torque_diff = self.turn_pid.update(turn_error, freeze_integrator=freeze_integrator)

      # Combine 2 PIDs outputs
      torque_r = torque + torque_diff
      torque_l = torque - torque_diff

      # Torque rate limits
      self.torque_r_filtered = np.clip(self.deadband_filter(torque_r, 10),
                                       self.torque_r_filtered - MAX_TORQUE_RATE,
                                       self.torque_r_filtered + MAX_TORQUE_RATE)
      self.torque_l_filtered = np.clip(self.deadband_filter(torque_l, 10),
                                       self.torque_l_filtered - MAX_TORQUE_RATE,
                                       self.torque_l_filtered + MAX_TORQUE_RATE)
      torque_r = int(np.clip(self.torque_r_filtered, -MAX_TORQUE, MAX_TORQUE))
      torque_l = int(np.clip(self.torque_l_filtered, -MAX_TORQUE, MAX_TORQUE))

    can_sends = []
    can_sends.append(bodycan.create_control(self.packer, torque_l, torque_r, self.frame // 2))

    new_actuators = CC.actuators.copy()
    new_actuators.accel = torque_l
    new_actuators.steer = torque_r

    self.frame += 1
    return new_actuators, can_sends
