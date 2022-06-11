#!/usr/bin/env python3
import time
from multiprocessing import Process, Queue
from typing import List, Optional

import numpy as np
from collections import defaultdict

from numpy.linalg import linalg

from cereal import log, messaging
from laika import AstroDog, constants
from laika.constants import SECS_IN_MIN
from laika.ephemeris import EphemerisType, convert_ublox_ephem
from laika.gps_time import GPSTime
from laika.helpers import ConstellationId
from laika.raw_gnss import GNSSMeasurement, calc_pos_fix, correct_measurements, get_DOP, process_measurements, read_raw_ublox
from selfdrive.locationd.models.constants import GENERATED_DIR, ObservationKind
from selfdrive.locationd.models.gnss_kf import GNSSKalman
from selfdrive.locationd.models.gnss_kf import States as GStates
import common.transformations.coordinates as coord
from selfdrive.swaglog import cloudlog

MAX_TIME_GAP = 10


class Laikad:

  def __init__(self, valid_const=("GPS", "GLONASS"), auto_update=False, valid_ephem_types=(EphemerisType.ULTRA_RAPID_ORBIT, EphemerisType.NAV)):
    self.astro_dog = AstroDog(valid_const=valid_const, auto_update=auto_update, valid_ephem_types=valid_ephem_types)
    self.gnss_kf = GNSSKalman(GENERATED_DIR)
    self.orbit_p: Optional[Process] = None
    self.orbit_q = Queue()
    self._first_correct_gps_message = None
    self.latest_pos_fix = None
    self.latest_pos_fix_time = None

  def get_kalman_pos(self, t):
    kf_pos_std = None
    if all(self.kf_valid(t)):
      self.gnss_kf.predict(t)
      kf_pos_std = np.sqrt(abs(self.gnss_kf.P[GStates.ECEF_POS].diagonal()))
    # If localizer is valid use its position to correct measurements
    if kf_pos_std is not None and linalg.norm(kf_pos_std) < 100:
      return self.gnss_kf.x[GStates.ECEF_POS]

  
  def get_pos_fix_func(self, t, processed_measurements):
    def get_pos_fix():
      if self.latest_pos_fix is None or abs(self.latest_pos_fix_time - t) >= 1:
        pos_fix = calc_pos_fix(processed_measurements, min_measurements=4)
        if len(pos_fix) > 0:
          self.latest_pos_fix = pos_fix
          self.latest_pos_fix_time = t
      return self.latest_pos_fix
    return get_pos_fix
  
  def process_ublox_msg(self, ublox_msg, ublox_mono_time: int, block=True):
    if ublox_msg.which == 'measurementReport':
      report = ublox_msg.measurementReport
      if report.gpsWeek > 0:
        if self._first_correct_gps_message is None:  # todo remove
          self._first_correct_gps_message = time.time()
        latest_msg_t = GPSTime(report.gpsWeek, report.rcvTow)
        self.fetch_orbits(latest_msg_t + SECS_IN_MIN, block)

      new_meas = read_raw_ublox(report)
      processed_measurements = process_measurements(new_meas, self.astro_dog)
      # todo temporary
      ephems_used = []
      for m in new_meas:
        sat_time = m.recv_time - m.observables['C1C'] / constants.SPEED_OF_LIGHT
        eph = None
        if self.astro_dog.pull_orbit:
          eph = self.astro_dog.get_orbit(m.prn, sat_time)
        if not eph and self.astro_dog.pull_nav:
          eph = self.astro_dog.get_nav(m.prn, sat_time)
        if eph:
          ephems_used.append(eph)
          
      corrected_measurements = []
      t = ublox_mono_time * 1e-9
      get_pos_fix = self.get_pos_fix_func(t, processed_measurements)
      est_pos = self.get_kalman_pos(t)
      if est_pos is None:
        pos_fix = get_pos_fix()
        if pos_fix is not None and abs(np.array(pos_fix[1])).mean() < 1000:
          est_pos = pos_fix[0][:3]
          
      if est_pos is not None:
        corrected_measurements = correct_measurements(processed_measurements, est_pos, self.astro_dog)

      self.update_localizer(get_pos_fix, t, corrected_measurements)
      kf_valid = all(self.kf_valid(t))

      ecef_pos = self.gnss_kf.x[GStates.ECEF_POS].tolist()
      ecef_vel = self.gnss_kf.x[GStates.ECEF_VELOCITY].tolist()

      pos_std = np.sqrt(abs(self.gnss_kf.P[GStates.ECEF_POS].diagonal())).tolist()
      vel_std = np.sqrt(abs(self.gnss_kf.P[GStates.ECEF_VELOCITY].diagonal())).tolist()

      bearing_deg, bearing_std = get_bearing_from_gnss(ecef_pos, ecef_vel, vel_std)

      meas_msgs = [create_measurement_msg(m) for m in corrected_measurements]
      dat = messaging.new_message("gnssMeasurements")
      measurement_msg = log.LiveLocationKalman.Measurement.new_message
      if kf_valid and self._first_correct_gps_message:
        # todo temp, remove
        cloudlog.info(f"Time until first fix after receiving first correct gps message: {time.time() - self._first_correct_gps_message:.2f}")
        self._first_correct_gps_message = False
      pos_fix_related = ''
      if self.latest_pos_fix is not None and abs(self.latest_pos_fix_time - t) < 1:
        dop = get_DOP(est_pos, [m.sat_pos for m in processed_measurements])
        pos_fix_related = f"diff kf pos vs fix {(ecef_pos - self.latest_pos_fix[0][:3]).round(1)} DOP {dop:.1f}"
      # todo cleanup
      cloudlog.error(
        f"incoming {len(new_meas)} processed {len(processed_measurements)} corrected {len(corrected_measurements)} types: {set([e.eph_type.name for e in ephems_used])}, localizer_valid {kf_valid}" +
        f" pos_std {linalg.norm(pos_std):.1f} c_ids {set([m.constellation_id.name for m in processed_measurements])} sv_id {sorted([m.sv_id for m in processed_measurements])[:5]} {pos_fix_related} ")
      assert all(np.isfinite(pos_std))
      assert all(np.isfinite(vel_std))
      dat.gnssMeasurements = {
        "positionECEF": measurement_msg(value=ecef_pos, std=pos_std, valid=kf_valid),
        "velocityECEF": measurement_msg(value=ecef_vel, std=vel_std, valid=kf_valid),
        "bearingDeg": measurement_msg(value=[bearing_deg], std=[bearing_std], valid=kf_valid),
        "ubloxMonoTime": ublox_mono_time,
        "correctedMeasurements": meas_msgs
      }
      return dat
    elif ublox_msg.which == 'ephemeris':
      ephem = convert_ublox_ephem(ublox_msg.ephemeris)
      self.astro_dog.add_ephems([ephem], self.astro_dog.nav)
    # elif ublox_msg.which == 'ionoData':
    # todo add this. Needed to better correct messages offline. First fix ublox_msg.cc to sent them.

  def update_localizer(self, get_pos_fix, t: float, measurements: List[GNSSMeasurement]):
    # Check time and outputs are valid
    valid = self.kf_valid(t)
    if not all(valid):
      # A position fix is needed when resetting the kalman filter.
      pos_fix = get_pos_fix()
      if pos_fix is None:
        return
      post_est = pos_fix[0][:3].tolist()
      if not valid[0]:
        cloudlog.info("Init gnss kalman filter")
      elif not valid[1]:
        cloudlog.error("Time gap of over 10s detected, gnss kalman reset")
      elif not valid[2]:
        cloudlog.error("Gnss kalman std too far")
      else:
        cloudlog.error("Gnss kalman filter state is nan")
      self.init_gnss_localizer(post_est, pos_fix[1])
    if len(measurements) > 0:
      kf_add_observations(self.gnss_kf, t, measurements)
    else:
      # Ensure gnss filter is updated even with no new measurements
      self.gnss_kf.predict(t)

  def kf_valid(self, t: float):
    filter_time = self.gnss_kf.filter.filter_time
    return [filter_time is not None,
            filter_time is not None and abs(t - filter_time) < MAX_TIME_GAP,
            linalg.norm(self.gnss_kf.P[GStates.ECEF_POS]) < 1e6,
            all(np.isfinite(self.gnss_kf.x[GStates.ECEF_POS]))]

  def init_gnss_localizer(self, est_pos, pos_std):
    x_initial, p_initial_diag = np.copy(GNSSKalman.x_initial), np.copy(np.diagonal(GNSSKalman.P_initial))
    x_initial[GStates.ECEF_POS] = est_pos
    # np.mean(abs(np.array(pos_std))) ** 2??
    p_initial_diag[GStates.ECEF_POS] = 1000 ** 2
    self.gnss_kf.init_state(x_initial, covs_diag=p_initial_diag)

  def get_orbit_data(self, t: GPSTime, queue):
    cloudlog.info(f"Start to download/parse orbits for time {t.as_datetime()}")
    start_time = time.monotonic()
    try:
      self.astro_dog.get_orbit_data(t, only_predictions=True)
    except RuntimeError as e:
      cloudlog.info(f"No orbit data found. {e}")
      return
    cloudlog.info(f"Done parsing orbits. Took {time.monotonic() - start_time:.2f}s")
    if queue is not None:
      queue.put((self.astro_dog.orbits, self.astro_dog.orbit_fetched_times))

  def fetch_orbits(self, t: GPSTime, block):
    if t not in self.astro_dog.orbit_fetched_times:
      if block:
        self.get_orbit_data(t, None)
        return
      if self.orbit_p is None:
        self.orbit_p = Process(target=self.get_orbit_data, args=(t, self.orbit_q))
        self.orbit_p.start()
      if not self.orbit_q.empty():
        ret = self.orbit_q.get()
        if ret:
          self.astro_dog.orbits, self.astro_dog.orbit_fetched_times = ret
          self.orbit_p.join()
          self.orbit_p = None

  def __del__(self):
    if self.orbit_p is not None:
      self.orbit_p.kill()


def create_measurement_msg(meas: GNSSMeasurement):
  c = log.GnssMeasurements.CorrectedMeasurement.new_message()
  c.constellationId = meas.constellation_id.value
  c.svId = meas.sv_id
  c.glonassFrequency = meas.glonass_freq if meas.constellation_id == ConstellationId.GLONASS else 0
  c.pseudorange = float(meas.observables_final['C1C'])
  c.pseudorangeStd = float(meas.observables_std['C1C'])
  c.pseudorangeRate = float(meas.observables_final['D1C'])
  c.pseudorangeRateStd = float(meas.observables_std['D1C'])
  c.satPos = meas.sat_pos_final.tolist()
  c.satVel = meas.sat_vel.tolist()
  return c


def kf_add_observations(gnss_kf: GNSSKalman, t: float, measurements: List[GNSSMeasurement]):
  ekf_data = defaultdict(list)
  for m in measurements:
    m_arr = m.as_array()
    if m.constellation_id == ConstellationId.GPS:
      ekf_data[ObservationKind.PSEUDORANGE_GPS].append(m_arr)
      ekf_data[ObservationKind.PSEUDORANGE_RATE_GPS].append(m_arr)
    elif m.constellation_id == ConstellationId.GLONASS:
      ekf_data[ObservationKind.PSEUDORANGE_GLONASS].append(m_arr)
      ekf_data[ObservationKind.PSEUDORANGE_RATE_GLONASS].append(m_arr)

  for kind, data in ekf_data.items():
    gnss_kf.predict_and_observe(t, kind, data)


def get_bearing_from_gnss(ecef_pos, ecef_vel, vel_std):
  # init orientation with direction of velocity
  converter = coord.LocalCoord.from_ecef(ecef_pos)

  ned_vel = np.einsum('ij,j ->i', converter.ned_from_ecef_matrix, ecef_vel)
  bearing = np.arctan2(ned_vel[1], ned_vel[0])
  bearing_std = np.arctan2(np.linalg.norm(vel_std), np.linalg.norm(ned_vel))
  return float(np.rad2deg(bearing)), float(bearing_std)


def main():
  sm = messaging.SubMaster(['ubloxGnss'])
  pm = messaging.PubMaster(['gnssMeasurements'])

  laikad = Laikad()
  while True:
    sm.update()

    if sm.updated['ubloxGnss']:
      ublox_msg = sm['ubloxGnss']
      msg = laikad.process_ublox_msg(ublox_msg, sm.logMonoTime['ubloxGnss'])
      if msg is not None:
        pm.send('gnssMeasurements', msg)


if __name__ == "__main__":
  main()
