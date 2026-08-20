import math

import config
from navigation import wrap180_deg, wrap180_rad


class PID:
    def __init__(self, kp, ki, kd, out_min=-1e9, out_max=1e9, int_max=1e9):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.int_max = int_max
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_err = 0.0
        self.last_out = 0.0

    def update(self, setpoint, measurement, dt):
        err = setpoint - measurement
        self.integral += err * dt
        self.integral = max(-self.int_max, min(self.int_max, self.integral))
        deriv = (err - self.last_err) / dt if dt > 0 else 0.0
        self.last_err = err
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        out = max(self.out_min, min(self.out_max, out))
        self.last_out = out
        return out


STANDBY = "STANDBY"
MANUAL = "MANUAL"
GPS = "GPS"
RTH = "RTH"
LANDING = "LANDING"


class FlightController:
    def __init__(self):
        self.mode = STANDBY
        self.target_alt = 0.0
        self.target_pos = None
        self.target_heading = None
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.alt_int = 0.0
        self.alt_pid = PID(config.ALT_KP, config.ALT_KI, config.ALT_KD)
        self.roll_rate_pid = PID(config.RATE_KP_ROLL, 0.0, 0.0, -config.MAX_TORQUE, config.MAX_TORQUE)
        self.pitch_rate_pid = PID(config.RATE_KP_PITCH, 0.0, 0.0, -config.MAX_TORQUE, config.MAX_TORQUE)
        self.yaw_rate_pid = PID(config.RATE_KP_YAW, 0.0, 0.0, -config.MAX_TORQUE, config.MAX_TORQUE)
        self.thrust = 0.0
        self.des_roll = 0.0
        self.des_pitch = 0.0
        self.des_yaw_rate = 0.0
        self.yaw_hold = 0.0
        self.yaw_hold_set = False

    def reset(self):
        self.mode = STANDBY
        self.target_alt = 0.0
        self.target_pos = None
        self.target_heading = None
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.alt_pid.reset()
        self.roll_rate_pid.reset()
        self.pitch_rate_pid.reset()
        self.yaw_rate_pid.reset()
        self.thrust = 0.0
        self.des_roll = 0.0
        self.des_pitch = 0.0
        self.des_yaw_rate = 0.0
        self.yaw_hold_set = False
        self.alt_int = 0.0

    def takeoff(self, alt=None):
        self.mode = MANUAL
        self.target_alt = alt if alt else config.TAKEOFF_ALT
        self.target_pos = None
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.alt_int = 0.0
        self.alt_pid.reset()
        self.yaw_hold_set = False

    def land(self):
        self.mode = LANDING
        self.target_alt = 0.0
        self.manual_vx = 0.0
        self.manual_vy = 0.0

    def hover(self):
        self.mode = MANUAL
        self.manual_vx = 0.0
        self.manual_vy = 0.0

    def hold_position(self, n, e):
        self.mode = GPS
        self.target_pos = (n, e)

    def goto(self, n, e, alt=None):
        self.mode = GPS
        self.target_pos = (n, e)
        if alt is not None:
            self.target_alt = alt
        self.manual_vx = 0.0
        self.manual_vy = 0.0

    def return_home(self, home_pos):
        self.mode = RTH
        self.target_pos = home_pos
        self.manual_vx = 0.0
        self.manual_vy = 0.0

    def set_manual(self, vx, vy):
        if self.mode in (MANUAL, GPS, RTH):
            self.mode = MANUAL
            self.manual_vx = vx
            self.manual_vy = vy

    def set_heading(self, deg):
        self.target_heading = deg

    def clear_heading(self):
        self.target_heading = None

    def update(self, drone, dt):
        if self.mode == STANDBY:
            drone.set_motors(0.0, 0.0, 0.0, 0.0)
            self.thrust = 0.0
            return

        err_alt = self.target_alt - drone.est_alt
        self.alt_int += err_alt * dt
        self.alt_int = max(-config.ALT_INT_MAX, min(config.ALT_INT_MAX, self.alt_int))
        des_acc = config.ALT_KP * err_alt + config.ALT_KI * self.alt_int - config.ALT_KD * drone.est_vel_up
        des_acc = max(-config.ALT_MAX_ACC, min(config.ALT_MAX_ACC, des_acc))
        self.thrust = config.MASS * (config.G + des_acc)
        self.thrust = max(0.0, min(4 * config.MOTOR_MAX_THRUST, self.thrust))

        if self.mode == GPS or self.mode == RTH:
            tp = self.target_pos
            if tp is not None:
                err_n = tp[0] - drone.est_n
                err_e = tp[1] - drone.est_e
                
                # Position Integral for steady-state wind rejection
                if drone.est_alt > 1.0:
                    self.pos_int_n = max(-5.0, min(5.0, getattr(self, 'pos_int_n', 0.0) + err_n * dt))
                    self.pos_int_e = max(-5.0, min(5.0, getattr(self, 'pos_int_e', 0.0) + err_e * dt))
                else:
                    self.pos_int_n = 0.0
                    self.pos_int_e = 0.0

                v_des_n = max(-config.POS_MAX_VEL, min(config.POS_MAX_VEL, err_n * config.POS_KP + 0.12 * self.pos_int_n))
                v_des_e = max(-config.POS_MAX_VEL, min(config.POS_MAX_VEL, err_e * config.POS_KP + 0.12 * self.pos_int_e))
                yaw = drone.est_yaw
                cy = math.cos(yaw)
                sy = math.sin(yaw)
                vfx = v_des_n * cy + v_des_e * sy
                vfy = -v_des_n * sy + v_des_e * cy
                vx_meas = drone.vel_n * cy + drone.vel_e * sy
                vy_meas = -drone.vel_n * sy + drone.vel_e * cy
                
                # If close to the ground (< 1.2m), maintain level pitch/roll for clean climb
                if drone.est_alt < 1.2 and self.target_alt > 1.5:
                    self.des_pitch = 0.0
                    self.des_roll = 0.0
                else:
                    self.des_pitch = max(-config.MAX_TILT, min(config.MAX_TILT, -(vfx - vx_meas) * config.PITCH_VEL_GAIN))
                    self.des_roll = max(-config.MAX_TILT, min(config.MAX_TILT, (vfy - vy_meas) * config.ROLL_VEL_GAIN))
                
                bearing = math.degrees(math.atan2(err_e, err_n)) % 360.0
                self.des_yaw_rate = max(
                    -config.MAX_YAW_RATE,
                    min(config.MAX_YAW_RATE, config.ATT_KP_YAW * math.radians(wrap180_deg(bearing - drone.est_heading_deg))),
                )
                self.yaw_hold_set = False


        else:
            vfx = self.manual_vx
            vfy = self.manual_vy
            self.des_pitch = max(-config.MAX_TILT, min(config.MAX_TILT, -vfx * config.PITCH_VEL_GAIN))
            self.des_roll = max(-config.MAX_TILT, min(config.MAX_TILT, vfy * config.ROLL_VEL_GAIN))
            if self.target_heading is not None:
                self.des_yaw_rate = max(
                    -config.MAX_YAW_RATE,
                    min(config.MAX_YAW_RATE, config.ATT_KP_YAW * math.radians(wrap180_deg(self.target_heading - drone.est_heading_deg))),
                )
                self.yaw_hold_set = False
            else:
                if not self.yaw_hold_set:
                    self.yaw_hold = drone.est_heading_deg
                    self.yaw_hold_set = True
                self.des_yaw_rate = max(
                    -config.MAX_YAW_RATE,
                    min(config.MAX_YAW_RATE, config.ATT_KP_YAW * math.radians(wrap180_deg(self.yaw_hold - drone.est_heading_deg))),
                )

        err_roll = self.des_roll - drone.est_roll
        err_pitch = self.des_pitch - drone.est_pitch
        rate_des_roll = max(-config.MAX_RATE_ATT, min(config.MAX_RATE_ATT, config.ATT_KP_ROLL * err_roll))
        rate_des_pitch = max(-config.MAX_RATE_ATT, min(config.MAX_RATE_ATT, config.ATT_KP_PITCH * err_pitch))
        rate_des_yaw = self.des_yaw_rate

        roll_t = self.roll_rate_pid.update(rate_des_roll, drone.est_p, dt) - config.RATE_DAMP * drone.est_p
        pitch_t = self.pitch_rate_pid.update(rate_des_pitch, drone.est_q, dt) - config.RATE_DAMP * drone.est_q
        yaw_t = self.yaw_rate_pid.update(rate_des_yaw, drone.est_r, dt) - config.RATE_DAMP * drone.est_r

        drone.set_motors(self.thrust, roll_t, pitch_t, yaw_t)
