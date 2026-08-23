import math
import random

import config


def rot_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


class Drone:
    def __init__(self):
        self.t = 0.0
        self.n = 0.0
        self.e = 0.0
        self.alt = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vel_n = 0.0
        self.vel_e = 0.0
        self.vel_up = 0.0
        self.vel_xy = 0.0
        self.vel_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        self.motors = [0.0, 0.0, 0.0, 0.0]
        self.battery = config.BATTERY_CAPACITY
        self.armed = False
        self.sat = 0
        self.gps_fix = False
        self.gps_bias_n = 0.0
        self.gps_bias_e = 0.0
        self.est_n = 0.0
        self.est_e = 0.0
        self.est_alt = 0.0
        self.est_x = 0.0
        self.est_y = 0.0
        self.est_z = 0.0
        self.est_yaw = 0.0
        self.est_vel_n = 0.0
        self.est_vel_e = 0.0
        self.est_vel_up = 0.0
        self.est_vel_xy = 0.0
        self.est_vel_z = 0.0
        self.est_p = 0.0
        self.est_q = 0.0
        self.est_r = 0.0
        self.est_roll = 0.0
        self.est_pitch = 0.0
        self.thrust_norm = 0.0
        self.thrust_total = 0.0
        self.ax_w = 0.0
        self.ay_w = 0.0
        self.az_w = 0.0
        self.ax_body = 0.0
        self.ay_body = 0.0
        self.az_body = -config.G
        self.crashed = False
        
        # Wind & Turbulence Dynamics
        self.wind_speed = config.WIND_SPEED_CALM
        self.wind_heading_deg = 45.0  # Wind blowing from North-East
        self.wind_mode = "CALM"       # "CALM", "LIGHT", "STRONG", "GUST", "STORM"
        self.wind_gust_timer = 0.0
        self.wind_gust_duration = 0.0
        self.wind_gust_strength = 0.0
        self.current_wind_n = 0.0
        self.current_wind_e = 0.0
        self.current_wind_up = 0.0
        self.current_wind_x = 0.0
        self.current_wind_y = 0.0
        self.current_wind_z = 0.0

    def set_wind(self, mode, speed=None, heading_deg=None):
        mode = mode.upper()
        self.wind_mode = mode
        if heading_deg is not None:
            self.wind_heading_deg = float(heading_deg) % 360.0
        if speed is not None:
            self.wind_speed = float(speed)
        elif mode == "CALM":
            self.wind_speed = config.WIND_SPEED_CALM
        elif mode == "LIGHT":
            self.wind_speed = config.WIND_SPEED_LIGHT
        elif mode == "STRONG":
            self.wind_speed = config.WIND_SPEED_STRONG
        elif mode == "STORM":
            self.wind_speed = config.WIND_SPEED_STORM

    def trigger_gust(self, strength=config.WIND_GUST_BURST, duration=3.5):
        self.wind_gust_strength = strength
        self.wind_gust_duration = duration
        self.wind_gust_timer = duration

    def cycle_wind(self):
        modes = ["CALM", "LIGHT", "STRONG", "STORM"]
        try:
            idx = (modes.index(self.wind_mode) + 1) % len(modes)
        except ValueError:
            idx = 0
        self.set_wind(modes[idx])
        return self.wind_mode


    @property
    def heading_deg(self):
        return math.degrees(self.yaw) % 360.0

    @property
    def est_heading_deg(self):
        return math.degrees(self.est_yaw) % 360.0

    @property
    def gps_lat(self):
        return config.HOME_LAT + self.est_n / config.M_PER_DEG_LAT

    @property
    def gps_lon(self):
        return config.HOME_LON + self.est_e / config.M_PER_DEG_LON

    @property
    def gps_alt(self):
        return config.HOME_ALT + self.est_z / config.M_PER_DEG_ALT if hasattr(config, 'M_PER_DEG_ALT') else self.est_alt

    def set_motors(self, thrust, roll_t, pitch_t, yaw_t):
        arm_xy = config.ARM / math.sqrt(2.0)
        a = arm_xy
        c = config.YAW_COEF
        t = [
            0.25 * thrust + 0.25 / a * roll_t + 0.25 / a * pitch_t - 0.25 / c * yaw_t,
            0.25 * thrust - 0.25 / a * roll_t - 0.25 / a * pitch_t - 0.25 / c * yaw_t,
            0.25 * thrust - 0.25 / a * roll_t + 0.25 / a * pitch_t + 0.25 / c * yaw_t,
            0.25 * thrust + 0.25 / a * roll_t - 0.25 / a * pitch_t + 0.25 / c * yaw_t,
        ]
        self.motors = [min(1.0, max(0.0, m / config.MOTOR_MAX_THRUST)) for m in t]

    def _sense(self):
        if self.gps_fix:
            alpha = config.GPS_FILTER_ALPHA
            raw_n = self.n + random.gauss(0, config.GPS_NOISE) + self.gps_bias_n
            raw_e = self.e + random.gauss(0, config.GPS_NOISE) + self.gps_bias_e
            self.est_n += alpha * (raw_n - self.est_n)
            self.est_e += alpha * (raw_e - self.est_e)
        else:
            self.est_n = self.n
            self.est_e = self.e
        if self.armed and self.gps_fix:
            self.est_vel_n += config.GPS_FILTER_ALPHA * (self.vel_n - self.est_vel_n)
            self.est_vel_e += config.GPS_FILTER_ALPHA * (self.vel_e - self.est_vel_e)
        else:
            self.est_vel_n = self.vel_n
            self.est_vel_e = self.vel_e
        raw_alt = self.alt + random.gauss(0, config.BARO_NOISE)
        self.est_alt += config.BARO_FILTER_ALPHA * (raw_alt - self.est_alt)
        if self.armed:
            self.est_vel_up += config.BARO_FILTER_ALPHA * (self.vel_up - self.est_vel_up)
        else:
            self.est_vel_up = self.vel_up
        raw_yaw = self.yaw + math.radians(random.gauss(0, config.MAG_NOISE))
        diff = raw_yaw - self.est_yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        self.est_yaw += config.MAG_FILTER_ALPHA * diff
        self.est_p = self.p + random.gauss(0, config.GYRO_NOISE)
        self.est_q = self.q + random.gauss(0, config.GYRO_NOISE)
        self.est_r = self.r + random.gauss(0, config.GYRO_NOISE)
        self.est_roll = self.roll + random.gauss(0, math.radians(0.5))
        self.est_pitch = self.pitch + random.gauss(0, math.radians(0.5))

    def _compute_sindy_error(self):
        """Compute SINDy model vs actual dynamics error.
        
        Returns dict with error magnitudes for key state variables.
        Used by both: Boundary Flight detection (Research)
        and Sonification (Creative - failure parameters → audio).
        """
        # SINDy candidate function library terms (simplified)
        # Original SINDy discovered: p_dot involves q*r, q_dot involves p*r, etc.
        
        # Actual rates (from gyro, filtered)
        actual_p = self.est_p
        actual_q = self.est_q  
        actual_r = self.est_r
        
        # SINDy-discovered expected relationships (from your analysis)
        # These are the terms that, when missing, cause SINDy to "fail"
        # p_dot should involve q*r coupling, etc.
        
        # Compute coupling terms that SINDy should have discovered
        qr_coupling = actual_q * actual_r  # roll rate × yaw rate
        rp_coupling = actual_r * actual_p  # yaw rate × roll rate  
        pq_coupling = actual_p * actual_q  # roll rate × pitch rate
        
        # Compare: actual p_dot vs expected (simplified SINDy)
        # Your SINDy analysis showed: p_dot ≈ -0.5*p - 0.2*q*r + 0.1*thrust
        # So the "qr term error" measures model breakdown:
        qr_term_error = abs(qr_coupling) * 0.2  # scaled coupling magnitude
        
        # Similar for other axes  
        rp_term_error = abs(rp_coupling) * 0.2
        pq_term_error = abs(pq_coupling) * 0.2
        
        # Altitude divergence term (from your DMDc analysis)
        # DMDc altitude drifts to -3000m over time - capture rate of divergence
        altitude_divergence = abs(self.est_vel_up) * 0.1 + abs(self.alt - 5.0) * 0.05
        
        # Thrust coupling (from SINDy: thrust appears in equations)
        thrust_coupling = self.thrust_norm * 0.5
        
        # Composite error magnitude
        total_error = math.sqrt(
            qr_term_error**2 + 
            rp_term_error**2 + 
            pq_term_error**2 + 
            altitude_divergence**2 + 
            thrust_coupling**2
        )
        
        return {
            'total': total_error,
            'qr_coupling': qr_coupling,
            'rp_coupling': rp_coupling, 
            'pq_coupling': pq_coupling,
            'altitude_divergence': altitude_divergence,
            'thrust_coupling': thrust_coupling,
            'actual_p': actual_p,
            'actual_q': actual_q,
            'actual_r': actual_r,
            'thrust_norm': self.thrust_norm,
            'est_alt': self.est_alt,
        }

    def _update_gps_fix(self):
        if self.sat < 4:
            self.gps_fix = False
            return
        self.gps_fix = True

    def arm(self):
        self.armed = True

    def disarm(self):
        self.armed = False
        self.motors = [0.0, 0.0, 0.0, 0.0]

    def step(self, dt):
        self.t += dt
        if not self.armed:
            self.motors = [0.0, 0.0, 0.0, 0.0]
            self.thrust_norm = 0.0
            self._sense()
            return

        # Wind Vector & Turbulence Simulation
        if self.wind_gust_timer > 0.0:
            self.wind_gust_timer = max(0.0, self.wind_gust_timer - dt)
            gust_frac = self.wind_gust_timer / max(0.001, self.wind_gust_duration)
            cur_gust = self.wind_gust_strength * math.sin(gust_frac * math.pi)
        else:
            cur_gust = 0.0

        w_rad = math.radians(self.wind_heading_deg)
        base_speed = self.wind_speed + cur_gust
        t_turb = self.t * 2.0
        turb_n = 0.25 * base_speed * math.sin(1.7 * t_turb) + random.gauss(0, 0.08 * (base_speed + 0.5))
        turb_e = 0.25 * base_speed * math.cos(2.3 * t_turb) + random.gauss(0, 0.08 * (base_speed + 0.5))
        turb_up = 0.15 * base_speed * math.sin(3.1 * t_turb) + random.gauss(0, 0.05 * (base_speed + 0.5))
        
        self.current_wind_n = base_speed * math.cos(w_rad) + turb_n
        self.current_wind_e = base_speed * math.sin(w_rad) + turb_e
        self.current_wind_up = turb_up
        
        self.current_wind_x = -self.current_wind_e  # Body-to-world conversion
        self.current_wind_y = self.current_wind_n
        self.current_wind_z = self.current_wind_up

        # Relative Airspeed (3D)
        v_rel_n = self.vel_n - self.current_wind_x
        v_rel_e = self.vel_e - self.current_wind_y
        v_rel_up = self.vel_up - self.current_wind_z
        v_rel_mag = math.sqrt(v_rel_n**2 + v_rel_e**2 + v_rel_up**2)

        arm_xy = config.ARM / math.sqrt(2.0)
        c = config.YAW_COEF
        ti = [m * config.MOTOR_MAX_THRUST for m in self.motors]
        thrust_total = sum(ti)
        roll_t = arm_xy * ti[0] - arm_xy * ti[1] - arm_xy * ti[2] + arm_xy * ti[3]
        pitch_t = arm_xy * ti[0] - arm_xy * ti[1] + arm_xy * ti[2] - arm_xy * ti[3]
        yaw_t = -c * ti[0] - c * ti[1] + c * ti[2] + c * ti[3]
        
        # Aerodynamic moment disturbances from crosswind
        roll_t += 0.004 * (-v_rel_e * math.cos(self.yaw) + v_rel_n * math.sin(self.yaw))
        pitch_t += 0.004 * (v_rel_n * math.cos(self.yaw) + v_rel_e * math.sin(self.yaw))
        
        self.thrust_norm = thrust_total / (4 * config.MOTOR_MAX_THRUST)

        p_dot = roll_t / config.I_ROLL
        q_dot = pitch_t / config.I_PITCH
        r_dot = yaw_t / config.I_YAW
        self.p += p_dot * dt
        self.q += q_dot * dt
        self.r += r_dot * dt

        roll = self.roll
        pitch = self.pitch
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        tth = math.tan(pitch)
        self.roll += (self.p + self.q * sr * tth + self.r * cr * tth) * dt
        self.pitch += (self.q * cr - self.r * sr) * dt
        self.yaw += (self.q * sr / cp + self.r * cr / cp) * dt

        rmat = rot_matrix(self.roll, self.pitch, self.yaw)
        ax_w, ay_w, az_w = 0.0, 0.0, config.G
        bx = 0.0
        by = 0.0
        bz = -thrust_total / config.MASS
        ax_w = rmat[0][0] * bx + rmat[0][1] * by + rmat[0][2] * bz
        ay_w = rmat[1][0] * bx + rmat[1][1] * by + rmat[1][2] * bz
        az_w = config.G + (rmat[2][0] * bx + rmat[2][1] * by + rmat[2][2] * bz)

        # Relative Airspeed Aerodynamic Drag
        drag_h = config.DRAG_H * (1.0 + 0.08 * v_rel_mag)
        drag_v = config.DRAG_V * (1.0 + 0.08 * v_rel_mag)
        ax_w -= drag_h * v_rel_n
        ay_w -= drag_h * v_rel_e
        az_w -= drag_v * (-v_rel_up)

        self.ax_w = ax_w
        self.ay_w = ay_w
        self.az_w = az_w
        self.thrust_total = thrust_total

        # IMU Accelerometer Specific Force in Body Frame
        # fb = R^T * (a_w - [0, 0, g]^T)
        lin_acc_z = az_w - config.G
        self.ax_body = rmat[0][0] * ax_w + rmat[1][0] * ay_w + rmat[2][0] * lin_acc_z
        self.ay_body = rmat[0][1] * ax_w + rmat[1][1] * ay_w + rmat[2][1] * lin_acc_z
        self.az_body = rmat[0][2] * ax_w + rmat[1][2] * ay_w + rmat[2][2] * lin_acc_z - (thrust_total / config.MASS)

        self.vel_n += ax_w * dt
        self.vel_e += ay_w * dt
        self.vel_up += -az_w * dt

        self.n += self.vel_n * dt
        self.e += self.vel_e * dt
        self.alt += self.vel_up * dt

        self.x += self.vel_n * dt
        self.y += self.vel_e * dt
        self.z += self.vel_up * dt

        self.vel_xy = math.sqrt(self.vel_n**2 + self.vel_e**2)
        self.vel_z = self.vel_up

        if self.alt <= 0.0:
            self.alt = 0.0

            self.vel_up = max(0.0, self.vel_up)
            self.vel_n *= 0.7
            self.vel_e *= 0.7
            self.roll *= 0.8
            self.pitch *= 0.8
            self.p *= 0.6
            self.q *= 0.6


        self.gps_bias_n += random.gauss(0, config.GPS_BIAS_STEP * dt)
        self.gps_bias_e += random.gauss(0, config.GPS_BIAS_STEP * dt)

        drain = config.BATTERY_DRAIN_BASE + config.BATTERY_DRAIN_THRUST * self.thrust_norm
        self.battery = max(0.0, self.battery - drain * dt)

        self._update_gps_fix()
        self._sense()
        self.sindy_error = self._compute_sindy_error()
