# Data-Driven Modeling & Simulation (DDMS) Final Report

## Indian Campus Perimeter Flight Analysis
- **Location**: Amrita Coimbatore Campus (10.9001°N, 76.9002°E)
- **Flight Duration**: 73.91 seconds
- **Sampling Frequency**: 250 Hz (18,477 total samples)
- **Atmospheric Wind**: 0.0 m/s with turbulence & gust bursts

---

## 📊 Summary Accuracy Metrics Table

| State | SINDy Eq R² | SINDy RMSE | DMDc Discrete R² | DMDc RMSE |
| --- | --- | --- | --- | --- |
| p (roll-rate) | 0.9798 | 14.8082 | 0.7833 | 0.3545 |
| q (pitch-rate) | 0.9867 | 14.8205 | 0.7149 | 0.4157 |
| r (yaw-rate) | 0.9954 | 5.1716 | 0.6869 | 0.2712 |
| roll | 0.9844 | 1.2064 | 0.4511 | 0.1453 |
| pitch | 1.0000 | 0.9973 | 0.0000 | 0.2509 |
| yaw | 0.0044 | 12.9836 | 0.0000 | 4.2008 |
| v_north | 0.2100 | 23.9345 | 0.0000 | 5.5038 |
| v_east | 0.2235 | 24.0836 | 0.0000 | 3.0918 |
| v_up | 0.1439 | 24.5872 | 0.0000 | 8.2025 |

---

## 🔬 Discovered SINDy Governing Equations
```
======================================================================
>>> SINDy DISCOVERED GOVERNING DIFFERENTIAL EQUATIONS
======================================================================

d(p (roll-rate))/dt = +4.1077 -0.8666 * p -0.5853 * q -3.0900 * r +24.5928 * roll +25.7303 * pitch +0.0209 * vn -0.0973 * ve -0.0794 * vup -1.0306 * u_thrust +36.8414 * u_roll +0.1008 * u_pitch +0.0450 * u_yaw +8.9666 * u1 -9.5044 * u2 -9.4315 * u3 +8.9388 * u4 -0.1045 * q*r +0.0747 * p*r +1.2675 * p*|p| -0.1465 * r*|r| -0.0199 * vn*|vn| +0.0122 * ve*|ve| +0.0234 * vup*|vup| -24.2856 * sin(roll) -27.3024 * sin(pitch) +0.0601 * sin(yaw) -4.1546 * cos(roll)*cos(pitch) +0.5230 * sin(pitch)*u_thrust -0.4057 * sin(roll)*u_thrust +1.3219 * cos(roll)*cos(pitch)*u_thrust +0.5764 * q*cos(roll) -0.7245 * r*sin(roll) +3.3468 * r*cos(roll) -0.2028 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.9798 | Derivative RMSE: 0.70233

d(q (pitch-rate))/dt = +12.5887 +0.3027 * p -2.7688 * q -7.4837 * r +12.4703 * roll -16.1683 * pitch +0.1877 * vn -0.1697 * ve -0.1742 * vup -3.0379 * u_thrust -0.1400 * u_roll +36.6395 * u_pitch +0.0401 * u_yaw +8.3554 * u1 -9.8944 * u2 +8.4454 * u3 -9.9443 * u4 -0.0519 * q*r +0.0210 * p*r +0.0741 * p*q -0.0309 * p*|p| +1.7045 * q*|q| +0.4166 * r*|r| -0.0175 * vn*|vn| +0.0169 * ve*|ve| +0.0492 * vup*|vup| -12.7531 * sin(roll) +14.3946 * sin(pitch) +0.3903 * sin(yaw) -0.6364 * cos(yaw) -11.6171 * cos(roll)*cos(pitch) +0.4918 * sin(pitch)*u_thrust +3.7853 * cos(roll)*cos(pitch)*u_thrust +1.3248 * q*cos(roll) -0.8734 * r*sin(roll) +7.5048 * r*cos(roll) -0.3127 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.9867 | Derivative RMSE: 0.59811

d(r (yaw-rate))/dt = -0.0801 +0.0271 * p -0.0251 * r +1.2448 * roll -1.0423 * pitch +0.0267 * u_thrust +2.8650 * u_yaw -0.7113 * u1 -0.7078 * u2 +0.7235 * u3 +0.7224 * u4 +0.0146 * r*|r| -1.2653 * sin(roll) +1.0605 * sin(pitch) +0.0859 * cos(roll)*cos(pitch) -0.0360 * cos(roll)*cos(pitch)*u_thrust -0.0252 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.9954 | Derivative RMSE: 0.07346

d(roll)/dt = -1.0692 -0.3771 * p +0.2894 * q +0.7459 * r +13.2849 * roll +1.0845 * pitch +0.0133 * vn +0.3410 * u_thrust +0.0415 * u_roll +0.0626 * u_yaw +0.0676 * u1 +0.0716 * u2 +0.0782 * u3 +0.1237 * u4 +0.0415 * q*r +0.1480 * p*r +0.1776 * p*|p| -0.0340 * q*|q| -12.4127 * sin(roll) -1.1486 * sin(pitch) -0.0129 * sin(yaw) -0.0431 * cos(yaw) +1.1405 * cos(roll)*cos(pitch) -0.4290 * sin(roll)*u_thrust -0.4530 * cos(roll)*cos(pitch)*u_thrust -0.2661 * q*cos(roll) +0.1040 * r*sin(roll) -0.7707 * r*cos(roll) +1.2206 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.9844 | Derivative RMSE: 0.09656

d(pitch)/dt = -0.0411 * pitch +0.0700 * u_pitch +0.0183 * u1 -0.0169 * u2 +0.0181 * u3 -0.0167 * u4 +0.0404 * sin(pitch) +1.0000 * q*cos(roll) -1.0034 * r*sin(roll)
  └── R² Score: 1.0000 | Derivative RMSE: 0.00312

d(yaw)/dt = -149.0282 +3.0740 * p +41.7966 * q +36.0125 * r -332.8592 * roll +440.0945 * pitch +0.2772 * vn +0.8220 * ve +1.2657 * vup +48.0138 * u_thrust +2.5925 * u_roll -1.1389 * u_pitch +0.5497 * u_yaw +12.2296 * u1 +11.5025 * u2 +11.2078 * u3 +13.0739 * u4 -0.2505 * q*r +0.7249 * p*r +0.1009 * p*q +0.8020 * p*|p| +2.7602 * q*|q| +0.5554 * r*|r| -0.1432 * vn*|vn| -0.2628 * ve*|ve| -0.2744 * vup*|vup| +373.3663 * sin(roll) -476.2828 * sin(pitch) -0.2700 * sin(yaw) +0.6753 * cos(yaw) +166.2801 * cos(roll)*cos(pitch) +11.3226 * sin(pitch)*u_thrust -13.7503 * sin(roll)*u_thrust -67.0822 * cos(roll)*cos(pitch)*u_thrust -44.6326 * q*cos(roll) +4.8349 * r*sin(roll) -37.6943 * r*cos(roll) -3.6560 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.0044 | Derivative RMSE: 28.81934

d(v_north)/dt = +60.9717 -2.2066 * p -8.5829 * q +20.7432 * r +12.9540 * roll +18.6931 * pitch -0.1159 * vn +0.7305 * ve -0.7474 * vup -19.6903 * u_thrust +0.5549 * u_roll +0.2687 * u_pitch -0.8446 * u_yaw -4.5056 * u1 -4.9173 * u2 -5.2052 * u3 -5.0623 * u4 -0.1857 * q*r +0.2541 * p*r -0.0475 * p*q -0.4027 * p*|p| -0.2564 * q*|q| -2.6110 * r*|r| +0.0573 * vn*|vn| -0.1671 * ve*|ve| +0.2331 * vup*|vup| -14.7879 * sin(roll) -13.4655 * sin(pitch) -0.6433 * sin(yaw) -0.2987 * cos(yaw) -61.8141 * cos(roll)*cos(pitch) -2.8926 * sin(pitch)*u_thrust +0.6815 * sin(roll)*u_thrust +24.9885 * cos(roll)*cos(pitch)*u_thrust +8.7293 * q*cos(roll) -2.6364 * r*sin(roll) -17.8805 * r*cos(roll) +2.6526 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.2100 | Derivative RMSE: 1.84491

d(v_east)/dt = -5.9962 +10.5572 * p +0.6988 * q -0.9004 * r +182.7938 * roll +16.4868 * pitch -0.1594 * vn +0.0659 * ve -0.2634 * vup +1.0169 * u_thrust +1.6971 * u_roll +0.4895 * u_pitch +0.7478 * u_yaw +0.6139 * u1 -0.4793 * u2 +0.1393 * u3 +0.7431 * u4 +0.3201 * q*r +0.2731 * p*r -0.0521 * p*q -0.5447 * p*|p| -1.5026 * q*|q| +0.4134 * r*|r| +0.0542 * vn*|vn| -0.0199 * ve*|ve| +0.1127 * vup*|vup| -177.3328 * sin(roll) -20.9345 * sin(pitch) -0.0301 * sin(yaw) -0.1400 * cos(yaw) +5.5594 * cos(roll)*cos(pitch) +1.2967 * sin(pitch)*u_thrust -1.6590 * sin(roll)*u_thrust -1.0248 * cos(roll)*cos(pitch)*u_thrust +0.5629 * q*cos(roll) +1.8062 * r*sin(roll) +0.8338 * r*cos(roll) -9.7694 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.2235 | Derivative RMSE: 1.70374

d(v_up)/dt = -0.1800 -0.1838 * p +1.1789 * q +9.1651 * r -14.7803 * roll -14.3233 * pitch -0.0191 * vn +0.1240 * ve -0.1679 * vup -3.6092 * u_thrust -0.2371 * u_roll +0.0628 * u_pitch +0.0178 * u_yaw -0.9503 * u1 -0.8632 * u2 -0.8228 * u3 -0.9728 * u4 +0.0886 * q*r -0.2109 * p*r -0.0379 * p*q -0.3854 * p*|p| +0.1583 * q*|q| -0.4497 * r*|r| +0.0135 * vn*|vn| -0.0166 * ve*|ve| +0.0172 * vup*|vup| +15.7660 * sin(roll) +14.5817 * sin(pitch) -0.2313 * sin(yaw) -0.1354 * cos(yaw) -10.2477 * cos(roll)*cos(pitch) -0.0674 * sin(pitch)*u_thrust -0.3952 * sin(roll)*u_thrust +8.7807 * cos(roll)*cos(pitch)*u_thrust -1.3531 * q*cos(roll) +0.4175 * r*sin(roll) -9.0463 * r*cos(roll) +0.5746 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.1439 | Derivative RMSE: 2.92912

======================================================================
```

---

## 🌀 Discovered DMDc System Dynamics
```
======================================================================
>>> DMDc (DYNAMIC MODE DECOMPOSITION WITH CONTROL) REPORT
======================================================================

Identified Discrete System Matrix A (9x9):
[[ 0.9997 -0.0002  0.0097 -0.0727  0.001  -0.0004 -0.0002 -0.0002 -0.0002]
 [ 0.0008  0.9996 -0.0001  0.     -0.0683 -0.0001  0.0002 -0.0001 -0.0018]
 [ 0.      0.      0.9992  0.0052 -0.0003  0.      0.     -0.      0.    ]
 [ 0.0039 -0.0001  0.0052  0.961   0.0007 -0.0002 -0.      0.     -0.0001]
 [ 0.0004  0.0037 -0.0004  0.001   0.9662  0.      0.0002 -0.0001 -0.0008]
 [-0.0006  0.0018  0.002  -0.0019 -0.0005  0.996  -0.0003 -0.0019 -0.0012]
 [ 0.0001 -0.0011  0.0026  0.0016 -0.0065  0.0001  1.      0.0002  0.0001]
 [ 0.0009 -0.0005  0.0018  0.01   -0.0043  0.     -0.0002  1.      0.0002]
 [ 0.0003 -0.0001 -0.0024  0.0022 -0.0017 -0.0007 -0.     -0.0005  0.9999]]

Identified Control Input Matrix B (9x4):
[[ 0.0076 -0.0082 -0.0052  0.0077]
 [ 0.0036 -0.0073  0.0073 -0.005 ]
 [-0.0021 -0.0264  0.0002  0.0282]
 [-0.0944  0.0938  0.0992 -0.0976]
 [-0.0915  0.0881 -0.0907  0.0903]
 [ 0.0032  0.0033  0.0068  0.0061]
 [ 0.0046  0.003  -0.003  -0.0055]
 [-0.0047 -0.0034  0.0053  0.0031]
 [-0.0001  0.0051 -0.0021  0.0008]]

Dynamic Mode Spectrum & Stability Analysis:
Mode #   Discrete Eigenvalue (z)      Continuous Pole (s)        Freq (Hz)    Damping ζ
----------------------------------------------------------------------------------------
Mode 1   +0.9695 +0.0000j (|z|=0.9695) -7.752 +0.000j             1.234        +1.0000
Mode 2   +0.9784 +0.0000j (|z|=0.9784) -5.447 +0.000j             0.867        +1.0000
Mode 3   +0.9875 +0.0000j (|z|=0.9875) -3.150 +0.000j             0.501        +1.0000
Mode 4   +0.9904 +0.0000j (|z|=0.9904) -2.400 +0.000j             0.382        +1.0000
Mode 5   +0.9959 +0.0000j (|z|=0.9959) -1.039 +0.000j             0.165        +1.0000
Mode 6   +0.9999 +0.0004j (|z|=0.9999) -0.026 +0.107j             0.018        +0.2402
Mode 7   +0.9999 -0.0004j (|z|=0.9999) -0.026 -0.107j             0.018        +0.2402
Mode 8   +1.0002 +0.0000j (|z|=1.0002) +0.044 +0.000j             0.007        -1.0000
Mode 9   +1.0000 +0.0000j (|z|=1.0000) -0.000 +0.000j             0.000        +1.0000

======================================================================
```

---

## 🖼️ Generated Visual Artifacts
1. **3D Flight Path Benchmark**: `trajectory_3d_comparison.png`
2. **6-DOF State Tracking Comparison**: `state_time_series_comparison.png`
3. **DMDc Eigenvalue Stability Circle**: `dmdc_eigenvalue_spectrum.png`
