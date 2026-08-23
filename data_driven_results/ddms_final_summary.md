# Data-Driven Modeling & Simulation (DDMS) Final Report

## Indian Campus Perimeter Flight Analysis
- **Location**: Amrita Coimbatore Campus (10.9001°N, 76.9002°E)
- **Flight Duration**: 55.81 seconds
- **Sampling Frequency**: 250 Hz (13,952 total samples)
- **Atmospheric Wind**: 0.0 m/s with turbulence & gust bursts

---

## 📊 Summary Accuracy Metrics Table

| State | SINDy Eq R² | SINDy RMSE | DMDc Discrete R² | DMDc RMSE |
| --- | --- | --- | --- | --- |
| p (roll-rate) | 0.9914 | 14.4457 | 0.0000 | 3.1176 |
| q (pitch-rate) | 0.9909 | 14.3936 | 0.0000 | 1.6340 |
| r (yaw-rate) | 0.9902 | 14.3645 | 0.0000 | 1.8949 |
| roll | 0.9959 | 1.1460 | 0.0000 | 0.6776 |
| pitch | 1.0000 | 1.1856 | 0.0000 | 0.8908 |
| yaw | 0.0016 | 12.7933 | 0.0000 | 64.2378 |
| v_north | 0.4473 | 24.1417 | 0.0000 | 43.6669 |
| v_east | 0.4174 | 23.8393 | 0.0000 | 54.4975 |
| v_up | 0.9806 | 23.5713 | 0.0000 | 76.3396 |

---

## 🔬 Discovered SINDy Governing Equations
```
======================================================================
>>> SINDy DISCOVERED GOVERNING DIFFERENTIAL EQUATIONS
======================================================================

d(p (roll-rate))/dt = +3.9145 -2.3421 * p +1.1324 * q +3.9637 * r +11.5719 * roll +24.9921 * pitch +0.0319 * vn -0.1620 * ve +0.0296 * vup -1.1550 * u_thrust +36.8665 * u_roll +0.1352 * u_pitch +0.0258 * u_yaw +8.9552 * u1 -9.5456 * u2 -9.4651 * u3 +8.9005 * u4 +0.0258 * q*r +0.1478 * p*r -0.0212 * p*q +1.6592 * p*|p| -0.0993 * q*|q| -0.7234 * r*|r| +0.0322 * ve*|ve| -13.3238 * sin(roll) -25.2495 * sin(pitch) -0.0215 * sin(yaw) -0.1646 * cos(yaw) -3.9842 * cos(roll)*cos(pitch) +0.0563 * sin(pitch)*u_thrust +0.3488 * sin(roll)*u_thrust +1.4959 * cos(roll)*cos(pitch)*u_thrust -1.0554 * q*cos(roll) +0.1413 * r*sin(roll) -3.3520 * r*cos(roll) +0.8726 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.9914 | Derivative RMSE: 0.52763

d(q (pitch-rate))/dt = +10.0556 -0.3124 * p -0.1513 * q +8.7219 * r -20.8513 * roll -32.3503 * pitch +0.2189 * vn -0.0556 * ve -0.3438 * vup -2.0008 * u_thrust -0.1970 * u_roll +36.6716 * u_pitch -0.0764 * u_yaw +8.6375 * u1 -9.5998 * u2 +8.6978 * u3 -9.7364 * u4 +0.1367 * q*r -0.0830 * p*r +0.0198 * p*q +0.2116 * p*|p| +1.7212 * q*|q| -0.8354 * r*|r| -0.0386 * vn*|vn| +0.0247 * ve*|ve| +0.0956 * vup*|vup| +20.7163 * sin(roll) +29.8632 * sin(pitch) -0.4773 * cos(yaw) -9.3354 * cos(roll)*cos(pitch) +0.7749 * sin(pitch)*u_thrust +2.6351 * cos(roll)*cos(pitch)*u_thrust -1.4425 * q*cos(roll) -1.6520 * r*sin(roll) -8.0996 * r*cos(roll) +0.1350 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.9909 | Derivative RMSE: 0.52833

d(r (yaw-rate))/dt = +0.2602 +0.0414 * p +0.1024 * q +0.4723 * roll +1.5352 * pitch -0.0916 * u_thrust +0.0170 * u_pitch +2.8366 * u_yaw -0.7221 * u1 -0.7420 * u2 +0.6848 * u3 +0.6876 * u4 -0.0139 * q*r -0.0373 * q*|q| -0.4110 * sin(roll) -1.5652 * sin(pitch) -0.2580 * cos(roll)*cos(pitch) +0.0153 * sin(pitch)*u_thrust -0.0309 * sin(roll)*u_thrust +0.1154 * cos(roll)*cos(pitch)*u_thrust -0.0690 * q*cos(roll) -0.0397 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.9902 | Derivative RMSE: 0.07145

d(roll)/dt = +2.3047 +0.3669 * p +0.0768 * q +1.7432 * r +0.7641 * roll +36.3227 * pitch -0.7442 * u_thrust +0.0556 * u_roll +0.1235 * u_pitch -0.1367 * u1 -0.2262 * u2 -0.1737 * u3 -0.2076 * u4 +0.0196 * q*r +0.1015 * p*|p| -0.1761 * r*|r| -2.7561 * sin(roll) -38.3363 * sin(pitch) -2.2766 * cos(roll)*cos(pitch) +0.7526 * sin(pitch)*u_thrust +0.7714 * sin(roll)*u_thrust +0.9229 * cos(roll)*cos(pitch)*u_thrust -0.0812 * q*cos(roll) +0.1607 * r*sin(roll) -1.7169 * r*cos(roll) +0.5566 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.9959 | Derivative RMSE: 0.05093

d(pitch)/dt = -0.0443 * roll +0.0518 * pitch +0.0719 * u_pitch +0.0187 * u1 -0.0170 * u2 +0.0190 * u3 -0.0172 * u4 +0.0447 * sin(roll) -0.0529 * sin(pitch) +0.9998 * q*cos(roll) -1.0040 * r*sin(roll)
  └── R² Score: 1.0000 | Derivative RMSE: 0.00233

d(yaw)/dt = -2.1869 -4.3199 * p -10.1380 * q +3.5544 * r -8.3591 * roll -110.6384 * pitch +0.3797 * vn +0.5563 * ve -0.6988 * vup +0.5885 * u_thrust +0.2319 * u_roll +0.3040 * u_pitch -0.1934 * u_yaw +0.3294 * u1 +0.0615 * u2 +0.1168 * u3 +0.0807 * u4 +0.0146 * q*r -0.4162 * p*r +0.1974 * p*q -2.4295 * p*|p| -1.5025 * q*|q| +1.4079 * r*|r| -0.1526 * vn*|vn| -0.1451 * ve*|ve| +0.1421 * vup*|vup| +0.4694 * sin(roll) +112.5880 * sin(pitch) -0.5100 * sin(yaw) +0.1242 * cos(yaw) +1.5992 * cos(roll)*cos(pitch) +0.0327 * sin(pitch)*u_thrust +2.8467 * sin(roll)*u_thrust -0.4936 * cos(roll)*cos(pitch)*u_thrust +11.3402 * q*cos(roll) -3.8803 * r*sin(roll) -3.2384 * r*cos(roll) +6.6488 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.0016 | Derivative RMSE: 16.27661

d(v_north)/dt = -42.3998 +6.6009 * p -3.4599 * q -105.4226 * r +275.3104 * roll +68.3734 * pitch -0.5885 * vn -0.4459 * ve +0.5951 * vup +16.4928 * u_thrust +1.3103 * u_roll -0.0454 * u_pitch -0.7188 * u_yaw +4.6191 * u1 +3.9866 * u2 +3.6045 * u3 +4.2824 * u4 -1.8355 * q*r +1.1979 * p*r -0.1282 * p*q -0.7474 * p*|p| +0.0637 * q*|q| -9.5319 * r*|r| +0.2087 * vn*|vn| +0.2319 * ve*|ve| -0.1742 * vup*|vup| -271.8073 * sin(roll) -88.6048 * sin(pitch) -0.4929 * sin(yaw) -0.3595 * cos(yaw) +42.9013 * cos(roll)*cos(pitch) +6.8601 * sin(pitch)*u_thrust -2.3878 * sin(roll)*u_thrust -20.9417 * cos(roll)*cos(pitch)*u_thrust +3.3495 * q*cos(roll) +19.5440 * r*sin(roll) +112.3935 * r*cos(roll) -6.0097 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.4473 | Derivative RMSE: 1.44280

d(v_east)/dt = -65.9881 +2.0291 * p -13.7166 * q -3.7994 * r +220.6911 * roll -68.0354 * pitch +1.3988 * vn +0.1973 * ve -0.5494 * vup +20.1991 * u_thrust +1.3131 * u_roll -1.6696 * u_pitch -0.4829 * u_yaw +5.0814 * u1 +5.2596 * u2 +4.1833 * u3 +5.6747 * u4 +0.6299 * q*r +0.6295 * p*r +0.0128 * p*q +0.7005 * p*|p| +1.7127 * q*|q| -0.1690 * r*|r| -0.6215 * vn*|vn| -0.1679 * ve*|ve| +0.1496 * vup*|vup| -222.2217 * sin(roll) +65.2305 * sin(pitch) +1.2317 * sin(yaw) +1.8397 * cos(yaw) +65.5564 * cos(roll)*cos(pitch) -0.2089 * sin(pitch)*u_thrust +1.0069 * sin(roll)*u_thrust -25.2407 * cos(roll)*cos(pitch)*u_thrust +12.2433 * q*cos(roll) -1.1129 * r*sin(roll) +2.7442 * r*cos(roll) -2.3154 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.4174 | Derivative RMSE: 1.33616

d(v_up)/dt = -11.5923 +0.0372 * p -0.0429 * q +0.0609 * r -0.2584 * roll -0.3237 * pitch -0.0756 * vup +0.5375 * u_thrust +0.1122 * u1 +0.1322 * u2 +0.1522 * u3 +0.1408 * u4 +0.2548 * sin(roll) +0.2519 * sin(pitch) +1.9714 * cos(roll)*cos(pitch) +0.0265 * sin(pitch)*u_thrust +3.2542 * cos(roll)*cos(pitch)*u_thrust +0.0450 * q*cos(roll) -0.0563 * r*cos(roll) -0.0373 * p+q*sin(roll)*tan(pitch)
  └── R² Score: 0.9806 | Derivative RMSE: 0.11805

======================================================================
```

---

## 🌀 Discovered DMDc System Dynamics
```
======================================================================
>>> DMDc (DYNAMIC MODE DECOMPOSITION WITH CONTROL) REPORT
======================================================================

Identified Discrete System Matrix A (9x9):
[[ 0.9991 -0.0002  0.0204 -0.0938 -0.0043  0.0005  0.      0.0003 -0.0002]
 [ 0.0002  0.9985 -0.0025 -0.0087 -0.0681 -0.0015  0.0003 -0.0008 -0.0018]
 [-0.0001  0.0002  0.9961  0.0164 -0.0014 -0.     -0.     -0.0001  0.0001]
 [ 0.0063 -0.0024  0.019   0.9173  0.0166 -0.0012  0.0005 -0.0004 -0.0008]
 [-0.004   0.0066 -0.0072  0.015   0.9306  0.0012 -0.0005  0.0008 -0.0001]
 [ 0.0008 -0.0011  0.0026 -0.0043 -0.0016  0.9969 -0.0003 -0.0019 -0.0016]
 [ 0.0003 -0.0015 -0.006   0.     -0.0077  0.001   1.      0.0005  0.0005]
 [ 0.0018 -0.0004 -0.0006  0.007  -0.0056 -0.0002 -0.      0.9999  0.0004]
 [ 0.0002  0.0001 -0.0001 -0.0009  0.0014 -0.0004 -0.     -0.0004  1.0005]]

Identified Control Input Matrix B (9x4):
[[-0.0027 -0.0313  0.001   0.0288]
 [ 0.0218  0.0105 -0.0074 -0.0192]
 [ 0.0114 -0.0412 -0.0171  0.0473]
 [-0.0853  0.1834  0.0739 -0.1677]
 [-0.184   0.0066 -0.0072  0.1737]
 [ 0.0079  0.0107 -0.0052  0.0008]
 [-0.0008 -0.0028  0.0012 -0.0008]
 [ 0.0036  0.0058 -0.0047 -0.0041]
 [-0.0007  0.0052 -0.0031  0.0004]]

Dynamic Mode Spectrum & Stability Analysis:
Mode #   Discrete Eigenvalue (z)      Continuous Pole (s)        Freq (Hz)    Damping ζ
----------------------------------------------------------------------------------------
Mode 1   +0.9123 +0.0000j (|z|=0.9123) -22.942 +0.000j            3.651        +1.0000
Mode 2   +0.9454 +0.0000j (|z|=0.9454) -14.027 +0.000j            2.232        +1.0000
Mode 3   +0.9900 +0.0000j (|z|=0.9900) -2.505 +0.000j             0.399        +1.0000
Mode 4   +0.9932 +0.0000j (|z|=0.9932) -1.694 +0.000j             0.270        +1.0000
Mode 5   +0.9986 +0.0000j (|z|=0.9986) -0.359 +0.000j             0.057        +1.0000
Mode 6   +0.9987 +0.0000j (|z|=0.9987) -0.314 +0.000j             0.050        +1.0000
Mode 7   +1.0006 +0.0000j (|z|=1.0006) +0.145 +0.000j             0.023        -1.0000
Mode 8   +1.0000 +0.0004j (|z|=1.0000) +0.008 +0.106j             0.017        -0.0706
Mode 9   +1.0000 -0.0004j (|z|=1.0000) +0.008 -0.106j             0.017        -0.0706

======================================================================
```

---

## 🖼️ Generated Visual Artifacts
1. **3D Flight Path Benchmark**: `trajectory_3d_comparison.png`
2. **6-DOF State Tracking Comparison**: `state_time_series_comparison.png`
3. **DMDc Eigenvalue Stability Circle**: `dmdc_eigenvalue_spectrum.png`
