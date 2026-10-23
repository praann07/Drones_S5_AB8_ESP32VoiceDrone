import numpy as np, pandas as pd, pysindy as ps, sys
d = pd.read_csv(sys.argv[1]).drop_duplicates("timestamp").reset_index(drop=True)
t = d.timestamp.values; dt = np.median(np.diff(t)); print("rows", len(d), "median dt %.4f s (%.1f Hz)"%(dt,1/dt))
r2 = lambda y,yh: 1-np.sum((y-yh)**2)/np.sum((y-y.mean())**2)
# 1) attitude: roll,pitch (rad) with body rates p,q (rad/s)
X = np.c_[np.deg2rad(d.roll), np.deg2rad(d.pitch), d.p, d.q]
names = ["roll","pitch","p","q"]
n = int(len(X)*0.7)
diff = ps.SmoothedFiniteDifference(smoother_kws={"window_length":9})
for deg,thr in [(1,0.05),(1,0.2),(2,0.2)]:
    m = ps.SINDy(feature_library=ps.PolynomialLibrary(degree=deg), optimizer=ps.STLSQ(threshold=thr, alpha=0.05),
                 differentiation_method=ps.SmoothedFiniteDifference(smoother_kws={"window_length":9}))
    m.fit(X[:n], t=dt, feature_names=names)
    Xd = diff._differentiate(X[n:], dt); pred = m.predict(X[n:])
    print(f"\n--- attitude, degree {deg}, threshold {thr}: nonzero terms {int((m.coefficients()!=0).sum())}")
    m.print()
    print("held-out one-step R2 per state:", [round(r2(Xd[:,i],pred[:,i]),3) for i in range(4)])
# 2) vertical: vz' = g + az  ?
vz=d.vz.values; az=d.az.values
vzd = diff._differentiate(vz.reshape(-1,1), dt)[:,0]
A = np.c_[np.ones_like(az), az]; coef,*_ = np.linalg.lstsq(A[:n], vzd[:n], rcond=None)
print("\nvz' = %.3f + %.3f*az   held-out R2 %.3f"%(coef[0],coef[1], r2(vzd[n:], A[n:]@coef)))
print("yaw range", d.yaw.min(), d.yaw.max(), " fraction of steps with |dyaw|>180:", np.mean(np.abs(np.diff(d.yaw))>180))
