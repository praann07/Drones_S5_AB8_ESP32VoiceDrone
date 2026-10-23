import pandas as pd
import numpy as np
import pysindy as ps

# 1. Load flight dataset recorded by data_logger.py
data = pd.read_csv('flight_data.csv')

# Remove duplicate timestamps if present
data = data.drop_duplicates(subset=['timestamp']).reset_index(drop=True)

# 2. Extract states (x) and control inputs (u)
state_names = ['roll', 'pitch', 'yaw', 'vx', 'vy', 'vz']
control_names = ['ax', 'ay', 'az']
all_feature_names = state_names + control_names

X = data[state_names].values
U = data[control_names].values
t = data['timestamp'].values

# Calculate average time step
dt = np.mean(np.diff(t))

# 3. Configure Feature Library & Optimizer
# Reduced threshold (0.001) prevents wiping out small dynamic coefficients
feature_library = ps.PolynomialLibrary(degree=2, include_interaction=True)
optimizer = ps.STLSQ(threshold=0.001, alpha=0.05)

# 4. Instantiate SINDy Model
model = ps.SINDy(
    feature_library=feature_library,
    optimizer=optimizer,
    differentiation_method=ps.SmoothedFiniteDifference()
)

# 5. Fit model with combined feature names (states + inputs)
model.fit(X, u=U, t=dt, feature_names=all_feature_names)

# 6. Print discovered governing dynamics
print("\n" + "="*50)
print("     DISCOVERED GOVERNING DYNAMICS (SINDy)")
print("="*50 + "\n")
model.print()
