import pandas as pd
import numpy as np

reportFile = "./delayMeasureResult.csv"
frame = pd.read_csv(reportFile)
print("\n// ==== Report summary ==== //\n")
print(frame.describe())

# Measure the average length between observed numbers
# Theoratically, the camera takes picture at 30 FPS, exposure time of around 1 ms and the LED array updates every 2 ms, the difference should be around 15 - 16 units
value_diff = pd.DataFrame({
    "Cam_0_value_diff": np.diff(frame["Camera 0 Value"], prepend=frame["Camera 0 Value"][0]),
    "Cam_1_value_diff": np.diff(frame["Camera 1 Value"], prepend=frame["Camera 0 Value"][1])
})
print("\n// ==== Value diff summary ==== //\n")
print(value_diff.describe())
