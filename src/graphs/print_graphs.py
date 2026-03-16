from collections import defaultdict
import os
import re
import glob
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

LOG_DIR = "src/logs"

time_data = []
s_data = []

def find_latest_file():
    files = glob.glob(os.path.join(LOG_DIR, "*"))
    if not files:
        return None
    return max(files, key=os.path.getmtime)

log_file = find_latest_file()

if log_file is None:
    print("No log files found")
    exit()

print("Using log file:", log_file)

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle("Realtime log plot")

plot = win.addPlot(title="s(t)")
plot.setLabel("left", "s")
plot.setLabel("bottom", "time", units="mins")
plot.addLegend()

curve_at = plot.plot(pen='r', name="a(t)")
curve_st = plot.plot(pen='r', name="s(t)")
curve_nt = plot.plot(pen='g', name="n(t)")

def update():
    global time_data, s_data

    try:
        with open(log_file, "r") as f:
            lines = f.readlines()
            
        columns = defaultdict(list)

        for line in lines:
          line_s = line.strip().replace(":", "").split(" ")
          
          for columnIndex, value in enumerate(line_s):
            if (columnIndex % 2 != 0):
              continue
            
            columnName = value
            columnValue = float(line_s[columnIndex + 1])
            
            if (columnIndex == 4):
              columnName = columnName * 2
            
            columns[columnName].append(columnValue)

        columns["t"] = np.array(columns["t"]) - columns["t"][0]
        columns["t"] = columns["t"] / 1000.0 / 60.0
        curve_at.setData(columns["t"], columns["tt"])
        curve_st.setData(columns["t"], columns["s"])
        curve_nt.setData(columns["t"], columns["n"])

    except Exception as e:
        print("read error:", e)

# timer = QtCore.QTimer()
# timer.timeout.connect(update)
# timer.start(1000)

update()

app.exec()