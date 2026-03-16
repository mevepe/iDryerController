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

plot = win.addPlot(title="a(t), s(t), n(t)")
plot.setTitle("a(t), s(t), n(t)", color='w')   # заголовок чёрный

plot.setLabel('left',  text='values', color='w')
plot.setLabel('bottom', text='time', units='mins', color='w')

plot.addLegend(labelTextColor='k')             # легенда чёрная

plot.getViewBox().setBackgroundColor('w')

plot.showGrid(x=True, y=True, alpha=0.5)

# black_pen = pg.mkPen(color='k', width=1)
# plot.getAxis('bottom').setPen(black_pen)
# plot.getAxis('left').setPen(black_pen)
# plot.getAxis('bottom').setTextPen(black_pen)
# plot.getAxis('left').setTextPen(black_pen)
# plot.getAxis('bottom').gridPen = black_pen
# plot.getAxis('left').gridPen = black_pen

curve_at = plot.plot(pen=pg.mkPen('r', width=3), name="a(t)")
curve_st = plot.plot(pen=pg.mkPen('g', width=3), name="s(t)")
curve_nt = plot.plot(pen=pg.mkPen('b', width=3), name="n(t)")

def update():
    global time_data, s_data

    try:
        with open(log_file, "r") as f:
          lines = f.readlines()
            
        columns = defaultdict(list)
        first_line = lines[0].strip().replace(":", "").split(" ")
        columnsCount = len(first_line)

        for line in lines:
          line_s = line.strip().replace(":", "").split(" ")
          
          if (len(line_s) != columnsCount):
            continue
          
          for columnIndex, value in enumerate(line_s):
            if (columnIndex % 2 != 0):
                continue
              
            columnName = value
            columnValue = float(line_s[columnIndex + 1])
              
            columns[columnName].append(columnValue)
              
        columns["t"] = np.array(columns["t"]) - columns["t"][0]
        columns["t"] = columns["t"] / 1000.0 / 60.0
        curve_at.setData(columns["t"], columns["at"])
        curve_st.setData(columns["t"], columns["s"])
        curve_nt.setData(columns["t"], columns["n"])

    except Exception as e:
        print("read error:", e)

update()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1000)

app.exec()