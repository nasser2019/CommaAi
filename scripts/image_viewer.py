#!/usr/bin/env python3
import os
import shutil
import signal
import subprocess
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from common.basedir import BASEDIR
from selfdrive.ui.qt.python_helpers import set_main_window

#img_dir = Path(os.path.join(BASEDIR, "selfdrive/assets/training_wide"))
img_dir = Path("/data/images")


if __name__ == "__main__":
  # setup images dir + server
  if img_dir.exists():
    shutil.rmtree(img_dir)
  img_dir.mkdir()
  subprocess.Popen("servefile -u /data/images/ -p 8080", shell=True)

  signal.signal(signal.SIGINT, signal.SIG_DFL)

  app = QApplication([])
  w = QWidget()
  layout = QStackedLayout()
  w.setLayout(layout)
  set_main_window(w)

  # home page
  home_widget = QWidget()
  home_layout = QVBoxLayout()
  home_widget.setLayout(home_layout)
  home_layout.addStretch(1)
  home_layout.addWidget(QLabel("port 8080"))
  ip_label = QLabel()
  home_layout.addWidget(ip_label)
  layout.addWidget(home_widget)

  # image widget
  img = QLabel()
  img.setAlignment(Qt.AlignCenter)
  layout.addWidget(img)

  idx = 0
  def ontouch(evt):
    global idx

    imgs = [f for f in sorted(img_dir.iterdir()) if f.name.endswith(('.jpg', '.jpeg', '.png'))]
    print(imgs)
    if not len(imgs):
      return

    idx = (idx + 1) % len(imgs)
    img.setPixmap(QPixmap(str(imgs[idx].resolve())))
    layout.setCurrentWidget(img)
  img.mouseReleaseEvent = ontouch
  home_widget.mouseReleaseEvent = ontouch

  def updateip():
    ips = subprocess.check_output('hostname -I', shell=True).decode().strip()
    ips = ips.replace(' ', '   ')
    ip_label.setText(ips)
  timer = QTimer()
  timer.timeout.connect(updateip)
  timer.start(1000)

  home_widget.setStyleSheet("""
    * {
      color: white;
      font-size: 64px;
      font-family: Inter;
      background-color: black;
    }
  """)

  app.exec_()
