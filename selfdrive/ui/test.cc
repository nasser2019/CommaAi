#include <sys/resource.h>

#include <QApplication>
#include <QSslConfiguration>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/window.h"

#include <QDebug>

int main(int argc, char *argv[]) {
  initApp(argc, argv);

  if (Hardware::EON()) {
    QSslConfiguration ssl = QSslConfiguration::defaultConfiguration();
    ssl.setCaCertificates(QSslCertificate::fromPath("/usr/etc/tls/cert.pem"));
    QSslConfiguration::setDefaultConfiguration(ssl);
  }

  QApplication a(argc, argv);
  MainWindow w;
  setMainWindow(&w);

  QTimer::singleShot(5000, &w, [&]() {
    for (QWidget *o : w.findChildren<QWidget *>()) {
      double start = millis_since_boot();
      o->repaint();
      double t = millis_since_boot() - start;
      if (t > 1) {
        qDebug() << o->metaObject()->className() << "  " << t << "ms";
        for (int i = 0; i < 5; i++) {
          start = millis_since_boot();
          o->repaint();
          t = millis_since_boot() - start;
          qDebug() << "  " << t;
        }
      }
    }
  });

  a.installEventFilter(&w);
  return a.exec();
}
