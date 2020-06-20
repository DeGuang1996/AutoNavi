#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import division
import os

from python_qt_binding import loadUi

# QtCore
#    SIGNAL, SLOT,  pyqtSlot, pyqtSignal
#    QString, QStringList,
#    QPoint, QEvent, QFile, QIODevice,
#    QAbstractListModel
#from python_qt_binding.QtCore import Qt, QAbstractListModel
# QtGui
#    QMainWindow,
#    QCheckBox, QComboBox,
#    QTextEdit, QCompleter,
#    QFileDialog,
#    QPushButton
#    QGraphicsScene, QImage, QPainter, QBrush, QColor, QPen, QPixmap
from python_qt_binding.QtGui import QWidget, QPixmap
# QtSvg
#    QSvgGenerator

from qt_gui.plugin import Plugin

import rospkg
import rocon_master_info

##############################################################################
# Plugin
##############################################################################


class MasterInfo(Plugin):

    def __init__(self, context):
        super(MasterInfo, self).__init__(context)
        self.initialised = False
        self.setObjectName('MasterInfo')
        self._current_dotcode = None

        self._master_info = rocon_master_info.get_master_info(0.5)  # at worst a small timeout here, but perhaps should be run in a thread.

        self._widget = QWidget()
        self._widget.setObjectName('RoconMasterInfoUi')
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_qt_master_info'), 'ui', 'master_info.ui')
        loadUi(ui_file, self._widget)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        pixmap = QPixmap()
        pixmap.loadFromData(self._master_info.icon.data, format=self._master_info.icon.format)
        self._widget.icon_label.setPixmap(pixmap)
        self._widget.icon_label.resize(pixmap.width(), pixmap.height())

        self._widget.info_label.resize(200, pixmap.height())
        self._widget.info_label.setText("<b>Name:</b> %s<br/><b>Rocon Uri:</b> %s<br/><b>Rocon Version:</b> %s<br/><b>Description:</b> %s" % (self._master_info.name, self._master_info.rocon_uri, self._master_info.version, self._master_info.description))
        self._widget.adjustSize()

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass
