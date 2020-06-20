#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QIcon, QPixmap, QStandardItem, QFont, QColor

##############################################################################
# Classes
##############################################################################


def rocon_icon_to_qpixmap(icon):
    """
    :param rocon_std_msgs.Icon icon: icon to use for the pixmap
    """
    pixmap = QPixmap()
    pixmap.loadFromData(icon.data, format=icon.format)
    return pixmap


def rocon_icon_to_qicon(icon):
    """
    :param rocon_std_msgs.Icon icon: icon to use for the pixmap
    """
    pixmap = QPixmap()
    pixmap.loadFromData(icon.data, format=icon.format)
    return QIcon(pixmap)


class QModelIconItem(QStandardItem):
    def __init__(self, implementation, enabled, running, extended_tooltip_info=""):
        """
        :param implementation: one of either rocon_interactions.Pairing or rocon_interactions.Interaction
        :param bool running:
        """
        QStandardItem.__init__(self, implementation.name)
        self.setSizeHint(QSize(100, 100))
        self.setIcon(rocon_icon_to_qicon(implementation.icon))
        f = QFont()
        f.setPointSize(8)
        self.setFont(f)
        self.setToolTip(implementation.description + extended_tooltip_info)
        self.setEditable(False)
        self.setEnabled(enabled)
        if running:
            self.setBackground(QColor(100, 100, 150))
        self.implementation = implementation
