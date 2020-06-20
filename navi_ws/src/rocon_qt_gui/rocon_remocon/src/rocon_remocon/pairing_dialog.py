#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import division

import os
import rocon_interaction_msgs.msg as interaction_msgs
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QEvent, SIGNAL
from python_qt_binding.QtGui import QDialog, QCursor, QSpacerItem, QCheckBox


from . import icon
from . import utils

##############################################################################
# Dialog
##############################################################################


class PairingDialog(QDialog):

    def __init__(self, parent, pairing, start_pairing_hook, stop_pairing_hook, is_enabled, is_running=False):
        super(PairingDialog, self).__init__(parent)

        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'pairing_dialog.ui')
        loadUi(path, self)
        self.pairing = pairing
        self._start_pairing_hook = start_pairing_hook
        self._stop_pairing_hook = stop_pairing_hook
        self._is_enabled = is_enabled
        self._is_running = is_running
        self._init_user_interface()
        self.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)
        self.adjustSize()  # resize to contents

    def _init_user_interface(self):
        self.setWindowTitle(self.pairing.name)
        pixmap = icon.rocon_icon_to_qpixmap(self.pairing.icon)
        self.pairing_icon.setPixmap(pixmap)
        self.pairing_name.setText("<b>%s</b><br/><small>%s</small>" % (self.pairing.name, self.pairing.rapp))
        self.pairing_description.setText("<br/>" + self.pairing.description + "<br/><br/>")
        # buttons
        if self._is_running:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        elif self._is_enabled:
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
        else:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)
        self.start_button.pressed.connect(self._press_start_button)
        self.stop_button.pressed.connect(self._press_stop_button)

    def _press_start_button(self):
        response = self._start_pairing_hook(self.pairing)
        if response.result == interaction_msgs.ErrorCodes.SUCCESS:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        else:
            utils.show_message(self, str(response.result), response.message)

    def _press_stop_button(self):
        response = self._stop_pairing_hook(self.pairing)
        if response.result == interaction_msgs.ErrorCodes.SUCCESS:
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
        else:
            utils.show_message(self, str(response.result), response.message)

    def showEvent(self, event):
        geom = self.frameGeometry()
        geom.moveTopLeft(QCursor.pos())
        self.setGeometry(geom)
        super(QDialog, self).showEvent(event)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.WindowDeactivate:
            self.close()
            return True
        return False
