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

import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QEvent, SIGNAL
from python_qt_binding.QtGui import QDialog, QCursor, QSpacerItem, QCheckBox

import rocon_interaction_msgs.msg as interaction_msgs

from . import icon
from . import utils

##############################################################################
# Dialog
##############################################################################


class InteractionDialog(QDialog):

    def __init__(self,
                 parent,
                 interaction,
                 start_interaction_hook,
                 stop_interaction_hook,
                 is_enabled,
                 is_running=False,
                 permit_new_launches=False
                 ):
        super(InteractionDialog, self).__init__(parent)

        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'interaction_dialog.ui')
        loadUi(path, self)
        self.interaction = interaction
        self._start_interaction_hook = start_interaction_hook
        self._stop_interaction_hook = stop_interaction_hook
        self._is_enabled = is_enabled
        self._is_running = is_running
        self._permit_new_launches = permit_new_launches
        self._init_user_interface()
        self.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)
        self.adjustSize()  # resize to contents

    def _init_user_interface(self):
        self.setWindowTitle(self.interaction.name)
        pixmap = icon.rocon_icon_to_qpixmap(self.interaction.icon)
        self.icon.setPixmap(pixmap)
        self.name.setText("<b>%s</b><br/><small>%s</small>" % (self.interaction.name, self.interaction.command))
        s = "<br/>" + self.interaction.description
        if self.interaction.required_pairings:
            s += " Requires "
            for required_pairing in self.interaction.required_pairings:
                s += "'" + required_pairing + "', "
            s = s.rstrip(', ')
            if not self.interaction.bringup_pairing:
                s += " to be running"
            s += "."
        self.description.setText(s + "<br/><br/>")
        # buttons
        if not self._is_enabled:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)
        else:
            self.stop_button.setEnabled(self._is_running)
            self.start_button.setEnabled(self._permit_new_launches)
        self.start_button.pressed.connect(self._press_start_button)
        self.stop_button.pressed.connect(self._press_stop_button)

    def _press_start_button(self):
        (result, message) = self._start_interaction_hook(self.interaction.hash)
        if result:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        else:
            utils.show_message(self, str(result), message)

    def _press_stop_button(self):
        (result, message) = self._stop_interaction_hook(self.interaction.hash)
        if result:
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
        else:
            utils.show_message(self, str(result), message)

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
