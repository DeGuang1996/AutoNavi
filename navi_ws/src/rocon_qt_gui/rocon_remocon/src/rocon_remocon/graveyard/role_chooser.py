#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
from rocon_console import console
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, QSize
from python_qt_binding.QtGui import QWidget


##############################################################################
# Remocon
##############################################################################


class QRoleChooser():
    # pyqt signals are always defined as class attributes
    # signal_interactions_updated = Signal()

    def __init__(self, interactive_client_interface=None, with_rqt=False):

        self.interactive_client_interface = interactive_client_interface
        self.with_rqt = with_rqt
        self.binded_function = {}
        self.role_list = []
        self.cur_selected_role = ''
        self.roles_widget = QWidget()
        # load ui
        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'role_list.ui')
        loadUi(path, self.roles_widget)

        # connect ui event role list widget
        self.roles_widget.role_list_widget.setIconSize(QSize(50, 50))
        self.roles_widget.role_list_widget.itemDoubleClicked.connect(self._select_role)
        self.roles_widget.refresh_btn.pressed.connect(self.refresh_role_list)
        self.roles_widget.back_btn.pressed.connect(self._back)
        self.roles_widget.stop_all_interactions_button.pressed.connect(self._stop_all_interactions)
        self.roles_widget.closeEvent = self._close_event
        self._init()

    def _init(self):
        """
        Initialization of role chooser. If it is launched with rqt, the back button is disabled.
        Viewer of interactions chooser is launched at once when the role list has one role.
        """
        if self.with_rqt:
            self.roles_widget.back_btn.setEnabled(False)
        self.refresh_role_list()
        if len(self.role_list) == 1:
            self.cur_selected_role = self.role_list[0]
            self.interactive_client_interface.get_runnable_interactions_list(self.cur_selected_role)

    def _back(self):
        """
        Public method to enable shutdown of the script - this function is primarily for
        shutting down the Role chooser from external signals (e.g. CTRL-C on the command
        line).
        """
        console.logdebug("Role Chooser : Back")
        if 'back' in self.binded_function.keys() and self.binded_function['back'] is not None:
            self.binded_function['back']()

    def _close_event(self, event):
        """
        Re-implementation of close event handlers for the interaction chooser's children
        (i.e. role and interactions list widgets).
        """
        console.logdebug("Role Chooser : Role Chooser shutting down.")
        self._back()

    def _select_role(self, item):
        """
        Take the selected role to switch interactions viewer as it.

        :param item: qt list widget item of selected role. The user does double click on item wanted to launch
        :type item: python_qt_binding.QtGui.QListWidgetItem
        """
        console.logdebug("Role Chooser : switching to the interactions list")
        self.cur_selected_role = str(item.text())
        if 'select_role' in self.binded_function.keys() and self.binded_function['select_role'] is not None:
            self.binded_function['select_role']()

    def _stop_all_interactions(self):
        """
        Stopping all running interactions. If no interactions is running, stop interactions button is disables.
        """
        console.logdebug("Role Chooser : stopping all running interactions")
        self.interactive_client_interface.stop_all_interactions()
        self.roles_widget.stop_all_interactions_button.setEnabled(False)

    def bind_function(self, name, function_handle):
        """
        Binding external function to map with ui button
        """
        self.binded_function[name] = function_handle

    def show(self, pos=None):
        """
        Showing the role chooser with rereshing role list
        """

        self.roles_widget.show()
        if pos is not None:
            self.roles_widget.move(pos)
        self.refresh_role_list()

    def hide(self):
        """
        Hiding the role chooser to show other widget
        """
        self.roles_widget.hide()

    def pos(self):
        """
        Postion of role chooser

        :return: xy position on desktop
        :rtype: python_qt_binding.QtCore.QPoint
        """
        return self.roles_widget.pos()

    def refresh_role_list(self):
        """
        Update a list of roles. define status of all interaction stop button as checking running interactions.
        """
        if self.interactive_client_interface.has_running_interactions():
            self.roles_widget.stop_all_interactions_button.setEnabled(True)
        else:
            self.roles_widget.stop_all_interactions_button.setEnabled(False)

        self.roles_widget.role_list_widget.clear()
        self.role_list = self.interactive_client_interface.get_role_list()
        # set list widget item (reverse order because we push them on the top)
        for role in reversed(self.role_list):
            self.roles_widget.role_list_widget.insertItem(0, role)
            # setting the list font
            font = self.roles_widget.role_list_widget.item(0).font()
            font.setPointSize(13)
            self.roles_widget.role_list_widget.item(0).setFont(font)
