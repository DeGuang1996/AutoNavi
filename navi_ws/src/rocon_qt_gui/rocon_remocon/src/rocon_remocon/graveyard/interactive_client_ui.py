#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Qt, QSize, QEvent
from python_qt_binding.QtGui import QIcon, QWidget, QMainWindow, QVBoxLayout, QMessageBox

from rocon_console import console

from . import utils
from .interactive_client_interface import InteractiveClientInterface
from .master_chooser import QMasterChooser
from .role_chooser import QRoleChooser
from .interactions_chooser import QInteractionsChooser

##############################################################################
# Interactive Client UI
##############################################################################


class InteractiveClientUI(QMainWindow):

    # pyqt signals are always defined as class attributes
    signal_interactions_updated = Signal()

    def __init__(self, parent, title, application, rocon_master_uri='localhost', host_name='localhost', with_rqt=False):
        super(InteractiveClientUI, self).__init__(parent)
        self.rocon_master_uri = rocon_master_uri
        self.host_name = host_name
        self.with_rqt = with_rqt
        self.cur_selected_interaction = None
        self.cur_selected_role = 0
        self.interactions = {}

        # desktop taskbar icon
        self.application = application
        if self.application:
            rospack = rospkg.RosPack()
            icon_file = os.path.join(rospack.get_path('rocon_icons'), 'icons', 'rocon_logo.png')
            self.application.setWindowIcon(QIcon(icon_file))

        # create a few directories for caching icons and ...
        utils.setup_home_dirs()

        # connect to the ros master with init node
        self.interactive_client_interface = InteractiveClientInterface(stop_interaction_postexec_fn=lambda: self.signal_interactions_updated.emit())

        if self.with_rqt:
            (result, message) = self.interactive_client_interface._connect(self.rocon_master_uri, self.host_name)
        else:
            (result, message) = self.interactive_client_interface._connect_with_ros_init_node(self.rocon_master_uri, self.host_name)

        if not result:
            QMessageBox.warning(self, 'Connection Failed', "%s." % message.capitalize(), QMessageBox.Ok)
            self._switch_to_master_chooser()
            return

        # interactive_client_ui widget setting
        self._interactive_client_ui_widget = QWidget()
        self._interactive_client_ui_layout = QVBoxLayout()

        self._role_chooser = QRoleChooser(self.interactive_client_interface, with_rqt)
        self._role_chooser.bind_function('shutdown', self._switch_to_master_chooser)
        self._role_chooser.bind_function('back', self._switch_to_master_chooser)
        self._role_chooser.bind_function('select_role', self._switch_to_interactions_list)

        self._interactions_chooser = QInteractionsChooser(self.interactive_client_interface)
        self._interactions_chooser.bind_function('shutdown', self._switch_to_master_chooser)
        self._interactions_chooser.bind_function('back', self._switch_to_role_list)

        self._interactive_client_ui_layout.addWidget(self._interactions_chooser.interactions_widget)
        self._interactive_client_ui_layout.addWidget(self._role_chooser.roles_widget)
        self._interactive_client_ui_widget.setLayout(self._interactive_client_ui_layout)

        self.signal_interactions_updated.connect(self.interactions_updated_relay)

        self._init()

    def _init(self):
        """
        Initialization of interactive client UI. First, show the role chooser and hide interactions chooser.
        """
        if (len(self._role_chooser.role_list) != 1):
            self._interactive_client_ui_widget.setWindowTitle('Role Chooser')
            self._interactive_client_ui_widget.show()
            self._role_chooser.show()
            self._interactions_chooser.hide()
        else:
            console.logdebug("interface skipping the role chooser and switching directly to the interactions list [exactly one role available]")
            self._switch_to_interactions_list()

    def get_main_ui_handle(self):
        """
        Returning an instance of interactive client ui widget. It is used to handle it in other widget.

        :return: return main qt widget.
        :rtype: python_qt_binding.QtGui.QWidget
        """
        return self._interactive_client_ui_widget

    def shutdown(self):
        """
        Public method to enable shutdown of the script - this function is primarily for
        shutting down the InteractiveClientUI from external signals (e.g. CTRL-C on the command
        line).
        """
        self.interactive_client_interface.shutdown()

    def _switch_to_master_chooser(self):
        """
        Switch to master chooser from role chooser. If it was launced by rqt or rqt standalone, It is just shutdown.
        """
        self.shutdown()
        if not self.with_rqt:
            console.logdebug("InteractiveClientUI : switching back to the master chooser")
            os.execv(QMasterChooser.rocon_remocon_script, ['', self.host_name])

    def _switch_to_interactions_list(self):
        """
        Take the selected role and switch to an interactions view of that role.
        """
        console.logdebug("InteractiveClientUI : switching to the interactions list")
        self._interactive_client_ui_widget.setWindowTitle('Interactions Chooser')
        self._interactions_chooser.select_role(self._role_chooser.cur_selected_role)
        self._interactions_chooser.show(self._role_chooser.pos())
        self._role_chooser.hide()

    def _switch_to_role_list(self):
        """
        Switch to role chooser from interactions chooser.
        """
        self._interactive_client_ui_widget.setWindowTitle('Role Chooser')
        console.logdebug("InteractiveClientUI : switching to the role list")
        self._role_chooser.show(self._interactions_chooser.interactions_widget.pos())
        self._interactions_chooser.hide()

    def interactions_updated_relay(self):
        """
        Called by the underlying interactive client whenever the gui needs to be updated with
        fresh information. Using this relay saves us from having to embed qt functions in the
        underlying class but makes sure we signal across threads so the gui can update things
        in its own thread.

        Currently this only handles updates caused by termination of an interaction. If we wished to
        handle additional situations, we should use an argument here indicating what kind of interaction
        update occurred.
        """
        console.logdebug("InteractiveClientUI : interactions_updated_relay")
        self._interactions_chooser.refresh_interactions_list()
        self._role_chooser.refresh_role_list()
