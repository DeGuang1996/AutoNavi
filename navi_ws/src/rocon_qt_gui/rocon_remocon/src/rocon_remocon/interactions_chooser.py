#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import copy
import os
from rocon_console import console
import rospkg
import rospy

# PySide is LGPL, pyqt is GPL, python_qt_bindings are ROS bindings around both
# We prefer LGPL, so convert over later so we can get proper indexing
from python_qt_binding import loadUi  # what is PySide's equivalent?
from python_qt_binding.QtCore import Qt, QSize, QEvent, Slot
from python_qt_binding.QtGui import QListView, QWidget, QStandardItemModel  # QIcon, QColor, QMainWindow, QMessageBox
# from PySide.QtUiTools import QUiLoader
# from PySide.QtCore import QSize, Slot, QFile
# from PySide.QtGui import QListView, QWidget, QStandardItemModel  # QIcon, QColor, QMainWindow, QMessageBox

from . import utils
from . import icon
from .interactions_remocon import InteractionsRemocon
from .pairing_dialog import PairingDialog
from .interaction_dialog import InteractionDialog

##############################################################################
# Remocon
##############################################################################


class InteractionsChooserUI():

    def __init__(self, rocon_master_uri='localhost', host_name='localhost', with_rqt=False):
        self.with_rqt = with_rqt
        self.widget = QWidget()
        self.pairings_view_model = QStandardItemModel()
        self.interactions_view_model = QStandardItemModel()
        self.interactions_remocon = InteractionsRemocon(rocon_master_uri, host_name)
        self.interactions_remocon.connect([self.update_group_combobox, self.refresh_grids])
        self.interactions_remocon.connect([self.update_pairings_group_combobox, self.refresh_grids])
        self.default_group = "All"

        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'interactions_chooser.ui')
        loadUi(ui_file, self.widget, {})

        # create a few directories for caching icons and ...
        utils.setup_home_dirs()
        self._init_ui()
        self._init_events()

    def shutdown(self):
        self.interactions_remocon.shutdown()

    @Slot()
    def update_group_combobox(self):
        """
        The underyling ros part of the remocon might get fresh data about the group list.
        Connect to this slot to update the combobox in the ui.
        """
        new_groups = copy.copy(self.interactions_remocon.interactions_table.groups())
        new_groups = [g for g in new_groups if g != "Hidden"]

        # did the underlying groups change - if so, update the combobox
        current_group = self.widget.interactions_group_combobox.currentText()
        current_size = self.widget.interactions_group_combobox.count()
        target_group = current_group if current_size != 1 else self.default_group
        current_group_list = [self.widget.interactions_group_combobox.itemText(i) for i in range(self.widget.interactions_group_combobox.count())]
        if set(current_group_list) != set(['All'] + new_groups):
            self.widget.interactions_group_combobox.clear()
            self.widget.interactions_group_combobox.addItems(['All'] + new_groups)
            index = self.widget.interactions_group_combobox.findText(target_group)
            if index != -1:
                self.widget.interactions_group_combobox.setCurrentIndex(index)
            self.refresh_grids()

    @Slot()
    def update_pairings_group_combobox(self):
        """
        The underyling ros part of the remocon might get fresh data about the group list.
        Connect to this slot to update the combobox in the ui.
        """
        new_groups = copy.copy(self.interactions_remocon.pairings_table.groups())

        # did the underlying groups change - if so, update the combobox
        current_group = self.widget.pairings_group_combobox.currentText()
        current_size = self.widget.pairings_group_combobox.count()
        target_group = current_group if current_size != 1 else self.default_pairings_group
        current_group_list = [self.widget.pairings_group_combobox.itemText(i) for i in range(self.widget.pairings_group_combobox.count())]
        if set(current_group_list) != set(['All'] + new_groups):
            self.widget.pairings_group_combobox.clear()
            self.widget.pairings_group_combobox.addItems(['All'] + new_groups)
            index = self.widget.pairings_group_combobox.findText(target_group)
            if index != -1:
                self.widget.pairings_group_combobox.setCurrentIndex(index)
            self.refresh_grids()

    @Slot()
    def refresh_grids(self):
        """
        This just does a complete redraw of the interactions with the
        currently selected role. It's a bit brute force doing this
        every time the interactions' 'state' changes, but this suffices for now.
        """
        self.pairings_view_model.clear()
        self.interactions_view_model.clear()

        active_pairing = copy.copy(self.interactions_remocon.active_pairing)
        group = self.widget.pairings_group_combobox.currentText()
        for p in self.interactions_remocon.pairings_table.sorted():
            if group != "All" and p.group != group:
                continue
            is_running = False
            enabled = False
            if active_pairing is not None and p.name == active_pairing.name:
                is_running = True
                enabled = True
            elif active_pairing is None:
                enabled = True
                # enabled = not p.requires_interaction
            item = icon.QModelIconItem(p, enabled=enabled, running=is_running)
            self.pairings_view_model.appendRow(item)

        group = self.widget.interactions_group_combobox.currentText()
        for i in self.interactions_remocon.interactions_table.sorted():
            if group != "All" and i.group != group:
                continue
            if i.hidden:
                continue
            extra_tooltip_info = ""
            if i.required_pairings:
                extra_tooltip_info += " Requires "
                for required_pairing in i.required_pairings:
                    extra_tooltip_info += "'" + required_pairing + "', "
                extra_tooltip_info = extra_tooltip_info.rstrip(', ')
                if not i.bringup_pairing:
                    extra_tooltip_info += " to be running"
                extra_tooltip_info += "."
            item = icon.QModelIconItem(i,
                                       enabled=self._is_interaction_enabled(i),
                                       running=self._is_interaction_running(i),
                                       extended_tooltip_info=extra_tooltip_info
                                       )
            self.interactions_view_model.appendRow(item)

    def _init_ui(self):
        self.widget.pairings_grid.setViewMode(QListView.IconMode)
        self.widget.pairings_grid.setModel(self.pairings_view_model)
        self.widget.pairings_grid.setWordWrap(True)
        self.widget.pairings_grid.setWrapping(True)
        # really need to get away from listview, or subclass it if we want to control better how many lines of text show up
        # self.widget.pairings_grid.setTextElideMode(Qt.ElideNone)
        self.widget.pairings_grid.setIconSize(QSize(60, 60))
        self.widget.pairings_grid.setSpacing(10)
        self.widget.interactions_grid.setViewMode(QListView.IconMode)
        self.widget.interactions_grid.setModel(self.interactions_view_model)
        self.widget.interactions_grid.setWordWrap(True)
        self.widget.interactions_grid.setWrapping(True)
        self.widget.interactions_grid.setIconSize(QSize(60, 60))
        self.widget.interactions_grid.setSpacing(10)
        for ns in self.interactions_remocon.namespaces:
            self.widget.namespace_checkbox.addItem(ns)
        self.refresh_grids()
        self.widget.pairings_group_combobox.addItems(['All'] + self.interactions_remocon.pairings_table.groups())
        self.widget.interactions_group_combobox.addItems(['All'] + self.interactions_remocon.interactions_table.groups())
        # TODO namespace checkbox to self.interactions_remocon.active_namespace

    ##############################################################################
    # Private
    ##############################################################################

    def _init_events(self):
        self.widget.namespace_checkbox.currentIndexChanged.connect(self._event_change_namespace)
        self.widget.pairings_grid.clicked.connect(self._pairing_single_click)
        self.widget.interactions_grid.clicked.connect(self._interaction_single_click)
        self.widget.button_stop_all_interactions.clicked.connect(self.interactions_remocon.stop_all_interactions)
        self.widget.pairings_group_combobox.currentIndexChanged.connect(self.refresh_grids)
        self.widget.interactions_group_combobox.currentIndexChanged.connect(self.refresh_grids)

    def _event_change_namespace(self):
        rospy.logwarn("Remocon : changing interaction managers is currently not supported.")

    def _pairing_single_click(self, index):
        pairing_item = self.pairings_view_model.item(index.row())
        pairing = pairing_item.implementation
        self._create_pairing_dialog(pairing)

    def _create_pairing_dialog(self, pairing):
        active_pairing = copy.copy(self.interactions_remocon.active_pairing)
        if active_pairing is not None:
            is_running = (active_pairing.name == pairing.name)
            is_enabled = is_running
        else:
            is_enabled = True  # not pairing.requires_interaction
            is_running = False
        self.selected_pairing = pairing
        self.dialog = PairingDialog(self.widget,
                                    pairing,
                                    self.interactions_remocon.start_pairing,
                                    self.interactions_remocon.stop_pairing,
                                    is_enabled,
                                    is_running
                                    )
        self.dialog.show()

    def _interaction_single_click(self, index):
        interaction_item = self.interactions_view_model.item(index.row())
        interaction = interaction_item.implementation
        self._create_interaction_dialog(interaction)

    def _create_interaction_dialog(self, interaction):
        self.selected_interaction = interaction
        self.dialog = InteractionDialog(self.widget,
                                        interaction,
                                        self.interactions_remocon.start_interaction,
                                        self.interactions_remocon.stop_interaction,
                                        self._is_interaction_enabled(interaction),
                                        self._is_interaction_running(interaction),
                                        self._is_interaction_permitted_new_launches(interaction)
                                        )
        self.dialog.show()

    def _is_interaction_permitted_new_launches(self, interaction):
        current_number_of_launches = len(self.interactions_remocon.launched_interactions.get_launch_details(interaction.hash))
        if interaction.max == -1 or current_number_of_launches < interaction.max:
            return True
        else:
            return False

    def _is_interaction_enabled(self, interaction):
        active_pairing = copy.copy(self.interactions_remocon.active_pairing)
        enabled = True
        if interaction.required_pairings:
            if active_pairing and active_pairing.name not in interaction.required_pairings:
                enabled = False
            elif active_pairing is None:
                if not interaction.bringup_pairing:
                    enabled = False
                else:
                    available_required_pairings = [name for name in interaction.required_pairings if self.interactions_remocon.pairings_table.find(name) is not None]
                    if not available_required_pairings:
                        enabled = False
        return enabled

    def _is_interaction_running(self, interaction):
        return True if self.interactions_remocon.launched_interactions.get_launch_details(interaction.hash) else False
