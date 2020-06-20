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
import sys

from qt_gui.plugin import Plugin

from rocon_console import console
from rocon_remocon.interactions_chooser import InteractionsChooserUI

##############################################################################
# Rqt Remocon
##############################################################################


class RqtRemocon(Plugin):

    def __init__(self, context):
        self._context = context
        super(RqtRemocon, self).__init__(context)
        self.rocon_master_uri = None
        self.host_name = None

        ##############################
        # Environment Variables
        ##############################
        try:
            self.rocon_master_uri = os.environ["ROS_MASTER_URI"]
        except KeyError as unused_e:
            console.logerror("ROS_MASTER_URI not set, aborting")
            sys.exit(1)

        try:
            self.host_name = os.environ["ROS_HOSTNAME"]
        except KeyError as unused_e:
            console.logwarn("ROS_HOSTNAME not set, you may drop comms across a network connection.")
            self.host_name = 'localhost'

        ##############################
        # Launch User Interface
        ##############################

        self.setObjectName('Rqt Remocon')
        self.rqt_remocon = InteractionsChooserUI(self.rocon_master_uri, self.host_name, True)
        # self._rqt_remocon = InteractiveClientUI(None, "Rqt remocon", None, self.rocon_master_uri, self.host_name, True)
        context.add_widget(self.rqt_remocon.widget)

    ##########################################################################
    # Rqt Plugin Api
    ##########################################################################

    def shutdown_plugin(self):
        """Shutdown and clean up the plugin before unloading."""
        self.rqt_remocon.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        """
        Save the intrinsic state of the plugin to the plugin-specific or instance-specific `Settings`.
        @param plugin_settings: The plugin-specific settings
        @type plugin_settings: qt_gui.settings.Settings
        @param instance_settings: The instance-specific settings
        @type instance_settings: qt_gui.settings.Settings
        """
        instance_settings.set_value('interactions_group_filter', self.rqt_remocon.widget.interactions_group_combobox.currentText())
        instance_settings.set_value('pairings_group_filter', self.rqt_remocon.widget.pairings_group_combobox.currentText())

    def restore_settings(self, plugin_settings, instance_settings):
        """
        Restore the intrinsic state of the plugin from the plugin-specific or instance-specific `Settings`.
        @param plugin_settings: The plugin-specific settings
        @type plugin_settings: qt_gui.settings.Settings
        @param instance_settings: The instance-specific settings
        @type instance_settings: qt_gui.settings.Settings
        """
        self.rqt_remocon.default_group = instance_settings.value('interactions_group_filter', "All")
        index = self.rqt_remocon.widget.interactions_group_combobox.findText(self.rqt_remocon.default_group)
        if index != -1:
            self.rqt_remocon.widget.interactions_group_combobox.setCurrentIndex(index)

        self.rqt_remocon.default_pairings_group = instance_settings.value('pairings_group_filter', "All")
        index = self.rqt_remocon.widget.pairings_group_combobox.findText(self.rqt_remocon.default_pairings_group)
        if index != -1:
            self.rqt_remocon.widget.pairings_group_combobox.setCurrentIndex(index)
