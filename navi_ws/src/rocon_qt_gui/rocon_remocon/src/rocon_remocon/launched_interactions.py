#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rocon_console.console as console

##############################################################################
# Class
##############################################################################


class LaunchedInteractions(object):
    def __init__(self):
        # need a mutex to protect this variable?
        self.launched_interactions = {}

    def add(self, interaction_hash, launch_name, launch_info):
        try:
            self.launched_interactions[interaction_hash][launch_name] = launch_info
        except KeyError:
            self.launched_interactions[interaction_hash] = {}
            self.launched_interactions[interaction_hash][launch_name] = launch_info

    def clear(self):
        self.launched_interactions = {}

    def active(self):
        return self.launched_interactions.keys()

    def clear_launch_details(self, interaction_hash):
        try:
            del self.launched_interactions[interaction_hash]
        except KeyError:
            return False
        return True

    def get_launch_details(self, interaction_hash):
        try:
            return self.launched_interactions[interaction_hash]
        except KeyError:
            return {}

    def remove(self, interaction_hash, launch_name):
        try:
            del self.launched_interactions[interaction_hash][launch_name]
            if not self.launched_interactions[interaction_hash]:
                del self.launched_interactions[interaction_hash]
        except KeyError:
            return False
        return True

    def __str__(self):
        s = ""
        for interaction_hash, launch_details in self.launched_interactions.iteritems():
            s += console.cyan + "  %s" % interaction_hash + console.reset + " : " + console.yellow + "%s" % launch_details.keys() + console.reset + "\n"
        return s
