#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: interactions_table
   :platform: Unix
   :synopsis: A database of interactions.


This module provides a class that acts as a database (dictionary style) of
some set of interactions.

----

"""
##############################################################################
# Imports
##############################################################################

import rocon_console.console as console

##############################################################################
# Classes
##############################################################################


class InteractionsTable(object):
    '''
      The runtime populated interactions table along with methods to
      manipulate it.

      .. include:: weblinks.rst
    '''
    __slots__ = [
        'interactions',  # rocon_interactions.interactions.Interaction[]
    ]

    def __init__(self, filter_pairing_interactions=False):
        """
        Constructs an empty interactions table.

        :param bool filter_pairing_interactions: do not load any paired interactions
        """
        self.interactions = []
        """List of :class:`.Interaction` objects that will form the elements of the table."""

    def roles(self):
        '''
          List all roles for the currently stored interactions.

          :returns: a list of all roles
          :rtype: str[]
        '''
        # uniquify the list
        return list(set([i.role for i in self.interactions]))

    def __len__(self):
        return len(self.interactions)

    def __str__(self):
        """
        Convenient string representation of the table.
        """
        s = ''
        for interaction in self.interactions:
            s += "\n".join("  " + i for i in str(interaction).splitlines()) + '\n'
        return s

    def sorted(self):
        """
        Return the pairing list sorted by name.
        """
        return sorted(self.interactions, key=lambda interaction: interaction.name)

    def generate_group_view(self, group_name):
        '''
        Creates a temporary copy of interactions filtered by the specified group
        and sorts them into a dictionary view keyed by hash. This is a convenient
        object for use by the interactions chooser.

        :param str role_name: the filter for retrieving interactions

        :returns: A role based view of the interactions
        :rtype: dict { hash : :class:`.interactions.Interaction` }
        '''
        # there's got to be a faster way of doing this.
        group_view = {}
        for interaction in self.interactions:
            if interaction.group == group_name:
                group_view[interaction.hash] = interaction
        return group_view

    def clear(self, role_name):
        """
        Clear all interactions belonging to this role.

        :param str role_name:
        """
        self.interactions[:] = [i for i in self.interactions if i.role != role_name]

    def append(self, interaction):
        """
        Append an interaction to the table.

        :param :class:`.Interaction` interaction:
        """
        matches = [i for i in self.interactions if i.hash == interaction.hash]
        if not matches:
            self.interactions.append(interaction)
        else:
            console.logdebug("Interactions Table : interaction already stored, not appending to the table [%s]" % interaction.hash)

    def find(self, interaction_hash):
        '''
        Find the specified interaction.

        :param str interaction_hash: in crc32 format

        :returns: interaction if found, None otherwise.
        :rtype: :class:`.Interaction` or None
        '''
        interaction = next((interaction for interaction in self.interactions
                            if interaction.hash == interaction_hash), None)
        return interaction
