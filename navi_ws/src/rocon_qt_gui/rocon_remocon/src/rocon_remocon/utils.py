#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rocon_python_utils
import rospkg
import rocon_std_msgs.msg as rocon_std_msgs

from python_qt_binding.QtGui import QLabel, QTextEdit, QSizePolicy, QFont, QCheckBox, QMessageBox

##############################################################################
# Methods
##############################################################################

def show_message(parent, title, message):
     QMessageBox.warning(parent, str(title), str(message), QMessageBox.Ok | QMessageBox.Ok)

def setup_home_dirs():
    if not os.path.isdir(get_home()):
        os.makedirs(get_home())
    if not os.path.isdir(get_icon_cache_home()):
        os.makedirs(get_icon_cache_home())
    if not os.path.isdir(get_settings_cache_home()):
        os.makedirs(get_settings_cache_home())


def get_home():
    '''
      Retrieve the location of the home directory for the rocon remocon's
      temporary storage needs

      @return the rocon remocon home directory (path object).
      @type str
    '''
    return os.path.join(rospkg.get_ros_home(), 'rocon', 'remocon')


def get_icon_cache_home():
    '''
      Retrieve the location of the directory used for storing icons.

      @return the rocon remocon icons directory (path object).
      @type str
    '''
    return os.path.join(get_home(), 'icons')


def get_settings_cache_home():
    '''
      Retrieve the location of the directory used for storing qt settings.

      @return the rocon remocon qt settings directory (path object).
      @type str
    '''
    return os.path.join(get_home(), 'cache')


def find_rocon_remocon_script(name):
    """
    Get the path to the internal script of the specified name. Note that this changes
    depending on whether you are working in a devel or an install space. Let the
    find resource handler discover where they are.

    :returns: full absolute pathnmae to the script
    :rtype: path
    :raises: `rospgk.ResourceNotFound`
    """
    return rocon_python_utils.ros.find_resource('rocon_remocon', name)


def get_web_browser():
    """
    Do a search through preferred browsers which most importantly can handle
    web apps and return the path to their executables.

    :returns: pathname to the browser
    :rtype: str
    """
    if rocon_python_utils.system.which("google-chrome"):
        return 'google-chrome'
    elif rocon_python_utils.system.which("google-chrome-unstable"):
        return 'google-chrome-unstable'
    elif rocon_python_utils.system.which("chromium-browser"):
        return 'chromium-browser'
    elif rocon_python_utils.system.which("firefox"):
        return 'firefox'
    return None


def get_web_browser_codename():
    """
    returns available browsers codename

    :returns:  web browser code name
    :rtype: str
    """
    # Currently it only supports chrome
    return rocon_std_msgs.Strings.OS_CHROME


def create_label_textedit_pair(key, value):
    '''
        Probabaly there should be better way to lay out param and remappings
    '''
    name = QLabel(key)
    name.setToolTip(key)
    name.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
    name.setMinimumWidth(400)
    name.setMaximumHeight(30)
    name.setWordWrap(True)

    textedit = QTextEdit()
    textedit.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
    textedit.setMinimumWidth(320)
    textedit.setMaximumHeight(30)
    textedit.append(str(value))
    return name, textedit


def create_label_checkbox_pair(key, value):
    label = QLabel(key)
    label.setToolTip(key)
    label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
    label.setMinimumWidth(400)
    label.setMaximumHeight(30)
    label.setWordWrap(True)

    checkbox = QCheckBox()
    checkbox.setChecked(value)
    return label, checkbox


def create_label(name, is_bold=False):
    qname = QLabel(name)
    f = QFont()
    f.setBold(is_bold)
    qname.setFont(f)
    return qname
