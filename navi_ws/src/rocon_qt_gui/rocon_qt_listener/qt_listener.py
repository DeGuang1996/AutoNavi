#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/license/LICENSE
#
"""
  A simple pyqt listener
"""
##############################################################################
# Imports
##############################################################################

import sys
import signal
import rocon_console.console as console

try:
    from PyQt4 import QtCore, QtGui
except ImportError:
    print(console.red + "[ERROR] : this app requires you to pre-install pyqt modules." + console.reset)
    sys.exit(1)

import rospy
import std_msgs.msg as std_msgs
import rocon_python_utils
import time

##############################################################################
# Classes
##############################################################################


class Window(QtGui.QWidget):

    def __init__(self):
        super(Window, self).__init__()
        self._icon = rocon_python_utils.ros.find_resource_from_string('rocon_bubble_icons/rocon.png')
        self.initUI()

    def initUI(self):
        self._list_view = QtGui.QListWidget(self)
        self._list_view.resize(400, 400)

        self.setWindowTitle("Qt Listener")
        self.setWindowIcon(QtGui.QIcon(self._icon))
        self.putUnderMouse()

    def putUnderMouse(self):
        mouse = QtGui.QCursor.pos()
        self.move(mouse.x() - self._list_view.geometry().width() / 2, mouse.y() - self._list_view.geometry().height() / 2)

    def listener(self, data):
        QtGui.QListWidgetItem("I heard %s" % data.data, self._list_view)
        self._list_view.scrollToBottom()
        #rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)


def shutdown(signal_number, unused_frame):
    """
    We have to catch any SIGINT's before the qt engine catches them
    because we have to ensure we shut down gracefully with ros so that
    it doesn't leave a zombie node on the ros master.

    Other signals are ok, qt handles them.
    """
    app.exit()
    # this will let the rest of the main code to run its course


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True, disable_signals=True)

    #signal.signal(signal.SIGINT, signal.SIG_DFL)  # make sure this comes after the rospy call, otherwise it will handle signals.
    signal.signal(signal.SIGINT, shutdown)  # make sure this comes after the rospy call, otherwise it will handle signals.
    app = QtGui.QApplication(sys.argv)
    window = Window()
    window.show()
    time.sleep(0.5)  # funny, qt window disappears from focus without this.
    rospy.Subscriber('chatter', std_msgs.String, window.listener)

    # Forces the qt main loop to stop for a while and let the python interpreter run some code
    # see http://stackoverflow.com/questions/4938723/what-is-the-correct-way-to-make-my-pyqt-application-quit-when-killed-from-the-co
    timer = QtCore.QTimer()
    timer.start(500)  # trigger a timeout periodically (ms)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    result = app.exec_()
    rospy.signal_shutdown("manual shutdown of a pyqt program")
    sys.exit(result)
