#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import sys

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QSize  # QFile, QIODevice, QAbstractListModel, pyqtSignal, QStringList
from python_qt_binding.QtGui import QIcon, QWidget, QLabel  # QFileDialog, QGraphicsScene, QImage, QPainter, QComboBox
from python_qt_binding.QtGui import QSizePolicy, QTextEdit, QPushButton, QDialog  # QCompleter, QBrush, QPen, QColor
from python_qt_binding.QtGui import QMainWindow, QCheckBox

from python_qt_binding.QtGui import QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox  # QTabWidget, QPlainTextEdit

import rospkg
from rocon_console import console
import rocon_python_utils

from .rocon_masters import RoconMasters
from . import utils

##############################################################################
# Main Window
##############################################################################


class QMasterChooser(QMainWindow):

    rocon_remocon_script = utils.find_rocon_remocon_script('rocon_remocon')
    rocon_remocon_sub_script = utils.find_rocon_remocon_script('rocon_remocon_sub')

    def __init__(self, parent, title, application):
        self._context = parent
        self.application = application
        super(QMasterChooser, self).__init__()
        utils.setup_home_dirs()

        self._init_widget()
        self._init_interface()

        # start application
        self._widget_main.show()
        self._widget_main.activateWindow()  # give it the focus
        self._widget_main.raise_()          # make sure it is on top
        self._update_rocon_master_list()

    def __del__(self):
        console.loginfo("RemoconMain: Destroy")

    def _init_host_configuration(self):
        self.host_name = "localhost"
        self.master_uri = "http://%s:11311" % (self.host_name)

        self.env_host_name = os.getenv("ROS_HOSTNAME")
        self.env_master_uri = os.getenv("ROS_MASTER_URI")
        if self.env_host_name:
            self.host_name = self.env_host_name
        if self.env_master_uri == None:
            self.env_master_uri = "http://%s:11311" % (self.host_name)
        elif self.env_master_uri:
            self.master_uri = self.env_master_uri

    def _init_icon_paths(self):
        self.icon_paths = {}
        try:
            self.icon_paths['unknown'] = rocon_python_utils.ros.find_resource_from_string('rocon_icons/unknown', extension='png')
        except (rospkg.ResourceNotFound, ValueError):
            console.logerror("Remocon : couldn't find icons on the ros package path (install rocon_icons and rocon_bubble_icons")
            sys.exit(1)

    def _init_widget(self):
        self._init_icon_paths()

        self.setObjectName('Remocon')
        self._widget_main = QWidget()

        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'remocon.ui')
        loadUi(path, self._widget_main)

        # main widget
        self._widget_main.list_widget.setIconSize(QSize(50, 50))
        self._widget_main.list_widget.itemDoubleClicked.connect(self._connect_rocon_master)  # list item double click event
        self._widget_main.list_widget.itemClicked.connect(self._select_rocon_master)  # list item double click event

        self._widget_main.add_concert_btn.pressed.connect(self._set_add_rocon_master)  # add button event
        self._widget_main.delete_btn.pressed.connect(self._delete_rocon_master)  # delete button event
        self._widget_main.delete_all_btn.pressed.connect(self._delete_all_rocon_masters)  # delete all button event
        self._widget_main.refresh_btn.pressed.connect(self._refresh_all_rocon_master_list)  # refresh all button event

        self._widget_main.list_info_widget.clear()

    def _init_interface(self):
        self._init_host_configuration()

        self.rocon_masters = RoconMasters()
        self._connect_dlg = None
        self.cur_selected_rocon_master = None
        self._is_init = True

    def _delete_all_rocon_masters(self):
        self.rocon_masters.clear()
        self._update_rocon_master_list()
        self._widget_main.list_info_widget.clear()

    def _delete_rocon_master(self):
        if self.cur_selected_rocon_master in self.rocon_masters.keys():
            self.rocon_masters.delete(self.cur_selected_rocon_master)
        self._update_rocon_master_list()
        self._widget_main.list_info_widget.clear()

    def _add_rocon_master(self, uri_text_widget, host_name_text_widget):
        rocon_master = self.rocon_masters.add(uri_text_widget.toPlainText(), host_name_text_widget.toPlainText())
        self._refresh_rocon_master(rocon_master)

    def _set_add_rocon_master(self):
        if self._connect_dlg:
            console.logdebug("Dialog is live!!")
            self._connect_dlg.done(0)

        self._connect_dlg = self._create_add_rocon_master_dialog()
        self._connect_dlg.setVisible(True)
        self._connect_dlg.finished.connect(self._destroy_connect_dlg)

    def _refresh_rocon_master(self, rocon_master):
        rocon_master.check()
        self._widget_main.list_info_widget.clear()
        self._update_rocon_master_list()

    def _refresh_all_rocon_master_list(self):
        self.rocon_masters.check()
        self._widget_main.list_info_widget.clear()
        self._update_rocon_master_list()

    def _update_rocon_master_list(self):
        self._widget_main.list_widget.clear()
        for rocon_master in self.rocon_masters.values():
            self._add_rocon_master_list_item(rocon_master)
        self.rocon_masters.dump()

    def _add_rocon_master_list_item(self, rocon_master):

        rocon_master.current_row = str(self._widget_main.list_widget.count())

        display_name = str(rocon_master.name) + "\n" + "[" + str(rocon_master.uri) + "]"
        self._widget_main.list_widget.insertItem(self._widget_main.list_widget.count(), display_name)

        # setting the list font
        font = self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).font()
        font.setPointSize(13)
        self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).setFont(font)

        # setToolTip
        rocon_master_info = ""
        rocon_master_info += "rocon_master_index: " + str(rocon_master.index) + "\n"
        rocon_master_info += "rocon_master_name: " + str(rocon_master.name) + "\n"
        rocon_master_info += "master_uri:  " + str(rocon_master.uri) + "\n"
        rocon_master_info += "host_name:  " + str(rocon_master.host_name) + "\n"
        rocon_master_info += "description:  " + str(rocon_master.description)
        self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).setToolTip(rocon_master_info)

        # set icon
        if rocon_master.icon == "unknown.png":
            icon = QIcon(self.icon_paths['unknown'])
            self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).setIcon(icon)
        elif len(rocon_master.icon):
            icon = QIcon(os.path.join(utils.get_icon_cache_home(), rocon_master.icon))
            self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).setIcon(icon)
        else:
            console.logdebug("%s : No icon" % rocon_master.name)

    def _select_rocon_master(self, Item):
        list_widget = Item.listWidget()
        for k in self.rocon_masters.values():
            if k.current_row == str(list_widget.currentRow()):
                self.cur_selected_rocon_master = k.index
                break
        self._widget_main.list_info_widget.clear()
        info_text = "<html>"
        info_text += "<p>-------------------------------------------</p>"
        info_text += "<p><b>name: </b>" + str(self.rocon_masters[self.cur_selected_rocon_master].name) + "</p>"
        info_text += "<p><b>master_uri: </b>" + str(self.rocon_masters[self.cur_selected_rocon_master].uri) + "</p>"
        info_text += "<p><b>host_name: </b>" + str(self.rocon_masters[self.cur_selected_rocon_master].host_name) + "</p>"
        info_text += "<p><b>description: </b>" + str(self.rocon_masters[self.cur_selected_rocon_master].description) + "</p>"
        info_text += "<p>-------------------------------------------</p>"
        info_text += "</html>"
        self._widget_main.list_info_widget.appendHtml(info_text)

    def _destroy_connect_dlg(self):
        self._connect_dlg = None

    def _connect_rocon_master(self):
        rocon_master_name = str(self.rocon_masters[self.cur_selected_rocon_master].name)
        rocon_master_uri = str(self.rocon_masters[self.cur_selected_rocon_master].uri)
        rocon_master_host_name = str(self.rocon_masters[self.cur_selected_rocon_master].host_name)

        rocon_master_index = str(self.cur_selected_rocon_master)
        self.rocon_masters[rocon_master_index].check()
        # Todo this use of flags is spanky
        if self.rocon_masters[rocon_master_index].flag == '0':
            QMessageBox.warning(self, 'Rocon Master Connection Error', "Could not find a rocon master at %s" % self.rocon_masters[rocon_master_index].uri, QMessageBox.Ok)
            return
        if self.rocon_masters[rocon_master_index].flag == '1':
            QMessageBox.warning(self, 'Rocon Master Communication Error', "Found a rocon master at %s but cannot communicate with it (are ROS_IP/ROS_MASTER_URI correctly configured locally and remotely?)" % self.rocon_masters[rocon_master_index].uri, QMessageBox.Ok)
            return

        self._widget_main.hide()
        arguments = ["", rocon_master_uri, rocon_master_host_name]
        os.execv(QMasterChooser.rocon_remocon_sub_script, arguments)
        console.logdebug("Spawning: %s with args %s" % (QMasterChooser.rocon_remocon_sub_script, arguments))

    def _create_add_rocon_master_dialog(self):
        # dialog
        connect_dlg = QDialog(self._widget_main)
        connect_dlg.setWindowTitle("Add Ros Master")
        connect_dlg.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Ignored)
        connect_dlg.setMinimumSize(350, 0)
        # dlg_rect = self._connect_dlg.geometry()

        # dialog layout
        ver_layout = QVBoxLayout(connect_dlg)
        ver_layout.setContentsMargins(9, 9, 9, 9)

        # param layout
        text_grid_sub_widget = QWidget()
        text_grid_layout = QGridLayout(text_grid_sub_widget)
        text_grid_layout.setColumnStretch(1, 0)
        text_grid_layout.setRowStretch(2, 0)

        # param 1
        title_widget1 = QLabel("MASTER_URI: ")
        context_widget1 = QTextEdit()
        context_widget1.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Ignored)
        context_widget1.setMinimumSize(0, 30)
        context_widget1.append(self.master_uri)

        # param 2
        title_widget2 = QLabel("HOST_NAME: ")
        context_widget2 = QTextEdit()
        context_widget2.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Ignored)
        context_widget2.setMinimumSize(0, 30)
        context_widget2.append(self.host_name)

        # add param
        text_grid_layout.addWidget(title_widget1)
        text_grid_layout.addWidget(context_widget1)
        text_grid_layout.addWidget(title_widget2)
        text_grid_layout.addWidget(context_widget2)

        # add param layout
        ver_layout.addWidget(text_grid_sub_widget)

        # button layout
        button_hor_sub_widget = QWidget()
        button_hor_layout = QHBoxLayout(button_hor_sub_widget)

        uri_text_widget = context_widget1
        host_name_text_widget = context_widget2

        # button
        btn_call = QPushButton("Add")
        btn_cancel = QPushButton("Cancel")

        btn_call.clicked.connect(lambda: connect_dlg.done(0))
        btn_call.clicked.connect(lambda: self._add_rocon_master(uri_text_widget, host_name_text_widget))

        btn_cancel.clicked.connect(lambda: connect_dlg.done(0))

        # add button
        button_hor_layout.addWidget(btn_call)
        button_hor_layout.addWidget(btn_cancel)

        # add button layout
        ver_layout.addWidget(button_hor_sub_widget)

        return connect_dlg
