#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import rospkg
import os
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFrame, QListWidgetItem, QVBoxLayout, QHBoxLayout, QLabel, QLayout, QSizePolicy, QSpacerItem, QShortcut
from python_qt_binding.QtGui import QColor, QKeySequence, QPixmap
from python_qt_binding.QtCore import Qt, pyqtSignal, QSize, QObject

from std_msgs.msg import Bool, String, Float64, Empty

class SignalRender(QObject):
    renderSignal = pyqtSignal([bool])

class SpeechStatusPlugin(Plugin):
    def __init__(self, context):
        super(SpeechStatusPlugin, self).__init__(context)
        self.setObjectName('SpeechStatusPlugin')

        self._widget = QWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_speech_status'), 'resource', 'speech_widget.ui')
        loadUi(ui_file, self._widget)
        context.add_widget(self._widget)

        file_live = os.path.join(rospkg.RosPack().get_path('rqt_speech_status'), 'resource', 'microphone.png')
        file_mute = os.path.join(rospkg.RosPack().get_path('rqt_speech_status'), 'resource', 'muted.png')
        file_silency = os.path.join(rospkg.RosPack().get_path('rqt_speech_status'), 'resource', 'silence-face.png')
        file_speech= os.path.join(rospkg.RosPack().get_path('rqt_speech_status'), 'resource', 'surprise-face.png')

        self.img_mute = QPixmap(file_mute).scaled(160, 160)
        self.img_live = QPixmap(file_live).scaled(160, 160)
        self.img_silency = QPixmap(file_silency).scaled(160, 160)
        self.img_speech = QPixmap(file_speech).scaled(160, 160)

        self._widget.imgRecognition.setPixmap(self.img_live)
        self._widget.imgSilency.setPixmap(self.img_silency)

        self.signal_render_recog_status = SignalRender()
        self.signal_render_recog_status.renderSignal.connect(self.render_recog_status)

        self.signal_render_silency_status = SignalRender()
        self.signal_render_silency_status.renderSignal.connect(self.render_silency_status)

        rospy.Subscriber('enable_recognition', Bool, self.handle_enable_recognition)
        rospy.Subscriber('start_of_speech', Empty, self.handle_start_speech)
        rospy.Subscriber('end_of_speech', Empty, self.handle_end_speech)
        rospy.Subscriber('silency_detected', Empty, self.handle_silency_detection)

    def handle_enable_recognition(self, msg):
        if msg.data:
            self.signal_render_recog_status.renderSignal.emit(True)
        else:
            self.signal_render_recog_status.renderSignal.emit(False)

    def handle_silency_detection(self, msg):
        self.signal_render_silency_status.renderSignal.emit(True)

    def handle_start_speech(self, msg):
        self.signal_render_silency_status.renderSignal.emit(False)

    def handle_end_speech(self, msg):
        self.signal_render_silency_status.renderSignal.emit(True)

    def render_recog_status(self, status):
        if status:
            self._widget.imgRecognition.setPixmap(self.img_live)
        else:
            self._widget.imgRecognition.setPixmap(self.img_mute)

    def render_silency_status(self, status):
        if status:
            self._widget.imgSilency.setPixmap(self.img_silency)
        else:
            self._widget.imgSilency.setPixmap(self.img_speech)



    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
