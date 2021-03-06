#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import rospkg
import os
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFrame, QListWidgetItem, QVBoxLayout, QHBoxLayout, QLabel, QLayout, QSizePolicy, QSpacerItem, QShortcut
from python_qt_binding.QtGui import QColor, QKeySequence
from python_qt_binding.QtCore import Qt, pyqtSignal, QSize

from mhri_social_msgs.msg import RecognizedWord
from mhri_msgs.msg import RaisingEvents, Reply


class ConversationViewPlugin(Plugin):
    signalAddItem = pyqtSignal(str, int)

    def __init__(self, context):
        super(ConversationViewPlugin, self).__init__(context)
        self.setObjectName('ConversationViewPlugin')

        self._widget = QWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_conversation_view'), 'resource', 'conversation_view.ui')
        loadUi(ui_file, self._widget)
        context.add_widget(self._widget)

        self.send_shortcut = QShortcut(QKeySequence("Ctrl+Return"), self._widget, self.handle_button_send)
        self._widget.buttonSend.clicked.connect(self.handle_button_send)
        self._widget.textInput.setFocus()

        self.signalAddItem.connect(self.add_item_to_conversation_view)

        self.pub_input = rospy.Publisher('recognized_word', RecognizedWord, queue_size=10)
        rospy.Subscriber('raising_events', RaisingEvents, self.handle_raising_events)
        rospy.Subscriber('reply', Reply, self.handle_reply)


    def add_item_to_conversation_view(self, msg, type=0):
        label_msg = QLabel(msg)
        label_msg.setWordWrap(True)
        label_msg.setStyleSheet('font-size:10pt;')

        inner_text_layout = QHBoxLayout()
        horizonalSpacer1 = QSpacerItem(0, 0, QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        if type == 0:
            inner_text_layout.addWidget(label_msg)
            inner_text_layout.addItem(horizonalSpacer1)
        elif type == 1:
            inner_text_layout.addItem(horizonalSpacer1)
            inner_text_layout.addWidget(label_msg)

        inner_layout = QVBoxLayout()
        time_msg = QLabel(str(time.asctime(time.localtime(time.time()))))
        time_msg.setStyleSheet('font-size:8pt;')

        inner_layout.addItem(inner_text_layout)
        inner_layout.addWidget(time_msg)
        inner_layout.setSizeConstraint(QLayout.SetFixedSize)

        outer_layout = QHBoxLayout()
        horizonalSpacer2 = QSpacerItem(0, 0, QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        if type == 0:
            label_msg.setStyleSheet('background: #e5e5ea; padding: 6px; border-radius: 8px;')
            time_msg.setAlignment(Qt.AlignLeft)
            outer_layout.addItem(inner_layout)
            outer_layout.addItem(horizonalSpacer2)
        elif type == 1:
            label_msg.setStyleSheet('background: #1d86f4; padding: 6px; border-radius: 8px; color:#fff;')
            time_msg.setAlignment(Qt.AlignRight)
            outer_layout.addItem(horizonalSpacer2)
            outer_layout.addItem(inner_layout)
        outer_layout.setSizeConstraint(QLayout.SetMinimumSize)

        widget = QWidget()
        widget.setLayout(outer_layout)
        widget.resize(widget.sizeHint())

        item = QListWidgetItem()
        item.setSizeHint(widget.sizeHint())

        self._widget.listWidget.addItem(item)
        self._widget.listWidget.setItemWidget(item, widget)
        self._widget.listWidget.scrollToBottom()


    def handle_raising_events(self, msg):
        event_text = msg.recognized_word
        for event in msg.events:
            event_text = event_text + '\n -%s'%event
        self.signalAddItem.emit(event_text, 1)


    def handle_reply(self, msg):
        event_text = msg.reply
        self.signalAddItem.emit(event_text, 0)


    def handle_button_send(self):
        input_text = self._widget.textInput.toPlainText()
        if input_text == '':
            return

        msg = RecognizedWord()
        msg.recognized_word = input_text
        msg.confidence = 1.0
        self.pub_input.publish(msg)
        self._widget.textInput.clear()


    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
