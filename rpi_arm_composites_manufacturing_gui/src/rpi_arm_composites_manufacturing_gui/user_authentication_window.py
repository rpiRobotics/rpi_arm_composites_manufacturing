#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QDialog
from python_qt_binding.QtCore import QMutex, QMutexLocker,QSemaphore, QThread
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *



class UserAuthenticationWindow(QDialog):
    def __init__(self):
        """
        simple user authentication popup window, username and password pairs are established in the file currently set to username:password, more entries can be added for different users
        currently unused by system

        """
        super(UserAuthenticationWindow,self).__init__()
        self.setWindowModality(Qt.WindowModal)
        self.userlogins={'username': 'password'}
        self.loginprompt = os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'resource', 'loginprompt.ui')
        loadUi(self.loginprompt,self)
        self.ok.pressed.connect(self.proceed)
        self.cancel.pressed.connect(self.cancel_window)



    def cancel_window(self):
        """function to close window"""
        self.close()

    def proceed(self):
        """
        function to check if user has entered in username and password pair, if username does not exist it sets text in username box to username incorrect, if password is incorrect it sets text in
        password box to password incorrect
        """
        username_entry=str(self.username_prompt.text())
        password_entry=str(self.password_prompt.text())
        if(self.userlogins.has_key(username_entry)):
            if(password_entry == self.userlogins[username_entry]):

                self.accept()
            else:
                self.password_prompt.setText("Password Incorrect")
        else:
            self.username_prompt.setText("Username Incorrect")
            self.password_prompt.setText("")

