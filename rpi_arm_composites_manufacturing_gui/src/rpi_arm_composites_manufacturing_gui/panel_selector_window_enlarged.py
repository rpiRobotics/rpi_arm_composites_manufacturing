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


class PanelSelectorWindow(QDialog):
    def __init__(self):
        super(PanelSelectorWindow,self).__init__()
        self.placement_chosen=None
        self.pickup_chosen=None
        self.panel=None
        self.panelselectorui = os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'resource', 'panelselectorpopup.ui')
        loadUi(self.panelselectorui,self)
        self.midlistentry=QListWidgetItem('Leeward Mid Panel')
        #self.midlistentry.setFlags(Qt.ItemIsSelectable)
        self.tiplistentry=QListWidgetItem('Leeward Tip Panel')
        #self.tiplistentry.setFlags(Qt.ItemIsSelectable)
        self.panelList.addItem(self.midlistentry)
        self.panelList.addItem(self.tiplistentry)
        self.panelList.itemSelectionChanged.connect(self.panel_selected)
        #self.graphics_scene=QGraphicsScene(self.workspaceView)
        #self.robot_representation=QGraphicsEllipseItem(-50,0,100,100)
        #self.robot_representation.setBrush(QColor(255,170,0,255))
        #self.placement_nest_1=QGraphicsRectItem(100,60,100,100)
        #self.placement_nest_2=QGraphicsRectItem(100,-40,100,100)
        #self.pickup_nest_1=QGraphicsRectItem(-70,150,100,100)
        self.panelList.item(0).setSelected(True)
        
        self.continue_button.pressed.connect(self.pass_values)

        #self.graphics_scene.addItem(self.robot_representation)
        #self.graphics_scene.addItem(self.placement_nest_1)
        #self.graphics_scene.addItem(self.placement_nest_2)
        #self.graphics_scene.addItem(self.pickup_nest_1)
        #self.workspaceView.setScene(self.graphics_scene)


#    def placement_nest_1_selected(self):
#        self.placement_chosen='panel_nest_leeward_mid_panel_target'

    def get_panel_selected(self):
        return self.panel

    def get_pickup_selected(self):
        return self.pickup_chosen

    def get_placement_selected(self):
        return self.placement_chosen

    def panel_selected(self):
        if(self.midlistentry.isSelected()):
            self.panel="leeward_mid_panel"
        if(self.tiplistentry.isSelected()):
            self.panel="leeward_tip_panel"
    '''
    def mousePressEvent(self,event):
        if(event.x()>295 and event.x()<395 and event.y()>120 and event.y()<220):
            self.placement_nest_1.setBrush(Qt.darkGray)
            self.placement_nest_2.setBrush(Qt.white)
            if(self.placement_chosen=='panel_nest_leeward_tip_panel_target'):
                self.placement_chosen=None
                self.placement_nest_1.setBrush(Qt.white)
                self.placementLocation.setText("")
            else:
                self.placement_chosen='panel_nest_leeward_tip_panel_target'
                self.placementLocation.setText("Leeward Tip Panel Target")

        elif(event.x()>295 and event.x()<395 and event.y()>20 and event.y()<120):
            self.placement_nest_2.setBrush(Qt.darkGray)
            self.placement_nest_1.setBrush(Qt.white)
            if(self.placement_chosen=='panel_nest_leeward_mid_panel_target'):
                self.placement_chosen=None
                self.placement_nest_2.setBrush(Qt.white)
                self.placementLocation.setText("")
            else:
                self.placement_chosen='panel_nest_leeward_mid_panel_target'
                self.placementLocation.setText("Leeward Mid Panel Target")

        if(event.x()>125 and event.x()<225 and event.y()>210 and event.y()<310):
            self.pickup_nest_1.setBrush(Qt.darkGray)
            if(self.pickup_chosen=='panel_pickup_1'):
                self.pickup_chosen=None
                self.pickup_nest_1.setBrush(Qt.white)
                self.pickupLocation.setText("")
            else:
                self.pickup_chosen='panel_pickup_1'
                self.pickupLocation.setText("Panel Pick Up Nest")
    '''
    def pass_values(self):
        self.accept()

