# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'coo.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1180, 941)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_10 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_10.setObjectName("gridLayout_10")
        self.O_setting = QtWidgets.QGroupBox(self.centralwidget)
        self.O_setting.setObjectName("O_setting")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.O_setting)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.pushButton_save_config = QtWidgets.QPushButton(self.O_setting)
        self.pushButton_save_config.setObjectName("pushButton_save_config")
        self.gridLayout_5.addWidget(self.pushButton_save_config, 1, 0, 1, 1)
        self.pushButton = QtWidgets.QPushButton(self.O_setting)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout_5.addWidget(self.pushButton, 1, 1, 1, 1)
        self.pushButton_cal_ver_task_list = QtWidgets.QPushButton(self.O_setting)
        self.pushButton_cal_ver_task_list.setObjectName("pushButton_cal_ver_task_list")
        self.gridLayout_5.addWidget(self.pushButton_cal_ver_task_list, 0, 0, 1, 2)
        self.gridLayout_10.addWidget(self.O_setting, 0, 2, 1, 1)
        self.Vru_setting_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.Vru_setting_2.setMaximumSize(QtCore.QSize(350, 16777215))
        self.Vru_setting_2.setObjectName("Vru_setting_2")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.Vru_setting_2)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.groupBox_4 = QtWidgets.QGroupBox(self.Vru_setting_2)
        self.groupBox_4.setObjectName("groupBox_4")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.groupBox_4)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.pushButton_alive_dut = QtWidgets.QPushButton(self.groupBox_4)
        self.pushButton_alive_dut.setText("")
        self.pushButton_alive_dut.setCheckable(False)
        self.pushButton_alive_dut.setObjectName("pushButton_alive_dut")
        self.gridLayout_4.addWidget(self.pushButton_alive_dut, 1, 1, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.groupBox_4)
        self.label_5.setObjectName("label_5")
        self.gridLayout_4.addWidget(self.label_5, 1, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.groupBox_4)
        self.label_2.setObjectName("label_2")
        self.gridLayout_4.addWidget(self.label_2, 0, 0, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_4.addItem(spacerItem, 1, 2, 1, 1)
        self.lineEdit_pos_dut = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_pos_dut.setEnabled(True)
        self.lineEdit_pos_dut.setReadOnly(True)
        self.lineEdit_pos_dut.setObjectName("lineEdit_pos_dut")
        self.gridLayout_4.addWidget(self.lineEdit_pos_dut, 0, 1, 1, 2)
        self.gridLayout_7.addWidget(self.groupBox_4, 0, 0, 1, 1)
        self.groupBox_5 = QtWidgets.QGroupBox(self.Vru_setting_2)
        self.groupBox_5.setObjectName("groupBox_5")
        self.gridLayout_8 = QtWidgets.QGridLayout(self.groupBox_5)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.pushButton_confirm_dut = QtWidgets.QPushButton(self.groupBox_5)
        self.pushButton_confirm_dut.setObjectName("pushButton_confirm_dut")
        self.gridLayout_8.addWidget(self.pushButton_confirm_dut, 3, 0, 1, 2)
        self.label_6 = QtWidgets.QLabel(self.groupBox_5)
        self.label_6.setObjectName("label_6")
        self.gridLayout_8.addWidget(self.label_6, 0, 0, 1, 2)
        self.set_route_dut = QtWidgets.QPushButton(self.groupBox_5)
        self.set_route_dut.setObjectName("set_route_dut")
        self.gridLayout_8.addWidget(self.set_route_dut, 2, 0, 1, 1)
        self.textBrowser_tasklist_dut = QtWidgets.QTextBrowser(self.groupBox_5)
        self.textBrowser_tasklist_dut.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.textBrowser_tasklist_dut.setObjectName("textBrowser_tasklist_dut")
        self.gridLayout_8.addWidget(self.textBrowser_tasklist_dut, 4, 0, 1, 2)
        self.set_stop_dut = QtWidgets.QPushButton(self.groupBox_5)
        self.set_stop_dut.setObjectName("set_stop_dut")
        self.gridLayout_8.addWidget(self.set_stop_dut, 2, 1, 1, 1)
        self.lineEdit_dut_start_s = QtWidgets.QLineEdit(self.groupBox_5)
        self.lineEdit_dut_start_s.setObjectName("lineEdit_dut_start_s")
        self.gridLayout_8.addWidget(self.lineEdit_dut_start_s, 1, 0, 1, 2)
        self.gridLayout_7.addWidget(self.groupBox_5, 1, 0, 1, 1)
        self.gridLayout_10.addWidget(self.Vru_setting_2, 0, 1, 3, 1)
        self.groupBox_6 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_6.setObjectName("groupBox_6")
        self.gridLayout_9 = QtWidgets.QGridLayout(self.groupBox_6)
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.system_output = QtWidgets.QTextBrowser(self.groupBox_6)
        self.system_output.setObjectName("system_output")
        self.gridLayout_9.addWidget(self.system_output, 0, 0, 1, 1)
        self.gridLayout_10.addWidget(self.groupBox_6, 1, 2, 1, 1)
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton_event_stop = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_event_stop.setObjectName("pushButton_event_stop")
        self.gridLayout.addWidget(self.pushButton_event_stop, 0, 1, 1, 1)
        self.pushButton_event_start = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_event_start.setEnabled(True)
        self.pushButton_event_start.setObjectName("pushButton_event_start")
        self.gridLayout.addWidget(self.pushButton_event_start, 0, 0, 1, 1)
        self.pushButton__reset = QtWidgets.QPushButton(self.groupBox)
        self.pushButton__reset.setObjectName("pushButton__reset")
        self.gridLayout.addWidget(self.pushButton__reset, 0, 2, 1, 1)
        self.gridLayout_10.addWidget(self.groupBox, 2, 2, 1, 1)
        self.Vru_setting = QtWidgets.QGroupBox(self.centralwidget)
        self.Vru_setting.setMaximumSize(QtCore.QSize(350, 16777215))
        self.Vru_setting.setObjectName("Vru_setting")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.Vru_setting)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.groupBox_3 = QtWidgets.QGroupBox(self.Vru_setting)
        self.groupBox_3.setObjectName("groupBox_3")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.groupBox_3)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.label = QtWidgets.QLabel(self.groupBox_3)
        self.label.setObjectName("label")
        self.gridLayout_6.addWidget(self.label, 0, 0, 1, 1)
        self.lineEdit_pos_vru = QtWidgets.QLineEdit(self.groupBox_3)
        self.lineEdit_pos_vru.setEnabled(True)
        self.lineEdit_pos_vru.setReadOnly(True)
        self.lineEdit_pos_vru.setObjectName("lineEdit_pos_vru")
        self.gridLayout_6.addWidget(self.lineEdit_pos_vru, 0, 1, 1, 3)
        self.label_4 = QtWidgets.QLabel(self.groupBox_3)
        self.label_4.setObjectName("label_4")
        self.gridLayout_6.addWidget(self.label_4, 1, 0, 1, 1)
        self.pushButton_alive_vru = QtWidgets.QPushButton(self.groupBox_3)
        self.pushButton_alive_vru.setText("")
        self.pushButton_alive_vru.setCheckable(False)
        self.pushButton_alive_vru.setObjectName("pushButton_alive_vru")
        self.gridLayout_6.addWidget(self.pushButton_alive_vru, 1, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.groupBox_3)
        self.label_7.setObjectName("label_7")
        self.gridLayout_6.addWidget(self.label_7, 1, 2, 1, 1)
        self.pushButton_armed_vru = QtWidgets.QPushButton(self.groupBox_3)
        self.pushButton_armed_vru.setText("")
        self.pushButton_armed_vru.setCheckable(False)
        self.pushButton_armed_vru.setObjectName("pushButton_armed_vru")
        self.gridLayout_6.addWidget(self.pushButton_armed_vru, 1, 3, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.groupBox_3)
        self.label_8.setObjectName("label_8")
        self.gridLayout_6.addWidget(self.label_8, 2, 0, 1, 1)
        self.lineEdit_pos_vru_2 = QtWidgets.QLineEdit(self.groupBox_3)
        self.lineEdit_pos_vru_2.setEnabled(True)
        self.lineEdit_pos_vru_2.setReadOnly(True)
        self.lineEdit_pos_vru_2.setObjectName("lineEdit_pos_vru_2")
        self.gridLayout_6.addWidget(self.lineEdit_pos_vru_2, 2, 1, 1, 3)
        self.gridLayout_3.addWidget(self.groupBox_3, 0, 0, 1, 1)
        self.groupBox_7 = QtWidgets.QGroupBox(self.Vru_setting)
        self.groupBox_7.setObjectName("groupBox_7")
        self.gridLayout_11 = QtWidgets.QGridLayout(self.groupBox_7)
        self.gridLayout_11.setObjectName("gridLayout_11")
        self.pushButton_arm_vru = QtWidgets.QPushButton(self.groupBox_7)
        self.pushButton_arm_vru.setObjectName("pushButton_arm_vru")
        self.gridLayout_11.addWidget(self.pushButton_arm_vru, 0, 0, 1, 1)
        self.pushButton_disarm_vru = QtWidgets.QPushButton(self.groupBox_7)
        self.pushButton_disarm_vru.setObjectName("pushButton_disarm_vru")
        self.gridLayout_11.addWidget(self.pushButton_disarm_vru, 0, 1, 1, 1)
        self.pushButton_Manual_vru = QtWidgets.QPushButton(self.groupBox_7)
        self.pushButton_Manual_vru.setObjectName("pushButton_Manual_vru")
        self.gridLayout_11.addWidget(self.pushButton_Manual_vru, 1, 0, 1, 1)
        self.pushButton_Manual_vru_2 = QtWidgets.QPushButton(self.groupBox_7)
        self.pushButton_Manual_vru_2.setObjectName("pushButton_Manual_vru_2")
        self.gridLayout_11.addWidget(self.pushButton_Manual_vru_2, 1, 1, 1, 1)
        self.pushButton_Return_vru = QtWidgets.QPushButton(self.groupBox_7)
        self.pushButton_Return_vru.setObjectName("pushButton_Return_vru")
        self.gridLayout_11.addWidget(self.pushButton_Return_vru, 1, 2, 1, 1)
        self.gridLayout_3.addWidget(self.groupBox_7, 1, 0, 1, 1)
        self.groupBox_2 = QtWidgets.QGroupBox(self.Vru_setting)
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox_2)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_3 = QtWidgets.QLabel(self.groupBox_2)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 0, 0, 1, 2)
        self.lineEdit_vru_start_s = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_vru_start_s.setObjectName("lineEdit_vru_start_s")
        self.gridLayout_2.addWidget(self.lineEdit_vru_start_s, 1, 0, 1, 2)
        self.set_route_vru = QtWidgets.QPushButton(self.groupBox_2)
        self.set_route_vru.setObjectName("set_route_vru")
        self.gridLayout_2.addWidget(self.set_route_vru, 2, 0, 1, 1)
        self.set_stop_vru = QtWidgets.QPushButton(self.groupBox_2)
        self.set_stop_vru.setObjectName("set_stop_vru")
        self.gridLayout_2.addWidget(self.set_stop_vru, 2, 1, 1, 1)
        self.pushButton_confirm_vru = QtWidgets.QPushButton(self.groupBox_2)
        self.pushButton_confirm_vru.setObjectName("pushButton_confirm_vru")
        self.gridLayout_2.addWidget(self.pushButton_confirm_vru, 3, 0, 1, 2)
        self.textBrowser_tasklist_vru = QtWidgets.QTextBrowser(self.groupBox_2)
        self.textBrowser_tasklist_vru.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.textBrowser_tasklist_vru.setObjectName("textBrowser_tasklist_vru")
        self.gridLayout_2.addWidget(self.textBrowser_tasklist_vru, 4, 0, 1, 2)
        self.gridLayout_3.addWidget(self.groupBox_2, 2, 0, 1, 1)
        self.gridLayout_10.addWidget(self.Vru_setting, 0, 0, 3, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1180, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.O_setting.setTitle(_translate("MainWindow", "System_setting"))
        self.pushButton_save_config.setText(_translate("MainWindow", "Save config"))
        self.pushButton.setText(_translate("MainWindow", "Load config"))
        self.pushButton_cal_ver_task_list.setText(_translate("MainWindow", "Calculate and_verify task list"))
        self.Vru_setting_2.setTitle(_translate("MainWindow", "Dut setting"))
        self.groupBox_4.setTitle(_translate("MainWindow", "Robot status"))
        self.label_5.setText(_translate("MainWindow", "alive:"))
        self.label_2.setText(_translate("MainWindow", "position:"))
        self.groupBox_5.setTitle(_translate("MainWindow", "Mission planning"))
        self.pushButton_confirm_dut.setText(_translate("MainWindow", "confirm"))
        self.label_6.setText(_translate("MainWindow", "Start second_after_test:"))
        self.set_route_dut.setText(_translate("MainWindow", "plan_route"))
        self.textBrowser_tasklist_dut.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Task list</p></body></html>"))
        self.set_stop_dut.setText(_translate("MainWindow", "plan_stop"))
        self.groupBox_6.setTitle(_translate("MainWindow", "System output"))
        self.groupBox.setTitle(_translate("MainWindow", "EventController"))
        self.pushButton_event_stop.setText(_translate("MainWindow", "stop"))
        self.pushButton_event_start.setText(_translate("MainWindow", "start_experiment"))
        self.pushButton__reset.setText(_translate("MainWindow", "reset"))
        self.Vru_setting.setTitle(_translate("MainWindow", "Vru setting"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Robot status"))
        self.label.setText(_translate("MainWindow", "position:"))
        self.label_4.setText(_translate("MainWindow", "alive:"))
        self.label_7.setText(_translate("MainWindow", "arm:"))
        self.label_8.setText(_translate("MainWindow", "Mode"))
        self.groupBox_7.setTitle(_translate("MainWindow", "Control"))
        self.pushButton_arm_vru.setText(_translate("MainWindow", "Arm"))
        self.pushButton_disarm_vru.setText(_translate("MainWindow", "Disarm"))
        self.pushButton_Manual_vru.setText(_translate("MainWindow", "Manual"))
        self.pushButton_Manual_vru_2.setText(_translate("MainWindow", "Hold"))
        self.pushButton_Return_vru.setText(_translate("MainWindow", "Offboard"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Mission planning"))
        self.label_3.setText(_translate("MainWindow", "Start second_after_test:"))
        self.set_route_vru.setText(_translate("MainWindow", "plan_route"))
        self.set_stop_vru.setText(_translate("MainWindow", "plan_stop"))
        self.pushButton_confirm_vru.setText(_translate("MainWindow", "confirm"))
        self.textBrowser_tasklist_vru.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Task list</p></body></html>"))
