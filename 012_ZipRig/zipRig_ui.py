import sys
import os
import json
import pymel.core as pm

from PySide2 import QtCore, QtUiTools, QtWidgets, QtGui
from shiboken2 import wrapInstance
import maya.OpenMayaUI as omui

import zipRig
reload(zipRig)


def maya_main_window():
    """
    Return the Maya main window widget as a Python object
    """
    main_window_ptr = omui.MQtUtil.mainWindow()
    if sys.version_info.major >= 3:
        return wrapInstance(int(main_window_ptr), QtWidgets.QWidget)
    else:
        return wrapInstance(long(main_window_ptr), QtWidgets.QWidget)

class ZipRigUI(QtWidgets.QDialog):
    WindowTitle = 'ZipCreator'

    def __init__(self, ui_path=None, parent=maya_main_window()):
        super(ZipRigUI, self).__init__(parent)

        self.setWindowTitle(self.WindowTitle)
        self.setWindowFlags(QtCore.Qt.WindowType.Window)

        self.init_ui(ui_path)
        self.create_layout()
        self.create_connections()

        # self.load_assets_from_json()

    def init_ui(self, ui_path=None):
        if not ui_path:
            ui_path = r"{0}\UI\zipRig_UI_test.ui".format(os.path.dirname(__file__))
        f = QtCore.QFile(ui_path)
        f.open(QtCore.QFile.ReadOnly)

        loader = QtUiTools.QUiLoader()
        self.ui = loader.load(f, parentWidget=self)

        f.close()

    def create_layout(self):
        self.ui.setContentsMargins(20, 20, 20, 20)

    def create_connections(self):
        self.ui.type_CB.currentTextChanged.connect(self.do_something)
        self.ui.ctlAmount_HRSD.valueChanged.connect(self.refresh_PTE)

        self.ui.zipDir_CB.currentTextChanged.connect(self.refresh_crv_CB)
        self.ui.selectCurve_00_BTTN.clicked.connect(self.refresh_crv_00_PTE)
        self.ui.selectCurve_01_BTTN.clicked.connect(self.refresh_crv_01_PTE)
        self.ui.selectCurve_02_BTTN.clicked.connect(self.refresh_crv_02_PTE)

        self.ui.create_BTTN.clicked.connect(self.Excute)

    # def load_assets_from_json(self):
    #     with open(self.json_file_path, "r") as file_for_read:
    #         self.assets = json.load(file_for_read)

    def do_something(self):
        print 'do something'

    def refresh_PTE(self):
        Value = self.ui.ctlAmount_HRSD.value()
        self.ui.ctlAmount_PTE.setPlainText(str(Value))

    def refresh_crv_CB(self):
        Value = self.ui.zipDir_CB.currentText()
        if Value == 'x':
            self.ui.curve_01_CB.setCurrentIndex(1)
            self.ui.curve_02_CB.setCurrentIndex(1)
        elif Value == 'y':
            self.ui.curve_01_CB.setCurrentIndex(0)
            self.ui.curve_02_CB.setCurrentIndex(0)
        elif Value == 'z':
            self.ui.curve_01_CB.setCurrentIndex(2)
            self.ui.curve_02_CB.setCurrentIndex(2)

    def refresh_crv_00_PTE(self):
        sel = pm.ls(sl=1)
        self.ui.selectCurve_00_PTE.setPlainText(str(sel[0]))

    def refresh_crv_01_PTE(self):
        sel = pm.ls(sl=1)
        self.ui.selectCurve_01_PTE.setPlainText(str(sel[0]))

    def refresh_crv_02_PTE(self):
        sel = pm.ls(sl=1)
        self.ui.selectCurve_02_PTE.setPlainText(str(sel[0]))

    def Excute(self):
        Prefix = self.ui.name_PTE.toPlainText()
        curve00 = self.ui.selectCurve_00_PTE.toPlainText()
        curve01 = self.ui.selectCurve_01_PTE.toPlainText()
        curve02 = self.ui.selectCurve_02_PTE.toPlainText()
        Curves = pm.ls(curve00, curve01, curve02)
        Direction = self.ui.zipDir_CB.currentText()
        PullerAmount = self.ui.puller_CB.isChecked()
        Ctl_Amount = self.ui.ctlAmount_PTE.toPlainText()
        DelCurves = self.ui.delCurve_CB.isChecked()

        zipRig.buildRig(Prefix, Curves, Ctl_Amount, PullerAmount, Direction, DelCurves)

try:
    ui.close() # pylint: disable=E0601
    ui.deleteLater()
except:
    pass

ui_path = r'D:\Dropbox\PythonScripts\012_ZipRig\UI\zipRig_UI_test.ui'
ui = ZipRigUI(ui_path)
ui.show()