# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'tool.ui'
##
## Created by: Qt User Interface Compiler version 6.7.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractItemView, QApplication, QComboBox, QGridLayout,
    QHBoxLayout, QHeaderView, QLabel, QSizePolicy,
    QSpacerItem, QTableWidget, QTableWidgetItem, QVBoxLayout,
    QWidget)

class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(984, 632)
        self.verticalLayout_6 = QVBoxLayout(Form)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.label = QLabel(Form)
        self.label.setObjectName(u"label")
        font = QFont()
        font.setPointSize(16)
        self.label.setFont(font)

        self.verticalLayout_6.addWidget(self.label)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.widget = QWidget(Form)
        self.widget.setObjectName(u"widget")
        self.verticalLayout = QVBoxLayout(self.widget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label_2 = QLabel(self.widget)
        self.label_2.setObjectName(u"label_2")
        font1 = QFont()
        font1.setPointSize(13)
        self.label_2.setFont(font1)

        self.verticalLayout.addWidget(self.label_2)

        self.label_3 = QLabel(self.widget)
        self.label_3.setObjectName(u"label_3")

        self.verticalLayout.addWidget(self.label_3)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.comboBox = QComboBox(self.widget)
        self.comboBox.setObjectName(u"comboBox")

        self.horizontalLayout.addWidget(self.comboBox)

        self.horizontalSpacer = QSpacerItem(20, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)


        self.verticalLayout.addLayout(self.horizontalLayout)

        self.label_4 = QLabel(self.widget)
        self.label_4.setObjectName(u"label_4")

        self.verticalLayout.addWidget(self.label_4)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.comboBox_2 = QComboBox(self.widget)
        self.comboBox_2.setObjectName(u"comboBox_2")

        self.horizontalLayout_2.addWidget(self.comboBox_2)

        self.horizontalSpacer_5 = QSpacerItem(20, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer_5)


        self.verticalLayout.addLayout(self.horizontalLayout_2)

        self.label_5 = QLabel(self.widget)
        self.label_5.setObjectName(u"label_5")

        self.verticalLayout.addWidget(self.label_5)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.comboBox_3 = QComboBox(self.widget)
        self.comboBox_3.setObjectName(u"comboBox_3")

        self.horizontalLayout_3.addWidget(self.comboBox_3)

        self.horizontalSpacer_3 = QSpacerItem(20, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_3.addItem(self.horizontalSpacer_3)


        self.verticalLayout.addLayout(self.horizontalLayout_3)

        self.label_6 = QLabel(self.widget)
        self.label_6.setObjectName(u"label_6")

        self.verticalLayout.addWidget(self.label_6)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.comboBox_4 = QComboBox(self.widget)
        self.comboBox_4.setObjectName(u"comboBox_4")

        self.horizontalLayout_4.addWidget(self.comboBox_4)

        self.horizontalSpacer_4 = QSpacerItem(20, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_4)


        self.verticalLayout.addLayout(self.horizontalLayout_4)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)


        self.horizontalLayout_5.addWidget(self.widget)

        self.widget_2 = QWidget(Form)
        self.widget_2.setObjectName(u"widget_2")
        self.gridLayout = QGridLayout(self.widget_2)
        self.gridLayout.setObjectName(u"gridLayout")
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.label_7 = QLabel(self.widget_2)
        self.label_7.setObjectName(u"label_7")
        font2 = QFont()
        font2.setPointSize(11)
        self.label_7.setFont(font2)

        self.verticalLayout_2.addWidget(self.label_7)

        self.tableWidget = QTableWidget(self.widget_2)
        if (self.tableWidget.columnCount() < 3):
            self.tableWidget.setColumnCount(3)
        __qtablewidgetitem = QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        if (self.tableWidget.rowCount() < 8):
            self.tableWidget.setRowCount(8)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(0, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(1, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(2, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(3, __qtablewidgetitem6)
        __qtablewidgetitem7 = QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(4, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(5, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(6, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(7, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        __qtablewidgetitem11.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(0, 0, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        __qtablewidgetitem12.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(0, 1, __qtablewidgetitem12)
        __qtablewidgetitem13 = QTableWidgetItem()
        __qtablewidgetitem13.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(0, 2, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        __qtablewidgetitem14.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(1, 0, __qtablewidgetitem14)
        __qtablewidgetitem15 = QTableWidgetItem()
        __qtablewidgetitem15.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(1, 1, __qtablewidgetitem15)
        __qtablewidgetitem16 = QTableWidgetItem()
        __qtablewidgetitem16.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(1, 2, __qtablewidgetitem16)
        __qtablewidgetitem17 = QTableWidgetItem()
        __qtablewidgetitem17.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(2, 0, __qtablewidgetitem17)
        __qtablewidgetitem18 = QTableWidgetItem()
        __qtablewidgetitem18.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(2, 1, __qtablewidgetitem18)
        __qtablewidgetitem19 = QTableWidgetItem()
        __qtablewidgetitem19.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(2, 2, __qtablewidgetitem19)
        __qtablewidgetitem20 = QTableWidgetItem()
        __qtablewidgetitem20.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(3, 0, __qtablewidgetitem20)
        __qtablewidgetitem21 = QTableWidgetItem()
        __qtablewidgetitem21.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(3, 1, __qtablewidgetitem21)
        __qtablewidgetitem22 = QTableWidgetItem()
        __qtablewidgetitem22.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(3, 2, __qtablewidgetitem22)
        __qtablewidgetitem23 = QTableWidgetItem()
        __qtablewidgetitem23.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(4, 0, __qtablewidgetitem23)
        __qtablewidgetitem24 = QTableWidgetItem()
        __qtablewidgetitem24.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(4, 1, __qtablewidgetitem24)
        __qtablewidgetitem25 = QTableWidgetItem()
        __qtablewidgetitem25.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(4, 2, __qtablewidgetitem25)
        __qtablewidgetitem26 = QTableWidgetItem()
        __qtablewidgetitem26.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(5, 0, __qtablewidgetitem26)
        __qtablewidgetitem27 = QTableWidgetItem()
        __qtablewidgetitem27.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(5, 1, __qtablewidgetitem27)
        __qtablewidgetitem28 = QTableWidgetItem()
        __qtablewidgetitem28.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(5, 2, __qtablewidgetitem28)
        __qtablewidgetitem29 = QTableWidgetItem()
        __qtablewidgetitem29.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(6, 0, __qtablewidgetitem29)
        __qtablewidgetitem30 = QTableWidgetItem()
        __qtablewidgetitem30.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(6, 1, __qtablewidgetitem30)
        __qtablewidgetitem31 = QTableWidgetItem()
        __qtablewidgetitem31.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(6, 2, __qtablewidgetitem31)
        __qtablewidgetitem32 = QTableWidgetItem()
        __qtablewidgetitem32.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(7, 0, __qtablewidgetitem32)
        __qtablewidgetitem33 = QTableWidgetItem()
        __qtablewidgetitem33.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(7, 1, __qtablewidgetitem33)
        __qtablewidgetitem34 = QTableWidgetItem()
        __qtablewidgetitem34.setTextAlignment(Qt.AlignCenter);
        self.tableWidget.setItem(7, 2, __qtablewidgetitem34)
        self.tableWidget.setObjectName(u"tableWidget")
        self.tableWidget.setAutoScrollMargin(16)
        self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableWidget.setDragDropOverwriteMode(True)
        self.tableWidget.setDragDropMode(QAbstractItemView.NoDragDrop)
        self.tableWidget.setSelectionBehavior(QAbstractItemView.SelectItems)
        self.tableWidget.setShowGrid(True)
        self.tableWidget.setGridStyle(Qt.SolidLine)
        self.tableWidget.setSortingEnabled(False)
        self.tableWidget.setWordWrap(True)
        self.tableWidget.horizontalHeader().setCascadingSectionResizes(False)
        self.tableWidget.horizontalHeader().setProperty("showSortIndicator", False)
        self.tableWidget.horizontalHeader().setStretchLastSection(False)
        self.tableWidget.verticalHeader().setVisible(False)
        self.tableWidget.verticalHeader().setCascadingSectionResizes(False)
        self.tableWidget.verticalHeader().setMinimumSectionSize(1)
        self.tableWidget.verticalHeader().setDefaultSectionSize(27)
        self.tableWidget.verticalHeader().setHighlightSections(True)
        self.tableWidget.verticalHeader().setProperty("showSortIndicator", False)
        self.tableWidget.verticalHeader().setStretchLastSection(False)

        self.verticalLayout_2.addWidget(self.tableWidget)


        self.gridLayout.addLayout(self.verticalLayout_2, 0, 0, 1, 1)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.label_8 = QLabel(self.widget_2)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setFont(font2)

        self.verticalLayout_3.addWidget(self.label_8)

        self.tableWidget_2 = QTableWidget(self.widget_2)
        if (self.tableWidget_2.columnCount() < 3):
            self.tableWidget_2.setColumnCount(3)
        __qtablewidgetitem35 = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(0, __qtablewidgetitem35)
        __qtablewidgetitem36 = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(1, __qtablewidgetitem36)
        __qtablewidgetitem37 = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(2, __qtablewidgetitem37)
        if (self.tableWidget_2.rowCount() < 8):
            self.tableWidget_2.setRowCount(8)
        __qtablewidgetitem38 = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(0, __qtablewidgetitem38)
        __qtablewidgetitem39 = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(1, __qtablewidgetitem39)
        __qtablewidgetitem40 = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(2, __qtablewidgetitem40)
        __qtablewidgetitem41 = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(3, __qtablewidgetitem41)
        __qtablewidgetitem42 = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(4, __qtablewidgetitem42)
        __qtablewidgetitem43 = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(5, __qtablewidgetitem43)
        __qtablewidgetitem44 = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(6, __qtablewidgetitem44)
        __qtablewidgetitem45 = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(7, __qtablewidgetitem45)
        __qtablewidgetitem46 = QTableWidgetItem()
        __qtablewidgetitem46.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(0, 0, __qtablewidgetitem46)
        __qtablewidgetitem47 = QTableWidgetItem()
        __qtablewidgetitem47.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(0, 1, __qtablewidgetitem47)
        __qtablewidgetitem48 = QTableWidgetItem()
        __qtablewidgetitem48.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(0, 2, __qtablewidgetitem48)
        __qtablewidgetitem49 = QTableWidgetItem()
        __qtablewidgetitem49.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(1, 0, __qtablewidgetitem49)
        __qtablewidgetitem50 = QTableWidgetItem()
        __qtablewidgetitem50.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(1, 1, __qtablewidgetitem50)
        __qtablewidgetitem51 = QTableWidgetItem()
        __qtablewidgetitem51.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(1, 2, __qtablewidgetitem51)
        __qtablewidgetitem52 = QTableWidgetItem()
        __qtablewidgetitem52.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(2, 0, __qtablewidgetitem52)
        __qtablewidgetitem53 = QTableWidgetItem()
        __qtablewidgetitem53.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(2, 1, __qtablewidgetitem53)
        __qtablewidgetitem54 = QTableWidgetItem()
        __qtablewidgetitem54.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(2, 2, __qtablewidgetitem54)
        __qtablewidgetitem55 = QTableWidgetItem()
        __qtablewidgetitem55.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(3, 0, __qtablewidgetitem55)
        __qtablewidgetitem56 = QTableWidgetItem()
        __qtablewidgetitem56.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(3, 1, __qtablewidgetitem56)
        __qtablewidgetitem57 = QTableWidgetItem()
        __qtablewidgetitem57.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(3, 2, __qtablewidgetitem57)
        __qtablewidgetitem58 = QTableWidgetItem()
        __qtablewidgetitem58.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(4, 0, __qtablewidgetitem58)
        __qtablewidgetitem59 = QTableWidgetItem()
        __qtablewidgetitem59.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(4, 1, __qtablewidgetitem59)
        __qtablewidgetitem60 = QTableWidgetItem()
        __qtablewidgetitem60.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(4, 2, __qtablewidgetitem60)
        __qtablewidgetitem61 = QTableWidgetItem()
        __qtablewidgetitem61.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(5, 0, __qtablewidgetitem61)
        __qtablewidgetitem62 = QTableWidgetItem()
        __qtablewidgetitem62.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(5, 1, __qtablewidgetitem62)
        __qtablewidgetitem63 = QTableWidgetItem()
        __qtablewidgetitem63.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(5, 2, __qtablewidgetitem63)
        __qtablewidgetitem64 = QTableWidgetItem()
        __qtablewidgetitem64.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(6, 0, __qtablewidgetitem64)
        __qtablewidgetitem65 = QTableWidgetItem()
        __qtablewidgetitem65.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(6, 1, __qtablewidgetitem65)
        __qtablewidgetitem66 = QTableWidgetItem()
        __qtablewidgetitem66.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(6, 2, __qtablewidgetitem66)
        __qtablewidgetitem67 = QTableWidgetItem()
        __qtablewidgetitem67.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(7, 0, __qtablewidgetitem67)
        __qtablewidgetitem68 = QTableWidgetItem()
        __qtablewidgetitem68.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(7, 1, __qtablewidgetitem68)
        __qtablewidgetitem69 = QTableWidgetItem()
        __qtablewidgetitem69.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_2.setItem(7, 2, __qtablewidgetitem69)
        self.tableWidget_2.setObjectName(u"tableWidget_2")
        self.tableWidget_2.setAutoScrollMargin(16)
        self.tableWidget_2.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableWidget_2.setSelectionBehavior(QAbstractItemView.SelectItems)
        self.tableWidget_2.setShowGrid(True)
        self.tableWidget_2.setGridStyle(Qt.SolidLine)
        self.tableWidget_2.setSortingEnabled(False)
        self.tableWidget_2.horizontalHeader().setCascadingSectionResizes(False)
        self.tableWidget_2.horizontalHeader().setProperty("showSortIndicator", False)
        self.tableWidget_2.horizontalHeader().setStretchLastSection(False)
        self.tableWidget_2.verticalHeader().setVisible(False)
        self.tableWidget_2.verticalHeader().setCascadingSectionResizes(False)
        self.tableWidget_2.verticalHeader().setMinimumSectionSize(1)
        self.tableWidget_2.verticalHeader().setDefaultSectionSize(27)
        self.tableWidget_2.verticalHeader().setHighlightSections(True)
        self.tableWidget_2.verticalHeader().setProperty("showSortIndicator", False)
        self.tableWidget_2.verticalHeader().setStretchLastSection(False)

        self.verticalLayout_3.addWidget(self.tableWidget_2)


        self.gridLayout.addLayout(self.verticalLayout_3, 0, 1, 1, 1)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.label_9 = QLabel(self.widget_2)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setFont(font2)

        self.verticalLayout_4.addWidget(self.label_9)

        self.tableWidget_3 = QTableWidget(self.widget_2)
        if (self.tableWidget_3.columnCount() < 3):
            self.tableWidget_3.setColumnCount(3)
        __qtablewidgetitem70 = QTableWidgetItem()
        self.tableWidget_3.setHorizontalHeaderItem(0, __qtablewidgetitem70)
        __qtablewidgetitem71 = QTableWidgetItem()
        self.tableWidget_3.setHorizontalHeaderItem(1, __qtablewidgetitem71)
        __qtablewidgetitem72 = QTableWidgetItem()
        self.tableWidget_3.setHorizontalHeaderItem(2, __qtablewidgetitem72)
        if (self.tableWidget_3.rowCount() < 8):
            self.tableWidget_3.setRowCount(8)
        __qtablewidgetitem73 = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(0, __qtablewidgetitem73)
        __qtablewidgetitem74 = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(1, __qtablewidgetitem74)
        __qtablewidgetitem75 = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(2, __qtablewidgetitem75)
        __qtablewidgetitem76 = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(3, __qtablewidgetitem76)
        __qtablewidgetitem77 = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(4, __qtablewidgetitem77)
        __qtablewidgetitem78 = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(5, __qtablewidgetitem78)
        __qtablewidgetitem79 = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(6, __qtablewidgetitem79)
        __qtablewidgetitem80 = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(7, __qtablewidgetitem80)
        __qtablewidgetitem81 = QTableWidgetItem()
        __qtablewidgetitem81.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(0, 0, __qtablewidgetitem81)
        __qtablewidgetitem82 = QTableWidgetItem()
        __qtablewidgetitem82.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(0, 1, __qtablewidgetitem82)
        __qtablewidgetitem83 = QTableWidgetItem()
        __qtablewidgetitem83.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(0, 2, __qtablewidgetitem83)
        __qtablewidgetitem84 = QTableWidgetItem()
        __qtablewidgetitem84.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(1, 0, __qtablewidgetitem84)
        __qtablewidgetitem85 = QTableWidgetItem()
        __qtablewidgetitem85.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(1, 1, __qtablewidgetitem85)
        __qtablewidgetitem86 = QTableWidgetItem()
        __qtablewidgetitem86.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(1, 2, __qtablewidgetitem86)
        __qtablewidgetitem87 = QTableWidgetItem()
        __qtablewidgetitem87.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(2, 0, __qtablewidgetitem87)
        __qtablewidgetitem88 = QTableWidgetItem()
        __qtablewidgetitem88.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(2, 1, __qtablewidgetitem88)
        __qtablewidgetitem89 = QTableWidgetItem()
        __qtablewidgetitem89.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(2, 2, __qtablewidgetitem89)
        __qtablewidgetitem90 = QTableWidgetItem()
        __qtablewidgetitem90.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(3, 0, __qtablewidgetitem90)
        __qtablewidgetitem91 = QTableWidgetItem()
        __qtablewidgetitem91.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(3, 1, __qtablewidgetitem91)
        __qtablewidgetitem92 = QTableWidgetItem()
        __qtablewidgetitem92.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(3, 2, __qtablewidgetitem92)
        __qtablewidgetitem93 = QTableWidgetItem()
        __qtablewidgetitem93.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(4, 0, __qtablewidgetitem93)
        __qtablewidgetitem94 = QTableWidgetItem()
        __qtablewidgetitem94.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(4, 1, __qtablewidgetitem94)
        __qtablewidgetitem95 = QTableWidgetItem()
        __qtablewidgetitem95.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(4, 2, __qtablewidgetitem95)
        __qtablewidgetitem96 = QTableWidgetItem()
        __qtablewidgetitem96.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(5, 0, __qtablewidgetitem96)
        __qtablewidgetitem97 = QTableWidgetItem()
        __qtablewidgetitem97.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(5, 1, __qtablewidgetitem97)
        __qtablewidgetitem98 = QTableWidgetItem()
        __qtablewidgetitem98.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(5, 2, __qtablewidgetitem98)
        __qtablewidgetitem99 = QTableWidgetItem()
        __qtablewidgetitem99.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(6, 0, __qtablewidgetitem99)
        __qtablewidgetitem100 = QTableWidgetItem()
        __qtablewidgetitem100.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(6, 1, __qtablewidgetitem100)
        __qtablewidgetitem101 = QTableWidgetItem()
        __qtablewidgetitem101.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(6, 2, __qtablewidgetitem101)
        __qtablewidgetitem102 = QTableWidgetItem()
        __qtablewidgetitem102.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(7, 0, __qtablewidgetitem102)
        __qtablewidgetitem103 = QTableWidgetItem()
        __qtablewidgetitem103.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(7, 1, __qtablewidgetitem103)
        __qtablewidgetitem104 = QTableWidgetItem()
        __qtablewidgetitem104.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_3.setItem(7, 2, __qtablewidgetitem104)
        self.tableWidget_3.setObjectName(u"tableWidget_3")
        self.tableWidget_3.setAutoScrollMargin(16)
        self.tableWidget_3.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableWidget_3.setSelectionBehavior(QAbstractItemView.SelectItems)
        self.tableWidget_3.setShowGrid(True)
        self.tableWidget_3.setGridStyle(Qt.SolidLine)
        self.tableWidget_3.setSortingEnabled(False)
        self.tableWidget_3.horizontalHeader().setCascadingSectionResizes(False)
        self.tableWidget_3.horizontalHeader().setProperty("showSortIndicator", False)
        self.tableWidget_3.horizontalHeader().setStretchLastSection(False)
        self.tableWidget_3.verticalHeader().setVisible(False)
        self.tableWidget_3.verticalHeader().setCascadingSectionResizes(False)
        self.tableWidget_3.verticalHeader().setMinimumSectionSize(1)
        self.tableWidget_3.verticalHeader().setDefaultSectionSize(27)
        self.tableWidget_3.verticalHeader().setHighlightSections(True)
        self.tableWidget_3.verticalHeader().setProperty("showSortIndicator", False)
        self.tableWidget_3.verticalHeader().setStretchLastSection(False)

        self.verticalLayout_4.addWidget(self.tableWidget_3)


        self.gridLayout.addLayout(self.verticalLayout_4, 1, 0, 1, 1)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.label_10 = QLabel(self.widget_2)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setFont(font2)

        self.verticalLayout_5.addWidget(self.label_10)

        self.tableWidget_4 = QTableWidget(self.widget_2)
        if (self.tableWidget_4.columnCount() < 3):
            self.tableWidget_4.setColumnCount(3)
        __qtablewidgetitem105 = QTableWidgetItem()
        self.tableWidget_4.setHorizontalHeaderItem(0, __qtablewidgetitem105)
        __qtablewidgetitem106 = QTableWidgetItem()
        self.tableWidget_4.setHorizontalHeaderItem(1, __qtablewidgetitem106)
        __qtablewidgetitem107 = QTableWidgetItem()
        self.tableWidget_4.setHorizontalHeaderItem(2, __qtablewidgetitem107)
        if (self.tableWidget_4.rowCount() < 8):
            self.tableWidget_4.setRowCount(8)
        __qtablewidgetitem108 = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(0, __qtablewidgetitem108)
        __qtablewidgetitem109 = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(1, __qtablewidgetitem109)
        __qtablewidgetitem110 = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(2, __qtablewidgetitem110)
        __qtablewidgetitem111 = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(3, __qtablewidgetitem111)
        __qtablewidgetitem112 = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(4, __qtablewidgetitem112)
        __qtablewidgetitem113 = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(5, __qtablewidgetitem113)
        __qtablewidgetitem114 = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(6, __qtablewidgetitem114)
        __qtablewidgetitem115 = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(7, __qtablewidgetitem115)
        __qtablewidgetitem116 = QTableWidgetItem()
        __qtablewidgetitem116.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(0, 0, __qtablewidgetitem116)
        __qtablewidgetitem117 = QTableWidgetItem()
        __qtablewidgetitem117.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(0, 1, __qtablewidgetitem117)
        __qtablewidgetitem118 = QTableWidgetItem()
        __qtablewidgetitem118.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(0, 2, __qtablewidgetitem118)
        __qtablewidgetitem119 = QTableWidgetItem()
        __qtablewidgetitem119.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(1, 0, __qtablewidgetitem119)
        __qtablewidgetitem120 = QTableWidgetItem()
        __qtablewidgetitem120.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(1, 1, __qtablewidgetitem120)
        __qtablewidgetitem121 = QTableWidgetItem()
        __qtablewidgetitem121.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(1, 2, __qtablewidgetitem121)
        __qtablewidgetitem122 = QTableWidgetItem()
        __qtablewidgetitem122.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(2, 0, __qtablewidgetitem122)
        __qtablewidgetitem123 = QTableWidgetItem()
        __qtablewidgetitem123.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(2, 1, __qtablewidgetitem123)
        __qtablewidgetitem124 = QTableWidgetItem()
        __qtablewidgetitem124.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(2, 2, __qtablewidgetitem124)
        __qtablewidgetitem125 = QTableWidgetItem()
        __qtablewidgetitem125.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(3, 0, __qtablewidgetitem125)
        __qtablewidgetitem126 = QTableWidgetItem()
        __qtablewidgetitem126.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(3, 1, __qtablewidgetitem126)
        __qtablewidgetitem127 = QTableWidgetItem()
        __qtablewidgetitem127.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(3, 2, __qtablewidgetitem127)
        __qtablewidgetitem128 = QTableWidgetItem()
        __qtablewidgetitem128.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(4, 0, __qtablewidgetitem128)
        __qtablewidgetitem129 = QTableWidgetItem()
        __qtablewidgetitem129.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(4, 1, __qtablewidgetitem129)
        __qtablewidgetitem130 = QTableWidgetItem()
        __qtablewidgetitem130.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(4, 2, __qtablewidgetitem130)
        __qtablewidgetitem131 = QTableWidgetItem()
        __qtablewidgetitem131.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(5, 0, __qtablewidgetitem131)
        __qtablewidgetitem132 = QTableWidgetItem()
        __qtablewidgetitem132.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(5, 1, __qtablewidgetitem132)
        __qtablewidgetitem133 = QTableWidgetItem()
        __qtablewidgetitem133.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(5, 2, __qtablewidgetitem133)
        __qtablewidgetitem134 = QTableWidgetItem()
        __qtablewidgetitem134.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(6, 0, __qtablewidgetitem134)
        __qtablewidgetitem135 = QTableWidgetItem()
        __qtablewidgetitem135.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(6, 1, __qtablewidgetitem135)
        __qtablewidgetitem136 = QTableWidgetItem()
        __qtablewidgetitem136.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(6, 2, __qtablewidgetitem136)
        __qtablewidgetitem137 = QTableWidgetItem()
        __qtablewidgetitem137.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(7, 0, __qtablewidgetitem137)
        __qtablewidgetitem138 = QTableWidgetItem()
        __qtablewidgetitem138.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(7, 1, __qtablewidgetitem138)
        __qtablewidgetitem139 = QTableWidgetItem()
        __qtablewidgetitem139.setTextAlignment(Qt.AlignCenter);
        self.tableWidget_4.setItem(7, 2, __qtablewidgetitem139)
        self.tableWidget_4.setObjectName(u"tableWidget_4")
        self.tableWidget_4.setAutoScrollMargin(16)
        self.tableWidget_4.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableWidget_4.setSelectionBehavior(QAbstractItemView.SelectItems)
        self.tableWidget_4.setShowGrid(True)
        self.tableWidget_4.setGridStyle(Qt.SolidLine)
        self.tableWidget_4.setSortingEnabled(False)
        self.tableWidget_4.horizontalHeader().setCascadingSectionResizes(False)
        self.tableWidget_4.horizontalHeader().setProperty("showSortIndicator", False)
        self.tableWidget_4.horizontalHeader().setStretchLastSection(False)
        self.tableWidget_4.verticalHeader().setVisible(False)
        self.tableWidget_4.verticalHeader().setCascadingSectionResizes(False)
        self.tableWidget_4.verticalHeader().setMinimumSectionSize(1)
        self.tableWidget_4.verticalHeader().setDefaultSectionSize(27)
        self.tableWidget_4.verticalHeader().setHighlightSections(True)
        self.tableWidget_4.verticalHeader().setProperty("showSortIndicator", False)
        self.tableWidget_4.verticalHeader().setStretchLastSection(False)

        self.verticalLayout_5.addWidget(self.tableWidget_4)


        self.gridLayout.addLayout(self.verticalLayout_5, 1, 1, 1, 1)


        self.horizontalLayout_5.addWidget(self.widget_2)


        self.verticalLayout_6.addLayout(self.horizontalLayout_5)


        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"myStudio Pro", None))
        self.label.setText(QCoreApplication.translate("Form", u"myStudio Pro", None))
        self.label_2.setText(QCoreApplication.translate("Form", u"Robotics Port", None))
        self.label_3.setText(QCoreApplication.translate("Form", u"LC Port:", None))
        self.label_4.setText(QCoreApplication.translate("Form", u"RC Port:", None))
        self.label_5.setText(QCoreApplication.translate("Form", u"LM Port:", None))
        self.label_6.setText(QCoreApplication.translate("Form", u"RM Port\uff1a", None))
        self.label_7.setText(QCoreApplication.translate("Form", u"LC-ARM", None))
        ___qtablewidgetitem = self.tableWidget.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("Form", u"Joint", None));
        ___qtablewidgetitem1 = self.tableWidget.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("Form", u"Encoder", None));
        ___qtablewidgetitem2 = self.tableWidget.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("Form", u"Speed", None));
        ___qtablewidgetitem3 = self.tableWidget.verticalHeaderItem(0)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem4 = self.tableWidget.verticalHeaderItem(1)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem5 = self.tableWidget.verticalHeaderItem(2)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem6 = self.tableWidget.verticalHeaderItem(3)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem7 = self.tableWidget.verticalHeaderItem(4)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem8 = self.tableWidget.verticalHeaderItem(5)
        ___qtablewidgetitem8.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem9 = self.tableWidget.verticalHeaderItem(6)
        ___qtablewidgetitem9.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem10 = self.tableWidget.verticalHeaderItem(7)
        ___qtablewidgetitem10.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));

        __sortingEnabled = self.tableWidget.isSortingEnabled()
        self.tableWidget.setSortingEnabled(False)
        ___qtablewidgetitem11 = self.tableWidget.item(0, 0)
        ___qtablewidgetitem11.setText(QCoreApplication.translate("Form", u"1", None));
        ___qtablewidgetitem12 = self.tableWidget.item(0, 1)
        ___qtablewidgetitem12.setText(QCoreApplication.translate("Form", u"20", None));
        ___qtablewidgetitem13 = self.tableWidget.item(0, 2)
        ___qtablewidgetitem13.setText(QCoreApplication.translate("Form", u"10", None));
        ___qtablewidgetitem14 = self.tableWidget.item(1, 0)
        ___qtablewidgetitem14.setText(QCoreApplication.translate("Form", u"2", None));
        ___qtablewidgetitem15 = self.tableWidget.item(2, 0)
        ___qtablewidgetitem15.setText(QCoreApplication.translate("Form", u"3", None));
        ___qtablewidgetitem16 = self.tableWidget.item(3, 0)
        ___qtablewidgetitem16.setText(QCoreApplication.translate("Form", u"4", None));
        ___qtablewidgetitem17 = self.tableWidget.item(4, 0)
        ___qtablewidgetitem17.setText(QCoreApplication.translate("Form", u"5", None));
        ___qtablewidgetitem18 = self.tableWidget.item(5, 0)
        ___qtablewidgetitem18.setText(QCoreApplication.translate("Form", u"6", None));
        ___qtablewidgetitem19 = self.tableWidget.item(6, 0)
        ___qtablewidgetitem19.setText(QCoreApplication.translate("Form", u"7", None));
        ___qtablewidgetitem20 = self.tableWidget.item(7, 0)
        ___qtablewidgetitem20.setText(QCoreApplication.translate("Form", u"8", None));
        self.tableWidget.setSortingEnabled(__sortingEnabled)

        self.label_8.setText(QCoreApplication.translate("Form", u"RC-ARM", None))
        ___qtablewidgetitem21 = self.tableWidget_2.horizontalHeaderItem(0)
        ___qtablewidgetitem21.setText(QCoreApplication.translate("Form", u"Joint", None));
        ___qtablewidgetitem22 = self.tableWidget_2.horizontalHeaderItem(1)
        ___qtablewidgetitem22.setText(QCoreApplication.translate("Form", u"Encoder", None));
        ___qtablewidgetitem23 = self.tableWidget_2.horizontalHeaderItem(2)
        ___qtablewidgetitem23.setText(QCoreApplication.translate("Form", u"Speed", None));
        ___qtablewidgetitem24 = self.tableWidget_2.verticalHeaderItem(0)
        ___qtablewidgetitem24.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem25 = self.tableWidget_2.verticalHeaderItem(1)
        ___qtablewidgetitem25.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem26 = self.tableWidget_2.verticalHeaderItem(2)
        ___qtablewidgetitem26.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem27 = self.tableWidget_2.verticalHeaderItem(3)
        ___qtablewidgetitem27.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem28 = self.tableWidget_2.verticalHeaderItem(4)
        ___qtablewidgetitem28.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem29 = self.tableWidget_2.verticalHeaderItem(5)
        ___qtablewidgetitem29.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem30 = self.tableWidget_2.verticalHeaderItem(6)
        ___qtablewidgetitem30.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem31 = self.tableWidget_2.verticalHeaderItem(7)
        ___qtablewidgetitem31.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));

        __sortingEnabled1 = self.tableWidget_2.isSortingEnabled()
        self.tableWidget_2.setSortingEnabled(False)
        ___qtablewidgetitem32 = self.tableWidget_2.item(0, 0)
        ___qtablewidgetitem32.setText(QCoreApplication.translate("Form", u"1", None));
        ___qtablewidgetitem33 = self.tableWidget_2.item(0, 1)
        ___qtablewidgetitem33.setText(QCoreApplication.translate("Form", u"20", None));
        ___qtablewidgetitem34 = self.tableWidget_2.item(0, 2)
        ___qtablewidgetitem34.setText(QCoreApplication.translate("Form", u"10", None));
        ___qtablewidgetitem35 = self.tableWidget_2.item(1, 0)
        ___qtablewidgetitem35.setText(QCoreApplication.translate("Form", u"2", None));
        ___qtablewidgetitem36 = self.tableWidget_2.item(2, 0)
        ___qtablewidgetitem36.setText(QCoreApplication.translate("Form", u"3", None));
        ___qtablewidgetitem37 = self.tableWidget_2.item(3, 0)
        ___qtablewidgetitem37.setText(QCoreApplication.translate("Form", u"4", None));
        ___qtablewidgetitem38 = self.tableWidget_2.item(4, 0)
        ___qtablewidgetitem38.setText(QCoreApplication.translate("Form", u"5", None));
        ___qtablewidgetitem39 = self.tableWidget_2.item(5, 0)
        ___qtablewidgetitem39.setText(QCoreApplication.translate("Form", u"6", None));
        ___qtablewidgetitem40 = self.tableWidget_2.item(6, 0)
        ___qtablewidgetitem40.setText(QCoreApplication.translate("Form", u"7", None));
        ___qtablewidgetitem41 = self.tableWidget_2.item(7, 0)
        ___qtablewidgetitem41.setText(QCoreApplication.translate("Form", u"8", None));
        self.tableWidget_2.setSortingEnabled(__sortingEnabled1)

        self.label_9.setText(QCoreApplication.translate("Form", u"LM-ARM", None))
        ___qtablewidgetitem42 = self.tableWidget_3.horizontalHeaderItem(0)
        ___qtablewidgetitem42.setText(QCoreApplication.translate("Form", u"Joint", None));
        ___qtablewidgetitem43 = self.tableWidget_3.horizontalHeaderItem(1)
        ___qtablewidgetitem43.setText(QCoreApplication.translate("Form", u"Encoder", None));
        ___qtablewidgetitem44 = self.tableWidget_3.horizontalHeaderItem(2)
        ___qtablewidgetitem44.setText(QCoreApplication.translate("Form", u"Speed", None));
        ___qtablewidgetitem45 = self.tableWidget_3.verticalHeaderItem(0)
        ___qtablewidgetitem45.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem46 = self.tableWidget_3.verticalHeaderItem(1)
        ___qtablewidgetitem46.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem47 = self.tableWidget_3.verticalHeaderItem(2)
        ___qtablewidgetitem47.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem48 = self.tableWidget_3.verticalHeaderItem(3)
        ___qtablewidgetitem48.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem49 = self.tableWidget_3.verticalHeaderItem(4)
        ___qtablewidgetitem49.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem50 = self.tableWidget_3.verticalHeaderItem(5)
        ___qtablewidgetitem50.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem51 = self.tableWidget_3.verticalHeaderItem(6)
        ___qtablewidgetitem51.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem52 = self.tableWidget_3.verticalHeaderItem(7)
        ___qtablewidgetitem52.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));

        __sortingEnabled2 = self.tableWidget_3.isSortingEnabled()
        self.tableWidget_3.setSortingEnabled(False)
        ___qtablewidgetitem53 = self.tableWidget_3.item(0, 0)
        ___qtablewidgetitem53.setText(QCoreApplication.translate("Form", u"1", None));
        ___qtablewidgetitem54 = self.tableWidget_3.item(0, 1)
        ___qtablewidgetitem54.setText(QCoreApplication.translate("Form", u"20", None));
        ___qtablewidgetitem55 = self.tableWidget_3.item(0, 2)
        ___qtablewidgetitem55.setText(QCoreApplication.translate("Form", u"10", None));
        ___qtablewidgetitem56 = self.tableWidget_3.item(1, 0)
        ___qtablewidgetitem56.setText(QCoreApplication.translate("Form", u"2", None));
        ___qtablewidgetitem57 = self.tableWidget_3.item(2, 0)
        ___qtablewidgetitem57.setText(QCoreApplication.translate("Form", u"3", None));
        ___qtablewidgetitem58 = self.tableWidget_3.item(3, 0)
        ___qtablewidgetitem58.setText(QCoreApplication.translate("Form", u"4", None));
        ___qtablewidgetitem59 = self.tableWidget_3.item(4, 0)
        ___qtablewidgetitem59.setText(QCoreApplication.translate("Form", u"5", None));
        ___qtablewidgetitem60 = self.tableWidget_3.item(5, 0)
        ___qtablewidgetitem60.setText(QCoreApplication.translate("Form", u"6", None));
        ___qtablewidgetitem61 = self.tableWidget_3.item(6, 0)
        ___qtablewidgetitem61.setText(QCoreApplication.translate("Form", u"7", None));
        ___qtablewidgetitem62 = self.tableWidget_3.item(7, 0)
        ___qtablewidgetitem62.setText(QCoreApplication.translate("Form", u"8", None));
        self.tableWidget_3.setSortingEnabled(__sortingEnabled2)

        self.label_10.setText(QCoreApplication.translate("Form", u"RM-ARM", None))
        ___qtablewidgetitem63 = self.tableWidget_4.horizontalHeaderItem(0)
        ___qtablewidgetitem63.setText(QCoreApplication.translate("Form", u"Joint", None));
        ___qtablewidgetitem64 = self.tableWidget_4.horizontalHeaderItem(1)
        ___qtablewidgetitem64.setText(QCoreApplication.translate("Form", u"Encoder", None));
        ___qtablewidgetitem65 = self.tableWidget_4.horizontalHeaderItem(2)
        ___qtablewidgetitem65.setText(QCoreApplication.translate("Form", u"Speed", None));
        ___qtablewidgetitem66 = self.tableWidget_4.verticalHeaderItem(0)
        ___qtablewidgetitem66.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem67 = self.tableWidget_4.verticalHeaderItem(1)
        ___qtablewidgetitem67.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem68 = self.tableWidget_4.verticalHeaderItem(2)
        ___qtablewidgetitem68.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem69 = self.tableWidget_4.verticalHeaderItem(3)
        ___qtablewidgetitem69.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem70 = self.tableWidget_4.verticalHeaderItem(4)
        ___qtablewidgetitem70.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem71 = self.tableWidget_4.verticalHeaderItem(5)
        ___qtablewidgetitem71.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem72 = self.tableWidget_4.verticalHeaderItem(6)
        ___qtablewidgetitem72.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem73 = self.tableWidget_4.verticalHeaderItem(7)
        ___qtablewidgetitem73.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));

        __sortingEnabled3 = self.tableWidget_4.isSortingEnabled()
        self.tableWidget_4.setSortingEnabled(False)
        ___qtablewidgetitem74 = self.tableWidget_4.item(0, 0)
        ___qtablewidgetitem74.setText(QCoreApplication.translate("Form", u"1", None));
        ___qtablewidgetitem75 = self.tableWidget_4.item(0, 1)
        ___qtablewidgetitem75.setText(QCoreApplication.translate("Form", u"20", None));
        ___qtablewidgetitem76 = self.tableWidget_4.item(0, 2)
        ___qtablewidgetitem76.setText(QCoreApplication.translate("Form", u"10", None));
        ___qtablewidgetitem77 = self.tableWidget_4.item(1, 0)
        ___qtablewidgetitem77.setText(QCoreApplication.translate("Form", u"2", None));
        ___qtablewidgetitem78 = self.tableWidget_4.item(2, 0)
        ___qtablewidgetitem78.setText(QCoreApplication.translate("Form", u"3", None));
        ___qtablewidgetitem79 = self.tableWidget_4.item(3, 0)
        ___qtablewidgetitem79.setText(QCoreApplication.translate("Form", u"4", None));
        ___qtablewidgetitem80 = self.tableWidget_4.item(4, 0)
        ___qtablewidgetitem80.setText(QCoreApplication.translate("Form", u"5", None));
        ___qtablewidgetitem81 = self.tableWidget_4.item(5, 0)
        ___qtablewidgetitem81.setText(QCoreApplication.translate("Form", u"6", None));
        ___qtablewidgetitem82 = self.tableWidget_4.item(6, 0)
        ___qtablewidgetitem82.setText(QCoreApplication.translate("Form", u"7", None));
        ___qtablewidgetitem83 = self.tableWidget_4.item(7, 0)
        ___qtablewidgetitem83.setText(QCoreApplication.translate("Form", u"8", None));
        self.tableWidget_4.setSortingEnabled(__sortingEnabled3)

    # retranslateUi

