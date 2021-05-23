# state 1
#   B1: Locate Chessboard  ==> task planning call detect chessboard 
#   B2: Detect Chessboard  ==> task planning call chessboard state
#                                correct the detection 
#   B3: Move Done          ==> task planning call robot move
#                          ==>  
#publisher and subscriber info from system and interact with User
#call the function and sent
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys

def window():
    app = QApplication(sys.argv)
    win = QMainWindow()
    win.setGeometry(200,200,300,300)
    win.setWindowTitle('abowkd')

    label = QtWidgets.QLabel(win)
    label.setText("Locating Chessboard")

    b1 = QtWidgets
    label.move(50,50)
    win.show()
    sys.exit(app.exec_())

window()