from cProfile import label
from turtle import clear, color
from PyQt5.QtWidgets import QApplication, QWidget,QComboBox,QVBoxLayout,QCheckBox,QHBoxLayout,QPushButton,QLabel,QFrame,QMainWindow,QLineEdit,QProgressBar,QFileDialog,QDesktopWidget
from PyQt5.QtGui import QIntValidator
from PyQt5.QtCore import QIODevice,Qt,QByteArray,QDataStream
from PyQt5.QtSerialPort import QSerialPort
from pyqtgraph import PlotWidget, mkPen, ViewBox, plot
#from ThreadCompute import ThreadCompute
from time import time,sleep
import sys
import serial
import glob
import csv

class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("IHM : Maquette ZG2 - 2023_P")
        self.layout = QFrame(self)
        self.setCentralWidget(self.layout)
        self.sizeOfScreen = QDesktopWidget().screenGeometry(-1)
        self.UIComponents()

    def UIComponents(self):
        self.layout_principale = QVBoxLayout(self.layout)

        # Initialisation des parties de l'IHM
        self.parties = QHBoxLayout()

        # Initialisation du panneau de commande
        self.panneau_de_commandes = QVBoxLayout()
        self.panneau_de_parametrages = QVBoxLayout()
        self.partie_panneaux = QHBoxLayout()
        self.partie_controle = QVBoxLayout()

        # Initialisation des frames
        # Frame connect
        self.frame_connexion()
        # Frame parametrage
        self.frame_parametre()
        # Frame fonctionnement moteur
        self.frame_moteur()
        self.setHiddenParametersMotor(True)

        # Partie bouton
        self.partie_bouton = QVBoxLayout()
        self.bouton_acquisition = QPushButton('Lancement acquisition')
        #Connexion du clic sur le bouton à la fonction enabledAcquistion
        self.bouton_acquisition.clicked.connect(self.enabledAcquistion)
        self.bouton_acquisition.setDisabled(True)
        self.bouton_acquisition.setMinimumHeight(50)
        self.bouton_acquisition.setStyleSheet("QPushButton"
                            "{"
                            "font-size: 16px;"
                            "background-color : #c4df9b;"
                            "border-style: outset;"
                            "padding: 5px;"
                            "}"
                            "QPushButton:disabled"
                            "{"
                            "font-size: 16px;"
                            "background-color : #eef6e3;"
                            "border-style: outset;"
                            "padding: 5px;"
                            "}")

        #Déclaration du bouton de sauvegarde des données
        self.boutonCsv = QPushButton("Exportation des données")
        self.boutonCsv.clicked.connect(self.createFile)
        self.boutonCsv.setDisabled(True)
        self.boutonCsv.setMinimumHeight(50)
        self.boutonCsv.setStyleSheet("font-size: 16px;")

        #Déclaration du bouton stop de l'acquisition
        self.bouton_stop = QPushButton('Fin acquisition')
        self.bouton_stop.setDisabled(True)
        self.bouton_stop.clicked.connect(self.stopAcquisition)
        self.bouton_stop.setMinimumHeight(50)
        self.bouton_stop.setStyleSheet("QPushButton"
                            "{"
                            "font-size: 16px;"
                            "background-color : #ff797b;"
                            "border-style: outset;"
                            "padding: 5px;"
                            "}"
                            "QPushButton:disabled"
                            "{"
                            "font-size: 16px;"
                            "background-color : #ffd6d7;"
                            "border-style: outset;"
                            "padding: 5px;"
                            "}")

        ##Déclaration de l'alignement de la partie bouton
        self.partie_bouton.setAlignment(Qt.AlignVCenter)

        # Ajout des frames dans le panneau de commande
        self.panneau_de_commandes.addWidget(self.frame_connect)
        self.panneau_de_commandes.addWidget(self.frame_fonctionnement)

        # Ajout des frame à la grande partie
        self.partie_panneaux.addLayout(self.panneau_de_commandes)
        self.panneau_de_parametrages.addWidget(self.frame_parametrage)
        self.partie_panneaux.addLayout(self.panneau_de_parametrages)

        #initialisation des variables
        self.freq_acq = 0 # fréquence d'aqquisition
        self.state_acq = 0 # 0 pas défini, 1 click lancement acq, 2 click arret acq
        self.state_mode_oscil = 0 # 0 osciliation libre, 1 osciliation forcée
        self.state_ctrl_mot = 0 # 0 Auto, 1 manuel
        self.kp = 0 # Kp PID
        self.ki = 0 # Ki PID
        self.kd = 0 # Kd PID
        self.speed_order = 0 # Consigne de vitesse du moteur mode manuel
        self.min_speed_order = 0 # Consigne min de vitesse du moteur mode auto
        self.max_speed_order = 0 # Consigne max de vitesse du moteur mode auto
        self.nb_pas = 0 # Nombre de pallier de vitesse mode auto
        self.tps_simu = 0 # Temps de simulation

        #Mise en place de la courbe
        self.data_cpt = []
        self.current_data_cpt = 0
        self.data_rpm = []
        self.current_data_rpm = 0
        self.data_time = []
        self.current_time = 0
        self.graph = PlotWidget(title="Courbe expérimentale",background='w')
        self.graph.plotItem.setLabel("left","distance (mm)")
        self.graph.plotItem.setLabel("right","RPM (tr/min)")
        self.graph.plotItem.setLabel("bottom","temps (s)")
        pen=mkPen('g', width=2)
        pen2=mkPen('r', width=1)
        self.data_graph = self.graph.plot(self.data_time,self.data_cpt,"Distance_Sensor",pen=pen)
        self.data_graph2 = self.graph.plot(self.data_time,self.data_rpm,"RPM",pen=pen2)

        # Ajout des boutons parties boutons
        self.partie_bouton.addWidget(self.bouton_acquisition)
        self.partie_bouton.addWidget(self.bouton_stop)
        self.partie_bouton.addWidget(self.boutonCsv)

        self.partie_controle.addLayout(self.partie_panneaux)
        self.partie_controle.addLayout(self.partie_bouton)

        self.parties.addLayout(self.partie_controle)
        self.parties.addWidget(self.graph)

        # Ajout des parties aux layout global
        self.layout_principale.addLayout(self.parties)
   
    def frame_moteur(self):
        self.frame_fonctionnement = QFrame()
        self.frame_fonctionnement.setFrameStyle(QFrame.Panel | QFrame.Raised)
        self.frame_fonctionnement.setLineWidth(2)

        self.frame_fonctionnement.setMaximumWidth(int(self.sizeOfScreen.width()*0.15))

        self.intern_frame_fonctionnement = QVBoxLayout(self.frame_fonctionnement)
        self.intern_frame_fonctionnement.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.intern_intern_frame_fonctionnement = QHBoxLayout()
        self.intern_intern_frame_fonctionnement.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.intern_intern_frame_fonctionnement_bouton = QVBoxLayout()

        self.label_titre_fonctionnement = QLabel()
        self.label_titre_fonctionnement.setText("Mode de fonctionnement\n")
        self.label_titre_fonctionnement.setStyleSheet("font-size: 16px;")

        # mode label
        self.zone_label_mode = QHBoxLayout()
        self.label_mode_texte = QLabel()
        self.label_mode_texte.setText("Mode :")
        self.liste_mode = QComboBox()
        self.mode_liste = ["Oscillations libres","Oscillations forcés"]
        self.liste_mode.addItems(self.mode_liste)
        self.liste_mode.currentIndexChanged.connect(self.updatemode)
        self.zone_label_mode.addWidget(self.label_mode_texte)
        self.zone_label_mode.addWidget(self.liste_mode)
        self.zone_label_mode.setAlignment(Qt.AlignLeft) 
        self.liste_mode.setEnabled(False)

        self.zone_label_mode_vitesse_moteur = QHBoxLayout()
        self.label_mode_vitesse_texte = QLabel()
        self.label_mode_vitesse_texte.setText("Mode vitesse :")
        self.liste_mode_vitesse = QComboBox(self)
        self.mode_vitesse_liste = ["Auto","Manuel"]
        self.liste_mode_vitesse.addItems(self.mode_vitesse_liste)
        self.liste_mode_vitesse.currentIndexChanged.connect(self.choixVitesseMoteur)
        self.zone_label_mode_vitesse_moteur.addWidget(self.label_mode_vitesse_texte)
        self.zone_label_mode_vitesse_moteur.addWidget(self.liste_mode_vitesse)
        self.zone_label_mode_vitesse_moteur.setAlignment(Qt.AlignLeft)
        self.liste_mode_vitesse.setEnabled(False)

        self.zone_label_min_vitesse_moteur = QHBoxLayout()
        self.label_min_vitesse_texte = QLabel()
        self.label_min_vitesse_texte.setText("Min vitesse (max 330):")
        self.edit_min_vitesse = QLineEdit("30")
        self.edit_min_vitesse.textChanged.connect(self.choixVitesseMoteur)
        self.zone_label_min_vitesse_moteur.addWidget(self.label_min_vitesse_texte)
        self.zone_label_min_vitesse_moteur.addWidget(self.edit_min_vitesse)
        self.zone_label_min_vitesse_moteur.setAlignment(Qt.AlignLeft)
        self.edit_min_vitesse.setEnabled(False)

        self.zone_label_max_vitesse_moteur = QHBoxLayout()
        self.label_max_vitesse_texte = QLabel()
        self.label_max_vitesse_texte.setText("Max vitesse (max 330):")
        self.edit_max_vitesse = QLineEdit("330")
        self.edit_max_vitesse.textChanged.connect(self.choixVitesseMoteur)
        self.zone_label_max_vitesse_moteur.addWidget(self.label_max_vitesse_texte)
        self.zone_label_max_vitesse_moteur.addWidget(self.edit_max_vitesse)
        self.zone_label_max_vitesse_moteur.setAlignment(Qt.AlignLeft)
        self.edit_max_vitesse.setEnabled(False)

        self.zone_label_pas = QHBoxLayout()
        self.label_pas = QLabel()
        self.label_pas.setText("Nombre de pas :")
        self.edit_pas = QLineEdit("2")
        self.edit_pas.textChanged.connect(self.choixVitesseMoteur)
        self.zone_label_pas.addWidget(self.label_pas)
        self.zone_label_pas.addWidget(self.edit_pas)
        self.zone_label_pas.setAlignment(Qt.AlignLeft)
        self.edit_pas.setEnabled(False)

        self.zone_label_vitesse_moteur = QHBoxLayout()
        self.label_vitesse_texte = QLabel()
        self.label_vitesse_texte.setText("Vitesse tr/min (max 330):")
        self.edit_vitesse = QLineEdit("150")
        self.edit_vitesse.textChanged.connect(self.choixVitesseMoteur)
        self.zone_label_vitesse_moteur.addWidget(self.label_vitesse_texte)
        self.zone_label_vitesse_moteur.addWidget(self.edit_vitesse)
        self.zone_label_vitesse_moteur.setAlignment(Qt.AlignLeft)
        self.edit_vitesse.setEnabled(False)

        self.intern_frame_fonctionnement.addWidget(self.label_titre_fonctionnement)
        self.intern_intern_frame_fonctionnement_bouton.addLayout(self.zone_label_mode)
        self.intern_intern_frame_fonctionnement_bouton.addLayout(self.zone_label_mode_vitesse_moteur)
        self.intern_intern_frame_fonctionnement_bouton.addLayout(self.zone_label_min_vitesse_moteur)
        self.intern_intern_frame_fonctionnement_bouton.addLayout(self.zone_label_max_vitesse_moteur)
        self.intern_intern_frame_fonctionnement_bouton.addLayout(self.zone_label_pas)
        self.intern_intern_frame_fonctionnement_bouton.addLayout(self.zone_label_vitesse_moteur)
        self.intern_intern_frame_fonctionnement.addLayout(self.intern_intern_frame_fonctionnement_bouton)
        self.intern_frame_fonctionnement.addLayout(self.intern_intern_frame_fonctionnement)

    def frame_parametre(self):
        self.frame_parametrage = QFrame()
        self.frame_parametrage.setFrameStyle(QFrame.Panel | QFrame.Raised)
        self.frame_parametrage.setLineWidth(2)

        self.frame_parametrage.setMaximumWidth(int(self.sizeOfScreen.width()*0.18))

        self.intern_frame_parametrage = QVBoxLayout(self.frame_parametrage)
        self.intern_frame_parametrage.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.label_titre_parametrage = QLabel()
        self.label_titre_parametrage.setText("Paramètres de simulation\n")
        self.label_titre_parametrage.setStyleSheet("font-size: 16px;")

        self.zone_label_freq_acq = QHBoxLayout()
        self.label_freq_acq_texte = QLabel()
        self.label_freq_acq_texte.setText("Fréquence aquisition (Hz) :")
        self.line_edit_freq_acq = QLineEdit("60")
        self.zone_label_freq_acq.addWidget(self.label_freq_acq_texte)
        self.zone_label_freq_acq.addWidget(self.line_edit_freq_acq)
        self.zone_label_freq_acq.setAlignment(Qt.AlignLeft) 

        self.zone_label_temps_simu = QHBoxLayout()
        self.label_temps_simu_texte = QLabel()
        self.label_temps_simu_texte.setText("Temps de simulation (s) :")
        self.line_edit_temps_simu = QLineEdit("5")
        self.zone_label_temps_simu.addWidget(self.label_temps_simu_texte)
        self.zone_label_temps_simu.addWidget(self.line_edit_temps_simu)
        self.zone_label_temps_simu.setAlignment(Qt.AlignLeft) 

        self.zone_label_kp = QHBoxLayout() 
        self.label_kp_texte = QLabel()
        self.label_kp_texte.setText("Kp :")
        self.line_edit_kp = QLineEdit("0.05")
        self.zone_label_kp.addWidget(self.label_kp_texte)
        self.zone_label_kp.addWidget(self.line_edit_kp)
        self.zone_label_kp.setAlignment(Qt.AlignLeft) 

        self.zone_label_ki = QHBoxLayout() 
        self.label_ki_texte = QLabel()
        self.label_ki_texte.setText("Ki :")
        self.line_edit_ki = QLineEdit("0.03")
        self.zone_label_ki.addWidget(self.label_ki_texte)
        self.zone_label_ki.addWidget(self.line_edit_ki)
        self.zone_label_ki.setAlignment(Qt.AlignLeft) 

        self.zone_label_kd = QHBoxLayout() 
        self.label_kd_texte = QLabel()
        self.label_kd_texte.setText("Kd :")
        self.line_edit_kd = QLineEdit("0.01")
        self.zone_label_kd.addWidget(self.label_kd_texte)
        self.zone_label_kd.addWidget(self.line_edit_kd)
        self.zone_label_kd.setAlignment(Qt.AlignLeft) 

        self.intern_frame_parametrage.addWidget(self.label_titre_parametrage)
        self.intern_frame_parametrage.addLayout(self.zone_label_freq_acq)
        self.intern_frame_parametrage.addLayout(self.zone_label_temps_simu)
        self.intern_frame_parametrage.addLayout(self.zone_label_kp)
        self.intern_frame_parametrage.addLayout(self.zone_label_ki)
        self.intern_frame_parametrage.addLayout(self.zone_label_kd)
 
    def frame_connexion(self):
        self.frame_connect = QFrame()
        self.frame_connect.setFrameStyle(QFrame.Panel | QFrame.Raised)
        self.frame_connect.setLineWidth(2)

        self.frame_connect.setMaximumWidth(int(self.sizeOfScreen.width()*0.15))

        self.intern_frame_connect = QVBoxLayout(self.frame_connect)
        self.intern_frame_connect.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.label_titre_connect = QLabel()
        self.label_titre_connect.setText("Connexion à la carte\n")
        self.label_titre_connect.setStyleSheet("font-size: 16px;")
        
        self.zone_label_port = QHBoxLayout()
        self.label_port_texte = QLabel()
        self.label_port_texte.setText("Port :")
        self.liste_port = QComboBox(self)
        self.port_liste = []
        self.liste_port.addItems(self.port_liste)
        self.bouton_recherche_port = QPushButton('Recherche des ports')
        self.bouton_recherche_port.clicked.connect(self.findPorts)
        self.zone_label_port.addWidget(self.label_port_texte)
        self.zone_label_port.addWidget(self.liste_port)
        self.zone_label_port.addWidget(self.bouton_recherche_port)
        self.zone_label_port.setAlignment(Qt.AlignLeft)    

        self.zone_label_baud = QHBoxLayout()
        self.label_baud_texte = QLabel()
        self.label_baud_texte.setText("Baud Rate :")
        self.liste_baud = QComboBox(self)
        self.baud_liste = ["115200","9600", "14400", "19200", "38400","57600"]
        self.liste_baud.addItems(self.baud_liste)
        self.zone_label_baud.addWidget(self.label_baud_texte)
        self.zone_label_baud.addWidget(self.liste_baud)
        self.zone_label_baud.setAlignment(Qt.AlignLeft)

        self.bouton_connect_voyant = QHBoxLayout()
        self.bouton_connect = QPushButton('Connexion')
        self.bouton_connect.clicked.connect(self.connectClick)
        self.connect = False
        self.checkbox = QCheckBox()
        self.checkbox.setGeometry(200, 150, 100, 40)
        self.checkbox.setStyleSheet("QCheckBox::indicator""{""background-color : 'red';""}")
        self.bouton_connect_voyant.addWidget(self.bouton_connect)
        self.bouton_connect_voyant.addWidget(self.checkbox)
        self.bouton_connect_voyant.setAlignment(Qt.AlignCenter)

        self.intern_frame_connect.addWidget(self.label_titre_connect)
        self.intern_frame_connect.addLayout(self.zone_label_port)
        self.intern_frame_connect.addLayout(self.zone_label_baud)
        self.intern_frame_connect.addLayout(self.bouton_connect_voyant)

    def connectClick(self):
        ##Fonction de definition du port série 
        self.serial = QSerialPort(self.liste_port.currentText(),baudRate= int(self.liste_baud.currentText()))
        #Si je suis déjà connecté alors je me deconnecte sinon je me connecte
        if(self.connect):
            self.checkbox.setStyleSheet("QCheckBox::indicator""{""background-color : 'red';""}")
            self.bouton_connect.setText("Connexion")
            self.serial.close()
            self.bouton_acquisition.setDisabled(True)
            self.bouton_stop.setDisabled(True)
            self.boutonCsv.setDisabled(True)
            #self.setVisibilityParameters(False)
            self.liste_mode.setDisabled(True)
            self.connect = False
        else:
            ##Connexion et barre de status
            self.result = self.serial.open(QIODevice.ReadWrite) #Tentative de connexion

            if(self.result):
                print("connecté")
                self.connect = True
                #On change le style du voyant en vert
                self.checkbox.setStyleSheet("QCheckBox::indicator""{""background-color : 'green';""}")
                self.bouton_connect.setText("Déconnexion")
                self.bouton_acquisition.setDisabled(False)
                self.setVisibilityParameters(True)
                self.liste_mode.setDisabled(False)
                self.stop = True
                #On connecte le port serial à la fonction readData
                self.serial.readyRead.connect(self.readData)
            else:
                self.connect = False
                print("non connecté")

    def setVisibilityParameters(self,bool):
        ##Fonction permettant de rendre visible ou non les champs de saisie
        self.line_edit_freq_acq.setEnabled(bool)      

    def enabledAcquistion(self):
        self.data_cpt = []
        self.data_rpm = []
        self.data_time = []
        self.current_data_cpt = 0
        self.current_data_rpm = 0
        self.count = 0

        #Affectation des parametres de simulation
        self.freq_acq = str(self.line_edit_freq_acq.text()) # fréquence d'aqquisition
        self.state_acq = 1 # 0 pas défini, 1 click lancement acq, 2 click arret acq
        self.kp = str(self.line_edit_kp.text()) # Kp PID
        self.ki = str(self.line_edit_ki.text()) # Ki PID
        self.kd = str(self.line_edit_kd.text()) # Kd PID 
        self.speed_order = str(self.edit_vitesse.text()) # Consigne de vitesse du moteur mode manuel
        self.min_speed_order = str(self.edit_min_vitesse .text()) # Consigne min de vitesse du moteur mode auto
        self.max_speed_order = str(self.edit_max_vitesse.text()) # Consigne max de vitesse du moteur mode auto
        self.nb_pas = str(self.edit_pas.text()) # Nombre de pallier de vitesse mode auto
        self.tps_simu = str(self.line_edit_temps_simu.text()) # Temps de simulation
        print('Début aquisition')
        self.bouton_acquisition.setDisabled(True)
        self.bouton_stop.setDisabled(False)
        self.boutonCsv.setDisabled(True)
        self.writeData()

    def stopAcquisition(self):
        self.bouton_acquisition.setDisabled(False)
        self.boutonCsv.setDisabled(False)
        self.bouton_stop.setDisabled(True)
        self.setVisibilityParameters(True)
        self.state_acq = 2 # 0 pas défini, 1 click lancement acq, 2 click arret acq
        print('stop aquisition')
        self.writeData()

    def writeData(self):
        # Freq acq | Etat acq | Mode oscilation | Mode ctr moteur | coeffM(Kp/Ki/Kd) | ParamMoteur (consigne/min/max/nb_pas) | Temps simu
        msg ="{},{},{},{},{},{},{},{},{},{},{},{}/".format(int(self.freq_acq),int(self.state_acq),int(self.state_mode_oscil),
                                                             int(self.state_ctrl_mot),float(self.kp),float(self.ki),float(self.kd),
                                                             int(self.speed_order),int(self.min_speed_order),int(self.max_speed_order),
                                                             int(self.nb_pas),int(self.tps_simu))
        print(msg)
        self.serial.write(msg.encode())   

    def readData(self): #Quand l'arduino écrit une trame sur le port série (b'T20R100P200\r\n'), on stock le temps,la rpm et la pos
        print("read data function")
        #read_data = self.serial.readLine()
        #print(read_data)
        #print("---")
        if self.serial.canReadLine():
            read_data = self.serial.readLine()
            print(read_data)  
            read_data_temps = str(read_data)[str(read_data).find("T")+1:str(read_data).find("R")]
            read_data_rpm = str(read_data)[str(read_data).find("R")+1:str(read_data).find("P")]
            read_data_pos = str(read_data)[str(read_data).find("P")+1:str(read_data).find("C")]
            read_data_simuCplt = str(read_data)[str(read_data).find("C")+1:-5]

            print('temps =',read_data_temps)
            print('rpm =',read_data_rpm)
            print('pos =',read_data_pos)
            print('simuCplt =',read_data_simuCplt)
            print(" ")
        
            if (read_data_simuCplt == "1" or int(read_data_temps) >= int(self.tps_simu)*1000 ):
                self.bouton_acquisition.setDisabled(False)
                self.bouton_stop.setDisabled(True)
                self.boutonCsv.setDisabled(False)
            if (read_data_simuCplt == "0"):
                try:
                    temps = float(read_data_temps)
                    pos = int(read_data_pos)
                    rpm = int(read_data_rpm)
                    self.setDataCourbeWindow(pos,rpm,temps)
                except:
                    pass

    def findPorts(self,ok):
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
                self.port_liste = result
                self.liste_port.clear()
                self.liste_port.addItems(self.port_liste)
            except (OSError, serial.SerialException):
                pass
 
    def choixVitesseMoteur(self):
        if(self.liste_mode_vitesse.currentText() == "Manuel"): # mode manuel
            self.state_ctrl_mot = 1 # 0 Auto, 1 manuel
            self.edit_vitesse.setEnabled(True)
            self.edit_min_vitesse.setEnabled(False)
            self.edit_max_vitesse.setEnabled(False)
            self.edit_pas.setEnabled(False)
        else: #Mode Auto
            self.state_ctrl_mot = 0 # 0 Auto, 1 manuel
            self.edit_vitesse.setEnabled(False)
            self.edit_min_vitesse.setEnabled(True)
            self.edit_max_vitesse.setEnabled(True)
            self.edit_pas.setEnabled(True)

    def updatemode(self): # modes d'oscillations
        if(self.liste_mode.currentText() == self.mode_liste[0]):  # == Oscilation libre
            self.setHiddenParametersMotor(True)
            self.state_mode_oscil = 0 
        else: # oscillation forcée
            #self.setVisibilityParametersMotor(False)
            self.setHiddenParametersMotor(False)
            self.liste_mode_vitesse.setCurrentText("Auto")
            self.setVisibilityParametersMotor(True)
            self.state_mode_oscil = 1

    def setVisibilityParametersMotor(self,bool):
        ##Fonction permettant de rendre actif ou non les champs de saisie
        self.edit_min_vitesse.setEnabled(bool)
        self.edit_max_vitesse.setEnabled(bool)
        self.liste_mode_vitesse.setEnabled(bool)
        self.edit_pas.setEnabled(bool)

    def setHiddenParametersMotor(self,bool):
        ##Fonction permettant de rendre visible ou non les champs de saisie
        self.liste_mode_vitesse.setHidden(bool)
        self.label_mode_vitesse_texte.setHidden(bool)
        self.label_min_vitesse_texte.setHidden(bool)
        self.edit_min_vitesse.setHidden(bool)
        self.label_max_vitesse_texte.setHidden(bool)
        self.edit_max_vitesse.setHidden(bool)
        self.label_pas.setHidden(bool)
        self.edit_pas.setHidden(bool)
        self.label_vitesse_texte.setHidden(bool)
        self.edit_vitesse.setHidden(bool)
   
    def setDataCourbeWindow(self,data,data2,time):
        try:
            self.data_cpt.append(data)
            self.data_rpm.append(data2)
            self.data_time.append(time)
            self.data_graph.setData(self.data_time,self.data_cpt)
            self.data_graph2.setData(self.data_time,self.data_rpm)
        except:
            pass

    def createFile(self):
        entetes = ["Temps","Postion","Vitesse Moteur","Fréquence aquisition", "Temps de simulation","Kp","Ki","Kd","consigne vitesse moteur","consigne vitesse min","consigne vitesse max","Nombre de pallier"]
        print("File create")
        #print(str(self.data))
        try :
            ligne = [str(self.data_time[0]),str(self.data_cpt[0]),str(self.data_rpm[0]),str(self.freq_acq),str(self.tps_simu),str(self.kp),str(self.ki),str(self.kd),str(self.speed_order),
                     str(self.min_speed_order),str(self.max_speed_order),str(self.nb_pas)]
        except:
            ligne = [str(0),str(0),str(0),str(0),str(0),str(0),str(0),str(0),str(0),str(0),str(0),str(0)]

        lignes = []
        lignes.append(ligne)

        for k in range(len(self.data_time)-1):
            ligne = [str(self.data_time[k+1]),str(self.data_cpt[k+1]),str(self.data_rpm[k+1])]
            lignes.append(ligne)

        path_save_json, _ = QFileDialog.getSaveFileName(self, 'Save File', "resultatsZG2.csv", 
                "CSV (*.csv)",None ,QFileDialog.DontConfirmOverwrite)

        if(path_save_json):
            f = open(path_save_json, "w",newline='',errors='ignore')
            ligneEntete = ";".join(entetes) + "\n"
            f.write(ligneEntete)
            for ligne in lignes:
                f.write(";".join(ligne) + "\n")
            f.close()
            
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
