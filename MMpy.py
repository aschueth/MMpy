'''
This program was written by Alex Schueth for the use in the Integrated Mesonet and Tracker (IMeT) for Dr. Adam Houston at the University of Nebraska-Lincoln.
This program was written in Python 2.7
The purpose of this program is to read atmospheric data from serial, time sync the data to the GPS time, and then plot the data real-time in a framework that is easily manipulated
The last commit of this program was on 8/31/2016
Any questions or comments, Alex can be reached at:   schueth.alex@gmail.com
'''
from multiprocessing import Process
from datetime import datetime, timedelta
from time import gmtime, strftime
import serial, io, sys, Queue, multiprocessing, time, os, shutil
import numpy as np
import pyqtgraph as pg
from PyQt4 import QtGui
from PyQt4.QtCore import Qt, QTime
from pyqtgraph.dockarea import *
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import re



ctime=datetime.utcnow()

class CAxisTime(pg.AxisItem):   #sets the x axis as current UTC time. I DON'T KNOW HOW IT WORKS IT'S MAGIC SO DON'T TOUCH IT
    def tickStrings(self, values, scale, spacing):
        return [(ctime+timedelta(hours=int(float(QTime().addSecs(value).toString('hh'))),minutes=int(float(QTime().addSecs(value).toString('mm'))),seconds=int(float(QTime().addSecs(value).toString('ss'))))).strftime("%H:%M:%S") for value in values]


class Window(QtGui.QMainWindow):
    '''This is the main window class, or the border if you will. This class controls the name of the window and the x buttons and such. All the graphics layout widgets and such are put inside this window.'''
    def __init__(self, *args, **kwargs):
        super(Window,self).__init__(*args, **kwargs)
            
        self.setWindowState(Qt.WindowMaximized)
        self.setWindowTitle("MM DAQ & Visualization")  
             
    def closeEvent(self,event): #ignores the action of pressing the red x in the corner and opens a dialog bax asking if you are sure you want to close. 
        choice = QtGui.QMessageBox.question(self, 'Close?',
                                                "Are you sure you want to close? Data from Mobile Mesonet will stop being logged.",
                                                QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        #if the option is yes then the it will terminate all of the threads cleanly and move all of the data to the backup log files on drive E, then close the GUI effectively terminating the program                                        
        if choice == QtGui.QMessageBox.Yes:
            presspro.terminate()
            GPSpro.terminate()
            THVpro.terminate()
            Fluxpro.terminate()
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'MobileMesonetlog_'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6],'MobileMesonetlog_'+starttime+'.txt'))
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'Fluxraw'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6],'Fluxraw'+starttime+'.txt'))
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'GPSraw'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6],'GPSraw'+starttime+'.txt'))
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'Pressureraw'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6],'Pressureraw'+starttime+'.txt'))
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'THVraw'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6], 'THVraw'+starttime+'.txt'))
            sys.exit(0)
        else:
            event.ignore()
            
            
class ThermoWidget(pg.GraphicsLayoutWidget):
    '''This class is a layout widget for the thermodynamic variables in tab one of the MMpy GUI. It embeds plots in a certain pattern or location in the window. In this case it is a 3x2 grid of plots. This layout is then inserted into a dock in order for the tabs to work'''
    def __init__(self,*args, **kwargs):
        super(ThermoWidget, self).__init__(*args, **kwargs)
        

        self.p1 = self.addPlot(axisItems = {'bottom': CAxisTime(orientation='bottom')})
        self.p2 = self.addPlot(axisItems = {'bottom': CAxisTime(orientation='bottom')})
        # Use automatic downsampling and clipping to reduce the drawing load
        self.p1.setDownsampling(mode='peak')
        self.p2.setDownsampling(mode='peak')
        self.p1.setClipToView(True)
        self.p2.setClipToView(True)
        self.p1.setTitle('Temperature [\xb0F]')
        self.p2.setTitle('Dewpoint [\xb0F]')
        self.p1.showGrid(True,True)
        self.p2.showGrid(True,True)
        self.curve1 = self.p1.plot()    #calls the plot function to actually plot the data
        self.curve2 = self.p2.plot()    #calls the plot function to actually plot the data
    
        self.nextRow()  #starts a new row to put 2 new plots
        self.p3 = self.addPlot(axisItems = {'bottom': CAxisTime(orientation='bottom')})
        self.p4 = self.addPlot(axisItems = {'bottom': CAxisTime(orientation='bottom')})
        # Use automatic downsampling and clipping to reduce the drawing load
        self.p3.setDownsampling(mode='peak')
        self.p4.setDownsampling(mode='peak')
        self.p3.setClipToView(True)
        self.p4.setClipToView(True)
        self.p3.setTitle('Theta-V [K]')
        self.p4.setTitle('Theta-E [K]')
        self.p3.showGrid(True,True)
        self.p4.showGrid(True,True)
        self.curve3 = self.p3.plot()    #calls the plot function to actually plot the data
        self.curve4 = self.p4.plot()    #calls the plot function to actually plot the data
        
        self.nextRow()  #starts a new row to put 2 new plots
        self.p5 = self.addPlot(axisItems = {'bottom': CAxisTime(orientation='bottom')})
        self.p6 = self.addPlot(axisItems = {'bottom': CAxisTime(orientation='bottom')})
        # Use automatic downsampling and clipping to reduce the drawing load
        self.p5.setDownsampling(mode='peak')
        self.p6.setDownsampling(mode='peak')
        self.p5.setClipToView(True)
        self.p6.setClipToView(True)
        self.p5.setTitle('Relative Humidity [%]')
        self.p6.setTitle('Pressure [hPa]')
        self.p5.showGrid(True,True)
        self.p6.showGrid(True,True)
        self.curve5 = self.p5.plot()    #calls the plot function to actually plot the data
        self.curve6 = self.p6.plot()    #calls the plot function to actually plot the data
            
            
    def plot1(self, data):
        self.curve1.setData(data)
        pg.QtGui.QApplication.processEvents()
            
    def plot2(self, data):
        self.curve2.setData(data)
        pg.QtGui.QApplication.processEvents()
        
    def plot3(self, data):
        self.curve3.setData(data)
        pg.QtGui.QApplication.processEvents()
        
    def plot4(self, data):
        self.curve4.setData(data)
        pg.QtGui.QApplication.processEvents()
            
    def plot5(self, data):
        self.curve5.setData(data)
        pg.QtGui.QApplication.processEvents()
            
    def plot6(self, data):
        self.curve6.setData(data)
        pg.QtGui.QApplication.processEvents()
            

#==========================================================================================
class KinemWidget(pg.GraphicsLayoutWidget):
    '''This class is a layout widget for the kinematic variables in tab two of the MMpy GUI. It plots the wind speed and wind direction on the top row and the vehicle direction and the wind direction as a compass on the bottom row.'''
    def __init__(self, background = 'w', *args, **kwargs):
        super(KinemWidget, self).__init__(*args, **kwargs)

        self.p7 = self.addPlot(colspan = 2, axisItems = {'bottom': CAxisTime(orientation='bottom')})
        self.p8 = self.addPlot(colspan = 2, axisItems = {'bottom': CAxisTime(orientation='bottom')})
        self.p7.setMinimumHeight(300)
        self.p8.setMinimumHeight(300)
        # Use automatic downsampling and clipping to reduce the drawing load
        self.p7.setDownsampling(mode='peak')
        self.p8.setDownsampling(mode='peak')
        self.p7.setClipToView(True)
        self.p8.setClipToView(True)
        self.p7.setTitle('Wind Speed [m/s]')
        self.p8.setTitle('Wind Direction [\xb0]')
        self.p7.showGrid(True,True)
        self.p8.showGrid(True,True)
        self.curve7 = self.p7.plot()    #calls the plot function to actually plot the data
        self.curve8 = self.p8.plot()    #calls the plot function to actually plot the data
    
        self.nextRow()  #starts a new row to put 2 new plots or in this case widgets

        self.addLabel('Vehicle Direction', size = '30pt', bold= 'true', angle = -90)

        self.compass1 = CompassWidget()
        self.compass2 = CompassWidget()
        pal = QPalette()
        pal.setColor(QPalette.Window, Qt.black) #sets the palette black which in turn eventually turns the background black

        self.compass1.setPalette(pal)
        self.compass2.setPalette(pal)

        self.proxycompass1 = QGraphicsProxyWidget()   #Needed to use a proxy widget in order for it to work in the graphics layout widget     
        self.proxycompass2 = QGraphicsProxyWidget()  
        
        self.proxycompass1.setWidget(self.compass1)
        self.proxycompass2.setWidget(self.compass2)
        
        
        self.addItem(self.proxycompass1)    #actually add the compass to the layout
        self.addItem(self.proxycompass2)
        
        self.addLabel('Wind Direction', size = '30pt', bold= 'true', angle = 90)
        
        
    def plot7(self, data):
        self.curve7.setData(data)
        pg.QtGui.QApplication.processEvents()
            
    def plot8(self, data):
        self.curve8.setData(data)
        pg.QtGui.QApplication.processEvents()
        
#==========================================================================================
#This widget was found online, I have not customized it too much other than modifying the pointer and colors to fit with the background.
class CompassWidget(QWidget):
    '''CLass to plot a compass'''
    angleChanged = pyqtSignal(float)

    def __init__(self, parent = None):

        QWidget.__init__(self, parent)
        self._angle = 0.0
        self._margins = 10
        self._pointText = {0: "N", 45: "NE", 90: "E", 135: "SE", 180: "S",
                           225: "SW", 270: "W", 315: "NW"}               

    def paintEvent(self, event):

        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(event.rect(), self.palette().brush(QPalette.Window))
        self.drawMarkings(painter)
        self.drawNeedle(painter)
        painter.end()

    def drawMarkings(self, painter):

        painter.save()
        painter.translate(self.width()/2, self.height()/2)
        scale = min((self.width() - self._margins)/120.0,
                    (self.height() - self._margins)/120.0)
        painter.scale(scale, scale)
        font = QFont(self.font())
        font.setPixelSize(10)
        metrics = QFontMetricsF(font)
        painter.setFont(font)
        painter.setPen(self.palette().color(QPalette.Shadow))
        
        i = 0
        while i < 360:
            if i % 45 == 0:
                painter.drawLine(0, -40, 0, -50)
                painter.drawText(-metrics.width(self._pointText[i])/2.0, -52,self._pointText[i])
            else:
                painter.drawLine(0, -45, 0, -50)
            painter.rotate(1)
            i += 1
        painter.restore()

    def drawNeedle(self, painter):

        painter.save()
        painter.translate(self.width()/2, self.height()/2)
        painter.rotate(self._angle)
        scale = min((self.width() - self._margins)/120.0,
                    (self.height() - self._margins)/120.0)
        painter.scale(scale, scale)
        painter.setPen(QPen(Qt.NoPen))
        painter.setBrush(self.palette().brush(QPalette.Shadow))

        painter.drawPolygon(
            QPolygon([QPoint(-10, 0), QPoint(0, -45), QPoint(10, 0),
                      QPoint(0, 45), QPoint(-10, 0)])
            )
        painter.setBrush(self.palette().brush(QPalette.Highlight))

        painter.drawPolygon(
            QPolygon([QPoint(-5, -25), QPoint(0, -45), QPoint(5, -25),
                      QPoint(0, -30), QPoint(-5, -25)])
            )
        painter.restore()

    def sizeHint(self):
        return QSize(600, 600)

    def angle(self):
        return self._angle

    @pyqtSlot(float)
    def setAngle(self, angle):

        if angle != self._angle:
            self._angle = angle
            self.angleChanged.emit(angle)
            self.update()

    angle = pyqtProperty(float, angle, setAngle)     
    
#=====================================================================================================================
    
def PressureProcess(Presspro):
    '''Function that opens the serial port for the Pressure, writes the raw data to a log file, processes the raw data into a coherent string, and adds that string into a queue. This function is ran through multiprocessing and uses the queue to transport information to the main program.'''
    ser4 = serial.Serial(port='COM6',baudrate=9600) #opens serial port 
    sio4 = io.TextIOWrapper(            #not sure if this section is needed, it isn't for the fluxgate. However the examples have it so it will stay for now
        io.BufferedRWPair(ser4, ser4, 1),
        newline='\r'
    )
    starttime=strftime("%Y%m%d_%H%M", gmtime())     #gets current time (when it started) to append to the file name
    rawPressure = open(starttime+'_raw_pres.txt', 'a')  #opens a file for the raw data to be logged
    
    #write the header
    try:
        rawPressure.write('Pressure [hPa], Epoch Time \n')
    finally:
        rawPressure.close()
        
    while 1:
        Pressure = ser4.readline()  #get data through the serial port
        P_str = str(Pressure)   #Convert it to a string in order to be logged
        Ptime=str(time.time())  #get the current computer time the data came in at
        rawPressure = open(starttime+'_raw_pres.txt', 'a')  #open the file to log raw data in to
        #write the raw data to the file
        try:
            rawPressure.write(P_str[:-2]+','+Ptime+'\n')
        finally:
            rawPressure.close()   
            
        Pressureout=P_str[:-2]+','+Ptime+'\n'     #takes all of the raw data and creates a combined logout string
        Presspro.put(Pressureout)       #adds the string to the queue
#=====================================================================================================================
        
def GPSProcess(GPSpro):
    '''Function that opens the serial port for the GPS, writes the raw data to a log file, processes the raw data into a coherent string, and adds that string into a queue. This function is ran through multiprocessing and uses the queue to transport information to the main program.'''
    ser3 = serial.Serial(port='COM5',baudrate=9600)     #opens serial port
    sio3 = io.TextIOWrapper(        #not sure if this section is needed, it isn't for the fluxgate. However the examples have it so it will stay for now
        io.BufferedRWPair(ser3, ser3, 1),
        newline='\r'
    )
    starttime=strftime("%Y%m%d_%H%M", gmtime())  #gets current time (when it started) to append to the file name
    
    while 1:
        GPS = ser3.readline()   #get data through the serial port
        GPS_str = str(GPS)      #Convert it to a string in order to be logged
        rawGPS = open(starttime+'_raw_gps.txt', 'a')    #opens a file for the raw data to be logged
        chasercode = open('C:\users\imet1\Documents\Chaser_Code\GPS_chaser.txt','wb')   #chasercode lines output lat lon to separate file; added by Alex Krull

        try:
            #If the first section of the GPS is the code GPGGA, as in it ends with an 'A', then get the current computer time, the # of GPS satellites and the altitude of the GPS
            if (GPS_str[5] == 'A'): #GPGGA
                compGPStime=str(time.time())
                GPS_s=GPS_str.split(',')
                numGPS=GPS_s[7]
                altGPS=GPS_s[9]
            #If the first section of the GPS is the code GPRMC, as in it ends with an 'C', then get the current computer time, the GPS date, GPS time, GPS lat, GPS lon, GPS speed, and the magnetic variance of the GPS
            if (GPS_str[5] == 'C'): #GPRMC
                compGPStime=str(time.time())
                GPS_s=GPS_str.split(',')
                dateGPS=GPS_s[9]
                timeGPS=GPS_s[1]
                latGPS=GPS_s[3]
                lonGPS=GPS_s[5]
                speedGPS=GPS_s[7]
                magvarGPS=GPS_s[10]
            #If the first section of the GPS is the code PGRME, as in it ends with an 'E', then get the current computer time, the horizontal position error, the vertical position error, and spherical position error
            if (GPS_str[5] == 'E'): #PGRME
                compGPStime=str(time.time())
                GPS_s=GPS_str.split(',')
                HPEGPS=GPS_s[1]
                VPSGPS=GPS_s[3]
                SEPGPS=GPS_s[5]
        
                #all three of these sections arrive in the duration of a second, but do not happen concurrently, this was the reason for splitting the serial data into multiprocesses
                #once the PGRME data is in, all of the data strings are combined into one string and then it is put in the queue
                try:
                    GPSout=dateGPS+','+timeGPS+','+compGPStime+','+latGPS+','+lonGPS+','+altGPS+','+speedGPS+','+numGPS+','+magvarGPS+','+HPEGPS+','+VPSGPS+','+SEPGPS+'\n'
                except:
                    pass
                else:
                    GPSpro.put(GPSout)
                
        except IndexError:
            pass
        try:
            rawGPS.write('{0:}'.format(GPS_str[:-2]+','+compGPStime+'\n'))  #writes the GPS data to the raw log file
            chasercode.write('%s,%s' % (latGPS,lonGPS)) 
        except(NameError):
            print "GPS NameError"
            compGPStime='still initializing'   #depending on the time the computer starts listening to the GPS, the data maybe be in halfway. Therefore it puts 'still initializing' until a whole cycle is done and the computer can read the data cleanly
            rawGPS.write('{0:}'.format(GPS_str[:-2]+','+compGPStime+'\n'))  #writes the what data it can to the raw log file
            chasercode.write('%s,%s' % (latGPS,lonGPS)) 
        finally:
            rawGPS.close()
            chasercode.close()
#=====================================================================================================================
            
def THVProcess(THVpro):
    '''Function that opens the serial port for the logger that reads THV data, writes the raw data to a log file, processes the raw data into a coherent string, and adds that string into a queue. This function is ran through multiprocessing and uses the queue to transport information to the main program.'''
    ser1 = serial.Serial(port='COM3',baudrate=9600)   #opens serial port
    sio1 = io.TextIOWrapper(    #not sure if this section is needed, it isn't for the fluxgate. However the examples have it so it will stay for now
        io.BufferedRWPair(ser1, ser1, 1),
        newline='\r'
    )
    starttime=strftime("%Y%m%d_%H%M", gmtime())   #gets current time (when it started) to append to the file name
    rawTHV = open(starttime+'_raw_THV.txt', 'a')    #opens a file for the raw data to be logged
    
    #write the header
    try:
        rawTHV.write('01+0401 02+HHMM 03+SS.S 04+(blank) 05+UT_fast 06+UT_slow 07+UT_RH 08+RMYAsp_fast 09+AirSpd 10+WndDir 11+solar 12+(blank) 13+boxTemp 14+RefVolts 15+BattVolts 16+(blank) 17+(blank) 18+(blank) 19+(blank) 20+EpochTime \n')
    finally:
        rawTHV.close()
        
    while 1:
        THV = ser1.readline()   #get data through the serial port
        THV_str = str(THV)      #Convert it to a string in order to be logged
        THVtime=str(time.time())    #get the current computer time the data came in at
        rawTHV = open(starttime+'_raw_THV.txt', 'a')    #open the file to log raw data in to
        #write the raw data to the file
        try:
            rawTHV.write(THV_str[:-2]+'  20+'+THVtime+'\n')
        finally:
            rawTHV.close()  
            
        THVout=THV_str[:-2]+' '+'  20+'+THVtime+'\n'    #takes all of the raw data and creates a combined logout string
        THVpro.put(THVout)  #adds the string to the queue
#=====================================================================================================================
        
def FluxProcess(Fluxpro):
    '''Function that opens the serial port for the fluxgate, writes the raw data to a log file, processes the raw data into a coherent string, and adds that string into a queue. This function is ran through multiprocessing and uses the queue to transport information to the main program.'''
    ser2 = serial.Serial(port='COM4',baudrate=9600, timeout=0.5)  #opens serial port
    starttime=strftime("%Y%m%d_%H%M", gmtime()) #gets current time (when it started) to append to the file name
    rawFlux = open(starttime+'_raw_fg.txt', 'a') #opens a file for the raw data to be logged
    
    #write the header
    try:
        rawFlux.write('Fluxgate, Epoch Time \n')
    finally:
        rawFlux.close()
        
    while 1:
        Flux = ser2.readline()  #get data through the serial port
        Flux_str = str(Flux)    #Convert it to a string in order to be logged
        Fluxtime=str(time.time())   #get the current computer time the data came in at
        rawFlux = open(starttime+'_raw_fg.txt', 'a')    #open the file to log raw data in to
        #write the raw data to the file
        try:
            rawFlux.write(Flux_str[1:6]+','+Fluxtime+'\n')
        finally:
            rawFlux.close()  

        Fluxout=Flux_str[1:6]+','+Fluxtime+'\n'  #takes all of the raw data and creates a combined logout string
        Fluxpro.put(Fluxout)    #adds the string to the queue
#=====================================================================================================================
        
def dewcalc(RH, temps):
    ''' This function calculates the dewpoint temperature in celsius given the relative humidity in % and the slow temperature sensor in celsius'''
    gma = np.log(0.01*RH*np.exp((18.678-temps/234.5)*(temps/(257.14+temps))))
    dewpoint = 257.14*gma/(18.678-gma)
    return dewpoint

def RHcorr(Td, tempf):
    '''This function corrects the Relative Humidity due to the response time of the temperature sensors given the dewpoint temperature in celsius and the fast temperature in celsius'''
    RH = 100*(np.exp((17.625*Td)/(243.04+Td))/np.exp((17.625*tempf)/(243.04+tempf)))
    return RH

def thetacalc(tempf, P):
    '''This function calculates the potential temperature in Kelvin given the fast temperature in celsius and the Pressure in hPa'''
    theta = (tempf+273.15)*np.power((100000/(P*100)),0.286)
    return theta
  
def wvmixratcalc(tempf, RH, P):
    '''This function calculates the water vapor mixing ratio in g/kg given the fast temperature in celsius, the relative humidity in %, and the pressure in hPa'''
    es = 6.112*np.exp((17.67*tempf)/(tempf+243.5))
    e = 0.01*es*RH
    qv = 100*0.622*e/(P*100)
    return qv*1000
  
def thetaecalc(RH, dew, temps, tempf, P):
    '''This function calculates the equivalent potential temperature in Kelvin given the relative humidity in %, the dewpoint temperature in celsius, the slow temperature in celsius, the fast temperature in celsius, and the pressure in hPa'''
    theta = thetacalc(tempf, P)
    es = 6.112*np.exp((17.67*tempf)/(tempf+243.5))
    e = 0.01*es*RH
    qv = 100*0.622*e/(P*100)
    tlcl = 55.0+(2840.0/(3.5*np.log(tempf+273.15)-np.log(e)-4.805))
    tm = theta*np.power(((tempf+273.15)/theta),(0.286*qv))
    thetae = tm*np.exp(((3376.0/tlcl)-2.54)*qv*(1.0+0.81*qv))
    return thetae 

def thetavcalc(tempf, qv, P):
    '''This function calculates the virtual potential temperature in Kelvin, given the fast temperature in celsius, the water vapor mixing ratio in g/kg, and the pressure in hPa'''
    theta=thetacalc(tempf, P)
    thetav=(1+0.61*qv)*theta
    return thetav
    
def windspdcalc(anemspd, anemdir, vehspd, vehdir):
    '''This function calculates the wind speed in m/s given the anemometer wind speed in m/s, anemometer direction in degrees, vehicle speed in m/s and vehicle direction in degrees'''
    Uvr = -anemspd * np.sin(np.deg2rad(anemdir-142.5))
    Vvr = vehspd - anemspd * np.cos(np.deg2rad(anemdir-142.5))
    
    windspd = np.sqrt(np.square(Uvr)+ np.square(Vvr))
    return windspd
    
def winddircalc(anemspd, anemdir, vehspd, vehdir):
    '''This function calculates the wind direction in degrees given the anemometer wind speed in m/s, anemometer direction in degrees, vehicle speed in m/s and vehicle direction in degrees'''
    Uvr = -anemspd * np.sin(np.deg2rad(anemdir-142.5))  #142.5 is a calibration angle. The anemdir=142.5 when it was lined up with the car, the vehdir is then added onto that
    Vvr = vehspd - anemspd * np.cos(np.deg2rad(anemdir-142.5))
    if Uvr == 0 and Vvr < 0: 
        DvrD = 0
    if Uvr == 0 and Vvr > 0:
        DvrD = 180
    if Vvr == 0 and Uvr < 0:
        DvrD=90
    if Vvr == 0 and Uvr > 0:
        DvrD=270
    if Vvr == 0 and Uvr == 0:
        DvrD=0
    if Vvr != 0 and Uvr != 0:
        DvrD=np.rad2deg(np.arctan(Uvr/Vvr))
    if Vvr >0:
        DvrD += 180
    if Uvr > 0 and Vvr < 0:
        DvrD += 360
        
    winddir = DvrD+vehdir  
    if winddir > 360:
        winddir -= 360
    return winddir 
     
#======================================================================================================================        
if __name__ == '__main__':  #likely a crappy way to call the "main" class/thread, but hey, it works

    app = QtGui.QApplication(sys.argv) #creates an instance of a GUI application
    GUI=Window() #calls the window class and names it GUI
    area=DockArea() #calls the dockarea class and names it area
    GUI.setCentralWidget(area) #adds the dock area as the central widget in the window
    
    #create an instance of these classes
    thermodynamic=ThermoWidget()
    kinematic=KinemWidget()
    compass=CompassWidget()
    
    #it was impossible to xlink plots across classes as far as I know, but this seems to work, linking all of the x-axis' together
    #There is one small bug with this however, if it is desired that the graph dynamically fit on the plot, when switching tabs it will not stay. However it is simple enough to hit 
    #the A at the bottom left of the plot to switch back to dynamic fitting
    thermodynamic.p1.setXLink(thermodynamic.p2)
    thermodynamic.p2.setXLink(thermodynamic.p3)
    thermodynamic.p3.setXLink(thermodynamic.p4)
    thermodynamic.p4.setXLink(thermodynamic.p5)
    thermodynamic.p5.setXLink(thermodynamic.p6)
    thermodynamic.p6.setXLink(kinematic.p7)
    kinematic.p7.setXLink(kinematic.p8)
    
    #added docking, one thermo one kinematic for easier viewing of similar variables 
    d1=Dock("Thermodynamics")
    d2=Dock("Kinematics")
    area.addDock(d2, 'left') #adds the kinematics dock first
    area.addDock(d1, 'above', d2) #then adds the thermo dock on top of the kinematics dock so the appear side by side in tab form. This is the only way to get the right tabs using docking
    
    #adds the pg.GraphicsLayoutWidget to the dock    
    d1.addWidget(thermodynamic)
    d2.addWidget(kinematic)
    
    #show the gui, otherwise it is initialized but never shown
    GUI.show()
    
#*******************************************************
    starttime=strftime("%Y%m%d_%H%M", gmtime())#gets the time the program started
    if not os.path.exists(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6])):    #Checking to see if directory exists, if not it will make the directory
        os.makedirs(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6]))
    os.chdir(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6])) #actually go to the directory
    loggerfile = open(starttime+'_IMeT.txt', 'a') #create log file in the directory
    
    #print header for the logfile then close the log file
    try:
        loggerfile.write("# Integrated Mesonet and Tracker (IMeT-1) synchronized data file")
        loggerfile.write("# Data logger script version")
        loggerfile.write("#   3.0")
        loggerfile.write("# Instrumentation")
        loggerfile.write("#   Heading: KVH Industries C-100 fluxgate compass")
        loggerfile.write("#   RH and slow temperature: Vaisala HMP155A-L-PT")
        loggerfile.write("#   Fast temperature: Campbell Scientific 10922-L Thermistor")
        loggerfile.write("#   Pressure: Vaisala PTB210 Barometer")
        loggerfile.write("#   Wind velocity: RM Young Wind Monitor - 05103-L-PT")
        loggerfile.write("# Missing data value")
        loggerfile.write("#   -999.00")
        loggerfile.write("# Data key")
        loggerfile.write("#   Date, Time (UTC HHMMSS), Latitude, Longitude, Altitude (Meters), Pressure (hPa), Temperature fast (Celsius), Temperature slow (Celsius), Logger RH, Calculated Corrected RH, Calculated Dewpoint (Celsius), Calculated Mixing Ratio (g/kg), Calculated Theta (Kelvin), Calculated Theta-V (Kelvin), Calculated Theta-E (Kelvin), Anemometer Speed (m/s), Anemometer Direction, Calculated Wind Speed (m/s), Calculated Wind Direction, Fluxgate Heading, Vehicle Speed (m/s), Number of Satellites, GPS Magnetic Variation, Estimated Horizontal Position Error (meters), Estimated Vertical Position Error (meters), Overall Spherical Equivalent Position Error, GPS Computer Time (epoch), Pressure Computer Time (epoch), THV Computer Time (epoch), Fluxgate Computer Time (epoch) \n")
    finally:
        loggerfile.close()
  #=====================================================================================================================
    #begin the Pressure process
    pressq=multiprocessing.Queue()  #opens the pressure queue
    presspro = Process(target=PressureProcess, args=(pressq,)) #inits the process
    presspro.start()    #starts the process
    
    #create the Pressure arrays
    pressure=np.empty(10)
    pressuretime=np.empty(10)
    
#--------------------------------------------------------------------
    #begin the GPS process
    GPSq=multiprocessing.Queue()
    GPSpro = Process(target=GPSProcess, args=(GPSq,))
    GPSpro.start()
    
    #create the GPS arrays
    GPSdate=np.empty(10)
    GPStime=np.empty(10)       
    GPScomptime=np.empty(10)
    GPSlat=np.empty(10) 
    GPSlon=np.empty(10) 
    GPSalt=np.empty(10) 
    vehspd=np.empty(10)
    GPSnum=np.empty(10) 
    GPSmagvar=np.empty(10) 
    GPSHPE=np.empty(10) 
    GPSVPS=np.empty(10) 
    GPSSPE=np.empty(10) 

#--------------------------------------------------------------------
    #begin the THV process
    THVq=multiprocessing.Queue()
    THVpro = Process(target=THVProcess, args=(THVq,))
    THVpro.start()
    
    #create the THV arrays
    THVtime=np.empty(10)
    fasttemp=np.empty(10)
    slowtemp=np.empty(10)
    uncorRH=np.empty(10)
    corRH=np.empty(10)
    anemspd=np.empty(10)
    anemdir=np.empty(10)
    dew=np.empty(10)
    mixrat=np.empty(10)
    theta=np.empty(10)
    thetav=np.empty(10)
    thetae=np.empty(10)
    
#--------------------------------------------------------------------
    #begin the flux process
    Fluxq=multiprocessing.Queue()
    Fluxpro = Process(target=FluxProcess, args=(Fluxq,))
    Fluxpro.start()
    
    #create the flux arrays
    Flux=np.empty(10)
    Fluxtime=np.empty(10)
    windspd=np.empty(10)
    winddir=np.empty(10)
    
#--------------------------------------------------------------------
    cf=-1 #fluxgate counter
    cp=-1 #pressure counter
    cg=-1 #gps counter
    ct=-1 #THV counter
    precg=0 #previous gps counter
    c=0 #global counter
    while 1: 
        if c > 0 and c % 1800 == 0: # every half hour, backup data to external drive
            if not os.path.exists(os.path.join("E:\data", starttime[0:6])):#creates directory if it doesn't exist
                os.makedirs(os.path.join("E:\data", starttime[0:6]))
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'MobileMesonetlog_'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6],'MobileMesonetlog_'+starttime+'.txt'))
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'Fluxraw'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6],'Fluxraw'+starttime+'.txt'))
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'GPSraw'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6],'GPSraw'+starttime+'.txt'))
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'Pressureraw'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6],'Pressureraw'+starttime+'.txt'))
            shutil.copy(os.path.join("C:\Users\imet1\Desktop\data", starttime[0:6],'THVraw'+starttime+'.txt'), os.path.join("E:\data", starttime[0:6], 'THVraw'+starttime+'.txt'))
            
       
        if cp+1 >= pressure.shape[0]: #pyqtgraph legacy code for doubling size of array while transferring the data
            pressuretmp = pressure
            pressure = np.empty(pressure.shape[0] * 2)
            pressure[:len(pressuretmp)] = pressuretmp
        if cp+1 >= pressuretime.shape[0]:
            pressuretimetmp = pressuretime
            pressuretime = np.empty(pressuretime.shape[0] * 2)
            pressuretime[:len(pressuretimetmp)] = pressuretimetmp
        #timeout for getting data from queue, if nothing is in the queue it will timeout and then pass which just continues the program    
        try:
            pressure_str=pressq.get(timeout=0.01)

        except Queue.Empty:
            pass
        else: #if the queue isn't empty, iterate the counter and add data to array
            cp+=1
            pressure[cp]=str(pressure_str)[0:6]
            pressuretime[cp]=str(pressure_str)[8:20]
            
#---------------------------------------------------------------------------- 
        #pyqtgraph legacy code for doubling size of array while transferring the data
        if cg+1 >= GPSdate.shape[0]: 
            GPSdatetmp = GPSdate
            GPSdate = np.empty(GPSdate.shape[0] * 2)
            GPSdate[:len(GPSdatetmp)] = GPSdatetmp
        if cg+1 >= GPStime.shape[0]:
            GPStimetmp = GPStime
            GPStime = np.empty(GPStime.shape[0] * 2)
            GPStime[:len(GPStimetmp)] = GPStimetmp
        if cg+1 >= GPScomptime.shape[0]:
            GPScomptimetmp = GPScomptime
            GPScomptime = np.empty(GPScomptime.shape[0] * 2)
            GPScomptime[:len(GPScomptimetmp)] = GPScomptimetmp
        if cg+1 >= GPSlat.shape[0]:
            GPSlattmp = GPSlat
            GPSlat = np.empty(GPSlat.shape[0] * 2)
            GPSlat[:len(GPSlattmp)] = GPSlattmp
        if cg+1 >= GPSlon.shape[0]:
            GPSlontmp = GPSlon
            GPSlon = np.empty(GPSlon.shape[0] * 2)
            GPSlon[:len(GPSlontmp)] = GPSlontmp
        if cg+1 >= GPSalt.shape[0]:
            GPSalttmp = GPSalt
            GPSalt = np.empty(GPSalt.shape[0] * 2)
            GPSalt[:len(GPSalttmp)] = GPSalttmp
        if cg+1 >= vehspd.shape[0]:
            vehspdtmp = vehspd
            vehspd = np.empty(vehspd.shape[0] * 2)
            vehspd[:len(vehspdtmp)] = vehspdtmp
        if cg+1 >= GPSnum.shape[0]:
            GPSnumtmp = GPSnum
            GPSnum = np.empty(GPSnum.shape[0] * 2)
            GPSnum[:len(GPSnumtmp)] = GPSnumtmp
        if cg+1 >= GPSmagvar.shape[0]:
            GPSmagvartmp = GPSmagvar
            GPSmagvar = np.empty(GPSmagvar.shape[0] * 2)
            GPSmagvar[:len(GPSmagvartmp)] = GPSmagvartmp
        if cg+1 >= GPSHPE.shape[0]:
            GPSHPEtmp = GPSHPE
            GPSHPE = np.empty(GPSHPE.shape[0] * 2)
            GPSHPE[:len(GPSHPEtmp)] = GPSHPEtmp
        if cg+1 >= GPSVPS.shape[0]:
            GPSVPStmp = GPSVPS
            GPSVPS = np.empty(GPSVPS.shape[0] * 2)
            GPSVPS[:len(GPSVPStmp)] = GPSVPStmp
        if cg+1 >= GPSSPE.shape[0]:
            GPSSPEtmp = GPSSPE
            GPSSPE = np.empty(GPSSPE.shape[0] * 2)
            GPSSPE[:len(GPSSPEtmp)] = GPSSPEtmp
        
        #timeout for getting data from queue, if nothing is in the queue it will timeout and then pass which just continues the program
        try:
            GPS_str=GPSq.get(timeout=0.01)
        except Queue.Empty:
            pass          
        else:  #if the queue isn't empty, iterate the counter and add data to array
            try:
                cg+=1
                GPS_str=GPS_str.split(",")
                GPSdate[cg]=GPS_str[0]
                GPStime[cg]=GPS_str[1]
                GPScomptime[cg]=GPS_str[2]
                GPSlat[cg]=GPS_str[3]
                GPSlon[cg]=GPS_str[4]
                GPSalt[cg]=GPS_str[5]
                vehspd[cg]=float(GPS_str[6])*0.5144444
                GPSnum[cg]=GPS_str[7]
                GPSmagvar[cg]=GPS_str[8]
                GPSHPE[cg]=GPS_str[9]
                GPSVPS[cg]=GPS_str[10]
                GPSSPE[cg]=GPS_str[11]   
            except ValueError:
                print "GPS ValueError"
                cg-=1
                pass
#----------------------------------------------------------------------------- 
        #pyqtgraph legacy code for doubling size of array while transferring the data
        if ct+1 >= THVtime.shape[0]: 
            THVtimetmp = THVtime
            THVtime = np.empty(THVtime.shape[0] * 2)
            THVtime[:len(THVtimetmp)] = THVtimetmp
        if ct+1 >= fasttemp.shape[0]:
            fasttemptmp = fasttemp
            fasttemp = np.empty(fasttemp.shape[0] * 2)
            fasttemp[:len(fasttemptmp)] = fasttemptmp
        if ct+1 >= slowtemp.shape[0]:
            slowtemptmp = slowtemp
            slowtemp = np.empty(slowtemp.shape[0] * 2)
            slowtemp[:len(slowtemptmp)] = slowtemptmp
        if ct+1 >= uncorRH.shape[0]:
            uncorRHtmp = uncorRH
            uncorRH = np.empty(uncorRH.shape[0] * 2)
            uncorRH[:len(uncorRHtmp)] = uncorRHtmp
        if ct+1 >= corRH.shape[0]:
            corRHtmp = corRH
            corRH = np.empty(corRH.shape[0] * 2)
            corRH[:len(corRHtmp)] = corRHtmp
        if ct+1 >= anemspd.shape[0]:
            anemspdtmp = anemspd
            anemspd = np.empty(anemspd.shape[0] * 2)
            anemspd[:len(anemspdtmp)] = anemspdtmp
        if ct+1 >= anemdir.shape[0]:
            anemdirtmp = anemdir
            anemdir = np.empty(anemdir.shape[0] * 2)
            anemdir[:len(anemdirtmp)] = anemdirtmp
        if ct+1 >= dew.shape[0]:
            dewtmp = dew
            dew = np.empty(dew.shape[0] * 2)
            dew[:len(dewtmp)] = dewtmp
        if ct+1 >= mixrat.shape[0]:
            mixrattmp = mixrat
            mixrat = np.empty(mixrat.shape[0] * 2)
            mixrat[:len(mixrattmp)] = mixrattmp
        if ct+1 >= theta.shape[0]:
            thetatmp = theta
            theta = np.empty(theta.shape[0] * 2)
            theta[:len(thetatmp)] = thetatmp
        if ct+1 >= thetav.shape[0]:
            thetavtmp = thetav
            thetav = np.empty(thetav.shape[0] * 2)
            thetav[:len(thetavtmp)] = thetavtmp
        if ct+1 >= thetae.shape[0]:
            thetaetmp = thetae
            thetae = np.empty(thetae.shape[0] * 2)
            thetae[:len(thetaetmp)] = thetaetmp
        
        try:
            THV_str=THVq.get(timeout=0.01)           
        except Queue.Empty:
            #plotting this data here just updates the graph with the same data so it is easy to manipulate with the mouse and not laggy
            thermodynamic.plot1(fasttemp[:c]*1.8+32)
            thermodynamic.plot2(dew[:c]*1.8+32)
            thermodynamic.plot3(thetav[:c])        
            thermodynamic.plot4(thetae[:c])    
            thermodynamic.plot5(corRH[:c])
            thermodynamic.plot6(pressure[:c])
            kinematic.plot7(windspd[:c])
            kinematic.plot8(winddir[:c])
            kinematic.compass1.setAngle(flux[:c])
            kinematic.compass2.setAngle(winddir[:c])
            pass
        else:
            try:
                ct+=1
                THV_str=re.split("\+|\-",THV_str)#negative to make it work in the winter
                fasttemp[ct]=str(THV_str[5])[0:6]
                slowtemp[ct]=str(THV_str[6])[0:6]
                uncorRH[ct]=str(THV_str[7])[0:6]
                corRH[ct]=RHcorr(dewcalc(uncorRH[ct], slowtemp[ct]), fasttemp[ct])
                anemspd[ct]=str(THV_str[9])[0:6]
                anemdir[ct]=str(THV_str[10])[0:6]
                dew[ct]=dewcalc(corRH[ct],fasttemp[ct])
                THVtime[ct]=str(THV_str[20])[0:14]
            except IndexError:
                print "THV IndexError"
                ct-=1
                pass
        #just for reference, this is the THV data organization straight out of the logger program
                
        #'01+0401 02+HHMM 03+SS.S 04+(blank) 05+UT_fast 06+UT_slow 07+UT_RH 08+RMYAsp_fast 
        #09+AirSpd 10+WndDir 11+solar 12+(blank) 13+boxTemp 14+RefVolts 15+BattVolts 16+(blank) 
        #17+(blank) 18+(blank) 19+(blank) 20+timeComputer
           
#----------------------------------------------------------------------------- 
        #pyqtgraph legacy code for doubling size of array while transferring the data
        if cf+1 >= Fluxtime.shape[0]:
            Fluxtimetmp = Fluxtime
            Fluxtime = np.empty(Fluxtime.shape[0] * 2)
            Fluxtime[:len(Fluxtimetmp)] = Fluxtimetmp
        if cf+1 >= Flux.shape[0]:
            Fluxtmp = Flux
            Flux = np.empty(Flux.shape[0] * 2)
            Flux[:len(Fluxtmp)] = Fluxtmp
        if cf+1 >= windspd.shape[0]:
            windspdtmp = windspd
            windspd = np.empty(windspd.shape[0] * 2)
            windspd[:len(windspdtmp)] = windspdtmp
        if cf+1 >= winddir.shape[0]:
            winddirtmp = winddir
            winddir = np.empty(winddir.shape[0] * 2)
            winddir[:len(winddirtmp)] = winddirtmp
        
        
        try:
            Flux_str=Fluxq.get(timeout=0.01)

        except Queue.Empty:
            pass            
        else:
            cf+=1
            Flux_str=Flux_str.split(",")
            Fluxtime[cf]=Flux_str[1]
            try:
                Flux[cf]=Flux_str[0]
            except ValueError:
                print "Flux ValueError", Fluxtime[cf]
                if cf > 1:
                    Flux[cf]=Flux[cf-1]
                else:
                    cf-=1
                    pass
        
##############################################################################################################################################        
        # if there is a new gps time, calculated the time differences in regards to the local counters, end goal is to find the smallest time difference and use that dataset
        if cg-precg == 1:
            
            #other data lags GPS data
            Pdiftime=abs(pressuretime[cp]-GPScomptime[cg-1])
            THVdiftime=abs(THVtime[ct]-GPScomptime[cg-1])
            Fluxdiftime=abs(Fluxtime[cf]-GPScomptime[cg-1])      
            
            #GPS and other data came in at same time
            Pdiftime1=abs(pressuretime[cp-1]-GPScomptime[cg-1])
            THVdiftime1=abs(THVtime[ct-1]-GPScomptime[cg-1])
            Fluxdiftime1=abs(Fluxtime[cf-1]-GPScomptime[cg-1])     
            
            #other data leads GPS data
            Pdiftime2=abs(pressuretime[cp-2]-GPScomptime[cg-1])
            THVdiftime2=abs(THVtime[ct-2]-GPScomptime[cg-1])
            Fluxdiftime2=abs(Fluxtime[cf-2]-GPScomptime[cg-1])     
        
            # This if statement is to make sure there are 3 values in the GPS time array so the data from the other sources can be matched with the closest
            if cg>3:
                logout=str(int(GPSdate[cg-1]))+','+str(int(GPStime[cg-1]))+','+str(GPSlat[cg-1])+','+str(GPSlon[cg-1])+','+str(GPSalt[cg-1])+','
#-----------------------------------------------------------------------------------------------------     
                #timesyncing pressure data
                if Pdiftime<Pdiftime1 and Pdiftime<Pdiftime2 and Pdiftime<1:
                    pindex=0
                    logout+=str(pressure[cp])+','
                elif Pdiftime1<Pdiftime and Pdiftime1<Pdiftime2 and Pdiftime1<1:
                    pindex=1
                    logout+=str(pressure[cp-1])+','
                elif Pdiftime2<Pdiftime and Pdiftime2<Pdiftime1 and Pdiftime2<1:
                    pindex=2
                    logout+=str(pressure[cp-2])+','
                elif Pdiftime1 == Pdiftime2 and Pdiftime1<1: 
                    pindex=1
                    logout+=str(pressure[cp-1])+','
                elif Pdiftime1 == Pdiftime and Pdiftime1<1: 
                    pindex=1
                    logout+=str(pressure[cp-1])+','
                else:
                    pindex=4
                    logout+="-999.00,"
#-----------------------------------------------------------------------------------------------------
                #timesyncing THV data
                if THVdiftime<THVdiftime1 and THVdiftime<THVdiftime2 and THVdiftime<1:
                    tindex=0
                    logout+=str(fasttemp[ct])+','+str(slowtemp[ct])+','+str(uncorRH[ct])+','+str(corRH[ct])+','+str(dew[ct])
                elif THVdiftime1<THVdiftime and THVdiftime1<THVdiftime2 and THVdiftime1<1:
                    tindex=1
                    logout+=str(fasttemp[ct-1])+','+str(slowtemp[ct-1])+','+str(uncorRH[ct-1])+','+str(corRH[ct-1])+','+str(dew[ct-1])
                elif THVdiftime2<THVdiftime and THVdiftime2<THVdiftime1 and THVdiftime2<1:
                    tindex=2
                    logout+=str(fasttemp[ct-2])+','+str(slowtemp[ct-2])+','+str(uncorRH[ct-2])+','+str(corRH[ct-2])+','+str(dew[ct-2])
                elif THVdiftime1 == THVdiftime2 and THVdiftime1<1: 
                    tindex=1
                    logout+=str(fasttemp[ct-1])+','+str(slowtemp[ct-1])+','+str(uncorRH[ct-1])+','+str(corRH[ct-1])+','+str(dew[ct-1])
                elif THVdiftime1 == THVdiftime and THVdiftime1<1: 
                    tindex=1
                    logout+=str(fasttemp[ct-1])+','+str(slowtemp[ct-1])+','+str(uncorRH[ct-1])+','+str(corRH[ct-1])+','+str(dew[ct-1])
                else:
                    tindex=4
                    logout+="-999.00"+','+"-999.00"+','+"-999.00"+','+"-999.00"+','+"-999.00"+','+"-999.00"+','+"-999.00"+','+"-999.00"+','+"-999.00"+','+"-999.00"+','+"-999.00"+','
#--------------------------------------------------------------------------------------------------------
                #calculating and mapping variables with their local counter to the global counter
                mixrat[c]=wvmixratcalc(fasttemp[ct-tindex], corRH[ct-tindex], pressure[cp-pindex])                               
                theta[c]=thetacalc(fasttemp[ct-tindex], pressure[cp-pindex])
                thetav[c]=thetavcalc(fasttemp[ct-tindex], mixrat[c]/1000, pressure[cp-pindex])
                thetae[c]=thetaecalc(corRH[ct-tindex], dew[ct-tindex], slowtemp[ct-tindex], fasttemp[ct-tindex], pressure[cp-pindex])
                
                #adds THV calculated vars to logout string with global counter
                if not tindex == 4:
                    logout+=','+str(mixrat[c])+','+str(theta[c])+','+str(thetav[c])+','+str(thetae[c])+','+str(anemspd[ct-tindex])+','+str(anemdir[ct-tindex])+','
                    
#--------------------------------------------------------------------------------------------------------
                #Timesyncinc fluxgate data
                if Fluxdiftime<Fluxdiftime1 and Fluxdiftime<Fluxdiftime2 and Fluxdiftime<1:
                    findex=0
                elif Fluxdiftime1<Fluxdiftime and Fluxdiftime1<Fluxdiftime2 and Fluxdiftime1<1:
                    findex=1
                elif Fluxdiftime2<Fluxdiftime and Fluxdiftime2<Fluxdiftime1 and Fluxdiftime2<1:
                    findex=2
                elif Fluxdiftime1 == Fluxdiftime2 and Fluxdiftime1<1: 
                    findex=1
                elif Fluxdiftime1 == Fluxdiftime and Fluxdiftime1<1: 
                    findex=1
                else:
                    findex=4
                    logout+="-999.00"+','+"-999.00"+','+"-999.00"+','
#--------------------------------------------------------------------------------------------------------
                    
                #calls the wind calculation function
                windspd[c]=windspdcalc(anemspd[ct-tindex], anemdir[ct-tindex], vehspd[cg-1], Flux[cf-findex])
                winddir[c]=winddircalc(anemspd[ct-tindex], anemdir[ct-tindex], vehspd[cg-1], Flux[cf-findex])
                
                #actually plots the data once it is all time synced
                thermodynamic.plot1(fasttemp[:c]*1.8+32)
                thermodynamic.plot2(dew[:c]*1.8+32)
                thermodynamic.plot3(thetav[:c])        
                thermodynamic.plot4(thetae[:c])    
                thermodynamic.plot5(corRH[:c])
                thermodynamic.plot6(pressure[:c])
                kinematic.plot7(windspd[:c])
                kinematic.plot8(winddir[:c])
                kinematic.compass1.setAngle(flux[:c])
                kinematic.compass2.setAngle(winddir[:c])
                
                #adds wind spd, wind dir, and flux to the logout string as long as it's time synced
                if not findex == 4:
                    logout+=str(windspd[c])+','+str(winddir[c])+','+str(Flux[cf-findex])+','
                
                #iterates global counter
                c+=1
                
#----------------------------------------------------------------------------------------------------- 
                
                #Adds GPS vars to logout string with local counter
                logout+=str(vehspd[cg-1])+','+str(GPSnum[cg-1])+','+str(GPSmagvar[cg-1])+','+str(GPSHPE[cg-1])+','+str(GPSVPS[cg-1])+','+str(GPSSPE[cg-1])+','
                #outputting time
                logout+=str(GPScomptime[cg-1])+','
                
#----------------------------------------------------------------------------------------------------- 
                
                #if it isn't time synced, it outputs -999.00
                if pindex == 4:
                    logout+='-999.00,'
                else:
                    logout+=str(pressuretime[cp-pindex])+','
                    
                if tindex == 4:
                    logout+='-999.00,'
                else:
                    logout+=str(THVtime[ct-tindex])+','
                    
                if findex == 4:
                    logout+='-999.00 \n'
                else:
                    logout+=str(Fluxtime[cf-findex])+'\n'

                #Opens the log file and appends the logout string then closes the log file
                loggerfile = open('MobileMesonetlog_'+starttime+'.txt', 'a')    
                try:
                    loggerfile.write(logout)
                finally:
                    loggerfile.close()
                    
        #Iterate local counter gps            
        precg=cg
        
    #I can't remember exactly why, but this is necessary for threading purposes, google is your friend
    presspro.active_children()
    GPSpro.active_children()
    THVpro.active_children()
    Fluxpro.active_children()
    sys.exit(app.exec_())