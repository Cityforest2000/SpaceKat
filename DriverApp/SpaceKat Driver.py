import threading, random, json, math, os, sys, time
import serial
import socket, select
import ctypes
import threading
from pynput.keyboard import Key,Controller
import pygetwindow


#clibrary=ctypes.CDLL("./clib.dll")
#clibrary.display()
#print ("\r\n")

configFile = os.path.split(os.path.realpath(sys.argv[0]))[0]+"/SpaceKat.cfg"
curPath = os.path.split(os.path.realpath(sys.argv[0]))[0]
configuration = {}
preTimeStamp = 0
curTimeStamp = 0
configString = ""
bConfigChanged = False

def LoadConfig():
    global configuration
    global configString
    global bConfigChanged
    global curTimeStamp

    if os.path.exists(configFile):
        print(configFile)
        JSONFile = open(configFile, encoding="utf-8")
        configuration=json.load(JSONFile)
        if curTimeStamp != configuration['TimeStamp']:
            bConfigChanged = True
            curTimeStamp = configuration['TimeStamp']
            print ("Configure changed.")
            comPort = configuration['Basic']['COMPort']
            tcpPort = configuration['Basic']['TCPPort']
            panZoomEnabled = configuration['Basic']['PanZoomEnabled']
            rotationEnabled = configuration['Basic']['RotationEnabled']
            dominantEnabled = configuration['Basic']['DominantEnabled']
            lockHorizonEnabled = configuration['Basic']['LockHorizonEnabled']
            zoomUsing = configuration['Basic']['ZoomBy']
            naviMode = configuration['Basic']['NaviMode']
            displayPivot = configuration['Basic']['DisplayPivot']
            useSelectObject = configuration['Basic']['UseSelectObject']
            
            panLREnabled = configuration['Speed']['PanLeftRightEnabled']
            panLRThreshold = configuration['Speed']['PanLeftRightThreshold']
            panLRSpeed = configuration['Speed']['PanLeftRightSpeed']
            panLRReverse = configuration['Speed']['PanLeftRightReverse']

            panFBEnabled = configuration['Speed']['PanForwardBackwardEnabled']
            panFBThreshold = configuration['Speed']['PanForwardBackwardThreshold']
            panFBSpeed = configuration['Speed']['PanForwardBackwardSpeed']
            panFBReverse = configuration['Speed']['PanForwardBackwardReverse']

            panUDEnabled = configuration['Speed']['PanUpDownEnabled']
            panUDThreshold = configuration['Speed']['PanUpDownThreshold']
            panUDSpeed = configuration['Speed']['PanUpDownSpeed']
            panUDReverse = configuration['Speed']['PanUpDownReverse']

            pitchEnabled = configuration['Speed']['PitchEnabled']
            pitchThreshold = configuration['Speed']['PitchThreshold']
            pitchSpeed = configuration['Speed']['PitchSpeed']
            pitchReverse = configuration['Speed']['PitchReverse']

            rollEnabled = configuration['Speed']['RollEnabled']
            rollThreshold = configuration['Speed']['RollThreshold']
            rollSpeed = configuration['Speed']['RollSpeed']
            rollReverse = configuration['Speed']['RollReverse']

            yawEnabled = configuration['Speed']['YawEnabled']
            yawThreshold = configuration['Speed']['YawThreshold']
            yawSpeed = configuration['Speed']['YawSpeed']
            yawReverse= configuration['Speed']['YawReverse']

            configString = f"{comPort},{tcpPort},{panZoomEnabled},{rotationEnabled},{dominantEnabled},{lockHorizonEnabled},{zoomUsing},{naviMode},{displayPivot},{useSelectObject},"+\
                f"{panLREnabled},{panLRThreshold},{panLRSpeed},{panLRReverse},{panFBEnabled},{panFBThreshold},{panFBSpeed},{panFBReverse},{panUDEnabled},{panUDThreshold},{panUDSpeed},{panUDReverse},"+\
                    f"{pitchEnabled},{pitchThreshold},{pitchSpeed},{pitchReverse},{rollEnabled},{rollThreshold},{rollSpeed},{rollReverse},{yawEnabled},{yawThreshold},{yawSpeed},{yawReverse}"
            print (configString)
        JSONFile.close()
        return True
    
    else:
        print ("Configuration file not exists. Default parameters will be used.")
        return False

bConfigLoaded = LoadConfig()


bSerialConnected = False
COMPort = 'COM7'

clientSocket = []
clientSocketTimeout = []
clientSocketNum = 0
currentSocketIndex = 0
TimetoCheck = 2.1
TimeoutCheck = 6.1
TimetoCheckConfig = 3.1
keyboard = Controller()
bLoop = True



localhost = "127.0.0.1"     # socket.gethostname()   # get local machine name
port = 8193  # Make sure it's within the > 1024 $$ <65535 range
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((localhost, port))

server.listen()

def SocketConnectionRemove(client):
    global clientSocket
    global clientSocketTimeout
    global clientSocketNum
    global SocketAcceptThread

    Index = clientSocket.index(client)
    clientSocket.remove(client)
    client.close()
    tmp = clientSocketTimeout[Index]
    clientSocketTimeout.remove(tmp)
    clientSocketNum -= 1
    server.setblocking(True)
    SocketAcceptThread = threading.Thread(target = SocketAccept)
    SocketAcceptThread.start()
    

def SocketConnectionCheck():
    global clientSocket
    global clientSocketNum
    print (f"Check Connections.")
    msgTest = 'SpaceKat Driver,0'

    for client in clientSocket:
        try:
            client.send(msgTest.encode())
        except:
            pass
  #          SocketConnectionRemove(client)
  #          print(f"Connection to {client} send error. It will be removed")
    
    for timecounter in clientSocketTimeout:
        if time.time() - timecounter > TimeoutCheck:
            Index = clientSocketTimeout.index(timecounter)
            tmpClient = clientSocket[Index]
            print(f"Connection to {Index}:{tmpClient} Timeout ({time.time() - timecounter}). It will be removed.")
            SocketConnectionRemove(tmpClient)

def SocketAccept():
    global clientSocket
    global clientSocketNum
    global currentSocketIndex
    global bLoop
    global preTimeStamp
    
    print('Socket Accept Thread Running. Waiting for Client Connection.')
    #while bLoop:
    try:
        server.setblocking(True)
        (client, addr) = server.accept()
        server.setblocking(False)
        clientSocket.append(client)
        clientSocketTimeout.append(time.time())
        currentSocketIndex = clientSocket.index(client)
        print(f'Client Connection Accepted. {clientSocketNum+1}')
        clientSocketNum += 1
        WorkingDir = "Path,"+curPath
        client.send(WorkingDir.encode())
        bConfigLoaded = LoadConfig()
        if bConfigLoaded:
            ParamsString = "Config,"+configString
            client.send(ParamsString.encode())
            preTimeStamp = curTimeStamp
            bConfigChanged = False
        time.sleep(0.01)
    except:
        pass

SocketAcceptThread = threading.Thread(target = SocketAccept)
SocketAcceptThread.start()

print("Program start")

#server.setblocking(False)

i=0
Time1 = time.time()
Time2 = time.time()

while bLoop:
    if not bSerialConnected:
        try:
            TheSerial = serial.Serial(COMPort, 115200, timeout=0.1)
            print("Serial Connected\r\n")
            bSerialConnected = True
            msg=b'a'
            TheSerial.write(msg)
        except KeyboardInterrupt:
            bLoop = False
            # sys.exit(0)
        except Exception:
            print("Serial not Connected\r\n")
            bSerialConnected = False
            try:
                time.sleep(1)
            except:
                print("Ctrl+C.")
                bLoop = False;
    else:
        try:
            Rawdata = TheSerial.readline()
            Rawdata1 = Rawdata.decode().strip()+','
            Data1 = Rawdata1.split(',')
            activeWindow = pygetwindow.getActiveWindow().title
            if Data1[0] == 'Data': # and bRotateFirst == False:      # Move
                xPos1 = eval(Data1[1])
                yPos1 = eval(Data1[2])
                zPos1 = eval(Data1[3])
                xPos2 = eval(Data1[4])
                yPos2 = eval(Data1[5])
                zPos2 = eval(Data1[6])
                xRoll = eval(Data1[7])
                yPitch = eval(Data1[8])
                zYaw = eval(Data1[9])
                # print(f"{xPos1, yPos1, zPos1}, {xPos2, yPos2, zPos2}, {xRoll, yPitch, zYaw}") 
                if clientSocketNum > 0:
                    CurrentSocket = clientSocket[currentSocketIndex]     # Last one for test
                    try:
                        if activeWindow.startswith('Autodesk Fusion') or activeWindow.startswith('Fusion360'):
                            CurrentSocket.send(Rawdata)
                    except:
                        print(f"Connection {currentSocketIndex} may have problem with connection to client")
                        SocketConnectionRemove(CurrentSocket)
            elif Data1[0] == 'Volume':
                if Data1[1] == "INC":
                    keyboard.press(Key.media_volume_up)
                else:
                    keyboard.press(Key.media_volume_down)
            elif Data1[0] == 'SM_MACRO':
                if Data1[1] == "FIND":
                    try:
                        CurrentSocket.send(Rawdata.decode().strip().encode())
                    except:
                        print(f"Connection {currentSocketIndex} may have problem with connection to client")
                        SocketConnectionRemove(CurrentSocket)
        
        except KeyboardInterrupt:
            bLoop = False
            # sys.exit(0)            
        
        except Exception:
            if TheSerial.isOpen:
                print("Reconnect Serial Port.")
                TheSerial.close()
            bSerialConnected = False
            pass

    

    inputs = clientSocket
    outputs = clientSocket
    

    if clientSocketNum > 0:
        #print ('Check Received Information')
        try:
            readable, writable, errinfo = select.select(inputs, outputs, inputs, 0.0005)
            
            for rsocket in readable:
                #print ('Read Socket Data')
                clientData = rsocket.recv(1024)
            # clientData = client.recv(1024)
                clientData = clientData.decode('utf-8')

                if len(clientData) > 0:
                    print (f"Received Data: {clientData}")
                    tmpIndex = clientSocket.index(rsocket)
                    clientSocketTimeout[tmpIndex] = time.time()
        except KeyboardInterrupt:
            bLoop = False
            # sys.exit(0)
        except Exception:
            SocketConnectionRemove(rsocket)
            pass
        #print ('Label2')
    if time.time() - Time1 >= TimetoCheck:
        print (f"Counter = {i}")
        i = 0
        Time1 = time.time()
        SocketConnectionCheck()
    if clientSocketNum > 0 and time.time() - Time2 >= TimetoCheckConfig:
        
        Time2 = time.time()
        bConfigLoaded = LoadConfig()
        print("Check configure change.")
        if bConfigLoaded and preTimeStamp != curTimeStamp:
            preTimeStamp = curTimeStamp
            CurrentSocket = clientSocket[currentSocketIndex]     # Last one for test
            try:
                ParamsString = "Config,"+configString
                CurrentSocket.send(ParamsString.encode())
                print("Send configure change to Fusion360.")
            except:
                print(f"Connection {currentSocketIndex} may have problem with connection to client")
                SocketConnectionRemove(CurrentSocket)
            print (f"{time.time()-Time2}")
        print (f"{time.time()-Time2}")
    time.sleep(0.007)
    i+=1

print("Quit application.")
server.close()

    