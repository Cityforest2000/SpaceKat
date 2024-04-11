import tkinter
import customtkinter
import spinbox
from PIL import Image
import os, sys
import json
import serial.tools.list_ports
import time

configFile = "./SpaceKat.cfg"
configFile = os.path.split(os.path.realpath(sys.argv[0]))[0]+"/SpaceKat.cfg"
plainconfigFile = os.path.split(os.path.realpath(sys.argv[0]))[0]+"/SpaceKatPlain.cfg"
print (configFile)
configuration = {}
configuration['TimeStamp'] = time.time()
configuration['Basic'] = {}
configuration['Speed'] = {}

panLREnabled = 1
panLRThreshold = 0.25
panLRSpeed = 50
panLRReverse = 1

panFBEnabled = 1
panFBThreshold = 0.25
panFBSpeed = 50
panFBReverse = 1

panUDEnabled = 1
panUDThreshold = 0.25
panUDSpeed = 50
panUDReverse = 1

pitchEnabled = 1
pitchThreshold = 1.5
pitchSpeed = 50
pitchReverse = 1

rollEnabled = 1
rollThreshold = 1.5
rollSpeed = 50
rollReverse = 1

yawEnabled = 1
yawThreshold = 1.5
yawSpeed = 50
yawReverse = 1

comPort = "COM7"
comPorts = []
comPortList = serial.tools.list_ports.comports()
for com in comPortList:
    #if com[0] != comPort:
    comPorts = comPorts + [com[0]]

tcpPort = 8193
panZoomEnabled = 1
rotationEnabled = 1
dominantEnabled = 0
lockHorizonEnabled = 0
zoomUsing = 1
naviMode = 0
displayPivot = 1
useSelectObject = 0

print (comPorts)

def LoadConfig():
    global configuration
    global comPort
    global panLREnabled
    global panLRThreshold
    global panLRSpeed
    global panLRReverse

    global panFBEnabled
    global panFBThreshold
    global panFBSpeed
    global panFBReverse

    global panUDEnabled
    global panUDThreshold
    global panUDSpeed
    global panUDReverse

    global pitchEnabled
    global pitchThreshold
    global pitchSpeed
    global pitchReverse

    global rollEnabled
    global rollThreshold
    global rollSpeed
    global rollReverse

    global yawEnabled
    global yawThreshold
    global yawSpeed
    global yawReverse

    global comPort

    global tcpPort
    global panZoomEnabled
    global rotationEnabled
    global dominantEnabled
    global lockHorizonEnabled
    global zoomUsing
    global naviMode
    global displayPivot
    global useSelectObject

    if os.path.exists(configFile):
        print(configFile)
        JSONFile = open(configFile, encoding="utf-8")
        configuration=json.load(JSONFile)

        JSONFile.close()


        try:

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
            print ("Load Configuration.")
        except:
            print ("Load configuration not completed.")
    else:
        print ("Configuration file not exists. Default parameters will be used.")

def SaveConfig():
        global configuration
    #try:
        panLREnabled = app.FrameSpeed.swPanLRVar.get()
        panLRThreshold = app.FrameSpeed.spinboxPanLR.get()
        panLRSpeed = app.FrameSpeed.panLRSpeedVar.get()
        panLRReverse = app.FrameSpeed.panLRReverseVar.get()

        panFBEnabled = app.FrameSpeed.swPanFBVar.get()
        panFBThreshold = app.FrameSpeed.spinboxPanFB.get()
        panFBSpeed = app.FrameSpeed.panFBSpeedVar.get()
        panFBReverse = app.FrameSpeed.panFBReverseVar.get()

        panUDEnabled = app.FrameSpeed.swPanUDVar.get()
        panUDThreshold = app.FrameSpeed.spinboxPanUD.get()
        panUDSpeed = app.FrameSpeed.panUDSpeedVar.get()
        panUDReverse = app.FrameSpeed.panUDReverseVar.get()

        pitchEnabled = app.FrameSpeed.swPitchVar.get()
        pitchThreshold = app.FrameSpeed.spinboxPitch.get()
        pitchSpeed = app.FrameSpeed.pitchSpeedVar.get()
        pitchReverse = app.FrameSpeed.pitchReverseVar.get()

        rollEnabled = app.FrameSpeed.swRollVar.get()
        rollThreshold = app.FrameSpeed.spinboxRoll.get()
        rollSpeed = app.FrameSpeed.rollSpeedVar.get()
        rollReverse = app.FrameSpeed.rollReverseVar.get()

        yawEnabled = app.FrameSpeed.swYawVar.get()
        yawThreshold = app.FrameSpeed.spinboxYaw.get()
        yawSpeed = app.FrameSpeed.yawSpeedVar.get()
        yawReverse = app.FrameSpeed.yawReverseVar.get()

        comPort = app.FrameBasic.ComboxCOMVar.get()
        tcpPort = app.FrameBasic.EntryTCP.get()
        panZoomEnabled = app.FrameBasic.ChkboxPanVar.get()
        rotationEnabled = app.FrameBasic.ChkboxRotationVar.get()
        dominantEnabled = app.FrameBasic.ChkboxDomVar.get()
        lockHorizonEnabled = app.FrameBasic.ChkboxHorizonVar.get()
        zoomUsing = app.FrameBasic.ZoomVar.get()
        naviMode = app.FrameBasic.ModeVar.get()
        displayPivot = app.FrameBasic.ShowCenterVar.get()
        useSelectObject = app.FrameBasic.swSelObjVar.get()

        
        configuration['TimeStamp'] = time.time()
        configuration['Basic']['COMPort'] = comPort
        configuration['Basic']['TCPPort'] = tcpPort
        configuration['Basic']['PanZoomEnabled'] = panZoomEnabled
        configuration['Basic']['RotationEnabled'] = rotationEnabled
        configuration['Basic']['DominantEnabled'] = dominantEnabled
        configuration['Basic']['LockHorizonEnabled'] = lockHorizonEnabled
        configuration['Basic']['ZoomBy'] = zoomUsing
        configuration['Basic']['NaviMode'] = naviMode
        configuration['Basic']['DisplayPivot'] = displayPivot
        configuration['Basic']['UseSelectObject'] = useSelectObject
        
        configuration['Speed']['PanLeftRightEnabled'] = panLREnabled
        configuration['Speed']['PanLeftRightThreshold'] = panLRThreshold
        configuration['Speed']['PanLeftRightSpeed'] = panLRSpeed
        configuration['Speed']['PanLeftRightReverse'] = panLRReverse

        configuration['Speed']['PanForwardBackwardEnabled'] = panFBEnabled
        configuration['Speed']['PanForwardBackwardThreshold'] = panFBThreshold
        configuration['Speed']['PanForwardBackwardSpeed'] = panFBSpeed
        configuration['Speed']['PanForwardBackwardReverse'] = panFBReverse

        configuration['Speed']['PanUpDownEnabled'] = panUDEnabled
        configuration['Speed']['PanUpDownThreshold'] = panUDThreshold
        configuration['Speed']['PanUpDownSpeed'] = panUDSpeed
        configuration['Speed']['PanUpDownReverse'] = panUDReverse

        configuration['Speed']['PitchEnabled'] = pitchEnabled
        configuration['Speed']['PitchThreshold'] = pitchThreshold
        configuration['Speed']['PitchSpeed'] = pitchSpeed
        configuration['Speed']['PitchReverse'] = pitchReverse

        configuration['Speed']['RollEnabled'] = rollEnabled
        configuration['Speed']['RollThreshold'] = rollThreshold
        configuration['Speed']['RollSpeed'] = rollSpeed
        configuration['Speed']['RollReverse'] = rollReverse

        configuration['Speed']['YawEnabled'] = yawEnabled
        configuration['Speed']['YawThreshold'] = yawThreshold
        configuration['Speed']['YawSpeed'] = yawSpeed
        configuration['Speed']['YawReverse'] = yawReverse

        JSONFile = open(configFile, "w", encoding="utf-8")
        json.dump(configuration, JSONFile)
        JSONFile.close()

        configString = f"{configuration['TimeStamp']},{comPort},{tcpPort},{panZoomEnabled},{rotationEnabled},{dominantEnabled},{lockHorizonEnabled},{zoomUsing},{naviMode},{displayPivot},{useSelectObject},"+\
                f"{panLREnabled},{panLRThreshold},{panLRSpeed},{panLRReverse},{panFBEnabled},{panFBThreshold},{panFBSpeed},{panFBReverse},{panUDEnabled},{panUDThreshold},{panUDSpeed},{panUDReverse},"+\
                    f"{pitchEnabled},{pitchThreshold},{pitchSpeed},{pitchReverse},{rollEnabled},{rollThreshold},{rollSpeed},{rollReverse},{yawEnabled},{yawThreshold},{yawSpeed},{yawReverse}"
        print (configString)
        stringFile = open(plainconfigFile, "w", encoding="utf-8")
        stringFile.write(configString)
        stringFile.close()

        print(f"{configuration} ")
        print(f"Config Saved.")
    #except:
        #print(f"Config not Saved.")
        #pass


LoadConfig()


customtkinter.set_appearance_mode("dark")
customtkinter.set_default_color_theme("blue")
# image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "test_images")
image_path = os.path.dirname(os.path.realpath(__file__))
button_fg_color = ('#3B8ED0', '#1F6AA5')
button_hover_color = ('#36719F', '#2896F0')

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()
        FontYaHei = customtkinter.CTkFont(family="Microsoft YaHei UI")
        self.geometry("1280x720")
        self.resizable(width=False, height=False)
        self.title("SpaceKat 设置")
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        

        self.FrameSidebar = Sidebar_Frame(self)
        self.FrameSidebar.grid(row=0, column=0, rowspan=2, padx=10, pady=(10, 0), sticky="nsw")        

        self.FrameBasic = Basic_Frame(self)
        #self.FrameBasic.grid(row=0, column=1, padx=10, pady=(10, 0), sticky="nsew")   

        self.FrameSpeed = Speed_Frame(self)
        #self.FrameSpeed.grid(row=0, column=1, padx=10, pady=(10, 0), sticky="nsew")
        #self.sidebar_frame = customtkinter.CTkFrame(self, width=350, corner_radius=0)
        #self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsw")

        self.FrameKey = Key_Frame(self)

        self.FrameSidebar.ConfigBasic()

        self.FrameBottom = customtkinter.CTkFrame(self)
        self.FrameBottom.grid(row=1, column=1, padx=10, pady=(10,10), sticky="sew")
        self.FrameBottom.grid_columnconfigure(0, weight=1)
        self.FrameBottom.grid_columnconfigure(1, weight=0)
        self.btnSave = customtkinter.CTkButton(self.FrameBottom, text="保存", font=FontYaHei, command=SaveConfig)
        self.btnSave.grid(row=0, column=1, padx=(10,30), pady=(15,15), sticky="w")
        print("This is SpaceKat Setting.")
        
class Sidebar_Frame(customtkinter.CTkFrame):
    def __init__(self, master):
        super().__init__(master)
        FontYaHei = customtkinter.CTkFont(family="Microsoft YaHei UI")
        FontYaHei15 = customtkinter.CTkFont(family="Microsoft YaHei UI",size=15, weight="bold")

        self.logo_image = customtkinter.CTkImage(Image.open(os.path.join(image_path, "CustomTkinter_logo_single.png")), size=(26, 26))
        self.grid_rowconfigure(4, weight=1)

        self.navigation_frame_label = customtkinter.CTkLabel(self, text="  SpaceKat 设置", image=self.logo_image,
                                                             compound="left", font=FontYaHei15)
        self.navigation_frame_label.grid(row=0, column=0, padx=20, pady=20, sticky="w")

        #self.logo_label = customtkinter.CTkLabel(self, text="选项", font=customtkinter.CTkFont(family="Microsoft YaHei UI"))
        #self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 0), sticky="w")
        self.ButtonBasic = customtkinter.CTkButton(self, text="基础设置", width = 200, font=FontYaHei, hover_color=button_hover_color, command=self.ConfigBasic)
        self.ButtonBasic.grid(row=1, column=0, padx=20, pady=(0, 0), sticky="w")
        self.ButtonSpeed = customtkinter.CTkButton(self, text="敏感度与速度", width = 200, font=FontYaHei, hover_color=button_hover_color, command=self.ConfigSpeed)
        self.ButtonSpeed.grid(row=2, column=0, padx=20, pady=(20, 0), sticky="w")
        #self.ButtonMode = customtkinter.CTkButton(self, text="模式", width = 200, font=FontYaHei)
        #self.ButtonMode.grid(row=3, column=0, padx=20, pady=(20,0), sticky="w")
        #self.ButtonRotation = customtkinter.CTkButton(self, text="旋转中心", width = 200, font=FontYaHei)
        #self.ButtonRotation.grid(row=4, column=0, padx=20, pady=(20,0), sticky="w")
        self.ButtonKey = customtkinter.CTkButton(self, text="自定义键盘", width = 200, font=FontYaHei, state="disabled", hover_color=button_hover_color, command=self.ConfigKey)
        self.ButtonKey.grid(row=3, column=0, padx=20, pady=(20,0), sticky="w")

        self.appearance_mode_menu = customtkinter.CTkOptionMenu(self, values=["Dark", "Light", "System"], width=200,
                                                                command=self.change_appearance_mode_event)
        self.appearance_mode_menu.grid(row=4, column=0, padx=20, pady=20, sticky="s")

    # self.sidebar_frame = customtkinter.CTkFrame(self, width=140, corner_radius=0)
    # self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
    # self.sidebar_frame.grid_rowconfigure(4, weight=1)
    def change_appearance_mode_event(self, new_appearance_mode):
        customtkinter.set_appearance_mode(new_appearance_mode)
    
    def ConfigBasic(self):
        self.ButtonBasic.configure(fg_color=button_hover_color)
        self.ButtonSpeed.configure(fg_color=button_fg_color)
        self.ButtonKey.configure(fg_color=button_fg_color)
        self.master.FrameSpeed.grid_forget()
        self.master.FrameBasic.grid(row=0, column=1, padx=10, pady=(10, 0), sticky="nsew")
        self.master.FrameKey.grid_forget()

    def ConfigSpeed(self):
        self.ButtonSpeed.configure(fg_color=button_hover_color)
        self.ButtonBasic.configure(fg_color=button_fg_color)
        self.ButtonKey.configure(fg_color=button_fg_color)
        self.master.FrameSpeed.grid(row=0, column=1, padx=10, pady=(10, 0), sticky="nsew")
        self.master.FrameBasic.grid_forget()
        self.master.FrameKey.grid_forget()

    def ConfigKey(self):
        self.ButtonKey.configure(fg_color=button_hover_color)
        self.ButtonBasic.configure(fg_color=button_fg_color)
        self.ButtonSpeed.configure(fg_color=button_fg_color)
        self.master.FrameKey.grid(row=0, column=1, padx=10, pady=(10, 0), sticky="nsew")
        self.master.FrameBasic.grid_forget()
        self.master.FrameSpeed.grid_forget()

class Basic_Frame(customtkinter.CTkFrame):
    def __init__(self, master):
        super().__init__(master)
        FontYaHei = customtkinter.CTkFont(family="Microsoft YaHei UI")
        FontYaHei16 = customtkinter.CTkFont(family="Microsoft YaHei UI",size=16, weight="bold")
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=0)
        self.grid_columnconfigure(2, weight=0)
        self.grid_rowconfigure(0, weight=0)
        self.grid_rowconfigure(1, weight=0)

        self.ConnectionFrame = customtkinter.CTkFrame(self, width=350)
        self.ConnectionFrame.grid(row=0, column=0, padx=20, pady=(20,0), sticky="nw")
        self.ConnectionFrame.grid_columnconfigure(0, weight=0)
        self.ConnectionFrame.grid_columnconfigure(1, weight=1)
        self.LabelConnection = customtkinter.CTkLabel(self.ConnectionFrame, text="连接配置 Connection", font=FontYaHei16)
        self.LabelConnection.grid(row=0, column=0, columnspan=2, padx=(20,116), pady=(5, 0), sticky="w")

        self.LabelCOM = customtkinter.CTkLabel(self.ConnectionFrame, text="COM Port", font=FontYaHei)
        self.LabelCOM.grid(row=1, column=0, padx=20, pady=(20,0), sticky="nw")
        self.ComboxCOMVar = customtkinter.StringVar(value=comPort)
        self.ComboxCOM = customtkinter.CTkComboBox(master=self.ConnectionFrame, variable=self.ComboxCOMVar, values=comPorts, font=FontYaHei)
        self.ComboxCOM.grid(row=1, column=1, pady=(20, 0), padx=20, sticky="nw")

        self.LabelTCP = customtkinter.CTkLabel(self.ConnectionFrame, text="TCP Port", font=FontYaHei)
        self.LabelTCP.grid(row=2, column=0, padx=20, pady=(20,20), sticky="nw")
        self.EntryTCPVar = customtkinter.IntVar(value=tcpPort)
        self.EntryTCP = customtkinter.CTkEntry(master=self.ConnectionFrame, font=FontYaHei)
        self.EntryTCP.insert(0, tcpPort)
        self.EntryTCP.grid(row=2, column=1, pady=(20, 20), padx=20, sticky="nw")

        self.NaviFrame = customtkinter.CTkFrame(self, width=350)
        self.NaviFrame.grid(row=1, column=0, padx=20, pady=(20,0), sticky="nw")
        self.Label1 = customtkinter.CTkLabel(self.NaviFrame, text="导航 Navigation", font=FontYaHei16)
        self.Label1.grid(row=0, column=0, padx=(20,150), pady=(5, 0), sticky="w")


        self.ChkboxPanVar = customtkinter.IntVar(value=panZoomEnabled)
        self.ChkboxPan = customtkinter.CTkCheckBox(master=self.NaviFrame, variable=self.ChkboxPanVar, text="  移动/缩放 Pan/Zoom", font=FontYaHei)
        self.ChkboxPan.grid(row=1, column=0, pady=(20, 0), padx=20, sticky="nw")
        self.ChkboxRotationVar = customtkinter.IntVar(value=rotationEnabled)
        self.ChkboxRotation = customtkinter.CTkCheckBox(master=self.NaviFrame, variable=self.ChkboxRotationVar, text="  旋转 Rotation", font=FontYaHei)
        self.ChkboxRotation.grid(row=2, column=0, pady=(20, 0), padx=20, sticky="nw")
        self.ChkboxDomVar = customtkinter.IntVar(value=dominantEnabled)
        self.ChkboxDom = customtkinter.CTkCheckBox(master=self.NaviFrame, variable=self.ChkboxDomVar, text="  轴向锁定 Dominant", font=FontYaHei)
        self.ChkboxDom.grid(row=3, column=0, pady=(20, 0), padx=20, sticky="nw")
        self.ChkboxHorizonVar = customtkinter.IntVar(value=lockHorizonEnabled)
        self.ChkboxHorizon = customtkinter.CTkCheckBox(master=self.NaviFrame, variable=self.ChkboxHorizonVar, text="  水平锁定 Lock Horizon", font=FontYaHei)
        self.ChkboxHorizon.grid(row=4, column=0, pady=(20, 20), padx=20, sticky="nw")

        self.ZoomFrame = customtkinter.CTkFrame(self)
        self.ZoomFrame.grid(row=2, column=0, padx=(20, 20), pady=(20, 0), sticky="nw")
        self.ZoomVar = tkinter.IntVar(value=zoomUsing)
        self.label_radio_group = customtkinter.CTkLabel(master=self.ZoomFrame, text="缩放方式 Zoom Using", font=FontYaHei16)
        self.label_radio_group.grid(row=0, column=0, columnspan=1, padx=(20,110), pady=(5,0), sticky="nw")
        self.RBForward = customtkinter.CTkRadioButton(master=self.ZoomFrame, text="  前后 Forward/Backward", font=FontYaHei, variable=self.ZoomVar, value=0)
        self.RBForward.grid(row=1, column=0, pady=(20,0), padx=20, sticky="nw")
        self.RBUpDown = customtkinter.CTkRadioButton(master=self.ZoomFrame, text="  上下 Up/Down", font=FontYaHei, variable=self.ZoomVar, value=1)
        self.RBUpDown.grid(row=2, column=0, pady=(20,20), padx=20, sticky="nw")

        self.ModeFrame = customtkinter.CTkFrame(self)
        self.ModeFrame.grid(row=0, column=1, padx=(20, 20), pady=(20, 0), sticky="nw")
        self.ModeVar = tkinter.IntVar(value=naviMode)
        self.label_Mode_Rgroup = customtkinter.CTkLabel(master=self.ModeFrame, text="导航模式 Navigation Mode", font=FontYaHei16)
        self.label_Mode_Rgroup.grid(row=0, column=0, columnspan=1, padx=(20,110), pady=(5,0), sticky="nw")
        self.RBObject = customtkinter.CTkRadioButton(master=self.ModeFrame, text="  对象模式 Object Mode", font=FontYaHei, variable=self.ModeVar, value=0)
        self.RBObject.grid(row=1, column=0, pady=(20,0), padx=20, sticky="nw")
        self.RBUpCamera = customtkinter.CTkRadioButton(master=self.ModeFrame, text="  目标相机模式 Target Camera Mode", state="disabled", font=FontYaHei, variable=self.ModeVar, value=1)
        self.RBUpCamera.grid(row=2, column=0, pady=(20,20), padx=20, sticky="nw")

        self.RCenterFrame = customtkinter.CTkFrame(self)
        self.RCenterFrame.grid(row=1, column=1, padx=(20, 20), pady=(20, 0), sticky="nw")
        self.ShowCenterVar = tkinter.IntVar(value=displayPivot)
        self.label_RCenter_Rgroup = customtkinter.CTkLabel(master=self.RCenterFrame, text="旋转中心 Rotation Center", font=FontYaHei16)
        self.label_RCenter_Rgroup.grid(row=0, column=0, columnspan=1, padx=(20,120), pady=(5,0), sticky="nw")
        self.swShowRCenter = customtkinter.CTkSwitch(self.RCenterFrame, state="disabled", text="显示中心点", variable=self.ShowCenterVar, font=FontYaHei)
        self.swShowRCenter.grid(row=1, column=0, padx=20, pady=(20,0), sticky="w")
        self.swSelObjVar = customtkinter.IntVar(value=useSelectObject)
        self.swSelObj = customtkinter.CTkSwitch(self.RCenterFrame, variable=self.swSelObjVar, state="disabled", text="使用选中对象", font=FontYaHei)
        self.swSelObj.grid(row=2, column=0, padx=20, pady=(20,0), sticky="w")        

    # Button1 = customtkinter.CTkButton(app, text='Ok', command=SaveConfig)
    # Button1.pack(padx=10, pady=10)

class Speed_Frame(customtkinter.CTkFrame):
    def panLREvent(self):
        if self.swPanLRVar.get() == 1:
            self.spinboxPanLR.configurestatenormal()
            self.sliderPanLR.configure(state="normal")
            self.ChkboxPanLRReverse.configure(state="normal")
            
        else:
            self.spinboxPanLR.configurestatedisabled()
            self.sliderPanLR.configure(state="disabled")
            self.ChkboxPanLRReverse.configure(state="disabled")

    def panFBEvent(self):
        if self.swPanFBVar.get() == 1:
            self.spinboxPanFB.configurestatenormal()
            self.sliderPanFB.configure(state="normal")
            self.ChkboxPanFBReverse.configure(state="normal")
            
        else:
            self.spinboxPanFB.configurestatedisabled()
            self.sliderPanFB.configure(state="disabled")
            self.ChkboxPanFBReverse.configure(state="disabled")

    def panUDEvent(self):
        if self.swPanUDVar.get() == 1:
            self.spinboxPanUD.configurestatenormal()
            self.sliderPanUD.configure(state="normal")
            self.ChkboxPanUDReverse.configure(state="normal")
            
        else:
            self.spinboxPanUD.configurestatedisabled()
            self.sliderPanUD.configure(state="disabled")
            self.ChkboxPanUDReverse.configure(state="disabled")

    def pitchEvent(self):
        if self.swPitchVar.get() == 1:
            self.spinboxPitch.configurestatenormal()
            self.sliderPitch.configure(state="normal")
            self.ChkboxPitchReverse.configure(state="normal")
            
        else:
            self.spinboxPitch.configurestatedisabled()
            self.sliderPitch.configure(state="disabled")
            self.ChkboxPitchReverse.configure(state="disabled")

    def rollEvent(self):
        if self.swRollVar.get() == 1:
            self.spinboxRoll.configurestatenormal()
            self.sliderRoll.configure(state="normal")
            self.ChkboxRollReverse.configure(state="normal")
            
        else:
            self.spinboxRoll.configurestatedisabled()
            self.sliderRoll.configure(state="disabled")
            self.ChkboxRollReverse.configure(state="disabled")

    def yawEvent(self):
        if self.swYawVar.get() == 1:
            self.spinboxYaw.configurestatenormal()
            self.sliderYaw.configure(state="normal")
            self.ChkboxYawReverse.configure(state="normal")
            
        else:
            self.spinboxYaw.configurestatedisabled()
            self.sliderYaw.configure(state="disabled")
            self.ChkboxYawReverse.configure(state="disabled")
            

            
    def __init__(self, master):
        super().__init__(master)
        FontYaHei = customtkinter.CTkFont(family="Microsoft YaHei UI")
        FontYaHei16 = customtkinter.CTkFont(family="Microsoft YaHei UI",size=16, weight="bold")
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=1)
        self.grid_columnconfigure(3, weight=1)
        self.grid_columnconfigure(4, weight=1)
        self.grid_rowconfigure(0, weight=0)
        self.grid_rowconfigure(1, weight=0)
        self.grid_rowconfigure(2, weight=0)
        self.grid_rowconfigure(3, weight=0)
        self.grid_rowconfigure(4, weight=0)

        self.SpeedFrame = customtkinter.CTkFrame(self)
        self.SpeedFrame.grid(row=0, column=0, padx=20, pady=(20,0), sticky="nw")
        self.Label1 = customtkinter.CTkLabel(self.SpeedFrame, text="敏感度与速度", font=FontYaHei16)
        self.Label1.grid(row=0, column=0, padx=(20,0), pady=(5, 0), sticky="w")

        self.Label2 = customtkinter.CTkLabel(self.SpeedFrame, text="启用", font=FontYaHei)
        self.Label2.grid(row=1, column=1, padx=20, pady=(20,0), sticky="w")
        self.Label3 = customtkinter.CTkLabel(self.SpeedFrame, text="敏感度阈值", font=FontYaHei)
        self.Label3.grid(row=1, column=2, padx=20, pady=(20,0))
        self.Label4 = customtkinter.CTkLabel(self.SpeedFrame, text="速度", font=FontYaHei)
        self.Label4.grid(row=1, column=3, padx=20, pady=(20,0))
        self.Label5 = customtkinter.CTkLabel(self.SpeedFrame, text="反向", font=FontYaHei)
        self.Label5.grid(row=1, column=4, padx=20, pady=(20,0), sticky="w")

        self.Label11 = customtkinter.CTkLabel(self.SpeedFrame, text="左右平移", font=FontYaHei)
        self.Label11.grid(row=2, column=0, padx=20, pady=(20,0), sticky="w")
        self.swPanLRVar = customtkinter.IntVar(value=panLREnabled)
        self.swPanLR = customtkinter.CTkSwitch(self.SpeedFrame, text="", variable=self.swPanLRVar, onvalue=1, offvalue=0, command=self.panLREvent)
        self.swPanLR.grid(row=2, column=1, padx=20, pady=(20,0), sticky="w")
        self.panLRThresholdVar = customtkinter.DoubleVar(value=panLRThreshold)
        self.spinboxPanLR = spinbox.FloatSpinbox(self.SpeedFrame, width=120, value=panLRThreshold, step_size=0.05)
        self.spinboxPanLR.grid(row=2, column=2, padx=20, pady=(20,0), sticky="w")
        self.panLRSpeedVar = customtkinter.IntVar(value=panLRSpeed)
        self.sliderPanLR = customtkinter.CTkSlider(self.SpeedFrame, variable=self.panLRSpeedVar, width=350, from_=1, to=100)
        self.sliderPanLR.grid(row=2, column=3, padx=20, pady=(20,0), sticky="w")
        self.panLRReverseVar = customtkinter.IntVar(value=panLRReverse)
        self.ChkboxPanLRReverse = customtkinter.CTkCheckBox(master=self.SpeedFrame, variable=self.panLRReverseVar, onvalue=-1, offvalue=1, text="", font=FontYaHei)
        self.ChkboxPanLRReverse.grid(row=2, column=4, pady=(20, 0), padx=20, sticky="nw")

        self.Label12 = customtkinter.CTkLabel(self.SpeedFrame, text="前后平移", font=FontYaHei)
        self.Label12.grid(row=3, column=0, padx=20, pady=(20,0), sticky="w")
        self.swPanFBVar = customtkinter.IntVar(value=panFBEnabled)
        self.swPanFB = customtkinter.CTkSwitch(self.SpeedFrame, variable=self.swPanFBVar,  onvalue=1, offvalue=0, command=self.panFBEvent, text="")
        self.swPanFB.grid(row=3, column=1, padx=20, pady=(20,0), sticky="w")
        self.spinboxPanFB = spinbox.FloatSpinbox(self.SpeedFrame, width=120, value=panFBThreshold, step_size=0.05)
        self.spinboxPanFB.grid(row=3, column=2, padx=20, pady=(20,0), sticky="w")
        self.panFBSpeedVar = customtkinter.IntVar(value=panFBSpeed)
        self.sliderPanFB = customtkinter.CTkSlider(self.SpeedFrame, variable=self.panFBSpeedVar, width=350, from_=1, to=100)
        self.sliderPanFB.grid(row=3, column=3, padx=20, pady=(20,0), sticky="w")
        self.panFBReverseVar = customtkinter.IntVar(value=panFBReverse)
        self.ChkboxPanFBReverse = customtkinter.CTkCheckBox(master=self.SpeedFrame, variable=self.panFBReverseVar, onvalue=-1, offvalue=1, text="", font=FontYaHei)
        self.ChkboxPanFBReverse.grid(row=3, column=4, pady=(20, 0), padx=20, sticky="nw")

        self.Label13 = customtkinter.CTkLabel(self.SpeedFrame, text="上下移动", font=FontYaHei)
        self.Label13.grid(row=4, column=0, padx=20, pady=(20,0), sticky="w")
        self.swPanUDVar = customtkinter.IntVar(value=panUDEnabled)
        self.swPanUD = customtkinter.CTkSwitch(self.SpeedFrame, variable=self.swPanUDVar,  onvalue=1, offvalue=0, command=self.panUDEvent, text="")
        self.swPanUD.grid(row=4, column=1, padx=20, pady=(20,0), sticky="w")
        self.spinboxPanUD = spinbox.FloatSpinbox(self.SpeedFrame, width=120, value=panUDThreshold, step_size=0.05)
        self.spinboxPanUD.grid(row=4, column=2, padx=20, pady=(20,0), sticky="w")
        self.panUDSpeedVar = customtkinter.IntVar(value=panUDSpeed)
        self.sliderPanUD = customtkinter.CTkSlider(self.SpeedFrame,variable=self.panUDSpeedVar,  width=350, from_=1, to=100)
        self.sliderPanUD.grid(row=4, column=3, padx=20, pady=(20,0), sticky="w")
        self.panUDReverseVar = customtkinter.IntVar(value=panUDReverse)
        self.ChkboxPanUDReverse = customtkinter.CTkCheckBox(master=self.SpeedFrame, variable=self.panUDReverseVar, onvalue=-1, offvalue=1, text="", font=FontYaHei)
        self.ChkboxPanUDReverse.grid(row=4, column=4, pady=(20, 0), padx=20, sticky="nw")

        self.Label14 = customtkinter.CTkLabel(self.SpeedFrame, text="前后倾斜 Pitch", font=FontYaHei)
        self.Label14.grid(row=5, column=0, padx=20, pady=(20,0), sticky="w")
        self.swPitchVar = customtkinter.IntVar(value=pitchEnabled)
        self.swPitch = customtkinter.CTkSwitch(self.SpeedFrame, variable=self.swPitchVar,  onvalue=1, offvalue=0, command=self.pitchEvent, text="")
        self.swPitch.grid(row=5, column=1, padx=20, pady=(20,0), sticky="w")
        self.spinboxPitch = spinbox.FloatSpinbox(self.SpeedFrame, width=120, value=pitchThreshold, step_size=0.05)
        self.spinboxPitch.grid(row=5, column=2, padx=20, pady=(20,0), sticky="w")
        self.pitchSpeedVar = customtkinter.IntVar(value=pitchSpeed)
        self.sliderPitch = customtkinter.CTkSlider(self.SpeedFrame, variable=self.pitchSpeedVar, width=350, from_=1, to=100)
        self.sliderPitch.grid(row=5, column=3, padx=20, pady=(20,0), sticky="w")
        self.pitchReverseVar = customtkinter.IntVar(value=pitchReverse)
        self.ChkboxPitchReverse = customtkinter.CTkCheckBox(master=self.SpeedFrame, variable=self.pitchReverseVar, onvalue=-1, offvalue=1, text="", font=FontYaHei)
        self.ChkboxPitchReverse.grid(row=5, column=4, pady=(20, 0), padx=20, sticky="nw")

        self.Label15 = customtkinter.CTkLabel(self.SpeedFrame, text="左右倾斜 Roll", font=FontYaHei)
        self.Label15.grid(row=6, column=0, padx=20, pady=(20,0), sticky="w")
        self.swRollVar = customtkinter.IntVar(value=pitchEnabled)
        self.swRoll = customtkinter.CTkSwitch(self.SpeedFrame, variable=self.swRollVar,  onvalue=1, offvalue=0, command=self.rollEvent, text="")
        self.swRoll.grid(row=6, column=1, padx=20, pady=(20,0), sticky="w")
        self.spinboxRoll = spinbox.FloatSpinbox(self.SpeedFrame, width=120, value=rollThreshold, step_size=0.05)
        self.spinboxRoll.grid(row=6, column=2, padx=20, pady=(20,0), sticky="w")
        self.rollSpeedVar = customtkinter.IntVar(value=rollSpeed)
        self.sliderRoll = customtkinter.CTkSlider(self.SpeedFrame,  variable=self.rollSpeedVar, width=350, from_=1, to=100)
        self.sliderRoll.grid(row=6, column=3, padx=20, pady=(20,0), sticky="w")
        self.rollReverseVar = customtkinter.IntVar(value=rollReverse)
        self.ChkboxRollReverse = customtkinter.CTkCheckBox(master=self.SpeedFrame, variable=self.rollReverseVar, onvalue=-1, offvalue=1, text="", font=FontYaHei)
        self.ChkboxRollReverse.grid(row=6, column=4, pady=(20, 0), padx=20, sticky="nw")

        self.Label16 = customtkinter.CTkLabel(self.SpeedFrame, text="旋转 Yaw", font=FontYaHei)
        self.Label16.grid(row=7, column=0, padx=20, pady=(20,20), sticky="w")
        self.swYawVar = customtkinter.IntVar(value=yawEnabled)
        self.swYaw = customtkinter.CTkSwitch(self.SpeedFrame, variable=self.swYawVar,  onvalue=1, offvalue=0, command=self.yawEvent, text="")
        self.swYaw.grid(row=7, column=1, padx=20, pady=(20,20), sticky="w")
        self.spinboxYaw = spinbox.FloatSpinbox(self.SpeedFrame, width=120, value=yawThreshold, step_size=0.05)
        self.spinboxYaw.grid(row=7, column=2, padx=20, pady=(20,20), sticky="w")
        self.yawSpeedVar = customtkinter.IntVar(value=yawSpeed)
        self.sliderYaw = customtkinter.CTkSlider(self.SpeedFrame,  variable=self.yawSpeedVar, width=350, from_=1, to=100)
        self.sliderYaw.grid(row=7, column=3, padx=20, pady=(20,20), sticky="w")
        self.yawReverseVar = customtkinter.IntVar(value=yawReverse)
        self.ChkboxYawReverse = customtkinter.CTkCheckBox(master=self.SpeedFrame, variable=self.yawReverseVar, onvalue=-1, offvalue=1, text="", font=FontYaHei)
        self.ChkboxYawReverse.grid(row=7, column=4, pady=(20, 20), padx=20, sticky="nw")




class Key_Frame(customtkinter.CTkFrame):
    def __init__(self, master):
        super().__init__(master)
        FontYaHei = customtkinter.CTkFont(family="Microsoft YaHei UI")
        FontYaHei16 = customtkinter.CTkFont(family="Microsoft YaHei UI",size=16, weight="bold")
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=1)
        self.grid_columnconfigure(3, weight=1)
        self.grid_columnconfigure(4, weight=1)
        self.grid_rowconfigure(0, weight=0)
        self.grid_rowconfigure(1, weight=0)
        self.grid_rowconfigure(2, weight=0)
        self.grid_rowconfigure(3, weight=0)
        self.grid_rowconfigure(4, weight=0)

        self.Label1 = customtkinter.CTkLabel(self, text="自定义键盘", font=FontYaHei16)
        self.Label1.grid(row=0, column=0, padx=(20,0), pady=(5, 0), sticky="w")
        self.KeyImgFrame = customtkinter.CTkFrame(self)
        self.KeyImgFrame.grid(row=2, column=0, padx=20, pady=(20,0), sticky="nw")


        
        self.btnLayer = customtkinter.CTkSegmentedButton(self)
        self.btnLayer.grid(row=1, column=0, columnspan=2, padx=20, pady=(20,0), sticky="nw")
        self.btnLayer.configure(values=["  Fusion360  ", "  Blender  ", "  Pr  ", "  Solidworks  ", "  Undefined1  ", "  Undefined2  ", "  Undefined3  "])
        self.btnLayer.set("  Fusion360  ")

        self.btnEncoder = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=30, fg_color="transparent", border_width=1, corner_radius=25, border_color=("gray50", "gray30"), text="Enc")
        self.btnEncoder.grid(row=0, column=0, padx=(10,10), pady=20, sticky="nw")
        self.btnLED = customtkinter.CTkButton(self.KeyImgFrame, height=45, width=160, fg_color="transparent", border_width=1, corner_radius=1, border_color=("gray50", "gray30"), text="LED")
        self.btnLED.grid(row=0, column=1, columnspan=3, padx=(10,10), pady=20, sticky="w")
        
        self.btnKey1 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=60, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="ESC")
        self.btnKey1.grid(row=1, column=0, padx=(10,10), pady=20, sticky="nw")
        self.btnKey2 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=60, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="Extru")
        self.btnKey2.grid(row=1, column=1, padx=(10,10), pady=20, sticky="nw")
        self.btnKey3 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=60, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="FIND")
        self.btnKey3.grid(row=1, column=2, padx=(10,10), pady=20, sticky="nw")
        self.btnKey4 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=60, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="UNDO")
        self.btnKey4.grid(row=1, column=3, padx=(10,10), pady=20, sticky="nw")
        self.btnKey5 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=60, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="REDO")
        self.btnKey5.grid(row=1, column=4, padx=(10,20), pady=20, sticky="nw")

        self.btnKey6 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=60, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="MOV")
        self.btnKey6.grid(row=2, column=0, padx=(10,10), pady=20, sticky="nw")
        self.btnKey7 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=60, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="COPY")
        self.btnKey7.grid(row=2, column=1, padx=(10,10), pady=20, sticky="nw")
        self.btnKey8 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=60, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="PASTE")
        self.btnKey8.grid(row=2, column=2, padx=(10,10), pady=20, sticky="nw")
        self.btnKey8 = customtkinter.CTkButton(self.KeyImgFrame, height=170, width=150, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), corner_radius=70, text="Knob")
        self.btnKey8.grid(row=2, column=3, columnspan=2, rowspan=2, padx=(10,10), pady=20, sticky="nw")

        self.btnKey9 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=60, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="Ctrl")
        self.btnKey9.grid(row=3, column=0, padx=(10,10), pady=20, sticky="nw")
        self.btnKey10 = customtkinter.CTkButton(self.KeyImgFrame, height=60, width=130, fg_color="transparent", border_width=1, border_color=("gray50", "gray30"), text="Shift")
        self.btnKey10.grid(row=3, column=1, columnspan=2, padx=(10,10), pady=20, sticky="nw")

        self.KeyInfoFrame = customtkinter.CTkFrame(self)
        self.KeyInfoFrame.grid(row=2, column=1, padx=20, pady=(20,0), sticky="nw")
        self.Label11 = customtkinter.CTkLabel(self.KeyInfoFrame, text="键位设置", font=FontYaHei)
        self.Label11.grid(row=0, column=0, padx=(20), pady=(5, 0), sticky="w")

        self.Label12 = customtkinter.CTkLabel(self.KeyInfoFrame, text="类型", font=FontYaHei)
        self.Label12.grid(row=1, column=0, padx=(20,0), pady=(5, 0), sticky="w")
        self.comboxKeyType = customtkinter.CTkComboBox(self.KeyInfoFrame, values=["键值", "字符串", "特殊功能"], font=FontYaHei)
        self.comboxKeyType.grid(row=1, column=1, columnspan=2, padx=(20,0), pady=(5, 0), sticky="w")

        self.Label12 = customtkinter.CTkLabel(self.KeyInfoFrame, text="键值", font=FontYaHei)
        self.Label12.grid(row=2, column=0, padx=(20,0), pady=(20, 0), sticky="w")
        self.ChkboxCtrl = customtkinter.CTkCheckBox(master=self.KeyInfoFrame, text=" Ctrl ", font=FontYaHei)
        self.ChkboxCtrl.grid(row=2, column=1, pady=(20, 0), padx=(20,0), sticky="nw")
        self.ChkboxAlt = customtkinter.CTkCheckBox(master=self.KeyInfoFrame, text=" Alt ", font=FontYaHei)
        self.ChkboxAlt.grid(row=3, column=1, pady=(10, 0), padx=(20,0), sticky="nw")
        self.ChkboxShift = customtkinter.CTkCheckBox(master=self.KeyInfoFrame, text=" Shift ", font=FontYaHei)
        self.ChkboxShift.grid(row=4, column=1, pady=(10, 0), padx=(20,0), sticky="nw")
        self.EntryKey = customtkinter.CTkEntry(self.KeyInfoFrame, width=50)
        self.EntryKey.grid(row=2, column=2, pady=(20,0), padx=(20,50), sticky="nw")
        
        self.Label13 = customtkinter.CTkLabel(self.KeyInfoFrame, text="字符串", font=FontYaHei)
        self.Label13.grid(row=5, column=0, padx=(20,0), pady=(20, 0), sticky="w")
        self.EntryKey = customtkinter.CTkEntry(self.KeyInfoFrame, width=200)
        self.EntryKey.grid(row=5, column=1, columnspan=2, pady=(20,0), padx=(20,50), sticky="nw")

        self.Label14 = customtkinter.CTkLabel(self.KeyInfoFrame, text="特殊功能", font=FontYaHei)
        self.Label14.grid(row=6, column=0, padx=(20,0), pady=(20, 20), sticky="w")
        self.EntryKey = customtkinter.CTkEntry(self.KeyInfoFrame, width=200)
        self.EntryKey.grid(row=6, column=1, columnspan=2, pady=(20,20), padx=(20,50), sticky="nw")
        

        



        


app=App()
app.mainloop()
