import pystray
import PIL.Image
import subprocess

global SpaceKatDriver
bStarted = False

image = PIL.Image.open("icon.png")

def on_clicked(icon, item):
    print ("Hello")
    
def miStartDriver(icon, item):
    global SpaceKatDriver
    global bStarted
    if not bStarted:
        SpaceKatDriver = subprocess.Popen(['python', 'SpaceKat Driver.py'])
        bStarted = True
        print ("Start Driver")
    else:
        print ("The driver has been started.")

def miStopDriver(icon, item):
    global SpaceKatDriver
    global bStarted
    if bStarted:
        SpaceKatDriver.terminate()
        print ("Stop Driver")
        bStarted = False
    else:
        print ("The driver has been stopped.")

def miSetting(icon, item):
    subprocess.Popen(['python', 'SpaceKatSetting.py'])

def miExit(icon, item):
    if bStarted:
        print ("SpaceKat Driver is running, please stop the driver first.")
    else:
        print ("Exit")
        icon.stop()

icon = pystray.Icon("SpaceKat", image, menu=pystray.Menu(
    pystray.MenuItem("Start Driver", miStartDriver),
    pystray.MenuItem("Stop Driver", miStopDriver),
    pystray.MenuItem("SpaceKat Settings", miSetting),
    pystray.MenuItem("Exit", miExit)
))

icon.run()