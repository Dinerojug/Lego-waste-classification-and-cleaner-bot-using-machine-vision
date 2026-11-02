from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, UltrasonicSensor
from pybricks.parameters import Button, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pupremote_hub import PUPRemoteHub
import umath, urandom

#Defining Variables
rand = 1 if urandom.randint(0, 1) else -1
material = 0
#ports
sw= StopWatch()
p = PUPRemoteHub(Port.E)
p.add_command('msg',"repr","repr")
p.add_channel('one_w',to_hub_fmt='bbbb')
p.add_command('num', from_hub_fmt="b", to_hub_fmt="b")
gripper = Motor(Port.A, gears =[36,28])
LeverArm  = Motor(Port.D)
LDB = Motor(Port.F, Direction.COUNTERCLOCKWISE)
RDB = Motor(Port.B)
LDB.control.stall_tolerances(speed=130, time=30)
RDB.control.stall_tolerances(speed=130, time=30)
DS = UltrasonicSensor(Port.C)
hub = PrimeHub()
lastxpos= False
drive = DriveBase(LDB, RDB, wheel_diameter=56, axle_track=130)
hub.display.orientation(Side.LEFT)

print(hub.imu.heading(), "degrees")

label = [ 'metal','aper', 'plastic']
Start = False
mode = 0
runs = 0
detected = 0
dlabel = 0
xpos = 0
hpos = 0
dOBJ = 2000
xychart = [0, 0]
centered = False
GrabbingDist = False
category = [0, 0, 0, 1,2,2]
lastexpos = False 

#Makes Sure Objects of Interest Are WIthin Grabbing Distance
def grabdistance(mytuple, Tagcheck = False):

    global material 
    global centered
    global GrabbingDist
    global detected
    global dlabel
    global xpos
    global hpos
    global dOBJ
    ran= 0
    middle = mytuple[len(mytuple) // 2]
    while DS.distance() not in (mytuple):
        if Tagcheck:
            if not(hpos > 2):
                return False
            turn(xpos*.2 if abs(xpos)> 30 else 0 )
        print(DS.distance())
        Backe = (DS.distance() - middle)
        if abs(Backe) >= 300:
            Backe = 120
        if abs(Backe <= 5):
            #   if not Tagcheck:
                #material = category[dlabel]
            #print(label[material])
            return True

        print('grabcomm',Backe)
        Straight(Backe, ignore=True,)
        if DS.distance() in mytuple:
            if not Tagcheck:
                material = category[dlabel]
            print(label[material])
            return True
        ran += 1
        if ran ==7:
            return False
        #Center(xpos)
    
        detected, dlabel, xpos, hpos = wait_or_detect(200)
    return True
#Updates Movement to an XY Chart
def movementrec(distance, angle):
    global xychart
    xychart[0] = xychart[0] + distance * (umath.cos(umath.radians(angle)))
    xychart[1] = xychart[1] + distance * (umath.sin(umath.radians(angle)))
#Scan for Waste Objects
def scan(angle, distancecheck = False, Tagscan= False):
    global detected
    global dlabel
    global xpos
    global hpos
    global dOBJ
    wait(500)
    turnorder = [1,-2,1]
    step = 0
    for i in turnorder:
        drive.reset(0)
        if detected == 1 and (distancecheck == False): return detected, dlabel, xpos, hpos
        if (DS.distance() > 92) or distancecheck:
            drive.turn(i*angle, wait=False,)
        while distancecheck == True:
                if DS.distance() <= 150:
                    drive.stop()
                    return False
                    break
                if drive.done():
                    if step == 2:
                        return True
                    break
        while not drive.done():
            detected, dlabel, xpos, hpos = p.call("one_w")
            if Tagscan:
                if hpos >2:
                    drive.stop()
                    movementrec(drive.distance(), hub.imu.heading())
                    return True
                else:
                    return False
            if detected == 1: 

                drive.stop()
                movementrec(drive.distance(), hub.imu.heading())
                return detected, dlabel, xpos, hpos
        step+= 1
        detected, dlabel, xpos, hpos  = wait_or_detect(550) 

    detected, dlabel, xpos, hpos  = wait_or_detect(550)     
    return detected, dlabel, xpos, hpos    
#Scans for AprilTags
def aprilsearch():
    global centered
    global detected
    global dlabel
    global xpos
    global hpos
    global dOBJ
    global runs
    xpos = 0
    runs  = 0
    if hpos > 2:
        return True
    for i in range(4):

        drive.reset(0)
        wait(10)
        if DS.distance() >= 80:
            detected, dlabel, xpos, DOBJ, hpos = turn(-90.2,Tagcheck=True)
            if hpos > 2:
                return True 
        else:
            retreat(False,JustBack=True,Tagcheck=True)
            detected, dlabel, xpos, DOBJ, hpos= turn(-90.2,Tagcheck=True)
            if hpos > 2:
                return True 
    return False
#Faces direction of starting position
def returnHome():
    if DS.distance()  < 90:
        retreat(False, JustBack=True)
    Coterminal = hub.imu.heading() % 360
    print(Coterminal, "coterminal")
    hypote = umath.sqrt(xychart[0]**2 + xychart[1]**2)
    HomeAngle = umath.asin(xychart[1]/hypote)
    if xychart[0] > 0:
        turn(180 - Coterminal, True)
    else:
        turn(-Coterminal)
    if xychart[1] > 0:
        turn(HomeAngle - 8.2, False)
    else:
        turn(-HomeAngle + 8.2, False)
        
def Straight(distance, ignore= False, detectnum = 0 ):
    global detected
    global dlabel
    global xpos
    global hpos
    global dOBJ   
    if (DS.distance() > 76) or ignore:
        drive.reset(0)
        drive.straight(distance,then = Stop.BRAKE, wait = False)  # start moving

        detected, dlabel, xpos, hpos = p.call("one_w")

        while not drive.done():
        
            detected, dlabel, xpos, hpos = p.call("one_w")
            if detectnum > 0 and (detected >= detectnum):
                drive.stop()
                movementrec(drive.distance(), hub.imu.heading())
                return detected, dlabel, xpos, dOBJ, hpos

            if (DS.distance() <= 78) and (not ignore):
                drive.stop()
                angle_turned = hub.imu.heading()
                scan(15)
                movementrec(drive.distance(), hub.imu.heading())
                print(xychart)
                dOBJ = DS.distance()
                detected, dlabel, xpos, hpos = wait_or_detect(1000)
                return detected, dlabel, xpos, dOBJ, hpos
                break

            if drive.stalled():
                print('drive stalles')
                drive.stop()
                movementrec(drive.distance(), hub.imu.heading())  
                wait(3000)
                Straight(-150, ignore =True)
                
                detected, dlabel, xpos, DOBJ, hpos = turn(rand * 82.2 , False)
                break
            

    detected, dlabel, xpos, hpos = wait_or_detect(2000)
    dOBJ = DS.distance()
    angle_turned = hub.imu.heading()
    movementrec(drive.distance(), hub.imu.heading())
    print(xychart)
    detected, dlabel, xpos, hpos = p.call("one_w")
    return detected, dlabel, xpos, dOBJ, hpos          
def turn(LorR, ignore =False, detectnum = 0, tagid = False, Tagcheck = False):
    global detected
    global dlabel
    global xpos
    global hpos
    global dOBJ
    detected, dlabel, xpos, hpos = p.call("one_w") 
    drive.reset(0)
    if (DS.distance() > 92) or ignore:
        drive.turn(LorR,then=Stop.BRAKE, wait=False,)

        while not drive.done():
                
            detected, dlabel, xpos, hpos = p.call("one_w")
            if Tagcheck:
                if hpos > 2:
                    drive.stop()
                    movementrec(drive.distance(), hub.imu.heading())
                    turn(xpos*.3 if abs(xpos)> 55 else 0)
                    return detected, dlabel, xpos, dOBJ, hpos
            if tagid == dlabel:
                drive.stop()
                movementrec(drive.distance(), hub.imu.heading())
                return detected, dlabel, xpos, dOBJ, hpos

            if detectnum > 0 and (detected >= detectnum):
                drive.stop()
                movementrec(drive.distance(), hub.imu.heading())
                return detected, dlabel, xpos, dOBJ, hpos
            if (DS.distance() <= 92) and(not ignore):
                drive.stop()
                angle_turned = hub.imu.heading()
                print(angle_turned, "Degrees")
                #print(drive.distance(), "tmm")
                movementrec(drive.distance(), hub.imu.heading())
                dOBJ = DS.distance() 
                detected, dlabel, xpos, hpos = p.call("one_w")
                return detected, dlabel, xpos, dOBJ, hpos
                break
            if drive.stalled():
                drive.stop()
                retreat(False)
       
    
    #print(drive.distance(), "tmm")
    angle_turned = hub.imu.heading()
    print(angle_turned, "degrees")
    movementrec(drive.distance(), hub.imu.heading())
    dOBJ = DS.distance()
    detected, dlabel, xpos, hpos = p.call("one_w")           
    detected, dlabel, xpos, hpos = wait_or_detect(2000)
    return detected, dlabel, xpos, dOBJ, hpos  
def wait_or_detect(ms):
    sw.reset()
    global detected
    global dlabel
    global xpos
    global hpos
    
    while sw.time() < ms:
        
        
        detected, dlabel, xpos, hpos = p.call('one_w')
        if detected == 1:
            return detected, dlabel, xpos, hpos
            break
        wait(10)
    
    return detected, dlabel, xpos, hpos  # full wait completed
def retreat(condition,JustBack=False, Tagcheck = False):
    hub.display.char('R')
    if condition != True:
        global detected
        detected = 0
        
        global GrabbingDist
        GrabbingDist = False
        global centered
        centered = False
        global runs
        runs = 0
        if Tagcheck:
            drive.straight(-200,wait=False,then =Stop.BRAKE)
            if hpos>=2:
                drive.stop()
                movementrec(drive.distance(), hub.imu.heading())
                return True
        Straight(-200, True)
        if not JustBack:
            turn(rand * 180, ignore=True)


        return True
    else:
        return False
#Reorients and approaches waste Target
def TargetMode():
    global material
    global centered
    global GrabbingDist
    global detected
    global dlabel
    global xpos
    global hpos
    global dOBJ
    global runs
    hub.display.char("T")
   
    runs =0
    wait(1000)
    detected, dlabel, xpos, hpos = wait_or_detect(2000)
    while not GrabbingDist:
        
            
        if DS.distance() < 90:
            print(DS.distance(),'mm entered range')
            if grabdistance(range(73,79)):
                GrabbingDist = True
                break
        
        
        
        print(xpos,'xpos')
        if DS.distance() > 150:
            detected, dlabel, xpos, DOBJ, hpos = turn(xpos*.1 if abs(xpos) > 50 else 0)
            material = category[dlabel]
            print(material)
        else:
            detected, dlabel, xpos, DOBJ, hpos = turn(xpos*.05 if abs(xpos) > 50 else 0)
        
        detected, dlabel, xpos, DOBJ, hpos = Straight(300)
        
        runs += 1

        #Center(xpos)
        if DS.distance() <= 100:
            #Center(xpos)
            continue
        detected, dlabel, xpos, hpos = scan(18)
        material = category[dlabel]
        print(material)
        
        if detected == 0: 
            return False
            break
        print(xpos,'xpos')
        if runs == 3:
            GrabbingDist = False
            return False
            break
    if  not GrabbingDist:
        return False
    return True

def safe_run_angle(motor, speed, angle, timeout=1000):
    motor.run_angle(speed, angle, wait=False, then = Stop.HOLD)
    sw.reset()

    while not motor.done():

        if sw.time() > timeout:
            motor.stop()
            print("Motor timeout — stopped safely")
            return False
        wait(10)

    return True
drive.settings(
    straight_speed=135,         # slower forward speed (default is ~200)
    straight_acceleration=90,   # slower ramp up
    turn_rate = 29,    # slower turning
    turn_acceleration =25  # slower turn ramp up
)
#idle Mode
while not Start:
    pressed = hub.buttons.pressed()

    if Button.RIGHT in pressed:
        Start = True

    mode, dlabel, xpos, hpos = wait_or_detect(500)
#Iniates Robot arm and begins Detection mode 
while Start:
    LeverArm.run_until_stalled(110, then=Stop.HOLD) 
    gripper.run_until_stalled(100, then=Stop.HOLD)
    angle = gripper.angle()
     
    mode, dlabel, xpos, hpos = wait_or_detect(2000)
    while mode == 2:
        p.call('num', 0)
        mode, dlabel, xpos, hpos = wait_or_detect(2000)
        
        wait(100)
    #ROAM Mode
    hub.display.char('R')
    runs = 0
    detected = 0
    dlabel = 0
    xpos = 0
    hpos = 0
    dOBJ = 2000
    centered = False
    GrabbingDist = False
    while detected != 1:
        detected, dlabel, xpos, hpos = wait_or_detect(1000)
        if detected == 1: break
        detected, dlabel, xpos, hpos = scan(50)
        if detected == 1: break
        detected, dlabel, xpos, DOBJ, hpos = Straight(500)
        print(xychart)
        if detected == 1: break
        detected, dlabel, xpos, hpos = wait_or_detect(1000)
        detected, dlabel, xpos, hpos = scan(18)
        if detected == 1: break
        rand = 1 if urandom.randint(0, 1) else -1
        detected, dlabel, xpos, DOBJ, hpos = turn(rand * 82.2, detectnum= 1)
        print(xychart)
        if detected == 1: break
        detected, dlabel, xpos, hpos = wait_or_detect(1000)
        if detected == 1: break
        if DS.distance() < 90:
            detected, dlabel, xpos, hpos = wait_or_detect(2000)
            if detected == 1:
                break
            else:
                detected, dlabel, xpos, DOBJ, hpos= Straight(-45, True)
                if detected == True: break
                turn(rand * 90, ignore=True, detectnum = 1)
    print(xychart,"yx")    
    #TARGET Mode
    if not TargetMode(): 
        retreat(False)
        continue
    #GRABBING Mode 
    detected = 0
    runs = 0
    
    while detected == 0:

        detected, dlabel, xpos, hpos = wait_or_detect(1000)
        runs += 1
        if runs == 2:
            break
    runsG = 0
    Grabbed  = False
    print(DS.distance(), "Grabbingdist")
    wait(1000)
    while DS.distance() < 200:
        gripper.run_until_stalled(100, then=Stop.HOLD)
        hub.display.char('G')
        Grabbed  = False
        if runsG >= 3:
            break
        #Arm Grab Heights
        if hpos > 130:

            while True:
                if not safe_run_angle(gripper,120,-45,timeout=4500):
                    wait(200)
                    gripper.run_until_stalled(100, then=Stop.HOLD)
                    wait(500)
                else:
                    break
            LeverArm.run_until_stalled(-120, then=Stop.HOLD)
            wait(1000)
        else:
            while True:

                if not safe_run_angle(gripper,120,-45,timeout=4000):
                    wait(200)
                    gripper.run_until_stalled(100, then=Stop.HOLD)
                    wait(500)
                else:
                    break
            safe_run_angle(LeverArm,120,-87,timeout=4000)
            wait(1000)
        gripper.run_until_stalled(-120, then=Stop.HOLD)
        wait(1000)
        LeverArm.run_until_stalled(150, then=Stop.HOLD)
        wait(1500)
        if not scan(15, distancecheck= True):
            grabdistance(range(73,79))
        if DS.distance() >= 210:
            Grabbed = True
            break
        #Another Grab Attempt
        else:
            print(DS.distance())
           
            if not TargetMode():
                break
            runsG += 1
            gripper.run_until_stalled(100, then=Stop.HOLD)
            safe_run_angle(gripper,-90,45, timeout= 1000) 
            if gripper.stalled():
                gripper.stop()
        if runsG >= 3:
            Grabbed = False
            break
    if Grabbed == False:
        retreat(Grabbed)
        continue
            

    
    

    #Dropoff Mode
    hub.display.char(str(label[material][0]))
    hpos = 0
    mode, dlabel, xpos, hpos = wait_or_detect(500)
    while mode  != 2:
        p.call('num', 2)
        
        mode, dlabel, xpos, hpos = wait_or_detect(2000)
        wait(100)

    hpos = 0
    print('Material:',material)
    while True:
        #Display Object Type
        hub.display.char(str(label[material][0]))
        
        if (aprilsearch() ==True) or (hpos > 2):
            #Apriltag found no need for xychart
            if dlabel == material:
                while ( hpos > 2) :
                    detected, dlabel, xpos, DOBJ, hpos = turn((xpos)*.5 if abs(xpos) > 40 else 0 )
                    if not grabdistance(range(145, 300),Tagcheck=True): break 
                    print("at", material)
                    if (abs(xpos) <= 60) and (dlabel == material):
                        lastxpos = True
                        break
                    else:
                        lastxpos = False
                        break 
                    #detected, dlabel, xpos, DOBJ, hpos = Straight(150)
    
                if not lastxpos:
                    print('not bin')
                    retreat(False, JustBack=True, Tagcheck = True)
                    continue
                #Drop Distance
                else:
                    print('Dropping')
                    wait(1000)
                    grabdistance(range(75,81))
                    safe_run_angle(LeverArm,120,-11,timeout=1000)
                    gripper.run_until_stalled(120, then=Stop.HOLD)
                    wait(700)
                    gripper.run_until_stalled(-120, then=Stop.HOLD)
                    retreat(False)
                    break
            else:
                if dlabel > material:
                    detected, dlabel, xpos, DOBJ, hpos = turn(-30)
                    if DS.distance() < 200:
                        detected, dlabel, xpos, DOBJ, hpos = Straight(-150)
                    else:
                        if dlabel > material:
                            detected, dlabel, xpos, DOBJ, hpos = turn(-30)
                            detected, dlabel, xpos, DOBJ, hpos = Straight(150)
                else:
                    detected, dlabel, xpos, DOBJ, hpos = turn(30,tagid= material)
                    if DS.distance() < 200:
                        detected, dlabel, xpos, DOBJ, hpos = Straight(-150)
                    else:
                         if dlabel > material:
                            detected, dlabel, xpos, DOBJ, hpos = turn(30)
                            detected, dlabel, xpos, DOBJ, hpos = Straight(150)
        else: 
            returnHome()
            print(DS.distance(),'trying to return home')
            if (scan(20,Tagscan = True)):
                print('tag not found returning home')
                detected, dlabel, xpos, DOBJ, hpos = Straight(200)
            
        




