import os
import time
import sys
import struct
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe

mqtt_client = mqtt.Client("ble_tx")
prev_xPos = [0, 0, 0]
prev_yPos = [0, 0, 0]
currPos = [(0, 0), (0, 0), (0, 0)]
targetPos = [(0, 0), (0, 0), (0, 0)]
kobuki_id = [0]
prevDeg = [0, 0, 0]
currDeg = [0, 0, 0]
targetDeg = [0, 0, 0]
message_queued = False
kobuki_moving = False
run = 1
gesture_id = 0
current_gesture = "None"
master = [0, 0, 0]

def float_to_hex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0]).lstrip("0x")

def ble_tx(kobuki_id, tx_id, command, currPos, targetPos, currDeg, targetDeg):
    global prev_xPos, prev_yPos, prevDeg, message_queued

    if(abs(currPos[0]-targetPos[0])>0.1 or abs(currPos[1]-targetPos[1])>0.1 or abs(currDeg-targetDeg)>5 or command!=2):
        #   If current position is different enough from target position, else ignore the advertisement
        message_queued = False

        if(targetPos != (0,0)):
            initialPos = (float_to_hex(targetPos[0]), float_to_hex(targetPos[1]))
            xPos_initial = " ".join(str(initialPos[0])[i:i+2] for i in range(0, len(initialPos[0]), 2))
            yPos_initial = " ".join(str(initialPos[1])[i:i+2] for i in range(0, len(initialPos[1]), 2))
        if(currPos != (0,0)):
            currentPos = (float_to_hex(currPos[0]), float_to_hex(currPos[1]))
            xPos_current = " ".join(str(currentPos[0])[i:i+2] for i in range(0, len(currentPos[0]), 2))
            yPos_current = " ".join(str(currentPos[1])[i:i+2] for i in range(0, len(currentPos[1]), 2))
        if(currPos[0] == 0):
            xPos_current = "0 0 0 0"
        if(currPos[1] == 0):
            yPos_current = "0 0 0 0"
        if(targetPos[0] == 0):
            xPos_initial = "0 0 0 0"
        if(targetPos[1] == 0):
            yPos_initial = "0 0 0 0"

        startCommand = "sudo hcitool -i hci0 cmd 0x08 0x0008 1e "
        startCommand += "FF FF "+hex(kobuki_id)+" "+hex(tx_id)+" "+hex(command)+" "+xPos_current+" "+yPos_current+" "+hex(currDeg/2)+" "
        startCommand += xPos_initial+" "+yPos_initial+" "+hex(targetDeg/2)+" FF 48 d2 b0 60 d0 f5 a7 10 96 e0 00 00 00 00 c5 00 00 00 00 00 00"
        os.system(startCommand)                                                                         #Sets advertising data
        os.system("sudo hcitool -i hci0 cmd 0x08 0x0006 A0 00 A0 00 03 00 00 00 00 00 00 00 00 07 00")  #Sets advertising interval to 100ms
        os.system("sudo hcitool -i hci0 cmd 0x08 0x000a 01")                                            #Turns on bluetooth advertising
        #os.system("sudo hciconfig hci0 leadv 0")
        time.sleep(0.5)
        os.system("sudo hciconfig hci0 noleadv")    #Turns off bluetooth advertising
    return tx_id + 1

def on_message(client, userdata, message):
    global currPos, currDeg, kobuki_id, message_queued, prev_xPos, prev_yPos, kobuki_moving, prevDeg, gesture_id, current_gesture, master
    if (message.topic == "kobuki"):
        #print "Position Received"
        payload = str(message.payload.decode("utf-8"))
        data = payload.split(';')
        temp_id = int(data[0])
        if not(temp_id in kobuki_id):
            kobuki_id.append(temp_id)
        xPos = float(data[1])
        yPos = float(data[2])
        deg = int(float(data[3]))
        count = 0
        if (abs(xPos-prev_xPos[temp_id])>0.05 or abs(yPos-prev_yPos[temp_id])>0.05 or abs(deg-prevDeg[temp_id])>7):     #and abs(deg-prevDeg[temp_id])<80
            prev_xPos[temp_id] = xPos
            prev_yPos[temp_id] = yPos
            prevDeg[temp_id] = deg
            currPos[temp_id] = (xPos, yPos)
            currDeg[temp_id] = deg
            if(abs(xPos)>0.65 or abs(yPos)>0.65):         #If the kobuki is leaving the camera's field of vision
                master[temp_id] = 3
            message_queued = True
            kobuki_moving = True
            print "Kobuki ID: ", kobuki_id[temp_id], "Current Position: ", currPos[temp_id], "Current Angle: ", currDeg[temp_id]
        else:
            #   If the same message is repeated 5 times, the kobuki is not moving
            #print "Message repeated"
            if count >= 5:
                kobuki_moving = False
                count = 0
            else:
                kobuki_moving = True
                count += 1
    if (message.topic == "gesture"):
        """
            master = 0  ----->  Kobuki OFF
            master = 1  ----->  Kobuki RANDOM
            master = 2  ----->  Kobuki GO TO TARGET LOCATION
        """
        #print "Gesture Received"
        current_gesture = str(message.payload.decode("utf-8"))
        if current_gesture == "ZoomIn":
            master = [2, 2, 2]
            assign_reference_positions()
        elif current_gesture == "ZoomOut":
            master = [1, 1, 1]
        elif current_gesture == "WaveRight" or current_gesture == "WaveLeft":
            master = [0, 0, 0]
        elif current_gesture == "SwipeRight":
            master = [2, 2, 2]
            translation_to_y_positive()
        elif current_gesture == "SwipeLeft":
            master = [2, 2, 2]
            translation_to_y_negative()
        elif current_gesture == "Turn0":
            master = [2, 2, 2]            
            targetPos[0] = currPos[0]     
            targetPos[1] = currPos[1]     
            targetPos[2] = currPos[2]
            targetDeg[0] = 0
            targetDeg[1] = 0
            targetDeg[2] = 0
        elif current_gesture == "Turn180":
            master = [2, 2, 2]            
            targetPos[0] = currPos[0]     
            targetPos[1] = currPos[1]     
            targetPos[2] = currPos[2]
            targetDeg[0] = 180
            targetDeg[1] = 180
            targetDeg[2] = 180
        else:
            gesture_id -= 1
        gesture_id += 1

def order_by_x():
    a=0
    b=1
    c=2
    if currPos[0][0]>currPos[1][0]:
        if currPos[2][0]>currPos[0][0]:
            a=2
            b=0
            c=1
        elif currPos[2][0]>currPos[1][0]:
            a=0
            b=2
            c=1
        else:
            a=0
            b=1
            c=2
    else:
        if currPos[2][0]>currPos[1][0]:
            a=2
            b=1
            c=0
        elif currPos[2][0]>currPos[0][0]:
            a=1
            b=2
            c=0
        else:
            a=1
            b=0
            c=2
    return (a,b,c)
    
def assign_reference_positions():
    a,b,c = order_by_x()
    targetPos[a] = (0.5, 0)
    targetPos[b] = (0, 0)
    targetPos[c] = (-0.5, 0)
    targetDeg[0] = 90
    targetDeg[1] = 90
    targetDeg[2] = 90

def translation_to_y_positive():
    a,b,c = order_by_x()
    targetPos[a] = (0.5, 0.3)
    targetPos[b] = (0, 0.3)
    targetPos[c] = (-0.5, 0.3)
    targetDeg[0] = 90
    targetDeg[1] = 90
    targetDeg[2] = 90
def translation_to_y_negative():
    a,b,c = order_by_x()
    targetPos[a] = (0.5, -0.3)
    targetPos[b] = (0, -0.3)
    targetPos[c] = (-0.5, -0.3)
    targetDeg[0] = 90
    targetDeg[1] = 90
    targetDeg[2] = 90
        

def main():
    global prev_xPos, prev_yPos, run, currPos, kobuki_id, currDeg, targetDeg, gesture_id, current_gesture, master, start_time, stop_time
    os.system("sudo hcitool dev")
    print("Creating MQTT mqtt_client")
    mqtt_client.on_message = on_message
    mqtt_client.connect("localhost")    #128.32.44.126
    mqtt_client.loop_start()
    mqtt_client.subscribe("kobuki")
    mqtt_client.subscribe("gesture")
    last_gesture_id = 0
    tx_id = 1
    while(1):
        if (gesture_id > last_gesture_id or 3 in master):       #If there is a new gesture, or the kobuki is leaving the camera's field of vision
            #HERE WE TAKE ACTION DEPENDING ON THE GESTURE
            #print(current_gesture)
            if(message_queued):
                for i in kobuki_id:
                    if master[i] == 2:
                        #STOPS THE KOBUKI FOR A LITTLE WHILE BEFORE TELLING IT TO GO TO TARGET LOCATION
                        tx_id = ble_tx(i, tx_id, 0, currPos[i], targetPos[i], currDeg[i], targetDeg[i])
                    mqtt_client.unsubscribe("kobuki")
                    print("")
                    print("Transmission ID ", tx_id)
                    print("Command: ", master)
                    print("Target Position: ", targetPos[i])
                    print("Current Position: ", currPos[i])
                    print("Current Angle: ", currDeg[i])
                    print("")
                    #Sets the target location and turns on bluetooth advertising
                    tx_id = ble_tx(i, tx_id, master[i], currPos[i], targetPos[i], currDeg[i], targetDeg[i])
                    if master[i] == 3:
                        master[i] = 1
                    mqtt_client.subscribe("kobuki")
            last_gesture_id = gesture_id

if __name__ == '__main__':
	main()
	sys.exit(0)
