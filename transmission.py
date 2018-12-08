import os
import time
import sys
import struct
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe

mqtt_client = mqtt.Client("ble_tx")
kinnect_mqtt = mqtt.Client("ble_tx")
prev_xPos = [0, 0]
prev_yPos = [0, 0]
currPos = [(0, 0), (0, 0)]
targetPos = [(0, 0), (0, 0.5), (-0.5, -0.5), (0.5, -0.5)]
kobuki_id = [0]
prevDeg = [0, 0]
currDeg = [0, 0]
targetDeg = [0, 0]
message_queued = False
kobuki_moving = False
run = 1

def float_to_hex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0]).lstrip("0x")

def ble_tx(kobuki_id, tx_id, master, currPos, targetPos, currDeg, targetDeg):
    global prev_xPos, prev_yPos, prevDeg, message_queued

    message_queued = False
    if(abs(currPos[0]-targetPos[0])>0.1 or abs(currPos[1]-targetPos[1])>0.1 or abs(currDeg-targetDeg)>5):

        #prev_xPos[kobuki_id] = currPos[0]
        #prev_yPos[kobuki_id] = currPos[1]
        #prevDeg[kobuki_id] = currDeg

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
        startCommand += "FF FF "+hex(kobuki_id)+" "+hex(tx_id)+" "+hex(master)+" "+xPos_current+" "+yPos_current+" "+hex(currDeg)+" "
        startCommand += xPos_initial+" "+yPos_initial+" "+hex(targetDeg)+" FF 48 d2 b0 60 d0 f5 a7 10 96 e0 00 00 00 00 c5 00 00 00 00 00 00"
        os.system(startCommand)
        os.system("sudo hciconfig hci0 leadv 0")
        time.sleep(1)
        print "Transmission complete"
        run = 0
    return tx_id + 1

def getKobukiPos(kobuki_id):
    if kobuki_id < len(targetPos)-1:
        pos = targetPos[kobuki_id]
        return pos
    else:
        return (0,0)

def on_message(client, userdata, message):
    global currPos, currDeg, kobuki_id, message_queued, prev_xPos, prev_yPos, kobuki_moving, prevDeg
    #print "Message received"
    if (message.topic == "kobuki"):
        payload = str(message.payload.decode("utf-8"))
        data = payload.split(';')
        temp_id = int(data[0])
        if not(temp_id in kobuki_id):
            kobuki_id.append(temp_id)
        xPos = float(data[1])
        yPos = float(data[2])
        deg = int(float(data[3]))
        count = 0
        #print(prev_xPos[temp_id])
        if (abs(xPos-prev_xPos[temp_id])>0.05 or abs(yPos-prev_yPos[temp_id])>0.05 or abs(deg-prevDeg[temp_id])>7 and abs(deg-prevDeg[temp_id])<80):
            prev_xPos[temp_id] = xPos
            prev_yPos[temp_id] = yPos
            prevDeg[temp_id] = deg
            currPos[temp_id] = (xPos, yPos)
            currDeg[temp_id] = deg
            message_queued = True
            kobuki_moving = True
            #mqtt_client.unsubscribe("kobuki")
            print "Kobuki ID: ", kobuki_id[temp_id], "Current Position: ", currPos[temp_id], "Current Angle: ", currDeg[temp_id]
        else:
            #print "Message repeated"
            if count >= 5:
                kobuki_moving = False
                count = 0
            else:
                kobuki_moving = True
                count += 1


def main():
    global prev_xPos, prev_yPos, run, currPos, kobuki_id, currDeg, targetDeg
    #message_queued = False
    os.system("sudo hcitool dev")
    print("Creating MQTT mqtt_client")
    mqtt_client.on_message = on_message
    mqtt_client.connect("localhost")    #128.32.44.126
    mqtt_client.loop_start()
    mqtt_client.subscribe("kobuki")
    tx_id = 1
    while(1):
        time.sleep(2)
        if(message_queued):
            master = input("Command: ")
            print("")
            run = 1
            for i in kobuki_id:
                mqtt_client.unsubscribe("kobuki")
                print("")
                print("Transmission ID ", tx_id)
                print("Target Position: ", targetPos[i])
                print("Current Position: ", currPos[i])
                print("")
                tx_id = ble_tx(i, tx_id, master, currPos[i], targetPos[i], currDeg[i], targetDeg[i])
                mqtt_client.subscribe("kobuki")
            time.sleep(5)
        else:
            run = 0
        os.system("sudo hciconfig hci0 noleadv")

if __name__ == '__main__':
	main()
	sys.exit(0)
