#!/usr/bin/python

import serial
import time
import rospy
import std_msgs

def writeout(cmd):
    print(cmd)
    i=0
    write_out = str(cmd)
    while(i<50):                   #wait for response from arduino, for up to 200 commands sent
        i += 1
        ard.write(write_out.encode())
        time.sleep(0.05) 
        ard.flush()
        time.sleep(0.05)
        response = ard.read(ard.inWaiting())    # read all characters in buffer
        if(response == write_out):                  #if response received, exit loop
            print(response + " success")
            break

    if(i>=50):
        response = "FAIL: Command: {0} not sent to Arduino".format(cmd)   #publish to failure topic. consider using error code

    print("{0} commands sent before {1}".format(i, write_out))        #debugging, take out
    
    return response
    

def callback_direction(data):
    print("callback direction")
    pub = rospy.Publisher("Arduino_response", std_msgs.msg.String, queue_size=10)
    rospy.Rate(10)

    cmd = data.data
    if(cmd == "forward"):
        writeout("0.0000")              #disconnect and wait, to prevent shorting of forward/reverse
        time.sleep(0.1)
        response = writeout("0.0010")
    elif(cmd == "reverse"):
        writeout("0.0000")              #consider adding state variable to monitor forward/reverse?
        time.sleep(0.1)
        response = writeout("0.0001")       #turn velocity to zero when changing direction?
    else:
        response = "Invalid Command"      #also need to publish to error topic, error code for direction switching
    pub.publish(response)

def callback_speed(data):
    print("Callback speed")
    pub = rospy.Publisher("Arduino_response", std_msgs.msg.String, queue_size=10)
    rospy.Rate(10)

    speed = data.data
    if(speed > 5 or speed < 0):      # conversion of speed to volts?
        response = "FAIL: invalid parameters"
    
    else:
        volts = round(speed,1)      
        response = writeout(str(volts) + "xxx")
    
    pub.publish(response)        

def listener():
    print("entered listener")
    rospy.Subscriber("Direction", std_msgs.msg.String, callback_direction)      #can change to int
    rospy.Subscriber("Speed", std_msgs.msg.Float32, callback_speed)
    rospy.spin()

def main():
    rospy.init_node('Relay_Speed_Control', anonymous=True)
    port = rospy.get_param("~port")
    baudrate = int(rospy.get_param("~baudrate"))
    timeout = int(rospy.get_param("~timeout"))

    print(port, baudrate, timeout)
    global ard
    ard = serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=timeout
    )
    time.sleep(2) # wait for Arduino

    listener()

if __name__ == '__main__':
    main()