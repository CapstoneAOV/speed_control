#!/usr/bin/python

import serial
import time
import rospy
import std_msgs

state = ''

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
    

def callback_speed(data):
    print("Callback speed")
    pub = rospy.Publisher("Arduino_response", std_msgs.msg.String, queue_size=1)
    rospy.Rate(10)

    speed = data.data
    print("speed is {0}".format(speed))
    if(speed > 5 or speed < -5):      # conversion of speed to volts?
        response = "FAIL: invalid parameters"       #need to publish to error topic, error code for direction switching

    elif(speed >-0.2 and speed < 0.1):
        print("turning on brakes")
        response = writeout(str(0.0) + "xxx")
        braking_pub = rospy.Publisher('Braking', std_msgs.msg.Bool, queue_size=1)
        print(braking_pub)
        braking_pub.publish(True)
    
    else:
        volts = round(speed,1)
        global state

        if(volts > 0):

            if(state == 'reverse'):
                response = writeout('0.0000')       #disconnect and wait, to prevent shorting of forward/reverse
                state = 'forward'
                time.sleep(0.01)
            response = writeout(str(volts) + "010")
            
        elif(volts < 0):
            if(state == 'forward'):
                response = writeout('0.0000')           
                state = 'reverse'
                time.sleep(0.01)
            response = writeout(str(volts) + "001")
            
    
    pub.publish(response)        

def listener():
    print("entered listener")
    # rospy.Subscriber("Direction", std_msgs.msg.String, callback_direction)      #can change to int
    rospy.Subscriber("Speed", std_msgs.msg.Float32, callback_speed)
    rospy.spin()

def main():
    print("main entered")
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