#!/usr/bin/python

#from ackermann_msgs.msg import AckermannDriveStamped
import serial
import time
import rospy
import std_msgs     #dont import entire library
from datetime import datetime

state = ''

def writeout(cmd):
    print(cmd)
    i=0
    write_out = str(cmd)
    while(i<6):                   #wait for response from arduino, for up to 200 commands sent
        i += 1
        ard.write(write_out.encode())
        time.sleep(0.02) 
        ard.flush()
        time.sleep(0.02)
        response = ard.read(ard.inWaiting())    # read all characters in buffer
        if(response == write_out):                  #if response received, exit loop
            print(response + " success")
            break

    if(i>=50):
        response = "FAIL: Command: {0} not sent to Arduino".format(cmd)   #publish to failure topic. consider using error code

    #print("{0} commands sent before {1}".format(i, write_out))        #debugging, take out
    #print("command executed at time {0}".format(datetime.now()))
    #return response
    return datetime.now()
    
def volts_to_speed(volts):
    speed = volts               #60 revs in 53.63s with 1V power, 120 revs/40.99s 2V
    return speed

def callback_speed(data):
    print("Callback speed")
    pub = rospy.Publisher("Arduino_response", std_msgs.msg.String, queue_size=1)    #remove
    rospy.Rate(10)

    #speed = data.speed
    speed = data.data
    time_now = datetime.now()
    #print("data received at time {0}".format(time_now))
    #print("speed sent {0}".format(speed))
    #print("speed is {0}".format(speed))
    if(speed > 3 or speed < -3):      # conversion of speed to volts?
        response = "FAIL: invalid parameters"       #need to publish to error topic, error code for direction switching

    elif(speed >-0.2 and speed < 0.1):
        #print("turning on brakes")
        response = writeout(str(0.0) + "xxx")
        braking_pub = rospy.Publisher('Braking', std_msgs.msg.Bool, queue_size=1)
        braking_pub.publish(True)
    
    else:
        volts = round(speed,1)
        global state
        print("speed is {0}".format(volts))

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
            response = writeout(str(volts)[1:] + "001")

    print("time taken is under {0}".format(response - time_now))
    
            
    
    #pub.publish(response)        

def listener():
    #print("entered listener")
    #rospy.Subscriber("rbcar_robot_control/command", AckermannDriveStamped, callback_speed)
    rospy.Subscriber("Speed", std_msgs.msg.Float32, callback_speed)
    rospy.spin()

def main():
    #print("main entered")
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

    listener()

if __name__ == '__main__':
    main()