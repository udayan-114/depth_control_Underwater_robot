
### code is developed by Udayan Pandey , IIT Patna



import smbus
import time
import RPi.GPIO as GPIO
from Tkinter import*
from time import sleep
import tkSimpleDialog
import  tkMessageBox




root = Tk()
w = Label(root,text="my program")
w.pack()
tkMessageBox.showinfo("welcome", "Underwater buoyancy contol robot")
distance= tkSimpleDialog.askstring("Depth","what is desire distance")

GPIO.setmode(GPIO.BCM)
motor1=20
led1= 21
led2 = 12

GPIO.setup(motor1,GPIO.OUT)
GPIO.setup(led1,GPIO.OUT)
GPIO.setup(led2,GPIO.OUT)
p = GPIO.PWM(motor1,50)
p.start(0)

#distance = raw_input('Welcomea, Please enter depth distance 0\n')

while True :

# Get I2C bus
    bus = smbus.SMBus(1)

# MS5803_02BA address, 0x76(118)
#		0x1E(30)	Reset command
    bus.write_byte(0x76, 0x1E)

    time.sleep(0.01)

# Read 12 bytes of calibration data
# Read pressure sensitivity
    data = bus.read_i2c_block_data(0x76, 0xA2, 2)
    C1 = data[0] * 256 + data[1]

# Read pressure offset
    data = bus.read_i2c_block_data(0x76, 0xA4, 2)
    C2 = data[0] * 256 + data[1]

# Read temperature coefficient of pressure sensitivity
    data = bus.read_i2c_block_data(0x76, 0xA6, 2)
    C3 = data[0] * 256 + data[1]

# Read temperature coefficient of pressure offset
    data = bus.read_i2c_block_data(0x76, 0xA8, 2)
    C4 = data[0] * 256 + data[1]

# Read reference temperature
    data = bus.read_i2c_block_data(0x76, 0xAA, 2)
    C5 = data[0] * 256 + data[1]

# Read temperature coefficient of the temperature
    data = bus.read_i2c_block_data(0x76, 0xAC, 2)
    C6 = data[0] * 256 + data[1]

# MS5803_02BA address, 0x76(118)
#		0x40(64)	Pressure conversion(OSR = 256) command
    bus.write_byte(0x76, 0x40)

    time.sleep(0.01)

# Read digital pressure value
# Read data back from 0x00(0), 3 bytes
# D1 MSB2, D1 MSB1, D1 LSB
    value = bus.read_i2c_block_data(0x76, 0x00, 3)
    D1 = value[0] * 65536 + value[1] * 256 + value[2]

# MS5803_02BA address, 0x76(118)
#		0x50(64)	Temperature conversion(OSR = 256) command
    bus.write_byte(0x76, 0x50)

    time.sleep(0.01)

# Read digital temperature value
# Read data back from 0x00(0), 3 bytes
# D2 MSB2, D2 MSB1, D2 LSB
    value = bus.read_i2c_block_data(0x76, 0x00, 3)
    D2 = value[0] * 65536 + value[1] * 256 + value[2]

    dT = D2 - C5 * 256
    TEMP = 2000 + dT * C6 / 8388608
    OFF = C2 * 65536 + (C4 * dT) / 128
    SENS = C1 * 32768 + (C3 * dT ) / 256
    T2 = 0
    OFF2 = 0
    SENS2 = 0

    if TEMP >= 2000 :
    
            T2 = 0
            OFF2 = 0
            SENS2 = 0
            if TEMP > 4500:
                SENS2 = SENS2-((TEMP-4500)*(TEMP-4500))/8
    elif TEMP < 2000 :
            T2 = (dT * dT) / 2147483648
            OFF2= 3 * ((TEMP - 2000) * (TEMP - 2000))
            SENS2= 7 * ((TEMP - 2000) * (TEMP - 2000)) / 8 
            if TEMP < -1500 :
        
                    SENS2 = SENS2 + 2 * ((TEMP + 1500) * (TEMP +1500))

    TEMP = TEMP - T2
    OFF = OFF - OFF2
    SENS = SENS - SENS2
    pressure = ((((D1 * SENS) / 2097152) - OFF) / 32768.0) /10
    cTemp = TEMP / 100.0
    fTemp = cTemp * 1.8 + 32

# Output data to screen
    print ("Pressure : %.2f mbar" %pressure)
    #print ("Temperature in Celsius : %.2f C" %cTemp)
    #print ("Temperature in Fahrenheit : %.2f F" %fTemp)
    #distance= distance+pressure

    error = float(pressure)-float(distance)
    kp=2
    fix_duty_cycle = 30
    #fix_dutycycle is the some numeric terms.
    \
    pwm1= kp*error+fix_duty_cycle
    pwm2 = fix_duty_cycle
    
    if float(distance) < float(pressure):

        p.ChangeDutyCycle(pwm1)
        GPIO.output(led1,True)
        

        #GPIO.output(motor1,True)
        
       
    elif float(distance) > float(pressure):
        
        p.ChangeDutyCycle(pwm2)
        GPIO.output(led2,True)
        
        #GPIO.output(motor1,False)
        time.sleep(0.01)

GPIO.clean()
