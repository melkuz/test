
#############################################################################
# Filename    : avion.py
# Description : Projet Avion
# Author      : Alberto Oviedo
# modification: 2022/04/13
########################################################################
from tkinter.tix import Y_REGION
import RPi.GPIO as GPIO
import time
from ADCDevice import *
from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD
import Keypad

#marges du servomoteur
OFFSE_DUTY = 0.5
SERVO_MIN_DUTY = 2.5+OFFSE_DUTY #define pulse duty cycle for minimum angle of servo
SERVO_MAX_DUTY = 12.5+OFFSE_DUTY
 


# definition des pins
RLedPin = 40
YLedPin = 38
GLedPin = 36
motorInput1 = 13
motorInput2 = 11
motorEnablePin =15
joystickPin = 22
pin_Servo = 12
interrupteur_pin=16


destination = ""
interrupteur =False
aeroports= {
    101:"YUL Montreal",
    111:"ATL Atlanta",
    222:"HND Tokyo",
    764:"LHR London",
    492:"CAN Baiyun",
    174:"CDG Paris",
    523:"AMS Amsterdam"

}


ROWS = 4        # number of rows of the Keypad
COLS = 4        #number of columns of the Keypad
keys =  [ '1','2','3','A',    #key code
        '4','5','6','B',
        '7','8','9','C',
        '*','0','#','D']

        
rowsPins = [37,35,33,31]        #connect to the row pinouts of the keypad
colsPins = [32,18,29,7]

#[37,35,33,3311] 
#[32,18,16,8]3
GPIO.setwarnings(False)
adc = ADCDevice() # Define an ADCDevice class object
def setup():
    global adc
    global destination
    global interrupteur
    interrupteur = False    
    destination = ""
    if(adc.detectI2C(0x48)): # Detect the pcf8591.
        adc = PCF8591()
    elif(adc.detectI2C(0x4b)): # Detect the ads7830
        adc = ADS7830()
    else:
        print("No correct I2C address found, \n"
        "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
        "Program Exit. \n")
        exit(-1)


    
    GPIO.setmode(GPIO.BOARD)  
    GPIO.setup(RLedPin, GPIO.OUT)   #Setup des LEDS
    GPIO.output(RLedPin, GPIO.HIGH)
    GPIO.setup(YLedPin, GPIO.OUT)   
    GPIO.output(YLedPin, GPIO.HIGH)
    GPIO.setup(GLedPin, GPIO.OUT)   
    GPIO.output(GLedPin, GPIO.HIGH)
    GPIO.setup(interrupteur_pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(joystickPin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
   
    
    global moteur
    global servoMoteur
    global currentstate
     
    GPIO.setup(motorInput1,GPIO.OUT)   # set pins to OUTPUT mode
    GPIO.setup(motorInput2,GPIO.OUT)
    GPIO.setup(motorEnablePin,GPIO.OUT)
    GPIO.setup(joystickPin,GPIO.IN,GPIO.PUD_UP)
    GPIO.setup(pin_Servo, GPIO.OUT)   # Set pin_Servo to OUTPUT mode
    GPIO.output(pin_Servo, GPIO.LOW) 

    mcp.output(3,1)     # turn on LCD backlight
    lcd.begin(16,2)
        
    moteur = GPIO.PWM(motorEnablePin,1000) # creat PWM and set Frequence to 1KHz
    servoMoteur = GPIO.PWM(pin_Servo,50)
    servoMoteur.start(1)
    moteur.start(0)

# mapNUM function: map the value from a range of mapping to another range.
def mapNUM(value,fromLow,fromHigh,toLow,toHigh):
    return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

def servoWrite(angle):
      
    servoMoteur.ChangeDutyCycle(mapNUM(angle,0,180,SERVO_MIN_DUTY,SERVO_MAX_DUTY))


def pourcentageMoteur(valeurADC):
    temp = 0
    if valeurADC == 128:
        temp =0
    if valeurADC < 128:
        temp = 0-(100 -((100 * valeurADC)/128))
    if valeurADC > 128: 
        temp =   ((valeurADC - 128)/ 127) * 100

    return int(temp)
	
# motor function: determine the direction and speed of the motor according to the input ADC value input
def motor(ADC):
    value = ADC -128
    if (value > 0):  # make motor turn forward
        GPIO.output(motorInput1,GPIO.HIGH)  # motorInput1 output HIHG level
        GPIO.output(motorInput2,GPIO.LOW)   # motorInput2 output LOW level
        print ('Turn Forward...')
    elif (value < 0): # make motor turn backward
        GPIO.output(motorInput1,GPIO.LOW)
        GPIO.output(motorInput2,GPIO.HIGH)
        print ('Turn Backward...')
    else :
        GPIO.output(motorInput1,GPIO.LOW)
        GPIO.output(motorInput2,GPIO.LOW)
        print ('Motor Stop...')
    moteur.start(mapNUM(abs(value),0,128,0,100))
    #print ('The PWM duty cycle is %d%%\n'%(abs(value)*100/127))   # print PMW duty cycle.

def defAngle(x):
    temp = x*0.71
    if (temp >= 84 and temp <=90):
        temp =90
    if(temp<0):
        temp = 0
    if(temp>180):
        temp =180  
    return int(temp)
    
def lcdOutputE3(motor,angle,destination):
   
   lcd.setCursor(0,0)  # set cursor position
   lcd.message(str(motor) + "," + str(angle) )
   lcd.message("               ")
   lcd.setCursor(0,1)
   lcd.message(str(destination))
   lcd.message("               ")
  
   
PCF8574_address = 0x27  # I2C address of the PCF8574 chip.
PCF8574A_address = 0x3F  # I2C address of the PCF8574A chip.
# Create PCF8574 GPIO adapter.
try:
    mcp = PCF8574_GPIO(PCF8574_address)
except:
    try:
        mcp = PCF8574_GPIO(PCF8574A_address)
    except:
        print ('I2C Address Error !')
        exit(1)
# Create LCD, passing in MCP GPIO adapter.
lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp)

def destroy():
    moteur.stop()  # stop PWM
    servoMoteur.stop()
    GPIO.cleanup()

def lcdOutput(message):
    lcd.message(str(message))
    lcd.message("         ")

def findDestination(code):
    if code in aeroports.keys():
        return True
    else:
        return False
    
def E1():
    GPIO.output(YLedPin, GPIO.LOW)
    GPIO.output(GLedPin, GPIO.LOW)
    lcdOutput("Scannez carte")
    time.sleep(2)

def E2():
    
    global destination
    global currentstate
    global interrupteur
    interrupteur = False
    lcd.clear()
    GPIO.output(YLedPin, GPIO.HIGH)
    GPIO.output(RLedPin, GPIO.LOW)
    keypad = Keypad.Keypad(keys,rowsPins,colsPins,ROWS,COLS)
    keypad.setDebounceTime(50)
    key = keypad.getKey()
    destinationTrouve = False
    while  destinationTrouve== False:
        code =""
        lcd.clear()
        for x in range (3):
            while key == keypad.NULL:
                key = keypad.getKey()
            print("You pressed : %c"%(key) )
            if (str(key) == '#'):
                lcd.clear()
                currentstate = "E1"
                loop()
            else:           
                lcd.message(str(key))
                code = code + str(key)
            key = keypad.NULL
        if findDestination(int(code)):
            destinationTrouve = True
            destination = aeroports.get(int(code))
            
        else:
            print("code inconnu")
            lcd.message("code inconnu")
    
    lcd.clear()
    lcd.message("Activer")
    lcd.setCursor(0,1)
    lcd.message("Interrupteur")

    while interrupteur == False:
        if GPIO.input(interrupteur_pin) == GPIO.LOW:
            interrupteur = True
            print("changement interrupteur")
            currentstate = "E3"
            time.sleep(1)

    
def E3():

    global destination
    global currentstate
    global interrupteur
    verrouille = False

    GPIO.output(YLedPin,GPIO.LOW)
    GPIO.output(GLedPin,GPIO.HIGH)

    while interrupteur:
        if GPIO.input(joystickPin) == GPIO.LOW:
            verrouille = True
            while verrouille == True:
                motor(valueMotor)
                servoWrite(angle)
                lcdOutputE3(pourcentageMoteur(valueMotor),angle,destination)
                time.sleep(0.01)
                if GPIO.input(joystickPin) == GPIO.LOW:
                    verrouille = False
                    time.sleep(0.01)
        if GPIO.input(interrupteur_pin) == GPIO.LOW:
            interrupteur = False
            time.sleep(0.1)
            currentstate = "E1"
        valueMotor = adc.analogRead(0) # read ADC value of channel 0
        angle = defAngle(adc.analogRead(1))
        print('value motor angle calculated : '+str(pourcentageMoteur(valueMotor)))
        motor(valueMotor)
        servoWrite(angle)
        lcdOutputE3(pourcentageMoteur(valueMotor),angle,destination)
        time.sleep(0.01)

    
        

def loop():
    global currentstate
    currentstate = "E1"
    while (True):
        if currentstate == "E1":
            E1()
            # fi C1
            currentstate = "E2"

        elif currentstate == "E2":
            if E2():
                currentstate = "E3"
            # Validation des conditions pour la mise à jour de l'état
            
        elif currentstate == "E3":
            E3()
            # Validation des conditions pour la mise à jour de l'état
            #if C7 is True:
            # currentstate = "E1"  
    
if __name__ == '__main__':  # Program entrance
    print ('Program is starting ... ')
    try:
        setup()
        loop()
    except KeyboardInterrupt: # Press ctrl-c to end the program.
        destroy()


