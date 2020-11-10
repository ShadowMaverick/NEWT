import time
import serial
import csv
import RPi.GPIO as GPIO	# To use GPIO pins
import math

#Connect to ADC with SPI, perform data transaction
def analogInput(channel):
  spi.max_speed_hz = 1350000
  adc = spi.xfer2([1,(8+channel)<<4,0]) # Perform SPI transaction. CS will be held active between blocks.
  data = ((adc[1]&3) << 8) + adc[2]
  return data

#handle temperature readings and convert to degrees from digital signal
def readThermistor():
    # pin_1 = GPIO8
    # pin_2 = GPIO7
    pin_1 = 0
    pin_2 = 1

    adcValue_1 = analogInput(pin_1) # Reading from CH0    //Thermistor 1
    rV_1 = ((1024/adcValue_1) - 1)*1000
    tempK_1 = 1/(9.6564E-04 + (2.1069E-04*math.Log(rV_1)) + (8.5826E-08*Math.Pow(Math.Log(rV_1), 3)));
    tempC_1 = tempK_1 - 273.15
    
    adcValue_2 = analogInput(pin_2) # Reading from CH0    // Thermistor 2
    rV_2 = ((1024/adcValue_2) - 1)*1000
    tempK_2 = 1/(9.6564E-04 + (2.1069E-04*math.Log(rV_2)) + (8.5826E-08*Math.Pow(Math.Log(rV_2), 3)));
    tempC_2 = tempK_2 - 273.15
    
    if (tempC_1 < 80) or (tempC_2 < 80):
        return True
    return False

def main():
    global spi
    
    # Initialize file for data logging
    filename = "data_log.csv"
    csv = open(filename, 'a')
    # Add heading title
    csv.write("timerBuffer, IMUX_angle, IMUX_velocity,angleBuffer_1, angleBuffer_2, velocityBuffer_1, velocityBuffer_2, controlBuffer_1, controlBuffer_2")
    csv.close()

    # Initialize pin output for interrupt to Arduino 
    flag_pin = 5 # interrupt to Arduino
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(flag_pin, GPIO.OUT)
    GPIO.output(flag_pin, GPIO.LOW)        # INPUT in Arduino now, change after testing!

    # Initialize SPI communication
    spi = spidev.SpiDev() # Created an object
    spi.open(0,0)   # open channel for SPI communication
    spi.open(0,1)

    # Read Serial data through USB from Arduino 
    ser = serial.Serial(  # setup usb communication
        port='/dev/ttyACM0',      #check with ls /dev/tty*
        baudrate=250000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        timeout=0)
    print("Connected to: " + ser.portstr)

    time.sleep(5)

    try:
        while readThermistor(): # as long as the temperature is within accepted range, keep reading from Serial
            inputvalue = ser.readline()
            with open(filename, 'a') as csv_file:    # open csv file, append data
                str_value =  inputvalue.decode('ISO-8859-1').strip()
##              print(str_value)
                csv_file.write(str_value + "\n")           
            csv.close()                   
            time.sleep(0.001)

        if not readThermistor():    # if the temperature is too high, trigger output to Arduino
            GPIO.output(flag_pin, GPIO.HIGH)
    except KeyboardInterrupt:
        GPIO.output(flag_pin, GPIO.HIGH)     #send interrupt command to Arduino

        
if __name__ == "__main__":
    main()
