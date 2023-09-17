from machine import Pin, I2C, Timer	
import utime as time
from dht11 import DHT11, InvalidChecksum, TemperatureBuffer
#import concurrent.futures
import _thread
from cfg import *
from ucollections import deque
from picozero import pico_led
import socket

lock = _thread.allocate_lock()

temperature = TemperatureBuffer(TEMPERATURE_BUFFER_LENGTH)


def StopApplication():
    print('Interrupted')
    pico_led.off()
    _thread.exit()
    #machine.reset()

def Socket_Init():
    # get the hostname
    #host = socket.gethostname()
    host = '192.168.178.44'
    port = SOCKET_PORT_NUMBER  # initiate port num above 1024

    serverSocket = socket.socket()  # get instance
    # look closely. The bind() function takes tuple as argument
    serverSocket.bind((host, port))  # bind host address and port together

    # configure how many client the server can listen simultaneously
    serverSocket.listen(SOCKET_MAX_CLIENTS)
    
    print("Socket init done")

    return serverSocket


def Socket_Serve(serverSocket: socket.socket):
    print("Waiting for a new connection...")
    conn, address = serverSocket.accept()  # accept new connection

    print("Connection from: " + str(address))
    while True:
        time.sleep(SOCKET_UPDATE_PERIOD_S)
        # receive data stream. it won't accept data packet greater than 1024 bytes
        data = conn.recv(1024).decode()
        #if not data:
        #    # if data is not received break
        #    break
        print("from connected user: " + str(data))
        if data == "update":
            lock.acquire()
            tempBuff = temperature.get() # buffer of 10 temperature samples
            lock.release()
        elif data == "stop":
            break
        conn.send(tempBuff)  # send data to the client

    print("Close connection")
    conn.close()  # close the connection


def Socket_Handle():
    try:
        serverSocket = Socket_Init()
        Socket_Serve(serverSocket)
    except Exception as e:
        print("Exception " + str(e) + " during socket handling");
        serverSocket.close()
        StopApplication()


def Sensor_Init():
    pin = Pin(DHT11_GPIO_NUM, Pin.OUT, Pin.PULL_DOWN)       # TODO verifica che pin usi
    sensor = DHT11(pin)
    print("Sensor init done")
    return sensor


def Sensor_Get(sensor):
    while(True):
        try:
            time.sleep(TEMPERATURE_UPDATE_PERIOD_S)
            t = sensor.temperature
            h = sensor.humidity
            #t  = ((sensor.temperature * TEMPERATURE_ALFA) + ((1-TEMPERATURE_ALFA)*temperature.getAvg()))
            #h = ((sensor.humidity * HUMIDIRY_ALFA) + ((1-HUMIDITY_ALFA)*t))
            print("Temperature = " + str(t) + ", humidity = " + str(h))
            Blink()
            lock.acquire()
            temperature.append(t)
            lock.release()
        except Exception as e:
            print("Exception " + str(e) + " during sensor data acquisition")


def Field_Handle():
    try:
        sensor= Sensor_Init()
        Sensor_Get(sensor)
    except Exception as e:
        print("Exception " + str(e) + " during sensor data acquisition");
        StopApplication()
        

def Blink():
    led.toggle()
    #pico_led.off()


if __name__ =='__main__':
    time.sleep(2)
    
    led = Pin("LED", Pin.OUT)
    #timer = Timer()
    #timer.init(freq=1, mode=Timer.PERIODIC, callback=Blink)

    # submit tasks to the second thread
    secondThread = _thread.start_new_thread(Socket_Handle, ())
    
    Field_Handle()

