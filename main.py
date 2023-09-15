from machine import Pin, I2C
import utime as time
from dht11 import DHT11, InvalidChecksum
import concurrent.futures
import _thread
from cfg import *
from collections import deque

import socket

lock = _thread.allocate_lock()
temperature = deque([-273 for idx in range(TEMPERATURE_BUFFER_LENGTH)], maxlen=TEMPERATURE_BUFFER_LENGTH)


def Socket_Init():
    # get the hostname
    host = socket.gethostname()
    port = SOCKET_PORT_NUMBER  # initiate port num above 1024

    serverSocket = socket.socket()  # get instance
    # look closely. The bind() function takes tuple as argument
    serverSocket.bind((host, port))  # bind host address and port together

    # configure how many client the server can listen simultaneously
    serverSocket.listen(SOCKET_MAX_CLIENTS)

    return serverSocket


def Socket_Serve(serverSocket: socket.socket):
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
            tempBuff = list(temperature)  # buffer of 10 temperature samples
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
        print("Exception " + str(e) + " in socket thread")


def Sensor_Init():
    pin = Pin(DHT11_GPIO_NUM, Pin.OUT, Pin.PULL_DOWN)       # TODO verifica che pin usi
    sensor = DHT11(pin)
    return sensor


def Sensor_Get(sensor):
    while(True):
        time.sleep(TEMPERATURE_UPDATE_PERIOD_S)
        t  = (sensor.temperature)
        lock.acquire()
        temperature.append(t)
        lock.release()
        h = (sensor.humidity)
        # TODO gestisci anche l'umidità
        print("Temperature: {}".format(sensor.temperature))
        print("Humidity: {}".format(sensor.humidity))

def Field_Handle():
    sensor = Sensor_Init()
    Sensor_Get(sensor)


if __name__ =='__main__':
    time.sleep(5)
    threadPool = concurrent.futures.ThreadPoolExecutor(max_workers=MAX_THREADPOOL_WORKERS)

    # submit tasks to the pool
    fieldThread = threadPool.submit(Field_Handle)
    socketThread = threadPool.submit(Socket_Handle)

    # wait for all tasks to complete
    threadPool.shutdown(wait=True)

        
        