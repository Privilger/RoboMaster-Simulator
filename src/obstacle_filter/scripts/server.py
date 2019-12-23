#!/usr/bin/python3
from socket import *
from time import ctime

#HOST = '192.168.2.115'
HOST = 'localhost'
PORT = 10111
ADDRESS = (HOST, PORT)

serverSocket = socket(AF_INET, SOCK_STREAM)
serverSocket.bind(ADDRESS)
serverSocket.listen(5)

while True:
    print("waiting...")
    clientSocket, address = serverSocket.accept()
    print(address, "being connected")

    while True:
        data = clientSocket.recv(1024)
        if not data:
            break

        replyMsg = data.decode() 
        clientSocket.send(replyMsg.encode())

    clientSocket.close()
serverSocket.close()
