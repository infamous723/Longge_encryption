# Mobile Station Code for basic communication
from multiprocessing import Process
from circuitpython_nrf24l01.rf24 import RF24
import board
import spidev
import digitalio as dio
import time
import struct
import argparse
import numpy as np
import os
import fcntl
import subprocess


#Initialize PINS connections
CE0=dio.DigitalInOut(board.D17)
CSN0=0
CE1=dio.DigitalInOut(board.D27)
CSN1=10

# initilize the tun:
def tun_init():
    iface = 'tun4'
    tun = os.open('/dev/net/tun', os.O_RDWR)
    ifr = struct.pack('16sH', bytes(iface,'utf-8'), 0x0001)
    fcntl.ioctl(tun,0x400454ca , ifr)  
    return tun

# Transmission Function
def tx(nrf, channel, address, tun):
    nrf.open_tx_pipe(address)  # set address of RX node into a TX pipe
    nrf.listen = False
    nrf.channel = channel
    status = []
    start = time.monotonic()
    
    while True:
 
        header = bytes([0b00000000])
        footer = bytes([0b11111111])
        mtu = 1500
        i = 0
        packet = []
        packet = os.read(tun, mtu)
        print("Packet received from tun and Packet size :")
        print(packet,len(packet))
        packet = header+packet+footer

        while i in range(len(packet)):
            fragment =  packet[i:i+32]
            result= nrf.send(fragment)
            i=i+32
        if not result:
            print("send() failed or timed out")
            status.append(False)
        else:
            print("send() successful")
            status.append(True)
            
    total_time = time.monotonic() - start
    print('{} successfull transmissions, {} failures, {} bps'.format(sum(status), len(status)-sum(status), 32*8*len(status)/total_time))

#Receiving Function
def rx(nrf, tun, channel, address):
    nrf.open_rx_pipe(1, address)
    nrf.listen = True  # put radio into RX mode and power up
    nrf.channel = channel
    print('Rx NRF24L01+ started w/ power {}, SPI freq: {} hz'.format(nrf.pa_level, nrf.spi_frequency))

    received = []
    data =[]
    start_time = None
    start = time.monotonic()
    
    while True: 
        if nrf.update() and nrf.pipe is not None:
        
            if start_time is None:
                start_time = time.monotonic()
            received.append(nrf.any())
            rx = nrf.read()  
            data.append(rx)
            if data[-1][-1] == 0xff:
                print("Full packet received with header and footer :")
                print(data)
                icmp_combine = b"".join(data)
                icmp_combine = icmp_combine[1:-1]
                rec = os.write(tun, icmp_combine)
                data=[]
                
    total_time = time.monotonic() - start_time
    print('{} received, {} average, {} bps'.format(len(received), np.mean(received), np.sum(received)*8/total_time))

if __name__ == "__main__":

# Define the shell scripts for creating the tuns 
    shell_script_path = '/root/Desktop/tun_generate.sh'
# Run the shell script and initialize the Default route to be the BS IP address
    subprocess.run([shell_script_path])
    subprocess.run(["ip route add default via 192.168.1.1 dev tun4"], shell = True)
    parser = argparse.ArgumentParser(description='NRF24L01+ test')
    parser.add_argument('--src', dest='src', type=str, default='Noder', help='NRF24L01+\'s source address')
    parser.add_argument('--dst', dest='dst', type=str, default='Nodet', help='NRF24L01+\'s destination address')
    parser.add_argument('--txchannel', dest='txchannel', type=int, default=53, help='Tx channel', choices=range(0,125))
    parser.add_argument('--rxchannel', dest='rxchannel', type=int, default=53, help='Rx channel', choices=range(0,125))
    args = parser.parse_args()

# Initialize the SPI0/SPI1 AND NRF24L01
    spi0= spidev.SpiDev()
    spi1 = spidev.SpiDev()
    spi0.open(0, 0)  # Specify the bus number and device number for SPI0
    spi1.open(1, 0)  # Specify the bus number and device number for SPI1
    # initialize the nRF24L01 on the spi bus object
    rx_nrf = RF24(spi0, CSN0, CE0)
    tx_nrf = RF24(spi1, CSN1, CE1)

    for nrf in [rx_nrf, tx_nrf]:
        nrf.data_rate = 2
        nrf.auto_ack = True
        #nrf.dynamic_payloads = True
        nrf.payload_length = 32
        nrf.crc = True
        nrf.ack = 1
        nrf.spi_frequency = 20000000
        
#Initialize Tun and start Transmit and Receive:
    tun = tun_init()
    tx_process = Process(target=tx, args=(tx_nrf, args.txchannel, bytes(args.src, 'utf-8'), tun))
    rx_process = Process(target=rx, args=(rx_nrf, tun, args.rxchannel, bytes(args.dst, 'utf-8')))
    tx_process.start()
    rx_process.start()
    time.sleep(1)
    tx_process.join()
    rx_process.join()