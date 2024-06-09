# Base Station Code for basic communication
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

# Initialize PINs connections
CE0=dio.DigitalInOut(board.D17)
CSN0=0
CE1=dio.DigitalInOut(board.D27)
CSN1=10

# Initilize the tun:
def tun_init():
    iface = 'tun1'
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

        i = 0 # used in fragmentation
        packet = []
        packet = os.read(tun, mtu)
        packet = header+packet+footer
        print("Packet received from tun and Packet size ")
        print(packet,len(packet))
        
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
    k = 0
    while True:
        if nrf.update() and nrf.pipe is not None:
            if start_time is None:
                start_time = time.monotonic()
            received.append(nrf.any())
            rx = nrf.read()  # also clears nrf.irq_dr status flag
            data.append(rx)         
            if data[-1][-1] == 0xff :
                print("Full packet received with header and footer :")
                print(data)
                icmp_combine = b"".join(data)
                icmp_combine= icmp_combine[1:-1]
#                if 100 > len(icmp_combine) >=88: # this for perform Latency measurements 
#                     k = k+1
                rec = os.write(tun, icmp_combine)
                print('rec content :',rec)
                data=[]
        total_time = time.monotonic() - start
#        if total_time >=12: this to perform Latency measurements
#                print('whole packets', k)
#                break
    print('{} received, {} average, {} bps'.format(len(received), np.mean(received), np.sum(received)*8/total_time))
    
    
if __name__ == "__main__":

# Define the shell scripts for creating the tuns and enable the  IPv4 route forwarding 
    shell_script_path = '/home/admin/Desktop/tun_generat.sh'

    
    shell_script_path_1 = '/home/admin/Desktop/route_generate.sh'
# Run shell scripts
    subprocess.run([shell_script_path]) 
    subprocess.run([shell_script_path_1])
    
    parser = argparse.ArgumentParser(description='NRF24L01+ test')
    parser.add_argument('--src', dest='src', type=str, default='Nodet', help='NRF24L01+\'s source address')
    parser.add_argument('--dst', dest='dst', type=str, default='Noder', help='NRF24L01+\'s destination address')
    parser.add_argument('--txchannel', dest='txchannel', type=int, default=53, help='Tx channel', choices=range(0,125))
    parser.add_argument('--rxchannel', dest='rxchannel', type=int, default=53, help='Rx channel', choices=range(0,125))
    args = parser.parse_args()
# Initialize the SPI0/SPI1 AND NRF24L01
    spi0= spidev.SpiDev()
    spi1 = spidev.SpiDev()
    spi0.open(0, 0)  # Specify the bus number and device number for SPI0
    spi1.open(1, 0)  # Specify the bus number and device number for SPI1
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
    rx_process = Process(target=rx, args=(rx_nrf, tun, args.rxchannel, bytes(args.dst, 'utf-8')))
    tx_process = Process(target=tx, args=(tx_nrf, args.txchannel, bytes(args.src, 'utf-8'), tun))
    rx_process.start()
    tx_process.start()
    time.sleep(1)
    rx_process.join()
    tx_process.join()