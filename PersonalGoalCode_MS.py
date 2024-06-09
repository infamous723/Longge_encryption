# Mobile Station code for encrypted communication
from multiprocessing import Process
from circuitpython_nrf24l01.rf24 import RF24
import board
import busio
import spidev
import digitalio as dio
import time
import struct
import argparse
from random import randint
import numpy as np
import os
import fcntl
import struct
import zlib
import subprocess
import threading
from Crypto.PublicKey import RSA
from Crypto.Cipher import AES, PKCS1_OAEP
from Crypto.Random import get_random_bytes
CE0=dio.DigitalInOut(board.D17)
CSN0=0
CE1=dio.DigitalInOut(board.D27)
CSN1=10

AES_key = get_random_bytes(16)
RSA_public_key = b'0'

# initializing the tun:
def tun_init():
    iface = 'tun4'
    # Open the existing TUN interface
    tun = os.open('/dev/net/tun', os.O_RDWR)
    # Prepare TUN interface settings
    ifr = struct.pack('16sH', bytes(iface,'utf-8'), 0x0001)
    # Configure the TUN interface
    fcntl.ioctl(tun,0x400454ca , ifr)  # TUNSETIFF
    return tun

# function for encrypting the packets using AES key
def data_encryption(data, AES_key):
    cipher = AES.new(AES_key, AES.MODE_CTR, nonce = b'c')
    encrypted_data = cipher.encrypt(data)
    return encrypted_data

# function for encrypting the AES key using RSA Public key
def key_encryption(AES_key, RSA_public_key):
    cipher_rsa = PKCS1_OAEP.new(RSA_public_key)
    encrypted_AES_key = cipher_rsa.encrypt(AES_key)
    return encrypted_AES_key

# function for decrypting the data using AES key
def data_decryption(data, AES_key): #decrypted AES key
    decipher = AES.new(AES_key, AES.MODE_CTR, nonce = b'c')
    decrypted_data = decipher.decrypt(data)
    return decrypted_data

# function for receiving the RSA Public key from the base station
def key_reception(nrf, channel, address):
    nrf.open_rx_pipe(1, address)
    nrf.listen = True  # put radio into RX mode and power up
    nrf.channel = channel
    print('Rx NRF24L01+ started w/ power {}, SPI freq: {} hz'.format(nrf.pa_level, nrf.spi_frequency))
    received = []
    data = []
    count = 9
    global RSA_public_key
    while count:
        if nrf.update() and nrf.pipe is not None:
            # fetch 1 payload from RX FIFO
            received.append(nrf.any())
            rx = nrf.read()  # also clears nrf.irq_dr status flag
            data.append(rx)
            if data[-1][-1] == 45:
                key_combine = b"".join(data)
                RSA_public_key = key_combine #type of key_combine?
                data = []
            count -= 1
    nrf.listen = False
    print('{} RSA Public key received, {} average'.format(len(received), np.mean(received)))

#function for sending the encrypted AES key to the base station
def key_transmission(nrf, channel, address):
    nrf.open_tx_pipe(address) # set address of RX node into a TX pipe
    nrf.listen = False
    nrf.channel = channel
    status = []
    header = bytes([0b00000000])
    footer = bytes([0b11101110])
    if(len(RSA_public_key) > 10):
        key_packet = key_encryption(AES_key, RSA.import_key(RSA_public_key))
        i = 0 # used for fragmentation
        key_packet = header + key_packet + footer
        while i in range(len(key_packet)):
           fragment = key_packet[i:i+32]
           result = nrf.send(fragment)
           i = i + 32
           if not result:
               print("send() Encrypted AES key failed or timed out")
               #print(nrf.what_happened())
               status.append(False)
           else:
              print("send() Encrypted AES key successful")
              status.append(True)

# function for transmission of data
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
        i = 0 # used for fragmentation
        packet = []
        packet = os.read(tun, mtu) # read from the tun interface
        if(len(packet) > 200):
            packet = zlib.compress(packet)
        encrypted_data = data_encryption(packet, AES_key)
        packet = header + encrypted_data + footer
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

# function for reception of data
def rx(nrf, tun, channel, address):
    nrf.open_rx_pipe(1, address)
    nrf.listen = True  # put radio into RX mode and power up
    nrf.channel = channel
    print('Rx NRF24L01+ started w/ power {}, SPI freq: {} hz'.format(nrf.pa_level, nrf.spi_frequency))
    received = []
    data = []
    start_time = None
    start = time.monotonic()
    while True:
        if nrf.update() and nrf.pipe is not None:
            if start_time is None:
                start_time = time.monotonic()
            received.append(nrf.any())
            rx = nrf.read()  # also clears nrf.irq_dr status flag
            data.append(rx)
            if data[-1][-1] == 0xff:
                icmp_combine = b"".join(data)
                icmp_combine = icmp_combine[1:-1]
                decrypted_data = data_decryption(icmp_combine, AES_key)
                original_data = decrypted_data
                if(len(decrypted_data) > 200):
                    original_data = zlib.decompress(decrypted_data)
                rec = os.write(tun, original_data)
                data = []
    total_time = time.monotonic() - start_time

    print('{} received, {} average, {} bps'.format(len(received), np.mean(received), np.sum(received)*8/total_time))



if __name__ == "__main__":

    shell_script_path = '/root/Desktop/tun_generate.sh'
    subprocess.run([shell_script_path])
    subprocess.run(["ip route add default via 192.168.1.1 dev tun4"], shell = True)

    parser = argparse.ArgumentParser(description='NRF24L01+ test')
    parser.add_argument('--src', dest='src', type=str, default='Noder', help='NRF24L01+\'s source address')
    parser.add_argument('--dst', dest='dst', type=str, default='Nodet', help='NRF24L01+\'s destination address')
    parser.add_argument('--txchannel', dest='txchannel', type=int, default=53, help='Tx channel', choices=range(0,125))
    parser.add_argument('--rxchannel', dest='rxchannel', type=int, default=53, help='Rx channel', choices=range(0,125))
    args = parser.parse_args()

    spi0= spidev.SpiDev()
    spi1 = spidev.SpiDev()
    spi0.open(0, 0)  # Specify the bus number and device number for SPI0
    spi1.open(1, 0)  # Specify the bus number and device number for SPI1
    # initialize the nRF24L01 on the spi bus object
    rx_nrf = RF24(spi0, CSN0, CE0)
    tx_nrf = RF24(spi1, CSN1, CE1)

    for nrf in [rx_nrf, tx_nrf]:
        nrf.data_rate = 1
        nrf.auto_ack = True
        #nrf.dynamic_payloads = True
        nrf.payload_length = 32
        nrf.crc = True
        nrf.ack = 1
        nrf.spi_frequency = 20000000
    #call the tun function:
    tun = tun_init()
    key_reception(rx_nrf, args.rxchannel, bytes(args.dst, 'utf-8')) #First step is receiving the RSA pubic key from the BS.
    time.sleep(1)
    key_transmission(tx_nrf, args.txchannel, bytes(args.src, 'utf-8')) #Second step is sending the encrypted AES key to the BS.
    tx_process = Process(target=tx, args=(tx_nrf, args.txchannel, bytes(args.src, 'utf-8'), tun))
    rx_process = Process(target=rx, args=(rx_nrf, tun, args.rxchannel, bytes(args.dst, 'utf-8')))
    tx_process.start()
    rx_process.start()
    time.sleep(1)
    tx_process.join()
    rx_process.join()