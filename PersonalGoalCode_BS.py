# Bs Code with Encryption
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
from Crypto.PublicKey import RSA
from Crypto.Cipher import AES, PKCS1_OAEP
from Crypto.Random import get_random_bytes
import zlib

#Initialize PINS connections
CE0=dio.DigitalInOut(board.D17)
CSN0=0
CE1=dio.DigitalInOut(board.D27)
CSN1=10

# RSA and AES keys global variables for all functions
AES_key = b'0000000000000000'
RS_private_key = RSA.generate(1024)
RSA_public_key = RS_private_key.publickey().export_key() #converting them to binary for easy transmission
RSA_private_key = RS_private_key.export_key() 

# initilize the tun:
def tun_init():
    iface = 'tun1'
    tun = os.open('/dev/net/tun', os.O_RDWR)
    ifr = struct.pack('16sH', bytes(iface,'utf-8'), 0x0001)
    fcntl.ioctl(tun,0x400454ca , ifr)  
    return tun

#Encrypt data sent using RX AES key
def data_encryption(data, AES_key):
	cipher = AES.new(AES_key, AES.MODE_CTR, nonce = b'c')
	encrypted_data = cipher.encrypt(data)
	return encrypted_data

#Decrypt the AES key sent by RX 
def key_decryption(encrypted_AES_key, RSA_private_key): 
	cipher_rsa = PKCS1_OAEP.new(RSA_private_key)
	decrypted_AES_key = cipher_rsa.decrypt(encrypted_AES_key)
	decrypted_AES_key
	return decrypted_AES_key 



#Decrypt data sent using RX AES key
def data_decryption(data, AES_key): #decrypted AES key
	decipher = AES.new(AES_key, AES.MODE_CTR, nonce = b'c')
	decrypted_data = decipher.decrypt(data)
	return decrypted_data

#Transmitt TX RSA public key to RX
def key_transmission(nrf, channel, address):
	#set address of RX node into a TX pipe
	nrf.open_tx_pipe(address)
	nrf.listen = False
	nrf.channel = channel
	status = []
	key_packet = RSA_public_key
	print(key_packet)
	i = 0
	while i in range(len(key_packet)):
		fragment = key_packet[i:i+32]
		result = nrf.send(fragment)
		i = i + 32
		if not result:
			print("send() Public key failed or timed out")
			status.append(False)
		else:
			print("send() Public key successful")
			status.append(True)

#Receive Rx AES key from RX
def key_reception(nrf, channel, address):
    nrf.open_rx_pipe(1, address)
    nrf.listen = True  # put radio into RX mode and power up
    nrf.channel = channel
    print('Rx key NRF24L01+ started w/ power {}, SPI freq: {} hz'.format(nrf.pa_level, nrf.spi_frequency))
    received = []
    data = []
    global AES_key
    count = 5
    while count:
        if nrf.update() and nrf.pipe is not None:
            received.append(nrf.any())
            rx = nrf.read()  # also clears nrf.irq_dr status flag
            data.append(rx)
            if data[-1][-1] == 0xee:
                key_combine = b"".join(data)
                key_combine = key_combine[1:-1]
                AES_key = key_decryption(key_combine, RS_private_key)
                print("AES key Received :")
                print(AES_key)
                data = []
            count -= 1
#Transmit encrypted data
def tx(nrf, channel, address, tun):
    nrf.open_tx_pipe(address)  # set address of RX node into a TX pipe
    nrf.listen = False
    nrf.channel = channel
    status = []
    start = time.monotonic()
    
    while True:
        #read from the tun interface
        header = bytes([0b00000000])
        footer = bytes([0b11111111])
        mtu = 1500
        i = 0
        packet = []
        packet = os.read(tun, mtu)
        print(" packet received from tun before encryption ")
        print(packet)
        if len(packet)> 200:# here only large packets can be compressed
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

#Receive encrypted data
def rx(nrf, tun, channel, address):
    nrf.open_rx_pipe(1, address)
    nrf.listen = True  # put radio into RX mode and power up
    nrf.channel = channel
    print('Rx NRF24L01+ started w/ power {}, SPI freq: {} hz'.format(nrf.pa_level, nrf.spi_frequency))
    received = []
    data =[]
    start_time = None
    start = time.monotonic()
    number = 0
    while True: 
        if nrf.update() and nrf.pipe is not None:
            if start_time is None:
                start_time = time.monotonic()
            received.append(nrf.any())
            rx = nrf.read()  
            data.append(rx)
            if data[-1][-1] == 0xff:
                icmp_combine = b"".join(data)
                icmp_combine = icmp_combine[1:-1]
                decrypted_data = data_decryption(icmp_combine, AES_key)
                #compression only happend for large packets
                if len(decrypted_data)> 200:
                    decrypted_data = zlib.decompress(decrypted_data)
#                if 100 > len(decrypted_data)>=88:# this to measure the Latency 
#                    number = number+1
                rec = os.write(tun, decrypted_data)
                data = []
        total_time = time.monotonic() - start
#        if total_time >= 10:# this to measure the Latency only
#            print('whole packets :', number)
#            break
    print('{} received, {} average, {} bps'.format(len(received), np.mean(received), np.sum(received)*8/total_time))

if __name__ == "__main__":

# Define the shell scripts for creating the tuns 
    shell_script_path = '/home/admin/Desktop/tun_generat.sh'
# Run the shell script and initialize the Default route to be the BS IP address
    subprocess.run([shell_script_path])
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

#Initialize Tun and start Transmit and Receive Key exchange:
    tun = tun_init()
    key_transmission(tx_nrf, args.txchannel, bytes(args.src, 'utf-8')) #First step is sending the RSA pubic key to the MS.
    time.sleep(1)
    key_reception(rx_nrf, args.rxchannel, bytes(args.dst, 'utf-8'))
#start Transmit and Receive encrypted data:
    rx_process = Process(target=rx, args=(rx_nrf, tun, args.rxchannel, bytes(args.dst, 'utf-8')))
    tx_process = Process(target=tx, args=(tx_nrf, args.txchannel, bytes(args.src, 'utf-8'), tun))
    rx_process.start()
    tx_process.start()
    time.sleep(1)
    rx_process.join()
    tx_process.join()    