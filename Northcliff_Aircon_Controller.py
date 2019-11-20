#!/usr/bin/env python3
# Northcliff Airconditioner Controller Version 3.48 Gen
import RPi.GPIO as GPIO
import time
from datetime import datetime
#import requests
#from threading import Thread
import paho.mqtt.client as mqtt
import struct
import json
import serial
import binascii
import sys
import spidev
import math
import os

class NorthcliffAirconController(object):
    def __init__(self, calibrate_damper_on_startup):
        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.control_enable = 17
        self.damper_control = 25
        self.damper_stop = 24
        self.damper_zone = 23
        GPIO.setup(self.control_enable, GPIO.OUT) 
        GPIO.setup(self.damper_control, GPIO.OUT)
        GPIO.setup(self.damper_stop, GPIO.OUT)
        GPIO.setup(self.damper_zone, GPIO.OUT)
        GPIO.output(self.control_enable, False)
        self.damper_control_state = False
        GPIO.output(self.damper_control, False)
        self.damper_stop_state = False
        GPIO.output(self.damper_stop, False)
        self.damper_zone_state = False
        GPIO.output(self.damper_zone, False)
        
        # Aircon Startup Mode
        self.remote_operation_on = False # This flag keeps track of whether the aircon is under remote or autonomous operation 
        self.enable_serial_comms_loop = False # This flag is set to True during remote operation to enable the serial comms loop when the aircon is under remote operations 
        self.heating = False # Mirrors aircon heating state indicator
        self.compressor = False # Mirrors aircon compressor state indicator
        self.malfunction = False # Mirrors aircon malfunction state indicator and is used to indicate a malfunction in the aircon/controller comms
        self.heat_mode = False # Mirrors aircon heat mode indicator
        self.cool_mode = False # Mirrors aircon cool mode indicator
        self.fan_mode = False # Mirrors aircon fan mode indicator
        self.fan_hi = False # Mirrors aircon fan hi indicator
        self.fan_med = False # Mirrors aircon fan med indicator
        self.fan_lo = False # Mirrors aircon fan lo indicator
        self.filter = False # Mirrors aircon filter indicator
        
        # Set up damper states
        self.requested_damper_percent = 100
        self.adjusting_damper = False
        
        # Set default damper positions
        self.damper_day_position = 416
        self.damper_night_position = 1648
        self.calibrate_damper_on_startup = calibrate_damper_on_startup
        
        # Set up heartbeat
        self.heartbeat_count = 0
        self.no_heartbeat_ack = False

        # Set up Serial Comms Data
        self.packet1_header_a = '00'
        self.packet1_header_b = '8f'
        self.packet1_header = self.packet1_header_a + self.packet1_header_b
        self.packet2_header_a = '80'
        self.packet2_header_b = '8c'
        self.packet2_header = self.packet2_header_a + self.packet2_header_b
        self.packet3_initial_header = self.packet1_header
        self.mode = {'Auto On': 'b0', 'Auto Off': '90', 'Dry On': 'b1', 'Dry Off': '91', 'Cool On': 'b2', 'Cool Off': '92', 'Fan On': 'b3', 'Fan Off': '93', 'Heat On': 'b4', 'Heat Off': '94'}
        self.set_temp = {'18 degrees': '48', '19 degrees': '4a', '20 degrees': '4c', '21 degrees': '4e', '22 degrees': '50', '23 degrees': '52', '24 degrees': '54', '25 degrees': '56', '26 degrees': '58',
                          '27 degrees': '5a', '28 degrees': '5c', '29 degrees': '5e', '30 degrees': '60'}
        self.fan_speed = {'Lo On': 'f0', 'Lo Off': 'e0', 'Med On': 'f1',  'Med Off': 'e1', 'Hi On': 'f2',  'Hi Off': 'e2'}
        self.clean_filter = {'Reset': 'f1', 'No Reset': 'f0'}
        self.alerts = {'Not in Warmup': ['f8', 'fa'], 'Warmup': ['f9', 'fb'], 'Clean Filter': ['fa', 'fb'], 'Filter OK': ['f8', 'f9']}
        self.compressor_state = {'Off': 'e0', 'On': 'e2'}
        
        # Set up dictionaries for Serial Comms Packets to Off, Fan Mode, Fan Lo
        self.packet_1_dictionary = {"1Header1": self.packet1_header, "2Mode1": self.mode['Fan Off'], "3Filler1a": "00", "4SetTemp1": self.set_temp['20 degrees'], "5Fan1": self.fan_speed['Hi Off'],
                                    "6Filler1b": "fffff03fffffffffff"}
        self.packet_2_dictionary = {"1Header2": self.packet2_header, "2Mode2": self.mode['Fan Off'], "3Filler2a": "00", "4SetTemp2": self.set_temp['20 degrees'], "5Fan2": self.fan_speed['Hi Off'],
                                    "6ActualTemp2": "90", "7Filler2b": "00", "8Unknown2": "e0", "9Alerts2": self.alerts['Warmup'], "10Filler2c": "ffff", "11Compressor2": self.compressor_state['On'],
                                    "12Filler2c": "ffff", "13Checksum2": "00"}
        self.packet_3_dictionary = {"1Header3": self.packet3_initial_header, "2Mode3": self.mode['Fan Off'], "3Filler3a": "00", "4SetTemp3": self.set_temp['20 degrees'],
                                    "5Fan3": self.fan_speed['Hi Off'], "6Filler3b": "fffff03fffffffffff"}
        
        # Set up serial port for aircon controller comms
        self.aircon_comms = serial.Serial("/dev/ttyAMA0", 1200, parity=serial.PARITY_EVEN, timeout=0.5) # After swapping serial and bluetooth ports so we can use parity

        # Set up SPI Port for the damper position sensor
        self.spi = spidev.SpiDev()
        speed = 50000
        self.spi.open(0,0)
        self.spi.max_speed_hz = speed

        # Initialise damper position sensor
        resp = self.spi.xfer2([0x0e, 0x00, 0x00]) # X-Channel Self Test
        time.sleep(0.3)
        resp = self.spi.xfer2([0x00, 0x00]) # Exit Self Test
        time.sleep(0.1)
        resp = self.spi.xfer2([0x0f, 0x00, 0x00]) # Y-Channel Self Test
        time.sleep(0.3)
        resp = self.spi.xfer2([0x00, 0x00]) # Exit Self Test
        time.sleep(0.1)

    def print_status(self, print_message):
        today = datetime.now()
        print("")
        print(print_message + today.strftime('%A %d %B %Y @ %H:%M:%S'))
        
    def startup(self):        
        self.print_status("Northcliff Aircon Controller starting up on ")
        # Set up mqtt client
        self.client = mqtt.Client('aircon') #Create new instance of mqtt Class
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect("<your mqtt Broker name>", 1883, 60) #Connect to mqtt broker
        self.client.loop_start() #Start mqtt monitor thread
        if self.calibrate_damper_on_startup == True:
        	self.calibrate_damper(damper_movement_time = 180)
        # Detect Damper Position and update Home Manager with aircon status
        self.detect_damper_position(calibrate = False)
        self.update_status()

    def on_connect(self, client, userdata, flags, rc): # Print mqtt status on connecting to broker
        time.sleep(1)
        self.print_status("Connected to mqtt server with result code "+str(rc)+" on ")
        print("")
        self.client.subscribe("AirconControl")
        
    def on_message(self, client, userdata, msg): # mqtt message method calls
        decoded_payload = str(msg.payload.decode("utf-8"))
        message = msg.topic+" "+ decoded_payload # Capture message with binary states converted to a string
        #print(message)
        if str(msg.topic) == 'AirconControl':
            parsed_json = json.loads(decoded_payload)
            if parsed_json['service'] == 'Off':
                self.process_thermo_off_command()
            elif parsed_json['service'] == 'Ventilate':
                self.process_ventilate_mode()
            elif parsed_json['service'] == 'Thermostat Heat':
                self.process_thermo_heat_command()
            elif parsed_json['service'] == 'Thermostat Cool':
                self.process_thermo_cool_command()
            elif parsed_json['service'] == 'Thermostat Auto':
                self.process_thermo_auto_command()
            elif parsed_json['service'] == 'Heat Mode':
                self.process_heat_command()
            elif parsed_json['service'] == 'Cool Mode':
                self.process_cool_command()
            elif parsed_json['service'] == 'Fan Mode':
                self.process_fan_command()
            elif parsed_json['service'] == 'Fan Hi':
                self.process_fan_hi_command()
            elif parsed_json['service'] == 'Fan Med':
                self.process_fan_med_command()
            elif parsed_json['service'] == 'Fan Lo':
                self.process_fan_lo_command()
            elif parsed_json['service'] == 'Damper Percent':
                self.requested_damper_percent = parsed_json['value']
                self.print_status("Damper Command Received on ")
                print("Requested Damper Percent is", self.requested_damper_percent, "Current Damper Percent is", self.reported_damper_percent)       
            elif parsed_json['service'] == 'Update Status': # If HomeManager wants a status update
                self.print_status("Status Update Requested on ")
                self.update_status()
            elif parsed_json['service'] == 'Heartbeat Ack': # If HomeManager sends a heartbeat ack
                self.heartbeat_ack()            
            else:
                print("Received unknown message", str(parsed_json))

    def update_status(self): # Send aircon status to Home Manager
        status = json.dumps({'service': 'Status Update', 'Remote Operation': self.remote_operation_on, 'Heat': self.heat_mode, 'Cool': self.cool_mode,
                                  'Fan': self.fan_mode, 'Fan Hi': self.fan_hi, 'Fan Med': self.fan_med, 'Fan Lo': self.fan_lo, 'Heating': self.heating,
                                  'Compressor': self.compressor, 'Malfunction': self.malfunction, 'Damper': self.reported_damper_percent, 'Filter': self.filter})
        self.client.publish('AirconStatus', status)

    ### Methods for mqtt messages received from Home Manager ###
    def process_thermo_off_command(self):
        self.print_status("Thermo Off Command received on ")
        self.packet_1_dictionary["2Mode1"] = self.mode['Fan Off'] # Set Fan to Off Mode
        self.packet_3_dictionary["2Mode3"] = self.mode['Fan Off'] # Set Fan to Off Mode
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Hi Off'] # Set Fan to High
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Hi Off'] # Set Fan to High
        self.cool_mode = False
        self.fan_mode = False
        self.heat_mode = False
        self.fan_med = False
        self.fan_hi = False
        self.fan_lo = False
        self.update_status()
        time.sleep(3) # Wait for packets to be sent before disconnecting
        self.enable_serial_comms_loop = False # Sets the flag to exit serial comms loop and prepare for disconnect
        # The disconnect is done in the main loop so it happens between packet 3 and packet 1
        
    def process_thermo_heat_command(self):
        self.print_status("Thermo Heat Command received on ")
        if self.remote_operation_on == False: # Turn On
            self.remote_operation_on = True
            self.enable_serial_comms_loop = True
            GPIO.output(self.control_enable, True) # Take Control of Remote
            self.damper_control_state = True
            GPIO.output(self.damper_control, True) # Take Control of Damper
            time.sleep (1.0)
        self.packet_1_dictionary["2Mode1"] = self.mode['Fan On'] # Set to Fan Mode
        self.packet_1_dictionary["4SetTemp1"] = self.set_temp['30 degrees'] # Set 30 degrees for Heating
        self.packet_3_dictionary["2Mode3"] = self.mode['Fan On'] # Set to Fan Mode
        self.packet_3_dictionary["4SetTemp3"] = self.set_temp['30 degrees'] # Set 30 degrees for Heating
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Lo On'] # Fan Lo
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Lo On'] # Fan Lo
        self.cool_mode = False
        self.fan_mode = True
        self.heat_mode = False
        self.fan_med = False
        self.fan_hi = False
        self.fan_lo = True
        self.update_status()

    def process_thermo_cool_command(self):
        self.print_status("Thermo Cool Command received on ")
        if self.remote_operation_on == False: # Turn On
            self.remote_operation_on = True
            self.enable_serial_comms_loop = True
            GPIO.output(self.control_enable, True) # Take Control of Remote
            self.damper_control_state = True
            GPIO.output(self.damper_control, True) # Take Control of Damper
            time.sleep (1.0)
        self.packet_1_dictionary["2Mode1"] = self.mode['Fan On'] # Set to Fan Mode
        self.packet_1_dictionary["4SetTemp1"] = self.set_temp['18 degrees'] # Set 18 Degrees for Cooling
        self.packet_3_dictionary["2Mode3"] = self.mode['Fan On'] # Set to Fan Mode
        self.packet_3_dictionary["4SetTemp3"] = self.set_temp['18 degrees'] # Set 18 Degrees for Cooling
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Lo On'] # Fan Lo
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Lo On'] # Fan Lo
        self.cool_mode = False
        self.fan_mode = True
        self.heat_mode = False
        self.fan_med = False
        self.fan_hi = False
        self.fan_lo = True
        self.update_status()
        
    def process_ventilate_mode(self):
        self.print_status("Ventilate Command received on ")
        if self.remote_operation_on == False: # Turn On
            self.remote_operation_on = True
            self.enable_serial_comms_loop = True
            GPIO.output(self.control_enable, True) # Take Control of Remote
            self.damper_control_state = True
            GPIO.output(self.damper_control, True) # Take Control of Damper
            time.sleep (1.0)
        self.packet_1_dictionary["2Mode1"] = self.mode['Fan On'] # Set to Fan Mode
        self.packet_1_dictionary["4SetTemp1"] = self.set_temp['21 degrees'] # Set 21 Degrees
        self.packet_3_dictionary["2Mode3"] = self.mode['Fan On'] # Set to Fan Mode
        self.packet_3_dictionary["4SetTemp3"] = self.set_temp['21 degrees'] # Set 21 Degrees
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Hi On'] # Fan Hi
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Hi On'] # Fan Hi
        self.cool_mode = False
        self.fan_mode = True
        self.heat_mode = False
        self.fan_med = False
        self.fan_hi = True
        self.fan_lo = False
        self.update_status()

    def process_thermo_auto_command(self): # Holding place if Auto method is to be added in the future
        pass

    def process_heat_command(self):
        self.print_status("Heat Mode Command received on ")
        self.packet_1_dictionary["2Mode1"] = self.mode['Heat On'] # Set to Heat Mode
        self.packet_1_dictionary["4SetTemp1"] = self.set_temp['30 degrees'] # Set 30 degrees for Heating
        self.packet_3_dictionary["2Mode3"] = self.mode['Heat On'] # Set to Heat Mode
        self.packet_3_dictionary["4SetTemp3"] = self.set_temp['30 degrees'] # Set 30 degrees for Heating
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Hi On'] # Fan Hi
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Hi On'] # Fan Hi
        self.cool_mode = False
        self.fan_mode = False
        self.heat_mode = True
        self.fan_med = False
        self.fan_hi = True
        self.fan_lo = False
        self.update_status()
            
    def process_cool_command(self):
        self.print_status("Cool Mode Command received on ")
        self.packet_1_dictionary["2Mode1"] = self.mode['Cool On'] # Set to Cool Mode
        self.packet_1_dictionary["4SetTemp1"] = self.set_temp['18 degrees'] # Set 18 Degrees for Cooling
        self.packet_3_dictionary["2Mode3"] = self.mode['Cool On'] # Set to Cool Mode
        self.packet_3_dictionary["4SetTemp3"] = self.set_temp['18 degrees'] # Set 18 Degrees for Cooling
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Hi On'] # Fan Hi
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Hi On'] # Fan Hi
        self.cool_mode = True
        self.fan_mode = False
        self.heat_mode = False
        self.fan_med = False
        self.fan_hi = True
        self.fan_lo = False
        self.update_status()

    def process_fan_command(self):
        self.print_status("Fan Mode Command received on ")
        self.packet_1_dictionary["2Mode1"] = self.mode['Fan On'] # Set to Fan Mode
        self.packet_3_dictionary["2Mode3"] = self.mode['Fan On'] # Set to Fan Mode
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Lo On'] # Fan Lo
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Lo On'] # Fan Lo
        self.cool_mode = False
        self.fan_mode = True
        self.heat_mode = False
        self.fan_med = False
        self.fan_hi = False
        self.fan_lo = True
        self.update_status()
        
    def process_fan_hi_command(self):
        self.print_status("Fan Hi Command received on ")
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Hi On'] # Fan Hi
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Hi On'] # Fan Hi
        self.fan_med = False
        self.fan_hi = True
        self.fan_lo = False
        self.update_status()
        
    def process_fan_med_command(self):
        self.print_status("Fan Med Command received on ")
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Med On'] # Fan Med
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Med On'] # Fan Med
        self.fan_med = True
        self.fan_hi = False
        self.fan_lo = False
        self.update_status()
        
    def process_fan_lo_command(self):
        self.print_status("Fan Lo Command received on ")
        self.packet_1_dictionary["5Fan1"] = self.fan_speed['Lo On'] # Fan Lo
        self.packet_3_dictionary["5Fan3"] = self.fan_speed['Lo On'] # Fan Lo
        self.fan_med = False
        self.fan_hi = False
        self.fan_lo = True
        self.update_status()       

    def heartbeat_ack(self):
        #self.print_status('Heartbeat received from Home Manager on ')
        self.heartbeat_count = 0
        self.no_heartbeat_ack = False
    ### End of Methods for mqtt messages received from Home Manager ###

    ### Methods called in main loop ###
    def process_home_manager_heartbeat(self): # Send heartbeat signal to Home Manager every 120 loops. Turn aircon off and reboot if there's no response within 80 more loops
        self.heartbeat_count += 1
        if self.heartbeat_count == 120:
            #self.print_status('Sending Heartbeat to Home Manager on ')
            self.send_heartbeat_to_home_manager()
        if self.heartbeat_count > 200:
            self.print_status('Home Manager Heartbeat Lost. Setting Aircon to Thermo Off Mode on ')
            self.client.publish('AirconStatus', '{"service": "Restart"}')
            self.no_heartbeat_ack = True
            self.process_thermo_off_command()
            time.sleep(10)
            os.system('sudo reboot')

    def send_heartbeat_to_home_manager(self):
        self.client.publish('AirconStatus', '{"service": "Heartbeat"}')

    def build_packets(self, packet_1, packet_3): # Build packets 1 and 3 for sending to the aircon
        packets = [packet_1, packet_3]
        for x in range(2):
            sorted_packet = ([value for (key, value) in sorted(packets[x].items())]) # Sort the bytes contained in each packet dictionary into the correct order by using the first digit in the byte key
            packet_no_checksum = ''.join(sorted_packet) # Join the packet dictionary bytes into one string
            checksum = self.calculate_checksum(packet_no_checksum) # Calculate the checksum
            packet_with_checksum = packet_no_checksum + checksum # Add the checksum to the end of the packet
            packet_send = bytes.fromhex(''.join(packet_with_checksum)) # Convert the joined packet to binary
            if x == 0: # Packet 1
                self.packet_1_with_checksum = packet_with_checksum
                self.packet_1_send = packet_send
            else: # Packet 3
                self.packet_3_with_checksum = packet_with_checksum
                self.packet_3_send = packet_send

    def send_serial_aircon_data(self, packet): # Send packet to aircon comms port
        self.aircon_comms.write(packet)

    def receive_serial_aircon_data(self): # Receive Packet 2 from aircon comms port 
        # Look for Packet 2 Header (x808c)
        header_loop_count = 0
        found_packet_2_header = False
        while header_loop_count < 16: # Test an entire packet for header
            test_for_header_1 = self.aircon_comms.read(1) # Read one byte to look for the first half of the header
            if test_for_header_1 == b'\x80':
                test_for_header_2 = self.aircon_comms.read(1) # Read one byte to look for the second half of the header, after sucessfully finding the first half of the header
                if test_for_header_2 == b'\x8c':
                    found_packet_2_header = True # Flag that the correct Packet 2 header has been found
                    exit_loop_count = header_loop_count # Record the loop count in which the correct header was found (for debugging purposes)
                    header_loop_count = 16 # Exit the loop if the complete correct header has been found
                else:
                    header_loop_count += 1 # Look for another instance of the first half of the header if the correct second half wasn't found immediately after the first half     
            else:
                header_loop_count += 1 # Keep looking for the first half of the header
        if found_packet_2_header == True: # Read the remaining bytes in the packet after the correct Packet 2 Header is found
            self.raw_response_1 = self.aircon_comms.read(6) # Capture the next 6 bytes
            self.raw_response_2 = self.aircon_comms.read(8) # capture the next 8 bytes
            self.raw_response = b"".join([test_for_header_1, test_for_header_2, self.raw_response_1, self.raw_response_2]) # Construct the entire Packet 2 in binary form
            self.packet_2 = str(binascii.hexlify(self.raw_response), "utf-8") # Convert Packet to to a string
            self.decode_packet(self.packet_2) # Extract each component of Packet 2 and place in a dictionary that decodes the aircon function of each packet byte
        else: # Flag that no correct Packet 2 header has been found
            print("No valid Packet 2 Header received")
            self.packet_2_error = True
            self.malfunction = True
            
    def decode_packet(self, packet_2): # Extract each component of Packet 2 and place in a dictionary that decodes the aircon function of each packet byte. Validate checksum and comparison with Packet 1 data
        self.packet_2_error = False # Flag that Packet 2 is OK
        self.previous_malfunction = self.malfunction # Capture the previous malfunction state
        self.malfunction = False # Clear the malfunction flag
        self.packet_2_dictionary["1Header2"] = packet_2[0:4]
        self.packet_2_dictionary["2Mode2"] = packet_2[4:6]
        self.packet_2_dictionary["3Filler2a"] = packet_2[6:8]
        self.packet_2_dictionary["4SetTemp2"] = packet_2[8:10]
        self.packet_2_dictionary["5Fan2"] = packet_2[10:12]
        self.previous_actual_temperature = self.packet_2_dictionary["6ActualTemp2"]
        self.packet_2_dictionary["6ActualTemp2"] = packet_2[12:14]
        self.packet_2_dictionary["7Filler2b"] = packet_2[14:16]
        self.packet_2_dictionary["8Unknown2"] = packet_2[16:18]
        self.packet_2_dictionary["9Alerts2"] = packet_2[18:20]
        self.packet_2_dictionary["10Filler2c"] = packet_2[20:24]
        self.packet_2_dictionary["11Compressor2"] = packet_2[24:26]
        self.packet_2_dictionary["12Filler2c"] = packet_2[26:30]
        self.packet_2_dictionary["13Checksum2"] = packet_2[30:32]
        packet_no_checksum = packet_2[0:30] # Capure the packet without the checksum so that the checksum can be calculated
        checksum = self.calculate_checksum(packet_no_checksum)
        if self.packet_2_dictionary["11Compressor2"] == self.compressor_state['On']:
            if self.compressor == False:
                self.compressor = True
                #self.print_status("Aircon Compressor Started on ")
                self.update_status()
        if self.packet_2_dictionary["11Compressor2"] == self.compressor_state['Off']:
            if self.compressor == True:
                self.compressor = False
                #self.print_status("Aircon Compressor Stopped on ")
                self.update_status()
        if self.packet_2_dictionary["9Alerts2"] == self.alerts['Warmup'][0] or self.packet_2_dictionary["9Alerts2"] == self.alerts['Warmup'][1]:
            if self.heating == False:
                self.heating = True
                #self.print_status("Aircon Warmup Started on ")
                self.update_status()
        if self.packet_2_dictionary["9Alerts2"] == self.alerts['Not in Warmup'][0] or self.packet_2_dictionary["9Alerts2"] == self.alerts['Not in Warmup'][1]:
            if self.heating == True:
                self.heating = False
                #self.print_status("Aircon Warmup Stopped on ")
                self.update_status()
        if self.packet_2_dictionary["9Alerts2"] == self.alerts['Clean Filter'][0] or self.packet_2_dictionary["9Alerts2"] == self.alerts['Clean Filter'][1]:
            if self.filter == False:
                self.filter = True
                self.print_status("Filter Clean Alert Active on ")
                self.update_status()
        if self.packet_2_dictionary["9Alerts2"] == self.alerts['Filter OK'][0] or self.packet_2_dictionary["9Alerts2"] == self.alerts['Filter OK'][1]:
            if self.filter == True:
                self.filter = False
                self.print_status("Filter Clean Alert Reset on ")
                self.update_status()
        if self.packet_2_dictionary["8Unknown2"] != "e0":
            self.print_status("Unknown Byte 8 of Packet 2 ")
            print("Expected e0 but received ", self.packet_2_dictionary["8Unknown2"])
        if checksum != self.packet_2_dictionary["13Checksum2"]:
            print ("Packet 2 Checksum Error. Expected ", checksum, " Received ", self.packet_2_dictionary["13Checksum2"])
            self.packet_2_error = True
            self.malfunction = True
        if packet_2[4:12] != self.packet_1_with_checksum[4:12]:
            print("Mismatch between Packets 1 and 2. Expected ", self.packet_1_with_checksum[4:12], " but received ", packet_2[4:12])
        if self.malfunction != self.previous_malfunction:
           self.update_status()     

    def calculate_checksum(self, packet_no_checksum): # Calculate and return Packet 2's checksum
        b = [packet_no_checksum[i:i+2] for i in range(0, len(packet_no_checksum), 2)] # Build a list of each non-checksum Packet 2 byte in hex string form
        c = [int(i, 16) for i in b] # Convert the hex string form list into a list of integers
        d = sum(c) % 256 # Sum the integer list in modulo 256
        return hex(d)[2:].zfill(2) # Return the checksum in 2 digit hex form

    def calculate_next_sequence_number(self, current_number): # Calculate to next Packet 3 sequence number
        current_first_byte = int(current_number[0:2], 16) # Convert the first byte in hex string form to an integer
        current_third_nibble = int(current_number[2:3], 16) # Convert the third nibble in hex string form to an integer
        if current_third_nibble == 11: # The third nibble cycles between Hex 8 and Hex b
            next_third_nibble = 8 # Reset to 8 if it's completed its full cycle
            if current_first_byte == 50: # The first byte cycles between Hex 00 and Hex 32, incrementing by one when the third nibble completes its full cycle
                next_first_byte = 0 # Reset to 0 if it's completed its full cycle
            else:
                next_first_byte = current_first_byte + 1
        else:
            next_first_byte = current_first_byte
            next_third_nibble = current_third_nibble + 1
        next_string = hex(next_first_byte)[2:].zfill(2) + hex(next_third_nibble)[2:] + "f" # Combine the first byte and third nibble in string form, adding hex f at the end to make it two complete bytes
        return next_string

    def detect_damper_position(self, calibrate):
        resp2 = self.spi.xfer2([0x11, 0x00, 0x00])
        resp2a = int(resp2.pop(1)/2) # Remove LSB since we only need 10% resolution
        resp2b = int(resp2.pop(1)) # Capture but ignore these three bits since we only need 10% resolution
        self.damper_position = int(resp2a) * 2 * 8 # Move bits to their correct position and use Y-Axis number as the position
        if calibrate == False:
            self.current_damper_percent = int((self.damper_night_position - self.damper_position)/((self.damper_night_position - self.damper_day_position)/100))# Sets Day Position at 100% and Night Position at 0% - Assuming that the Night Position has a higher reading from the damper position sensor that the Day Position
            # Convert the reported damper percentage to the nearest 10% of the current percentage
            if self.current_damper_percent >=95:
                self.reported_damper_percent = 100
            elif self.current_damper_percent < 95 and self.current_damper_percent >= 85:
                self.reported_damper_percent = 90
            elif self.current_damper_percent < 85 and self.current_damper_percent >= 75:
                self.reported_damper_percent = 80
            elif self.current_damper_percent < 75 and self.current_damper_percent >= 65:
                self.reported_damper_percent = 70
            elif self.current_damper_percent < 65 and self.current_damper_percent >= 55:
                self.reported_damper_percent = 60
            elif self.current_damper_percent < 55 and self.current_damper_percent >= 45:
                self.reported_damper_percent = 50
            elif self.current_damper_percent < 45 and self.current_damper_percent >= 35:
                self.reported_damper_percent = 40
            elif self.current_damper_percent < 35 and self.current_damper_percent >= 25:
                self.reported_damper_percent = 30
            elif self.current_damper_percent < 25 and self.current_damper_percent >= 15:
                self.reported_damper_percent = 20   
            elif self.current_damper_percent < 15 and self.current_damper_percent >= 5:
                self.reported_damper_percent = 10
            else:
                self.reported_damper_percent = 0

    def adjust_damper_position(self): 
        if self.requested_damper_percent != self.reported_damper_percent:    
            self.adjusting_damper = True
            if self.requested_damper_percent > self.reported_damper_percent:
                self.damper_day_zone() # Set damper switch to day zone if the damper's to be moved towards the day zone
            else:
                self.requested_damper_percent < self.reported_damper_percent
                self.damper_night_zone() # Set damper switch to night zone if the damper's to be moved towards the night zone
        else:
            if self.adjusting_damper == True: # Flag that the damper is no longer being adjusted if it was previously being adjusted
                self.adjusting_damper = False
                self.update_status()
            if self.requested_damper_percent == 100: # Lock damper in Day Zone if the damper is to be wholly in Day Zone
                self.damper_day_zone()
            elif self.requested_damper_percent == 0: # Lock damper in Night Zone if the dampr is to be wholly in Night Zone
                self.damper_night_zone()
            else:
                self.hold_damper() # Hold damper in position if the damper is to be between zones

    def damper_day_zone(self): # Move damper towards the Day Zone
        self.damper_stop_state = False
        GPIO.output(self.damper_stop, False)
        self.damper_zone_state = False
        GPIO.output(self.damper_zone, False)

    def damper_night_zone(self): # Move damper towards the Night Zone
        self.damper_stop_state = False
        GPIO.output(self.damper_stop, False)
        self.damper_zone_state = True
        GPIO.output(self.damper_zone, True)

    def hold_damper(self): # Stop damper motion
        self.damper_stop_state = True
        GPIO.output(self.damper_stop, True)
        
    def calibrate_damper(self, damper_movement_time):
        print('Calibrating Damper')
        print('Taking Control of Damper')
        self.damper_control_state = True
        GPIO.output(self.damper_control, True) # Take Control of Damper
        time.sleep(1)
        print('Moving Damper to Night Zone')
        self.damper_night_zone()
        time.sleep(damper_movement_time)
        print('Moved Damper to Night Zone')
        self.detect_damper_position(calibrate = True)
        print('Night Zone Damper Position', self.damper_position)
        print('Changing Night Zone Damper Position from', self.damper_night_position, 'to', self.damper_position)
        self.damper_night_position = self.damper_position
        print('Moving Damper to Day Zone')
        self.damper_day_zone()
        time.sleep(damper_movement_time)
        print('Moved Damper to Day Zone')
        self.detect_damper_position(calibrate = True)
        print('Day Zone Damper Position', self.damper_position)
        print('Changing Day Zone Damper Position from', self.damper_day_position, 'to', self.damper_position)
        self.damper_day_position = self.damper_position
        print('Relinquishing Control of Damper')
        self.damper_control_state = False # Flag that the damper is no longer being controlled
        GPIO.output(self.damper_control, False) # Relinquish Control of Damper
        time.sleep(1)

    def shutdown_cleanup(self):
        self.print_status("Northcliff Aircon Controller shutting down on ")
        self.process_thermo_off_command() #Turn Aircon off
        GPIO.cleanup()
        self.client.loop_stop() #Stop monitoring mqtt thread
        self.spi.close() 
        sys.exit(0)
    ### End of methods called in the main loop ###

    ### Debugging methods ###
    def capture_and_print_serial(self): # Only used for serial comms debugging
        self.controller_msg = self.aircon_comms.read(8)
        print(str(self.controller_msg))

    def capture_and_file_serial_data(self, capture_file_name): # Only used for serial comms debugging
        a = 0
        with open(capture_file_name, "wb+") as f:
            while a <= 1000:
                self.controller_msg = self.aircon_comms.read(8)
                f.write(self.controller_msg)
                print(str(a) + str(self.controller_msg))
                a = a + 1
    ### End end of debugging methods ###
		
    ### Main Loop ###  
    def run(self):
        try:
            self.startup()
            while True:
                self.process_home_manager_heartbeat() # Send heartbeat to Home Manager every 120 loops.
                if self.enable_serial_comms_loop == True: 
                    self.aircon_comms.flushInput() # remove sent packets from aircon comms buffer
                    self.build_packets(self.packet_1_dictionary, self.packet_3_dictionary) # Build Packets 1 and 3
                    self.send_serial_aircon_data(self.packet_1_send) # Send Packet 1 to aircon comms port
                    time.sleep(0.160) # Wait until Packet 1 has been sent before clearing aircon comms buffer
                    self.aircon_comms.flushInput() # remove sent packets from aircon comms buffer
                    time.sleep(0.15) # Gap between Packets 1 and 2
                    self.receive_serial_aircon_data() # Receive Packet 2 and decode it
                    if self.packet_2_error == False: #Only send packet 3 if packet 2 was OK
                        time.sleep(0.16) # Gap between Packets 2 and 3
                        self.send_serial_aircon_data(self.packet_3_send) # Send Packet 3
                        self.packet_3_dictionary["1Header3"] = self.calculate_next_sequence_number(self.packet_3_dictionary["1Header3"]) # Set up the sequence number for the next transmission of Packet 3
                    else:
                        print("Packet 3 not sent because of Packet 2 error")
                    time.sleep(0.45) # Wait until Packet 3 has been sent, plus 0.05 sec gap (or equivalent time if it isn't sent)
                    self.detect_damper_position(calibrate = False) # Determine the damper's current position
                    self.adjust_damper_position() # Adjusts damper position if the current damper position is different from the requested damper position
                else:
                    if self.remote_operation_on == True: # This ensures that the disconnect is only done once
                        self.remote_operation_on = False # Flag that the aircon is not being controlled
                        GPIO.output(self.control_enable, False) # Relinquish Control of the aircon
                        self.damper_control_state = False # Flag that the damper is no longer being controlled
                        GPIO.output(self.damper_control, False) # Relinquish Control of Damper
                        self.damper_day_zone() # Turn Damper Zone and Stop relays Off
                        self.heartbeat_count = 0 # Reset the heartbeat count to start from zero when Home Manager comms is restored
                        if self.no_heartbeat_ack == True:
                            self.malfunction = True
                        else:
                            self.malfunction = False #Clear Malfunction Flag (Packets might be corrupted on disconnect) unless there's a loss of heartbeat
                        self.update_status()
                    else:
                        time.sleep (1)
        except KeyboardInterrupt:
            self.shutdown_cleanup()
    ### End of Main Loop ###

if __name__ =='__main__':
    controller = NorthcliffAirconController(calibrate_damper_on_startup = False)
    controller.run()
    
