#!/usr/bin/python

import smbus
import time
import sys

class Sampler:
    def __init__(self, PUMP_ADDRESS=[0x01, 0x02, 0x03, 0x04],
                 PUMP_MAX_VOLUME=350,
                 PUMP_COMMAND_TIMEOUT=0.5,
                 PUMP_FLOW_RATE=105.0,**kwargs):
        self.PUMP_ADDRESS = PUMP_ADDRESS
        self.PUMP_MAX_VOLUME = PUMP_MAX_VOLUME
        self.PUMP_COMMAND_TIMEOUT = PUMP_COMMAND_TIMEOUT
        self.PUMP_FLOW_RATE = PUMP_FLOW_RATE

        self.SLEEP_COMMAND = "Sleep"
        self.TOTAL_VOLUME_COMMAND = "TV,?"
        self.CHECK_DISPENSING_COMMAND = "D,?"
        self.LED_OFF_COMMAND = "L,0"
        self.PUMP_EMPTY_VOLUME = "clear"
        self.PUMP_STOP_DISPENSING = "X"

        self.bus = smbus.SMBus(1)

        for adr in self.PUMP_ADDRESS:
            self.initialize_pump(adr)

    def convert_string_to_bytes(self, cmd):
        converted = []
        for b in cmd:
            converted.append(ord(b))
        return converted

    def write_pump(self, pump_address, cmd):
        start = ord(cmd[0])
        end = cmd[1:] + "\00"
        end = self.convert_string_to_bytes(end)
        try:
            self.bus.write_i2c_block_data(pump_address, start, end)
        except:
            print("[PumpControl] Either the pump was asleep or an error occurred.")
        time.sleep(self.PUMP_COMMAND_TIMEOUT)

    def read_pump(self, pump_address):
        response = self.bus.read_i2c_block_data(pump_address, 0x00)
        response = [i for i in response if i != '\00']
        char_list = list(map(lambda x: chr(x & ~0x80), list(response[1:])))
        return [response[0], ''.join(char_list).strip("\x00")]

    def sleep_pump(self, pump_number):
        self.write_pump(self.PUMP_ADDRESS[pump_number-1], self.SLEEP_COMMAND)

    def verify_pump_volume(self, pump_address):
        self.write_pump(pump_address, self.TOTAL_VOLUME_COMMAND)
        response = self.read_pump(pump_address)
        response_code = response[0]
        if response_code == 1:
            pump_volume = int(float(response[1][4:]))
            return [response_code, pump_volume]
        else:
            print("Error {0} verifying pump {1} volume.").format(response_code, pump_address)
            return [0, 0]

    def fill_pump(self, pump_number, volume):
        [response_code, pump_volume] = self.verify_pump_volume(self.PUMP_ADDRESS[pump_number-1])
        if response_code == 1:
            volume_difference = volume - pump_volume
            if pump_volume < volume:
                # print(volume_difference)
                command = "D," + str(volume_difference)
                self.write_pump(self.PUMP_ADDRESS[pump_number-1], command)
                response_code = self.read_pump(self.PUMP_ADDRESS[pump_number-1])[0]
                if response_code == 1:
                    return volume_difference #Usar para sleep
                else:
                    print("[Pump Control] Error {0} setting pump {1} to dispense.").format(response_code, self.PUMP_ADDRESS[pump_number-1])
                    return -2
            else:
                return -1
        else:
            return -2

    def empty_pump(self, pump_number):
        self.write_pump(self.PUMP_ADDRESS[pump_number-1], self.PUMP_EMPTY_VOLUME)
        _, pump_volume = self.verify_pump_volume(self.PUMP_ADDRESS[pump_number-1])
        return pump_volume

    def stop_pump(self, pump_number):
        self.write_pump(self.PUMP_ADDRESS[pump_number-1], self.PUMP_STOP_DISPENSING)
        _, result = self.read_pump(self.PUMP_ADDRESS[pump_number-1])
        return str(result)

    def initialize_pump(self, pump_address):
        self.write_pump(pump_address, self.LED_OFF_COMMAND)
        response = self.read_pump(pump_address)
        if response[0] == 1:
            self.sleep_pump(pump_address)
        else:
            return response[0]

    def check_pump_state(self, pump_number):
        self.write_pump(self.PUMP_ADDRESS[pump_number-1], self.CHECK_DISPENSING_COMMAND)
        response = self.read_pump(self.PUMP_ADDRESS[pump_number-1])
        if response[0] == 1:
            for _ in range(3):
                try:
                    resp = int(response[1].split(",")[2])
                    return resp
                except:
                    pass
            print("[PumpControl] Error: null byte in check_pump_state")
            return -1   # Erro!
        else:
            return response[0]

# def main():
#     i = 0
#     pump_volume_left = [0, 0, 0, 0]
#     for address in PUMP_ADDRESS:
#         [response_code, pump_volume] = verify_pump_volume(address)
#         if response_code == 1:
#             pump_volume_left[i] = PUMP_MAX_VOLUME - pump_volume
#         else:
#             print("Couldn't verify the volume from pump {0}.").format(address)
# ##        sleep_pump(address)
#     while True:
#         print("The pumps' addresses are {0}.\n\tType \'q\' to exit.").format(PUMP_ADDRESS)
#         keyboard = raw_input("\tType from 0 to 3 to choose which pump will be activated: ")
#         if '0' <= keyboard <= '3':
#             keyboard = int(keyboard)
#             volume_difference = fill_pump(PUMP_ADDRESS[keyboard])
#             if volume_difference >= 0:
#                 print("Pump {0} started dispensing water.").format(PUMP_ADDRESS[keyboard])
#                 pump_volume_left[i] = volume_difference
#                 time_until_full = (pump_volume_left[i]/PUMP_FLOW_RATE)*60
#                 time.sleep(time_until_full)
#                 while True:
#                     pump_done = check_pump_state(PUMP_ADDRESS[keyboard])
#                     if pump_done == 0:
#                         print("Dispensing complete!")
#                         break
#                     elif pump_done == 1:
#                         sys.stdout.write(".")
#                         sys.stdout.flush()
#                         time.sleep(5)
#                     else:
#                         print("Error verifying pump {0} state.").format(PUMP_ADDRESS[keyboard])
#                         break
#             elif volume_difference == -1:
#                 print("Pump {0} already full.").format(PUMP_ADDRESS[keyboard])
#             else:
#                 print("Error sending 'dispense command' to pump {0}.").format(PUMP_ADDRESS[keyboard])
#         elif keyboard == 'q':
#             break
#         else:
#             print("Input not understood. Try again.")

# if __name__ == '__main__':
#     main()
