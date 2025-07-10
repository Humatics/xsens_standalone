#!/usr/bin/env python
import serial
import struct
import sys
import argparse
import time
import glob
import re
import pprint
import os

from mtdef import (
    MID,
    OutputMode,
    OutputSettings,
    MTException,
    Baudrates,
    XDIGroup,
    getMIDName,
    DeviceState,
    DeprecatedMID,
    MTErrorMessage,
    MTWarningMessage,
    MTTimeoutException,
)


################################################################
# MTDevice class
################################################################
class MTDevice(object):
    """XSens MT device communication object."""

    def __init__(
        self, port, baudrate=115200, timeout=0.02, autoconf=True, config_mode=False, verbose=False, no_ack=False
    ):
        """Open device."""
        self.verbose = verbose
        self.no_ack = no_ack
        # serial interface to the device
        try:
            self.device = serial.Serial(port, baudrate, timeout=timeout, writeTimeout=timeout)
        except IOError:
            # FIXME with pyserial3 we might need some specific flags
            self.device = serial.Serial(port, baudrate, timeout=timeout, writeTimeout=timeout, rtscts=True, dsrdtr=True)
        self.device.flushInput()  # flush to make sure the port is ready TODO
        self.device.flushOutput()  # flush to make sure the port is ready TODO
        # timeout for communication
        self.timeout = 100 * timeout
        # state of the device
        self.state = None
        if autoconf and not self.no_ack:
            self.auto_config_legacy()
        elif autoconf and self.no_ack:
            if self.verbose:
                print("No-ACK mode: skipping auto-configuration")
        else:
            # mode parameter of the IMU
            self.mode = None
            # settings parameter of the IMU
            self.settings = None
            # length of the MTData message
            self.length = None
            # header of the MTData message
            self.header = None
        if config_mode:
            self.GoToConfig()

    ############################################################
    # Low-level communication
    ############################################################
    def write_msg(self, mid, data=b''):
        """Low-level message sending function."""
        length = len(data)
        if length > 254:
            lendat = b'\xff' + struct.pack('!H', length)
        else:
            lendat = struct.pack('!B', length)
        packet = b'\xfa\xff' + struct.pack('!B', mid) + lendat + data
        packet += struct.pack('!B', 0xFF & (-(sum(packet[1:]))))
        msg = packet
        start = time.time()
        while ((time.time() - start) < self.timeout) and self.device.read():
            pass
        self.device.write(msg)
        if self.verbose:
            print(
                "MT: Write message id 0x%02X (%s) with %d data bytes: [%s]"
                % (
                    mid,
                    getMIDName(mid),
                    length,
                    ' '.join("%02X" % v for v in data),
                )
            )

    def waitfor(self, size=1):
        """Get a given amount of data."""
        buf = bytearray()
        for _ in range(100):
            buf.extend(self.device.read(size - len(buf)))
            if len(buf) == size:
                return buf
            if self.verbose:
                print("waiting for %d bytes, got %d so far: [%s]" % (size, len(buf), ' '.join('%02X' % v for v in buf)))
        raise MTTimeoutException("waiting for message")

    def read_data_msg(self, buf=bytearray()):
        """Low-level MTData receiving function.
        Take advantage of known message length.
        """
        start = time.time()
        if self.length <= 254:
            totlength = 5 + self.length
        else:
            totlength = 7 + self.length
        while (time.time() - start) < self.timeout:
            buf.extend(self.waitfor(totlength - len(buf)))
            preamble_ind = buf.find(self.header)
            if preamble_ind == -1:  # not found
                # discard unexploitable data
                if self.verbose:
                    sys.stderr.write("MT: discarding (no preamble).\n")
                del buf[:-3]
                continue
            elif preamble_ind:  # found but not at start
                # discard leading bytes
                if self.verbose:
                    sys.stderr.write("MT: discarding (before preamble).\n")
                del buf[:preamble_ind]
                # complete message for checksum
                buf.extend(self.waitfor(totlength - len(buf)))
            if 0xFF & sum(buf[1:]):
                if self.verbose:
                    sys.stderr.write("MT: invalid checksum; discarding data and" " waiting for next message.\n")
                del buf[: buf.find(self.header) - 2]
                continue
            data = str(buf[-self.length - 1 : -1])
            del buf[:]
            return data
        else:
            raise MTException("could not find MTData message.")

    def read_msg(self):
        """Low-level message receiving function."""
        start = time.time()
        while (time.time() - start) < self.timeout:
            # first part of preamble
            if self.waitfor()[0] != 0xFA:
                continue
            # second part of preamble
            if self.waitfor()[0] != 0xFF:  # we assume no timeout anymore
                continue
            # read message id and length of message
            mid, length = struct.unpack('!BB', self.waitfor(2))
            if length == 255:  # extended length
                (length,) = struct.unpack('!H', self.waitfor(2))
            # read contents and checksum
            buf = self.waitfor(length + 1)
            checksum = buf[-1]
            data = struct.unpack('!%dB' % length, buf[:-1])
            # check message integrity
            if 0xFF & sum(data, 0xFF + mid + length + checksum):
                if self.verbose:
                    sys.stderr.write("invalid checksum; discarding data and " "waiting for next message.\n")
                continue
            if self.verbose:
                print(
                    "MT: Got message id 0x%02X (%s) with %d data bytes: [%s]"
                    % (
                        mid,
                        getMIDName(mid),
                        length,
                        ' '.join("%02X" % v for v in data),
                    )
                )
            if mid == MID.Error:
                raise MTErrorMessage(data[0])
            elif mid == MID.Warning:
                # Warning message format: uint32_t result_value + char string[128]
                if length >= 4:
                    result_value = struct.unpack('!I', buf[:4])[0]
                    warning_string = ""
                    if length > 4:
                        # Extract string part, removing null terminators
                        string_bytes = buf[4:-1]  # exclude checksum
                        warning_string = string_bytes.decode('ascii', errors='ignore').rstrip('\x00')
                    if self.verbose:
                        print(f"MT: Warning received: {MTWarningMessage(result_value, warning_string)}")
                    # Continue processing instead of raising an exception
                else:
                    if self.verbose:
                        print("MT: Malformed warning message received")
                # Don't return warning messages, continue reading next message
                continue
            return (mid, buf[:-1])
        else:
            raise MTException("could not find message.")

    def write_ack(self, mid, data=b'', n_retries=500):
        """Send a message and read confirmation."""
        self.write_msg(mid, data)

        # If no_ack mode is enabled, skip waiting for acknowledgment
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: skipping acknowledgment wait for MID 0x%02X" % mid)
            # Add a small delay to let the device process the command
            time.sleep(0.01)
            return b''  # Return empty data since we didn't read anything

        # Normal ACK waiting behavior
        for _ in range(n_retries):
            mid_ack, data_ack = self.read_msg()
            if mid_ack == (mid + 1):
                break
            elif self.verbose:
                print("ack (0x%02X) expected, got 0x%02X instead" % (mid + 1, mid_ack))
        else:
            raise MTException(
                "Ack (0x%02X) expected, MID 0x%02X received "
                "instead (after %d retries)." % (mid + 1, mid_ack, n_retries)
            )
        return data_ack

    def _ensure_config_state(self):
        """Switch device to config state if necessary."""
        if self.state != DeviceState.Config:
            self.GoToConfig()

    def _ensure_measurement_state(self):
        """Switch device to measurement state if necessary."""
        if self.state != DeviceState.Measurement:
            self.GoToMeasurement()

    ############################################################
    # High-level functions
    ############################################################
    def Reset(self, go_to_config=False):
        """Reset MT device.

        If go_to_config then send WakeUpAck in order to leave the device in
        config mode.
        """
        self.write_msg(MID.Reset)
        if go_to_config:
            time.sleep(0.01)
            mid, _ = self.read_msg()
            if mid == MID.WakeUp:
                self.write_msg(MID.WakeUpAck)
                self.state = DeviceState.Config
        else:
            self.state = DeviceState.Measurement

    def ExportXSA(self, filename):
        """
        Queries the device for its complete configuration and exports it
        to a .xsa file.
        """
        if not filename.endswith('.xsa'):
            print(f"XSA file {filename} is not a valid XSA file")
            return
        if self.no_ack:
            print("No-ack mode is enabled. Device will not respond with serial data.")
            return

        print(f"Exporting settings to {filename}...")
        self._ensure_config_state()

        # This map defines which settings to export. It maps the .xsa name
        # to the MID and the data required to request the setting.
        # For most, an empty payload is sent to request the data.
        # For some (like SetAlignmentRotation), a parameter is needed.
        export_map = {
            'AlignmentRotationLocal': (MID.SetAlignmentRotation, b'\x01'),
            'AlignmentRotationSensor': (MID.SetAlignmentRotation, b'\x00'),
            'CanConfig': (MID.SetCanConfig, b''),
            'FilterProfile': (MID.SetCurrentScenario, b''),
            'GnssLeverArm': (MID.SetGnssLeverArm, b''),
            'GnssReceiverType': (MID.SetGnssReceiverSettings, b''),
            'LatLonAlt': (MID.SetLatLonAlt, b''),
            'LocationId': (MID.SetLocationID, b''),
            'OptionFlags': (MID.SetOptionFlags, b''),
            'OutputConfiguration': (MID.SetOutputConfiguration, b''),
            'PortConfig': (MID.SetPortConfig, b''),
            'SyncConfiguration': (MID.SetSyncSettings, b''),
        }

        with open(filename, 'w') as f:
            # Write file header
            f.write("; XSA file created by MTDevice Python client\n")
            f.write(f"; Exported on: {time.ctime()}\n")

            # Write Info section
            try:
                fw_rev = self.GetFirmwareRev()
                f.write(f"Info.FirmwareVersion=S:{fw_rev[0]}.{fw_rev[1]}.{fw_rev[2]}\n")
                prod_code = self.GetProductCode()
                f.write(f"Info.ProductCode=S:{prod_code}\n")
            except MTException as e:
                print(f"Warning: Could not get device info: {e}")

            # Write Config section
            for name, (mid, req_data) in export_map.items():
                try:
                    # Get the configuration data from the device
                    ack_data = self.write_ack(mid, req_data)
                    data_payload_for_file = ack_data  # default assumption

                    # Handle the special case for OptionFlags
                    if name == 'OptionFlags':
                        # The SetOptionFlags message requires an 8-byte payload:
                        # 4 bytes for set_flags, 4 bytes for clear_flags.
                        # The ack_data is only the current 4-byte state.
                        # To recreate the exact flag state, we clear all flags first,
                        # then set only the desired flags.
                        set_mask = ack_data  # The flags we want to set
                        clear_mask = b'\xff\xff\xff\xff'  # Clear all flags first
                        data_payload_for_file = set_mask + clear_mask

                    # Construct the full message that would be sent to set this value
                    length = len(data_payload_for_file)
                    if length > 254:
                        lendat = b'\xff' + struct.pack('!H', length)
                    else:
                        lendat = struct.pack('!B', length)

                    # Reconstruct the Xbus message format for the .xsa file
                    # This packet doesn't actually get sent, just formatted
                    packet_core = b'\xfa\xff' + struct.pack('!B', mid) + lendat + data_payload_for_file
                    checksum_val = 0xFF & (-(sum(packet_core[1:])))
                    checksum = struct.pack('!B', checksum_val)
                    full_message = packet_core + checksum

                    hex_string = full_message.hex().upper()
                    f.write(f"Config.{name}=N:{len(full_message)}:{hex_string}\n")
                    if self.verbose:
                        print(f"  Successfully exported {name}")

                except MTException as e:
                    if self.verbose:
                        print(f"  Could not export {name}: {e}")
        print("Export complete.")

    def ImportXSA(self, filename):
        """
        Imports and applies settings from a .xsa file to the device.
        """
        if not os.path.exists(filename):
            print(f"XSA file {filename} does not exist")
            return
        if not filename.endswith('.xsa'):
            print(f"XSA file {filename} is not a valid XSA file")
            return

        print(f"Importing settings from {filename}...")
        self._ensure_config_state()

        with open(filename, 'r') as f:
            for line in f:
                # Match lines starting with "Config."
                match = re.match(r'Config\.(.+)=N:\d+:(.+)', line)
                if not match:
                    continue

                setting_name, hex_data = match.groups()
                hex_data = hex_data.strip()
                try:
                    # Convert the full hex message from the file into bytes
                    full_message_bytes = bytes.fromhex(hex_data)

                    # Deconstruct the message to get the MID and data payload
                    # Format: FA FF MID LEN [Extended_LEN] DATA CS
                    if len(full_message_bytes) < 5:
                        print(f"    Skipping malformed message for {setting_name}")
                        continue

                    mid = full_message_bytes[2]
                    length = full_message_bytes[3]
                    offset = 4
                    if length == 255:  # Extended length
                        (length,) = struct.unpack('!H', full_message_bytes[4:6])
                        offset = 6

                    data_payload = full_message_bytes[offset:-1]

                    # Send the configuration command with its data payload
                    self.write_ack(mid, data_payload)
                    print(f"    Successfully applied {setting_name}")

                except ValueError:
                    print(f"    ERROR: Invalid hexadecimal data for {setting_name}")
                except MTException as e:
                    print(f"    ERROR applying {setting_name}: {e}")

        print("Import complete. Resetting device to apply settings.")
        self.Reset()
        self.device.flush()
        time.sleep(0.01)
        if not self.no_ack:
            self.read_msg()
        self.write_msg(MID.WakeUpAck)

    def GoToConfig(self):
        """Place MT device in configuration mode."""
        self.write_ack(MID.GoToConfig)
        self.state = DeviceState.Config

    def GoToMeasurement(self):
        """Place MT device in measurement mode."""
        self._ensure_config_state()
        self.write_ack(MID.GoToMeasurement)
        self.state = DeviceState.Measurement

    def GetDeviceID(self):
        """Get the device identifier."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqDID)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read device ID")
            return 0  # Return dummy value
        (deviceID,) = struct.unpack('!Q', data)
        return deviceID

    def GetProductCode(self):
        """Get the product code."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqProductCode)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read product code")
            return "N/A (no-ack mode)"
        # Properly decode the product code
        if isinstance(data, (bytes, bytearray)):
            return data.decode('ascii').strip('\x00 ')
        return str(data).strip()

    def GetFirmwareRev(self):
        """Get the firmware version."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqFWRev)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read firmware revision")
            return (0, 0, 0)  # Return dummy values
        # XXX unpacking only 3 characters in accordance with the documentation
        # but some devices send 11 bytes instead.
        major, minor, revision = struct.unpack('!BBB', data[:3])
        return (major, minor, revision)

    def RunSelfTest(self):
        """Run the built-in self test."""
        self._ensure_config_state()
        data = self.write_ack(MID.RunSelfTest)
        bit_names = ['accX', 'accY', 'accZ', 'gyrX', 'gyrY', 'gyrZ', 'magX', 'magY', 'magZ']
        self_test_results = []
        for i, name in enumerate(bit_names):
            self_test_results.append((name, (data >> i) & 1))
        return self_test_results

    def GetBaudrate(self):
        """Get the current baudrate id of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetBaudrate)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read baudrate")
            return 0  # Return dummy value
        return data[0]

    def SetBaudrate(self, brid):
        """Set the baudrate of the device using the baudrate id."""
        self._ensure_config_state()
        data = struct.pack('!B', brid)
        self.write_ack(MID.SetBaudrate, data)

    def GetCanConfig(self):
        self._ensure_config_state()
        data = self.write_ack(MID.SetCanConfig)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read CAN config")
            return 0  # Return dummy value
        (config_word,) = struct.unpack('!I', data)
        return config_word

    def SetCanConfig(self, baudrate):
        self._ensure_config_state()
        if baudrate == 0:
            # Disable CAN
            config_word = 0
        else:
            # Enable CAN with specified baudrate
            brid = Baudrates.get_can_BRID(baudrate)
            config_word = (1 << 8) | brid  # Set bit 8 (enable) and bits 7:0 (baudrate code)
        data = struct.pack('!I', config_word)
        self.write_ack(MID.SetCanConfig, data)

    def GetCanOutputConfig(self):
        self._ensure_config_state()
        data = self.write_ack(MID.SetCanOutputConfig)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read CAN output config")
            return b''  # Return empty bytes
        return data

    def SetCanOutputConfig(self, output_configuration):
        self._ensure_config_state()
        data = b''.join(struct.pack('!BBIH', *output) for output in output_configuration)
        self.write_ack(MID.SetCanOutputConfig, data)

    def GetGnssReceiverSettings(self):
        self._ensure_config_state()
        data = self.write_ack(MID.SetGnssReceiverSettings)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read GNSS receiver")
            return b''  # Return empty bytes
        return data

    def GetHardwareVersion(self):
        self._ensure_config_state()
        data = self.write_ack(MID.SetHardwareVersion)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read hardware version")
            return b''  # Return empty bytes
        return data

    def GetPortConfig(self):
        self._ensure_config_state()
        data = self.write_ack(MID.SetPortConfig)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read port config")
            return 115200, 0, 38400  # Return dummy values (xbus_baud, xbus_flow, rtcm_baud)

        # Initialize default values
        xbus_baud = 115200
        xbus_flow = 0
        rtcm_baud = 38400

        port_config = [struct.unpack('!BBBB', data[o : o + 4]) for o in range(0, len(data), 4)]
        for port in port_config:
            protocol = port[1]
            flowcontrol = port[2]
            brid = port[3]
            if protocol == MID.XbuxInterface:
                try:
                    xbus_baud = Baudrates.get_BR(brid)
                except MTException:
                    if self.verbose:
                        print(f"  Warning: Unknown XBus baudrate ID: 0x{brid:02X}")
                    xbus_baud = f"Unknown (ID: 0x{brid:02X})"
                xbus_flow = flowcontrol
            elif protocol == MID.RtcmInterface:
                try:
                    rtcm_baud = Baudrates.get_BR(brid)
                except MTException:
                    if self.verbose:
                        print(f"  Warning: Unknown RTCM baudrate ID: 0x{brid:02X}")
                    rtcm_baud = f"Unknown (ID: 0x{brid:02X})"
        return xbus_baud, xbus_flow, rtcm_baud

    def SetPortConfig(self, xbus_brid, xbus_flow, rtcm_brid):
        """Set the baudrate of the device using the baudrate id."""
        self._ensure_config_state()
        data = struct.pack(
            '!HBBHBBHBB',
            MID.XbuxInterface,
            int(xbus_flow),
            xbus_brid,
            0x00,
            0x00,
            0x00,
            MID.RtcmInterface,
            0x00,
            rtcm_brid,
        )
        self.write_ack(MID.SetPortConfig, data)

    def GetOptionFlags(self):
        """Get the option flags (MTi-1 series)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOptionFlags)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read option flags")
            return (0, 0)  # Return dummy tuple
        # Response contains 4 bytes: current option flags
        if len(data) == 4:
            (current_flags,) = struct.unpack('!I', data)
            return (current_flags, 0)  # Return current flags and 0 for clear flags
        else:
            raise MTException(f"Unexpected option flags response length: {len(data)} bytes")

    def SetOptionFlags(self, set_flags, clear_flags):
        """Set the option flags (MTi-1 series)."""
        self._ensure_config_state()
        # Pack 8 bytes: SetFlags (4 bytes) + ClearFlags (4 bytes)
        data = struct.pack('!II', set_flags, clear_flags)
        self.write_ack(MID.SetOptionFlags, data)

    def GetLocationID(self):
        """Get the location ID of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetLocationID)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read location ID")
            return 0  # Return dummy value
        (location_id,) = struct.unpack('!H', data)
        return location_id

    def SetLocationID(self, location_id):
        """Set the location ID of the device (arbitrary)."""
        self._ensure_config_state()
        data = struct.pack('!H', location_id)
        self.write_ack(MID.SetLocationID, data)

    def RestoreFactoryDefaults(self):
        """Restore MT device configuration to factory defaults (soft version)."""
        self._ensure_config_state()
        self.write_ack(MID.RestoreFactoryDef)
        self.Reset()

    def GetTransmitDelay(self):
        """Get the transmission delay (only RS485)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetTransmitDelay)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read transmit delay")
            return 0  # Return dummy value
        (transmit_delay,) = struct.unpack('!H', data)
        return transmit_delay

    def SetTransmitDelay(self, transmit_delay):
        """Set the transmission delay (only RS485)."""
        self._ensure_config_state()
        data = struct.pack('!H', transmit_delay)
        self.write_ack(MID.SetTransmitDelay, data)

    def GetSyncSettings(self):
        """Get the synchronisation settings."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetSyncSettings)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read sync settings")
            return []  # Return empty list
        sync_settings = [struct.unpack('!BBBBHHHH', data[o : o + 12]) for o in range(0, len(data), 12)]
        return sync_settings

    def SetSyncSettings(self, sync_settings):
        """Set the synchronisation settings (mark IV)"""
        self._ensure_config_state()
        data = b''.join(struct.pack('!BBBBHHHH', *sync_setting) for sync_setting in sync_settings)
        self.write_ack(MID.SetSyncSettings, data)

    def GetConfiguration(self):
        """Ask for the current configuration of the MT device.

        Supports both formats:
        - Older MTi devices (118 bytes): include period, skip factor, output mode/settings
        - MTi-600s devices (114 bytes): 8-byte device IDs, no legacy output fields
        """
        self._ensure_config_state()
        config = self.write_ack(MID.ReqConfiguration)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read configuration")
            return {
                'device_type': 'unknown',
                'output-mode': 0,
                'output-settings': 0,
                'length': 0,
                'period': 0,
                'skipfactor': 0,
                'sync_mode': 0,
                'sync_skip_factor': 0,
                'sync_offset': 0,
                'Master device ID': 0,
                'date': b'',
                'time': b'',
                'number of devices': 0,
                'device ID': 0,
            }

        # Determine format based on message length
        config_len = len(config)

        if config_len == 118:
            # Older MTi devices format (118 bytes)
            try:
                (
                    masterID,
                    period,
                    skipfactor,
                    sync_mode,
                    sync_skip_factor,
                    sync_offset,
                    date,
                    time,
                    num,
                    deviceID,
                    length,
                    mode,
                    settings,
                ) = struct.unpack('!IHHHHI8s8s32x32xHIHHI8x', config)
                device_type = 'legacy'
            except struct.error:
                raise MTException("could not parse legacy MTi configuration (118 bytes).")

        elif config_len == 114:
            # MTi-600s format (114 bytes)
            try:
                (masterID, sync_mode, sync_skip_factor, sync_offset, date, time, num, deviceID) = struct.unpack(
                    '!QHHI8s8s32x32xHQ12x', config
                )
                # MTi-600s doesn't have these legacy fields
                period = 0
                skipfactor = 0
                length = 0
                mode = 0
                settings = 0
                device_type = 'mti600s'
            except struct.error:
                raise MTException("could not parse MTi-600s configuration (114 bytes).")
        else:
            raise MTException(f"unknown configuration format: {config_len} bytes (expected 114 or 118).")

        # Store relevant fields in instance
        self.mode = mode
        self.settings = settings
        self.length = length
        if self.length <= 254:
            self.header = b'\xfa\xff\x32' + struct.pack('!B', self.length)
        else:
            self.header = b'\xfa\xff\x32\xff' + struct.pack('!H', self.length)

        conf = {
            'device_type': device_type,
            'output-mode': mode,
            'output-settings': settings,
            'length': length,
            'period': period,
            'skipfactor': skipfactor,
            'sync_mode': sync_mode,
            'sync_skip_factor': sync_skip_factor,
            'sync_offset': sync_offset,
            'Master device ID': masterID,
            'date': date,
            'time': time,
            'number of devices': num,
            'device ID': deviceID,
        }
        return conf

    def GetOutputConfiguration(self):
        """Get the output configuration of the device (mark IV).

        This method is the modern replacement for the removed legacy methods
        GetPeriod, GetOutputMode, and GetOutputSettings.

        Returns a list of (data_identifier, frequency) tuples showing the current
        output configuration.
        """
        self._ensure_config_state()
        data = self.write_ack(MID.SetOutputConfiguration)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read output configuration")
            return []  # Return empty list
        output_configuration = [struct.unpack('!HH', data[o : o + 4]) for o in range(0, len(data), 4)]
        return output_configuration

    def SetOutputConfiguration(self, output_configuration):
        """Set the output configuration of the device (mark IV).

        This is the modern way to configure output on MTi devices. It replaces the
        following removed legacy methods:
        - SetPeriod (0x04)
        - SetOutputSkipFactor (0xD4)
        - SetOutputMode (0xD0)
        - SetOutputSettings (0xD2)

        The data is a list of up to 32 data identifiers combined with desired output frequencies.
        Each entry is (data_identifier, frequency) where frequency of 0x0000 or 0xFFFF
        selects maximum frequency for the given data identifier.
        """
        self._ensure_config_state()
        data = b''.join(struct.pack('!HH', *output) for output in output_configuration)
        self.write_ack(MID.SetOutputConfiguration, data)

    def GetStringOutputType(self):
        """Get the NMEA data output configuration."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetStringOutputType)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read string output type")
            return 0  # Return dummy value
        (string_output_type,) = struct.unpack('!H', data[-2:])
        return string_output_type

    def SetStringOutputType(self, string_output_type):
        """Set the configuration of the NMEA data output."""
        self._ensure_config_state()
        data = struct.pack('!H', string_output_type)
        self.write_ack(MID.SetStringOutputType, data)

    def GetAlignmentRotation(self, parameter):
        """Get the object alignment.

        parameter indicates the desired alignment quaternion:
            0 for sensor alignment (RotSensor),
            1 for local alignment (RotLocal).
        """
        self._ensure_config_state()
        data = struct.pack('!B', parameter)
        data = self.write_ack(MID.SetAlignmentRotation, data)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read alignment rotation")
            return 0, 0.0, 0.0, 0.0, 0.0  # Return dummy values
        frame, q0, q1, q2, q3 = struct.unpack('!Bffff', data)
        return frame, q0, q1, q2, q3

    def SetAlignmentRotation(self, parameter, quaternion):
        """Set the object alignment.

        parameter indicates the desired alignment quaternion:
            0 for sensor alignment (RotSensor),
            1 for local alignment (RotLocal).
        """
        self._ensure_config_state()
        data = struct.pack('!Bffff', parameter, *quaternion)
        self.write_ack(MID.SetAlignmentRotation, data)

    def GetExtOutputMode(self):
        """Get current extended output mode (for alternative UART)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetExtOutputMode)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read extended output mode")
            return 0  # Return dummy value
        (ext_mode,) = struct.unpack('!H', data)
        return ext_mode

    def SetExtOutputMode(self, ext_mode):
        """Set extended output mode (for alternative UART)."""
        self._ensure_config_state()
        data = struct.pack('!H', ext_mode)
        self.write_ack(MID.SetExtOutputMode, data)

    def ReqDataLength(self):  # deprecated
        """Get data length for mark III devices."""
        self._ensure_config_state()
        try:
            data = self.write_ack(DeprecatedMID.ReqDataLength)
        except MTErrorMessage as e:
            if e.code == 0x04:
                sys.stderr.write("ReqDataLength message is deprecated and not " "recognised by your device.")
                return
            raise e
        (self.length,) = struct.unpack('!H', data)
        if self.length <= 254:
            self.header = b'\xfa\xff\x32' + struct.pack('!B', self.length)
        else:
            self.header = b'\xfa\xff\x32\xff' + struct.pack('!H', self.length)
        return self.length

    def GetLatLonAlt(self):
        """Get the stored position of the device.
        It is used internally for local magnetic declination and local gravity.
        """
        self._ensure_config_state()
        data = self.write_ack(MID.SetLatLonAlt)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read lat/lon/alt")
            return (0.0, 0.0, 0.0)  # Return dummy values
        if len(data) == 24:
            lat, lon, alt = struct.unpack('!ddd', data)
        elif len(data) == 12:
            lat, lon, alt = struct.unpack('!fff', data)
        else:
            raise MTException('Could not parse ReqLatLonAltAck message: wrong' 'size of message.')
        return (lat, lon, alt)

    def SetLatLonAlt(self, lat, lon, alt):
        """Set the position of the device.
        It is used internally for local magnetic declination and local gravity.
        """
        self._ensure_config_state()
        data = struct.pack('!ddd', lat, lon, alt)
        self.write_ack(MID.SetLatLonAlt, data)

    def GetGnssLeverArm(self):
        """Get the lever arm of the GNSS receiver."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetGnssLeverArm)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read GNSS lever arm")
            return (0.0, 0.0, 0.0)  # Return dummy values
        if len(data) == 24:
            x, y, z = struct.unpack('!ddd', data)
        elif len(data) == 12:
            x, y, z = struct.unpack('!fff', data)
        else:
            raise MTException(
                'Could not parse GetGnssLeverArmAck message: wrong size of message (%d bytes).' % len(data)
            )
        return (x, y, z)

    def SetGnssLeverArm(self, x, y, z):
        """Set the lever arm of the GNSS receiver."""
        self._ensure_config_state()
        data = struct.pack('!fff', x, y, z)
        self.write_ack(MID.SetGnssLeverArm, data)

    def GetAvailableScenarios(self):
        """Get the available XKF scenarios on the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqAvailableScenarios)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read available scenarios")
            return []  # Return empty list
        scenarios = []
        try:
            for i in range(len(data) // 22):
                scenario_type, version, label = struct.unpack('!BB20s', data[22 * i : 22 * (i + 1)])
                scenarios.append((scenario_type, version, label.strip()))
            # available XKF scenarios
            self.scenarios = scenarios
        except struct.error:
            raise MTException("could not parse the available XKF scenarios.")
        return scenarios

    def GetCurrentScenario(self):
        """Get the currently used XKF scenario filter profile(s)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetCurrentScenario)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read current scenario")
            return "N/A (no-ack mode)"  # Return dummy value
        if isinstance(data, (bytes, bytearray)):
            scenario_string = data.decode('ascii').strip('\x00 ')
        else:
            scenario_string = str(data)
        return scenario_string

    def SetCurrentScenario(self, scenario_id):
        """Sets the XKF scenario to use."""
        self._ensure_config_state()
        data = struct.pack('!BB', 0, scenario_id)  # version, id
        self.write_ack(MID.SetCurrentScenario, data)

    def ResetOrientation(self, code):
        """Reset the orientation.

        Code can take several values:
            0x0000: store current settings (only in config mode),
            0x0001: heading reset (NOT supported by MTi-G),
            0x0003: object reset.
        """
        data = struct.pack('!H', code)
        self.write_ack(MID.ResetOrientation, data)

    def SetNoRotation(self, duration):
        """Initiate the "no rotation" procedure to estimate gyro biases."""
        self._ensure_measurement_state()
        data = struct.pack('!H', duration)
        self.write_ack(MID.SetNoRotation, data)

    def GetUTCTime(self):
        """Get UTC time from device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetUTCTime)
        if self.no_ack:
            if self.verbose:
                print("  No-ACK mode: cannot read UTC time")
            return (0, 0, 0, 0, 0, 0, 0, 0)  # Return dummy values
        ns, year, month, day, hour, minute, second, flag = struct.unpack('!IHBBBBBB', data)
        return (ns, year, month, day, hour, minute, second, flag)

    def SetUTCTime(self, ns, year, month, day, hour, minute, second, flag):
        """Set UTC time on the device."""
        self._ensure_config_state()
        data = struct.pack(
            '!IHBBBBBB', ns, year, month, day, hour, minute, second, flag
        )  # no clue what setting flag can mean
        self.write_ack(MID.SetUTCTime, data)

    def AdjustUTCTime(self, ticks):
        """Adjust UTC Time of device using correction ticks (0.1 ms)."""
        self._ensure_config_state()
        data = struct.pack('!i', ticks)
        self.write(MID.AdjustUTCTime, data)  # no ack mentioned in the doc

    ############################################################
    # High-level utility functions
    ############################################################
    def configure_legacy(self, mode, settings, period=None, skipfactor=None):
        """Configure the mode and settings of the MT device in legacy mode."""
        try:
            # switch mark IV devices to legacy mode
            self.SetOutputConfiguration([(0x0000, 0)])
        except MTErrorMessage as e:
            if e.code == 0x04:
                # mark III device
                pass
            else:
                raise
        except MTException as e:
            if self.verbose:
                print("no ack received while switching from MTData2 to MTData: %s" % e)
            pass  # no ack???
        self.GetConfiguration()

    def auto_config_legacy(self):
        """Read configuration from device in legacy mode."""
        self.GetConfiguration()
        return self.mode, self.settings, self.length

    def read_measurement(self, mode=None, settings=None):
        self._ensure_measurement_state()
        # getting data
        # data = self.read_data_msg()
        mid, data = self.read_msg()
        if mid == MID.MTData:
            return self.parse_MTData(data, mode, settings)
        elif mid == MID.MTData2:
            return self.parse_MTData2(data)
        else:
            raise MTException("unknown data message: mid=0x%02X (%s)." % (mid, getMIDName(mid)))

    def parse_MTData2(self, data):
        # Functions to parse each type of packet
        def parse_temperature(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Temperature
                (o['Temp'],) = struct.unpack('!' + ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_timestamp(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # UTC Time
                o['ns'], o['Year'], o['Month'], o['Day'], o['Hour'], o['Minute'], o['Second'], o['Flags'] = (
                    struct.unpack('!LHBBBBBB', content)
                )
            elif (data_id & 0x00F0) == 0x20:  # Packet Counter
                (o['PacketCounter'],) = struct.unpack('!H', content)
            elif (data_id & 0x00F0) == 0x30:  # Integer Time of Week
                (o['TimeOfWeek'],) = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x40:  # GPS Age  # deprecated
                (o['gpsAge'],) = struct.unpack('!B', content)
            elif (data_id & 0x00F0) == 0x50:  # Pressure Age  # deprecated
                (o['pressureAge'],) = struct.unpack('!B', content)
            elif (data_id & 0x00F0) == 0x60:  # Sample Time Fine
                (o['SampleTimeFine'],) = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x70:  # Sample Time Coarse
                (o['SampleTimeCoarse'],) = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x80:  # Frame Range
                o['startFrame'], o['endFrame'] = struct.unpack('!HH', content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_orientation_data(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Quaternion
                o['Q0'], o['Q1'], o['Q2'], o['Q3'] = struct.unpack('!' + 4 * ffmt, content)
            elif (data_id & 0x00F0) == 0x20:  # Rotation Matrix
                o['a'], o['b'], o['c'], o['d'], o['e'], o['f'], o['g'], o['h'], o['i'] = struct.unpack(
                    '!' + 9 * ffmt, content
                )
            elif (data_id & 0x00F0) == 0x30:  # Euler Angles
                o['Roll'], o['Pitch'], o['Yaw'] = struct.unpack('!' + 3 * ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_pressure(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Baro pressure
                (o['Pressure'],) = struct.unpack('!L', content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_acceleration(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Delta V
                o['Delta v.x'], o['Delta v.y'], o['Delta v.z'] = struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x20:  # Acceleration
                o['accX'], o['accY'], o['accZ'] = struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Free Acceleration
                o['freeAccX'], o['freeAccY'], o['freeAccZ'] = struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # AccelerationHR
                o['accX'], o['accY'], o['accZ'] = struct.unpack('!' + 3 * ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_position(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Altitude MSL  # deprecated
                (o['altMsl'],) = struct.unpack('!' + ffmt, content)
            elif (data_id & 0x00F0) == 0x20:  # Altitude Ellipsoid
                (o['altEllipsoid'],) = struct.unpack('!' + ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Position ECEF
                o['ecefX'], o['ecefY'], o['ecefZ'] = struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # LatLon
                o['lat'], o['lon'] = struct.unpack('!' + 2 * ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_GNSS(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # GNSS PVT data
                (
                    o['itow'],
                    o['year'],
                    o['month'],
                    o['day'],
                    o['hour'],
                    o['min'],
                    o['sec'],
                    o['valid'],
                    o['tAcc'],
                    o['nano'],
                    o['fixtype'],
                    o['flags'],
                    o['numSV'],
                    o['lon'],
                    o['lat'],
                    o['height'],
                    o['hMSL'],
                    o['hAcc'],
                    o['vAcc'],
                    o['velN'],
                    o['velE'],
                    o['velD'],
                    o['gSpeed'],
                    o['headMot'],
                    o['sAcc'],
                    o['headAcc'],
                    o['headVeh'],
                    o['gdop'],
                    o['pdop'],
                    o['tdop'],
                    o['vdop'],
                    o['hdop'],
                    o['ndop'],
                    o['edop'],
                ) = struct.unpack('!IHBBBBBBIiBBBBiiiiIIiiiiiIIiHHHHHHH', content)
                # scaling correction
                o['lon'] *= 1e-7
                o['lat'] *= 1e-7
                o['headMot'] *= 1e-5
                o['headVeh'] *= 1e-5
                o['gdop'] *= 0.01
                o['pdop'] *= 0.01
                o['tdop'] *= 0.01
                o['vdop'] *= 0.01
                o['hdop'] *= 0.01
                o['bdop'] *= 0.01
                o['edop'] *= 0.01
            elif (data_id & 0x00F0) == 0x20:  # GNSS satellites info
                o['iTOW'], o['numSvs'] = struct.unpack('!LBxxx', content[:8])
                svs = []
                ch = {}
                for i in range(o['numSvs']):
                    ch['gnssId'], ch['svId'], ch['cno'], ch['flags'] = struct.unpack(
                        '!BBBB', content[8 + 4 * i : 12 + 4 * i]
                    )
                    svs.append(ch)
                o['svs'] = svs
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_angular_velocity(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x20:  # Rate of Turn
                o['gyrX'], o['gyrY'], o['gyrZ'] = struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Delta Q
                o['Delta q0'], o['Delta q1'], o['Delta q2'], o['Delta q3'] = struct.unpack('!' + 4 * ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # RateOfTurnHR
                o['gyrX'], o['gyrY'], o['gyrZ'] = struct.unpack('!' + 3 * ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_GPS(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x30:  # DOP
                o['iTOW'], g, p, t, v, h, n, e = struct.unpack('!LHHHHHHH', content)
                o['gDOP'], o['pDOP'], o['tDOP'], o['vDOP'], o['hDOP'], o['nDOP'], o['eDOP'] = (
                    0.01 * g,
                    0.01 * p,
                    0.01 * t,
                    0.01 * v,
                    0.01 * h,
                    0.01 * n,
                    0.01 * e,
                )
            elif (data_id & 0x00F0) == 0x40:  # SOL
                (
                    o['iTOW'],
                    o['fTOW'],
                    o['Week'],
                    o['gpsFix'],
                    o['Flags'],
                    o['ecefX'],
                    o['ecefY'],
                    o['ecefZ'],
                    o['pAcc'],
                    o['ecefVX'],
                    o['ecefVY'],
                    o['ecefVZ'],
                    o['sAcc'],
                    o['pDOP'],
                    o['numSV'],
                ) = struct.unpack('!LlhBBlllLlllLHxBx', content)
                # scaling correction
                o['pDOP'] *= 0.01
            elif (data_id & 0x00F0) == 0x80:  # Time UTC
                (
                    o['iTOW'],
                    o['tAcc'],
                    o['nano'],
                    o['year'],
                    o['month'],
                    o['day'],
                    o['hour'],
                    o['min'],
                    o['sec'],
                    o['valid'],
                ) = struct.unpack('!LLlHBBBBBB', content)
            elif (data_id & 0x00F0) == 0xA0:  # SV Info
                o['iTOW'], o['numCh'] = struct.unpack('!LBxxx', content[:8])
                channels = []
                ch = {}
                for i in range(o['numCh']):
                    (
                        ch['chn'],
                        ch['svid'],
                        ch['flags'],
                        ch['quality'],
                        ch['cno'],
                        ch['elev'],
                        ch['azim'],
                        ch['prRes'],
                    ) = struct.unpack('!BBBBBbhl', content[8 + 12 * i : 20 + 12 * i])
                    channels.append(ch)
                o['channels'] = channels
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_SCR(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # ACC+GYR+MAG+Temperature
                (
                    o['accX'],
                    o['accY'],
                    o['accZ'],
                    o['gyrX'],
                    o['gyrY'],
                    o['gyrZ'],
                    o['magX'],
                    o['magY'],
                    o['magZ'],
                    o['Temp'],
                ) = struct.unpack("!9Hh", content)
            elif (data_id & 0x00F0) == 0x20:  # Gyro Temperature
                o['tempGyrX'], o['tempGyrY'], o['tempGyrZ'] = struct.unpack("!hhh", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_analog_in(data_id, content, ffmt):  # deprecated
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Analog In 1
                (o['analogIn1'],) = struct.unpack("!H", content)
            elif (data_id & 0x00F0) == 0x20:  # Analog In 2
                (o['analogIn2'],) = struct.unpack("!H", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_magnetic(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x20:  # Magnetic Field
                o['magX'], o['magY'], o['magZ'] = struct.unpack("!3" + ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_velocity(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Velocity XYZ
                o['velX'], o['velY'], o['velZ'] = struct.unpack("!3" + ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_status(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Status Byte
                (o['StatusByte'],) = struct.unpack("!B", content)
            elif (data_id & 0x00F0) == 0x20:  # Status Word
                (o['StatusWord'],) = struct.unpack("!L", content)
            elif (data_id & 0x00F0) == 0x40:  # RSSI  # deprecated
                (o['RSSI'],) = struct.unpack("!b", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        # data object
        output = {}
        while data:
            try:
                data_id, size = struct.unpack('!HB', data[:3])
                if (data_id & 0x0003) == 0x3:
                    float_format = 'd'
                elif (data_id & 0x0003) == 0x0:
                    float_format = 'f'
                else:
                    raise MTException("fixed point precision not supported.")
                content = data[3 : 3 + size]
                data = data[3 + size :]
                group = data_id & 0xF800
                ffmt = float_format
                if group == XDIGroup.Temperature:
                    output.setdefault('Temperature', {}).update(parse_temperature(data_id, content, ffmt))
                elif group == XDIGroup.Timestamp:
                    output.setdefault('Timestamp', {}).update(parse_timestamp(data_id, content, ffmt))
                elif group == XDIGroup.OrientationData:
                    output.setdefault('Orientation Data', {}).update(parse_orientation_data(data_id, content, ffmt))
                elif group == XDIGroup.Pressure:
                    output.setdefault('Pressure', {}).update(parse_pressure(data_id, content, ffmt))
                elif group == XDIGroup.Acceleration:
                    output.setdefault('Acceleration', {}).update(parse_acceleration(data_id, content, ffmt))
                elif group == XDIGroup.Position:
                    output.setdefault('Position', {}).update(parse_position(data_id, content, ffmt))
                elif group == XDIGroup.GNSS:
                    output.setdefault('GNSS', {}).update(parse_GNSS(data_id, content, ffmt))
                elif group == XDIGroup.AngularVelocity:
                    output.setdefault('Angular Velocity', {}).update(parse_angular_velocity(data_id, content, ffmt))
                elif group == XDIGroup.GPS:
                    output.setdefault('GPS', {}).update(parse_GPS(data_id, content, ffmt))
                elif group == XDIGroup.SensorComponentReadout:
                    output.setdefault('SCR', {}).update(parse_SCR(data_id, content, ffmt))
                elif group == XDIGroup.AnalogIn:  # deprecated
                    output.setdefault('Analog In', {}).update(parse_analog_in(data_id, content, ffmt))
                elif group == XDIGroup.Magnetic:
                    output.setdefault('Magnetic', {}).update(parse_magnetic(data_id, content, ffmt))
                elif group == XDIGroup.Velocity:
                    output.setdefault('Velocity', {}).update(parse_velocity(data_id, content, ffmt))
                elif group == XDIGroup.Status:
                    output.setdefault('Status', {}).update(parse_status(data_id, content, ffmt))
                else:
                    raise MTException("unknown XDI group: 0x%04X." % group)
            except struct.error:
                raise MTException("couldn't parse MTData2 message.")
        return output

    def parse_MTData(self, data, mode=None, settings=None):
        """Read and parse a legacy measurement packet."""
        # getting mode
        if mode is None:
            mode = self.mode
        if settings is None:
            settings = self.settings
        # data object
        output = {}
        try:
            # raw IMU first
            if mode & OutputMode.RAW:
                o = {}
                (
                    o['accX'],
                    o['accY'],
                    o['accZ'],
                    o['gyrX'],
                    o['gyrY'],
                    o['gyrZ'],
                    o['magX'],
                    o['magY'],
                    o['magZ'],
                    o['temp'],
                ) = struct.unpack('!10H', data[:20])
                data = data[20:]
                output['RAW'] = o
            # raw GPS second
            if mode & OutputMode.RAWGPS:
                o = {}
                (
                    o['Press'],
                    o['bPrs'],
                    o['ITOW'],
                    o['LAT'],
                    o['LON'],
                    o['ALT'],
                    o['VEL_N'],
                    o['VEL_E'],
                    o['VEL_D'],
                    o['Hacc'],
                    o['Vacc'],
                    o['Sacc'],
                    o['bGPS'],
                ) = struct.unpack('!HBI6i3IB', data[:44])
                data = data[44:]
                output['RAWGPS'] = o
            # temperature
            if mode & OutputMode.Temp:
                (temp,) = struct.unpack('!f', data[:4])
                data = data[4:]
                output['Temp'] = temp
            # calibrated data
            if mode & OutputMode.Calib:
                o = {}
                if settings & OutputSettings.Coordinates_NED:
                    o['frame'] = 'NED'
                else:
                    o['frame'] = 'ENU'
                if not (settings & OutputSettings.CalibMode_GyrMag):
                    o['accX'], o['accY'], o['accZ'] = struct.unpack('!3f', data[:12])
                    data = data[12:]
                if not (settings & OutputSettings.CalibMode_AccMag):
                    o['gyrX'], o['gyrY'], o['gyrZ'] = struct.unpack('!3f', data[:12])
                    data = data[12:]
                if not (settings & OutputSettings.CalibMode_AccGyr):
                    o['magX'], o['magY'], o['magZ'] = struct.unpack('!3f', data[:12])
                    data = data[12:]
                output['Calib'] = o
            # orientation
            if mode & OutputMode.Orient:
                o = {}
                if settings & OutputSettings.Coordinates_NED:
                    o['frame'] = 'NED'
                else:
                    o['frame'] = 'ENU'
                if settings & OutputSettings.OrientMode_Euler:
                    o['roll'], o['pitch'], o['yaw'] = struct.unpack('!3f', data[:12])
                    data = data[12:]
                elif settings & OutputSettings.OrientMode_Matrix:
                    a, b, c, d, e, f, g, h, i = struct.unpack('!9f', data[:36])
                    data = data[36:]
                    o['matrix'] = ((a, b, c), (d, e, f), (g, h, i))
                else:  # OutputSettings.OrientMode_Quaternion:
                    q0, q1, q2, q3 = struct.unpack('!4f', data[:16])
                    data = data[16:]
                    o['quaternion'] = (q0, q1, q2, q3)
                output['Orient'] = o
            # auxiliary
            if mode & OutputMode.Auxiliary:
                o = {}
                if not (settings & OutputSettings.AuxiliaryMode_NoAIN1):
                    (o['Ain_1'],) = struct.unpack('!H', data[:2])
                    data = data[2:]
                if not (settings & OutputSettings.AuxiliaryMode_NoAIN2):
                    (o['Ain_2'],) = struct.unpack('!H', data[:2])
                    data = data[2:]
                output['Auxiliary'] = o
            # position
            if mode & OutputMode.Position:
                o = {}
                o['Lat'], o['Lon'], o['Alt'] = struct.unpack('!3f', data[:12])
                data = data[12:]
                output['Pos'] = o
            # velocity
            if mode & OutputMode.Velocity:
                o = {}
                if settings & OutputSettings.Coordinates_NED:
                    o['frame'] = 'NED'
                else:
                    o['frame'] = 'ENU'
                o['Vel_X'], o['Vel_Y'], o['Vel_Z'] = struct.unpack('!3f', data[:12])
                data = data[12:]
                output['Vel'] = o
            # status
            if mode & OutputMode.Status:
                (status,) = struct.unpack('!B', data[:1])
                data = data[1:]
                output['Stat'] = status
            # sample counter
            if settings & OutputSettings.Timestamp_SampleCnt:
                (TS,) = struct.unpack('!H', data[:2])
                data = data[2:]
                output['Sample'] = TS
            # UTC time
            if settings & OutputSettings.Timestamp_UTCTime:
                o = {}
                o['ns'], o['Year'], o['Month'], o['Day'], o['Hour'], o['Minute'], o['Second'], o['Flags'] = (
                    struct.unpack('!ihbbbbb', data[:12])
                )
                data = data[12:]
                output['Timestamp'] = o
            # TODO at that point data should be empty
        except struct.error as e:
            raise MTException("could not parse MTData message: %s" % e)
        if data != '':
            raise MTException("could not parse MTData message (too long).")
        return output

    def ChangeBaudrate(self, xbus_baud, xbus_flow, rtcm_baud):
        """Change the baudrate, reset the device and reopen communication."""
        # Get current configuration for reference
        try:
            current_xbus_baud, current_xbus_flow, current_rtcm_baud = self.GetPortConfig()
        except (MTException, MTTimeoutException):
            # If we can't read current config, use defaults
            current_xbus_baud = 115200
            current_xbus_flow = 0
            current_rtcm_baud = 38400

        # Use current values if new values not provided
        if xbus_baud is None:
            xbus_baud = current_xbus_baud
            xbus_flow = current_xbus_flow
        if rtcm_baud is None:
            rtcm_baud = current_rtcm_baud

        xbus_brid = Baudrates.get_BRID(xbus_baud)
        rtcm_brid = Baudrates.get_BRID(rtcm_baud)
        self.SetPortConfig(xbus_brid, xbus_flow, rtcm_brid)
        self.Reset()
        self.device.flush()
        self.device.baudrate = xbus_baud
        self.device.flush()
        time.sleep(0.01)
        self.read_msg()
        self.write_msg(MID.WakeUpAck)


################################################################
# Auto detect port
################################################################
def find_devices(verbose=False):
    mtdev_list = []
    for port in glob.glob("/dev/tty*S*"):
        if verbose:
            print("Trying '%s'" % port)
        try:
            br = find_baudrate(port, verbose)
            if br:
                mtdev_list.append((port, br))
        except MTException:
            pass
    return mtdev_list


################################################################
# Auto detect baudrate
################################################################
def find_baudrate(port, verbose=False):
    baudrates = [2000000, 115200, 916200, 460800, 230400, 57600, 38400, 19200, 9600, 4800]

    for br in baudrates:
        if verbose:
            print("Trying %d bd:" % br)
            sys.stdout.flush()
        try:
            mt = MTDevice(port, br, verbose=verbose)
        except serial.SerialException:
            if verbose:
                print("fail: unable to open device.")
            raise MTException("unable to open %s" % port)
        except Exception as e:
            if verbose:
                print(e)
            continue

        try:
            mt.GoToConfig()
            mt.GoToMeasurement()
            if verbose:
                print("ok.")
            return br
        except MTException:
            if verbose:
                print("fail.")


################################################################
# Argument parser setup
################################################################
def setup_argument_parser():
    """Set up and return the argument parser."""
    parser = argparse.ArgumentParser(
        description='MT device driver.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
OUTPUT
    The format is a sequence of "<group><type><frequency>?<format>?"
    separated by commas.
    The frequency and format are optional.
    The groups and types can be:
        t  temperature (max frequency: 1 Hz):
            tt  temperature
        i  timestamp (max frequency: 2000 Hz):
            iu  UTC time
            ip  packet counter
            ii  Integer Time of the Week (ITOW)
            if  sample time fine
            ic  sample time coarse
            ir  frame range
        o  orientation data (max frequency: 400 Hz):
            oq  quaternion
            om  rotation matrix
            oe  Euler angles
        b  pressure (max frequency: 50 Hz):
            bp  baro pressure
        a  acceleration (max frequency: 2000 Hz (see documentation)):
            ad  delta v
            aa  acceleration
            af  free acceleration
            ah  acceleration HR (max frequency 1000 Hz)
        p  position (max frequency: 400 Hz):
            pa  altitude ellipsoid
            pp  position ECEF
            pl  latitude longitude
        n  GNSS (max frequency: 4 Hz):
            np  GNSS PVT data
            ns  GNSS satellites info
        w  angular velocity (max frequency: 2000 Hz (see documentation)):
            wr  rate of turn
            wd  delta q
            wh  rate of turn HR (max frequency 1000 Hz)
        g  GPS (max frequency: 4 Hz):
            gd  DOP
            gs  SOL
            gu  time UTC
            gi  SV info
        r  Sensor Component Readout (max frequency: 2000 Hz):
            rr  ACC, GYR, MAG, temperature
            rt  Gyro temperatures
        m  Magnetic (max frequency: 100 Hz):
            mf  magnetic Field
        v  Velocity (max frequency: 400 Hz):
            vv  velocity XYZ
        s  Status (max frequency: 2000 Hz):
            sb  status byte
            sw  status word
    Frequency is specified in decimal and is assumed to be the maximum
    frequency if it is omitted.
    Format is a combination of the precision for real valued numbers and
    coordinate system:
        precision:
            f  single precision floating point number (32-bit) (default)
            d  double precision floating point number (64-bit)
        coordinate system:
            e  East-North-Up (default)
            n  North-East-Down
            w  North-West-Up
    Examples:
        The default configuration for the MTi-1/10/100 IMUs can be
        specified either as:
            "wd,ad,mf,ip,if,sw"
        or
            "wd2000fe,ad2000fe,mf100fe,ip2000,if2000,sw2000"
        For getting quaternion orientation in float with sample time:
            "oq400fw,if2000"
        For longitude, latitude, altitude and orientation (on MTi-G-700):
            "pl400fe,pa400fe,oq400fe"
CAN_OUTPUT
    The format is a sequence of "<group><type><frequency>?"
    separated by commas. The frequency is optional.
        t  Timestamp:
            ts  SampleTime
            tu  UTC Time
            tg  GroupCounter
        s  Status:
            ss  Status Word
            se  Error
            sw  Warning
        o  Orientation data (max frequency: 400 Hz):
            oq  Quaternion
            oe  Euler Angles
        i  Inertial data (max frequency: 400 Hz):
            iq  DeltaQ
            iv  DeltaV
            if  Free Acceleration
            ir  Rate of Turn
            ia  Acceleration
        m  Magentic Field data (max frequency: 100 Hz):
            mm  Magnetic Field
        f  Temperature data (max frequency: 400 Hz):
            ft  Temperature
        b  Pressure data (max frequency: 100 Hz):
            bp  Barometric Pressure
        h  High-Rate data
            ha  Accekeration HR (max frequency: 2000 Hz)
            hr  Rate of Turn HR (max frequency: 1600 Hz)
        p  Position and Velocity data (max frequency: 400 Hz):
            pl  Latitude and Longitude
            pv  Velocity
            pa  Altitude Ellipsoid
        g  GNSS data
            gs  GNSS receiver status
            gd  GNSS receiver DOP
    Frequency is specified in decimal and is assumed to be the maximum
    frequency if it is omitted.
    Example:
        The default configuration for Focus Mti680G can be specified as:
            "se,sw,ts,tg,tu,ss,oq400,iv400,ir,iq,ia,pl400,pv,pa,gd,gs"


Legacy options:
    -m, --output-mode=MODE
        Legacy mode of the device to select the information to output.
        This is required for 'legacy-configure' command.
        MODE can be either the mode value in hexadecimal, decimal or
        binary form, or a string composed of the following characters
        (in any order):
            t  temperature, [0x0001]
            c  calibrated data, [0x0002]
            o  orientation data, [0x0004]
            a  auxiliary data, [0x0008]
            p  position data (requires MTi-G), [0x0010]
            v  velocity data (requires MTi-G), [0x0020]
            s  status data, [0x0800]
            g  raw GPS mode (requires MTi-G), [0x1000]
            r  raw (incompatible with others except raw GPS), [0x4000]
        For example, use "--output-mode=so" to have status and
        orientation data.
    -s, --output-settings=SETTINGS
        Legacy settings of the device. This is required for 'legacy-configure'
        command.
        SETTINGS can be either the settings value in hexadecimal,
        decimal or binary form, or a string composed of the following
        characters (in any order):
            t  sample count (excludes 'n')
            n  no sample count (excludes 't')
            u  UTC time
            q  orientation in quaternion (excludes 'e' and 'm')
            e  orientation in Euler angles (excludes 'm' and 'q')
            m  orientation in matrix (excludes 'q' and 'e')
            A  acceleration in calibrated data
            G  rate of turn in calibrated data
            M  magnetic field in calibrated data
            i  only analog input 1 (excludes 'j')
            j  only analog input 2 (excludes 'i')
            N  North-East-Down instead of default: X North Z up
        For example, use "--output-settings=tqMAG" for all calibrated
        data, sample counter and orientation in quaternion.
    -p, --period=PERIOD
        Sampling period in (1/115200) seconds (default: 1152).
        Minimum is 225 (1.95 ms, 512 Hz), maximum is 1152
        (10.0 ms, 100 Hz).
        Note that for legacy devices it is the period at which sampling occurs,
        not the period at which messages are sent (see below).

Deprecated options:
    -f, --deprecated-skip-factor=SKIPFACTOR
        Only for mark III devices.
        Number of samples to skip before sending MTData message
        (default: 0).
        The frequency at which MTData message is send is:
            115200/(PERIOD * (SKIPFACTOR + 1))
        If the value is 0xffff, no data is send unless a ReqData request
        is made.
        ''',
    )

    # Commands
    commands = parser.add_argument_group('Commands')
    commands.add_argument('-r', '--reset', action='store_true', help='Reset device to factory defaults')
    commands.add_argument('-e', '--echo', action='store_true', help='Print MTData (default if no other command)')
    commands.add_argument('-i', '--inspect', action='store_true', help='Print current MT device configuration')
    commands.add_argument(
        '-l',
        '--legacy-configure',
        action='store_true',
        help='Configure device in legacy mode (needs MODE and SETTINGS)',
    )

    # Configuration commands
    config_commands = parser.add_argument_group('Configuration Commands')
    config_commands.add_argument(
        '-c', '--configure', metavar='OUTPUT', help='Configure the device output (see OUTPUT description)'
    )
    config_commands.add_argument(
        '-a',
        '--change-baudrate',
        metavar='BAUD_CONFIG',
        help='Change baudrate: format "x<freq><f>?,r<freq>" (e.g., "x115200f,r38400")',
    )
    config_commands.add_argument(
        '--cb',
        '--can-baudrate',
        metavar='NEW_CAN_BAUD',
        type=int,
        dest='can_baudrate',
        help='Change CAN baudrate (0 to disable)',
    )
    config_commands.add_argument(
        '--cc',
        '--can-configure',
        metavar='CAN_OUTPUT',
        dest='can_configure',
        help='Configure CAN output (see CAN_OUTPUT description)',
    )
    config_commands.add_argument('-x', '--xkf-scenario', metavar='ID', type=int, help='Change current XKF scenario')
    config_commands.add_argument(
        '--gnss-lever-arm', metavar='X,Y,Z', help='Set GNSS receiver lever arm (format: X,Y,Z as floats)'
    )
    config_commands.add_argument('--import-xsa', metavar='FILE', help='Import XSA file (must end with .xsa)')
    config_commands.add_argument('--export-xsa', metavar='FILE', help='Export XSA file (must end with .xsa)')

    # Device options
    device_options = parser.add_argument_group('Device Options')
    device_options.add_argument(
        '-d',
        '--device',
        metavar='DEV',
        default='/dev/ttyUSB0',
        help='Serial interface (default: /dev/ttyUSB0, use "auto" for auto-detection)',
    )
    device_options.add_argument(
        '-b',
        '--baudrate',
        metavar='BAUD',
        type=int,
        help='Baudrate of serial interface (leave out to auto-detect)',
    )
    device_options.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    device_options.add_argument(
        '-n',
        '--no-ack',
        action='store_true',
        help='No-acknowledgment mode: send commands without waiting for responses',
    )

    # Legacy options
    legacy_options = parser.add_argument_group('Legacy Options')
    legacy_options.add_argument(
        '-m', '--output-mode', metavar='MODE', help='Legacy output mode (required for legacy-configure)'
    )
    legacy_options.add_argument(
        '-s', '--output-settings', metavar='SETTINGS', help='Legacy output settings (required for legacy-configure)'
    )
    legacy_options.add_argument(
        '-p', '--period', metavar='PERIOD', type=int, help='Sampling period in (1/115200) seconds (default: 1152)'
    )
    legacy_options.add_argument(
        '-f',
        '--deprecated-skip-factor',
        metavar='SKIPFACTOR',
        type=int,
        help='Skip factor for mark III devices (default: 0)',
    )

    return parser


################################################################
# Main function
################################################################
def main():
    parser = setup_argument_parser()
    args = parser.parse_args()

    # Determine actions based on arguments
    actions = []

    # Add actions based on arguments
    if args.reset:
        actions.append('reset')
    if args.import_xsa:
        actions.append('import-xsa')
    if args.export_xsa:
        actions.append('export-xsa')
    if args.change_baudrate:
        baud_config = get_baudrate_config(args.change_baudrate)
        if baud_config is None:
            return 1
        actions.append('change-baudrate')
    if args.configure:
        output_config = get_output_config(args.configure)
        if output_config is None:
            return 1
        actions.append('configure')
    if args.can_baudrate is not None:
        new_can_baudrate = args.can_baudrate
        actions.append('can-baudrate')
    if args.can_configure:
        can_output_config = get_can_output_config(args.can_configure)
        if can_output_config is None:
            return 1
        actions.append('can-configure')
    if args.echo:
        actions.append('echo')
    if args.inspect:
        actions.append('inspect')
    if args.legacy_configure:
        actions.append('legacy-configure')
    if args.xkf_scenario is not None:
        new_xkf = args.xkf_scenario
        actions.append('xkf-scenario')
    if args.gnss_lever_arm:
        try:
            gnss_lever_arm = [float(x) for x in args.gnss_lever_arm.split(',')]
        except ValueError:
            print("gnss-lever-arm argument must be in the format X,Y,Z (float)")
            return 1
        if len(gnss_lever_arm) != 3:
            print("gnss-lever-arm argument must be in the format X,Y,Z (float)")
            return 1
        actions.append('gnss-lever-arm')

    # Parse legacy mode and settings if provided
    mode = None
    settings = None
    if args.output_mode:
        mode = get_mode(args.output_mode)
        if mode is None:
            return 1
    if args.output_settings:
        settings = get_settings(args.output_settings)
        if settings is None:
            return 1

    # Default action if none specified
    if len(actions) == 0:
        actions.append('echo')

    try:
        # Handle device auto-detection
        device = args.device
        baudrate = args.baudrate

        if device == 'auto':
            devs = find_devices(args.verbose)
            if devs:
                print("Detected devices:" + "".join('\n\t%s @ %d' % (d, p) for d, p in devs))
                print("Using %s @ %d" % devs[0])
                device, baudrate = devs[0]
            else:
                print("No suitable device found.")
                return 1

        # Find baudrate if not specified
        if not baudrate:
            baudrate = find_baudrate(device, args.verbose)
        if not baudrate:
            print("No suitable baudrate found.")
            return 1

        # Open device
        try:
            mt = MTDevice(device, baudrate, verbose=args.verbose, no_ack=args.no_ack)
        except serial.SerialException:
            raise MTException("unable to open %s" % device)

        # Execute actions
        if 'inspect' in actions:
            if not args.no_ack:
                inspect(mt, device, baudrate)
            else:
                print("No-ack mode is enabled. Device will not respond with serial data.")

        if 'change-baudrate' in actions:
            xbus_baud = None
            xbus_flow = None
            rtcm_baud = None

            if ('x' in baud_config) and (baud_config['x'][0] is not None):
                print("Changing xbus baudrate from %d to %d:" % (baudrate, baud_config['x'][0]))
                sys.stdout.flush()
                xbus_baud = baud_config['x'][0]
                xbus_flow = baud_config['x'][1]

            if ('r' in baud_config) and (baud_config['r'][0] is not None):
                print("Changing rtcm baudrate to %d:" % (baud_config['r'][0]))
                sys.stdout.flush()
                rtcm_baud = baud_config['r'][0]

            mt.ChangeBaudrate(xbus_baud, xbus_flow, rtcm_baud)
            print(" Ok")

        if 'reset' in actions:
            print("Restoring factory defaults")
            sys.stdout.flush()
            mt.RestoreFactoryDefaults()
            print(" Ok")

        if 'import-xsa' in actions:
            print("Importing XSA file")
            sys.stdout.flush()
            mt.ImportXSA(args.import_xsa)
            print(" Ok")

        if 'export-xsa' in actions:
            print("Exporting XSA file")
            sys.stdout.flush()
            mt.ExportXSA(args.export_xsa)
            print(" Ok")

        if 'configure' in actions:
            print("Changing output configuration")
            sys.stdout.flush()
            mt.SetOutputConfiguration(output_config)
            print(" Ok")

        if 'can-baudrate' in actions:
            print("Changing can baudrate to %d:" % new_can_baudrate)
            sys.stdout.flush()
            mt.SetCanConfig(new_can_baudrate)
            print(" Ok")

        if 'can-configure' in actions:
            print("Changing output can configuration")
            sys.stdout.flush()
            mt.SetCanOutputConfig(can_output_config)
            print(" Ok")

        if 'legacy-configure' in actions:
            if mode is None:
                print("output-mode is required to configure the device in legacy mode.")
                return 1
            if settings is None:
                print("output-settings is required to configure the device in legacy mode.")
                return 1
            print("Configuring in legacy mode")
            sys.stdout.flush()
            mt.configure_legacy(mode, settings, args.period, args.deprecated_skip_factor)
            print(" Ok")

        if 'xkf-scenario' in actions:
            print("Changing XKF scenario")
            sys.stdout.flush()
            mt.SetCurrentScenario(new_xkf)
            print("Ok")

        if 'gnss-lever-arm' in actions:
            print("Setting GNSS lever arm to %s" % gnss_lever_arm)
            sys.stdout.flush()
            mt.SetGnssLeverArm(gnss_lever_arm[0], gnss_lever_arm[1], gnss_lever_arm[2])
            print("Ok")

        if 'echo' in actions:
            try:
                while True:
                    if args.no_ack:
                        print("No-ack mode is enabled. Device will not respond with serial data.")  
                        break
                    print(mt.read_measurement(mode, settings))
            except KeyboardInterrupt:
                pass

        mt._ensure_measurement_state()

    except MTErrorMessage as e:
        print("MTErrorMessage:", e)
    except MTException as e:
        print("MTException:", e)


def inspect(mt, device, baudrate):
    """Inspection."""

    def config_fmt(config):
        """Enhanced output configuration with XDI decoding, showing all possible outputs for MTi-680G."""
        # Base XDI types supported by MTi-680G with their max frequencies
        base_xdi_types = {
            # Temperature (MTi 600-s: 100 Hz)
            0x0810: ('Temperature', 100, False),
            # Timestamp group (always with data, frequency ignored)
            0x1010: ('UTC Time', 'Always', False),
            0x1020: ('Packet Counter', 'Always', False),
            0x1060: ('Sample Time Fine', 'Always', False),
            0x1070: ('Sample Time Coarse', 'Always', False),
            # Orientation group (400 Hz max, supports format flags)
            0x2010: ('Quaternion', 400, True),
            0x2020: ('Rotation Matrix', 400, True),
            0x2030: ('Euler Angles', 400, True),
            # Pressure (50 Hz max for MTi-680G)
            0x3010: ('Baro Pressure', 50, False),
            # Acceleration (400 Hz max, supports format flags)
            0x4010: ('Delta V', 400, True),
            0x4020: ('Acceleration', 400, True),
            0x4030: ('Free Acceleration', 400, True),
            0x4040: ('AccelerationHR', 'Variable', True),  # Device dependent
            # Position (400 Hz max, supports format flags)
            0x5020: ('Altitude Ellipsoid', 400, True),
            0x5030: ('Position ECEF', 400, True),
            0x5040: ('LatLon', 400, True),
            # GNSS (4 Hz max)
            0x7010: ('GNSS PVT Data', 4, False),
            0x7020: ('GNSS Satellites Info', 4, False),
            0x7030: ('GNSS PVT Pulse', 4, False),
            # Angular Velocity (400 Hz max, supports format flags)
            0x8020: ('Rate of Turn', 400, True),
            0x8030: ('Delta Q', 400, True),
            0x8040: ('RateOfTurnHR', 'Variable', True),  # Device dependent
            # Magnetic (100 Hz max, supports format flags)
            0xC020: ('Magnetic Field', 100, True),
            # Velocity (400 Hz max, supports format flags)
            0xD010: ('Velocity XYZ', 400, True),
            # Status (always with data, frequency ignored)
            0xE010: ('Status Byte', 'Always', False),
            0xE020: ('Status Word', 'Always', False),
        }

        # Format mappings for XDI identifiers
        precision_formats = {0x0: 'Float32', 0x1: 'Fp1220', 0x2: 'Fp1632', 0x3: 'Float64'}

        coordinate_systems = {0x0: 'ENU', 0x4: 'NED', 0x8: 'NWU'}

        # Parse configured messages into a dictionary
        configured_messages = {}
        if config:
            for mode, freq in config:
                configured_messages[mode] = freq

        # Start with base XDI types
        all_display_xdis = {}
        for base_xdi, (name, max_freq, supports_format) in base_xdi_types.items():
            all_display_xdis[base_xdi] = (name, max_freq)

        # Add any configured XDIs with their specific format variants
        for configured_xdi in configured_messages.keys():
            base_xdi = configured_xdi & 0xFFF0
            if base_xdi in base_xdi_types:
                name, max_freq, supports_format = base_xdi_types[base_xdi]
                format_flags = configured_xdi & 0x000F

                if supports_format and format_flags:
                    format_parts = []
                    precision = format_flags & 0x3
                    coord = format_flags & 0xC

                    if precision in precision_formats:
                        format_parts.append(precision_formats[precision])
                    if coord in coordinate_systems:
                        format_parts.append(coordinate_systems[coord])

                    if format_parts:
                        formatted_name = f"{name} ({', '.join(format_parts)})"
                    else:
                        formatted_name = name
                else:
                    formatted_name = name

                # Replace the base entry with the specifically configured variant
                all_display_xdis[configured_xdi] = (formatted_name, max_freq)
                # Remove the base entry if it's different from the configured one
                if configured_xdi != base_xdi and base_xdi in all_display_xdis:
                    del all_display_xdis[base_xdi]
            else:
                # Unknown XDI
                all_display_xdis[configured_xdi] = (f'Unknown XDI (0x{configured_xdi:04X})', 'Unknown')

        # Create table rows
        configs = []
        for xdi in sorted(all_display_xdis.keys()):
            name, max_freq = all_display_xdis[xdi]

            if xdi in configured_messages:
                frequency = configured_messages[xdi]
                if frequency == 0xFFFF or frequency == 0:
                    freq_str = 'Max freq'
                elif max_freq == 'Always':
                    freq_str = 'Always'
                else:
                    freq_str = f'{frequency} Hz'
            else:
                freq_str = 'Disabled'

            configs.append(f'    {name:<35} | {freq_str:>10}')

        # Add header for the table
        header = f'    {"Message Type":<35} | {"Frequency":>10}'
        separator = f'    {"-" * 35}-+-{"-" * 10}'
        return '\n' + header + '\n' + separator + '\n' + '\n'.join(configs)

    def hex_fmt(size=4):
        """Factory for hexadecimal representation formatter."""
        fmt = '0x%%0%dX' % (2 * size)

        def f(value):
            """Hexadecimal representation."""
            # length of string is twice the size of the value (in bytes)
            return fmt % value

        return f

    def sync_fmt(settings):
        """Format sync settings with MTi-600s specific knowledge and aligned values."""
        if not settings:
            return "No sync settings"

        # Function mapping based on MTi documentation
        function_map = {
            3: "TriggerIndication",
            4: "Interval Transition Measurement",
            8: "SendLatest",
            9: "Clock Bias Estimation",
            11: "StartSampling",
            14: "GNSS 1 PPS",
        }

        # MTi-600s sync line mapping (MTi-680G has internal line 10 for GNSS 1 PPS)
        line_map = {
            1: "SyncIn",
            2: "SyncOut",
            4: "ClockIn",
            5: "GPSClockIn (Obsolete)",
            8: "ReqData",
            9: "SyncOut2",
            10: "In3 (Input line 3)",  # MTi-680G internal sync line
        }

        # Polarity mapping
        polarity_map = {
            0: "Disabled",
            1: "Rising Edge / Positive Pulse",
            2: "Falling Edge / Negative Pulse",
            3: "Both Edges / Toggle",
        }

        lines = []
        setting_num = 1

        # GetSyncSettings returns a list of tuples: (function, line, polarity, trigger, skip_first, skip_factor, pulse_width, delay)
        for setting in settings:
            if len(setting) != 8:
                continue

            function, line, polarity, trigger, skip_first, skip_factor, pulse_width, delay = setting

            # Skip disabled settings
            if polarity == 0:
                continue

            lines.append(f"\n    Setting {setting_num}:")

            # Function (using 32-char field to align values at column 38)
            func_name = function_map.get(function, f"Unknown ({function})")
            lines.append(f"\n      Function:                       {func_name}")

            # Line
            line_name = line_map.get(line, f"Unknown line ({line})")
            lines.append(f"\n      Line:                           {line_name}")

            # Polarity
            pol_name = polarity_map.get(polarity, f"Unknown ({polarity})")
            lines.append(f"\n      Polarity:                       {pol_name}")

            # Trigger type
            if trigger == 0:
                lines.append(f"\n      Trigger:                        Continuous")
            else:
                lines.append(f"\n      Trigger:                        Single shot")

            # Skip settings
            lines.append(f"\n      Skip First:                     {skip_first}")
            lines.append(f"\n      Skip Factor:                    {skip_factor}")

            # Timing (convert from 100μs units and handle sync in/out differences)
            if function in [3, 4, 9]:  # Sync in functions - delay
                delay_ms = delay * 100 / 1000  # Convert 100μs to ms
                lines.append(f"\n      Delay:                          {delay_ms:.1f} ms")
            elif function in [8, 11, 14]:  # Sync out functions - pulse width and offset
                pulse_width_ms = pulse_width * 100 / 1000
                delay_ms = delay * 100 / 1000
                lines.append(f"\n      Pulse Width:                    {pulse_width_ms:.1f} ms")
                lines.append(f"\n      Offset:                         {delay_ms:.1f} ms")

            setting_num += 1

        if not lines:
            return "All settings disabled"

        return ''.join(lines)

    def bytearray_fmt(data):
        """Format bytearray data in a readable way."""
        if isinstance(data, (bytes, bytearray)):
            # Try to decode as ASCII string first
            try:
                decoded = data.decode('ascii').strip('\x00 ')
                if decoded and all(c.isprintable() for c in decoded):
                    return '"{}"'.format(decoded)
            except UnicodeDecodeError:
                pass

            # If not a readable string, show as hex
            if len(data) <= 16:
                return '[' + ' '.join('{:02X}'.format(b) for b in data) + ']'
            else:
                # For longer data, show first few bytes + length
                preview = ' '.join('{:02X}'.format(b) for b in data[:8])
                return '[{} ... ] ({} bytes)'.format(preview, len(data))
        return str(data)

    def port_config_fmt(config):
        """Format port configuration with aligned values."""
        if isinstance(config, tuple) and len(config) == 3:
            xbus_baud, xbus_flow, rtcm_baud = config
            lines = []

            # XBus configuration (aligned values at column 38)
            flow_str = "with flow control" if xbus_flow else "no flow control"
            lines.append(f"XBus: {xbus_baud:,} baud ({flow_str}), RTCM: {rtcm_baud:,} baud")

            return lines[0]
        elif isinstance(config, dict):
            lines = []

            # XBus configuration (aligned values at column 38)
            xbus_baud = config.get('XBus Baudrate')
            xbus_flow = config.get('XBus Flow Control')

            if xbus_baud is not None:
                flow_str = "with flow control" if xbus_flow else "no flow control"
                lines.append(f"    XBus:                           {xbus_baud:,} baud ({flow_str})")

            # RTCM configuration
            rtcm_baud = config.get('RTCM Baudrate')
            if rtcm_baud is not None:
                lines.append(f"    RTCM:                           {rtcm_baud:,} baud")

            if not lines:
                return "No port configuration"

            return '\n' + '\n'.join(lines)
        else:
            return str(config)

    def baudrate_fmt(brid):
        """Format baudrate ID to actual baudrate."""
        try:
            actual_baud = Baudrates.get_BR(brid)
            return '{} baud'.format(actual_baud)
        except MTException:
            return 'Unknown baudrate ID: {}'.format(brid)

    def scenario_fmt(scenarios):
        """Format available scenarios."""
        if not scenarios:
            return 'None'
        formatted = []
        for scenario_type, version, label in scenarios:
            label_str = label.decode('ascii').strip('\x00 ') if isinstance(label, bytes) else str(label)
            formatted.append('      {}: {} (v{})'.format(scenario_type, label_str, version))
        return '\n' + '\n'.join(formatted)

    def current_scenario_fmt(scenario_string):
        """Format current scenario filter profile(s)."""
        if not isinstance(scenario_string, str):
            return str(scenario_string)

        # The scenario string may contain multiple profiles separated by '/'
        if '/' in scenario_string:
            profiles = scenario_string.split('/')
            return f'"{scenario_string}" (Combined: {", ".join(profiles)})'
        else:
            return f'"{scenario_string}"'

    def firmware_fmt(fw):
        """Format firmware revision."""
        major, minor, revision = fw
        return '{}.{}.{}'.format(major, minor, revision)

    def hardware_version_fmt(hw):
        """Format hardware version."""
        major, minor = hw
        return '{}.{}'.format(major, minor)

    def gps_coords_fmt(coords):
        """Format GPS coordinates."""
        lat, lon, alt = coords
        return '{:.6f}°, {:.6f}°, {:.1f}m'.format(lat, lon, alt)

    def lever_arm_fmt(coords):
        """Format lever arm coordinates."""
        x, y, z = coords
        return '({:.3f}, {:.3f}, {:.3f}) meters'.format(x, y, z)

    def utc_time_fmt(time_data):
        """Format UTC time."""
        ns, year, month, day, hour, minute, second, flag = time_data
        return '{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}.{:03d} (flags: 0x{:02X})'.format(
            year, month, day, hour, minute, second, ns // 1000000, flag
        )

    def can_output_config_fmt(data):
        """CAN output configuration with complete XDI mapping and tabular format."""
        # Complete XCDI message type mapping based on MTi documentation
        all_message_types = {
            # Timestamp messages
            0x05: 'Timestamp Sample',
            0x06: 'Group Counter',
            0x07: 'UTC Time',
            # Error/Status messages
            0x01: 'Error',
            0x02: 'Warning',
            0x11: 'Status Word',
            # Orientation messages
            0x21: 'Quaternion',
            0x22: 'Euler Angles',
            # Motion messages
            0x31: 'Delta V',
            0x32: 'Rate of Turn',
            0x33: 'Delta Q',
            0x34: 'Acceleration',
            0x35: 'Free Acceleration',
            # Magnetic/Temperature
            0x41: 'Magnetic Field',
            0x51: 'Temperature',
            0x52: 'Barometric Pressure',
            # High Rate sensors
            0x61: 'Rate of Turn HR',
            0x62: 'Acceleration HR',
            # Position/GNSS
            0x71: 'Latitude/Longitude',
            0x72: 'Altitude Ellipsoid',
            0x73: 'Position ECEF X',
            0x74: 'Position ECEF Y',
            0x75: 'Position ECEF Z',
            0x76: 'Velocity XYZ',
            0x79: 'GNSS Status',
            0x7A: 'GNSS DOP',
        }

        # Message types that are enable/disable only (no configurable frequency)
        no_frequency_types = {
            0x01,
            0x02,
            0x05,
            0x06,
            0x07,
            0x11,
        }  # Error, Warning, SampleTime, GroupCounter, UTC Time, Status Word

        # Parse configured messages
        configured_messages = {}
        if isinstance(data, (bytes, bytearray)) and len(data) >= 8:
            try:
                for i in range(0, len(data), 8):
                    if i + 8 <= len(data):
                        config_bytes = data[i : i + 8]
                        msg_type, frequency = struct.unpack('!B5xH', config_bytes)
                        configured_messages[msg_type] = frequency
            except (struct.error, IndexError):
                # If parsing fails, fall back to hex display
                return bytearray_fmt(data)

        # Create table rows for all possible message types
        configs = []
        for msg_type in sorted(all_message_types.keys()):
            type_name = all_message_types[msg_type]

            if msg_type in configured_messages:
                frequency = configured_messages[msg_type]

                # Handle message types that are enable/disable only
                if msg_type in no_frequency_types:
                    if frequency == 1:
                        freq_str = 'Enabled'
                    else:
                        freq_str = 'Disabled'
                else:
                    # Format frequency for configurable message types
                    if frequency == 0x07FF:
                        freq_str = 'Max freq'
                    elif frequency == 0xFFFF:
                        freq_str = 'On change'
                    elif frequency == 0:
                        freq_str = 'Disabled'
                    else:
                        freq_str = f'{frequency} Hz'
            else:
                freq_str = 'Disabled'

            # Create tabular row with proper spacing
            configs.append(f'    {type_name:<25} | {freq_str:>10} | 0x{msg_type:02X}')

        # Add header for the table
        header = f'    {"Message Type":<25} | {"Frequency":>10} | {"ID"}'
        separator = f'    {"-" * 25}-+-{"-" * 10}-+-----'
        return '\n' + header + '\n' + separator + '\n' + '\n'.join(configs)

    def can_config_fmt(config):
        """Format CAN configuration with aligned values."""
        if not isinstance(config, int):
            return str(config)

        lines = []

        # Extract bit fields from 32-bit configuration word
        baudrate_code = config & 0xFF  # Bits 7:0
        can_enable = bool(config & 0x100)  # Bit 8
        termination_120 = bool(config & 0x800)  # Bit 11

        # Get CAN baudrate using proper lookup from mtdef.py
        try:
            baudrate_value = Baudrates.get_can_BR(baudrate_code)
            baudrate_str = f"{baudrate_value:,}"
        except MTException:
            baudrate_str = f"Unknown ({baudrate_code})"

        # Format with aligned values (using 32-char field to align values at column 38)
        if can_enable:
            lines.append(f"Enabled, {baudrate_str} baud")
            if termination_120:
                lines.append(f"    120Ω termination:               Enabled")
        else:
            lines.append(f"Disabled")

        return '\n    '.join([''] + lines) if len(lines) > 1 else lines[0] if lines else "No CAN config"

    def gnss_receiver_settings_fmt(data):
        """Format GNSS receiver settings with aligned values."""
        if not isinstance(data, (bytes, bytearray)):
            return str(data)

        if len(data) < 10:
            return f"Invalid data length ({len(data)} bytes, expected 10): {bytearray_fmt(data)}"

        try:
            # Parse the 10-byte GNSS receiver settings structure (original format)
            # uint16: GNSS receiver type, uint16: Baud rate, uint16: Input rate, uint32: Options
            receiver_type, baudrate_code, input_rate, options = struct.unpack('!HHHI', data[:10])

            lines = []

            # Receiver type mapping
            receiver_types = {
                0x0000: "u-blox MAX-M8Q",
                0x0001: "Generic NMEA",
                0x0002: "u-blox NEO-M8P",
                0x0003: "u-blox ZED-F9P",
                0x0004: "Septentrio Mosaic X5",
                0x0005: "Trimble BX992",
            }

            receiver_name = receiver_types.get(receiver_type, f"Unknown (0x{receiver_type:04X})")
            lines.append(f"    Receiver Type:                  {receiver_name}")

            # Baudrate
            try:
                baudrate_value = Baudrates.get_BR(baudrate_code)
                baudrate_str = f"{baudrate_value:,} baud"
            except MTException:
                baudrate_str = f"Unknown baudrate (code: 0x{baudrate_code:04X})"
            lines.append(f"    Baudrate:                       {baudrate_str}")

            # Input rate
            lines.append(f"    Input Rate:                     {input_rate} Hz")

            # Receiver options (depends on receiver type)
            if receiver_type == 0x0001:  # Generic NMEA
                nmea_talker_ids = {
                    0x0000: "GL",
                    0x0001: "GN",
                    0x0002: "GP",
                }
                talker_id = nmea_talker_ids.get(options, f"Unknown (0x{options:08X})")
                lines.append(f"    NMEA Talker ID:                 {talker_id}")

            elif receiver_type in [0x0000, 0x0002, 0x0003]:  # u-blox receivers
                platform_models = {
                    0x0000: "Portable (default)",
                    0x0002: "Stationary",
                    0x0003: "Pedestrian",
                    0x0004: "Automotive",
                    0x0005: "At sea",
                    0x0006: "Airborne (<1 g)",
                    0x0007: "Airborne (<2 g)",
                    0x0008: "Airborne (<4 g)",
                    0x0009: "Wrist worn watch",
                    0x000A: "Bike",
                }
                platform = platform_models.get(options, f"Unknown (0x{options:08X})")
                lines.append(f"    Dynamic Platform Model:         {platform}")

            elif receiver_type in [0x0004, 0x0005]:  # Septentrio/Trimble
                lines.append(f"    Options:                        Ignored (0x{options:08X})")
            else:
                lines.append(f"    Options:                        0x{options:08X}")

            return '\n' + '\n'.join(lines)

        except struct.error:
            return f"Parse error: {bytearray_fmt(data)}"

    def option_flags_fmt(flags_data):
        """Format option flags with descriptions."""
        if not isinstance(flags_data, tuple) or len(flags_data) != 2:
            return str(flags_data)

        current_flags, clear_flags = flags_data

        # Option flags definitions from MTi documentation
        option_flags = {
            0x00000001: ("DisableAutoStore", "MTi 1-series", "Disable automatic writing to flash memory"),
            0x00000002: ("DisableAutoMeasurement", "MTi 1/600-series", "Stay in Config Mode upon startup"),
            0x00000004: ("EnableBeidou", "MTi-7/670/G-710", "Enable Beidou, disable GLONASS"),
            0x00000008: ("Reserved", "", "Reserved flag"),
            0x00000010: ("EnableAhs", "MTi 1/10/100-series", "Enable Active Heading Stabilization"),
            0x00000020: ("EnableOrientationSmoother", "MTi-670(G)/680(G)/G-710", "Enable Orientation Smoother"),
            0x00000040: ("EnableConfigurableBusId", "MTi 10/100/600-series", "Use configured BusId for Xbus"),
            0x00000080: (
                "EnableInRunCompassCalibration",
                "MTi 1/10/100/600-series",
                "Enable In-run Compass Calibration",
            ),
            0x00000200: (
                "EnableConfigMessageAtStartup",
                "MTi 1/600-series",
                "Send eMTS and Config messages at startup",
            ),
            0x00000800: ("EnablePositionVelocitySmoother", "MTi-680(G)", "Enable Position/Velocity Smoother"),
            0x00001000: ("EnableContinuousZRU", "MTi-680(G)", "Enable continuous Zero Rotation Updates"),
        }

        # List active flags with aligned formatting
        active_flags = []
        for flag_value, (name, products, description) in option_flags.items():
            if current_flags & flag_value:
                active_flags.append((name, products, description))

        if active_flags:
            lines = []
            for name, products, description in active_flags:
                product_str = f" ({products})" if products else ""
                lines.append(f"    {name}: {description}{product_str}")
            return '\n' + '\n'.join(lines)
        else:
            return "None"

    def try_message(m, f, formater=None, *args, **kwargs):
        print('  {:<35}'.format(m), end=' ')
        try:
            result = f(*args, **kwargs)
            if formater is not None:
                print(formater(result))
            else:
                if isinstance(result, (bytes, bytearray)):
                    print(bytearray_fmt(result))
                elif isinstance(result, tuple) and len(result) <= 4:
                    # Format simple tuples nicely
                    if len(result) == 2:
                        print('({}, {})'.format(*result))
                    elif len(result) == 3:
                        print('({}, {}, {})'.format(*result))
                    elif len(result) == 4:
                        print('({}, {}, {}, {})'.format(*result))
                    else:
                        print(result)
                else:
                    pprint.pprint(result, indent=4)
        except MTErrorMessage as e:
            if e.code == 0x04:
                print('unsupported by device')
            else:
                raise e
        except MTException as e:
            if "no-ack mode" in str(e).lower() or mt.no_ack:
                print('N/A (no-ack mode)')
            else:
                raise e

    print("\n{}".format('=' * 60))
    print("  Xsens MTi Device Information")
    print("{}".format('=' * 60))
    print("Device: {} at {:,} baud\n".format(device, baudrate))

    print("Device Information:")
    try_message("Device ID:", mt.GetDeviceID, hex_fmt(8))
    try_message("Product Code:", mt.GetProductCode)
    try_message("Firmware Revision:", mt.GetFirmwareRev, firmware_fmt)
    try_message("Hardware Version:", mt.GetHardwareVersion, hardware_version_fmt)

    print("\nCommunication Settings:")
    try_message("Current Baudrate:", mt.GetBaudrate, baudrate_fmt)
    try_message("Port Configuration:", mt.GetPortConfig, port_config_fmt)
    try_message("Option Flags:", mt.GetOptionFlags, option_flags_fmt)
    try_message("Location ID:", mt.GetLocationID)
    try_message("Transmit Delay:", mt.GetTransmitDelay)

    print("\nCAN Bus Configuration:")
    try_message("CAN Config:", mt.GetCanConfig, can_config_fmt)
    try_message("CAN Output Config:", mt.GetCanOutputConfig, can_output_config_fmt)

    print("\nGNSS Configuration:")
    try_message("GNSS Receiver Settings:", mt.GetGnssReceiverSettings, gnss_receiver_settings_fmt)
    try_message("GNSS Lever Arm:", mt.GetGnssLeverArm, lever_arm_fmt)
    try_message("GPS Coordinates:", mt.GetLatLonAlt, gps_coords_fmt)

    print("\nOutput Configuration:")
    try_message("Extended Output Mode:", mt.GetExtOutputMode, hex_fmt(2))
    try_message("Output Configuration:", mt.GetOutputConfiguration, config_fmt)
    try_message("String Output Type:", mt.GetStringOutputType)

    print("\nSynchronization & Timing:")
    try_message("Sync Settings:", mt.GetSyncSettings, sync_fmt)
    try_message("UTC Time:", mt.GetUTCTime, utc_time_fmt)

    print("\nOrientation & Scenarios:")
    try_message("Alignment (Sensor):", mt.GetAlignmentRotation, parameter=0)
    try_message("Alignment (Local):", mt.GetAlignmentRotation, parameter=1)
    try_message("Available Scenarios:", mt.GetAvailableScenarios, scenario_fmt)
    try_message("Current Scenario:", mt.GetCurrentScenario, current_scenario_fmt)

    print("\n{}".format('=' * 60))


def get_can_output_config(config_arg):
    freqs_list = [
        [400, 200, 100, 80, 50, 40, 25, 20, 16, 10, 8, 5, 4, 2, 1],
        [100, 50, 25, 20, 10, 5, 4, 2, 1],
        [2000, 1000, 500, 400, 250, 200, 125, 100, 80, 50, 40, 25, 20, 16, 10, 8, 5, 4, 2, 1],
        [1600, 800, 400, 320, 200, 160, 100, 80, 64, 50, 40, 32, 25, 20, 16, 10, 8, 5, 4, 2, 1],
        [65535],
    ]
    code_dict = {
        'ts': (0x05, [1]),
        'tu': (0x07, [1]),
        'tg': (0x06, [1]),
        'ss': (0x11, [1]),
        'se': (0x01, [1]),
        'sw': (0x02, [1]),
        'oq': (0x21, freqs_list[0]),
        'oe': (0x22, freqs_list[0]),
        'iq': (0x33, freqs_list[0]),
        'iv': (0x31, freqs_list[0]),
        'if': (0x35, freqs_list[0]),
        'ir': (0x32, freqs_list[0]),
        'ia': (0x34, freqs_list[0]),
        'mm': (0x41, freqs_list[1]),
        'ft': (0x51, freqs_list[0]),
        'bp': (0x52, freqs_list[1]),
        'ha': (0x62, freqs_list[2]),
        'hr': (0x61, freqs_list[3]),
        'pl': (0x71, freqs_list[0]),
        'pv': (0x76, freqs_list[0]),
        'pa': (0x72, freqs_list[0]),
        'gs': (0x79, freqs_list[4]),
        'gd': (0x7A, freqs_list[4]),
    }

    config_re = re.compile(r'([a-z]{2})(\d+)?')
    output_configuration = []
    try:
        for item in config_arg.split(','):
            group, frequency = config_re.findall(item.lower())[0]
            code, freqs = code_dict[group]
            if frequency:
                frequency = min(freqs, key=lambda x: abs(x - int(frequency)))
            else:
                frequency = freqs[0]
            output_configuration.append((code, 0, 0, frequency))
        return output_configuration
    except (IndexError, KeyError):
        print('could not parse output specification "%s"' % item)
        return


def get_output_config(config_arg):
    """Parse the mark IV output configuration argument."""
    # code and max frequency
    code_dict = {
        'tt': (0x0810, 1),
        'iu': (0x1010, 2000),
        'ip': (0x1020, 2000),
        'ii': (0x1030, 2000),
        'if': (0x1060, 2000),
        'ic': (0x1070, 2000),
        'ir': (0x1080, 2000),
        'oq': (0x2010, 400),
        'om': (0x2020, 400),
        'oe': (0x2030, 400),
        'bp': (0x3010, 50),
        'ad': (0x4010, 2000),
        'aa': (0x4020, 2000),
        'af': (0x4030, 2000),
        'ah': (0x4040, 1000),
        'pa': (0x5020, 400),
        'pp': (0x5030, 400),
        'pl': (0x5040, 400),
        'np': (0x7010, 4),
        'ns': (0x7020, 4),
        'wr': (0x8020, 2000),
        'wd': (0x8030, 2000),
        'wh': (0x8040, 1000),
        'gd': (0x8830, 4),
        'gs': (0x8840, 4),
        'gu': (0x8880, 4),
        'gi': (0x88A0, 4),
        'rr': (0xA010, 2000),
        'rt': (0xA020, 2000),
        'mf': (0xC020, 100),
        'vv': (0xD010, 400),
        'sb': (0xE010, 2000),
        'sw': (0xE020, 2000),
    }
    # format flags
    format_dict = {'f': 0x00, 'd': 0x03, 'e': 0x00, 'n': 0x04, 'w': 0x08}
    config_re = re.compile(r'([a-z]{2})(\d+)?([fdenw])?([fdnew])?')
    output_configuration = []
    try:
        for item in config_arg.split(','):
            group, frequency, fmt1, fmt2 = config_re.findall(item.lower())[0]
            code, max_freq = code_dict[group]
            if fmt1 in format_dict:
                code |= format_dict[fmt1]
            if fmt2 in format_dict:
                code |= format_dict[fmt2]
            if frequency:
                frequency = min(max_freq, int(frequency))
            else:
                frequency = max_freq
            output_configuration.append((code, frequency))
        return output_configuration
    except (IndexError, KeyError):
        print('could not parse output specification "%s"' % item)
        return


def get_baudrate_config(baud_config):
    config_re = re.compile(r'([xr])(\d+)([f]?)')
    baud_configuration = {}
    try:
        for item in baud_config.split(','):
            interface, frequency, flow_control = config_re.findall(item.lower())[0]
            baud_configuration[interface] = [int(frequency), not (bool(flow_control))]
        return baud_configuration
    except (IndexError, KeyError):
        print('could not parse baudrate specification "%s"' % item)
        return


def get_mode(arg):
    """Parse command line output-mode argument."""
    try:  # decimal
        mode = int(arg)
        return mode
    except ValueError:
        pass
    if arg[0] == '0':
        try:  # binary
            mode = int(arg, 2)
            return mode
        except ValueError:
            pass
        try:  # hexadecimal
            mode = int(arg, 16)
            return mode
        except ValueError:
            pass
    # string mode specification
    mode = 0
    for c in arg:
        if c == 't':
            mode |= OutputMode.Temp
        elif c == 'c':
            mode |= OutputMode.Calib
        elif c == 'o':
            mode |= OutputMode.Orient
        elif c == 'a':
            mode |= OutputMode.Auxiliary
        elif c == 'p':
            mode |= OutputMode.Position
        elif c == 'v':
            mode |= OutputMode.Velocity
        elif c == 's':
            mode |= OutputMode.Status
        elif c == 'g':
            mode |= OutputMode.RAWGPS
        elif c == 'r':
            mode |= OutputMode.RAW
        else:
            print("Unknown output-mode specifier: '%s'" % c)
            return
    return mode


def get_settings(arg):
    """Parse command line output-settings argument."""
    try:  # decimal
        settings = int(arg)
        return settings
    except ValueError:
        pass
    if arg[0] == '0':
        try:  # binary
            settings = int(arg, 2)
            return settings
        except ValueError:
            pass
        try:  # hexadecimal
            settings = int(arg, 16)
            return settings
        except ValueError:
            pass
    # strings settings specification
    timestamp = 0
    orient_mode = 0
    calib_mode = OutputSettings.CalibMode_Mask
    NED = 0
    for c in arg:
        if c == 't':
            timestamp = OutputSettings.Timestamp_SampleCnt
        elif c == 'n':
            timestamp = OutputSettings.Timestamp_None
        elif c == 'u':
            timestamp |= OutputSettings.Timestamp_UTCTime
        elif c == 'q':
            orient_mode = OutputSettings.OrientMode_Quaternion
        elif c == 'e':
            orient_mode = OutputSettings.OrientMode_Euler
        elif c == 'm':
            orient_mode = OutputSettings.OrientMode_Matrix
        elif c == 'A':
            calib_mode &= OutputSettings.CalibMode_Acc
        elif c == 'G':
            calib_mode &= OutputSettings.CalibMode_Gyr
        elif c == 'M':
            calib_mode &= OutputSettings.CalibMode_Mag
        elif c == 'i':
            calib_mode &= OutputSettings.AuxiliaryMode_NoAIN2
        elif c == 'j':
            calib_mode &= OutputSettings.AuxiliaryMode_NoAIN1
        elif c == 'N':
            NED = OutputSettings.Coordinates_NED
        else:
            print("Unknown output-settings specifier: '%s'" % c)
            return
    settings = timestamp | orient_mode | calib_mode | NED
    return settings


if __name__ == '__main__':
    main()
