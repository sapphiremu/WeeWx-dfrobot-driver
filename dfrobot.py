#!/usr/bin/env python

# Driver adapted by Saffron Murcia (2020) for
# DF Robot weather station kit with Anemometer/Wind Vane/Rain Bucket
# All credit for original driver goes to Matthew Wall and other mentions below.
# Matthew Wall GitHub: https://github.com/matthewwall

# Development Level: Very, VERY ALPHA

# Changes
# 0.02 - adapted parser to work with new data format
# 0.02.1 - adapted packet checking to new format
# 0.02.2 - changed multipliers to work with data from station
# 0.02.3 - Corrections to multipliers
# 0.03 - intruduced checksum error detection
#           weather station tends to corrupt data in high rain conditions (December 2021)
# 0.03.1 - driver tends to drop a clanger on syslog. Incorporated try/except in to logging. To monitor.
# 0.03.2 - Updated some comments prior to uploading to github.

# Future updates:
# - Remove references to TCP comm stack as not used by DFRobot
# - Rework parsing with common formulae
# - 

"""
#
Copyright 2014 Matthew Wall
See the file LICENSE.txt for your rights.

Driver for ADS WS1 weather stations.

Thanks to Kevin and Paul Caccamo for adding the serial-to-tcp capability.

Thanks to Steve (sesykes71) for the testing that made this driver possible.

Thanks to Jay Nugent (WB8TKL) and KRK6 for weather-2.kr6k-V2.1
  http://server1.nuge.com/~weather/
"""

from __future__ import with_statement
import syslog
import time

from weewx.units import INHG_PER_MBAR, MILE_PER_KM
import weewx.drivers
import weewx.wxformulas

DRIVER_NAME = 'dfrobot'
DRIVER_VERSION = '0.03.2'


def loader(config_dict, _):
    return dfrobotDriver(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return dfrobotConfEditor()


DEFAULT_SER_PORT = '/dev/ttyS0'
DEFAULT_TCP_ADDR = '192.168.36.25'
DEFAULT_TCP_PORT = 3000
PACKET_SIZE = 36
DEBUG_READ = 0


def logmsg(level, msg):
    try:
        syslog.syslog(level, 'dfrobot: ' + str(msg))
    except:
        syslog.syslog(syslog.LOG_ERR, 'Error in syslog handler for DFRobot.')

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)

class dfrobotDriver(weewx.drivers.AbstractDevice):
    """weewx driver that communicates with an ADS-dfrobot station

    mode - Communication mode - TCP, UDP, or Serial.
    [Required. Default is serial]

    port - Serial port or network address.
    [Required. Default is /dev/ttyS0 for serial,
     and 192.168.36.25:3000 for TCP/IP]

    max_tries - how often to retry serial communication before giving up.
    [Optional. Default is 5]

    wait_before_retry - how long to wait, in seconds, before retrying after a failure.
    [Optional. Default is 10]

    timeout - The amount of time, in seconds, before the connection fails if
    there is no response.
    [Optional. Default is 3]

    debug_read - The level of message logging. The higher this number, the more
    information is logged.
    [Optional. Default is 0]
    """
    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)

        con_mode = stn_dict.get('mode', 'serial').lower()
        if con_mode == 'tcp' or con_mode == 'udp':
            port = stn_dict.get(
                'port', '%s:%d' % (DEFAULT_TCP_ADDR, DEFAULT_TCP_PORT))
        elif con_mode == 'serial':
            port = stn_dict.get('port', DEFAULT_SER_PORT)
        else:
            raise ValueError("Invalid driver connection mode %s" % con_mode)

        self.max_tries = int(stn_dict.get('max_tries', 5))
        self.wait_before_retry = float(stn_dict.get('wait_before_retry', 10))
        timeout = int(stn_dict.get('timeout', 3))

        self.last_rain = None

        loginf('using %s port %s' % (con_mode, port))

        global DEBUG_READ
        DEBUG_READ = int(stn_dict.get('debug_read', DEBUG_READ))

        if con_mode == 'tcp' or con_mode == 'udp':
            self.station = StationSocket(port, protocol=con_mode, 
                                         timeout=timeout,
                                         max_tries=self.max_tries, 
                                         wait_before_retry=self.wait_before_retry)
        else:
            self.station = StationSerial(port, timeout=timeout)
        self.station.open()

    def closePort(self):
        if self.station is not None:
            self.station.close()
            self.station = None

    @property
    def hardware_name(self):
        return "dfrobot"

    def genLoopPackets(self):
        while True:
            packet = {'dateTime': int(time.time() + 0.5),
                      'usUnits': weewx.US}
            readings = self.station.get_readings_with_retry(self.max_tries,
                                                            self.wait_before_retry)
            data = StationData.parse_readings(readings)
            packet.update(data)
            self._augment_packet(packet)
            yield packet

    def _augment_packet(self, packet):
        # calculate the rain delta from rain total
        packet['rain'] = weewx.wxformulas.calculate_rain(packet.get('rain_total'), self.last_rain)
        self.last_rain = packet.get('rain_total')


# =========================================================================== #
#       Station data class - parses and validates data from the device        #
# =========================================================================== #


class StationData(object):
    def __init__(self):
        pass

    @staticmethod
    def validate_string(buf):
        if len(buf) != PACKET_SIZE:
            raise weewx.WeeWxIOError("Unexpected buffer length %d" % len(buf))
        if buf[0:1] != 'c':
            raise weewx.WeeWxIOError("Unexpected header bytes '%s'" % buf[0:2])
	if StationData.invalid_checksum(buf):
            raise weewx.WeeWxIOError("Checksum error '%s'" % buf)
        return buf

    @staticmethod
    # VERY hacky - needs refinement
    def parse_readings(raw):
        buf = raw
	data = dict()
        data['windDir'] = StationData._decode(buf[1:4])  # compass deg
        data['windSpeed'] = StationData._decode(buf[5:8])*10 # mph
        data['wind_average'] = StationData._decode(buf[9:12])*10  # mph
	data['outTemp'] = StationData._decode(buf[13:16]) # Fahrenheit is expected from device
        data['rain_total'] = StationData._decode(buf[17:20])/100  # inch
        data['daily_rain'] = StationData._decode(buf[21:24])/100  # inch
        data['outHumidity'] = StationData._decode(buf[25:27])  # percent
        data['pressure'] = StationData._decode(buf[28:33])/10*INHG_PER_MBAR  # inHg 0.01554094892716838*
        return data

    @staticmethod
    # Determine is checksum is INVALID
    def invalid_checksum(datagram=None):
        if datagram==None:
            return(True)
        try:
            data,checksum=datagram.split("*")
        except:
            return(True)
        try:
            checksum=int(checksum.strip("\n\t\r"),16)
        except:
            return(True)
        calculated_sum = 0
        for el in data:
            try:
              calculated_sum ^=ord(el)
            except:
              return(True)
        if checksum==calculated_sum:
            return(False)
        else:
            return(True)

    @staticmethod
    def _decode(s):
        v = None
        try:
           v = float(s)
        except ValueError as e:
           if s!= '----':
             logdbg("decode failed for '%s': %s" % (s, e))
        return v


# =========================================================================== #
#          Station Serial class - Gets data through a serial port             #
# =========================================================================== #


class StationSerial(object):
    def __init__(self, port, timeout=3):
        self.port = port
        self.baudrate = 9600
        self.timeout = timeout
        self.serial_port = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):  # @UnusedVariable
        self.close()

    def open(self):
        import serial
        logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(self.port, self.baudrate,
                                         timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            logdbg("close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    # FIXME: use either CR or LF as line terminator.  apparently some dfrobot/WS1
    # hardware occasionally ends a line with only CR instead of the standard
    # CR-LF, resulting in a line that is too long.
    def get_readings(self):
        buf = self.serial_port.readline()
        if DEBUG_READ >= 2:
            logdbg("bytes: '%s'" % ' '.join(["%0.2X" % ord(c) for c in buf]))
        buf = buf.strip()
        return buf

    def get_readings_with_retry(self, max_tries=5, wait_before_retry=10):
        import serial
        for ntries in range(0, max_tries):
            try:
                buf = self.get_readings()
                StationData.validate_string(buf)
                return buf
            except (serial.serialutil.SerialException, weewx.WeeWxIOError) as e:
                loginf("Failed attempt %d of %d to get readings: %s" % (ntries + 1, max_tries, e))
                time.sleep(wait_before_retry)
        else:
            msg = "Max retries (%d) exceeded for readings" % max_tries
            logerr(msg)
            raise weewx.RetriesExceeded(msg)


# =========================================================================== #
#          Station TCP class - Gets data through a TCP/IP connection          #
#                  For those users with a serial->TCP adapter                 #
# =========================================================================== #
# Not used by DFRobot device - pruning candidate

class StationSocket(object):
    def __init__(self, addr, protocol='tcp', timeout=3, max_tries=5,
                 wait_before_retry=10):
        import socket

        ip_addr = None
        ip_port = None

        self.max_tries = max_tries
        self.wait_before_retry = wait_before_retry

        if addr.find(':') != -1:
            self.conn_info = addr.split(':')
            self.conn_info[1] = int(self.conn_info[1], 10)
            self.conn_info = tuple(self.conn_info)
        else:
            ip_addr = addr
            ip_port = DEFAULT_TCP_PORT
            self.conn_info = (ip_addr, ip_port)

        try:
            if protocol == 'tcp':
                self.net_socket = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM)
            elif protocol == 'udp':
                self.net_socket = socket.socket(
                    socket.AF_INET, socket.SOCK_DGRAM)
        except (socket.error, socket.herror) as ex:
            logerr("Cannot create socket for some reason: %s" % ex)
            raise weewx.WeeWxIOError(ex)

        self.net_socket.settimeout(timeout)
        self.rec_start = False

    def open(self):
        import socket

        logdbg("Connecting to %s:%d." % (self.conn_info[0], self.conn_info[1]))

        for conn_attempt in range(self.max_tries):
            try:
                if conn_attempt > 1:
                    logdbg("Retrying connection...")
                self.net_socket.connect(self.conn_info)
                break
            except (socket.error, socket.timeout, socket.herror) as ex:
                logerr("Cannot connect to %s:%d for some reason: %s. "
                       "%d tries left." % (
                           self.conn_info[0], self.conn_info[1], ex,
                           self.max_tries - (conn_attempt + 1)))
                logdbg("Will retry in %.2f seconds..." % self.wait_before_retry)
                time.sleep(self.wait_before_retry)
        else:
            logerr("Max tries (%d) exceeded for connection." %
                   self.max_tries)
            raise weewx.RetriesExceeded("Max tries exceeding while attempting connection")

    def close(self):
        import socket

        logdbg("Closing connection to %s:%d." %
               (self.conn_info[0], self.conn_info[1]))
        try:
            self.net_socket.close()
        except (socket.error, socket.herror, socket.timeout) as ex:
            logerr("Cannot close connection to %s:%d. Reason: %s" % (
                self.conn_info[0], self.conn_info[1], ex))
            raise weewx.WeeWxIOError(ex)

    def get_readings(self):
        import socket
        if self.rec_start is not True:
            # Find the record start
            if DEBUG_READ >= 1:
                logdbg("Attempting to find record start..")
            buf = ''
            while True:
                try:
                    buf += self.net_socket.recv(8, socket.MSG_WAITALL)
                except (socket.error, socket.timeout) as ex:
                    raise weewx.WeeWxIOError(ex)
                if DEBUG_READ >= 1:
                    logdbg("(searching...) buf: %s" % buf)
                if '!!' in buf:
                    self.rec_start = True
                    if DEBUG_READ >= 1:
                        logdbg("Record start found!")
                    # Cut to the record start
                    buf = buf[buf.find('!!'):]
                    if DEBUG_READ >= 1:
                        logdbg("(found!) buf: %s" % buf)
                    break
            # Add the rest of the record
            try:
                buf += self.net_socket.recv(
                    PACKET_SIZE - len(buf), socket.MSG_WAITALL)
            except (socket.error, socket.timeout) as ex:
                raise weewx.WeeWxIOError(ex)
        else:
            # Keep receiving data until we find an exclamation point or two
            try:
                buf = self.net_socket.recv(2, socket.MSG_WAITALL)
            except (socket.error, socket.timeout) as ex:
                raise weewx.WeeWxIOError(ex)
            while True:
                if buf == '\r\n':
                    # CRLF is expected
                    if DEBUG_READ >= 2:
                        logdbg("buf is CRLF")
                    buf = ''
                    break
                elif '!' in buf:
                    excmks = buf.count('!')
                    # Assuming exclamation points are at the end of the buffer
                    buf = buf[len(buf) - excmks:]
                    if DEBUG_READ >= 2:
                        logdbg("buf has %d exclamation points." % (excmks))
                    break
                else:
                    try:
                        buf = self.net_socket.recv(2, socket.MSG_WAITALL)
                    except (socket.error, socket.timeout) as ex:
                        raise weewx.WeeWxIOError(ex)
                    if DEBUG_READ >= 2:
                            logdbg("buf: %s" % ' '.join(
                                   ['%02X' % ord(bc) for bc in buf]))
            try:
                buf += self.net_socket.recv(
                    PACKET_SIZE - len(buf), socket.MSG_WAITALL)
            except (socket.error, socket.timeout) as ex:
                raise weewx.WeeWxIOError(ex)
        if DEBUG_READ >= 2:
            logdbg("buf: %s" % buf)

        buf.strip()
        return buf

    def get_readings_with_retry(self, max_tries=5, wait_before_retry=10):
        for _ in range(0, max_tries):
            buf = ''
            try:
                buf = self.get_readings()
                StationData.validate_string(buf)
                return buf
            except (weewx.WeeWxIOError) as e:
                logdbg("Failed to get data. Reason: %s" % e)
                self.rec_start = False

                # NOTE: WeeWx IO Errors may not always occur because of
                # invalid data. These kinds of errors are also caused by socket
                # errors and timeouts.

                if DEBUG_READ >= 1:
                    logdbg("buf: %s (%d bytes), rec_start: %r" %
                           (buf, len(buf), self.rec_start))

                time.sleep(wait_before_retry)
        else:
            msg = "Max retries (%d) exceeded for readings" % max_tries
            logerr(msg)
            raise weewx.RetriesExceeded(msg)


class dfrobotConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[dfrobot]
    # This section is for the ADS dfrobot series of weather stations.

    # Driver mode - tcp, udp, or serial
    mode = serial

    # If serial, specify the serial port device. (ex. /dev/ttyS0, /dev/ttyUSB0,
    # or /dev/cuaU0)
    # If TCP, specify the IP address and port number. (ex. 192.168.36.25:3000)
    port = /dev/ttyUSB0

    # The amount of time, in seconds, before the connection fails if there is
    # no response
    timeout = 3

    # The driver to use:
    driver = weewx.drivers.dfrobot
"""

    def prompt_for_settings(self):
        print "How is the station connected? tcp, udp, or serial."
        con_mode = self._prompt('mode', 'serial')
        con_mode = con_mode.lower()

        if con_mode == 'serial':
            print "Specify the serial port on which the station is connected, "
            "for example: /dev/ttyUSB0 or /dev/ttyS0."
            port = self._prompt('port', '/dev/ttyUSB0')
        elif con_mode == 'tcp' or con_mode == 'udp':
            print "Specify the IP address and port of the station. For "
            "example: 192.168.36.40:3000."
            port = self._prompt('port', '192.168.36.40:3000')

        print "Specify how long to wait for a response, in seconds."
        timeout = self._prompt('timeout', 3)

        return {'mode': con_mode, 'port': port, 'timeout': timeout}


# define a main entry point for basic testing of the station without weewx
# engine and service overhead.  invoke this as follows from the weewx root dir:
#
# PYTHONPATH=bin python bin/weewx/drivers/dfrobot.py

if __name__ == '__main__':
    import optparse

    usage = """%prog [options] [--help]"""

    syslog.openlog('dfrobot', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('--version', dest='version', action='store_true',
                      help='display driver version')
    parser.add_option('--port', dest='port', metavar='PORT',
                      help='serial port to which the station is connected',
                      default=DEFAULT_SER_PORT)
    (options, args) = parser.parse_args()

    if options.version:
        print "ADS dfrobot driver version %s" % DRIVER_VERSION
        exit(0)

    with StationSerial(options.port) as s:
        while True:
            print time.time(), s.get_readings()
