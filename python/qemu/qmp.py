""" QEMU Monitor Protocol Python class """
# Copyright (C) 2009, 2010 Red Hat Inc.
#
# Authors:
#  Luiz Capitulino <lcapitulino@redhat.com>
#
# This work is licensed under the terms of the GNU GPL, version 2.  See
# the COPYING file in the top-level directory.

import json
import errno
import socket
import logging


class QMPError(Exception):
    """
    QMP base exception
    """


class QMPConnectError(QMPError):
    """
    QMP connection exception
    """


class QMPCapabilitiesError(QMPError):
    """
    QMP negotiate capabilities exception
    """


class QMPTimeoutError(QMPError):
    """
    QMP timeout exception
    """


class QEMUMonitorProtocol:
    """
    Provide an API to connect to QEMU via QEMU Monitor Protocol (QMP) and then
    allow to handle commands and events.
    """

    #: Logger object for debugging messages
    logger = logging.getLogger('QMP')

    def __init__(self, address, server=False, nickname=None):
        """
        Create a QEMUMonitorProtocol class.

        @param address: QEMU address, can be either a unix socket path (string)
                        or a tuple in the form ( address, port ) for a TCP
                        connection
        @param server: server mode listens on the socket (bool)
        @raise OSError on socket connection errors
        @note No connection is established, this is done by the connect() or
              accept() methods
        """
        self.__events = []
        self.__address = address
        self.__sock = self.__get_sock()
        self.__sockfile = None
        self._nickname = nickname
        if self._nickname:
            self.logger = logging.getLogger('QMP').getChild(self._nickname)
        if server:
            self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__sock.bind(self.__address)
            self.__sock.listen(1)

    def __get_sock(self):
        if isinstance(self.__address, tuple):
            family = socket.AF_INET
        else:
            family = socket.AF_UNIX
        return socket.socket(family, socket.SOCK_STREAM)

    def __negotiate_capabilities(self):
        greeting = self.__json_read()
        if greeting is None or "QMP" not in greeting:
            raise QMPConnectError
        # Greeting seems ok, negotiate capabilities
        resp = self.cmd('qmp_capabilities')
        if resp and "return" in resp:
            return greeting
        raise QMPCapabilitiesError

    def __json_read(self, only_event=False):
        while True:
            data = self.__sockfile.readline()
            if not data:
                return None
            resp = json.loads(data)
            if 'event' in resp:
                self.logger.debug("<<< %s", resp)
                self.__events.append(resp)
                if not only_event:
                    continue
            return resp

    def __get_events(self, wait=False):
        """
        Check for new events in the stream and cache them in __events.

        @param wait (bool): block until an event is available.
        @param wait (float): If wait is a float, treat it as a timeout value.

        @raise QMPTimeoutError: If a timeout float is provided and the timeout
                                period elapses.
        @raise QMPConnectError: If wait is True but no events could be
                                retrieved or if some other error occurred.
        """

        # Check for new events regardless and pull them into the cache:
        self.__sock.setblocking(0)
        try:
            self.__json_read()
        except OSError as err:
            if err.errno == errno.EAGAIN:
                # No data available
                pass
        self.__sock.setblocking(1)

        # Wait for new events, if needed.
        # if wait is 0.0, this means "no wait" and is also implicitly false.
        if not self.__events and wait:
            if isinstance(wait, float):
                self.__sock.settimeout(wait)
            try:
                ret = self.__json_read(only_event=True)
            except socket.timeout:
                raise QMPTimeoutError("Timeout waiting for event")
            except:
                raise QMPConnectError("Error while reading from socket")
            if ret is None:
                raise QMPConnectError("Error while reading from socket")
            self.__sock.settimeout(None)

    def __enter__(self):
        # Implement context manager enter function.
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        # Implement context manager exit function.
        self.close()
        return False

    def connect(self, negotiate=True):
        """
        Connect to the QMP Monitor and perform capabilities negotiation.

        @return QMP greeting dict, or None if negotiate is false
        @raise OSError on socket connection errors
        @raise QMPConnectError if the greeting is not received
        @raise QMPCapabilitiesError if fails to negotiate capabilities
        """
        self.__sock.connect(self.__address)
        self.__sockfile = self.__sock.makefile()
        if negotiate:
            return self.__negotiate_capabilities()
        return None

    def accept(self, timeout=15.0):
        """
        Await connection from QMP Monitor and perform capabilities negotiation.

        @param timeout: timeout in seconds (nonnegative float number, or
                        None). The value passed will set the behavior of the
                        underneath QMP socket as described in [1]. Default value
                        is set to 15.0.
        @return QMP greeting dict
        @raise OSError on socket connection errors
        @raise QMPConnectError if the greeting is not received
        @raise QMPCapabilitiesError if fails to negotiate capabilities

        [1]
        https://docs.python.org/3/library/socket.html#socket.socket.settimeout
        """
        self.__sock.settimeout(timeout)
        self.__sock, _ = self.__sock.accept()
        self.__sockfile = self.__sock.makefile()
        return self.__negotiate_capabilities()

    def cmd_obj(self, qmp_cmd):
        """
        Send a QMP command to the QMP Monitor.

        @param qmp_cmd: QMP command to be sent as a Python dict
        @return QMP response as a Python dict or None if the connection has
                been closed
        """
        self.logger.debug(">>> %s", qmp_cmd)
        try:
            self.__sock.sendall(json.dumps(qmp_cmd).encode('utf-8'))
        except OSError as err:
            if err.errno == errno.EPIPE:
                return None
            raise err
        resp = self.__json_read()
        self.logger.debug("<<< %s", resp)
        return resp

    def cmd(self, name, args=None, cmd_id=None):
        """
        Build a QMP command and send it to the QMP Monitor.

        @param name: command name (string)
        @param args: command arguments (dict)
        @param cmd_id: command id (dict, list, string or int)
        """
        qmp_cmd = {'execute': name}
        if args:
            qmp_cmd['arguments'] = args
        if cmd_id:
            qmp_cmd['id'] = cmd_id
        return self.cmd_obj(qmp_cmd)

    def command(self, cmd, **kwds):
        """
        Build and send a QMP command to the monitor, report errors if any
        """
        ret = self.cmd(cmd, kwds)
        if "error" in ret:
            raise Exception(ret['error']['desc'])
        return ret['return']

    def pull_event(self, wait=False):
        """
        Pulls a single event.

        @param wait (bool): block until an event is available.
        @param wait (float): If wait is a float, treat it as a timeout value.

        @raise QMPTimeoutError: If a timeout float is provided and the timeout
                                period elapses.
        @raise QMPConnectError: If wait is True but no events could be
                                retrieved or if some other error occurred.

        @return The first available QMP event, or None.
        """
        self.__get_events(wait)

        if self.__events:
            return self.__events.pop(0)
        return None

    def get_events(self, wait=False):
        """
        Get a list of available QMP events.

        @param wait (bool): block until an event is available.
        @param wait (float): If wait is a float, treat it as a timeout value.

        @raise QMPTimeoutError: If a timeout float is provided and the timeout
                                period elapses.
        @raise QMPConnectError: If wait is True but no events could be
                                retrieved or if some other error occurred.

        @return The list of available QMP events.
        """
        self.__get_events(wait)
        return self.__events

    def clear_events(self):
        """
        Clear current list of pending events.
        """
        self.__events = []

    def close(self):
        """
        Close the socket and socket file.
        """
        if self.__sock:
            self.__sock.close()
        if self.__sockfile:
            self.__sockfile.close()

    def settimeout(self, timeout):
        """
        Set the socket timeout.

        @param timeout (float): timeout in seconds, or None.
        @note This is a wrap around socket.settimeout
        """
        self.__sock.settimeout(timeout)

    def get_sock_fd(self):
        """
        Get the socket file descriptor.

        @return The file descriptor number.
        """
        return self.__sock.fileno()

    def is_scm_available(self):
        """
        Check if the socket allows for SCM_RIGHTS.

        @return True if SCM_RIGHTS is available, otherwise False.
        """
        return self.__sock.family == socket.AF_UNIX
