/****************************************************************************
 *
 *   Copyright (C) 2021 Thomas Watson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ZoomH6.hpp"

#include <uORB/topics/actuator_controls.h>

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

ZoomH6::ZoomH6(const char *serial_port):
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port))
{
	_serial_port = strdup(serial_port);
}

ZoomH6::~ZoomH6()
{
	stop();

	free((char *)_serial_port);
	perf_free(_comms_error);
	perf_free(_sample_perf);
}

int
ZoomH6::receive_exactly(int nbytes)
{
	int to_get = nbytes - _buffer_len;
	if (to_get == 0) {
		return _buffer_len;
	}

	int got = ::read(_fd, _buffer+_buffer_len, to_get);
	if (got < 0) { // (probably) nothing to read
		return _buffer_len;
	}
	_buffer_len += got;

	return _buffer_len;
}

comm_state_t
ZoomH6::step_comm()
{
	if (_timeout > 0) {
		_timeout = _timeout - 1;
	}

	switch (_comm_state) {
	case comm_state_t::connect:
		// did we receive the ack character?
		if ((receive_exactly(1) == 1) && (_buffer[0] == 0x82)) {
			_buffer_len = 0; // we have consumed the ack
			// yes, send the rest of the handshake
			int ret = ::write(_fd, handshake_msg, sizeof(handshake_msg));
			(void)ret; // we assume it is all sent, and if not, then we will time out and try again
			_timeout = 10; // give the recorder one second to reply
			return comm_state_t::handshake;
		} else {
			// make sure there isn't any junk from previous attempts
			tcflush(_fd, TCIOFLUSH);
			_buffer_len = 0;

			// try to connect to the recorder again
			uint8_t val = 0x00;
			int ret = ::write(_fd, &val, 1);
			(void)ret;
			return comm_state_t::connect;
		}
		// should not get here
		return comm_state_t::connect;

	case comm_state_t::handshake:
		// did we receive the handshake reply?
		if ((receive_exactly(3) == 3) && (memcmp(_buffer, handshake_reply_msg, 3) == 0)) {
			_buffer_len = 0; // we have consumed the reply
			// yes, push the right button to make sure the state
			// machine matches up. this also makes sure we start a
			// new recording if we are already recording.
			return comm_state_t::push;
		} else if (_timeout == 0) {
			// we didn't receive anything in time, reset communication
			return comm_state_t::connect;
		} else {
			// waiting for reply still...
			return comm_state_t::handshake;
		}
		// should not get here
		return comm_state_t::connect;

	case comm_state_t::watch:
		if (receive_exactly(3) == 3) { // is there an LED packet?
			_buffer_len = 0; // we have consumed the packet
			if (_buffer[0] != 0x84) {
				// reset communication if it's invalid
				return comm_state_t::connect;
			}
			// check if the RECORD LED is lit
			_is_recording = !!(_buffer[1] & 1);
			_timeout = 10; // receive more LED packets
			return comm_state_t::watch;
		} else if (_timeout == 0) { // recorder has stopped talking about LEDs
			if (receive_exactly(1) > 0) {
				// reset communication if there is a partial packet
				return comm_state_t::connect;
			} else if (_is_recording != _should_be_recording) {
				// push the button to set the correct state
				return comm_state_t::push;
			} else {
				// listen for more LED packets in case somebody
				// pushes a button
				return comm_state_t::idle;
			}
		} else {
			// listen for more LED packets
			return comm_state_t::watch;
		}
		// should not get here
		return comm_state_t::connect;

	case comm_state_t::idle:
		if (receive_exactly(1) > 0) { // the LEDs have changed
			return comm_state_t::watch;
		} else {
			return comm_state_t::idle;
		}
		// should not get here
		return comm_state_t::connect;

	case comm_state_t::push:
		if (_should_be_recording) {
			// press the RECORD button. if already recording, it
			// will stop, then we will notice the LED is not lit
			// and press the button again to start a new recording
			int ret = ::write(_fd, push_record_msg, sizeof(push_record_msg));
			(void)ret; // if it's not all sent then we just time out
		} else {
			// press the STOP button to stop the recording
			int ret = ::write(_fd, push_stop_msg, sizeof(push_stop_msg));
			(void)ret; // if it's not all sent then we just time out
		}
		_timeout = 10; // wait for the recorder to update the LEDs
		return comm_state_t::confirm;
		// should not get here
		return comm_state_t::connect;

	case comm_state_t::confirm:
		if (receive_exactly(1) > 0) { // the LEDs have changed
			return comm_state_t::watch;
		} else if (_timeout == 0) { // the recorder never got back to us
			return comm_state_t::connect; // so reset communication
		} else {
			// waiting for the LEDs to change
			return comm_state_t::confirm;
		}
		// should not get here
		return comm_state_t::connect;
	}

	// should not get here
	return comm_state_t::connect;
}

int
ZoomH6::init()
{
	if (open_serial_port() != PX4_OK) {
		return PX4_ERROR;
	}

	// The file descriptor can only be accessed by the process that opened it,
	// so closing here allows the port to be opened from scheduled work queue.
	stop();

	_comm_state = comm_state_t::connect;
	_should_be_recording = false;

	return PX4_OK;
}

int
ZoomH6::open_serial_port()
{
	int speed = B2400; // no option to change

	// File descriptor already initialized?
	if (_fd > 0) {
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_fd = ::open(_serial_port, flags);

	if (_fd < 0) {
		PX4_ERR("open failed (%i) (%s)", errno, _serial_port);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	if (tcgetattr(_fd, &uart_config)) {
		PX4_ERR("Unable to get termios from %s.", _serial_port);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	// ignore breaks and errors (state machine will just time out and reset)
	uart_config.c_iflag = (IGNBRK | IGNPAR);
	uart_config.c_oflag = 0;
	// 8 bit characters, enable reception, ignore modem control lines
	uart_config.c_cflag = (CS8 | CREAD | CLOCAL);
	uart_config.c_lflag = 0;

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_fd, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	// Flush the hardware buffers.
	tcflush(_fd, TCIOFLUSH);

	PX4_DEBUG("opened UART port %s", _serial_port);

	return PX4_OK;
}

void
ZoomH6::print_info()
{
	perf_print_counter(_comms_error);
	perf_print_counter(_sample_perf);
}

void
ZoomH6::Run()
{
	// open the serial port if not already open
	if (_fd < 0) {
		if (open_serial_port() != PX4_OK) {
			return;
		}
	}

	// check to see if the recorder should on or off
	bool updated = false;
	bool should_be_recording = false;

	actuator_controls_s controls;
	if (_control_sub.update(&controls)) {
		float val = controls.control[ZOOM_H6_CONTROL_IDX];
		updated = true;
		if (val <= -0.875f) { // >= servo value of 1750
			should_be_recording = true;
		} else if (val >= -0.625f) { // <= servo value of 1250
			should_be_recording = false;
		} else {
			updated = false;
		}
	}

	if (updated && (_should_be_recording != should_be_recording)) {
		_should_be_recording = should_be_recording;
		// push the button to update the status ASAP. even if we are in
		// the wrong state tihs will eventually time out and restat
		// communication, which will make sure we restart recording if
		// we are already recording.
		_comm_state = comm_state_t::push;
	}

	_comm_state = step_comm(); // communicate with the recorder
}

void
ZoomH6::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(ZOOM_H6_COMM_INTERVAL, ZOOM_H6_COMM_INTERVAL);
}

void
ZoomH6::stop()
{
	// Ensure the serial port is closed.
	::close(_fd);
	_fd = -1;

	// Clear the work queue schedule.
	ScheduleClear();
}
