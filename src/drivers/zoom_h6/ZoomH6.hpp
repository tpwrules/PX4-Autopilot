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

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>

using namespace time_literals;

#define ZOOM_H6_COMM_INTERVAL     (100_ms) // 10Hz
#define ZOOM_H6_BUFFER_SIZE (3) // maximum number of bytes that need to be received at once
#define ZOOM_H6_CONTROL_IDX (4) // which actuator_controls_2 control to look at

enum comm_state_t {
	connect, // connect to the recorder by sending null bytes
	handshake, // wait for the handshake reply string
	watch, // wait for the recorder to update us on the LED state
	idle, // monitor the recorder LEDs in case something changes
	push, // push the appropriate recorder buttons
	confirm, // wait for the recorder to confirm that the button was pressed
};

static const uint8_t handshake_msg[] = {
	0xc2, 0xe1, 0x31, 0x2e, 0x30, 0x30, 0xa1, 0x80, 0x80,
};

static const uint8_t handshake_reply_msg[] = {
	0x83, 0x80, 0x81,
};

static const uint8_t push_record_msg[] = {
	0x81, 0x00, 0x80, 0x00,
};

static const uint8_t push_stop_msg[] = {
	0x90, 0x00, 0x80, 0x00,
};

class ZoomH6 : public px4::ScheduledWorkItem
{
public:
	ZoomH6(const char *serial_port);
	~ZoomH6() override;

	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Initialise the communication state machine and start it.
	 */
	void start();

	/**
	 * Stop the communication state machine.
	 */
	void stop();

private:

	/**
	 * Receive exactly the given number of bytes from the serial port.
	 * Returns the number of bytes currently in the buffer, which can be no
	 * more than `nbytes`. If equal to `nbytes`, then the buffer is cleared.
	 */
	int receive_exactly(int nbytes);

	/**
	 * Update the communications state machine.
	 */
	comm_state_t step_comm();

	/**
	 * Open and configure the serial port.
	 */
	int open_serial_port();

	void Run() override;

	uORB::Subscription _control_sub{ORB_ID(actuator_controls_2)};

	const char *_serial_port{nullptr};
	int _fd{-1};
	uint8_t _buffer[ZOOM_H6_BUFFER_SIZE];
	uint8_t _buffer_len{0};

	comm_state_t _comm_state{comm_state_t::connect};
	int _timeout{0}; // timeout for various state machine functions
	bool _is_recording{false}; // true if we know the recorder is recording
	bool _should_be_recording{false}; // true if we want the recorder to be recording

	perf_counter_t _comms_error{perf_alloc(PC_COUNT, MODULE_NAME": comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sample")};
};
