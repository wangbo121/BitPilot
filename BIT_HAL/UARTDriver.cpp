/*
 * AP_HAL::UARTDriver.cpp
 *
 *  Created on: 2017-8-6
 *      Author: wangbo
 */



#include "BIT_HAL.h"


#include "UARTDriver.h"
#include "Print.h"

#include <stdio.h>

#include <stdlib.h>
#include <errno.h>

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>


#include <assert.h>


extern const AP_HAL::HAL& hal;

AP_HAL::UARTDriver::UARTDriver(const char *devpath, const char *perf_name)
{}




/*
  this UART driver maps to a serial device in /dev
 */

void AP_HAL::UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
	if (!_initialised) {
		_fd = open(_devpath, O_RDWR);
		if (_fd == -1) {
			fprintf(stdout, "Failed to open UART device %s - %s\n",
				_devpath, strerror(errno));
			return;
		}

        if (rxS == 0) {
            rxS = 128;
        }
        if (txS == 0) {
            txS = 128;
        }
	}

    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);

	if (b != 0) {
		// set the baud rate
		struct termios t;
		tcgetattr(_fd, &t);
		cfsetspeed(&t, b);
		// disable LF -> CR/LF
		t.c_oflag &= ~ONLCR;
		tcsetattr(_fd, TCSANOW, &t);
	}

    /*
      allocate the read buffer
     */
	if (rxS != 0 && rxS != _readbuf_size) {
		_readbuf_size = rxS;
		if (_readbuf != NULL) {
			free(_readbuf);
		}
		_readbuf = (uint8_t *)malloc(_readbuf_size);
		_readbuf_head = 0;
		_readbuf_tail = 0;
	}

    /*
      allocate the write buffer
     */
	if (txS != 0 && txS != _writebuf_size) {
		_writebuf_size = txS;
		if (_writebuf != NULL) {
			free(_writebuf);
		}
		_writebuf = (uint8_t *)malloc(_writebuf_size+16);
		_writebuf_head = 0;
		_writebuf_tail = 0;
	}

    if (_writebuf_size != 0 && _readbuf_size != 0) {
        _initialised = true;
    }
}

void AP_HAL::UARTDriver::begin(uint32_t b)
{
	begin(b, 0, 0);
}


void AP_HAL::UARTDriver::end() {}
void AP_HAL::UARTDriver::flush() {}
bool AP_HAL::UARTDriver::is_initialized() { return true; }
void AP_HAL::UARTDriver::set_blocking_writes(bool blocking)
{
    _nonblocking_writes = !blocking;
}
bool AP_HAL::UARTDriver::tx_pending() { return false; }

/* PX4 implementations of BetterStream virtual methods */
void AP_HAL::UARTDriver::print_P(const char *pstr) {
	print(pstr);
}

void AP_HAL::UARTDriver::println_P(const char *pstr) {
	println(pstr);
}

void AP_HAL::UARTDriver::printf(const char *fmt, ...) {

}

void AP_HAL::UARTDriver::_printf_P(const char *fmt, ...) {

}

void AP_HAL::UARTDriver::vprintf(const char *fmt, va_list ap) {
    _vprintf(fmt, ap);
}

void AP_HAL::UARTDriver::vprintf_P(const char *fmt, va_list ap) {
    _vprintf(fmt, ap);
}


void AP_HAL::UARTDriver::_internal_vprintf(const char *fmt, va_list ap)
{

}

// handle %S -> %s
void AP_HAL::UARTDriver::_vprintf(const char *fmt, va_list ap)
{
    if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
        return;
    }
    // we don't use vdprintf() as it goes directly to the file descriptor
	if (strstr(fmt, "%S")) {
		char *fmt2 = strdup(fmt);
		if (fmt2 != NULL) {
			for (uint16_t i=0; fmt2[i]; i++) {
				if (fmt2[i] == '%' && fmt2[i+1] == 'S') {
					fmt2[i+1] = 's';
				}
			}
            _internal_vprintf(fmt2, ap);
			free(fmt2);
		}
	} else {
        _internal_vprintf(fmt, ap);
	}
}


/*
  buffer handling macros
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size

/*
  return number of bytes available to be read from the buffer
 */
int16_t AP_HAL::UARTDriver::available()
{
	if (!_initialised) {
		return 0;
	}
    uint16_t _tail;
    return BUF_AVAILABLE(_readbuf);
}

/*
  return number of bytes that can be added to the write buffer
 */
int16_t AP_HAL::UARTDriver::txspace()
{
	if (!_initialised) {
		return 0;
	}
    uint16_t _head;
    return BUF_SPACE(_writebuf);
}

/*
  read one byte from the read buffer
 */
int16_t AP_HAL::UARTDriver::read()
{
	uint8_t c;
	if (!_initialised || _readbuf == NULL) {
		return -1;
	}
    if (BUF_EMPTY(_readbuf)) {
        return -1;
    }
    c = _readbuf[_readbuf_head];
    BUF_ADVANCEHEAD(_readbuf, 1);
	return c;
}

/*
   write one byte to the buffer
 */
size_t AP_HAL::UARTDriver::write(uint8_t c)
{
    if (!_initialised) {
        return 0;
    }
    if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
        return 0;
    }
    uint16_t _head;

    while (BUF_SPACE(_writebuf) == 0) {
        if (_nonblocking_writes) {
            return 0;
        }
        hal.scheduler->delay(1);
    }
    _writebuf[_writebuf_tail] = c;
    BUF_ADVANCETAIL(_writebuf, 1);
    return 1;
}

/*
  write size bytes to the write buffer
 */
size_t AP_HAL::UARTDriver::write(const uint8_t *buffer, size_t size)
{
	if (!_initialised) {
		return 0;
	}
    if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
        return 0;
    }

    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    uint16_t _head, space;
    space = BUF_SPACE(_writebuf);
    if (space == 0) {
        return 0;
    }
    if (size > space) {
        size = space;
    }
    if (_writebuf_tail < _head) {
        // perform as single memcpy
        assert(_writebuf_tail+size <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], buffer, size);
        BUF_ADVANCETAIL(_writebuf, size);
        return size;
    }

    // perform as two memcpy calls
    uint16_t n = _writebuf_size - _writebuf_tail;
    if (n > size) n = size;
    assert(_writebuf_tail+n <= _writebuf_size);
    memcpy(&_writebuf[_writebuf_tail], buffer, n);
    BUF_ADVANCETAIL(_writebuf, n);
    buffer += n;
    n = size - n;
    if (n > 0) {
        assert(_writebuf_tail+n <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], buffer, n);
        BUF_ADVANCETAIL(_writebuf, n);
    }
    return size;
}

/*
  try writing n bytes, handling an unresponsive port
 */
int AP_HAL::UARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    int ret = 0;


    return ret;
}

/*
  try reading n bytes, handling an unresponsive port
 */
int AP_HAL::UARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    int ret = 0;

    // the FIONREAD check is to cope with broken O_NONBLOCK behaviour
    // in NuttX on ttyACM0
    int nread = 0;
    if (ioctl(_fd, FIONREAD, (unsigned long)&nread) == 0) {
        if (nread > n) {
            nread = n;
        }
        if (nread > 0) {
            ret = ::read(_fd, buf, nread);
        }
    }
    if (ret > 0) {
        BUF_ADVANCETAIL(_readbuf, ret);
    }
    return ret;
}


/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void AP_HAL::UARTDriver::_timer_tick(void)
{
    uint16_t n;

    if (!_initialised) return;

    _in_timer = true;



    _in_timer = false;
}


