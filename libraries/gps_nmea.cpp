/*
 * gps_nmea.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */


/// @file	AP_GPS_NMEA.cpp
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.
///

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include "gps_nmea.h"
#include "fdm.h"
#include "BIT_MATH.h"
#include "all_external_device.h"

// SiRF init messages //////////////////////////////////////////////////////////
//
// Note that we will only see a SiRF in NMEA mode if we are explicitly configured
// for NMEA.  GPS_AUTO will try to set any SiRF unit to binary mode as part of
// the autodetection process.
//
const char AP_GPS_NMEA::_SiRF_init_string[]  =
	"$PSRF103,0,0,1,1*25\r\n"	// GGA @ 1Hz
	"$PSRF103,1,0,0,1*25\r\n"	// GLL off
	"$PSRF103,2,0,0,1*26\r\n"	// GSA off
	"$PSRF103,3,0,0,1*27\r\n"	// GSV off
	"$PSRF103,4,0,1,1*20\r\n"	// RMC off
	"$PSRF103,5,0,1,1*20\r\n"	// VTG @ 1Hz
	"$PSRF103,6,0,0,1*22\r\n"	// MSS off
	"$PSRF103,8,0,0,1*2C\r\n"	// ZDA off
	"$PSRF151,1*3F\r\n"			// WAAS on (not always supported)
	"$PSRF106,21*0F\r\n"		// datum = WGS84
	"";

// MediaTek init messages //////////////////////////////////////////////////////
//
// Note that we may see a MediaTek in NMEA mode if we are connected to a non-DIYDrones
// MediaTek-based GPS.
//
const char AP_GPS_NMEA::_MTK_init_string[]  =
	"$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"	// GGA & VTG once every fix
	"$PMTK330,0*2E"										// datum = WGS84
	"$PMTK313,1*2E\r\n"									// SBAS on
	"$PMTK301,2*2E\r\n"									// use SBAS data for DGPS
	"";

// ublox init messages /////////////////////////////////////////////////////////
//
// Note that we will only see a ublox in NMEA mode if we are explicitly configured
// for NMEA.  GPS_AUTO will try to set any ublox unit to binary mode as part of
// the autodetection process.
//
// We don't attempt to send $PUBX,41 as the unit must already be talking NMEA
// and we don't know the baudrate.
//
const char AP_GPS_NMEA::_ublox_init_string[]  =
	"$PUBX,40,gga,0,1,0,0,0,0*7B\r\n"	// GGA on at one per fix
	"$PUBX,40,vtg,0,1,0,0,0,0*7F\r\n"	// VTG on at one per fix
	"$PUBX,40,rmc,0,0,0,0,0,0*67\r\n"	// RMC off (XXX suppress other message types?)
	"";

// NMEA message identifiers ////////////////////////////////////////////////////
//
const char AP_GPS_NMEA::_gprmc_string[]  = "GPRMC";
const char AP_GPS_NMEA::_gpgga_string[]  = "GPGGA";
const char AP_GPS_NMEA::_gpvtg_string[]  = "GPVTG";

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)	(_x - '0')

#if 0
// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_NMEA::AP_GPS_NMEA(Stream *s) :
	GPS(s)
{
	FastSerial	*fs = (FastSerial *)_port;

	// Re-open the port with enough receive buffering for the messages we expect
	// and very little tx buffering, since we don't care about sending.
	// Leave the port speed alone as we don't actually know at what rate we're running...
	//
	fs->begin(0, 200, 16);
}
#endif

// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_NMEA::init(void)
{
#if 0
	BetterStream	*bs = (BetterStream *)_port;

	// send the SiRF init strings
	bs->print_P((const prog_char_t *)_SiRF_init_string);

	// send the MediaTek init strings
	bs->print_P((const prog_char_t *)_MTK_init_string);

	// send the ublox init strings
	bs->print_P((const prog_char_t *)_ublox_init_string);
#endif
}

bool AP_GPS_NMEA::read(void)
{
	int numc;
	bool parsed = false;

#if 0
	numc = _port->available();
	while (numc--) {
		if (_decode(_port->read())) {
			parsed = true;
		}
	}
#endif


//	longitude=1161234567;
//	latitude=391234567;
//	altitude=10000;//100米，单位厘米
//	ground_speed=10;
//	ground_course=90;

#if 1
	longitude=(fdm_feed_back.longitude *RAD_TO_DEG)*1e7;
	latitude=(fdm_feed_back.latitude *RAD_TO_DEG)*1e7;
	altitude=(fdm_feed_back.altitude )*1e2;
	ground_speed=sqrt(pow(fdm_feed_back.v_north,2)+pow(fdm_feed_back.v_east,2)+pow(fdm_feed_back.v_down,2));
	//ground_course=90;
#else
	longitude=(long)all_external_device_input.longitude;
	latitude=(long)all_external_device_input.latitude;
	altitude=(long)all_external_device_input.altitude;
	ground_speed=(long)sqrt(pow(all_external_device_input.v_north,2)+pow(all_external_device_input.v_east,2)+pow(all_external_device_input.v_down,2));

#endif

//	std::cout<<"gps nmea longitude="<<longitude*1e-7<<std::endl;
//	std::cout<<"gps nmea latitude="<<latitude*1e-7<<std::endl;
//	std::cout<<"gps nmea altitude="<<altitude*1e-2<<std::endl;
//	std::cout<<"gps nmea ground_speed="<<ground_speed<<std::endl;
//	std::cout<<"gps nmea ground_course="<<ground_course<<std::endl;




	return parsed;
}

bool AP_GPS_NMEA::_decode(char c)
{
	bool valid_sentence = false;

	switch (c) {
	case ',': // term terminators
		_parity ^= c;
	case '\r':
	case '\n':
	case '*':
		if (_term_offset < sizeof(_term)) {
			_term[_term_offset] = 0;
			valid_sentence = _term_complete();
		}
		++_term_number;
		_term_offset = 0;
		_is_checksum_term = c == '*';
		return valid_sentence;

	case '$': // sentence begin
		_term_number = _term_offset = 0;
		_parity = 0;
		_sentence_type = _GPS_SENTENCE_OTHER;
		_is_checksum_term = false;
		_gps_data_good = false;
		return valid_sentence;
	}

	// ordinary characters
	if (_term_offset < sizeof(_term) - 1)
		_term[_term_offset++] = c;
	if (!_is_checksum_term)
		_parity ^= c;

	return valid_sentence;
}

//
// internal utilities
//
int AP_GPS_NMEA::_from_hex(char a)
{
	if (a >= 'A' && a <= 'F')
		return a - 'A' + 10;
	else if (a >= 'a' && a <= 'f')
		return a - 'a' + 10;
	else
		return a - '0';
}

unsigned long AP_GPS_NMEA::_parse_decimal()
{
	char *p = _term;
	unsigned long ret = 100UL * atol(p);
	while (isdigit(*p))
		++p;
	if (*p == '.') {
		if (isdigit(p[1])) {
			ret += 10 * (p[1] - '0');
			if (isdigit(p[2]))
				ret += p[2] - '0';
		}
	}
	return ret;
}

unsigned long AP_GPS_NMEA::_parse_degrees()
{
	char *p, *q;
	uint8_t deg = 0, min = 0;
	unsigned int frac_min = 0;

	// scan for decimal point or end of field
	for (p = _term; isdigit(*p); p++)
		;
	q = _term;

	// convert degrees
	while ((p - q) > 2) {
		if (deg)
			deg *= 10;
		deg += DIGIT_TO_VAL(*q++);
	}

	// convert minutes
	while (p > q) {
		if (min)
			min *= 10;
		min += DIGIT_TO_VAL(*q++);
	}

	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.') {
		q = p + 1;
		for (int i = 0; i < 4; i++) {
			frac_min *= 10;
			if (isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	return deg * 100000UL + (min * 10000UL + frac_min) / 6;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool AP_GPS_NMEA::_term_complete()
{
	// handle the last term in a message
	if (_is_checksum_term) {
		uint8_t checksum = 16 * _from_hex(_term[0]) + _from_hex(_term[1]);
		if (checksum == _parity) {
			if (_gps_data_good) {
				switch (_sentence_type) {
				case _GPS_SENTENCE_GPRMC:
					time			= _new_time;
					date			= _new_date;
					latitude		= _new_latitude * 100;	// degrees*10e5 -> 10e7
					longitude		= _new_longitude * 100;	// degrees*10e5 -> 10e7
					ground_speed	= _new_speed;
					ground_course	= _new_course;
					num_sats		= _new_satellite_count;
					hdop			= _new_hdop;
					fix				= true;
					break;
				case _GPS_SENTENCE_GPGGA:
					altitude		= _new_altitude;
					time			= _new_time;
					latitude		= _new_latitude * 100;	// degrees*10e5 -> 10e7
					longitude		= _new_longitude * 100;	// degrees*10e5 -> 10e7
					fix				= true;
					break;
				case _GPS_SENTENCE_VTG:
					ground_speed	= _new_speed;
					ground_course	= _new_course;
					// VTG has no fix indicator, can't change fix status
					break;
				}
			} else {
				switch (_sentence_type) {
				case _GPS_SENTENCE_GPRMC:
				case _GPS_SENTENCE_GPGGA:
					// Only these sentences give us information about
					// fix status.
					fix = false;
				}
			}
			// we got a good message
			return true;
		}
		// we got a bad message, ignore it
		return false;
	}

	// the first term determines the sentence type
	if (_term_number == 0) {
		if (!strcmp(_term, _gprmc_string)) {
			_sentence_type = _GPS_SENTENCE_GPRMC;
		} else if (!strcmp(_term, _gpgga_string)) {
			_sentence_type = _GPS_SENTENCE_GPGGA;
		} else if (!strcmp(_term, _gpvtg_string)) {
			_sentence_type = _GPS_SENTENCE_VTG;
			// VTG may not contain a data qualifier, presume the solution is good
			// unless it tells us otherwise.
			_gps_data_good = true;
		} else {
			_sentence_type = _GPS_SENTENCE_OTHER;
		}
		return false;
	}

	// 10 = RMC, 20 = GGA, 30 = VTG
	if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
		switch (_sentence_type + _term_number) {
		// operational status
		//
		case 12: // validity (RMC)
			_gps_data_good = _term[0] == 'A';
			break;
		case 26: // Fix data (GGA)
			_gps_data_good = _term[0] > '0';
			break;
		case 39: // validity (VTG) (we may not see this field)
			_gps_data_good = _term[0] != 'N';
			break;
		case 27: // satellite count (GGA)
			_new_satellite_count = atol(_term);
			break;
		case 28: // HDOP (GGA)
			_new_hdop = _parse_decimal();
			break;

			// time and date
			//
		case 11: // Time (RMC)
		case 21: // Time (GGA)
			_new_time = _parse_decimal();
			break;
		case 19: // Date (GPRMC)
			_new_date = atol(_term);
			break;

			// location
			//
		case 13: // Latitude
		case 22:
			_new_latitude = _parse_degrees();
			break;
		case 14: // N/S
		case 23:
			if (_term[0] == 'S')
				_new_latitude = -_new_latitude;
			break;
		case 15: // Longitude
		case 24:
			_new_longitude = _parse_degrees();
			break;
		case 16: // E/W
		case 25:
			if (_term[0] == 'W')
				_new_longitude = -_new_longitude;
			break;
		case 29: // Altitude (GPGGA)
			_new_altitude = _parse_decimal();
			break;

			// course and speed
			//
		case 17: // Speed (GPRMC)
			_new_speed = (_parse_decimal() * 514) / 1000; 	// knots-> m/sec, approximiates * 0.514
			break;
		case 37: // Speed (VTG)
			_new_speed = _parse_decimal();
			break;
		case 18: // Course (GPRMC)
		case 31: // Course (VTG)
			_new_course = _parse_decimal();
			break;
		}
	}

	return false;
}

