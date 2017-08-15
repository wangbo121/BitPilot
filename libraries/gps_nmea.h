/*
 * gps_nmea.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef GPS_NMEA_H_
#define GPS_NMEA_H_


#include "gps.h"

/// NMEA parser
///
class AP_GPS_NMEA : public GPS
{
public:
	/// Constructor
	///
	//AP_GPS_NMEA(Stream *s);
	AP_GPS_NMEA(){}

	/// Perform a (re)initialisation of the GPS; sends the
	/// protocol configuration messages.
	///
	virtual void	init();

	/// Checks the serial receive buffer for characters,
	/// attempts to parse NMEA data and updates internal state
	/// accordingly.
	///
	 bool	read();

private:
	/// Coding for the GPS sentences that the parser handles
    enum _sentence_types {
    	_GPS_SENTENCE_GPRMC = 10,
    	_GPS_SENTENCE_GPGGA = 20,
    	_GPS_SENTENCE_VTG   = 30,
    	_GPS_SENTENCE_OTHER = 0
    };

	/// Update the decode state machine with a new character
	///
	/// @param	c		The next character in the NMEA input stream
	/// @returns		True if processing the character has resulted in
	///					an update to the GPS state
	///
	bool			_decode(char c);

	/// Return the numeric value of an ascii hex character
	///
	/// @param	a		The character to be converted
	/// @returns		The value of the character as a hex digit
	///
	int 			_from_hex(char a);

	/// Parses the current term as a NMEA-style decimal number with
	/// up to two decimal digits.
	///
	/// @returns		The value expressed by the string in _term,
	///					multiplied by 100.
	///
	unsigned long	_parse_decimal();

	/// Parses the current term as a NMEA-style degrees + minutes
	/// value with up to four decimal digits.
	///
	/// This gives a theoretical resolution limit of around 18cm.
	///
	/// @returns		The value expressed by the string in _term,
	///					multiplied by 10000.
	///
	unsigned long	_parse_degrees();

	/// Processes the current term when it has been deemed to be
	/// complete.
	///
	/// Each GPS message is broken up into terms separated by commas.
	/// Each term is then processed by this function as it is received.
	///
	/// @returns		True if completing the term has resulted in
	///					an update to the GPS state.
	bool			_term_complete();

    uint8_t			_parity;				///< NMEA message checksum accumulator
    bool			_is_checksum_term;		///< current term is the checksum
    char			_term[15];				///< buffer for the current term within the current sentence
    uint8_t			_sentence_type;			///< the sentence type currently being processed
    uint8_t			_term_number;			///< term index within the current sentence
    uint8_t			_term_offset;			///< character offset with the term being received
    bool			_gps_data_good;			///< set when the sentence indicates data is good

    // The result of parsing terms within a message is stored temporarily until
    // the message is completely processed and the checksum validated.
    // This avoids the need to buffer the entire message.
    long 			_new_time;				///< time parsed from a term
    long 			_new_date;				///< date parsed from a term
    long 			_new_latitude;			///< latitude parsed from a term
    long 			_new_longitude;			///< longitude parsed from a term
    long 			_new_altitude;			///< altitude parsed from a term
    long			_new_speed;				///< speed parsed from a term
    long			_new_course;			///< course parsed from a term
    int 			_new_hdop;				///< HDOP parsed from a term
    uint8_t			_new_satellite_count;	///< satellite count parsed from a term

    /// @name	Init strings
    ///			In ::init, an attempt is made to configure the GPS
    ///			unit to send just the messages that we are interested
    ///			in using these strings
    //@{
    static const char	_SiRF_init_string[];	///< init string for SiRF units
    static const char	_MTK_init_string[];		///< init string for MediaTek units
    static const char	_ublox_init_string[];	///< init string for ublox units
    //@}

    /// @name	GPS message identifier strings
    //@{
    static const char	_gprmc_string[];
    static const char	_gpgga_string[];
    static const char	_gpvtg_string[];
    //@}
};




#endif /* GPS_NMEA_H_ */
