/*
 * system.cpp
 *
 *  Created on: 2017-9-19
 *      Author: wangbo
 */

#include "copter.h"
void Copter::init_ardupilot()
{

	// init the GCS，地面站以Serial这个硬件驱动串口初始化
	char uart_device[]={'u','a','r','t','\0'};
	gcs0.init(uart_device);

#if 0
	// Console serial port
	//
	Serial.begin(SERIAL0_BAUD, 128, 128);

	// GPS serial port.
	//
	Serial1.begin(38400, 128, 16);

	//
	// Initialize Wire and SPI libraries
	//
    I2c.begin();
    I2c.timeOut(5);
    // initially set a fast I2c speed, and drop it on first failures
    I2c.setSpeed(true);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // 1MHZ SPI rate

	//
	// Initialize the isr_registry.
	//
    isr_registry.init();

	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	report_version();

	/*
	 * 如果固件版本加载成功，则从flash中读取加载所有参数
	 */
	if (!g.format_version.load() ||
	     g.format_version != Parameters::k_format_version) {
		//Serial.printf_P(PSTR("\n\nForcing complete parameter reset..."));

		/*Serial.printf_P(PSTR("\n\nEEPROM format version  %d not compatible with this firmware (requires %d)"
		                     "\n\nForcing complete parameter reset..."),
		                     g.format_version.get(),
		                     Parameters::k_format_version);
		*/

		// erase all parameters
		Serial.printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
		delay(100); // wait for serial send
		AP_Var::erase_all();

		// save the new format version
		g.format_version.set_and_save(Parameters::k_format_version);

		// save default radio values
		default_dead_zones();
	}else{
		// save default radio values
		//default_dead_zones();

	    // Load all auto-loaded EEPROM variables
	    AP_Var::load_all();
	}

	// init the GCS，地面站以Serial这个硬件驱动串口初始化
    gcs0.init(&Serial);

    // identify ourselves correctly with the ground station
	mavlink_system.sysid = g.sysid_this_mav;

	/*
	 * 日志初始化
	 */
#if LOGGING_ENABLED == ENABLED
    DataFlash.Init();
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
		do_erase_logs();
    }
	if (g.log_bitmask != 0){
		DataFlash.start_new_log();
	}
#endif

    RC_Channel::set_apm_rc(&APM_RC);
	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

	init_camera();

    timer_scheduler.init( &isr_registry );

	// Do GPS init
	g_gps = &g_gps_driver;
	g_gps->init();			// GPS Initialization
    g_gps->callback = mavlink_delay;

	if(g.compass_enabled)
		init_compass();

	// init the optical flow sensor
	if(g.optflow_enabled) {
		init_optflow();
	}

    GPS_enabled = false;

	#if HIL_MODE == HIL_MODE_DISABLED
    // Read in the GPS
	for (byte counter = 0; ; counter++) {
		g_gps->update();
		if (g_gps->status() != 0){
			GPS_enabled = true;
			break;
		}

		if (counter >= 2) {
			GPS_enabled = false;
			break;
	    }
	}
	#else
		GPS_enabled = true;
	#endif

	// lengthen the idle timeout for gps Auto_detect
	// ---------------------------------------------
	g_gps->idleTimeout = 20000;

	// print the GPS status
	// --------------------
	report_gps();

	// read Baro pressure at ground
	//-----------------------------
	init_barometer();

	// initialise sonar
	#if CONFIG_SONAR == ENABLED
	init_sonar();
	#endif

	// initialize commands
	// -------------------
	init_commands();

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();

	dcm.kp_roll_pitch(0.130000);
	dcm.ki_roll_pitch(0.00001278),	// 50 hz I term
	dcm.kp_yaw(0.08);
	dcm.ki_yaw(0.00004);
	dcm._clamp = 5;

	// init the Z damopener
	// --------------------
	#if ACCEL_ALT_HOLD != 0
	init_z_damper();
	#endif

	startup_ground();

#if LOGGING_ENABLED == ENABLED
	Log_Write_Startup();
	Log_Write_Data(10, g.pi_stabilize_roll.kP());
	Log_Write_Data(11, g.pi_stabilize_pitch.kP());
	Log_Write_Data(12, g.pid_rate_roll.kP());
	Log_Write_Data(13, g.pid_rate_pitch.kP());
#endif

	SendDebug("\nReady to FLY ");
#endif
}


