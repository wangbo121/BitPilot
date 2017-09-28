/*
 * commands.cpp
 *
 *  Created on: 2017-8-20
 *      Author: wangbo
 */

#include "copter.h"


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

 void Copter:: init_commands()
{
    g.command_index 		= NO_COMMAND;
	command_nav_index		= NO_COMMAND;
	command_cond_index		= NO_COMMAND;
	prev_nav_index 			= NO_COMMAND;
	command_cond_queue.id 	= NO_COMMAND;
	command_nav_queue.id 	= NO_COMMAND;

	// default Yaw tracking
	yaw_tracking 			= MAV_ROI_WPNEXT;
}

// Getters
// -------
 struct Location Copter::get_cmd_with_index(int i)
{
	struct Location temp;

	// Find out proper location in memory by using the start_byte position + the index
	// --------------------------------------------------------------------------------
	if (i >= g.command_total) {
		// we do not have a valid command to load
		// return a WP with a "Blank" id
		temp.id = CMD_BLANK;

		// no reason to carry on
		return temp;

	}else{
		// we can load a command, we don't process it yet
		// read WP position
		/*
		 * 下面这里是读取航点信息的程序片段
		 * 20170820我把航点先放在数组里进行测试
		 */

		memcpy(&temp,&wp_total_array[i],sizeof(struct Location));

#if  0


		int32_t mem = (WP_START_BYTE) + (i * WP_SIZE);

		temp.id = eeprom_read_byte((uint8_t*)mem);

		mem++;
		temp.options = eeprom_read_byte((uint8_t*)mem);

		mem++;
		temp.p1 = eeprom_read_byte((uint8_t*)mem);

		mem++;
		temp.alt = eeprom_read_dword((uint32_t*)mem);	// alt is stored in CM! Alt is stored relative!

		mem += 4;
		temp.lat = eeprom_read_dword((uint32_t*)mem); // lat is stored in decimal * 10,000,000

		mem += 4;
		temp.lng = eeprom_read_dword((uint32_t*)mem); // lon is stored in decimal * 10,000,000
#endif
	}

	// Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
	//if((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) && temp.options & MASK_OPTIONS_RELATIVE_ALT){
		//temp.alt += home.alt;
	//}

	if(temp.options & WP_OPTION_RELATIVE){
		// If were relative, just offset from home
		temp.lat	+=	home.lat;
		temp.lng	+=	home.lng;
	}

	return temp;
}

// Setters
// -------
 void Copter:: set_cmd_with_index(struct Location temp, int i)
{

	i = constrain(i, 0, g.command_total);
	//Serial.printf("set_command: %d with id: %d\n", i, temp.id);

	// store home as 0 altitude!!!
	// Home is always a MAV_CMD_NAV_WAYPOINT (16)
	if (i == 0){
		temp.alt = 0;
		temp.id = MAV_CMD_NAV_WAYPOINT;
	}

	/*
	 * 20170820下面是把地面站发送过来的航点信息保存到flash中去，
	 * 我还是把航点放在数组里
	 */

	temp.lng=-temp.lng;//20170928只是为了显示，实际使用中这个不需要反号

	memcpy(&wp_total_array[i],&temp,sizeof(struct Location));

#if 0
	uint32_t mem = WP_START_BYTE + (i * WP_SIZE);

	eeprom_write_byte((uint8_t *)	mem, temp.id);

	mem++;
	eeprom_write_byte((uint8_t *)	mem, temp.options);

	mem++;
	eeprom_write_byte((uint8_t *)	mem, temp.p1);

	mem++;
	eeprom_write_dword((uint32_t *)	mem, temp.alt);	// Alt is stored in CM!

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lat);	// Lat is stored in decimal degrees * 10^7

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lng); // Long is stored in decimal degrees * 10^7
#endif
	// Make sure our WP_total
	if(g.command_total < (i+1))
		//g.command_total.set_and_save(i+1);
		g.command_total=i+1;//应该就是这个意思
}

 int32_t Copter::read_alt_to_hold()
{
	return current_loc.alt;
	/*
	if(g.RTL_altitude <= 0)
		return current_loc.alt;
	else
		return g.RTL_altitude;// + home.alt;
	*/
}


//********************************************************************************
// This function sets the waypoint and modes for Return to Launch
// It's not currently used
//********************************************************************************

/*
This function sets the next waypoint command
It precalculates all the necessary stuff.
*/

 void Copter:: set_next_WP(struct Location *wp)
{
	//SendDebug("MSG <set_next_wp> wp_index: ");
	//SendDebugln(g.command_index, DEC);

		std::cout<<"进入set next wp"<<std::endl;
		std::cout<<"set_next_WP    prev_WP.lng="<<prev_WP.lng<<std::endl;
		std::cout<<"set_next_WP    prev_WP.lat="<<prev_WP.lat<<std::endl;

	// copy the current WP into the OldWP slot
	// ---------------------------------------
	if (next_WP.lat == 0 || command_nav_index <= 1){
		std::cout<<"command_nav_index ="<<command_nav_index <<std::endl;
		prev_WP = current_loc;
	}else{
		//if (get_distance(&current_loc, &next_WP) < 10)
		if (get_distance(&current_loc, &next_WP) < 100)//改了100
		{
			std::cout<<"set next wp success"<<std::endl;
			prev_WP = next_WP;
		}
		else
			prev_WP = current_loc;
	}

	//Serial.printf("set_next_WP #%d, ", command_nav_index);
	//print_wp(&prev_WP, command_nav_index -1);

	// Load the next_WP slot
	// ---------------------
	next_WP = *wp;

	// used to control and limit the rate of climb
	// -------------------------------------------
	// We don't set next WP below 1m
	next_WP.alt = max(next_WP.alt, 100);

	// Save new altitude so we can track it for climb_rate
	set_new_altitude(next_WP.alt);//这个函数应该在navigation.cpp文件中写的，这里还没有写所以先注释掉

	// this is used to offset the shrinking longitude as we go towards the poles
	float rads 			= (fabs((float)next_WP.lat)/t7) * 0.0174532925;
	scaleLongDown 		= cos(rads);
	scaleLongUp 		= 1.0f/cos(rads);

	// this is handy for the groundstation
	// -----------------------------------
	wp_distance 		= get_distance(&current_loc, &next_WP);
	target_bearing 		= get_bearing_cd(&prev_WP, &next_WP);////这个函数应该在navigation.cpp文件中写的，这里还没有写所以先注释掉
	nav_bearing 		= target_bearing;

	// calc the location error:
	calc_location_error(&next_WP);

	// to check if we have missed the WP
	// ---------------------------------

	original_target_bearing = target_bearing;

	std::cout<<"set_next_WP original_target_bearing="<<original_target_bearing<<std::endl;
	std::cout<<"set_next_WP target_bearing="<<target_bearing<<std::endl;

	// reset speed governer
	// --------------------
	waypoint_speed_gov = WAYPOINT_SPEED_MIN;
}


// run this at setup on the ground
// -------------------------------
 void Copter:: init_home()
{
	home_is_set = true;
	home.id 	= MAV_CMD_NAV_WAYPOINT;
//	home.lng 	= gps.longitude;				// Lon * 10**7
//	home.lat 	= gps.latitude;				// Lat * 10**7


	home.lng 	= wp_total_array[0].lng;				// Lon * 10**7
	home.lat 	= wp_total_array[0].lat;				// Lat * 10**7



	home.alt 	= 0;							// Home is always 0

	// to point yaw towards home until we set it with Mavlink
	target_WP 	= home;

	// Save Home to EEPROM
	// -------------------
	// no need to save this to EPROM
	set_cmd_with_index(home, 0);
	//print_wp(&home, 0);

	// Save prev loc this makes the calcs look better before commands are loaded
	prev_WP = home;

	// in case we RTL
	next_WP = home;

	// Load home for a default guided_WP
	// -------------
	guided_WP = home;
	guided_WP.alt += g.RTL_altitude;
}




