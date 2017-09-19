/*
 * commands_process.cpp
 *
 *  Created on: 2017-8-20
 *      Author: wangbo
 */


#include "copter.h"

// For changing active command mid-mission
//----------------------------------------
 void Copter:: Copter:: change_command(uint8_t cmd_index)
{
	//Serial.printf("change_command: %d\n",cmd_index );
	// limit range
	cmd_index = min(g.command_total-1, cmd_index);

	// load command
	struct Location temp = get_cmd_with_index(cmd_index);

	//Serial.printf("loading cmd: %d with id:%d\n", cmd_index, temp.id);

	// verify it's a nav command
	if (temp.id > MAV_CMD_NAV_LAST ){
		//gcs_send_text_P(SEVERITY_LOW,PSTR("error: non-Nav cmd"));

	} else {
		// clear out command queue
		init_commands();

		// copy command to the queue
		command_nav_queue		= temp;
		command_nav_index 		= cmd_index;
		execute_nav_command();
	}
}

// called by 10 Hz loop
// --------------------
 void Copter:: update_commands()
{
	//Serial.printf("update_commands: %d\n",increment );
	// A: if we do not have any commands there is nothing to do
	// B: We have completed the mission, don't redo the mission
	// XXX debug
	//uint8_t tmp = g.command_index.get();
	//Serial.printf("command_index %u \n", tmp);

//	 std::cout<<"g.command_total ="<<g.command_total <<std::endl;
//	 std::cout<<"g.command_index ="<<g.command_index <<std::endl;
	 printf("g.command_total =%d\n",g.command_total);
	 printf("g.command_index =%d\n",g.command_index);

	if (g.command_total <= 1 || g.command_index >= 127)
		return;

	printf("update_commands   command_nav_queue.id  =%d\n",command_nav_queue.id );
	printf("update_commands   command_nav_index =%d\n",command_nav_index );


	if(command_nav_queue.id == NO_COMMAND){
		// Our queue is empty
		// fill command queue with a new command if available, or exit mission
		// -------------------------------------------------------------------

		//command_nav_index=0;//测试

		command_nav_index=command_nav_index%(g.command_total -1);//20170919添加这个的目的主要是为了能够循环执行命令
		if (command_nav_index < (g.command_total -1)) {

			std::cout << "进入了command_nav_index < (g.command_total -1) "<<std::endl;
			command_nav_index++;
			command_nav_queue = get_cmd_with_index(command_nav_index);


			if (command_nav_queue.id <= MAV_CMD_NAV_LAST ){
				std::cout<<"execute_nav_command();"<<std::endl;
				execute_nav_command();
			} else{
				// this is a conditional command so we skip it
				command_nav_queue.id = NO_COMMAND;
			}
		}else{
			/*
			 * 完成任务后按道理说应该悬停或者降落
			 * 我还没有写，暂时先把command_nav_index归零吧，重复进行
			 */

			// we are out of commands
			//g.command_index  = command_nav_index = 255;//255对应有符号是-1

			/*
			 * 不知道为什么回不来了，难道还有啥设置，设置为1了，为啥还不能朝1走
			 * 0821好像可以了，执行任务结束后直接回到航点1
			 */
			std::cout<<"任务已经完成，重新执行任务"<<std::endl;
			init_commands();
			g.command_index  = command_nav_index = 1;//255对应有符号是-1
			command_nav_queue = get_cmd_with_index(command_nav_index);
			//set_next_WP(&command_nav_queue);
			reset_nav_params();
			execute_nav_command();


			// if we are on the ground, enter stabilize, else Land
			if (land_complete == true){
				// we will disarm the motors after landing.
			} else {
				//set_mode(LAND);
			}
		}
	}
#if 0
	if(command_cond_queue.id == NO_COMMAND){
		// Our queue is empty
		// fill command queue with a new command if available, or do nothing
		// -------------------------------------------------------------------

		// no nav commands completed yet
		if (prev_nav_index == NO_COMMAND)
			return;

		if (command_cond_index >= command_nav_index){
			// don't process the fututre
			//command_cond_index = NO_COMMAND;
			return;

		}else if (command_cond_index == NO_COMMAND){
			// start from scratch
			// look at command after the most recent completed nav
			command_cond_index = prev_nav_index + 1;

		}else{
			// we've completed 1 cond, look at next command for another
			command_cond_index++;
		}

		if(command_cond_index < (g.command_total -2)){
			// we're OK to load a new command (last command must be a nav command)
			command_cond_queue = get_cmd_with_index(command_cond_index);

			if (command_cond_queue.id > MAV_CMD_CONDITION_LAST){
				// this is a do now command
				process_now_command();

				// clear command queue
				command_cond_queue.id = NO_COMMAND;

			}else if (command_cond_queue.id > MAV_CMD_NAV_LAST ){
				// this is a conditional command
				process_cond_command();

			}else{
				// this is a nav command, don't process
				// clear the command conditional queue and index
				prev_nav_index			= NO_COMMAND;
				command_cond_index 		= NO_COMMAND;
				command_cond_queue.id 	= NO_COMMAND;
			}

		}
	}
#endif
}

 void Copter:: execute_nav_command(void)
{
//	// This is what we report to MAVLINK
	g.command_index  = command_nav_index;
//
//	// Save CMD to Log
//	if (g.log_bitmask & MASK_LOG_CMD)
//		Log_Write_Cmd(g.command_index, &command_nav_queue);
//
//	// clear navigation prameters
	reset_nav_params();
//
//	// Act on the new command
	process_nav_command();
//
//	// clear May indexes to force loading of more commands
//	// existing May commands are tossed.
	/*
	 * 先舍弃下面的cond index
	 */
	//command_cond_index	= NO_COMMAND;//这个是cond index也就是条件命令，跟上面的nav_commands没有关系
}

// called with GPS navigation update - not constantly
 void Copter:: verify_commands(void)
{
	 /*
	  * 这个函数就是确定有没有到达目标航点，确认是否完成command
	  * 如果完成则把命令导航的队列清零，即command_nav_queue.id = NO_COMMAND;
	  */
	if(verify_must()){
		//Serial.printf("verified must cmd %d\n" , command_nav_index);
		command_nav_queue.id = NO_COMMAND;

		// store our most recent executed nav command
		prev_nav_index = command_nav_index;

		/*
		 * 下面几句是cond_queue的，跟导航没关系
		 */
		// Wipe existing conditionals
#if 0
		command_cond_index 		= NO_COMMAND;
		command_cond_queue.id 	= NO_COMMAND;

		printf("verify must  command_cond_queue.id=%d\n",command_cond_queue.id);
		std::cout<<"verify must    original_target_bearing="<<original_target_bearing<<std::endl;
#endif

	}else{
		//Serial.printf("verified must false %d\n" , command_nav_index);
		std::cout<<"verify_must failed 也就是还没有到达目标航点"<<std::endl;
	}

//	if(verify_may()){
//		//Serial.printf("verified may cmd %d\n" , command_cond_index);
//		command_cond_queue.id = NO_COMMAND;
//	}
}
