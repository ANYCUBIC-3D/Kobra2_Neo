#include "./autoGetZoffset.h"
#include "../../HAL/shared/eeprom_api.h"
#include "../../lcd/marlinui.h"


#if ENABLED(LEVEING_CALIBRATION_MODULE)

extern bed_mesh_t z_values;
AutoProbe autoProbe;
#define ADD_DEC  10
void AutoProbe::fan_control()
{
	 if(swtich_cool_fan == true){
	   if(thermalManager.degHotend(0)> 45 && thermalManager.degTargetHotend(0) > 0){
		   thermalManager.set_fan_speed(0, 255);
		   autoProbe.swtich_cool_fan = false;
	   }
	
	   
	}

}

void AutoProbe::set_fan_speed(uint8_t target, uint16_t speed){

		if(need_close_fan)  return;
		
		thermalManager.set_fan_speed(target,speed);
}

void AutoProbe::write_sdcard_log(const char * str)
{
	if(!autoProbe.sdcard_enabled) return;
	
		recovery.open(false);
		printf(str);
		recovery.file.writeln_P(str);
}

void AutoProbe::run_z_mm(float mm,uint8_t num)
{
	if(!need_z_down) return;
	enable_calibration_module = false;
	do_blocking_move_to_z(current_position.z+mm,5);
	safe_delay(500);

	enable_calibration_module = true;
}

void AutoProbe::Shake()
{
	if(need_shake == false) return;
	
	static uint32_t  Shake_timeout = 0;
	char cmd[64]= {0},str_1[16]={0},str_2[16]={0};
	
	 sprintf_P(cmd, PSTR("G1Y%s F3000\nG1 Y%s F3000"),
	 	dtostrf(calibration_positon.y-0.3, 1, 3, str_1),
		dtostrf(calibration_positon.y, 1, 3, str_2)
	 );
  
	Shake_timeout = millis();
	
	write_sdcard_log("AutoProbe::Shake()");
	
	while( READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING)
	{		
		if(millis() - Shake_timeout >5000)
		{
			write_sdcard_log("AutoProbe::Shake() timeout");
			break;
		}
		
    	gcode.process_subcommands_now(cmd);
		safe_delay(2);
	}
	

}
void AutoProbe::load_config()
{
	 read();
	 if (isnan(postion_data.leveing_postion.x) || isnan(postion_data.leveing_postion.y) ) 
	 {
		calibration_positon.x = NOZZLE_X;
		calibration_positon.y = NOZZLE_Y;
		calibration_positon.z = WIPE;
		frist_load_config = true;
		need_save_data = true;
		write_sdcard_log("load_config error or frist load_config");
		SERIAL_ECHOLNPGM_P("frist load config");
		write();
	}
	else if( (postion_data.leveing_postion.x > NOZZLE_X + ADD_DEC) || (postion_data.leveing_postion.x < NOZZLE_X - ADD_DEC) \
		|| (postion_data.leveing_postion.y > NOZZLE_Y + ADD_DEC) || (postion_data.leveing_postion.y < NOZZLE_Y - ADD_DEC))
	{
		
		calibration_positon.x = NOZZLE_X;
		calibration_positon.y = NOZZLE_Y;
		SERIAL_ECHOLNPGM_P("calibration x or y out of bounds");
		write_sdcard_log("calibration x or y out of bounds");
		need_save_data = true;
		write();
	}

	else
	{
		calibration_positon.x = postion_data.leveing_postion.x;
		calibration_positon.y = postion_data.leveing_postion.y;
		calibration_positon.z = postion_data.leveing_postion.z;
		SERIAL_ECHOLNPGM_P("load config sucess");
		write_sdcard_log("load config sucess");
	}

}


void AutoProbe::read()
{

	memcpy(postion_data.leveing_postion, (uint8_t *)(FLASH_AUTOGETZOFFSET_DATA_ADDR), sizeof(postion_data.leveing_postion));

}

void AutoProbe::clean()
{
	calibration_positon.x = postion_data.leveing_postion.x = NOZZLE_X;
	calibration_positon.y = postion_data.leveing_postion.y = NOZZLE_Y;
	calibration_positon.z =postion_data.leveing_postion.z = 0;

	if(persistentStore.FLASH_If_Erase(FLASH_AUTOGETZOFFSET_DATA_ADDR,FLASH_AUTOGETZOFFSET_DATA_ADDR+0x400) != FLASHIF_OK)
   	{
	 	SERIAL_ECHOLNPGM("erase error");
   	}
	
	if(persistentStore.FLASH_If_Write(FLASH_AUTOGETZOFFSET_DATA_ADDR, (uint32_t *)&autoProbe.postion_data, sizeof(autoProbe.postion_data)) !=FLASHIF_OK)
	{
		SERIAL_ECHOLNPGM_P("write error");
	}
	SERIAL_ECHOLNPGM_P("clean");
}

void AutoProbe::write()
{

  if(need_save_data ){
		postion_data.leveing_postion.x = calibration_positon.x;
		postion_data.leveing_postion.y = calibration_positon.y;
		postion_data.leveing_postion.z = calibration_positon.z ;
	   if(persistentStore.FLASH_If_Erase(FLASH_AUTOGETZOFFSET_DATA_ADDR,FLASH_AUTOGETZOFFSET_DATA_ADDR+0x400) != FLASHIF_OK)
   		{
	 		SERIAL_ECHOLNPGM("erase error");
   		}
		if(persistentStore.FLASH_If_Write(FLASH_AUTOGETZOFFSET_DATA_ADDR, (uint32_t *)&autoProbe.postion_data, sizeof(autoProbe.postion_data))!= FLASHIF_OK)
		{
			SERIAL_ECHOLNPGM_P("write error");
		}
		
		need_save_data =false;
  }

}



void AutoProbe::calculation()
{
	ui.lcdLeveingstate = LEVEING_DONE;

	LeveingTipStatus = 0x04;
	thermalManager.set_fan_speed(0, 0); 

	xyz_pos_t temp_lev; 
	xyz_pos_t temp_lev1;
	temp_lev.x = calibration_positon.x;
	temp_lev.y = Y_BED_SIZE - probe.offset_xy.y;
	temp_lev.z = 0;
	planner.apply_leveling(temp_lev);
	
	temp_lev1.x = calibration_positon.x;
	temp_lev1.y = Y_BED_SIZE - probe.offset_xy.y - 40 ;
	temp_lev1.z = 0;
  	planner.apply_leveling(temp_lev1);
	  
	
	float diff = temp_lev1.z - temp_lev.z;
	z_offset = module_probe_value - DEVIATION - SWTICH_DEVIATION + diff;
	probe.offset.z = z_offset + calibration_positon.z;
	
  	if(0)
	{
		char str[256] = {0};
		char str1[16] = {0};
		char str2[16] = {0};
		char str3[16] = {0};
		char str4[16] = {0};
		recovery.open(false);
		sprintf_P(str, PSTR("diff:%s calibration_positon.z:%s module_probe_value:%s  probe.offset.z:%s "),
							dtostrf(diff,1,3,str1), dtostrf(calibration_positon.z,1,3,str2),
							dtostrf(module_probe_value,1,3,str3), dtostrf(probe.offset.z,1,3,str4));
		serial_println_P(str);
		recovery.file.writeln_P(str);	
		recovery.file.writeln_P("Bilinear Leveling Grid:");	
		const float *values = bedlevel.z_values[0];
		for(int i = 0; i < GRID_MAX_POINTS_X; i++)
		{
			uint8_t count = 6;
			count *= (PROPORTIONAL_FONT_RATIO);
			while(count--){
				recovery.file.write_P(" ");
			}
			sprintf_P(str,"%d",i);
			//serial_print_P(str);
			recovery.file.write_P(str);

		}
		recovery.file.write_P("\n");
		//serial_print_P("\n");
		for(int y = 0; y < GRID_MAX_POINTS_Y; y++)
		{

				recovery.file.write_P(" ");

				sprintf_P(str,"%d ",y);
				//serial_print_P(str);
				recovery.file.write_P(str);
				for(int x = 0; x < GRID_MAX_POINTS_X; x++)
				{		
						memset(str,0,sizeof(str));
						float offset = values[ x*GRID_MAX_POINTS_Y + y];
						if (!isnan(offset)){
							if (offset >= 0) 
								sprintf_P(str,PSTR("+%s"),dtostrf(offset,1,3,str1));
							else
								sprintf_P(str,PSTR("%s "),dtostrf(offset,1,3,str1));
						}			
						//serial_print_P(str);
						recovery.file.write_P(str);
				}
				//serial_print_P("\n");
				recovery.file.write_P("\n");
		}

		recovery.file.writeln_P("====================================");
		recovery.close();
	}


	

}


bool AutoProbe::run_calibration_probe()
{
	

	xy_pos_t nozzle_position = {
    parser.linearval('X', current_position.x + probe.offset_xy.x),  // If no X use the probe's current X position
    parser.linearval('Y', current_position.y + probe.offset_xy.y)   // If no Y, ditto
  	};
			
		do_blocking_move_to_z(10,5);
		
		thermalManager.set_fan_speed(0, 255);
		thermalManager.setTargetHotend(NOZZLE_TEMP, 0);
		thermalManager.wait_for_hotend(0);
		
		while(thermalManager.degHotend(0) > (NOZZLE_TEMP + 10) )
		{
			idle();
			safe_delay(2000);
			
		}			
		ui.lcdLeveingstate = LEVEING_WIPE_NOZZLE;
		start_leveing = false;//
		soft_endstop._enabled = false;// Turn off the software limit
		LeveingFailSattue = false;

		LeveingTipStatus = 0x02;
		do_blocking_move_to_xy(X_MIN_POS,Y_MIN_POS,40);
		//Load the  filament and retract 
		thermalManager.set_fan_speed(0, 0);
		thermalManager.setTargetHotend(0, 0);
 		unscaled_e_move(LOAD_LENGTH,LOAD_SPEED);
		unscaled_e_move(-UNLOAD_LENGTH,UNLOAD_SPEED);
		//open fan and cool nozzle
		thermalManager.set_fan_speed(0, 0);
		thermalManager.setTargetHotend(0, 0);
		
		thermalManager.set_fan_speed(0, 255);
		//move to calibration module 
		do_blocking_move_to_xy(calibration_positon.x+25,calibration_positon.y,50);//88
		do_blocking_move_to_z(WIPE_Z,5);//
		//wipe nozzle	
		for(int i = 0; i< 3; i++)
		{
			do_blocking_move_to_xy(calibration_positon.x+25,calibration_positon.y,10);//88
			do_blocking_move_to_xy(calibration_positon.x+30,calibration_positon.y,10);//88

		}
		do_blocking_move_to_xy(calibration_positon.x+27,calibration_positon.y,10);

		//wait nozzle cooling		
		thermalManager.wait_for_hotend(0);		
		while(thermalManager.degHotend(0) > (NOZZLE_COOL_TEMP + 10) )//等待温度达到100以下
		{
			idle();
			safe_delay(2000);
		
		}
		thermalManager.set_fan_speed(0, 0);
		do_blocking_move_to_z(current_position.z+5,5);//抬升			
	
		LeveingTipStatus = 0x03;
		ui.lcdLeveingstate = LEVEING_PROBE;			

		nozzle_position.x = calibration_positon.x;
		nozzle_position.y = calibration_positon.y;
		enable_calibration_module = true;
		enable_probe_swtich = true;
		need_z_down = need_shake = true;	
		
		//check need shake?
		if( READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING){
				Shake();
		}	
		//start probing calibration module
		module_probe_value = probe_at_point_test(nozzle_position, PROBE_PT_RAISE, 0,false);
					
		//SERIAL_ECHOLNPAIR_F("module_probe_value:",module_probe_value);
		
		if (isnan(module_probe_value)) 
		{
			SERIAL_ECHOLNPGM("calibration failed");
			LeveingFailSattue = true;
			return 1;
		}
		

		planner.synchronize();
		
		do_blocking_move_to_z(current_position.z+10,5);// raise 
		planner.synchronize();

		enable_probe_swtich = false;
		enable_calibration_module = false;
		endstops.enable_globally(false);
		endstops.enable_z_probe(false);	
		probe.stow();
		soft_endstop._enabled = true;
		LeveingFailSattue = false;
		need_home_leveing = false;
		need_z_down = need_shake = need_move_x = false;
		
		return 0;

}

bool AutoProbe::ValibrationValueIsnan()
{
		
		if( run_calibration_probe())
		{
			//probe faild
			enable_calibration_module = false;
			enable_probe_swtich = 	false;	
			endstops.enable_globally(false);
			endstops.enable_z_probe(false);
			probe.stow();
			soft_endstop._enabled = true;
			return 1;
		}	
		
		//probe ok
		return 0;
}

bool AutoProbe::module_calibration()
{
		float z1;
		soft_endstop._enabled = false;
		enable_probe_swtich = true;		

		enable_calibration_module = true;
		endstops.z_probe_enabled = false;
		need_z_down = need_shake = true;
		LeveingFailSattue = false;
				
		do_blocking_move_to_z(current_position.z+5,5);

		do_blocking_move_to_xy(calibration_positon.x,calibration_positon.y,100);
		
		//
		if( READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING){
				Shake();

		}	
		//
		z1 = probe_at_point_test(calibration_positon, PROBE_PT_RAISE, 0,false);
	
		SERIAL_ECHOLNPAIR_F("module_calibration_z1:",z1);

		//
		can_move_calibration = true;
		enable_calibration_module = false;
		endstops.enable_globally(false);
		endstops.enable_z_probe(false);
		LeveingFailSattue = false;
		enable_probe_swtich = 	false;
		need_z_down = need_move_x = need_shake = false;
		planner.synchronize();
		
		probe.stow();
		//soft_endstop._enabled = true;
		thermalManager.set_fan_speed(0, 0);//
		//
		if (isnan(z1)) {
			LeveingFailSattue = true;

			return 1;
		}
		soft_endstop._enabled = true;
		LCD_MESSAGE(MSG_CALIBRATION_DONE);
		//calibration_finsh = true;
		//ui.goto_previous_screen();
    	//ui.previous_callbackFunc();
		return 0;
}


void GcodeSuite::M2000(){

	autoProbe.module_calibration();
	// 	float z1;
	// 	xy_pos_t nozzle_position = {
    // 	parser.linearval('X', current_position.x + probe.offset_xy.x),  // If no X use the probe's current X position
  	//  	parser.linearval('Y', current_position.y + probe.offset_xy.y)   // If no Y, ditto
  	// };
		
	// 	//printer_state = AC_printer_probing;
	// 	soft_endstop._enabled = false;// Turn off the software limit
	// 	autoProbe.enable_probe_swtich = 	false;
	// 	autoProbe.enable_calibration_module = true;
	// 	z1 = autoProbe.probe_at_point_test(autoProbe.calibration_positon, PROBE_PT_RAISE, 0,false);
		
	// 	autoProbe.enable_calibration_module = false;
	// 	endstops.enable_globally(false);
	// 	endstops.enable_z_probe(false);
	// 	planner.synchronize();
		
	// 	SERIAL_ECHOLNPAIR_F("nozzle_value:",z1);
		
	// 	if (isnan(z1)) return;
	
	// 	autoProbe.count++;
}

//
void GcodeSuite::M2001(){
	
	  if (!parser.seen("XYZ")) {
    	SERIAL_ECHOLNPGM_P(
        PSTR("Probe Module Offset" " X"), autoProbe.calibration_positon.x, SP_Y_STR, autoProbe.calibration_positon.y,SP_Z_STR
				,autoProbe.calibration_positon.z
			);
    return;
  }
		
  if (parser.seenval('X')) {
    const float x = parser.value_float();
		autoProbe.calibration_positon.x = x;
  }
	
	if (parser.seenval('Y')) {
    const float y = parser.value_float();
		autoProbe.calibration_positon.y = y;
  }
	
		if (parser.seenval('Z')) {
    const float z = parser.value_float();
		autoProbe.calibration_positon.z = z;
  }
}

//
void GcodeSuite::M2002(){

	if (!parser.seen("XYZ")) {
	  autoProbe.read();
	  SERIAL_ECHOLNPGM_P(
	  PSTR("Flash Probe Module Offset" " X"), autoProbe.postion_data.leveing_postion.x, SP_Y_STR, autoProbe.postion_data.leveing_postion.y,SP_Z_STR
			  ,autoProbe.postion_data.leveing_postion.z
		  );
  return;
}


}
//
void GcodeSuite::M2003()
{
	//write data
	if (parser.seen("W")){
		autoProbe.need_save_data = true;
		autoProbe.write();
	}

	//read data
	if (parser.seen("L")){
		autoProbe.load_config();
    	SERIAL_ECHOLNPGM_P(
        PSTR("Probe Module Offset" " X"), autoProbe.calibration_positon.x, SP_Y_STR, autoProbe.calibration_positon.y,SP_Z_STR
				,autoProbe.calibration_positon.z
			);
	}
	//clean data
	if (parser.seen("C")){
		autoProbe.clean();
	}

	if (parser.seen("P")){
		while(1);
	}
	
	SERIAL_ECHOLNPGM_P("dowen_error_count:",autoProbe.down_error_count);
	SERIAL_ECHOLNPGM_P("up_error_count:",autoProbe.up_error_count);
	
}

void GcodeSuite::M2005()
{
	if (parser.seen('L'))
		autoProbe.sdcard_enabled = parser.value_bool();	
	if (parser.seen('F'))
		autoProbe.need_close_fan = parser.value_bool();
	else {
	  SERIAL_ECHO_START();
	  SERIAL_ECHOPGM("Write Log statue: ");
	  serialprintln_onoff(autoProbe.sdcard_enabled);
		SERIAL_ECHOPGM("Leving Fan statue: ");
		serialprintln_onoff(!autoProbe.need_close_fan);
	}
}

#endif

