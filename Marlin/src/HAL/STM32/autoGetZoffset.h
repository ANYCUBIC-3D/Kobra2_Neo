#ifndef 	__AUTOGETOFFSET_H_
#define	  	__AUTOGETOFFSET_H_

#include "../../module/probe.h"
#include "../../gcode/gcode.h"
#include "../../module/endstops.h"
#include "../../module/temperature.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../module/planner.h"
#include "../../feature/powerloss.h"
#include "../../core/serial.h"

#define NOZZLE_X    60 		      //calibration module x pos
#define NOZZLE_Y    240			  //calibration module y pos
#define WIPE_Z    	-2.0		  //
#define WIPE        0
#define DEVIATION   -0.1      
#define SWTICH_DEVIATION   -0.1 
 
#define NOZZLE_TEMP 			220 		
#define NOZZLE_COOL_TEMP 	140 			

#define LOAD_LENGTH   		120		//load length
#define LOAD_SPEED			8	   	//load speed
#define UNLOAD_LENGTH 		50		//unload length	
#define UNLOAD_SPEED  		20		//unload speed	

#define RUN_DOWN_MM       -1.2      
#define MOVE_XY_MM        10

#define FLASH_AUTOGETZOFFSET_DATA_ADDR   0x0807E800


typedef struct {
	xyz_pos_t leveing_postion;
}autoprobe_data;



class AutoProbe
{
	public:
		
		autoprobe_data postion_data; 
		uint32_t up_error_count;
		uint32_t down_error_count;
	  	bool sdcard_enabled;						//
		bool enable_calibration_module;//
		bool enable_probe_swtich;//
		bool frist_load_config;	//
		bool start_leveing;//
		bool need_save_data;	//
		bool need_z_down;//
		bool need_move_x;//
		bool need_shake;//
		bool need_close_fan;//
		bool swtich_cool_fan;
		bool need_home_leveing;//
	  	bool LeveingFailSattue;	//probe faild
	  	bool calibration_finsh;
		bool can_move_calibration;
		uint8_t LeveingTipStatus;//
		uint32_t count;
		xyz_pos_t calibration_positon;	//
		xyz_pos_t home_position;
		float z_offset;

		void set_fan_speed(uint8_t target, uint16_t speed);			
		void write_sdcard_log(const char * str);
		void run_z_mm(float mm,uint8_t num);
		void calculation();
		void Shake();
		void write();
		void clean();
		void read();
		void load_config();
		bool ValibrationValueIsnan();
	  	bool module_calibration();
		void fan_control();
		static float probe_at_point_test(const xy_pos_t &pos, const ProbePtRaise raise_after, const uint8_t verbose_level=0,  bool probe_relative=true, const bool sanity_check=true) {
      		return probe.probe_at_point(pos.x, pos.y, raise_after, verbose_level, probe_relative, sanity_check);
		}
		
	private:
		float module_probe_value,nozzle_deviation,switch_deviation;
		bool run_calibration_probe();

};

extern AutoProbe autoProbe;
#endif
