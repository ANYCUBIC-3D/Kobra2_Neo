/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfigPre.h"
#include "../../HAL/STM32/autoGetZoffset.h"

#if HAS_MARLINUI_MENU

#include "menu.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include "../../module/printcounter.h"
#include "../../module/temperature.h"
#include "../../gcode/queue.h"
#include "../tft/tft.h"

#if HAS_SOUND
  #include "../../libs/buzzer.h"
#endif

#if ENABLED(BABYSTEP_ZPROBE_OFFSET)
  #include "../../module/probe.h"
#endif

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

////////////////////////////////////////////
///////////// Global Variables /////////////
////////////////////////////////////////////
bool calibration_state = false;
#if HAS_LEVELING && ANY(LCD_BED_TRAMMING, PROBE_OFFSET_WIZARD, X_AXIS_TWIST_COMPENSATION)
  bool leveling_was_active; // = false
#endif
#if ANY(PROBE_MANUALLY, MESH_BED_LEVELING, X_AXIS_TWIST_COMPENSATION)
  uint8_t manual_probe_index; // = 0
#endif

// Menu Navigation
int8_t encoderTopLine, encoderLine, screen_items;

typedef struct {
  screenFunc_t menu_function;     // The screen's function
  uint32_t encoder_position;      // The position of the encoder
  int8_t top_line, items;         // The amount of scroll, and the number of items
  #if HAS_SCREEN_TIMEOUT
    bool sticky;                  // The screen is sticky
  #endif
} menuPosition;
menuPosition screen_history[6];
uint8_t screen_history_depth = 0;

int8_t MenuItemBase::itemIndex;         // Index number for draw and action
FSTR_P MenuItemBase::itemStringF;       // A string for substitution
const char *MenuItemBase::itemStringC;
chimera_t editable;                     // Value Editing

// Menu Edit Items
FSTR_P       MenuEditItemBase::editLabel;
void*        MenuEditItemBase::editValue;
int32_t      MenuEditItemBase::minEditValue,
             MenuEditItemBase::maxEditValue;
screenFunc_t MenuEditItemBase::callbackFunc;
bool         MenuEditItemBase::liveEdit;

////////////////////////////////////////////
//////// Menu Navigation & History /////////
////////////////////////////////////////////

void MarlinUI::return_to_status() { goto_screen(status_screen); }

void MarlinUI::push_current_screen() {
  if (screen_history_depth < COUNT(screen_history))
    screen_history[screen_history_depth++] = { currentScreen, encoderPosition, encoderTopLine, screen_items OPTARG(HAS_SCREEN_TIMEOUT, screen_is_sticky()) };
}

void MarlinUI::_goto_previous_screen(TERN_(TURBO_BACK_MENU_ITEM, const bool is_back/*=false*/)) {
  IF_DISABLED(TURBO_BACK_MENU_ITEM, constexpr bool is_back = false);
  TERN_(HAS_TOUCH_BUTTONS, on_edit_screen = false);
  if (screen_history_depth > 0) {
    menuPosition &sh = screen_history[--screen_history_depth];
    goto_screen(sh.menu_function,
      is_back ? 0 : sh.encoder_position,
      is_back ? 0 : sh.top_line,
      sh.items
    );
    defer_status_screen(TERN_(HAS_SCREEN_TIMEOUT, sh.sticky));
  }
  else
    return_to_status();
}

////////////////////////////////////////////
/////////// Menu Editing Actions ///////////
////////////////////////////////////////////

// All Edit Screens run the same way, but `draw_edit_screen` is implementation-specific
void MenuEditItemBase::edit_screen(strfunc_t strfunc, loadfunc_t loadfunc) {
  // Reset repeat_delay for Touch Buttons
  TERN_(HAS_TOUCH_BUTTONS, ui.repeat_delay = BUTTON_DELAY_EDIT);
  // Constrain ui.encoderPosition to 0 ... maxEditValue (calculated in encoder steps)
  if (int32_t(ui.encoderPosition) < 0) ui.encoderPosition = 0;
  if (int32_t(ui.encoderPosition) > maxEditValue) ui.encoderPosition = maxEditValue;
  // If drawing is flagged then redraw the (whole) edit screen
  if (ui.should_draw())
    draw_edit_screen(strfunc(ui.encoderPosition + minEditValue));
  // If there was a click or "live editing" and encoder moved...
  if (ui.lcd_clicked || (liveEdit && ui.should_draw())) {
    // Pass the editValue pointer to the loadfunc along with the encoder plus min
    if (editValue) loadfunc(editValue, ui.encoderPosition + minEditValue);
    // If a callbackFunc was set, call it for click or always for "live editing"
    if (callbackFunc && (liveEdit || ui.lcd_clicked)) (*callbackFunc)();
    // Use up the click to finish editing and go to the previous screen
    if (ui.use_click()) ui.goto_previous_screen();
  }
}

// Going to an edit screen sets up some persistent values first
void MenuEditItemBase::goto_edit_screen(
  FSTR_P const el,        // Edit label
  void * const ev,        // Edit value pointer
  const int32_t minv,     // Encoder minimum
  const int32_t maxv,     // Encoder maximum
  const uint16_t ep,      // Initial encoder value
  const screenFunc_t cs,  // MenuItem_type::draw_edit_screen => MenuEditItemBase::edit()
  const screenFunc_t cb,  // Callback after edit
  const bool le           // Flag to call cb() during editing
) {
  TERN_(HAS_TOUCH_BUTTONS, ui.on_edit_screen = true);
  ui.screen_changed = true;
  ui.push_current_screen();
  ui.refresh();
  editLabel = el;
  editValue = ev;
  minEditValue = minv;
  maxEditValue = maxv;
  ui.encoderPosition = ep;
  ui.currentScreen = cs;
  callbackFunc = cb;
  liveEdit = le;
}

////////////////////////////////////////////
///////////////// Menu Tree ////////////////
////////////////////////////////////////////

#include "../../MarlinCore.h"

/**
 * General function to go directly to a screen
 */
void MarlinUI::goto_screen(screenFunc_t screen, const uint16_t encoder/*=0*/, const uint8_t top/*=0*/, const uint8_t items/*=0*/) {
  if (currentScreen != screen) {
    thermalManager.set_menu_cold_override(false);

    TERN_(IS_DWIN_MARLINUI, did_first_redraw = false);

    TERN_(HAS_TOUCH_BUTTONS, repeat_delay = BUTTON_DELAY_MENU);

    TERN_(SET_PROGRESS_PERCENT, progress_reset());

    #if BOTH(DOUBLECLICK_FOR_Z_BABYSTEPPING, BABYSTEPPING)
      static millis_t doubleclick_expire_ms = 0;
      // Going to menu_main from status screen? Remember first click time.
      // Going back to status screen within a very short time? Go to Z babystepping.
      if (screen == menu_main) {
        if (on_status_screen())
          doubleclick_expire_ms = millis() + DOUBLECLICK_MAX_INTERVAL;
      }
      else if (screen == status_screen && currentScreen == menu_main && PENDING(millis(), doubleclick_expire_ms)) {
        if (BABYSTEP_ALLOWED())
          screen = TERN(BABYSTEP_ZPROBE_OFFSET, lcd_babystep_zoffset, lcd_babystep_z);
        else {
          #if ENABLED(MOVE_Z_WHEN_IDLE)
            ui.manual_move.menu_scale = MOVE_Z_IDLE_MULTIPLICATOR;
            screen = []{ lcd_move_axis(Z_AXIS); };
          #endif
        }
      }
    #endif

    currentScreen = screen;
    encoderPosition = encoder;
    encoderTopLine = top;
    screen_items = items;
    if (on_status_screen()) {
      defer_status_screen(false);
      clear_menu_history();
      TERN_(AUTO_BED_LEVELING_UBL, bedlevel.lcd_map_control = false);
    }

	clear_lcd();
	

    //flexible_clear_lcd(50,0,TFT_WIDTH-50, TFT_HEIGHT);
    // Re-initialize custom characters that may be re-used
    #if HAS_MARLINUI_HD44780
      if (TERN1(AUTO_BED_LEVELING_UBL, !bedlevel.lcd_map_control))
        set_custom_characters(on_status_screen() ? CHARSET_INFO : CHARSET_MENU);
    #endif

    refresh(LCDVIEW_CALL_REDRAW_NEXT);
    screen_changed = true;
    TERN_(HAS_MARLINUI_U8GLIB, drawing_screen = false);

    TERN_(HAS_MARLINUI_MENU, encoder_direction_normal());

    set_selection(false);
  }
}

////////////////////////////////////////////
///////////// Manual Movement //////////////
////////////////////////////////////////////

//
// Display a "synchronize" screen with a custom message until
// all moves are finished. Go back to calling screen when done.
//
void MarlinUI::synchronize(FSTR_P const fmsg/*=nullptr*/) {
  static FSTR_P sync_message = fmsg ?: GET_TEXT_F(MSG_MOVING);
  push_current_screen();
  goto_screen([]{
    //if (should_draw()) MenuItem_static::draw(LCD_HEIGHT >= 4, sync_message);
    if (should_draw()) MenuItem_static::draw(3, sync_message);
  });
  defer_status_screen();
  planner.synchronize(); // idle() is called until moves complete
  //goto_previous_screen_no_defer();
  //ui.return_to_status();
}

/**
 * Scrolling for menus and other line-based screens
 *
 *   encoderLine is the position based on the encoder
 *   encoderTopLine is the top menu line to display
 *   screen_items is the total number of items in the menu (after one call)
 */
void scroll_screen(const uint8_t limit, const bool is_menu) {
  ui.encoder_direction_menus();
  ENCODER_RATE_MULTIPLY(false);
  if (int32_t(ui.encoderPosition) < 0) ui.encoderPosition = 0;
  if (ui.first_page) {
    encoderLine = ui.encoderPosition / (ENCODER_STEPS_PER_MENU_ITEM);
    ui.screen_changed = false;
  }
  if (screen_items > 0 && encoderLine >= screen_items - limit) {
    encoderLine = _MAX(0, screen_items - limit);
    ui.encoderPosition = encoderLine * (ENCODER_STEPS_PER_MENU_ITEM);
  }
  if (is_menu) {
    NOMORE(encoderTopLine, encoderLine);
    if (encoderLine >= encoderTopLine + LCD_HEIGHT)
      encoderTopLine = encoderLine - LCD_HEIGHT + 1;
  }
  else
    encoderTopLine = encoderLine;
}

#if HAS_SOUND
  void MarlinUI::completion_feedback(const bool good/*=true*/) {
    TERN_(HAS_TOUCH_SLEEP, wakeup_screen()); // Wake up on rotary encoder click...
    if (good) OKAY_BUZZ(); else ERR_BUZZ();
  }
#endif

#if HAS_LINE_TO_Z

  void line_to_z(const_float_t z) {
    current_position.z = z;
    line_to_current_position(manual_feedrate_mm_s.z);
  }

#endif

#if ENABLED(BABYSTEP_ZPROBE_OFFSET)

  #include "../../feature/babystep.h"

  void lcd_babystep_zoffset() {
    if (ui.use_click()) return ui.goto_previous_screen_no_defer();
    ui.defer_status_screen();
    const bool do_probe = DISABLED(BABYSTEP_HOTEND_Z_OFFSET) || active_extruder == 0;
    if (ui.encoderPosition) {
      const int16_t babystep_increment = int16_t(ui.encoderPosition) * (BABYSTEP_SIZE_Z);
      ui.encoderPosition = 0;

      const float diff = planner.mm_per_step[Z_AXIS] * babystep_increment,
                  new_probe_offset = probe.offset.z + diff,
                  new_offs = TERN(BABYSTEP_HOTEND_Z_OFFSET
                    , do_probe ? new_probe_offset : hotend_offset[active_extruder].z - diff
                    , new_probe_offset
                  );
      if (WITHIN(new_offs, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) {

        babystep.add_steps(Z_AXIS, babystep_increment);

        if (do_probe)
          probe.offset.z = new_offs;
        else
          TERN(BABYSTEP_HOTEND_Z_OFFSET, hotend_offset[active_extruder].z = new_offs, NOOP);

        ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
      }
    }
    if (ui.should_draw()) {
      if (do_probe) {
        MenuEditItemBase::draw_edit_screen(GET_TEXT_F(MSG_ZPROBE_ZOFFSET), BABYSTEP_TO_STR(probe.offset.z));
        TERN_(BABYSTEP_ZPROBE_GFX_OVERLAY, ui.zoffset_overlay(probe.offset.z));
      }
      else {
        #if ENABLED(BABYSTEP_HOTEND_Z_OFFSET)
          MenuEditItemBase::draw_edit_screen(GET_TEXT_F(MSG_HOTEND_OFFSET_Z), ftostr54sign(hotend_offset[active_extruder].z));
        #endif
      }
    }
  }

#endif // BABYSTEP_ZPROBE_OFFSET

void _lcd_draw_homing() {
  if (ui.should_draw()) {
    constexpr uint8_t line = (LCD_HEIGHT - 1) / 2;
    MenuItem_static::draw(line, GET_TEXT_F(MSG_LEVEL_BED_HOMING));
  }
}

#if ENABLED(LCD_BED_LEVELING) || (HAS_LEVELING && DISABLED(SLIM_LCD_MENUS))
  void _lcd_toggle_bed_leveling() { set_bed_leveling_enabled(!planner.leveling_active); }
#endif

//
// Selection screen presents a prompt and two options
//
bool MarlinUI::selection; // = false
bool MarlinUI::update_selection() {
  encoder_direction_select();
  if (encoderPosition) {
    selection = int16_t(encoderPosition) > 0;
    encoderPosition = 0;
  }
  return selection;
}

void MenuItem_confirm::select_screen(
  FSTR_P const yes, FSTR_P const no,
  selectFunc_t yesFunc, selectFunc_t noFunc,
  FSTR_P const pref, const char * const string/*=nullptr*/, FSTR_P const suff/*=nullptr*/
) {
  ui.defer_status_screen();
  const bool ui_selection = !yes ? false : !no || ui.update_selection(),
             got_click = ui.use_click();
  if (got_click || ui.should_draw()) {
    ui.last_confirm_windown_enabled = ui.confirm_windown_enabled;
    ui.confirm_windown_enabled = true;
    draw_select_screen(yes, no, ui_selection, pref, string, suff);
    if (got_click) {
	    ui.confirm_windown_enabled = false;
      selectFunc_t callFunc = !ui_selection ? yesFunc : noFunc; 
      if (callFunc)
      {
          callFunc(); 
      }
      else 
      {
        ui.goto_previous_screen();
        ui.previous_callbackFunc();
      }
    }
    
  }
}

uint8_t MarlinUI::multi_selection; // = false
uint8_t MarlinUI::update_multi_selection(uint8_t num) {
    //encoder_direction_select();
    if(int16_t(encoderPosition)>= 1){
      multi_selection++;
      if(multi_selection >=num) multi_selection = num;
        encoderPosition = 0;
    }
    else if(int16_t(encoderPosition)<= -1){
      if(multi_selection == 0){
        multi_selection = 0;

      }
      else multi_selection--;
      encoderPosition = 0;

    }
	return multi_selection;
}

float MarlinUI::zoffset;
void draw_zoffset_select_screen(uint16_t back_color,uint16_t upcolor, uint16_t downcolor,float zoffset,uint16_t selection)
{
	//titol
	if(selection){
	ui.fresh_flag = false;
	tft.canvas(124,26,100, 20);
  	tft.set_background(COLOR_BACKGROUND);
	tft_string.set(GET_TEXT_F(MSG_UBL_Z_OFFSET));
	tft.add_text(0,0,COLOR_WHITE,tft_string);
		
	//Tip
	tft.canvas(34,74,136, 136);
	tft.set_background(COLOR_BACKGROUND);
	tft.add_image(0, 0,imgZoffsetTip,COLOR_GREY);

	tft.add_image(0, 90,imgZoffsetTip1,0x07FE);
	
	}

	//back
	tft.canvas(22,22,40, 40);
	tft.set_background(COLOR_BACKGROUND);
	tft.add_image(0,0,imgBack,back_color);

	//up
	tft.canvas(224,63,62, 56);
	tft.set_background(COLOR_BACKGROUND);
	tft.add_image(0,0,imgUp,upcolor);

	//z_offset
	tft.canvas(216,129,78, 32);
	tft.set_background(COLOR_BACKGROUND);
	tft_string.set(ftostr42_52(zoffset));
	tft_string.add("mm");
	tft.add_text(tft_string.center(78),tft_string.center(32),COLOR_WHITE,tft_string);

	//down
	tft.canvas(224,170,62, 56);
	tft.set_background(COLOR_BACKGROUND);
	tft.add_image(0,0,imgDown,downcolor);

	
}
void tft_babystep_zoffset()
{
	 
  uint8_t selection = ui.update_multi_selection(3-1);
  const bool got_click = ui.use_click();
  uint16_t back_color,up_color,down_color;
  static int16_t direction;

  ui.defer_status_screen();

   if (got_click || ui.should_draw()) 
   {
	 	//draw zoffset page
    switch(selection)
		{
		  case 0:
          back_color = COLOR_WHITE;
          up_color = down_color =COLOR_GREY;
			break;
				case 1://UP
          up_color = COLOR_WHITE;
          back_color = down_color =COLOR_GREY;
				direction = 1;
					break;
				case 2://DOWN
          down_color = COLOR_WHITE;
          back_color = up_color =COLOR_GREY;
				direction = -1;
					break;

		  }    
	  if(got_click){
      if(selection)
      {
      		 
      	float zoffset = ui.getzoffset(); //get probe.offset.z
		    const int16_t babystep_increment  = direction * BABYSTEP_SIZE_Z;//Get the number of pulses  20 or -20
		    float diff = planner.mm_per_step[Z_AXIS] * babystep_increment; //-0.05 or +0.05
		    float new_probe_offset = zoffset + diff;//get new probe.offset.z
		

        if(new_probe_offset < Z_PROBE_OFFSET_RANGE_MIN) new_probe_offset = Z_PROBE_OFFSET_RANGE_MIN;//-5.00007
        if(new_probe_offset > Z_PROBE_OFFSET_RANGE_MAX) new_probe_offset = Z_PROBE_OFFSET_RANGE_MAX;//=5.00007
		    if(Z_PROBE_OFFSET_RANGE_MIN <= new_probe_offset && new_probe_offset <= Z_PROBE_OFFSET_RANGE_MAX)
		    {
			
			    babystep.add_steps(Z_AXIS, babystep_increment);
			    ui.setzoffset(new_probe_offset);
		    }
		
      }
      else
      {
      	if(planner.leveling_active){
      		autoProbe.calibration_positon.z += ui.getzoffset() - ui.temp_probe_zoffset;
			    autoProbe.need_save_data  = true;
      	}
		
        ui.goto_previous_screen();
		    ui.clear_all = false;
        if(ui.temp_probe_zoffset!= ui.getzoffset()) 		  
           queue.inject("M500");
        return;
      }

	  }
    	draw_zoffset_select_screen(back_color,up_color,down_color,ui.getzoffset(),ui.fresh_flag);
	 }
}

void tft_set_speed(){

 static uint16_t change = 1;
 static bool fresh_flag;
 static int16_t temp_feedrate_percentage = 100;

 if (ui.use_click()){  
   fresh_flag = false;
   ui.clear_all = false;
   feedrate_percentage = temp_feedrate_percentage;
   return  ui.goto_previous_screen_no_defer();
 }

 //ui.encoder_direction_select();
 if(!fresh_flag)
 {
   fresh_flag = true;
   temp_feedrate_percentage = feedrate_percentage;
   LIMIT(temp_feedrate_percentage, 80, 120);   //limit 80~120
	 change = (temp_feedrate_percentage -80) /20;//confirm 
 }
 
 if(ui.encoderPosition){

    temp_feedrate_percentage = temp_feedrate_percentage + 20 * ui.encoderPosition;
	  ui.encoderPosition = 0;
	  LIMIT(temp_feedrate_percentage, 80, 120);   //limit 80~120
	  change = (temp_feedrate_percentage -80) /20;//confirm 

	  //ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
  }
 
 if(ui.should_draw())
 {
	//titol
	tft.canvas(0,20,TFT_WIDTH, 32);
  tft.set_background(COLOR_BACKGROUND);
	tft_string.set(GET_TEXT_F(MSG_SPEED));
	tft.add_text(tft_string.center(TFT_WIDTH),0,COLOR_WHITE,tft_string);
  	#define SPEED_BUTTON_WIDTH  50
  	#define SPEED_BUTTON_HEIGHT 32

	tft.canvas(30,83,SPEED_BUTTON_WIDTH, SPEED_BUTTON_HEIGHT);
  tft.set_background(COLOR_BACKGROUND);
	tft_string.set("80%");
	tft.add_text(tft_string.center(SPEED_BUTTON_WIDTH),tft_string.center(SPEED_BUTTON_HEIGHT),COLOR_WHITE,tft_string);

	tft.canvas(134,83,SPEED_BUTTON_WIDTH, SPEED_BUTTON_HEIGHT);
  tft.set_background(COLOR_BACKGROUND);
	tft_string.set("100%");
	tft.add_text(tft_string.center(SPEED_BUTTON_WIDTH),tft_string.center(SPEED_BUTTON_HEIGHT),COLOR_WHITE,tft_string);

	tft.canvas(244,83,SPEED_BUTTON_WIDTH, SPEED_BUTTON_HEIGHT);
  tft.set_background(COLOR_BACKGROUND);
	tft_string.set("120%");
	tft.add_text(tft_string.center(SPEED_BUTTON_WIDTH),tft_string.center(SPEED_BUTTON_HEIGHT),COLOR_WHITE,tft_string);
	
	//Left image tip
	tft.canvas(10,131,36, 36);
  	tft.set_background(COLOR_BACKGROUND);
	tft.add_image(0,0, imgLeftRound, COLOR_WHITE);
	
	//right image tip
	tft.canvas(274,131,36, 36);
  tft.set_background(COLOR_BACKGROUND);
	tft.add_image(0,0, imgRightRound, COLOR_WHITE);

	#define SLIDER_LENGTH 208
	tft.canvas(56,141,SLIDER_LENGTH, 16);
  tft.set_background(COLOR_SLIDER_INACTIVE);
	tft.add_rectangle(0, 0, SLIDER_LENGTH, 16, COLOR_SLIDER_INACTIVE);
	tft.add_bar(1, 1, ((SLIDER_LENGTH-2) *change)/2, 14, COLOR_BLUE);
  }

}
void draw_edit_temp_screen(FSTR_P const fstr,uint16_t maxlimit,uint16_t tempdata){
  	//titol
	tft.canvas(0,20,TFT_WIDTH, 32);
  tft.set_background(COLOR_BACKGROUND);
	tft_string.set(fstr);
	tft.add_text(tft_string.center(TFT_WIDTH),0,COLOR_WHITE,tft_string);


	tft.canvas(131,83,58, 31);
  tft.set_background(COLOR_BACKGROUND);
	tft_string.set(i16tostr3rj(tempdata));
  	tft_string.add(GET_TEXT_F(MSG_TEMP_UINT));
	tft.add_text(tft_string.center(58),tft_string.center(31),COLOR_WHITE,tft_string);

	//Left image tip
	tft.canvas(10,131,36, 36);
 	tft.set_background(COLOR_BACKGROUND);
	tft.add_image(0,0, imgLeftRound, COLOR_WHITE);
	
	//right image tip
	tft.canvas(274,131,36, 36);
  tft.set_background(COLOR_BACKGROUND);
	tft.add_image(0,0, imgRightRound, COLOR_WHITE);

	#define SLIDER_LENGTH 208
	tft.canvas(56,141,SLIDER_LENGTH, 16);
  	tft.set_background(COLOR_SLIDER_INACTIVE);
	tft.add_rectangle(0, 0, SLIDER_LENGTH, 16, COLOR_SLIDER_INACTIVE);
	tft.add_bar(1, 1, ((SLIDER_LENGTH-2) *tempdata)/maxlimit, 14, COLOR_BLUE);
}

void draw_edit_move_axis_screen(FSTR_P const fstr, int8_t axis,const char * const value/*=nullptr*/,uint16_t pos){
	
  if(ui.fresh_flag){
    ui.fresh_flag = false;
    tft.canvas(0, 0, 50, 240); 
	  tft.set_background(COLOR_BACKGROUND);
	  //titol
	  tft.canvas(0,20,TFT_WIDTH, 32);
    tft.set_background(COLOR_BACKGROUND);
	  tft_string.set(fstr,axis);
	  tft.add_text(tft_string.center(TFT_WIDTH),0,COLOR_WHITE,tft_string);
    	//Left image tip
	  tft.canvas(10,131,36, 36);
 	  tft.set_background(COLOR_BACKGROUND);
	  tft.add_image(0,0, imgLeftRound, COLOR_WHITE);
	
	  //right image tip
	  tft.canvas(274,131,36, 36);
  	tft.set_background(COLOR_BACKGROUND);
	  tft.add_image(0,0, imgRightRound, COLOR_WHITE);

  }

	tft.canvas(110,83,100, 31);
  tft.set_background(COLOR_BACKGROUND);
	tft_string.set(value);
  tft_string.add('m');
  tft_string.add('m');
	tft.add_text(tft_string.center(100),tft_string.center(31),COLOR_WHITE,tft_string);


  #define SLIDER_LENGTH 208
    LIMIT(pos,0,Y_BED_SIZE);
	  tft.canvas(56,141,SLIDER_LENGTH, 16);
    tft.set_background(COLOR_SLIDER_INACTIVE);
	  tft.add_rectangle(0, 0, SLIDER_LENGTH, 16, COLOR_SLIDER_INACTIVE);
	  tft.add_bar(1, 1, ((SLIDER_LENGTH-2) *(int16_t)pos)/Y_BED_SIZE, 14, COLOR_BLUE);


    // tft.add_bar(0, 0, 4, 16,COLOR_SLIDER_INACTIVE);
    // tft.add_bar(1, 15, pos, 16, COLOR_SLIDER);
    // tft.add_bar(pos + 1, 15, SLIDER_LENGTH - 2 - pos, 16, COLOR_SLIDER_INACTIVE);
    // tft.add_bar(SLIDER_LENGTH - 1, 15, 4, 16, COLOR_SLIDER_INACTIVE);
}

void tft_setTargetHotend(){

 static int16_t target_temp_data;
 static bool fresh_flag;

 	ui.defer_status_screen();
 	if (ui.use_click())	 {  
		ui.enable_encoder_multiplier(false);
		fresh_flag = false;
		ui.clear_all = false;
		thermalManager.temp_hotend[0].target = target_temp_data;
//    if(printingIsActive() || target_temp_data == 0){ 
//      	thermalManager.temp_hotend[0].target = target_temp_data;
//    }
//    else{
//        ui.nozzle_heated_state = true;
//        ui.have_heated_task = true;
//        ui.nozzle_target = target_temp_data;
//
//    }
   	return  ui.goto_previous_screen_no_defer();
 	}
	
  if(!fresh_flag)
  {
    ui.enable_encoder_multiplier(true);
    fresh_flag = true;
    target_temp_data = thermalManager.temp_hotend[0].target;

  }
	  
	 if (ui.encoderPosition) {
       target_temp_data +=ui.encoderPosition;
       ui.encoderPosition = 0;
	   if(printingIsActive())
       	LIMIT(target_temp_data, 170, thermalManager.hotend_max_target(0));
	   else
	   	  LIMIT(target_temp_data, 0, thermalManager.hotend_max_target(0));
       //ui.refresh(LCDVIEW_CALL_REDRAW_NEXT); 
    }
 
 	if(ui.should_draw())
 	{
		draw_edit_temp_screen(GET_TEXT_F(MSG_UBL_HOTEND_TEMP_CUSTOM),thermalManager.hotend_max_target(0),target_temp_data);
  	}

}

void tft_setTargetBed(){

 static int16_t target_temp_data;
 static bool fresh_flag;

 	ui.defer_status_screen();
 	if (ui.use_click())	 {  
		ui.enable_encoder_multiplier(false);
		fresh_flag = false;
		ui.clear_all = false;
		thermalManager.temp_bed.target = target_temp_data;
//    if(printingIsActive() || target_temp_data == 0){ 
//      	thermalManager.temp_bed.target = target_temp_data;
//    }
//    else{
//        ui.bed_heated_state = true;
//        ui.have_heated_task = true;
//        ui.bed_target = target_temp_data;
//    }
   	return  ui.goto_previous_screen_no_defer();
 	}
	
  if(!fresh_flag)
  {
    ui.enable_encoder_multiplier(true);
    fresh_flag = true;
    target_temp_data = thermalManager.temp_bed.target;
  }
	  
	 if (ui.encoderPosition) {
       target_temp_data +=ui.encoderPosition;
       ui.encoderPosition = 0;
       LIMIT(target_temp_data, 0, BED_MAX_TARGET);
       //ui.refresh(LCDVIEW_CALL_REDRAW_NEXT); 
    }
 
 	if(ui.should_draw())
 	{
		draw_edit_temp_screen(GET_TEXT_F(MSG_UBL_BED_TEMP_CUSTOM),BED_MAX_TARGET,target_temp_data);
  	}

}

void tft_stop_print()
{
	 ui.defer_status_screen();
	const bool ui_selection = ui.update_selection(), got_click = ui.use_click();
	if (got_click || ui.should_draw()) {
	  ui.last_confirm_windown_enabled = ui.confirm_windown_enabled;
      ui.confirm_windown_enabled = true;
	  MenuItem_confirm::draw_select_screen(
      GET_TEXT_F(MSG_BUTTON_STOP), GET_TEXT_F(MSG_BACK), 
      ui_selection,
      GET_TEXT_F(MSG_STOP_PRINT), (const char *)nullptr,nullptr
      );
	  if (got_click) {
	  	 ui.confirm_windown_enabled = false;
		selectFunc_t callFunc = !ui_selection ? ui.abort_print :  ui.return_to_status;
		if (callFunc) { 
			callFunc();
		}
		else ui.goto_previous_screen();
	  }
	  //ui.Cancel_callbackFunc();
	}

}

void tft_pause_print()
{
	ui.defer_status_screen();
	const bool ui_selection = ui.update_selection(), got_click = ui.use_click();
	if (got_click || ui.should_draw()) {
	  ui.last_confirm_windown_enabled = ui.confirm_windown_enabled;
	  ui.confirm_windown_enabled = true;
	  MenuItem_confirm::draw_select_screen(
      GET_TEXT_F(MSG_BUTTON_STOP), GET_TEXT_F(MSG_BACK), 
      ui_selection, 
      GET_TEXT_F(MSG_PAUSE_PRINT), (const char *)nullptr,nullptr
      );
	  
    if (got_click) {
	  	ui.confirm_windown_enabled = false;
		  selectFunc_t callFunc = !ui_selection ? ui.pause_print :  ui.return_to_status;
		  if (callFunc) {
		  	callFunc();
			//ui.goto_previous_screen();
		  } 
		  else 
		  	ui.goto_previous_screen();
	  }
	  
	}

}

LCDLeveingState MarlinUI::lcdLeveingstate = LEVEING_NONE;

void lcd_level_top_windown()
{
  uint16_t preheating_color,wipe_nozzle_color,probe_color,confirm_color;
  uint16_t preheating_Fontcolor,wipe_nozzle_Fontcolor,probe_Fontcolor;
  char nozzle_buf[16];
  char bed_buf[16];
  sprintf(nozzle_buf,"E: %u/%u",(uint16_t)thermalManager.wholeDegHotend(0), (uint16_t)thermalManager.degTargetHotend(0));
  sprintf(bed_buf,"B: %u/%u",(uint16_t)thermalManager.wholeDegBed(), (uint16_t)thermalManager.degTargetBed());


  if(ui.lcdLeveingstate == LEVEING_DONE)
  {
	  if(ui.use_click()) {		
		  ui.return_to_status();
		  ui.clear_all = false;
		  ui.lcdLeveingstate = LEVEING_NONE;
		  return;
	  }
  }
  if (ui.should_draw()) {
   if(ui.lcdLeveingstate == LEVEING_WIPE_NOZZLE)
   {
      wipe_nozzle_color = probe_color = confirm_color = COLOR_GREY;
      preheating_color = COLOR_GREEN;
      preheating_Fontcolor = wipe_nozzle_Fontcolor = COLOR_WHITE;
      probe_Fontcolor = COLOR_GREY;
   }
   else if(ui.lcdLeveingstate == LEVEING_PROBE)
   {
	    probe_color = confirm_color = COLOR_GREY;
      wipe_nozzle_color = preheating_color = COLOR_GREEN;
      preheating_Fontcolor = wipe_nozzle_Fontcolor = probe_Fontcolor = COLOR_WHITE;
   }
   else if(ui.lcdLeveingstate == LEVEING_DONE)
   {
     wipe_nozzle_color = preheating_color = probe_color  = COLOR_GREEN;
	 confirm_color = COLOR_BLUE;
	 preheating_Fontcolor = wipe_nozzle_Fontcolor = probe_Fontcolor = COLOR_WHITE;
	 
   }
   else
   {
     wipe_nozzle_color =  preheating_color = probe_color = confirm_color = COLOR_GREY;
     wipe_nozzle_Fontcolor = probe_Fontcolor = COLOR_GREY;
     preheating_Fontcolor = COLOR_WHITE;
   }		
	  //nozzle temp
	  tft.canvas(20, 10, 100, 32);
	  tft.set_background(COLOR_BACKGROUND);
	  tft_string.set(nozzle_buf);
	  tft_string.trim();
	  tft.add_text(0,0,COLOR_WHITE,tft_string);

	  //bed temp
	  tft.canvas(173, 10, 100, 32);
	  tft.set_background(COLOR_BACKGROUND);
	  tft_string.set(bed_buf);
	  tft_string.trim();
	  tft.add_text(0,0,COLOR_WHITE,tft_string);

	  //preheating
	  tft.canvas(20, 52, 106, 31);
	  tft.set_background(COLOR_BACKGROUND);
	  tft_string.set(GET_TEXT(MSG_LEVEING_PREHEATING));
	  tft_string.trim();
	  tft.add_text(0,5,preheating_Fontcolor,tft_string);

	  //Wipe nozzle
	  tft.canvas(20, 92, 130, 31);
	  tft.set_background(COLOR_BACKGROUND);
	  tft_string.set(GET_TEXT(MSG_LEVEING_WIPE));
	  tft_string.trim();
	  tft.add_text(0,5,wipe_nozzle_Fontcolor,tft_string);

	  //probing
	  tft.canvas(20, 132, 106, 31);
	  tft.set_background(COLOR_BACKGROUND);
	  tft_string.set(GET_TEXT(MSG_LEVEING_PROBE));
	  tft_string.trim();
	  tft.add_text(0,5,probe_Fontcolor,tft_string);
    
    tft.canvas(276, 57, 24, 24);
	  tft.set_background(COLOR_BACKGROUND);
    tft.add_image(0,0,imgOK,preheating_color);

    tft.canvas(276, 97, 24, 24);
	  tft.set_background(COLOR_BACKGROUND);
    tft.add_image(0,0,imgOK,wipe_nozzle_color);

	  tft.canvas(276, 137, 24, 24);
	  tft.set_background(COLOR_BACKGROUND);
    tft.add_image(0,0,imgOK,probe_color);

    tft.canvas(105, 180, 110, 44);
	  tft.set_background(COLOR_BACKGROUND);
    tft.add_image(0,0,imgConfirm,confirm_color);
  }
	ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
}
void menu_about(){

	ui.defer_status_screen();
	if (ui.should_draw()){

			tft.canvas(0, 0, 50, 240);		
			tft.set_background(COLOR_BACKGROUND);
		
			tft.canvas(0, 40, TFT_WIDTH, 30);		
			tft.set_background(COLOR_BACKGROUND);
			tft_string.set(DEVICE_NAME);
			tft_string.trim();
			tft.add_text(tft_string.center(TFT_WIDTH),5,COLOR_MENU_TEXT,tft_string);

			tft.canvas(0, 80, TFT_WIDTH, 30);
			tft.set_background(COLOR_BACKGROUND);
			tft_string.set(FIRMWARE_VER);
			tft_string.trim();
			tft.add_text(tft_string.center(TFT_WIDTH),5,COLOR_MENU_TEXT,tft_string);
			
			tft.canvas(0, 120, TFT_WIDTH, 30);
			tft.set_background(COLOR_BACKGROUND);
			tft_string.set(BUILD_VOLUME);
			tft_string.trim();
			tft.add_text(tft_string.center(TFT_WIDTH),5,COLOR_MENU_TEXT,tft_string);

			tft.canvas(0, 160, TFT_WIDTH, 30);
			tft.set_background(COLOR_BACKGROUND);
			tft_string.set(TECH_SUPPORT);
			tft_string.trim();
			tft.add_text(tft_string.center(TFT_WIDTH),5,COLOR_MENU_TEXT,tft_string);
			//ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
	}
	if(ui.use_click()) {
	  ui.goto_previous_screen();
      ui.previous_callbackFunc();
	}

}

void sd_card_removed(){

	static millis_t flash_time = 0;
	if(millis() < (flash_time + 1000) ) {
    SERIAL_ECHOLNPGM("sd_card_removed");

  }
  
  SERIAL_ECHOLNPGM("sd_card_removed");
  if(ui.use_click()){
	  ui.start_print_status = false;
    return ui.return_to_status();
  }

  if(ui.should_draw()){

    ui.flexible_clear_lcd(0,0,50,240);
	  tft.canvas(18, 52, 284, 32);
	  tft.set_background(COLOR_BACKGROUND);
	  tft_string.set(GET_TEXT_F(MSG_TF_CARD_REMOVED));

    tft.add_text(tft_string.center(284) ,tft_string.center(32) , COLOR_WHITE, tft_string);

   
    tft.canvas(105, 136, 110, 104);
	  tft.set_background(COLOR_BACKGROUND);
    tft.add_image(0, 0, imgConfirm, COLOR_BLUE);     
  }  
}

void calibration_tip(){

  if(ui.should_draw()){

    ui.flexible_clear_lcd(0,0,50,240);
	  tft.canvas(0, 110, 320, 20);
	  tft.set_background(COLOR_BACKGROUND);
	  //tft_string.set(GET_TEXT_F(MSG_TF_CARD_REMOVED));
    tft_string.set("Calibration...");
    tft.add_text(tft_string.center(320) ,tft_string.center(20) , COLOR_WHITE, tft_string);     
  }  
}

bool filament_staring = false;
uint8_t filament_cmd = FILA_NO_ACT;
bool unloaOrloaddfilamentstate = false;
void unload_load_filament(){

 	if(!filament_staring) return;
	
	static millis_t return_ms = 0;
	#define RETURN_TIMEOUT_MS  25000


	if(filament_cmd == FILA_IN && unloaOrloaddfilamentstate == true){
		  return_ms =  millis() + RETURN_TIMEOUT_MS;
    	unloaOrloaddfilamentstate = false;
		  queue.inject_P("M83\nG1 E100 F300\nM82");
	}
	else if(filament_cmd == FILA_OUT && unloaOrloaddfilamentstate == true){
		  return_ms =  millis() + RETURN_TIMEOUT_MS;
		  unloaOrloaddfilamentstate = false;
		  queue.inject_P("M83\n G1 E30 F300\n G1 E-70 F400\nM82");
	}

	if (ELAPSED(millis(), return_ms))
	{
		    filament_cmd = FILA_NO_ACT;
			filament_staring = false;
			ui.clear_all = false;
			planner.quick_stop();	
			ui.goto_previous_screen_no_defer();
			ui.previous_callbackFunc();
	}
	
}

 void draw_unload_load_filament(){

	 filament_staring = true;
    if (ui.use_click()) 
    {
    		ui.clear_all = false;
    		filament_cmd = FILA_NO_ACT;
			filament_staring = false;
			planner.quick_stop();	
			ui.goto_previous_screen_no_defer();
			ui.previous_callbackFunc();
			return;
    }

	if (ui.should_draw()){ 
		
			tft.canvas(0, 68, TFT_WIDTH, 30);	
			tft.set_background(COLOR_BACKGROUND);
			if(filament_cmd == FILA_IN)       tft_string.set(GET_TEXT_F(MSG_FILAMENTLOADING));
			else if(filament_cmd == FILA_OUT) tft_string.set(GET_TEXT_F(MSG_FILAMENTUNLOADING));
			tft_string.trim();
			tft.add_text(tft_string.center(TFT_WIDTH),5,COLOR_WHITE,tft_string);

      tft.canvas(80, 136, 160, 44);	
			tft.set_background(COLOR_BACKGROUND);
      tft.add_image(0,0,imgBtn44Round,COLOR_GREY);
		 	tft_string.set(GET_TEXT_F(MSG_FILAMENT_STOP));
			tft_string.trim();
			tft.add_text(tft_string.center(160),12,COLOR_WHITE,tft_string);

	}
	ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
}

 void preheat_to_move_E()
 {
 	static uint32_t refresh_ms;
	int16_t currentTemperature =  thermalManager.temp_hotend[0].celsius;
	int16_t targetTemperature =   thermalManager.temp_hotend[0].target;
 //   START_SCREEN();
 	  
	  ui.defer_status_screen();
    if (ui.should_draw())
	  {
		  if(currentTemperature<205)
			{
          char str_buf[16];
          sprintf(str_buf,"E: %u/%u",(uint16_t)currentTemperature, (uint16_t)targetTemperature);
  


			   tft.canvas(0, 72, 320, 32);
			   tft.set_background(COLOR_BACKGROUND);
			   tft_string.set(GET_TEXT_F(MSG_HEATING_NOZZLE));
			   tft_string.trim();
			   tft.add_text(tft_string.center(320),tft_string.center(32),COLOR_WHITE,tft_string);	  
			   
			   
			   tft.canvas(0, 104, 320, 32);
			   tft.set_background(COLOR_BACKGROUND);
			   tft_string.set(GET_TEXT_F(MSG_PLEASE_WAIT));
			   tft_string.trim();
			   tft.add_text(tft_string.center(320),tft_string.center(32),COLOR_WHITE,tft_string);	
 
				
			   tft.canvas(0, 136, 320, 32);
			   tft.set_background(COLOR_BACKGROUND); 
         tft_string.set(str_buf);
         tft_string.trim();
         tft.add_text(tft_string.center(320) ,tft_string.center(32) , COLOR_WHITE, tft_string);

		   }	 
		   else if(currentTemperature >= 205)
		   {
			  ui.goto_screen(draw_unload_load_filament);
		   }
	 }
 	ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
 // 	END_SCREEN(); 
   
 
 }

void runout_sensor()
{
    if(ui.use_click()){
      return ui.return_to_status();
    }

    if(ui.should_draw){
	  //ui.flexible_clear_lcd(0,0,50,240);
      tft.canvas(18, 81, 284, 32);
	    tft.set_background(COLOR_BACKGROUND);
      tft_string.set(GET_TEXT_F(MSG_RUNOUT_SENSOR));
      tft.add_text(tft_string.center(284) ,5 , COLOR_WHITE, tft_string);
      
      tft.canvas(105, 136, 110, 44);
	    tft.set_background(COLOR_BACKGROUND);
      tft.add_image(0, 0, imgConfirm, COLOR_BLUE);     
    }


}

void printinf_finish()
{


  char buffer[22];
  duration_t(print_job_timer.duration()).toString(buffer);
  if(ui.use_click()){
  	ui.confirm_windown_enabled = false; 
	  ui.start_print_status = ui.print_task_done = false;
    return ui.return_to_status();
  }

  if(ui.should_draw()){
    
      ui.flexible_clear_lcd(0,0,50,240);
	    tft.canvas(18, 38, 284, 32);
	    tft.set_background(COLOR_BACKGROUND);
	    tft_string.set(GET_TEXT_F(MSG_PRINT_FINISH));
      tft.add_text(tft_string.center(284) ,5 , COLOR_WHITE, tft_string);

      tft.canvas(18, 81, 284, 20);
		  tft.set_background(COLOR_BACKGROUND);
      tft_string.set(buffer);
      tft.add_text(tft_string.center(284) ,tft_string.center(20) , COLOR_WHITE, tft_string);
      
      tft.canvas(105, 136, 110, 44);
		  tft.set_background(COLOR_BACKGROUND);
      tft.add_image(0, 0, imgConfirm, COLOR_BLUE);     
  }
}
void Probing_Failed()
{

  if(ui.use_click()){
  	//autoProbe.load_config();
    //soft_endstop._enabled = true;
    ui.clear_all = false;
    autoProbe.LeveingFailSattue = false;
    ui.flexible_clear_lcd(0,0,50,240);
    ui.goto_previous_screen();
    ui.previous_callbackFunc();
    return;
  }

  if(ui.should_draw()){
      //ui.flexible_clear_lcd(0,0,50,240);
      tft.canvas(18, 81, 284, 32);
	    tft.set_background(COLOR_BACKGROUND);
      if(autoProbe.LeveingFailSattue)
        tft_string.set(GET_TEXT_F(MSG_MODULE_PROBE_FAILD));
      else 
        tft_string.set(GET_TEXT_F(MSG_PROBE_FAILD));

      tft.add_text(tft_string.center(284),5, COLOR_WHITE, tft_string);
      
      tft.canvas(105, 136, 110, 44);
		  tft.set_background(COLOR_BACKGROUND);
      tft.add_image(0, 0, imgConfirm, COLOR_BLUE);  
  }

}

void MarlinUI::StatusChange(const char * const msg)
{
  
    //SERIAL_ECHOLNPGM("StatusChange() ", msg);
    alert_level = 0;//No abnormal pop-ups when appeared Media init faild,need reset alert_level to 0;
    if (strncmp_P(msg, "Probing Failed",strlen_P("Probing Failed")) == 0){
        lcdLeveingstate = LEVEING_NONE;
        soft_endstop._enabled = false;
        calibration_state = false;
		    queue.inject_P(PSTR("G1 Z20 F500"));
        ui.clear_all = true;
        goto_screen(Probing_Failed);
    }
    else if(strncmp_P(msg, "CalibrationStart",strlen_P("CalibrationStart"))== 0){
        ui.push_current_screen();
        ui.clear_all = true;
        ui.goto_screen([]{
          if (should_draw()) MenuItem_static::draw(3, GET_TEXT_F(MSG_POSITION_CALIBRATION),1,"...");
        });
    }
    else if(strncmp_P(msg, "CalibrationDone",strlen_P("CalibrationDone"))== 0){
      calibration_state = false;//not display homing tip
      ui.clear_all = false;
      ui.goto_previous_screen();
      ui.previous_callbackFunc();
    }
    else if(strncmp_P(msg, "HomingStart",strlen_P("HomingStart"))== 0){
      if(calibration_state ||  lcdLeveingstate || ui.start_print_status )return; 
      ui.push_current_screen();
      ui.clear_all = true;
      ui.goto_screen([]{
          if (should_draw()) MenuItem_static::draw(3, GET_TEXT_F(MSG_HOMING));
        });

    }
    else if(strncmp_P(msg, "HomingDone",strlen_P("HomingStart"))== 0){
      if(calibration_state ||  lcdLeveingstate || ui.start_print_status )return;
      ui.clear_all = false;
      ui.goto_previous_screen();
      ui.previous_callbackFunc();
    }

}

void check_endstops()
{

       if(!thermalManager.allow_cold_extrude) return;

       bool x_state = READ(X_MIN_PIN)!= X_MIN_ENDSTOP_INVERTING;
       bool y_state = READ(Y_MIN_PIN)!= Y_MIN_ENDSTOP_INVERTING;
       bool z_state = READ(Z_MIN_PIN)!= Z_MIN_ENDSTOP_INVERTING;
       bool m_state = READ(Z_MAX_PIN)!= Z_MAX_ENDSTOP_INVERTING;
	   bool e_state = READ(FIL_RUNOUT1_PIN)!= FIL_RUNOUT1_STATE;

       static uint8_t cur_endstops_state = e_state<<4 | x_state<<3 | y_state<<2 | z_state <<1 | m_state;
       static uint8_t prv_endstops_state = cur_endstops_state;

       cur_endstops_state = e_state<<4 | x_state<<3 | y_state<<2 | z_state <<1 | m_state;

       if(cur_endstops_state == prv_endstops_state)  return ;

       prv_endstops_state = cur_endstops_state;

      char cmd[50];

      sprintf_P(cmd, PSTR("endstop_state: x%d  y%d z%d m%d e%d"), x_state,y_state,z_state,m_state,e_state);  
 
	   tft.canvas(50, 70, 270, 20);
	   tft.set_background(COLOR_BACKGROUND);
       tft_string.set(cmd);
       tft.add_text(1 ,5 , COLOR_RED, tft_string);
 


}
bool MarlinUI::nozzle_heated_state = false;
bool MarlinUI::bed_heated_state = false;
bool MarlinUI::nozzle_beform_bed = false;
bool MarlinUI::preheat_state = false;
bool MarlinUI::have_heated_task = false;
int16_t MarlinUI::nozzle_target = 0;
int16_t MarlinUI::bed_target = 0;

void heating_handle(){

  if(!ui.preheat_state) return;

  int16_t nozzle_target,nozzle_cur,bed_target,bed_cur;
  bool nozzle_heating,nozzle_heatedComplete,bed_heating,bed_heatedComplete;

	nozzle_cur = thermalManager.degHotend(0);         //nozzle current temperature
	nozzle_target = thermalManager.degTargetHotend(0);// nozzle target temperature
	bed_cur = thermalManager.degBed();                //bed current temperature
	bed_target = thermalManager.degTargetBed();       //bed target temperature

	nozzle_heating = nozzle_target > 0? true:false; //nozzle is heating?
	bed_heating = bed_target >0 ? true:false;       //bed is heating?

  // = ((nozzle_target - nozzle_cur) < TEMP_HYSTERESIS)? true:false;
  //bed_heatedComplete = ((bed_target - bed_cur) < TEMP_HYSTERESIS)? true:false;
  nozzle_heatedComplete = ((nozzle_cur +5 >=nozzle_target) && nozzle_heating)?true:false;
  bed_heatedComplete = ((bed_cur +5 >=bed_target) && bed_heating)?true:false;

  if(!ui.nozzle_beform_bed &&  nozzle_heating && nozzle_heatedComplete)
  {
    ui.preheat_state = false;
    thermalManager.temp_bed.target = ui.bed_target;
  }
  else if(ui.nozzle_beform_bed &&  bed_heating && bed_heatedComplete)
  {
    ui.preheat_state = false;
    ui.nozzle_beform_bed = false;
    thermalManager.temp_hotend[0].target = ui.nozzle_target;
  }
}

void nozzle_or_bed_heating_tark(){
  
  int16_t nozzle_target,nozzle_cur,bed_target,bed_cur;
  bool nozzle_heating,nozzle_heatedComplete,bed_heating,bed_heatedComplete;

	nozzle_cur = thermalManager.degHotend(0);         //nozzle current temperature
	nozzle_target = thermalManager.degTargetHotend(0);// nozzle target temperature
	bed_cur = thermalManager.degBed();                //bed current temperature
	bed_target = thermalManager.degTargetBed();       //bed target temperature

	nozzle_heating = nozzle_target > 0? true:false; //nozzle is heating?
	bed_heating = bed_target >0 ? true:false;       //bed is heating?

  // = ((nozzle_target - nozzle_cur) < TEMP_HYSTERESIS)? true:false;
  //bed_heatedComplete = ((bed_target - bed_cur) < TEMP_HYSTERESIS)? true:false;
  nozzle_heatedComplete = ((nozzle_cur +5 >=nozzle_target) && nozzle_heating)?true:false;
  bed_heatedComplete = ((bed_cur +5 >=bed_target) && bed_heating)?true:false;


  if(!ui.have_heated_task) return;

  if(ui.nozzle_heated_state && !ui.bed_heated_state)//Set nozzle temperature
  {
    //ui.have_heated_task = false;//
    thermalManager.temp_hotend[0].target = ui.nozzle_target;
    nozzle_cur = thermalManager.degHotend(0);         //nozzle current temperature
	  nozzle_target = thermalManager.degTargetHotend(0);// nozzle target temperature
    nozzle_heatedComplete = ((nozzle_cur +5 >=nozzle_target) && nozzle_heating)?true:false;
    //SERIAL_ECHOLNPGM("Set Nozzle Temperature");
  }
  else if(!ui.nozzle_heated_state && ui.bed_heated_state) //Set Bed temperature 
  {
    //ui.have_heated_task = false;
    ui.nozzle_beform_bed = true;//   
    thermalManager.temp_bed.target = ui.bed_target;
    bed_cur = thermalManager.degBed();                //bed current temperature
	  bed_target = thermalManager.degTargetBed();       //bed target temperature
    bed_heatedComplete = ((bed_cur +5 >=bed_target) && bed_heating)?true:false;
    //SERIAL_ECHOLNPGM("Set Bed Temperature");
  }
  else if(ui.nozzle_heated_state && ui.bed_heated_state)//Set Nozzle or Bed temperature 
  {
      ui.preheat_state = true;
      //ui.have_heated_task = false;
      //SERIAL_ECHOLNPGM("Set Nozzle or Bed Temperature");
  }

  if(nozzle_heatedComplete && bed_heatedComplete ) {
    ui.nozzle_heated_state = ui.bed_heated_state = false;
    ui.have_heated_task = false;
  	//ui.nozzle_beform_bed = false;
  }

}
#endif // HAS_MARLINUI_MENU
