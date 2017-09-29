/**
  ******************************************************************************
  * @file    buzzer.c
  * @author  Kenneth Au
  * @version V1.0.0
  * @date    01-February-2015
  * @brief   This file provides buzzer functions, including initialization, 
  *          toggle control function using the ticks timer interrupt, the buzzer
  *          period control function using the linked timer and song-playing
  *          functions. 
	*
  ******************************************************************************
  * @attention
  *
  * This source is designed for application use. Unless necessary, try NOT to
	* modify the function definition. The constants which are more likely to 
	* vary among different schematics have been placed as pre-defined constant
	* (i.e., "#define") in the header file.
	*
  ******************************************************************************
  */
  
 
#include "buzzer.h"


/* buzzer_control related */
static u8 buzzer_on_flag = 0;                       /*!< Buzzer flag for buzzer_control */
static u16 buzzer_period = 0;                       /*!< The current buzzer on period (in millsecond) for buzzer_control */
static u16 buzzer_time_ms = 0;		                  /*!< The time left (in millisecond) for a buzzer buzz for buzzer_control */
static u8 buzzer_count = 0;                         /*!< Storing the buzzer buzzing count for buzzer_control */

/* Note frequency related */
static u8 buzzer_volume = 25;	                	    /*!< Volume percentage for the buzzer, 0 - 100 (101 for full buzz) */
static u16 buzzer_note_period = 1;                  /*!< The current buzzer musical note period (in microsecond) */
// Musical note period (1/freq) of the 0th octave in macroseconds (us), array to be completed through buzzer_init
static u16 NOTE0_PERIOD[13] = {0};
  
// Song related
static u8 buzzer_song_flag = 0;	                    /*!< Flag for a current song, true if a song is being played */
static const MUSIC_NOTE* buzzer_current_song = 0;   /*!< Pointer of an array of musical note (a song) */
static u16 buzzer_current_song_note_id = 0;         /*!< The current playing note (element id of the buzzer_current song) */
static u16 buzzer_song_note_length = 0;             /*!< Note length of a song (in millisecond) (constant for a particular song) */
static u16 buzzer_song_note_length_left = 0;        /*!< Note length left for the current playing note */
static u16 buzzer_song_note_break = 0;	            /*!< Note break length between notes (in millisecond), can be 0 */
static u8 buzzer_song_note_break_flag = 0;          /*!< Flag for a playing break */

/**
  * @brief  Initialization of buzzer
  * @param  None
  * @retval None
  */
void buzzer_init(void)
{
  // Calculating the NOTE0_PERIOD array
  NOTE0_PERIOD[0] = 0;
  NOTE0_PERIOD[1] = C0_PERIOD;
  for (u8 i = 2; i < 13; ++i) {
    NOTE0_PERIOD[i] = NOTE0_PERIOD[i-1] * 10000 / TWELFTH_ROOT_OF_TWOx10000;
  }
  
  /* GPIO Init */
  gpio_init(BUZZER_GPIO, GPIO_Speed_50MHz, GPIO_Mode_AF_PP, 1);
	
	// Buzzer frequency init (timer and output compare)
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;                 // TimeBase is for timer setting   > refer to P. 344 of library
  TIM_OCInitTypeDef  TIM_OCInitStructure;                         // OC is for channel setting within a timer  > refer to P. 342 of library

  RCC_APB1PeriphClockCmd(BUZZER_TIM_RCC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    // counter will count up (from 0 to FFFF)
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;        //timer clock = dead-time and sampling clock 	
  TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / BUZZER_COUNT_PER_SECOND - 1;                         // 1MHz
  TIM_TimeBaseStructure.TIM_Period = buzzer_note_period;	                    
 
  TIM_TimeBaseInit(BUZZER_TIM, &TIM_TimeBaseStructure);           // this part feeds the parameter we set above
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //set "high" to be effective output
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	              //produce output when counter < CCR
  TIM_OCInitStructure.TIM_Pulse = buzzer_volume;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      
  TIM_ARRPreloadConfig(BUZZER_TIM, ENABLE);
  TIM_Cmd(BUZZER_TIM, ENABLE);	

  BUZZER_TIM_OC_INIT(BUZZER_TIM, &TIM_OCInitStructure);
  
	buzzer_off();
}


/**
  * @brief Turning on the buzzer (keeping the original frequency)
  * @param None
  * @retval None
  */
void buzzer_on(void)
{
	if (buzzer_note_period == 0) {
		buzzer_off();
	} else {
		BUZZER_TIM_SETCOMPARE(BUZZER_TIM, buzzer_note_period * buzzer_volume / 100);
	}
	
}

/**
  * @brief Turning off the buzzer (keeping the original frequency)
  * @param None
  * @retval None
  */
void buzzer_off(void)
{
	BUZZER_TIM_SETCOMPARE(BUZZER_TIM, 0);
}


/**
  * @brief  Buzzer check for handling "buzzer_control" and "buzzer_play_song" (to be called per 1 ms)
  * @param  None
  * @retval None
  */
void buzzer_check(void)
{
  /* Checking for buzzer_control triggered action */
	if (buzzer_on_flag > 0 || buzzer_count > 0) {
		--buzzer_time_ms;
		if (buzzer_time_ms == 0) {
			buzzer_on_flag = !buzzer_on_flag;
			buzzer_on_flag ? buzzer_on() : buzzer_off();

			buzzer_time_ms = buzzer_period;
			
			if (!buzzer_on_flag) {
				--buzzer_count;
			}
		}
	} else if (buzzer_song_flag) {
    
    /* Checking for buzzer_play_song triggered action */
		MUSIC_NOTE current_note = buzzer_current_song[buzzer_current_song_note_id];
		
		// Check if the song is ended
		if (current_note.note == NOTE_END) {
			buzzer_song_flag = 0;
			buzzer_off();
		}
		
		if (buzzer_song_note_length_left > 0) {
			--buzzer_song_note_length_left;
		} else {
			buzzer_song_note_length_left = buzzer_song_note_length;
			if (buzzer_song_note_break) {
				buzzer_song_note_break_flag = !buzzer_song_note_break_flag;
				if (buzzer_song_note_break_flag) {
					buzzer_song_note_length_left = buzzer_song_note_break;
					buzzer_off();
				} else {
					buzzer_on();
				}
			}

			// Change note
			if ((buzzer_song_note_break && !buzzer_song_note_break_flag) || buzzer_song_note_break == 0) {
				current_note = buzzer_current_song[++buzzer_current_song_note_id];
				buzzer_set_note_period(get_note_period(current_note.note, current_note.octave));
				buzzer_on();
			}
		}
	}
}




/**
  * @brief  Generate specific pattern of buzzer
  * @param  count: number of buzz to be generated
  * @param  period: time (in millisecond) for each buzz and each break in between each buzz
  * @retval None
  */
void buzzer_control(u8 count, u16 period)
{
	if (count == 0 || period == 0) {return;}	/* Do nothing */

	buzzer_count = count;
	buzzer_period = buzzer_time_ms = period;
	buzzer_on_flag = 1;
	buzzer_on();
	
	buzzer_stop_song();	/* Cut current playing song */
}


/**
  * @brief Set the buzzer musical note period (in microseconds)
  * @param The musical note period (in microseconds), e.g., 1/440 for note A4
  * @retval None
  */
void buzzer_set_note_period(u16 p)
{
	buzzer_note_period = p;
	if (buzzer_note_period > 0) {
		TIM_SetAutoreload(BUZZER_TIM, buzzer_note_period);
	} else {
		buzzer_off(); 
	}
}

/**
  * @brief Set the volume of the buzzer (Output compare of the timer)
  * @param vol: Volume of timer (0-100)
  */
void buzzer_set_volume(u8 vol)
{
	if (vol > 100) {vol = 100;}
	buzzer_volume = vol;
}

/**
  * @brief Calculate the musical note period 
  * @param note: The musical note enumator
  * @param octave: The selected octave number
  */
u16 get_note_period(MUSIC_NOTE_LETTER note, u8 octave)
{
	u16 note_period = 0;
	if (note == NOTE_REST || note == NOTE_END || note > 12) return 0;
	
	note_period = NOTE0_PERIOD[note];
	
	while (octave--) {
		note_period /= 2;		/* An octave higher is equivalent to half of the period */
	}
	
	return note_period;
}


void buzzer_control_note(u8 count, u16 period, MUSIC_NOTE_LETTER note, u8 octave)
{
  buzzer_set_note_period(get_note_period(note, octave)); 
  buzzer_control(count, period); 
}

/**
  * @brief Start playing song
  * @param Song (an array of MUSIC_NOTE)
  * @param Note length of each note (in millisecond)
  * @param Note length of each break (in millisecond)
  */
void buzzer_play_song(const MUSIC_NOTE* song, u16 note_length, u16 note_break)
{
	if (song == 0) {return;}
	// Cut buzzer_control
	buzzer_on_flag = 0;
	buzzer_count = 0;
	
	buzzer_song_flag = 1;
	buzzer_current_song_note_id = 0;
	buzzer_song_note_length = note_length;
	buzzer_song_note_length_left = note_length;
	buzzer_song_note_break = note_break;
	buzzer_song_note_break_flag = 0;
	
	buzzer_current_song = song;
	buzzer_set_note_period(get_note_period(song[0].note, song[0].octave));
	buzzer_on();
	
}

/**
  * @brief Stop playing a song
  * @param None
  * @retval None
  */
void buzzer_stop_song(void)
{
	buzzer_song_flag = 0;
}