#include <LiquidCrystal.h>
#include <Time.h>
#include <SD.h>
#include <stdint.h>
#include <math.h>
#define PCB_LCD_RS 28
#define PCB_LCD_EN 29
#define PCB_LCD_D4 30
#define PCB_LCD_D5 31
#define PCB_LCD_D6 32
#define PCB_LCD_D7 33
#define PCB_BUTTON_1 14
#define PCB_BUTTON_2 15
#define PCB_BUTTON_3 16
#define PCB_BUTTON_4 17
#define PCB_EXT_PIN_0 23
#define PCB_EXT_PIN_1 22
#define PCB_EXT_PIN_2 21
#define PCB_EXT_PIN_3 20
#define PCB_EXT_PIN_4 19
#define PCB_EXT_PIN_5 18
#define PCB_RELAY_PIN_0 6
#define PCB_RELAY_PIN_1 2
#define PCB_RELAY_PIN_2 0
#define PCB_RELAY_PIN_3 27
#define PCB_CURRENT_SENSE_PIN_0 A12
#define PCB_CURRENT_SENSE_PIN_1 A10
#define PCB_CURRENT_SENSE_PIN_2 A11
#define PCB_CURRENT_SENSE_PIN_3 26
#define PCB_VOLTAGE_SENSE_PIN A14
#define SOCKET_ON HIGH
#define SOCKET_OFF LOW
#define MASTER_COMMAND_TRANSMISSION_START 31
#define SLAVE_COMMAND_ACK 30
#define MASTER_COMMAND_TOGGLE_SOCKET 29
#define MASTER_COMMAND_REQUEST_SOCKET_STATUS 28
#define MASTER_COMMAND_SET_TIME 27
#define MASTER_COMMAND_ENERGY_QUERY 26
#define BUF_SIZE 32
#define CLEAR_SEND_BUF() memset(send_buf, 0, BUF_SIZE)
#define CLEAR_RECV_BUF() memset(recv_buf, 0, BUF_SIZE)
#define CLEAR_LCD() lcd.clear()
#define SET_TO_BEGINNING() lcd.setCursor(0, 0)
#define SET_TO_BEGINNING_ROW2() lcd.setCursor(0, 1)
#define SET_TO_BEGINNING_ROW3() lcd.setCursor(0, 2)
#define SET_TO_BEGINNING_ROW4() lcd.setCursor(0, 3)
#define SD_SLAVE_SELECT 10
#define MAINS_VOLTAGE_RMS 120
#define ONE_DAY_IN_SEC 86400
#define KWH_IN_J 3600000
#define ENERGY_LOG_PERIOD_SEC 10
#define UI_BUF_SIZE 25
#define ZERO_CROSS_THRESHOLD 200
#define MENU_PAGE_NUM 4
#define CUSTOM_FUNC_SIZE 3

// button class, supports both click and hold
class button
{
private:
	uint16_t hold_timeout_ms;
	time_t last_press;
	uint8_t pin, state, prev_level, button_pressed, button_released;
public:
	button(uint8_t pin_num, uint8_t button_mode)
	{
		// mode 0 = active low, mode 1 = active high
		button_pressed = button_mode % 2;
		button_released = (button_mode + 1) % 2;
		pin = pin_num;
		pinMode(pin, INPUT);
		prev_level = button_released;
		hold_timeout_ms = 800;
		last_press = 0;
		state = 0;
	}
	
	bool is_pressed()
	{
		return digitalRead(pin) == button_pressed;
	}
	
	void set_hold_timeout_ms(uint16_t timeout)
	{
		hold_timeout_ms = timeout;
	}
	
	uint8_t update()
	{
		uint8_t curr_level = digitalRead(pin);
		if(prev_level == button_released && curr_level == button_pressed) // start timing
		{
			state = 1;
			last_press = millis();
			prev_level = curr_level;
			return 0;
		}
		
		if(state == 1)
		{
			uint32_t duration = millis() - last_press;
			if(curr_level == button_released)
			{
				state = 0;
				prev_level = curr_level;
				if(duration <= 50)
					return 0;
				if(duration > 50 && duration < hold_timeout_ms)
					return 1;
				if(duration >= hold_timeout_ms)
					return 2;
			}
		}
		return 0;
	};

	bool unique_Press()
	{
		return update() == 1;
	}
	bool is_held()
	{
		return update() == 2;
	}
};

// please see the software overview for a detailed
// description of algorithm used here
class current_reader
{
private:
	uint8_t current_sensor_pin[4];
	int16_t sample_array[3][83];
	uint16_t output_buf[3][10];
	uint8_t size, buffer_index;

	void read_current_internal()
	{
		int32_t sqsum[3] = {0,0,0};
		int32_t sum[3] = {0,0,0};
		int16_t avg[3] = {0,0,0};

		buffer_index = ++buffer_index % 10;

		for(int i = 0; i < size; i++)
		{
			for(int j = 0; j < 3; j++)
			{
				sample_array[j][i] = analogRead(current_sensor_pin[j]);
				sum[j] += sample_array[j][i];
			}
			delayMicroseconds(200);
		}

		for(int j = 0; j < 3; j++)
		{
			avg[j] = sum[j] / size;
			for(int i = 0; i < size; i++)
			{
				sample_array[j][i] -= avg[j];
				sqsum[j] += sample_array[j][i] * sample_array[j][i];
			}
		}

		for(int j = 0; j < 3; j++)
		{
			double current = ((((double)4096 + sqrt((double)sqsum[j] / size)) / 8192) * 50 - 25);
			if(j == 0)
                current - 0.06 < 0 ? current = 0 : current -= 0.06;
            current <= 0.06 ? output_buf[j][buffer_index] = 0 : output_buf[j][buffer_index] = current * 1000;
		}
	}

	uint16_t calc_avg(uint16_t *list, uint16_t count)
	{
		int32_t sum = 0;
		for(int i = 0; i < count; i++)
			sum += list[i];
		return sum / count;
	}
public:
	current_reader(uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4)
	{
		current_sensor_pin[0] = c1;
		current_sensor_pin[1] = c2;
		current_sensor_pin[2] = c3;
		current_sensor_pin[3] = c4;
		size = 83;
		buffer_index = 0;
		memset( sample_array, 0, sizeof( sample_array ) );
		memset( output_buf,   0, sizeof( output_buf )   );
	}

	void read_current(uint16_t current_array[4])
	{
		read_current_internal();
		for(int j = 0; j < 3; j++)
			current_array[j] = calc_avg(output_buf[j], 10);
		current_array[3]= 0;
	}
};

// timer with two operation modes
class timer
{
private:
	bool is_mod;
	uint16_t alert_period;
	time_t last_alert;
	int8_t enabled;
public:
	timer(bool type, uint16_t period)
	{
		is_mod = type;
		alert_period = period;
		if(is_mod)
			last_alert = now();
		else
			last_alert = millis();
		enabled = 1;
	}

	void set_state(int8_t state)
	{
		enabled = state % 2;
	}

	void toggle()
	{
		enabled = ++enabled % 2;
	}

	int8_t is_enabled()
	{
		return enabled;
	}

	bool has_expired()
	{
		if(!enabled)
			return false;
		
		if(is_mod)
		{
			time_t curr_time = now();
			if((curr_time % alert_period == 0) && (curr_time != last_alert))
			{
				last_alert = curr_time;
				return true;
			}
		}
		else
		{
			time_t curr_time = millis();
			if(curr_time - last_alert > alert_period)
			{
				last_alert = curr_time;
				return true;
			}
		}
		return false;
	}
};

// object that holds the value for a setting
class setting
{
private:
	uint8_t size;
	uint8_t curr_val;
public:
	setting(uint8_t s)
	{
		size = s;
		curr_val = 0;
	}
	uint8_t toggle()
	{
		curr_val = ++curr_val % size;
		return curr_val;
	}
	uint8_t get_val()
	{
		return curr_val;
	}
	void set_val(uint8_t val)
	{
		curr_val = val % size;
	}
};

// object that holds a function pointer as well as
// a name, used for user's custom functions
class custom_function_holder
{
private:
	void (*custom_function_ptr)();
	char name[20];
	int8_t enabled;
public:
	custom_function_holder()
	{
		custom_function_ptr = NULL;
		memset(name, 0, 20);
		strcpy(name, "Undefined");
		enabled = 0;
	}
	
	void attach_custom_function(void (*func_ptr)(), char* func_name)
	{
		// assign function pointer
		custom_function_ptr = func_ptr;
		// copy the name for display
		memset(name, 0, 20);
		for(int i = 0; i < 19; i++)
		{
			if(func_name[i] != 0)
				name[i] = func_name[i];
			else
				break;
		}
	}

	void enable()
	{
		enabled = 1;
	}

	void disable()
	{
		enabled = 0;
	}

	void set_state(int8_t state)
	{
		enabled = state % 2;
	}

	void toggle()
	{
		enabled = ++enabled % 2;
	}

	int8_t is_enabled()
	{
		return enabled;
	}
	
	char* get_func_name()
	{
		return name;
	}
	
	void do_custom_function()
	{
		if(custom_function_ptr != NULL && enabled)
			(*custom_function_ptr)();
	}
};


class zero_cross_detector
{
private:
	time_t last_update;
	uint8_t pin, enabled;
	uint16_t last_reading;
	uint16_t zero_cross_threshold;
public:
	zero_cross_detector(uint8_t voltage_sense_pin, uint16_t threshold)
	{
		last_update = micros();
		last_reading = ZERO_CROSS_THRESHOLD + 1;
		pin = voltage_sense_pin;
		zero_cross_threshold = threshold;
		enabled = 1;
	}

	void set_state(int8_t state)
	{
		enabled = state % 2;
	}

	void toggle()
	{
		enabled = ++enabled % 2;
	}

	int8_t is_enabled()
	{
		return enabled;
	}

	// returns 1 if voltage is at a zero crossing going upwards
	// returns 2 if voltage is at a zero crossing going downwards
	// returns 0 if not at a zero crossing
	uint8_t is_zero_cross()
	{
		if(!enabled)
			return 0;

		uint16_t this_reading;
		uint8_t ret = 0;
		// checks every 200 microseconds
		if(micros() - last_update < 200)
			return 0;

		last_update = micros();
		this_reading = analogRead(pin);
		if(last_reading <= zero_cross_threshold && this_reading > zero_cross_threshold)
			ret = 1;
		else if(last_reading >= zero_cross_threshold && this_reading < zero_cross_threshold)
			ret = 2;
	
		last_reading = this_reading;
		return ret;
	}
};

void toggle_socket(uint8_t socket_index, uint8_t socket_state, zero_cross_detector *zcd, uint8_t save_state_to_sd);

uint8_t send_buf[BUF_SIZE];
uint8_t recv_buf[BUF_SIZE];
button button_1(PCB_BUTTON_1, 1);
button button_2(PCB_BUTTON_2, 1);
button button_3(PCB_BUTTON_3, 1);
button button_4(PCB_BUTTON_4, 1);
current_reader c_reader(PCB_CURRENT_SENSE_PIN_0, PCB_CURRENT_SENSE_PIN_1, PCB_CURRENT_SENSE_PIN_2, PCB_CURRENT_SENSE_PIN_3);
timer current_log_timer(true, ENERGY_LOG_PERIOD_SEC);
timer energy_update_timer(false, 3000);
timer UI_update_timer(false, 300);
LiquidCrystal lcd(PCB_LCD_RS, PCB_LCD_EN, PCB_LCD_D4, PCB_LCD_D5, PCB_LCD_D6, PCB_LCD_D7);
setting setting_current_limiter(2);
custom_function_holder custom_func[CUSTOM_FUNC_SIZE];
zero_cross_detector zd(PCB_VOLTAGE_SENSE_PIN, ZERO_CROSS_THRESHOLD);
IntervalTimer current_read_timer;
volatile uint16_t current_array_global[4];

// timer interrupt handler, read current and store them in
// a global current array
void ISR_read_current()
{
	c_reader.read_current((uint16_t*)current_array_global);
}

void setup()
{
	Serial3.begin(9600);
	Serial.begin(9600);
	lcd.begin(20, 4);
	setSyncProvider(getTeensy3Time);
	analogReadResolution(13);
	pinMode(PCB_RELAY_PIN_0, OUTPUT);
	pinMode(PCB_RELAY_PIN_1, OUTPUT);
	pinMode(PCB_RELAY_PIN_2, OUTPUT);
	pinMode(PCB_RELAY_PIN_3, OUTPUT);
	pinMode(PCB_EXT_PIN_0, INPUT);
	pinMode(PCB_EXT_PIN_1, INPUT);
	pinMode(PCB_EXT_PIN_2, INPUT);
	pinMode(PCB_EXT_PIN_3, INPUT);
	pinMode(PCB_EXT_PIN_4, INPUT);
	pinMode(PCB_EXT_PIN_5, INPUT);
	while(!SD.begin(SD_SLAVE_SELECT))
	{
		SET_TO_BEGINNING();
		lcd.print("insert SD card");
	}
	if(recover_state() == -1)
		for(int i; i < 3; i++)
			digitalWrite(get_socket_pin(i), SOCKET_OFF);
	CLEAR_RECV_BUF();
	CLEAR_SEND_BUF();
	custom_func[0].attach_custom_function(demo_auto_lamp, "auto_lamp");
	custom_func[1].attach_custom_function(demo_light_dimmer, "light_dimmer");
	custom_func[2].attach_custom_function(demo_ext_ctrl, "ext_control");
	// the timer interrupt fires every 0.3 seconds to update current reading
	current_read_timer.begin(ISR_read_current, 300000);
	CLEAR_LCD();
	SET_TO_BEGINNING();
}

// custom function of a light dimmer on socket 3
void demo_light_dimmer()
{
	int8_t dimming_delay = 50;
	// turn on zero crossing detector
	zd.set_state(1);
	// turn off interrupts since precise timing is required
	current_read_timer.end();
	noInterrupts();
	CLEAR_LCD();
	print_brightness(dimming_delay);

	while(1)
	{
		// increase brightness when pressing button 1
		if(button_1.unique_Press())
		{
			if(dimming_delay - 10 >= 0)
				dimming_delay -= 10;
			print_brightness(dimming_delay);
			// wait for the next zero crossing
			while(!zd.is_zero_cross());
		}

		// decrease brightness when pressing button 2
		if(button_2.unique_Press())
		{
			if(dimming_delay + 10 < 100)
				dimming_delay += 10;
			print_brightness(dimming_delay);
			while(!zd.is_zero_cross());
		}

		// exit when pressing button 4
		if(button_4.unique_Press())
		{
			// restart timer interrupt and other interrupts
			current_read_timer.begin(ISR_read_current, 300000);
			interrupts();
			custom_func[1].disable();
			return;
		}

		if(zd.is_zero_cross())
		{
			// if at full brightness, just turn it on
			if(dimming_delay == 0)
			{
				digitalWrite(PCB_RELAY_PIN_2, SOCKET_ON);
				continue;
			}
			delayMicroseconds(dimming_delay * 80);
			digitalWrite(PCB_RELAY_PIN_2, SOCKET_ON);
			delayMicroseconds(200);
			digitalWrite(PCB_RELAY_PIN_2, SOCKET_OFF);
		}
	}
}

void print_brightness(int8_t dimming_delay)
{
	SET_TO_BEGINNING();
	lcd.print("brightness: ");
	lcd.print((int)(100 - dimming_delay));
	lcd.print("  ");
}

// a custom function that let another device take over
// control of the power strip, sockets are controlled by
// 3 external pins
void demo_ext_ctrl()
{
	digitalWrite(PCB_RELAY_PIN_0, digitalRead(PCB_EXT_PIN_0));
	digitalWrite(PCB_RELAY_PIN_1, digitalRead(PCB_EXT_PIN_1));
	digitalWrite(PCB_RELAY_PIN_2, digitalRead(PCB_EXT_PIN_2));
}

void loop()
{
	// execute command from PC if available
	if(get_serial_commands())
	{
		uint8_t master_command = recv_buf[0];
		switch(master_command)
		{
			case MASTER_COMMAND_TOGGLE_SOCKET:
			toggle_socket(recv_buf[1], recv_buf[2], &zd, 1);
			send_default_ACK();
			break;
				
			case MASTER_COMMAND_REQUEST_SOCKET_STATUS:
			send_socket_status();
			break;

			case MASTER_COMMAND_SET_TIME:
			Teensy3Clock.set(char_to_int32(recv_buf+1));
			setTime(Teensy3Clock.get());
			send_default_ACK();
			break;
				
			case MASTER_COMMAND_ENERGY_QUERY:
			send_energy(char_to_int32(recv_buf + 1), char_to_int32(recv_buf + 5));
			break;
		}
	}
	
	print_UI();

	// execute custom functions, if they're enabled
	for(int i = 0; i < CUSTOM_FUNC_SIZE; i++)
		custom_func[i].do_custom_function();

	// execute current limiter, if they're enabled
	if(setting_current_limiter.get_val())
		limit_current(5000);

	// store current reading to SD card for energy logging
	if(current_log_timer.has_expired())
		append_current_log(getTeensy3Time(), (uint16_t*)current_array_global);
}

// unfinished :(
void limit_current(int limit_mA)
{
	;
}

// read a light sensor on PCB_EXT_PIN_5, turn
// socket 1 on if dark, on otherwise.
void demo_auto_lamp()
{
	if(analogRead(PCB_EXT_PIN_5) > 1000)
		digitalWrite(PCB_RELAY_PIN_0, SOCKET_OFF);
	else
		digitalWrite(PCB_RELAY_PIN_0, SOCKET_ON);
}

void print_UI()
{
	// press button 4 to change pages
	static uint8_t page = 0;
	if(button_4.unique_Press())
	{
		CLEAR_LCD();
		page = (page + 1) % MENU_PAGE_NUM;
	}
	switch(page)
	{
		case 0:
			SET_TO_BEGINNING();
			lcd.print("Sockets:");
			// first 3 sockets controlled by button press of first 3 buttons
			if(button_1.unique_Press())
				toggle_socket(0, !digitalRead(get_socket_pin(0)), &zd, 1);
			if(button_2.unique_Press())
				toggle_socket(1, !digitalRead(get_socket_pin(1)), &zd, 1);
			if(button_3.unique_Press())
				toggle_socket(2, !digitalRead(get_socket_pin(2)), &zd, 1);
			if(UI_update_timer.has_expired())
			{
				char message[UI_BUF_SIZE];
				for(int i = 0; i < 3; i++)
				{
					make_message(message, i, digitalRead(get_socket_pin(i)), (double)current_array_global[i] / 1000);
					lcd.setCursor(0, i+1);
					lcd.print(message);
				}
			}
			break;
			
		case 1:
			SET_TO_BEGINNING();
			lcd.print("Energy Today:");
			print_time();
			if(energy_update_timer.has_expired())
			{
				uint32_t result[4];
				char message[21];
				calc_energy(now() - ONE_DAY_IN_SEC, now(), result);
				sprintf(message, "1:%.2fkWh 2:%.2fkWh", (double)result[0] / KWH_IN_J, (double)result[1] / KWH_IN_J);
				SET_TO_BEGINNING_ROW2();
				lcd.print(message);
				sprintf(message, "3:%.2fkWh T:%.2fkWh", (double)result[2] / KWH_IN_J, (double)(result[0]+result[1]+result[2]) / KWH_IN_J);
				SET_TO_BEGINNING_ROW3();
				lcd.print(message);
			}
			break;
			
		case 2:
			SET_TO_BEGINNING();
			lcd.print("Custom Programs:");
			if(button_1.unique_Press())
				custom_func[0].toggle();
			if(button_2.unique_Press())
				custom_func[1].toggle();
			if(button_3.unique_Press())
				custom_func[2].toggle();

			for(int i = 0; i < CUSTOM_FUNC_SIZE; i++)
			{
				lcd.setCursor(0, i+1);
				lcd.print(custom_func[i].get_func_name());
				lcd.setCursor(17, i+1);
				if(custom_func[i].is_enabled())
					lcd.print("ON ");
				else
					lcd.print("OFF");
			}
			break;
			
		case 3:
			SET_TO_BEGINNING();
			lcd.print("Settings:");
			// save settings to SD card
			save_state();
			if(button_1.unique_Press())
				current_log_timer.toggle();
			
			if(button_2.unique_Press())
				zd.toggle();

			if(button_3.unique_Press())
				setting_current_limiter.toggle();
			
			SET_TO_BEGINNING_ROW2();
			lcd.print("Log energy: ");
			lcd.setCursor(17, 1);
			if(current_log_timer.is_enabled())
				lcd.print("ON ");
			else
				lcd.print("OFF");

			SET_TO_BEGINNING_ROW3();
			lcd.print("0-cross toggle: ");
			lcd.setCursor(17, 2);
			if(zd.is_enabled())
				lcd.print("ON ");
			else
				lcd.print("OFF");

			SET_TO_BEGINNING_ROW4();
			lcd.print("current limiter: ");
			lcd.setCursor(17, 3);
			if(setting_current_limiter.get_val() == 1)
				lcd.print("ON ");
			else
				lcd.print("OFF");
			break;
			
		default:
			CLEAR_LCD();
			SET_TO_BEGINNING();
			lcd.print("unknown page");
	}
}

void print_time()
{
	SET_TO_BEGINNING_ROW4();
	time_t local_time = now() - 18000;
	lcd.print(year(local_time));
	lcd.print("-");
	lcd.print(month(local_time));
	lcd.print("-");
	lcd.print(day(local_time));
	lcd.print(" ");
	lcd.print(hour(local_time));
	lcd.print(":");
	lcd.print(minute(local_time));
	lcd.print(":");
	lcd.print(second(local_time));
	lcd.print("  ");
}

void make_message(char* message, uint8_t socket_index, uint8_t socket_status, double socket_current)
{
	memset(message, 0, UI_BUF_SIZE);
	sprintf(message, "S%d: ", socket_index + 1);
	if(socket_status == 1)
		sprintf(message + 4, "ON");
	else
	{
		sprintf(message + 4, "OFF             ");
		return;
	}
	if(socket_current > 1)
		sprintf(message + 8, "%.1fA", socket_current);
	else
		sprintf(message + 8, "%.2fA", socket_current);
	double power = socket_current * MAINS_VOLTAGE_RMS;
	if(power > 100)
		sprintf(message + 14, "%.0fW", power);
	else
		sprintf(message + 14, "%.2fW", power);
	for(int i = 0; i < 20; i++)
		message[i] == 0 ? message[i] = ' ' : message[i];
	for(int i = 20; i < 25; i++)
		message[i] = 0;
}

// send out energy consumption of all sockets
// over serial in response of PC's command
void send_energy(time_t start_utc, time_t end_utc)
{
	CLEAR_SEND_BUF();
	uint32_t result[4];
	calc_energy(start_utc, end_utc, result);
	send_buf[0] = SLAVE_COMMAND_ACK;
	send_buf[1] = 16; // 16 bytes of data
	for(int i = 0; i < 4; i++)
		int32_to_char(result[i], &send_buf[2 + 4 * i]);
	Serial3.write(send_buf, 18);
}

// calculates how much energy was used by all sockets, stores the 
// result in result[4], unit is in Joules
void calc_energy(time_t start_utc, time_t end_utc, uint32_t result[4])
{
	if(start_utc > end_utc)
		return;
	
	time_t start_day = get_start_of_day(start_utc);
	time_t current_timestamp, last_timestamp;
	uint16_t current_array[4];
	// how many days between start and end
	uint16_t span = ((get_start_of_day(end_utc) - start_day) / ONE_DAY_IN_SEC) + 1;
	char file_name[10];
	File log_file;
	
	for(int i = 0; i < 4; i++)
		result[i] = 0;
	// for each day between start and end
	for(int i = 0; i < span; i++)
	{
		// look for that day's file, it doesn't exist go to next day
		get_filename(start_day + i * ONE_DAY_IN_SEC, file_name);
		if(!SD.exists(file_name))
			continue;
		
		log_file = SD.open(file_name, FILE_READ);
		if(log_file == NULL)
		{
			CLEAR_LCD();
			SET_TO_BEGINNING();
			lcd.print("cannot read log file");
			delay(1000);
			return;
		}
		
		// after the file is found, go to the first entry after start_utc
		while(1)
		{
			if(read_current_log_entry(&log_file, &current_timestamp, current_array) == -1)
				goto next_file;
			if(current_timestamp >= start_utc)
				break;
		}
		// now we're at the start
		last_timestamp = current_timestamp;
		while(read_current_log_entry(&log_file, &current_timestamp, current_array) != -1)
		{
			if(current_timestamp >= end_utc)
				goto calc_energy_finish;
			// sum up the current reading for integration
			if(current_timestamp - last_timestamp <= ENERGY_LOG_PERIOD_SEC)
				for(int j = 0; j < 4; j++)
					result[j] += current_array[j];
			last_timestamp = current_timestamp;
		}
	next_file:
		log_file.close();
	}
calc_energy_finish:
	// Energy = power * time = current * 120V * time
	for(int j = 0; j < 4; j++)
		result[j] = (uint32_t)(((double)result[j] / 1000) * MAINS_VOLTAGE_RMS * ENERGY_LOG_PERIOD_SEC);
}

// read a current log entry from the log file(at its current position)
// fills up timestamp with the timestamp of that entry, and current_array with
// the current reading of that entry, returns -1 if eof is reached.
int8_t read_current_log_entry(File *log_file, time_t *timestamp, uint16_t *current_array)
{
	// copy an entry to read_buf
	uint8_t read_buf[12];
	for(int i = 0; i < 12; i++)
	{
		if(!log_file->available())
			return -1;
		read_buf[i] = log_file->read();
	}
	*timestamp = char_to_int32(read_buf);
	for(int i = 0; i < 4; i++)
		current_array[i] = char_to_int16(&read_buf[4 + 2*i]);
	return 0;
}

time_t get_start_of_day(time_t time)
{
	TimeElements tm;
	tm.Second = 0;
	tm.Minute = 0;
	tm.Hour = 0;
	tm.Wday = 0;
	tm.Day = day(time);
	tm.Month = month(time);
	tm.Year = year(time) - 1970;
	return makeTime(tm);
}

void get_filename(time_t time, char buf[10])
{
	memset(buf, 0, 10);
	sprintf(buf, "%d%02d%02d", year(time), month(time), day(time));
}

void append_current_log(time_t time, uint16_t current_array[4])
{
	// each entry: time_t current1 current2 current3 current4
	File log_file;
	char file_name[10];
	get_filename(time, file_name);
	log_file = SD.open(file_name, FILE_WRITE);
	if(log_file == NULL)
	{
		CLEAR_LCD();
		SET_TO_BEGINNING();
		lcd.print("cannot write log file");
		delay(100);
		return;
	}
	uint8_t write_buf[12];
	// first 4 bytes is timestamp
	int32_to_char(time, &write_buf[0]);
	// then 2 * 4 bytes of current reading
	for(int i = 0; i < 4; i++)
		int16_to_char(current_array[i], &write_buf[4 + 2*i]);
	// write them into sd card
	log_file.write(write_buf, 12);
	log_file.close();
}

// changes the state of a socket, you can also choose whether or not to save the change to SD card or use
// zero crossing toggle.
void toggle_socket(uint8_t socket_index, uint8_t socket_state, zero_cross_detector *zcd, uint8_t save_state_to_sd)
{
	int8_t socket_pin = get_socket_pin(socket_index);
	if(socket_pin == -1 || socket_index >= 3)
		return;
	// if zero crossing toggles is on, wait until zero crossing
	if(zcd != NULL && zcd->is_enabled() && socket_state == SOCKET_ON && socket_index != 1)
		while(!zcd->is_zero_cross());
	digitalWrite(socket_pin, socket_state);
	if(save_state_to_sd)
		save_state();
}

// save the state of sockets and setting to SD card so
// they can be restored upon restarting 
void save_state()
{
	File state_file = SD.open("STATE", FILE_WRITE);
	if(state_file == NULL)
	{
		CLEAR_LCD();
		SET_TO_BEGINNING();
		lcd.print("cannot write state file");
		delay(1000);
		return;
	}
	state_file.seek(0);
	for(int i = 0; i < 4; i++)
		state_file.write((int8_t)digitalRead(get_socket_pin(i)));
	state_file.write(zd.is_enabled());
	state_file.write(current_log_timer.is_enabled());
	state_file.write((int8_t)setting_current_limiter.get_val());
	state_file.close();
}

// read the state file and restore socket states and settings
int8_t recover_state()
{
	if(!SD.exists("STATE"))
	{
		CLEAR_LCD();
		SET_TO_BEGINNING();
		lcd.print("state file not found");
		delay(1000);
		return -1;
	}
	File state_file = SD.open("STATE", FILE_READ);
	state_file.seek(0);
	for(int i = 0; i < 4; i++)
		digitalWrite(get_socket_pin(i), state_file.read());
	zd.set_state(state_file.read());
	current_log_timer.set_state(state_file.read());
	setting_current_limiter.set_val((uint8_t)state_file.read());
	state_file.close();
	return 0;
}

// fill a byte array with each byte in an int16_t, little endian
void int16_to_char(int16_t int16, uint8_t *c)
{
	c[0] = int16 & 0xff;
	c[1] = (int16 & 0xff00) >> 8;
}

// extract an int16_t from a byte array, little endian
int16_t char_to_int16(uint8_t* c)
{
	int16_t ret = 0;
	ret = c[0];
	ret |= c[1] << 8;
	return ret;
}

// fill a byte array with each byte in an int32_t, little endian
void int32_to_char(int32_t int32, uint8_t *c)
{
	c[0] = int32 & 0xff;
	c[1] = (int32 & 0xff00) >> 8;
	c[2] = (int32 & 0xff0000) >> 16;
	c[3] = (int32 & 0xff000000) >> 24;
}

// extract an int32_t from a byte array, little endian
int32_t char_to_int32(uint8_t* c)
{
	int32_t ret = 0;
	ret = c[0];
	ret |= c[1] << 8;
	ret |= c[2] << 16;
	ret |= c[3] << 24;
	return ret;
}

void send_default_ACK()
{
	Serial3.write(SLAVE_COMMAND_ACK);
	Serial3.write(0); // no data
}

void send_socket_status()
{
	CLEAR_SEND_BUF();	
	uint8_t socket_status = 0;	
	socket_status |= (digitalRead(PCB_RELAY_PIN_3) << 3); // bit position 3 for socket 4's state
	socket_status |= (digitalRead(PCB_RELAY_PIN_2) << 2);
	socket_status |= (digitalRead(PCB_RELAY_PIN_1) << 1);
	socket_status |= (digitalRead(PCB_RELAY_PIN_0) << 0); // bit position 0 for socket 1's state
	
	send_buf[0] = SLAVE_COMMAND_ACK;
	send_buf[1] = 9; // 9 bytes of data
	send_buf[2] = socket_status;
	int16_to_char(current_array_global[0], &send_buf[3]);
	int16_to_char(current_array_global[1], &send_buf[5]);
	int16_to_char(current_array_global[2], &send_buf[7]);
	int16_to_char(current_array_global[3], &send_buf[9]);
	Serial3.write(send_buf, 11); // 2B header + 9B data
}

int8_t get_serial_commands()
{
	uint8_t len;
	if(Serial3.available())
	{
		char c = Serial3.read();
		Serial.print((char)c); // print it back through USB serial for debugging
		if(c == MASTER_COMMAND_TRANSMISSION_START)
		{
			CLEAR_RECV_BUF();
			while(Serial3.available() <= 0);
			// get master command's data length
			len = Serial3.read();
			// read that much bytes
			for(int i = 0; i < len; i++)
			{
				while(Serial3.available() <= 0);
				recv_buf[i] = Serial3.read();
			}	
			return 1;
		}
	}
	else
		return 0;
}

time_t getTeensy3Time()
{
	return Teensy3Clock.get();
}

int8_t get_socket_pin(uint8_t socket_index)
{
	switch(socket_index)
	{
		case 0:	return PCB_RELAY_PIN_0;
		case 1:	return PCB_RELAY_PIN_1;
		case 2:	return PCB_RELAY_PIN_2;
		case 3:	return PCB_RELAY_PIN_3;
		default:	return -1;
	}
}

int8_t get_current_sensing_pin(uint8_t socket_index)
{
	switch(socket_index)
	{
		case 0:	return PCB_CURRENT_SENSE_PIN_0;
		case 1:	return PCB_CURRENT_SENSE_PIN_1;
		case 2:	return PCB_CURRENT_SENSE_PIN_2;
		case 3:	return PCB_CURRENT_SENSE_PIN_3;
		default:	return -1;
	}
}
