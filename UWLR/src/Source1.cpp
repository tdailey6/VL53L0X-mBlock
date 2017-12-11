#include <Wire.h>

#include "VL53L0X_1.h"
#include "LSM9DS1.h"
#include "Header1.h"

void inline check(int function)
{
	// in each function
	if (digitalRead(22)) {
		//stop motors
		function = 1;
	}
	else if (digitalRead(23)) {
		//stop motors
		function = 2;
	}
	else if (digitalRead(24)) {
		//stop motors
		function = 3;
	}
	else if (digitalRead(25)) {
		//stop motors
		function = 4;
	}
}

void Seattle(VL53L0X Sensor1, VL53L0X Sensor2, VL53L0X Sensor3, VL53L0X Sensor4, VL53L0X Sensor5, VL53L0X Sensor6, LSM9DS1 imu, int function, int seed)
{
	while (1)
	{
		if (function == 1) {
			Follow_Wall(Sensor1, Sensor2, Sensor3, Sensor4, Sensor5, Sensor6, imu, function);
		}
		else if (function == 2) {
			Follow_Object(Sensor1, Sensor2, Sensor3, Sensor4, Sensor5, Sensor6, imu, function);
		}
		else if (function == 3) {
			Object_Avoidance(Sensor1, Sensor2, Sensor3, Sensor4, Sensor5, Sensor6, imu, function, seed);
		}
		else if (function == 4) {
			return;
		}
		else {
			if (digitalRead(22)) {
				function = 1;
			}
			else if (digitalRead(23)) {
				function = 2;
			}
			else if (digitalRead(24)) {
				function = 3;
			}
			else if (digitalRead(25)) {
				function = 4;
			}
		}
	}
}



void Follow_Wall(VL53L0X Sensor1, VL53L0X Sensor2, VL53L0X Sensor3, VL53L0X Sensor4, VL53L0X Sensor5, VL53L0X Sensor6, LSM9DS1 imu, int function)
{
	check(function);
	if (function != 1)
		return;
	// Variables
	// cyclic distance buffer
	int Buf_sz = 10;        // size of the buffer below
	int Dist_buf[10];		// buffer holding previous distances
	int circ_buf_loc = 0;   // write location in buffer
							// distances and thresholds
	int front_min = 300;
	int right_min = 120;
	int right_max = 150;
	int temp_dist = 0;

	int side = 0;           // -1: to far from wall; 0: just right; 1: to close to wall
	int turned_for_n = 0;   // amount of loop the robot spent turning while near to the wall
	int turned_for_f = 0;   // amount of loop the robot spent turning while far from the wall

	int check1 = 0;

	digitalWrite(51, HIGH);
	digitalWrite(52, HIGH);
	/// Init Wall_Follow
	// clear buffer
	temp_dist = Sensor1.readRangeContinuousMillimeters();
	for (int i = 0; i < Buf_sz; i++)
	{
		Dist_buf[i] = temp_dist;
	}

	// move forward


	/// Start Wall_Follow
	while (1)
	{
		// add to the distances buffer
		Dist_buf[circ_buf_loc%Buf_sz] = Sensor1.readRangeContinuousMillimeters();
		Serial.print(Sensor1.readRangeContinuousMillimeters());
		Serial.print('\n');

		// if wall is in detected in front
		if ((Sensor3.readRangeContinuousMillimeters() > 200))
		{
			Serial.print("close to front wall");
			Serial.print('\n');

			// stop
			analogWrite(2, 95);
			analogWrite(3, 95);
			Serial.print("stop");
			Serial.print('\n');
			delay(500);

			// clear buffer
			temp_dist = Sensor1.readRangeContinuousMillimeters();
			for (int i = 0; i < Buf_sz; i++)
			{
				Dist_buf[i] = temp_dist;
			}

			// rotate left
			delay(500);
			digitalWrite(52, HIGH);
			digitalWrite(51, LOW);
			delay(500);
			analogWrite(2, 110);
			analogWrite(3, 110);
			Serial.print("rotate left");
			Serial.print('\n');

			// check if sensor distance is increasing
			while (check1 < 6)
			{
				if (Dist_buf[circ_buf_loc%Buf_sz] < Dist_buf[(circ_buf_loc + 1) % Buf_sz])
					check1++;
				else
					check1 = 0;
				Dist_buf[++circ_buf_loc%Buf_sz] = Sensor1.readRangeContinuousMillimeters();
				Serial.print("loop 1    ");
				Serial.print(Sensor1.readRangeContinuousMillimeters());
				Serial.print('\n');
			}
			check1 = 0;

			// clear buffer
			temp_dist = Sensor1.readRangeContinuousMillimeters();
			for (int i = 0; i < Buf_sz; i++)
			{
				Dist_buf[i] = temp_dist;
			}

			// check if sensor distance is decreasing
			while (check1 < 6)
			{
				if (Dist_buf[circ_buf_loc%Buf_sz] > Dist_buf[(circ_buf_loc + 1) % Buf_sz])
					check1++;
				else
					check1 = 0;
				Dist_buf[++circ_buf_loc%Buf_sz] = Sensor1.readRangeContinuousMillimeters();
				Serial.print("loop 2    ");
				Serial.print(Sensor1.readRangeContinuousMillimeters());
				Serial.print('\n');
			}
			check1 = 0;

			// stop
			analogWrite(2, 95);
			analogWrite(3, 95);
			Serial.print("stop");
			Serial.print('\n');
			delay(500);

			// should be correct dist from wall
			//side = 0;

			// clear buffer
			temp_dist = Sensor1.readRangeContinuousMillimeters();
			for (int i = 0; i < Buf_sz; i++)
			{
				Dist_buf[i] = temp_dist;
			}

			// move forward
			delay(500);
			digitalWrite(51, HIGH);
			digitalWrite(52, HIGH);
			delay(500);
			Serial.print("move forward");
			Serial.print('\n');
		}

		// correction
		// if robot is to close to wall
		else if (Dist_buf[circ_buf_loc % 10] < right_min)
		{
			analogWrite(2, 100);
			analogWrite(3, 105);
			Serial.print("away from wall");
			Serial.print(Sensor1.readRangeContinuousMillimeters());
			Serial.print('\n');
		}

		// if robot is to far from wall
		else if (Dist_buf[circ_buf_loc % 10] > right_max)
		{
			analogWrite(2, 105);
			analogWrite(3, 100);
			Serial.print("towards wall");
			Serial.print(Sensor1.readRangeContinuousMillimeters());
			Serial.print('\n');
		}

		else
		{
			analogWrite(2, 100);
			analogWrite(3, 100);

		}

		// move next write location over one
		circ_buf_loc++;
	}
}


int rand_num_gen(int num_range, int seed)
{
	// linear feedback shift register (LFSR)

	// variables
	int temp_seed = seed;
	int new_bit;
	int num_gen;

	// algorithem
	// bit = x^16 + x^15 + x^13 + x^11 + 1
	new_bit = ((temp_seed >> 0) ^ (temp_seed >> 1) ^ (temp_seed >> 3) ^ (temp_seed >> 5)) & 1;
	// shift seed left by one and add new_bit to the left end
	seed = (temp_seed >> 1) | (new_bit << 7);
	// determine random number
	num_gen = seed % num_range;

	return num_gen;
}

void Object_Avoidance(VL53L0X Sensor1, VL53L0X Sensor2, VL53L0X Sensor3, VL53L0X Sensor4, VL53L0X Sensor5, VL53L0X Sensor6, LSM9DS1 imu, int function, int seed)
{
	// variables
	int stopping_dist = 100;

	while (1)
	{
		int x = rand_num_gen(200, seed);
		digitalWrite(51, HIGH);
		digitalWrite(52, LOW);
		delay(500);
		analogWrite(3, 110);
		delay(500);
		analogWrite(2, 110);
		int newx = x * 30;
		Serial.print("delay by ");
		Serial.println(newx);

		// rotate for x * 30 ms
		delay(newx);
		Serial.println("done turning");

		// stop
		analogWrite(2, 95);
		analogWrite(3, 95);
		delay(1000);

		// set motors direction to forward

		// determine time to move

		int sideright = Sensor2.readRangeContinuousMillimeters();
		int front = Sensor3.readRangeContinuousMillimeters();
		int sideleft = Sensor4.readRangeContinuousMillimeters();

		while ((sideright < 100 && sideright >= 40) && (front < 200 && front >  20) && (sideleft < 90 && sideleft > 60))
		{
			sideright = Sensor2.readRangeContinuousMillimeters();
			front = Sensor3.readRangeContinuousMillimeters();
			sideleft = Sensor4.readRangeContinuousMillimeters();




			check(function);
			if (function != 3)
				return;

			digitalWrite(51, HIGH);
			digitalWrite(52, HIGH);
			delay(250);
			// move forward
			analogWrite(3, 110);
			delay(250);
			analogWrite(2, 110);
			Serial.println("moving forward");
			Serial.print(sideleft);
			Serial.print("       ");
			Serial.print(front);
			Serial.print("       ");
			Serial.println(sideright);



		}
		analogWrite(2, 95);
		analogWrite(3, 95);
		delay(2000);

	}

}


void Follow_Object(VL53L0X Sensor1, VL53L0X Sensor2, VL53L0X Sensor3, VL53L0X Sensor4, VL53L0X Sensor5, VL53L0X Sensor6, LSM9DS1 imu, int function)
{
	// turn right x degrees until it senses something or reaches that distance
	// if nothing turn left 2x degree until it senses something or reaches that distance

	// 3 front sensors
	// last side to detect the distance turns that way until center sensor detects something


	// variables
	double follow_dist = 300;
	double max_dist = 600;
	double thres = 20;

	double side = 0;      // -1: left, 0 : center, 1 : right

	double c_dist[3];     // current distance
	double p_dist[3];     // previous distance
	double delta_dist[3];   // diffence between c_dist and p_dist

	bool following[3] = { 0, 0, 0 }; // is the sensor following

	double diff_thres = 60;
	bool start = 1;

	while (1)
	{
		// find distances of all sensors
		c_dist[1] = Sensor1.readRangeContinuousMillimeters();
		c_dist[2] = Sensor2.readRangeContinuousMillimeters();
		c_dist[3] = Sensor3.readRangeContinuousMillimeters();

		// if just starting, set previous distances to current distances
		// so the robot would not toggle on following when a wall is in front of it
		if (start == 1)
		{
			p_dist[1] = c_dist[1];
			p_dist[2] = c_dist[2];
			p_dist[3] = c_dist[3];
			start = 0;
		}
		// determine distance differance
		// detects if something left the sonars view
		delta_dist[1] = abs(c_dist[1] - p_dist[1]);
		delta_dist[2] = abs(c_dist[2] - p_dist[2]);
		delta_dist[3] = abs(c_dist[3] - p_dist[3]);

		/// object needs to be within certain distance before following for the first time

		if (delta_dist[1] > diff_thres)
			// toggle followeing
			following[1] = !following[1];
		if (delta_dist[2] > diff_thres)
		{
			// toggle followeing
			following[2] = !following[2];
			if (following[2])
				side = 0;
		}
		if (delta_dist[3] > diff_thres)
			// toggle followeing
			following[3] = !following[3];

		// Following
		if (following[2])
		{
			if ((c_dist[2] > follow_dist + thres) && (c_dist[2] < max_dist))
			{
				// set motors direction to forward
				digitalWrite(51, HIGH);
				digitalWrite(52, HIGH);
				check(function);
				if (function != 1)
					return;

				// move forward
				analogWrite(2, 100);
				analogWrite(3, 100);
				Serial.println("moving forward");
			}
			else if (c_dist[2] < follow_dist - thres)
			{
				// set motors direction to reverse
				digitalWrite(51, LOW);
				digitalWrite(52, LOW);
				check(function);
				if (function != 1)
					return;

				// move backward
				analogWrite(2, 100);
				analogWrite(3, 100);
				Serial.println("moving backward");
			}
			else
			{
				// stop moving
				analogWrite(2, 95);
				analogWrite(3, 95);
				Serial.println("stopped");
			}
		}

		else if (following[1])
		{
			if (!following[3])
				side = -1;
		}
		else if (following[3])
		{
			if (!following[1])
				side = 1;
		}
		else
		{
			// stop
			analogWrite(2, 95);
			analogWrite(3, 95);
			delay(500);
			Serial.println("stopped");

			if (side == 1)
			{
				// turn right
				digitalWrite(51, 1);
				digitalWrite(52, 0);

				analogWrite(2, 100);
				analogWrite(3, 100);
				Serial.println("turning right");
			}
			else if (side == -1)
			{
				// turn left
				digitalWrite(51, 0);
				digitalWrite(52, 1);

				analogWrite(2, 100);
				analogWrite(3, 100);
				Serial.println("turning left");
			}
			else
			{
				// should not go here
				// stop robot
				analogWrite(2, 95);
				analogWrite(3, 95);
				delay(500);
				Serial.println("error");
			}
		}

		p_dist[1] = c_dist[1];
		p_dist[2] = c_dist[2];
		p_dist[3] = c_dist[3];



		// will have a problem if someone is out of range and moves toward the robot

		// free bluetooth remote on phones
		// https://os.mbed.com/components/Adafruit-Bluefruit-LE-UART-Friend/
	}
}