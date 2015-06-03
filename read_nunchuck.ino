#include <Servo.h>
#include <Wire.h>
#include <math.h>

#define MOTOR_PIN 13
#define SDA_PIN 2
#define SCL_PIN 3

int lastMotor = 25;

// Wii nunchuck globals
#define MAX_STR_BUF 255

// I2C uses SCL and SDA
#define WIICHUCKID 0x52
#define USEFAKEPOWER false  // If true, use analog pins 2&3 as fake gnd & pwr
#define ENC_VALUE  0x00    // No encryption. Set to 0x17 if experimenting with encryption modes

struct nunchuck_data_s { 
        uint16_t joy_x;
        uint16_t joy_y;
        uint16_t accel_x;
        uint16_t accel_y;
        uint16_t accel_z;
        uint16_t button_z;
        uint16_t button_c;
};
nunchuck_data_s *g_nunchuck_curr = new nunchuck_data_s();
static uint8_t   g_nunchuck_buf[6];
static uint32_t  g_nunchuck_seq = 0;

Servo motor;

void setup() {
	Serial.begin(115200);
	Serial.println("Begin...");
	
	motor.attach(MOTOR_PIN);

        // Set up Wii nunchuck
    Serial.print("-------------------------------------------------------------------------------\n");
    Serial.print("-- Setting up the Wii nunchuck...\n");
   // if (USEFAKEPOWER) nunchuck_setpowerpins();
    nunchuck_init();
    Serial.print("\n");
	

}

void loop() {
	 if (!nunchuck_get_data())
    {
        Serial.print("!! Error: incomplete data from device (controller not synced?)\n");
        delay(1000);
        return;
    }
    else if (g_nunchuck_buf[0] == 0x00 && g_nunchuck_buf[1] == 0x00 && 
             g_nunchuck_buf[2] == 0x00 && g_nunchuck_buf[3] == 0x00 && 
             g_nunchuck_buf[4] == 0x00 && g_nunchuck_buf[5] == 0x00)
    {
        Serial.print("!! Error: Data pegged low (controller not synced?)\n");
        delay(1000);
        return;
    }
    else if (g_nunchuck_buf[0] == 0xFF && g_nunchuck_buf[1] == 0xFF && 
             g_nunchuck_buf[2] == 0xFF && g_nunchuck_buf[3] == 0xFF && 
             g_nunchuck_buf[4] == 0xFF && g_nunchuck_buf[5] == 0xFF)
    {
        Serial.print("!! Error: Data pegged high: attempting re-initialization\n");
        nunchuck_init(); // send the initilization handshake
        delay(1000);
        return;
    }
    else
    {
            //nunchuck_print_data();
            if (g_nunchuck_curr->button_z == 0) {
              Serial.println(motorSpeed(stickVal(g_nunchuck_curr->joy_y)));
              lastMotor = motorSpeed(stickVal(g_nunchuck_curr->joy_y));
            }
            else {
              Serial.println("Z button not pressed.");
              lastMotor = 25;
            }
            motor.write(lastMotor);
            delay(1);
    }
	//while (!Serial.available());
	//motor.write(motorSpeed(Serial.parseInt());
}

// takes raw stick input and converts to percent, delta 132 -> 255
int stickVal(int raw_in) {
  if (raw_in < 133)
    return 0;
  return ((100*(raw_in-133))/(255 - 133));
}

//server write values: min is 62, max 112
int motorSpeed(int percent) {
	if (percent < 0)
	return 25;
	else if (percent > 100)
	return 112;
	return (percent / 2) + 62;   
}
//#############################################################################
//#############################################################################
// Wii nunchuck code
//#############################################################################
// Uses port C (analog in) pins as power & ground for Nunchuck
//static void nunchuck_setpowerpins()
//{
//#define pwrpin PORTC3
//#define gndpin PORTC2
//    DDRC |= _BV(pwrpin) | _BV(gndpin);
//    PORTC &=~ _BV(gndpin);
//   PORTC |=  _BV(pwrpin);
//    delay(100);  // wait for things to stabilize        
//}

//#############################################################################
// initialize the I2C system, join the I2C bus,
// and tell the nunchuck we're talking to it
void nunchuck_init()
{ 
    Serial.print("-- Initializing the controller\n");

    g_nunchuck_seq = 0; // Reset the sequence counter

    Wire.begin(); // join i2c bus as master

    boolean INITIALIZING = true;

    while (INITIALIZING) {
        if (ENC_VALUE == 0) { // Third-party controllers don't allow encryption
            // It seems critical for F0:55 and FB:00 to be transmitted
            // "separately" when it comes to the Nyko Kama
            Wire.beginTransmission(WIICHUCKID);
            Wire.write((uint8_t)0xF0);
            Wire.write((uint8_t)0x55);
            Wire.endTransmission();

            Wire.beginTransmission(WIICHUCKID);
            Wire.write((uint8_t)0xFB);
            Wire.write((uint8_t)0x00);
            Wire.endTransmission();
        }
        else { // Using encryption (only works with Nintendo-made controllers)
            Wire.beginTransmission(WIICHUCKID);
            Wire.write((uint8_t)0xFB);
            Wire.write((uint8_t)0x00);
            Wire.write((uint8_t)0x40);
            Wire.write((uint8_t)0x17);
            Wire.endTransmission();
        }

        Wire.beginTransmission(WIICHUCKID);
        Wire.write((uint8_t)0xFA);
        Wire.endTransmission();

        const int BYTES_REQD = 6;
        Wire.requestFrom(WIICHUCKID, BYTES_REQD, true); // Request n bytes then release bus

        delay(1); // Doesn't seem to make a difference

        char tmp_str_buf[MAX_STR_BUF];
        int length = 0;
        length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "-- Controller ident = [");

        int cnt = 0;
        while (Wire.available ())
        {
            g_nunchuck_buf[cnt] = nunchuk_decode_byte(Wire.read());
            if (cnt > 0)
            {
                length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, ", ");
            }
            length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "%02X", g_nunchuck_buf[cnt]);
            cnt++;
        }
        length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "]\n");

        if (cnt == BYTES_REQD)
        {
            INITIALIZING = false;
            Serial.print(tmp_str_buf);
        }
        else
        {
            Serial.print("!! Error: Init failed. Retrying...\n");
            delay(1000);
        }
    }

    delay(100); // This seemed to be important but things may work fine without it
}

//#############################################################################
// Send a request for data to the nunchuck
void nunchuck_send_request()
{
    Wire.beginTransmission(WIICHUCKID);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
}

//#############################################################################
// Receive data back from the nunchuck
int nunchuck_get_data()
{
    int cnt = 0;
    int BYTES_REQD = 6;

    Wire.requestFrom (WIICHUCKID, BYTES_REQD); // Request n bytes then release bus

    while (Wire.available())
    {
        g_nunchuck_buf[cnt] = nunchuk_decode_byte(Wire.read());
        cnt++;
    }

    nunchuck_send_request(); // Request next data payload

    if (cnt != BYTES_REQD)
    {
        return 0; // Failure
    }

    g_nunchuck_seq++;

    // Also unpack g_nunchuck_buf[5] byte as ZZYYXXcz (accel LSBs and then the buttons)
    g_nunchuck_curr->joy_x    =                                   g_nunchuck_buf[0];
    g_nunchuck_curr->joy_y    =                                   g_nunchuck_buf[1];
    g_nunchuck_curr->button_z =  (g_nunchuck_buf[5] >> 0) & 1;
    g_nunchuck_curr->button_c =  (g_nunchuck_buf[5] >> 1) & 1;
    g_nunchuck_curr->accel_x  = ((g_nunchuck_buf[5] >> 2) & 3) | (g_nunchuck_buf[2] << 2);
    g_nunchuck_curr->accel_y  = ((g_nunchuck_buf[5] >> 4) & 3) | (g_nunchuck_buf[3] << 2);
    g_nunchuck_curr->accel_z  = ((g_nunchuck_buf[5] >> 6) & 3) | (g_nunchuck_buf[4] << 2);

    return 1; // Success
}

//#############################################################################
// Print the input data we have recieved
void nunchuck_print_data()
{ 
    char tmp_str_buf[MAX_STR_BUF];
    int length = 0;

    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "SEQ# %10d:  ", g_nunchuck_seq);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "joy: (h,v)=(%3d, %3d)  ",
        g_nunchuck_curr->joy_x, g_nunchuck_curr->joy_y);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "accel: (x,y,z)=(%4d, %4d, %4d)  ",
        g_nunchuck_curr->accel_x, g_nunchuck_curr->accel_y, g_nunchuck_curr->accel_z);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "buttons: (c,z)=(%1d, %1d)  ",
        g_nunchuck_curr->button_c, g_nunchuck_curr->button_z);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "RAW: [%02X, %02X, %02X, %02X, %02X, %02X]",
        g_nunchuck_buf[0], g_nunchuck_buf[1], g_nunchuck_buf[2],
        g_nunchuck_buf[3], g_nunchuck_buf[4], g_nunchuck_buf[5]);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "\n");

    Serial.print(tmp_str_buf);
}

//#############################################################################
// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
char nunchuk_decode_byte(char x)
{
    if (ENC_VALUE != 0) {
        x = (x ^ ENC_VALUE) + ENC_VALUE;
    }
    return x;
}

//#############################################################################
