#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#define COM A0
#define SENSOR_NUM 16
#define LED_PIN D10
#define SELECT_PIN0 D1
#define SELECT_PIN1 D2
#define SELECT_PIN2 D3
#define SELECT_PIN3 D4
#define NEOPIXEL_NUM 33
#define NEOPIXEL_DIN D5
#define BUFFER_SIZE 5

int threshold[SENSOR_NUM] = {90, 90, 90, 90, 90, 90, 90, 90,
                             90, 90, 90, 90, 90, 90, 90, 90};
int neopixel_coloer[3] = {255, 0, 0};

Adafruit_NeoPixel pixels(NEOPIXEL_NUM, NEOPIXEL_DIN, NEO_GRB + NEO_KHZ800);

void pixels_init();
const float SENSOR_THETA[SENSOR_NUM] = {
    PI / 16.0,        2.0 * PI / 16.0,  3.0 * PI / 16.0,  4.0 * PI / 16.0,
    5.0 * PI / 16.0,  6.0 * PI / 16.0,  7.0 * PI / 16.0,  8.0 * PI / 16.0,
    9.0 * PI / 16.0,  10.0 * PI / 16.0, 11.0 * PI / 16.0, 12.0 * PI / 16.0,
    13.0 * PI / 16.0, 14.0 * PI / 16.0, 15.0 * PI / 16.0, 16.0 * PI / 16.0};

float sensor_x[SENSOR_NUM] = {};
float sensor_y[SENSOR_NUM] = {};
uint8_t mode = 0;
int line_flag[SENSOR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int sensor_value[SENSOR_NUM] = {};
int sensor_value_buff[SENSOR_NUM][BUFFER_SIZE];
float line_x = 0;
float line_y = 0;

void setup() {
    // put your setup code here, to run once:
    pinMode(SELECT_PIN0, OUTPUT);
    pinMode(SELECT_PIN1, OUTPUT);
    pinMode(SELECT_PIN2, OUTPUT);
    pinMode(SELECT_PIN3, OUTPUT);

    digitalWrite(SELECT_PIN0, LOW);
    digitalWrite(SELECT_PIN1, LOW);
    digitalWrite(SELECT_PIN2, LOW);
    digitalWrite(SELECT_PIN3, LOW);

    for (int i = 0; i < SENSOR_NUM; i++) {
        sensor_x[i] = cos(SENSOR_THETA[i]);
        sensor_y[i] = sin(SENSOR_THETA[i]);
    }
    Serial.begin(115200);
    Serial1.begin(115200);
}

void loop() {
    // put your main code here, to run repeatedly:
    int _sensor_value[SENSOR_NUM];
    int _line_flag[SENSOR_NUM];
    for (int i = 0; i < SENSOR_NUM; i++) {
        digitalWrite(SELECT_PIN0, byte(i) & (1 << 0));
        digitalWrite(SELECT_PIN1, byte(i) & (1 << 1));
        digitalWrite(SELECT_PIN2, byte(i) & (1 << 2));
        digitalWrite(SELECT_PIN3, byte(i) & (1 << 3));
        delayMicroseconds(10);
        _sensor_value[i] = analogRead(COM);
        delayMicroseconds(10);
    }
    sensor_value[7] = _sensor_value[0];
    sensor_value[6] = _sensor_value[1];
    sensor_value[5] = _sensor_value[2];
    sensor_value[4] = _sensor_value[3];
    sensor_value[3] = _sensor_value[4];
    sensor_value[2] = _sensor_value[5];
    sensor_value[1] = _sensor_value[6];
    sensor_value[0] = _sensor_value[7];
    sensor_value[15] = _sensor_value[8];
    sensor_value[14] = _sensor_value[9];
    sensor_value[13] = _sensor_value[10];
    sensor_value[12] = _sensor_value[11];
    sensor_value[11] = _sensor_value[12];
    sensor_value[10] = _sensor_value[13];
    sensor_value[9] = _sensor_value[14];
    sensor_value[8] = _sensor_value[15];

    for (int i = 0; i < SENSOR_NUM; i++) {
        for (int j = BUFFER_SIZE - 1; j > 0; j--) {
            sensor_value_buff[i][j] = sensor_value_buff[i][j - 1];
        }
        sensor_value_buff[i][0] = sensor_value[i];
        int sum = 0;
        for (int j = 0; j < BUFFER_SIZE; j++) {
            sum += sensor_value_buff[i][j];
        }
        sensor_value[i] = round(sum / float(BUFFER_SIZE));
    }

    for (int i = 0; i < SENSOR_NUM; i++) {
        if (sensor_value[i] > threshold[i]) {
            line_flag[i] = 1;
        } else {
            line_flag[i] = 0;
        }
    }
    for (int i = 0; i < SENSOR_NUM; i++) {
        Serial.print(line_flag[i]);
        Serial.print(" ");
    }
    Serial.println();
    delay(100);

    for (int i = 0; i < SENSOR_NUM; i++) {
        if (line_flag[i] == 1) {
            line_x += sensor_x[i] * line_flag[i];
            line_y += sensor_y[i] * line_flag[i];
        }
    }
}

void setup1() {
    pinMode(NEOPIXEL_DIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    Serial1.begin(9600);
    pixels.begin();
    pixels_init();
}

void loop1() {
    pixels_init();
    while (!Serial1.available() > 0) {
    }
    byte recv_data = Serial1.read();
    if (int(recv_data) == 255) {
        for (int i = 0; i < 3; i++) {
            while (!Serial1.available() > 0) {
            }
            neopixel_coloer[i] = Serial1.read();
        }
        pixels_init();
    } else if (int(recv_data) == 254) {
        int send_data[2] = {0, 0};
        for (int i = 0; i < 2; i++) {
            send_data[i] += line_flag[i] * 1;
            send_data[i] += line_flag[i + 1] * 2;
            send_data[i] += line_flag[i + 2] * 4;
            send_data[i] += line_flag[i + 3] * 8;
            send_data[i] += line_flag[i + 4] * 16;
            send_data[i] += line_flag[i + 5] * 32;
            send_data[i] += line_flag[i + 6] * 64;
            send_data[i] += line_flag[i + 7] * 128;
        }
        Serial1.write(255);
        for (int i = 0; i < 2; i++) {
            Serial1.write(send_data[i]);
        }
    } else if (int(recv_data) == 253) {
        Serial1.write((int)255);
    } else if (int(recv_data) == 252) {
        Serial1.write(byte(255));
        for (int i = 0; i < SENSOR_NUM; i++) {
            Serial1.write(sensor_value[i]);
        }
    }
}

void pixels_init() {
    for (int i = 0; i < NEOPIXEL_NUM; i++) {
        pixels.setPixelColor(
            i, pixels.Color(neopixel_coloer[0], neopixel_coloer[1],
                            neopixel_coloer[2]));
    }
    pixels.setBrightness(150);
    pixels.show();
}