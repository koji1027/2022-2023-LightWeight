#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

const String mode[1] = {"Game"};

bool line_flag = false;
float ball_dir = 0.00;
int ball_dist = 150;
int ball_x, ball_y;
void draw_ball(float ball_dir);
void draw_template();

void setup() {
    Serial.begin(115200);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.drawCircle(32, 32, 12, WHITE);
    display.writeLine(17, 32, 47, 32, WHITE);
    display.writeLine(32, 17, 32, 47, WHITE);
    display.display();
}

void loop() {
    for (int i = 0; i < 360; i++) {
        ball_dir = i / 180.0 * PI;
        draw_ball(ball_dir);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(64, 28);
        if (line_flag) {
            display.println("On Line");
        } else {
            display.println("No Line");
        }
        display.setCursor(64, 40);
        display.print("Mode:");
        display.println(mode[0]);
        display.setTextSize(1);
        display.setCursor(64, 52);
        display.print("MadeByKoji");
        display.display();
        delay(1);
    }
}

void draw_ball(float ball_dir) {
    draw_template();
    ball_x = round(sin(ball_dir) * 27 + 32);
    ball_y = round(32 - cos(ball_dir) * 27);
    display.fillCircle(ball_x, ball_y, 3, WHITE);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(64, 0);
    display.println("Ball:");
    display.setCursor(80, 8);
    float theta = round(ball_dir / PI * 100) / 100.0;
    if (theta > 1) {
        theta -= 2;
    }
    display.print(theta);
    display.println("PI");
    display.setCursor(64, 16);
    display.print("Dist:");
    display.print(ball_dist);
    display.println("cm");
}

void draw_template() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.drawCircle(32, 32, 12, WHITE);
    display.writeLine(17, 32, 47, 32, WHITE);
    display.writeLine(32, 17, 32, 47, WHITE);
}