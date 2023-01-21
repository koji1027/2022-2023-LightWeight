#include <Arduino.h>

#define SPEAKER_PIN 8

#define Bt2 1200 // 2分音符
#define Bt4 500  // 4分音符、スタッカート気味
#define Bt8f 450 // 付点8分音符
#define Bt8 300  // 8分音符
#define Bt16 150 // 16分音符

// 音符に対してのDelay Time(msec)を設定
#define Dt2 1100
#define Dt4 500
#define Dt8f 350
#define Dt8 200
#define Dt16 50

// 音程を定義する
#define G4 392  // ソ
#define Ds4 311 // レ＃
#define As4 466 // ラ＃
#define D5 587  // レ
#define Ds5 622 // レ＃
#define Fs4 370 // ファ＃
#define G5 784  // ソ
#define Fs5 740 // ファ＃
#define F5 698  // ファ
#define E5 659  // ミ
#define Gs4 415 // ソ＃
#define Cs5 554 // ド＃
#define C5 523  // ド
#define B4 494  // シ
#define A4 440  // ラ

const int WIN_3_STARTUP[5] = {3136, 2349, 2093, 1568, 784};
const int WIN_3_STARTUP_DURATION[5] = {100, 100, 100, 100, 250};

void playStartupSound()
{
    for (int i = 0; i < 5; i++)
    {
        tone(SPEAKER_PIN, WIN_3_STARTUP[i]);
        delay(WIN_3_STARTUP_DURATION[i]);
    }
    noTone(SPEAKER_PIN);
}

void playStarWars()
{
    tone(8, G4, Bt4);
    delay(Dt4);
    tone(8, G4, Bt4);
    delay(Dt4);
    tone(8, G4, Bt4);
    delay(Dt4);
    tone(8, Ds4, Bt8f);
    delay(Dt8f);
    tone(8, As4, Bt8);
    delay(Dt8);
    tone(8, G4, Bt4);
    delay(Dt4);
    tone(8, Ds4, Bt8f);
    delay(Dt8f);
    tone(8, As4, Bt8);
    delay(Dt8);
    tone(8, G4, Bt2);
    delay(Dt2);

    // 3~4小節
    tone(8, D5, Bt4);
    delay(Dt4);
    tone(8, D5, Bt4);
    delay(Dt4);
    tone(8, D5, Bt4);
    delay(Dt4);
    tone(8, Ds5, Bt8f);
    delay(Bt8f);
    tone(8, As4, Bt16);
    delay(Dt16);
    tone(8, Fs4, Bt4);
    delay(Dt4);
    tone(8, Ds4, Bt8f);
    delay(Dt8f);
    tone(8, As4, Bt16);
    delay(Dt16);
    tone(8, G4, Bt2);
    delay(Dt2);

    // 5~6小節
    tone(8, G5, Bt4);
    delay(Dt4);
    tone(8, G4, Bt8f);
    delay(Dt8f + 50);
    tone(8, G4, Bt16);
    delay(Dt16);
    tone(8, G5, Bt4);
    delay(Dt4);
    tone(8, Fs5, Bt8f);
    delay(Dt8f);
    tone(8, F5, Bt16);
    delay(Dt16 + 100);
    tone(8, E5, Bt16);
    delay(Dt16 + 100);
    tone(8, Ds5, Bt8);
    delay(Dt8);
    delay(Bt8 - 50);
    tone(8, Gs4, Bt8);
    delay(Dt8);
    tone(8, Cs5, Bt4);
    delay(Dt4);
    tone(8, C5, Bt8f);
    delay(Dt8f);
    tone(8, B4, Bt16);
    delay(Bt16);

    // 7~8小節
    tone(8, As4, Bt16);
    delay(Dt16 + 100);
    tone(8, A4, Bt16);
    delay(Dt16 + 100);
    tone(8, Gs4, Bt8);
    delay(Dt8);
    delay(Bt8 - 50);
    tone(8, Ds4, Bt8);
    delay(Dt8);
    tone(8, Fs4, Bt4);
    delay(Dt4);
    tone(8, Ds4, Bt8f);
    delay(Dt8f);
    tone(8, As4, Bt16);
    delay(Dt16);
    tone(8, G4, Bt4);
    delay(Dt4);
    tone(8, Ds4, Bt8f);
    delay(Dt8f);
    tone(8, As4, Bt16);
    delay(Dt16);
    tone(8, G4, Bt2);
    delay(Dt2);
}