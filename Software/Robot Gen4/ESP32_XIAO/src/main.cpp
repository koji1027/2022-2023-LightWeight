#include <Arduino.h>
#include <Ultrasonic.h>

#define BUFFER_SIZE 2

Ultrasonic right(A0);
Ultrasonic back(A1);
Ultrasonic left(A2);

long distance[3][BUFFER_SIZE];
long now_distance[3];

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial0.begin(115200);
}

void loop()
{
  // put your main code here, to run repeatedly:
  for (int i = 0; i < BUFFER_SIZE - 1; i++)
  {
    distance[0][i] = distance[0][i + 1];
    distance[1][i] = distance[1][i + 1];
    distance[2][i] = distance[2][i + 1];
  }
  distance[0][BUFFER_SIZE - 1] = right.MeasureInCentimeters();
  if (distance[0][BUFFER_SIZE - 1] > 400)
  {
    distance[0][BUFFER_SIZE - 1] = distance[0][BUFFER_SIZE - 2];
  }
  delay(20);
  distance[1][BUFFER_SIZE - 1] = back.MeasureInCentimeters();
  if (distance[1][BUFFER_SIZE - 1] > 400)
  {
    distance[1][BUFFER_SIZE - 1] = distance[1][BUFFER_SIZE - 2];
  }
  delay(20);
  distance[2][BUFFER_SIZE - 1] = left.MeasureInCentimeters();
  if (distance[2][BUFFER_SIZE - 1] > 400)
  {
    distance[2][BUFFER_SIZE - 1] = distance[2][BUFFER_SIZE - 2];
  }
  delay(20);
  for (int i = 0; i < 3; i++)
  {
    int min_index = 0;
    for (int j = 0; j < BUFFER_SIZE; j++)
    {
      if (distance[i][j] < distance[i][min_index])
      {
        min_index = j;
      }
    }
    now_distance[i] = distance[i][min_index];
  }
  Serial.print(now_distance[0]);
  Serial.print("\t");
  Serial.print(now_distance[1]);
  Serial.print("\t");
  Serial.println(now_distance[2]);
  byte data[7];
  data[0] = 255;
  for (int i = 0; i < 3; i++)
  {
    data[i * 2 + 1] = lowByte(now_distance[i]);
    data[i * 2 + 2] = highByte(now_distance[i]);
  }
  Serial0.write(data, 7);
  delay(10);
}
