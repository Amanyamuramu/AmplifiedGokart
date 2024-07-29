#include <Arduino.h>

// ロータリーエンコーダ設定

// Devkit選択
#define DEVKIT_25 1
//#define DEVKIT_26 1

#if defined(DEVKIT_25)
  #define LED_BLUE 2
  #define PinEncA 16
  #define PinEncB 17
#elif defined(DEVKIT_26)
  #define LED_BLUE 2
  #define PinEncA 25
  #define PinEncB 26
#else
  #error "Devkit not defined"
#endif

volatile int counter = 0;
bool led_switch = false;

// 割り込み関数
void IRAM_ATTR handleEncoder()
{
  if (digitalRead(PinEncA) == LOW)
  {
    if (digitalRead(PinEncB) == LOW)
    {
      counter++;
    }
    else
    {
      counter--;
    }
  }
}

void setup()
{
  // 入力ピン設定
  pinMode(PinEncA, INPUT_PULLUP);
  pinMode(PinEncB, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);

  // 割り込み設定
  attachInterrupt(digitalPinToInterrupt(PinEncA), handleEncoder, FALLING);
  Serial.begin(115200);
}

void loop()
{
  static int lastCounter = 0;
  if (counter != lastCounter)
  {
    Serial.print("Encoder value: ");
    Serial.println(counter);
    lastCounter = counter;
    led_switch = !led_switch;
    if (led_switch)
    {
      digitalWrite(LED_BLUE, HIGH);
    }
    else
    {
      digitalWrite(LED_BLUE, HIGH);
    }
  }
  delay(10);
}
