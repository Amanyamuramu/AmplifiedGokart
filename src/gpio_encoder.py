import RPi.GPIO as GPIO
import time

PinEncA = 14  # A相（BCM方式で GPIO 14）
PinEncB = 15  # B相（BCM方式で GPIO 15)
counter = 0

def handle_encoder(channel):
    global counter
    if GPIO.input(PinEncA) == GPIO.LOW:
        if GPIO.input(PinEncB) == GPIO.LOW:
            counter += 1
        else:
            counter -= 1
    print("Encoder value:", counter)

GPIO.setmode(GPIO.BCM)
GPIO.setup(PinEncA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PinEncB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.add_event_detect(PinEncA, GPIO.FALLING, callback=handle_encoder, bouncetime=10)

try:
    while True:
        time.sleep(0.1)  # メインループで待機
except KeyboardInterrupt:
    print("終了します")
finally:
    GPIO.cleanup()
