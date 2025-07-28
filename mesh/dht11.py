from machine import Pin
from time import sleep
import dht

# Initialize the DHT11 sensor on GPIO pin 4
sensor = dht.DHT11(Pin(4))

while True:
  try:
    sleep(2)
    sensor.measure()
    temp = sensor.temperature()
    hum = sensor.humidity()
    print('Temperature: %3.1f C' %temp)
    print('Humidity: %3.1f %%' %hum)
  except OSError as e:
    print('Failed to read sensor.')
