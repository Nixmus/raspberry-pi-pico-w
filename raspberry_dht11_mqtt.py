from machine import Pin, I2C
from time import sleep
import dht
import network
import time
from umqtt.simple import MQTTClient
from ssd1306 import SSD1306_I2C

# === CONFIG WIFI ===
wifi_ssid = "pocox6"
wifi_password = "nanin1708"

# === CONFIG MQTT LOCAL (MOSQUITTO) ===
mqtt_host = "192.168.101.236"  # Ton broker local
mqtt_port = 1883
mqtt_topic = "capteur/temp"
mqtt_client_id = "dht11_sensor_client_local"

# === CONFIG OLED ===
# Configuración I2C para OLED en Raspberry Pi Pico W
# I2C0: SDA=GP0, SCL=GP1 o I2C1: SDA=GP2, SCL=GP3
# Usando I2C0 en los pines GP0 y GP1
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

# Capteur DHT11 sur GPIO2 (GP2)
sensor = dht.DHT11(Pin(4))

# LED onboard para indicación visual
led_onboard = Pin("LED", Pin.OUT)

def display_message(line1="", line2="", line3="", line4="", clear=True):
    """Función para mostrar mensajes en la OLED"""
    if clear:
        oled.fill(0)
    
    # Limitar el texto a 16 caracteres por línea para pantalla 128x64
    oled.text(line1[:16], 0, 0)
    oled.text(line2[:16], 0, 16)
    oled.text(line3[:16], 0, 32)
    oled.text(line4[:16], 0, 48)
    oled.show()

def blink_led(times=1):
    """Parpadear LED onboard"""
    for _ in range(times):
        led_onboard.on()
        sleep(0.1)
        led_onboard.off()
        sleep(0.1)

def connect_wifi():
    display_message("Conectando WiFi", wifi_ssid, "Pico W iniciando")
    
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(wifi_ssid, wifi_password)

    timeout = 0
    while not wlan.isconnected() and timeout < 20:
        print('Connecting to WiFi...')
        blink_led(1)
        time.sleep(1)
        timeout += 1

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        print("✓ Connected to WiFi:", ip)
        
        display_message("WiFi Conectado", wifi_ssid, "IP:", ip)
        blink_led(3)  # 3 parpadeos = WiFi OK
        sleep(3)
        return True
    else:
        print("✗ WiFi connection failed")
        display_message("WiFi ERROR", "Timeout", "Check network", "Restarting...")
        return False

def connect_mqtt():
    display_message("Conectando MQTT", mqtt_host, "Puerto: " + str(mqtt_port))
    
    try:
        client = MQTTClient(
            client_id=mqtt_client_id,
            server=mqtt_host,
            port=mqtt_port
        )
        client.connect()
        print("✓ Connected to MQTT broker")
        
        display_message("Conexiones OK", "WiFi: Conectado", "MQTT: Conectado", "Iniciando...")
        blink_led(5)  # 5 parpadeos = Todo OK
        sleep(3)
        
        return client
    except Exception as e:
        print("✗ MQTT connection error:", e)
        error_msg = str(e)[:16]  # Limitar mensaje de error
        display_message("WiFi: OK", "MQTT: ERROR", error_msg, "Reintentando...")
        sleep(3)
        return None

def read_sensor():
    try:
        sensor.measure()
        temp = sensor.temperature()
        hum = sensor.humidity()
        return temp, hum
    except Exception as e:
        print("✗ DHT11 sensor error:", e)
        return None, None

def display_sensor_data(temp, hum, counter, status="OK"):
    """Mostrar datos del sensor en formato optimizado para OLED"""
    display_message(
        "T: {:.1f}C H:{:.1f}%".format(temp, hum),
        "Enviados: {}".format(counter),
        "Estado: {}".format(status),
        "Pico W Sensor"
    )

def main():
    # Mostrar mensaje de inicio
    display_message("DHT11 + MQTT", "Raspberry Pico W", "Sensor Station", "v1.0")
    blink_led(2)
    sleep(3)
    
    # Conectar WiFi
    if not connect_wifi():
        return
    
    # Conectar MQTT
    mqtt_client = connect_mqtt()
    if not mqtt_client:
        display_message("ERROR CRITICO", "MQTT no conecta", "Check broker", "IP y puerto")
        while True:
            blink_led(10)  # Parpadeo de error
            sleep(2)
    
    print("🚀 Starting data transmission to MQTT...")
    
    try:
        counter = 0
        error_count = 0
        
        while True:
            # Leer sensor
            temp, hum = read_sensor()
            
            if temp is not None and hum is not None:
                payload = '{{"temperature": {:.1f}, "humidity": {:.1f}, "device": "picow_dht11"}}'.format(temp, hum)
                
                try:
                    # Publicar a MQTT
                    mqtt_client.publish(mqtt_topic, payload)
                    print("Published:", payload)
                    counter += 1
                    error_count = 0  # Reset error count on success
                    
                    # Mostrar datos en OLED
                    display_sensor_data(temp, hum, counter, "OK")
                    
                    # Parpadeo corto = envío exitoso
                    blink_led(1)
                    
                except Exception as e:
                    print("MQTT publish error:", e)
                    error_count += 1
                    
                    display_sensor_data(temp, hum, counter, "MQTT ERR")
                    
                    # Si hay muchos errores MQTT, intentar reconectar
                    if error_count > 5:
                        print("Too many MQTT errors, reconnecting...")
                        try:
                            mqtt_client.disconnect()
                        except:
                            pass
                        
                        mqtt_client = connect_mqtt()
                        if not mqtt_client:
                            display_message("MQTT perdido", "Reintentando...", "Check broker", "Error: " + str(error_count))
                            sleep(10)
                            continue
                        error_count = 0
                        
            else:
                print("✗ Invalid sensor data")
                display_message(
                    "Sensor: ERROR",
                    "DHT11 no lee",
                    "Check conexion",
                    "Cont: {}".format(counter)
                )
                
                # Parpadeo largo = error de sensor
                for _ in range(3):
                    led_onboard.on()
                    sleep(0.5)
                    led_onboard.off()
                    sleep(0.5)
            
            sleep(5)  # Delay entre mediciones
            
    except KeyboardInterrupt:
        print("Stopped by user")
        display_message("DETENIDO", "Por usuario", "Desconectando", "...")
        sleep(2)
        
    except Exception as e:
        print("General error:", e)
        display_message("ERROR GENERAL", str(e)[:16], "Sistema", "reiniciando...")
        sleep(5)
        
    finally:
        if mqtt_client:
            try:
                mqtt_client.disconnect()
                print("Disconnected from MQTT")
            except:
                pass
                
        display_message("DESCONECTADO", "MQTT cerrado", "Fin programa", "Pico W OFF")
        
        # Parpadeo final
        for _ in range(10):
            led_onboard.on()
            sleep(0.1)
            led_onboard.off()
            sleep(0.1)

if __name__ == '__main__':
    main()
