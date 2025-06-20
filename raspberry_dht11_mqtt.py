from machine import Pin, I2C
from time import sleep
import dht
import network
import time
from umqtt.simple import MQTTClient
from ssd1306 import SSD1306_I2C
import gc  # Para gestión de memoria

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

def init_wifi_module():
    """Inicializar módulo WiFi con tiempo de espera adecuado"""
    print("Inicializando módulo WiFi...")
    display_message("Iniciando...", "Modulo WiFi", "Esperando", "sistema...")
    
    # Esperar a que el sistema se estabilice
    sleep(3)
    
    # Desactivar WiFi primero para reset completo
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)
    sleep(2)
    
    # Activar WiFi
    wlan.active(True)
    sleep(2)
    
    # Verificar que el módulo WiFi esté listo
    timeout = 0
    while not wlan.active() and timeout < 10:
        print("Esperando módulo WiFi...")
        blink_led(1)
        sleep(1)
        timeout += 1
    
    if wlan.active():
        print("✓ Módulo WiFi inicializado correctamente")
        return wlan
    else:
        print("✗ Error inicializando módulo WiFi")
        display_message("ERROR WiFi", "Modulo no", "responde", "Reiniciar Pico")
        return None

def connect_wifi():
    """Conectar a WiFi con reintentos mejorados"""
    display_message("Conectando WiFi", wifi_ssid, "Preparando...", "")
    
    # Inicializar módulo WiFi primero
    wlan = init_wifi_module()
    if not wlan:
        return False
    
    # Configurar modo estación si no está configurado
    if not wlan.active():
        wlan.active(True)
        sleep(2)
    
    # Limpiar memoria antes de conectar
    gc.collect()
    
    # Intentar conexión con múltiples reintentos
    max_attempts = 3
    
    for attempt in range(max_attempts):
        print(f"Intento de conexión WiFi {attempt + 1}/{max_attempts}")
        display_message("WiFi Intent {}".format(attempt + 1), wifi_ssid, "Conectando...", "")
        
        try:
            # Desconectar si ya está conectado
            if wlan.isconnected():
                wlan.disconnect()
                sleep(2)
            
            # Intentar conexión
            wlan.connect(wifi_ssid, wifi_password)
            
            # Esperar conexión con timeout más largo
            timeout = 0
            while not wlan.isconnected() and timeout < 30:
                print('Connecting to WiFi... ({}/30)'.format(timeout + 1))
                blink_led(1)
                sleep(1)
                timeout += 1
                
                # Mostrar progreso en pantalla
                if timeout % 5 == 0:
                    display_message("WiFi Intent {}".format(attempt + 1), wifi_ssid, "Tiempo: {}s".format(timeout), "Esperando...")

            if wlan.isconnected():
                ip = wlan.ifconfig()[0]
                print("✓ Connected to WiFi:", ip)
                
                display_message("WiFi Conectado", wifi_ssid, "IP:", ip)
                blink_led(3)  # 3 parpadeos = WiFi OK
                sleep(3)
                return True
            else:
                print(f"✗ WiFi connection failed - Attempt {attempt + 1}")
                display_message("Intent {} fallo".format(attempt + 1), "Reintentando", "en 5 segundos", "")
                sleep(5)
                
        except Exception as e:
            print(f"WiFi connection error attempt {attempt + 1}: {e}")
            display_message("Error WiFi", "Intent {}".format(attempt + 1), str(e)[:16], "Reintentando...")
            sleep(5)
    
    # Si llegamos aquí, todos los intentos fallaron
    print("✗ All WiFi connection attempts failed")
    display_message("WiFi ERROR", "Todos intentos", "fallaron", "Check config")
    return False

def connect_mqtt():
    """Conectar a MQTT con manejo de errores mejorado"""
    display_message("Conectando MQTT", mqtt_host, "Puerto: " + str(mqtt_port), "")
    
    max_attempts = 3
    
    for attempt in range(max_attempts):
        try:
            print(f"Intento MQTT {attempt + 1}/{max_attempts}")
            display_message("MQTT Intent {}".format(attempt + 1), mqtt_host, "Puerto: {}".format(mqtt_port), "Conectando...")
            
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
            print(f"✗ MQTT connection error attempt {attempt + 1}: {e}")
            error_msg = str(e)[:16]  # Limitar mensaje de error
            display_message("MQTT Intent {}".format(attempt + 1), "ERROR:", error_msg, "Reintentando...")
            sleep(5)
    
    # Si llegamos aquí, todos los intentos fallaron
    print("✗ All MQTT connection attempts failed")
    display_message("MQTT ERROR", "Todos intentos", "fallaron", "Check broker")
    return None

def read_sensor():
    """Leer sensor con reintentos"""
    max_attempts = 3
    
    for attempt in range(max_attempts):
        try:
            sensor.measure()
            temp = sensor.temperature()
            hum = sensor.humidity()
            
            # Validar datos
            if temp is not None and hum is not None and temp > -40 and temp < 80 and hum >= 0 and hum <= 100:
                return temp, hum
            else:
                print(f"Datos inválidos del sensor: T={temp}, H={hum}")
                if attempt < max_attempts - 1:
                    sleep(1)
                    
        except Exception as e:
            print(f"✗ DHT11 sensor error attempt {attempt + 1}: {e}")
            if attempt < max_attempts - 1:
                sleep(2)
    
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
    """Función principal con inicialización robusta"""
    print("=== INICIO SISTEMA DHT11 + MQTT ===")
    
    # Espera inicial para estabilización del sistema
    print("Esperando estabilización del sistema...")
    display_message("DHT11 + MQTT", "Raspberry Pico W", "Inicializando", "v1.1")
    
    # Espera más larga al inicio para que el sistema se estabilice
    for i in range(5, 0, -1):
        display_message("DHT11 + MQTT", "Raspberry Pico W", "Inicio en: {}s".format(i), "v1.1")
        blink_led(1)
        sleep(1)
    
    # Limpiar memoria
    gc.collect()
    
    # Conectar WiFi con reintentos
    wifi_connected = False
    for wifi_attempt in range(3):
        print(f"=== INTENTO GENERAL WiFi {wifi_attempt + 1}/3 ===")
        if connect_wifi():
            wifi_connected = True
            break
        else:
            print(f"Fallo general WiFi intento {wifi_attempt + 1}")
            display_message("WiFi Fallo", "Intent general {}".format(wifi_attempt + 1), "Esperando 10s", "antes reintento")
            sleep(10)
    
    if not wifi_connected:
        print("✗ No se pudo establecer conexión WiFi después de todos los intentos")
        display_message("ERROR CRITICO", "WiFi no conecta", "Check SSID/PASS", "Reiniciar Pico")
        while True:
            blink_led(10)  # Parpadeo de error
            sleep(5)
    
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
            # Limpiar memoria periódicamente
            if counter % 10 == 0:
                gc.collect()
            
            # Leer sensor
            temp, hum = read_sensor()
            
            if temp is not None and hum is not None:
                payload = '{{"temperature": {:.1f}, "humidity": {:.1f}, "device": "picow_dht11", "counter": {}}}'.format(temp, hum, counter)
                
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
                    if error_count > 3:
                        print("Too many MQTT errors, reconnecting...")
                        try:
                            mqtt_client.disconnect()
                        except:
                            pass
                        
                        mqtt_client = connect_mqtt()
                        if not mqtt_client:
                            display_message("MQTT perdido", "Reintentando...", "Check broker", "Error: {}".format(error_count))
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
