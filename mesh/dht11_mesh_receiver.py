from machine import Pin, I2C, ADC, UART
from time import sleep
import dht
import network
import time
from umqtt.simple import MQTTClient
from ssd1306 import SSD1306_I2C
import gc  # Para gestión de memoria
import json  # Para procesar datos JSON

# === CONFIG SERIAL ===
# Configurar UART1 para comunicación serial (GPIO 8=TX, GPIO 9=RX)
uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))  # UART1 a 115200 baudios

# === CONFIG WIFI ===
wifi_ssid = "123456789"
wifi_password = "123456789"

# === CONFIG MQTT LOCAL (MOSQUITTO) ===
mqtt_host = "192.168.232.27"  # Ton broker local
mqtt_port = 1883
mqtt_topic = "capteur/temperature"
mqtt_client_id = "dht11_sensor_client_local"

# === CONFIG OLED ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

# === CONFIG SENSORES LOCALES (OPCIONAL) ===
sensor = dht.DHT11(Pin(4))  # DHT11 en GPIO4
led_onboard = Pin("LED", Pin.OUT)
ldr = ADC(Pin(26))  # LDR en GP26 (ADC0)

# Variables para almacenar datos recibidos
received_data = {
    'temperature': None,
    'humidity': None,
    'luminosity_percent': None,
    'luminosity_raw': None,
    'device': 'unknown',
    'counter': 0
}

def serial_print(payload):
    """Función para enviar mensajes por serial (solo consola)"""
    try:
        print(payload)
    except Exception as e:
        print(f"Error serial: {e}")

def uart_read_data():
    """Función para leer datos por UART1"""
    try:
        if uart.any():  # Verificar si hay datos disponibles
            raw_data = uart.readline()  # Leer una línea completa
            if raw_data:
                # Decodificar bytes a string
                data_str = raw_data.decode('utf-8').strip()
                serial_print(f"Datos recibidos por UART: {data_str}")
                return data_str
    except Exception as e:
        serial_print(f"Error leyendo UART: {e}")
    return None

def parse_received_data(data_str):
    """Procesar datos recibidos (JSON o formato personalizado)"""
    try:
        # Intentar parsear como JSON
        data = json.loads(data_str)
        
        # Actualizar datos globales
        if 'temperature' in data:
            received_data['temperature'] = float(data['temperature'])
        if 'humidity' in data:
            received_data['humidity'] = float(data['humidity'])
        if 'luminosity_percent' in data:
            received_data['luminosity_percent'] = int(data['luminosity_percent'])
        if 'luminosity_raw' in data:
            received_data['luminosity_raw'] = int(data['luminosity_raw'])
        if 'device' in data:
            received_data['device'] = str(data['device'])
        if 'counter' in data:
            received_data['counter'] = int(data['counter'])
            
        return True
        
    except json.JSONDecodeError:
        # Si no es JSON, intentar parsear formato personalizado
        serial_print(f"No es JSON válido, procesando como texto: {data_str}")
        
        # Ejemplo para formato: "T:25.3,H:60.5,L:45"
        if 'T:' in data_str and 'H:' in data_str:
            try:
                parts = data_str.split(',')
                for part in parts:
                    if part.startswith('T:'):
                        received_data['temperature'] = float(part.split(':')[1])
                    elif part.startswith('H:'):
                        received_data['humidity'] = float(part.split(':')[1])
                    elif part.startswith('L:'):
                        received_data['luminosity_percent'] = int(part.split(':')[1])
                return True
            except:
                pass
                
    except Exception as e:
        serial_print(f"Error procesando datos: {e}")
    
    return False

def display_message(line1="", line2="", line3="", line4="", clear=True):
    """Función para mostrar mensajes en la OLED"""
    if clear:
        oled.fill(0)
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
    serial_print("Inicializando módulo WiFi...")
    display_message("Iniciando...", "Modulo WiFi", "Esperando", "sistema...")
    sleep(3)
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)
    sleep(2)
    wlan.active(True)
    sleep(2)
    timeout = 0
    while not wlan.active() and timeout < 10:
        serial_print("Esperando módulo WiFi...")
        blink_led(1)
        sleep(1)
        timeout += 1
    if wlan.active():
        serial_print("✓ Módulo WiFi inicializado correctamente")
        return wlan
    else:
        serial_print("✗ Error inicializando módulo WiFi")
        display_message("ERROR WiFi", "Modulo no", "responde", "Reiniciar Pico")
        return None

def connect_wifi():
    """Conectar a WiFi con reintentos mejorados"""
    display_message("Conectando WiFi", wifi_ssid, "Preparando...", "")
    wlan = init_wifi_module()
    if not wlan:
        return False
    if not wlan.active():
        wlan.active(True)
        sleep(2)
    gc.collect()
    max_attempts = 3
    for attempt in range(max_attempts):
        serial_print(f"Intento de conexión WiFi {attempt + 1}/{max_attempts}")
        display_message("WiFi Intent {}".format(attempt + 1), wifi_ssid, "Conectando...", "")
        try:
            if wlan.isconnected():
                wlan.disconnect()
                sleep(2)
            wlan.connect(wifi_ssid, wifi_password)
            timeout = 0
            while not wlan.isconnected() and timeout < 30:
                serial_print('Connecting to WiFi... ({}/30)'.format(timeout + 1))
                blink_led(1)
                sleep(1)
                timeout += 1
                if timeout % 5 == 0:
                    display_message("WiFi Intent {}".format(attempt + 1), wifi_ssid, "Tiempo: {}s".format(timeout), "Esperando...")
            if wlan.isconnected():
                ip = wlan.ifconfig()[0]
                serial_print("✓ Connected to WiFi:" + ip)
                display_message("WiFi Conectado", wifi_ssid, "IP:", ip)
                blink_led(3)
                sleep(3)
                return True
            else:
                serial_print(f"✗ WiFi connection failed - Attempt {attempt + 1}")
                display_message("Intent {} fallo".format(attempt + 1), "Reintentando", "en 5 segundos", "")
                sleep(5)
        except Exception as e:
            serial_print(f"WiFi connection error attempt {attempt + 1}: {e}")
            display_message("Error WiFi", "Intent {}".format(attempt + 1), str(e)[:16], "Reintentando...")
            sleep(5)
    serial_print("✗ All WiFi connection attempts failed")
    display_message("WiFi ERROR", "Todos intentos", "fallaron", "Check config")
    return False

def connect_mqtt():
    """Conectar a MQTT con manejo de errores mejorado"""
    display_message("Conectando MQTT", mqtt_host, "Puerto: " + str(mqtt_port), "")
    max_attempts = 3
    for attempt in range(max_attempts):
        try:
            serial_print(f"Intento MQTT {attempt + 1}/{max_attempts}")
            display_message("MQTT Intent {}".format(attempt + 1), mqtt_host, "Puerto: {}".format(mqtt_port), "Conectando...")
            client = MQTTClient(
                client_id=mqtt_client_id,
                server=mqtt_host,
                port=mqtt_port
            )
            client.connect()
            serial_print("✓ Connected to MQTT broker")
            display_message("Conexiones OK", "WiFi: Conectado", "MQTT: Conectado", "Leyendo serial...")
            blink_led(5)
            sleep(3)
            return client
        except Exception as e:
            serial_print(f"✗ MQTT connection error attempt {attempt + 1}: {e}")
            error_msg = str(e)[:16]
            display_message("MQTT Intent {}".format(attempt + 1), "ERROR:", error_msg, "Reintentando...")
            sleep(5)
    serial_print("✗ All MQTT connection attempts failed")
    display_message("MQTT ERROR", "Todos intentos", "fallaron", "Check broker")
    return None

def display_received_data(data, status="WAITING"):
    """Mostrar datos recibidos en formato optimizado para OLED"""
    temp = data.get('temperature', 'N/A')
    hum = data.get('humidity', 'N/A')
    lum = data.get('luminosity_percent', 'N/A')
    counter = data.get('counter', 0)
    
    if temp != 'N/A' and hum != 'N/A':
        display_message(
            "T:{:.1f}C H:{:.1f}%".format(temp, hum),
            "Luz:{}%".format(lum) if lum != 'N/A' else "Luz: N/A",
            "Rcv:{} St:{}".format(counter, status),
            "Serial Reader"
        )
    else:
        display_message(
            "Esperando datos",
            "por serial...",
            "Status: {}".format(status),
            "Serial Reader"
        )

def main():
    """Función principal con lectura serial"""
    serial_print("=== INICIO SISTEMA LECTOR SERIAL + MQTT ===")
    display_message("Serial Reader", "Raspberry Pico W", "Inicializando", "v2.0")
    
    for i in range(5, 0, -1):
        display_message("Serial Reader", "Raspberry Pico W", "Inicio en: {}s".format(i), "v2.0")
        blink_led(1)
        sleep(1)
    
    gc.collect()
    
    # Conectar WiFi
    wifi_connected = False
    for wifi_attempt in range(3):
        serial_print(f"=== INTENTO GENERAL WiFi {wifi_attempt + 1}/3 ===")
        if connect_wifi():
            wifi_connected = True
            break
        else:
            serial_print(f"Fallo general WiFi intento {wifi_attempt + 1}")
            display_message("WiFi Fallo", "Intent general {}".format(wifi_attempt + 1), "Esperando 10s", "antes reintento")
            sleep(10)
    
    if not wifi_connected:
        serial_print("✗ No se pudo establecer conexión WiFi")
        display_message("ERROR CRITICO", "WiFi no conecta", "Check SSID/PASS", "Reiniciar Pico")
        while True:
            blink_led(10)
            sleep(5)
    
    # Conectar MQTT
    mqtt_client = connect_mqtt()
    if not mqtt_client:
        display_message("ERROR CRITICO", "MQTT no conecta", "Check broker", "IP y puerto")
        while True:
            blink_led(10)
            sleep(2)
    
    serial_print("🚀 Starting serial data reading and MQTT forwarding...")
    
    try:
        counter = 0
        error_count = 0
        last_data_time = time.time()
        
        while True:
            if counter % 10 == 0:
                gc.collect()
            
            # Leer datos por serial
            serial_data = uart_read_data()
            
            if serial_data:
                # Procesar datos recibidos
                if parse_received_data(serial_data):
                    # Datos válidos recibidos
                    last_data_time = time.time()
                    
                    # Crear payload para MQTT (mantener formato original o modificar según necesidad)
                    payload = '{{"temperature": {}, "humidity": {}, "luminosity_percent": {}, "luminosity_raw": {}, "device": "{}", "counter": {}, "source": "serial_reader"}}'.format(
                        received_data.get('temperature', 'null'),
                        received_data.get('humidity', 'null'),
                        received_data.get('luminosity_percent', 'null'),
                        received_data.get('luminosity_raw', 'null'),
                        received_data.get('device', 'unknown'),
                        received_data.get('counter', counter)
                    )
                    
                    try:
                        # Publicar a MQTT
                        mqtt_client.publish(mqtt_topic, payload)
                        serial_print("Forwarded to MQTT: " + payload)
                        
                        counter += 1
                        error_count = 0
                        
                        # Mostrar datos en OLED
                        display_received_data(received_data, "OK")
                        
                        blink_led(1)
                        
                    except Exception as e:
                        serial_print("MQTT publish error: " + str(e))
                        error_count += 1
                        display_received_data(received_data, "MQTT ERR")
                        
                        if error_count > 3:
                            serial_print("Too many MQTT errors, reconnecting...")
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
                    serial_print(f"Datos seriales no válidos: {serial_data}")
                    display_received_data(received_data, "BAD DATA")
                    
            else:
                # No hay datos nuevos
                current_time = time.time()
                if current_time - last_data_time > 30:  # 30 segundos sin datos
                    display_received_data(received_data, "NO DATA")
                else:
                    display_received_data(received_data, "WAITING")
            
            sleep(1)  # Verificar serial más frecuentemente
            
    except KeyboardInterrupt:
        serial_print("Stopped by user")
        display_message("DETENIDO", "Por usuario", "Desconectando", "...")
        sleep(2)
        
    except Exception as e:
        serial_print("General error: " + str(e))
        display_message("ERROR GENERAL", str(e)[:16], "Sistema", "reiniciando...")
        sleep(5)
        
    finally:
        if mqtt_client:
            try:
                mqtt_client.disconnect()
                serial_print("Disconnected from MQTT")
            except:
                pass
        display_message("DESCONECTADO", "MQTT cerrado", "Fin programa", "Pico W OFF")
        for _ in range(10):
            led_onboard.on()
            sleep(0.1)
            led_onboard.off()
            sleep(0.1)

if __name__ == '__main__':
    main()
