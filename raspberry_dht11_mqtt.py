from machine import Pin, I2C, ADC, UART
from time import sleep
import dht
import network
import time
from umqtt.simple import MQTTClient
from ssd1306 import SSD1306_I2C
import gc  # Para gestión de memoria

# === CONFIG SERIAL ===
# Configurar UART1 para comunicación serial (GPIO 8=TX, GPIO 9=RX)
uart = UART(1, baudrate= 115200, tx=Pin(8), rx=Pin(9))  # UART1 a 115200 baudios

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

# === CONFIG SENSORES ===
sensor = dht.DHT11(Pin(4))  # DHT11 en GPIO4
led_onboard = Pin("LED", Pin.OUT)
ldr = ADC(Pin(26))  # LDR en GP26 (ADC0)

# === CONFIG DAVIS 6450 ===
davis_signal = ADC(Pin(27))  # Cable verde en GP27
davis_enabled = True

# Constantes del sensor Davis 6450
MAX_IRRADIANCE = 1800  # W/m²
SENSITIVITY = 0.00167  # V por W/m² (1.67 mV/W/m²)
ADC_MAX = 65535
VREF = 3.3

# Configuración LDR con valores fijos
MIN_LDR = 27000  # Valor mínimo fijo
MAX_LDR = 65000  # Valor máximo fijo

# Datos globales del sensor Davis
davis_data = {
    'irradiance': 0,
    'voltage': 0,
    'light_class': 'Unknown',
    'status': 'Unknown'
}

def serial_print(payload):
    """Función para enviar mensajes por serial (solo consola)"""
    try:
        # Solo enviar por consola local
        print(payload)
    except Exception as e:
        print(f"Error serial: {e}")

def uart_send_payload(payload):
    """Función para enviar solo payload por UART1"""
    try:
        uart.write(str(payload) + "\r\n")
    except Exception as e:
        print(f"Error UART: {e}")

def show_ldr_config():
    """Mostrar configuración del LDR"""
    serial_print(f"LDR configurado: MIN={MIN_LDR}, MAX={MAX_LDR}")
    display_message("LDR Config", f"Min: {MIN_LDR}", f"Max: {MAX_LDR}", "Valores fijos")
    sleep(2)

def map_value(x, in_min, in_max, out_min, out_max):
    """Función para mapear rangos mejorada"""
    # Asegurar que no haya división por cero
    if in_max == in_min:
        return out_min
    
    # Limitar el valor de entrada al rango válido
    x = max(in_min, min(in_max, x))
    
    # Mapear el valor
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def read_luminosity(samples=5, delay=0.1):
    """Leer intensidad luminosa con promedio de muestras"""
    total = 0
    
    for _ in range(samples):
        reading = ldr.read_u16()
        total += reading
        sleep(delay)
    
    avg = total / samples
    
    # Mapear a porcentaje (0-100%) usando valores fijos
    lum_percent = map_value(avg, MIN_LDR, MAX_LDR, 0, 100)
    
    return lum_percent, int(avg)

# === FUNCIONES DAVIS 6450 ===
def voltage_to_irradiance(voltage):
    """Convertir voltaje a irradiancia"""
    if voltage < 0.005:  # Umbral mínimo
        return 0.0
    irradiance = voltage / SENSITIVITY
    return min(irradiance, MAX_IRRADIANCE)

def classify_light(irradiance):
    """Clasificar nivel de luz"""
    if irradiance < 10: return "Noche"
    elif irradiance < 150: return "Bajo"
    elif irradiance < 600: return "Medio"
    elif irradiance < 1200: return "Alto"
    else: return "Extremo"

def get_sensor_status(voltage, irradiance):
    """Estado del sensor"""
    if voltage < 0.001: return "Sin_señal"
    elif voltage > 3.1: return "Saturado"
    elif irradiance > MAX_IRRADIANCE: return "Fuera_rango"
    else: return "OK"

def init_davis():
    """Inicializar sensor Davis 6450"""
    if not davis_enabled:
        serial_print("Davis 6450 desactivado")
        return False
    
    try:
        # Prueba de lectura
        raw = davis_signal.read_u16()
        voltage = (raw / ADC_MAX) * VREF
        irradiance = voltage_to_irradiance(voltage)
        
        serial_print(f"Davis 6450 inicializado: {irradiance:.1f} W/m²")
        display_message("Davis 6450", "Inicializado", f"{irradiance:.1f} W/m²", "Sensor OK")
        sleep(2)
        return True
        
    except Exception as e:
        serial_print(f"Error Davis 6450: {e}")
        display_message("Davis 6450", "ERROR", str(e)[:16], "Check sensor")
        sleep(2)
        return False

def read_davis(samples=5):
    """Leer sensor Davis con promedio de muestras"""
    if not davis_enabled:
        return None, None, None, None
    
    try:
        total = 0
        for _ in range(samples):
            total += davis_signal.read_u16()
            time.sleep(0.01)
        
        raw = total / samples
        voltage = (raw / ADC_MAX) * VREF
        irradiance = voltage_to_irradiance(voltage)
        light_class = classify_light(irradiance)
        status = get_sensor_status(voltage, irradiance)
        
        return voltage, irradiance, light_class, status
        
    except Exception as e:
        serial_print(f"Error leyendo Davis: {e}")
        return None, None, None, None

def update_davis_data():
    """Actualizar datos del sensor Davis"""
    global davis_data
    
    voltage, irradiance, light_class, status = read_davis()
    
    if voltage is not None:
        davis_data.update({
            'voltage': round(voltage, 4),
            'irradiance': round(irradiance, 1),
            'light_class': light_class,
            'status': status
        })
        return True
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
            display_message("Conexiones OK", "WiFi: Conectado", "MQTT: Conectado", "Iniciando...")
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

def read_sensor():
    """Leer sensor DHT11 con reintentos"""
    max_attempts = 3
    for attempt in range(max_attempts):
        try:
            sensor.measure()
            temp = sensor.temperature()
            hum = sensor.humidity()
            if temp is not None and hum is not None and temp > -40 and temp < 80 and hum >= 0 and hum <= 100:
                return temp, hum
            else:
                serial_print(f"Datos inválidos del sensor: T={temp}, H={hum}")
                if attempt < max_attempts - 1:
                    sleep(1)
        except Exception as e:
            serial_print(f"✗ DHT11 sensor error attempt {attempt + 1}: {e}")
            if attempt < max_attempts - 1:
                sleep(2)
    return None, None

def display_sensor_data(temp, hum, lum, irradiance, counter, status="OK"):
    """Mostrar datos del sensor en formato optimizado para OLED"""
    display_message(
        "T:{:.1f}C H:{:.1f}%".format(temp, hum),
        "LDR:{}% Sol:{:.0f}".format(lum, irradiance),
        "Env:{} St:{}".format(counter, status),
        "Pico W 3Sens"
    )

def main():
    """Función principal con inicialización robusta"""
    serial_print("=== INICIO SISTEMA DHT11 + LDR + DAVIS 6450 + MQTT ===")
    display_message("DHT11+LDR+DAVIS", "Raspberry Pico W", "Inicializando", "v2.0")
    
    for i in range(5, 0, -1):
        display_message("DHT11+LDR+DAVIS", "Raspberry Pico W", "Inicio en: {}s".format(i), "v2.0")
        blink_led(1)
        sleep(1)
    
    # Mostrar configuración LDR
    show_ldr_config()
    
    # Inicializar sensor Davis 6450
    davis_ok = init_davis()
    if not davis_ok:
        serial_print("⚠️ Davis 6450 no disponible, continuando sin él")
    
    gc.collect()
    
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
    
    mqtt_client = connect_mqtt()
    if not mqtt_client:
        display_message("ERROR CRITICO", "MQTT no conecta", "Check broker", "IP y puerto")
        while True:
            blink_led(10)
            sleep(2)
    
    serial_print("🚀 Starting data transmission to MQTT...")
    
    try:
        counter = 0
        error_count = 0
        
        while True:
            if counter % 10 == 0:
                gc.collect()
            
            # Leer sensores
            temp, hum = read_sensor()
            lum, raw_lum = read_luminosity()
            
            # Actualizar datos Davis 6450
            davis_ok = update_davis_data()
            
            if temp is not None and hum is not None:
                # Crear payload base
                payload_dict = {
                    "temperature": round(temp, 1),
                    "humidity": round(hum, 1),
                    "luminosity_percent": lum,
                    "luminosity_raw": raw_lum,
                    "device": "picow_dht11_ldr_davis",
                    "counter": counter
                }
                
                # Agregar datos Davis si están disponibles
                if davis_ok and davis_data['irradiance'] > 0:
                    payload_dict.update({
                        "solar_irradiance": davis_data['irradiance'],
                        "solar_voltage": davis_data['voltage'],
                        "solar_class": davis_data['light_class'],
                        "solar_status": davis_data['status']
                    })
                
                # Convertir a JSON string
                payload = str(payload_dict).replace("'", '"')
                
                try:
                    mqtt_client.publish(mqtt_topic, payload)
                    
                    # Solo enviar payload por UART1
                    uart_send_payload(payload)
                    
                    # Todo lo demás por consola serial
                    serial_print("Published: " + payload)
                    serial_print(f"LDR Debug - Raw: {raw_lum}, Percent: {lum}%")
                    if davis_ok:
                        serial_print(f"Davis Debug - Irr: {davis_data['irradiance']} W/m², Class: {davis_data['light_class']}, Status: {davis_data['status']}")
                    
                    counter += 1
                    error_count = 0
                    
                    # Mostrar datos en OLED
                    irradiance_display = davis_data['irradiance'] if davis_ok else 0
                    display_sensor_data(temp, hum, lum, irradiance_display, counter, "OK")
                    
                    blink_led(1)
                    
                except Exception as e:
                    serial_print("MQTT publish error: " + str(e))
                    error_count += 1
                    irradiance_display = davis_data['irradiance'] if davis_ok else 0
                    display_sensor_data(temp, hum, lum, irradiance_display, counter, "MQTT_ERR")
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
                serial_print("✗ Invalid sensor data")
                display_message(
                    "Sensor: ERROR",
                    "DHT11 no lee",
                    "Check conexion",
                    "Cont: {}".format(counter)
                )
                for _ in range(3):
                    led_onboard.on()
                    sleep(0.5)
                    led_onboard.off()
                    sleep(0.5)
            
            sleep(5)
            
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
