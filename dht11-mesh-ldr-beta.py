from machine import Pin, I2C, ADC, UART
from time import sleep
import dht
import time
from ssd1306 import SSD1306_I2C
import gc  # Para gestión de memoria

# === CONFIG SERIAL ===
# Configurar UART1 para comunicación serial (GPIO 8=TX, GPIO 9=RX)
uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))  # UART1 a 115200 baudios

# === CONFIG OLED ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

# === CONFIG SENSORES ===
sensor = dht.DHT11(Pin(4))  # DHT11 en GPIO4
led_onboard = Pin("LED", Pin.OUT)
ldr = ADC(Pin(26))  # LDR en GP26 (ADC0)

# === CONFIG DAVIS 6450 ===
davis_signal = ADC(Pin(27))  # Cable verde del Davis 6450 en GP27
davis_enabled = True

# Configuración LDR con valores fijos
MIN_LDR = 27000  # Valor mínimo fijo
MAX_LDR = 65000  # Valor máximo fijo

# Constantes del sensor Davis 6450
MAX_IRRADIANCE = 1800  # W/m²
SENSITIVITY = 0.00167  # V por W/m² (1.67 mV/W/m²)
ADC_MAX = 65535
VREF = 3.3

# Datos del sensor Davis
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

def show_sensor_config():
    """Mostrar configuración de sensores"""
    serial_print(f"LDR configurado: MIN={MIN_LDR}, MAX={MAX_LDR}")
    serial_print(f"Davis 6450: {'Habilitado' if davis_enabled else 'Deshabilitado'}")
    display_message("Sensor Config", f"LDR: {MIN_LDR}-{MAX_LDR}", f"Davis: {'ON' if davis_enabled else 'OFF'}", "Configurado")
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
    if voltage < 0.001: return "Sin señal"
    elif voltage > 3.1: return "Saturado"
    elif irradiance > MAX_IRRADIANCE: return "Fuera rango"
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
        return True
        
    except Exception as e:
        serial_print(f"Error Davis 6450: {e}")
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

def display_sensor_data(temp, hum, lum, davis_irr, counter, status="OK"):
    """Mostrar datos de todos los sensores en formato optimizado para OLED"""
    if davis_irr is not None:
        display_message(
            "T:{:.1f}C H:{:.1f}%".format(temp, hum),
            "LDR:{}% D:{:.0f}".format(lum, davis_irr),
            "Env:{} St:{}".format(counter, status),
            "Triple Sensor"
        )
    else:
        display_message(
            "T:{:.1f}C H:{:.1f}%".format(temp, hum),
            "LDR:{}% D:ERR".format(lum),
            "Env:{} St:{}".format(counter, status),
            "Triple Sensor"
        )

def create_combined_payload(temp, hum, lum, raw_lum, counter):
    """Crear payload combinado con todos los sensores"""
    payload_data = {
        "temperature": round(temp, 1),
        "humidity": round(hum, 1),
        "luminosity_percent": lum,
        "luminosity_raw": raw_lum,
        "device": "picow_triple_sensor",
        "counter": counter,
        "timestamp": time.time()
    }
    
    # Agregar datos del Davis 6450 si están disponibles
    if davis_enabled and davis_data['irradiance'] > 0:
        payload_data.update({
            "solar_irradiance": davis_data['irradiance'],
            "solar_voltage": davis_data['voltage'],
            "solar_class": davis_data['light_class'],
            "solar_status": davis_data['status']
        })
    
    # Convertir a JSON string
    payload_str = str(payload_data).replace("'", '"')
    return payload_str

def test_all_sensors():
    """Prueba rápida de todos los sensores"""
    serial_print("=== Test de todos los sensores ===")
    
    # Test DHT11
    temp, hum = read_sensor()
    if temp is not None:
        serial_print(f"DHT11: T={temp}°C, H={hum}%")
    else:
        serial_print("DHT11: Error")
    
    # Test LDR
    lum, raw_lum = read_luminosity()
    serial_print(f"LDR: {lum}% (Raw: {raw_lum})")
    
    # Test Davis 6450
    if davis_enabled:
        voltage, irradiance, light_class, status = read_davis()
        if voltage is not None:
            serial_print(f"Davis: {irradiance:.1f} W/m² ({light_class}) - {status}")
        else:
            serial_print("Davis: Error")
    
    sleep(2)

def main():
    """Función principal con triple sensor"""
    serial_print("=== INICIO SISTEMA TRIPLE SENSOR (DHT11 + LDR + DAVIS 6450) ===")
    display_message("Triple Sensor", "DHT11+LDR+Davis", "Raspberry Pico W", "v2.0")
    
    for i in range(5, 0, -1):
        display_message("Triple Sensor", "Raspberry Pico W", "Inicio en: {}s".format(i), "v2.0")
        blink_led(1)
        sleep(1)
    
    # Mostrar configuración de sensores
    show_sensor_config()
    
    # Inicializar Davis 6450
    davis_ready = init_davis()
    if davis_ready:
        display_message("Davis 6450", "Inicializado", "Sensor OK", "Continuando...")
    else:
        display_message("Davis 6450", "Error/Desactivado", "Solo DHT11+LDR", "Continuando...")
    sleep(2)
    
    # Test inicial de sensores
    test_all_sensors()
    
    gc.collect()
    
    serial_print("🚀 Starting triple sensor data collection...")
    display_message("INICIANDO", "Lectura triple", "sensor local", "DHT11+LDR+Davis")
    sleep(2)
    
    try:
        counter = 0
        
        while True:
            if counter % 10 == 0:
                gc.collect()
            
            # Leer sensores DHT11 y LDR
            temp, hum = read_sensor()
            lum, raw_lum = read_luminosity()
            
            # Actualizar datos del Davis 6450
            davis_success = update_davis_data()
            
            if temp is not None and hum is not None:
                # Crear payload combinado
                payload = create_combined_payload(temp, hum, lum, raw_lum, counter)
                
                # Enviar payload por UART1
                uart_send_payload(payload)
                
                # Información detallada por consola serial
                serial_print("=== Datos Combined ===")
                serial_print(f"DHT11 - T: {temp}°C, H: {hum}%")
                serial_print(f"LDR - Percent: {lum}%, Raw: {raw_lum}")
                if davis_success:
                    serial_print(f"Davis - Irr: {davis_data['irradiance']} W/m², Class: {davis_data['light_class']}, Status: {davis_data['status']}")
                else:
                    serial_print("Davis - No data/Error")
                serial_print(f"Payload: {payload}")
                serial_print("=" * 30)
                
                counter += 1
                
                # Mostrar datos en OLED
                davis_irr = davis_data['irradiance'] if davis_success else None
                display_sensor_data(temp, hum, lum, davis_irr, counter, "OK")
                
                blink_led(1)
                
            else:
                serial_print("✗ Invalid DHT11 sensor data")
                display_message(
                    "DHT11: ERROR",
                    "No lee temp/hum",
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
        display_message("DETENIDO", "Por usuario", "Fin programa", "...")
        sleep(2)
        
    except Exception as e:
        serial_print("General error: " + str(e))
        display_message("ERROR GENERAL", str(e)[:16], "Sistema", "reiniciando...")
        sleep(5)
        
    finally:
        display_message("PROGRAMA", "TERMINADO", "Pico W OFF", "Fin")
        for _ in range(5):
            led_onboard.on()
            sleep(0.2)
            led_onboard.off()
            sleep(0.2)

if __name__ == '__main__':
    main()
