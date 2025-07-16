from machine import Pin, I2C, ADC, UART
from time import sleep
import dht
import time
from ssd1306 import SSD1306_I2C
import gc  # Para gestión de memoria

# === CONFIG SERIAL ===
# Configurar UART1 para comunicación serial (GPIO 8=TX, GPIO 9=RX)
uart = UART(1, baudrate= 115200, tx=Pin(8), rx=Pin(9))  # UART1 a 115200 baudios

# === CONFIG OLED ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

# === CONFIG SENSORES ===
sensor = dht.DHT11(Pin(4))  # DHT11 en GPIO4
led_onboard = Pin("LED", Pin.OUT)
ldr = ADC(Pin(26))  # LDR en GP26 (ADC0)

# Configuración LDR con valores fijos
MIN_LDR = 27000  # Valor mínimo fijo
MAX_LDR = 65000  # Valor máximo fijo

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
    """Leer sensor con reintentos"""
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

def display_sensor_data(temp, hum, lum, counter, status="OK", ldr_raw=None):
    """Mostrar datos del sensor en formato optimizado para OLED"""
    if ldr_raw is not None:
        display_message(
            "T:{:.1f}C H:{:.1f}%".format(temp, hum),
            "Luz:{}% R:{}".format(lum, ldr_raw),
            "Env:{} St:{}".format(counter, status),
            "Pico W Sensor"
        )
    else:
        display_message(
            "T:{:.1f}C H:{:.1f}%".format(temp, hum),
            "Luz:{}%".format(lum),
            "Env:{} St:{}".format(counter, status),
            "Pico W Sensor"
        )

def main():
    """Función principal sin conectividad WiFi/MQTT"""
    serial_print("=== INICIO SISTEMA DHT11 + LDR (LOCAL) ===")
    display_message("DHT11 + LDR", "Raspberry Pico W", "Modo Local", "v1.5")
    
    for i in range(5, 0, -1):
        display_message("DHT11 + LDR", "Raspberry Pico W", "Inicio en: {}s".format(i), "V1.5")
        blink_led(1)
        sleep(1)
    
    # Mostrar configuración LDR
    show_ldr_config()
    
    gc.collect()
    
    serial_print("🚀 Starting local data collection...")
    display_message("INICIANDO", "Lectura local", "de sensores", "DHT11 + LDR")
    sleep(2)
    
    try:
        counter = 0
        
        while True:
            if counter % 10 == 0:
                gc.collect()
            
            # Leer sensores
            temp, hum = read_sensor()
            lum, raw_lum = read_luminosity()
            
            if temp is not None and hum is not None:
                payload = '{{"temperature": {:.1f}, "humidity": {:.1f}, "luminosity_percent": {}, "luminosity_raw": {}, "device": "picow_dht11_ldr", "counter": {}}}'.format(
                    temp, hum, lum, raw_lum, counter)
                
                # Solo enviar payload por UART1
                uart_send_payload(payload)
                
                # Todo lo demás por consola serial
                serial_print("Data: " + payload)
                serial_print(f"LDR Debug - Raw: {raw_lum}, Percent: {lum}%")
                
                counter += 1
                
                # Mostrar datos en OLED
                display_sensor_data(temp, hum, lum, counter, "OK", raw_lum)
                
                blink_led(1)
                
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
