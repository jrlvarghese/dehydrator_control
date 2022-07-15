# PIN CONNECTIONS  
Pin 2 - SW_ROT_ENCODER  
Pin 3 - DT_ROT_ENCODER  
Pin 4 - CLK_ROT_ENCODER  

Pin 5 - DT_DISPLAY  
Pin 6 - CLK_DISPLAY  

Pin 7 - DS18B20   

Pin 9 - Servo  

Pin 10, 11, 12 - FOR HEATER CONTROL  
Pin 13 - FOR FAN CONTROL  

### FOR AHT SENSOR    
A4 - SDA  
A5 - SCL  

# Functional elements
Sense temperature using DS18B20 sensor which is used for temperature control.  
AHT21 sensor used for humidity sensing and is for humidity control.  
### 
Sensor readings are taken every one seconds  
Display updated every 2 seconds  
Average of 10 readings taken for controlling temperature and humidity  
Hysteresis of 2 is used for controlling both temperature and humidity  