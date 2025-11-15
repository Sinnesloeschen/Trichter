# Biertrichter – Function Overview

The Biertrichter is a standalone ESP32-based device that measures the drinking time and flow rate of a beer funnel using a Hall-effect flow sensor. 
The system automatically detects the start and end of the flow, times the drinking duration, and calculates the average flow speed in milliliters per second. 
All data is displayed in real time on a 0.96" OLED display. A passive buzzer provides acoustic signals at the start and end of the measurement. 
The device is powered by a 18650 Li-Ion battery, charged via TP4056, and stabilized using an MT3608 step-up converter.
