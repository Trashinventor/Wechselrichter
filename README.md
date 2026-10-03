# GaN-HEMT-Wechselrichter -- Bachlorarbeit
ESP32-Sourcecode für den 3-phasigen GaN-Wechselrichter.

## Features
- 3-phasige SPWM-Generierung mit einstellbaren Parametern
- FreeRTOS auf 2 Kernen
  - Kern 0: UI-System
  - Kern 1: Ausschließlich SPWM-Generierung
- OLED-Display mit Menüsystem
  - User-Interface per Dreh-Encoder bzw. Tastenfeld 
  - Anzeige des Hochschullogos beim booten
  - Ausgabe 2 Ströme zwischen den Phasen

## Menü-Parameter
Über Menü einstellbare Parameter

Parameter | min. | max. | default | Kommentar
----------|-----|-----|---------|-----------
f-PWM     |1 kHz|100 kHz| 100 kHz |PWM-Trägerfrequenz 
f-Sin     |0 Hz | 1kHz | 50 Hz | Frequenz des Sinus-Modulationssignals
Ausst.    | 0 % | 100 % | 100 % | Austeuergrad  

## Ausblick/Ideen
- Betrieb als Halb-, H-Brücke oder 3-phasiger Wechselrichter
- FOR/BLDC-Driver
- Regelschleife über externe Sensorik
