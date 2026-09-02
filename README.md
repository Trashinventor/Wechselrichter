# GaN-FET-Wechselrichter Bachlorarbeit
ESP32-Sourcecode für den GaN-Wechselrichter für Bachlorarbeit.
Getestet mit Platine.
Source-Code mit neuer MCPWM-Lib version

## Features
- 3-phasige SPWM-Generierung per ESP32-MCU
- FreeRTOS auf 2 Kernen
  - Kern 0: UI-System
  - Kern 1: Ausschließlich SPWM-Generierung
- OLED-Display mit Menüsystem
  - User-Interface per Dreh-Encoder bzw. Tastenfeld 
  - Anzeige des Hochschullogos beim booten
  - Ausgabe 2 Ströme zwischen den Phasen
  - Debug: CPU-Auslastung je Kern

## Menü-Parameter
Über Menü einstellbare Parameter

Parameter | min. | max. | default | Kommentar
----------|-----|-----|---------|-----------
f-PWM     |10 kHz|100 kHz| 100 kHz |PWM-Trägerfrequenz 
f-Sin     |2 Hz | 1kHz | 50 Hz | Frequenz des Sinus-Modulationssignals
Kontrast  | 0 | 255 | 255 | Display-Kontrast (aktuell nicht funktional)

## Ausblick/Ideen
- Ausgabe der Ströme als Kurven
- Ausschließen von evtl. Glitches bei Änderung der PWM-Trägerfrequenz
