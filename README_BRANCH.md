# SI4735 Arduino Library (Questo ramo)

Questa directory contiene la libreria Arduino **PU2CLR SI4735** per i ricevitori Silicon Labs SI47XX (AM/SSB/FM con supporto RDS). Questo README fornisce una panoramica rapida del ramo e i punti di ingresso principali per partire.

## Panoramica

- Libreria C++ per Arduino che usa il bus **I²C**.
- Supporto per vari chip SI47XX (inclusi SI4735-D60 con patch SSB).
- Esempi pronti per diversi display e piattaforme.

## Requisiti

- Arduino IDE o PlatformIO.
- Una scheda compatibile (es. ATmega328, ESP32, STM32, RP2040) e un modulo SI47XX.
- Alimentazione **3.3V** per il chip SI47XX (con level shifter se la MCU lavora a 5V).

## Installazione rapida

### Arduino IDE
1. Apri **Library Manager**.
2. Cerca **“PU2CLR SI4735”**.
3. Installa l’ultima versione.

### PlatformIO
Aggiungi al `platformio.ini`:

```ini
lib_deps =
  pu2clr/SI4735
```

## Esempi

Gli esempi sono nella cartella [`examples/`](examples). Alcuni punti di partenza comuni:

- `examples/SI47XX_01_SERIAL_MONITOR`
- `examples/SI47XX_02_LCD_20x4_I2C`
- `examples/SI47XX_03_OLED_I2C`
- `examples/SI47XX_04_TFT`

## Documentazione

- Documentazione completa e API: <https://pu2clr.github.io/SI4735/>
- API reference: <https://pu2clr.github.io/SI4735/extras/apidoc/html/>
- Schemi e note hardware: <https://pu2clr.github.io/SI4735/#schematic>

## Note hardware importanti

- **Non collegare** direttamente segnali a 5V al SI47XX: usa un **level shifter**.
- Mantieni i collegamenti I²C il più corti possibile.

## Licenza

Distribuito con licenza MIT. Vedi `license.txt`.

## Supporto

Per domande e contributi, usa le issue su GitHub o contatta l’autore: **pu2clr@gmail.com**.
