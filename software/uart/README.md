# ESP32 DevKitC + ADS131E04: acquisizione su comando via UART

L'ESP32 resta in attesa. Il PC manda un frame START con i parametri (frequenza di campionamento, canali, guadagni, ingresso, durata), l'ESP32 configura l'ADC, acquisisce e invia i campioni al PC, poi chiude con un frame END che riporta le statistiche di integrità.

```
ads131e04_uart/
  CMakeLists.txt          progetto ESP-IDF
  sdkconfig.defaults      240 MHz, tick 1 ms, nessun log/console su UART0
  main/
    board.h               pin (dallo schema), baud, SPI, tempi
    ads131e04.c/.h        driver ADS131E04 (SPI, comandi, registri, PCNT su DRDY)
    proto.c/.h            protocollo binario (portabile, testabile su PC)
    main.c                task: comandi, acquisizione, trasmissione
  pc/
    ads_client.py         client Python: START, ricezione, verifica, CSV in volt
```

## Collegamenti (dallo schema)

| ADS131E04 | ESP32 | Note |
|---|---|---|
| CS (39) | GPIO23 | gestito a mano dal firmware |
| SCLK (40) | GPIO22 | SPI modo 1, 8 MHz |
| DIN (34) | GPIO21 | MOSI |
| DOUT (43) | GPIO19 | MISO |
| DRDY (47) | GPIO18 | interrupt + contatore hardware PCNT |
| START (38) | GPIO5 | |
| RESET (36) | GPIO4 | |
| PWDN, CLKSEL | +3.3V_D | sempre acceso, oscillatore interno 2.048 MHz |
| CLK | GND | CONFIG1.CLK_EN resta sempre 0 |
| DAISY_IN | GND | |

## Compilare e caricare il firmware

Serve ESP-IDF 5.3 o successivo (verificato con v5.5.6 e v6.1.1).

```
cd ads131e04_uart
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

Non usare `idf.py monitor`: la UART0 trasporta il protocollo binario.

## Usare il client sul PC

```
pip install pyserial

python pc/ads_client.py --port /dev/ttyUSB0 --rate 1000 --duration 5
python pc/ads_client.py --port COM5 --rate 4000 --duration 2 --channels 1,3 --gain 8 --status
python pc/ads_client.py --port /dev/ttyUSB0 --rate 1000 --duration 1 --mux test     # verifica catena
python pc/ads_client.py --port /dev/ttyUSB0 --rate 2000 --duration 0                # fino a Ctrl+C
python pc/ads_client.py --port /dev/ttyUSB0 --info
python pc/ads_client.py --port /dev/ttyUSB0 --regs
```

Il CSV contiene `sample, time_s, CHn_V` (con `--raw` anche i codici, con `--status` la status word). La tensione è quella differenziale INnP − INnN, cioè `Vout_n − VREF`:

V = codice × 2.4 V / G / (2^23 − 1) a 24 bit, oppure / (2^15 − 1) a 16 bit (solo 32 kSPS).

`time_s` usa la frequenza nominale. L'oscillatore interno ha una tolleranza di ±0.5 % a 25 °C e ±2.5 % su tutto il range di temperatura.

## Protocollo

Ogni frame, in entrambe le direzioni:

```
A5 5A | type (1) | len (2, LE) | payload (len) | CRC16 (2, LE)
```

Il CRC è CRC16 CCITT FALSE (poly 0x1021, init 0xFFFF) calcolato su type, len e payload. Gli interi nel payload sono little endian, tranne i campioni ADC, che restano big endian in complemento a due, come escono dall'ADS131E04.

### START (0x02), payload di 16 byte

| off | byte | campo | valori |
|---|---|---|---|
| 0 | 1 | dr | 1 = 32 kSPS (16 bit), 2 = 16k, 3 = 8k, 4 = 4k, 5 = 2k, 6 = 1k (24 bit) |
| 1 | 1 | ch_mask | bit0..bit3 = CH1..CH4 |
| 2 | 4 | gain[4] | 1, 2, 4, 8, 12 |
| 6 | 4 | mux[4] | 0 normale, 1 corto, 3 MVDD, 4 temperatura, 5 segnale di test |
| 10 | 4 | duration_ms | 0 = fino a STOP |
| 14 | 1 | flags | bit0 forza OFFSETCAL, bit1 status word in ogni campione |
| 15 | 1 | test_cfg | bit2 TEST_AMP, bit1:0 TEST_FREQ (00 lento, 01 veloce, 11 continua) |

Esempio, 5 s a 1 kSPS, 4 canali, gain 1, OFFSETCAL:

```
A5 5A 02 10 00 | 06 0F 01 01 01 01 00 00 00 00 88 13 00 00 01 00 | CRC
```

Altri comandi: PING 0x01 (risposta INFO), STOP 0x03, READ_REGS 0x04 (dump registri 0x00..0x14).

### Risposte

* ACK 0x81: `cmd, status` (0 OK, 1 CRC errato, 2 lunghezza, 3 parametro, 4 occupato, 5 banda UART insufficiente, 6 ADC non risponde). Per START OK seguono `sample_rate(4) n_samples(4) ch_mask(1) bytes_per_ch(1) flags(1) samples_per_packet(1) calibrated(1) vref_mV(2)`.
* DATA 0x90: `seq(2) first_sample(4) n(1) ch_mask(1) bytes_per_ch(1) flags(1)` poi n record: `[status 3 byte] + un valore per canale attivo`. I campioni di un frame sono sempre consecutivi; un salto di `first_sample` tra due frame indica campioni persi.
* END 0x91: `reason(1) conversions(4) sent(4) missed(4) dropped(4) status_err(4) packets(4)`, con conversions = sent + missed + dropped.
* INFO 0x82: versione, ID ADC, ADC ok, baud, SPI, VREF, stato. Viene inviato anche spontaneamente all'avvio.

### Combinazioni accettate (controllo di banda fatto dal firmware)

A 921600 baud: 4 canali fino a 4 kSPS, 2 o 3 canali fino a 8 kSPS, 1 canale fino a 32 kSPS. Con la status word: 3 o 4 canali fino a 4 kSPS, 1 o 2 canali fino a 8 kSPS.
A 2 Mbaud (solo con bridge CP2102N, `PC_UART_BAUD` in board.h e `--baud`): 4 canali fino a 8 kSPS, 3 canali fino a 16 kSPS, 1 o 2 canali fino a 32 kSPS.
64 kSPS viene rifiutato: un DRDY ogni 15.6 µs non è gestibile in modo affidabile con interrupt e task.

## Scelte prese dal datasheet (SBAS561C)

* Accensione: attesa alimentazioni e VCAP1, impulso di RESET, 18 tCLK, poi SDATAC, perché il chip si sveglia in RDATAC dove RREG viene ignorato. Ogni scrittura di registro viene riletta e verificata.
* Riferimento interno: CONFIG3 = 0xC0 (buffer acceso, VREF_4V = 0). Con AVDD = 3.3 V il riferimento da 4 V non è ammesso. Attesa di 200 ms per l'assestamento (150 ms tipici).
* SPI modo 1, almeno 4 tCLK (3 µs usati) tra i byte dei comandi multibyte e prima di alzare CS.
* Lettura dati con RDATA in modo SDATAC, non con RDATAC: con RDATA la lettura può sovrapporsi al DRDY successivo senza corrompere i dati, mentre in RDATAC una lettura in ritardo nei 4 tCLK di aggiornamento restituisce dati corrotti.
* I fronti di DRDY sono contati in hardware dal PCNT. Il conteggio letto prima e dopo ogni RDATA dà l'indice esatto della conversione; se durante la lettura arriva una conversione nuova, il campione viene scartato e contato come perso. Un campione consegnato non è quindi mai etichettato male, duplicato o corrotto, qualunque sia la latenza dell'interrupt.
* OFFSETCAL dopo ogni cambio di guadagno (raccomandazione del datasheet) o su richiesta, con START alto perché il chip deve essere in conversione (TI E2E), con attesa di 200 ms (153 ms da datasheet).
* Canali spenti: PD = 1, MUX = corto, come raccomandato.
* Vengono letti solo status e canali fino al più alto attivo: la posizione dei canali nel frame non cambia.
