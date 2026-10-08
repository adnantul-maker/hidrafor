/*
   ============================================================
   HIDROFOR KONTROL SISTEMI - ARDUINO LEONARDO
   SURUM: v5.8

   v5.8 DEGISIKLIKLER (SADECE OLED GORUNUMU, KONTROL MANTIGI AYNI)
   ------------------------------------------------------------
   1. Sag tarafta dikey tank seviye cubugu (25/50/75 isaretli),
      altinda yuzde ve litre.
   2. Pompa baslama/durma geri sayimi: "BASLIYOR 7s", "DURUYOR 4s".
   3. Durum satiri: POMPA CALISIYOR / KULLANICI STOP /
      BEKLEMEDE / MANUEL - KAPALI.
   4. Pompa simgesi: calisirken donen, kapaliyken bos daire.
   5. Alarm ekrani: sebep buyuk yaziyla, altinda RESET gerekli mi
      ve ses durumu.
   Basinc degerleri buyuk yazi olarak korundu.

   v5.7 DEGISIKLIKLER
   ------------------------------------------------------------
   1. Pompa baslatma/durdurma gecikmesi 15 sn -> 10 sn.
   2. Kuru calisma suresi 120 sn -> 60 sn.
   3. Tank olculeri guncellendi: tank yuksekligi 140 cm, su
      en fazla 130 cm, sensor tank tabanindan 150 cm yukarida.
      Dolu mesafe 20 cm (sensorun guvenilir minimumu), bos
      mesafe 150 cm.

   v5.6 DEGISIKLIKLER
   ------------------------------------------------------------
   1. Kuru calisma kilidi (zeroPressureLock) EEPROM'a kaydediliyor.
      Alarm verdikten sonra elektrik kesilip gelse bile AUTO modda
      alarm ekraninda, pompa kapali olarak RESET bekler.
      Diger alarmlar (seviye, samandira) kaydedilmez, kosul
      duzelince kendiliginden kalkar.
      MANUAL moda gecince kilit temizlenir (eskisi gibi).

   v5.5 DEGISIKLIKLER
   ------------------------------------------------------------
   1. STOP durumu (userStopped) EEPROM'a kaydediliyor. Elektrik
      kesilip gelse bile AUTO modda pompa STOP'ta kalir, START
      veya RESET verilene kadar calismaz. EEPROM'a sadece durum
      degisince yazilir (EEPROM.update).
   2. Kuru calisma esigi 0.10 -> 0.30 bar (KURU_CALISMA_BAR).

   v5.3 DEGISIKLIKLER
   ------------------------------------------------------------
   1. userStopped gercekten kullaniliyor (AUTO'da STOP kalici).
   2. Telefon komutlari char tamponu ile bloklamadan okunuyor.
   3. Buton ve AUTO/MANUAL anahtari icin 30 ms debounce.
   4. "$STATUS?" hemen cevap veriyor.
   5. Fiziksel RESET de userStopped'i temizliyor.

   v5.2 DEGISIKLIKLER
   ------------------------------------------------------------
   1. MANUAL modda alarm durumu temizleniyor.
   2. MANUAL'den AUTO'ya donuste eski mute temizleniyor.

   v5.1 DEGISIKLIKLER
   ------------------------------------------------------------
   1. 'updateRelays' syntax hatasi duzeltildi.
   2. Ultrasonik sensor bosluga baktiginda %100 takilma sorunu
      cozuldu.
   3. Timeout ve gecersiz okumalarda tank %0 (BOS) kabul edilir.
   ============================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);


// ============================================================
// PINLER
// ============================================================

// Basinc sensorleri
const byte PIN_SEBEKE    = A0;
const byte PIN_TESISAT   = A1;

// Ultrasonik
const byte PIN_TRIG      = 8;
const byte PIN_ECHO      = 9;

// Dijital samandira
const byte PIN_SAMANDIRA = 10;


// ------------------------------------------------------------
// ROLELER
// ------------------------------------------------------------

const byte PIN_RELAY_PUMP   = 4;   // R1 Pompa
const byte PIN_RELAY_ALARM  = 5;   // R2 Alarm
const byte PIN_RELAY_BUZZER = A2;  // R3 Buzzer
const byte PIN_RELAY_VALVE  = A3;  // R4 Vana / AUX


// ------------------------------------------------------------
// BUTON / ANAHTAR
// ------------------------------------------------------------

const byte PIN_RESET       = 7;
const byte PIN_MUTE        = 11;

const byte PIN_AUTO_MANUAL = 6;

const byte PIN_START       = 12;
const byte PIN_STOP        = 13;


// ============================================================
// ROLE POLARITESI
// ============================================================

#define ROLE_ON  LOW
#define ROLE_OFF HIGH


// ============================================================
// TANK AYARLARI
// ============================================================

const float TANK_YUKSEKLIK_CM = 140.0;
const float TANK_KAPASITE_LITRE = 1500.0;

const float TANK_DOLU_MESAFE_CM = 20.0;   // sensor 150 cm, su 130 cm
const float TANK_BOS_MESAFE_CM  = 150.0;   // sensor -> tank tabani


// ============================================================
// BASINC AYARLARI
// ============================================================

const float SEBEKE_MIN_VOLT = 1.125;
const float SEBEKE_V_BAR    = 0.55;

const float TESISAT_MIN_VOLT = 0.50;
const float TESISAT_VSPAN     = 2.00;
const float TESISAT_MAX_BAR   = 4.00;


// ============================================================
// AUTO KONTROL ESIKLERI
// ============================================================

const float SEBEKE_BASLATMA_BAR = 1.00;
const float SEBEKE_DURDURMA_BAR = 2.00;

const float TESISAT_BASLATMA_BAR = 2.50;
const float TESISAT_DURDURMA_BAR = 3.50;

// Kuru calisma: pompa acikken tesisat basinci bu degerin
// altinda ZERO_PRESSURE_TIME boyunca kalirsa kilitlenir
const float KURU_CALISMA_BAR = 0.30;


// ============================================================
// TANK KORUMASI
// ============================================================

const int TANK_DURDURMA_YUZDE = 20;
const int TANK_BASLATMA_YUZDE = 30;


// ============================================================
// ZAMANLAR
// ============================================================

const unsigned long PUMP_DELAY = 10000UL;       // 10 saniye
const unsigned long ZERO_PRESSURE_TIME = 60000UL;  // 60 saniye
const unsigned long ALARM_DEBOUNCE_TIME = 1000UL;  // 1 saniye
const unsigned long BUTTON_DEBOUNCE_MS = 30UL;     // buton debounce



// ============================================================
// EEPROM
// ============================================================

const int  EEPROM_ADDR_STOP = 0;
const byte EEPROM_STOP_VAL  = 0xA5;   // STOP kayitli
const byte EEPROM_RUN_VAL   = 0x5A;   // STOP yok

const int  EEPROM_ADDR_DRY  = 1;
const byte EEPROM_DRY_VAL   = 0xA5;   // Kuru calisma kilidi kayitli
const byte EEPROM_DRY_OK    = 0x5A;   // Kilit yok


// ============================================================
// SISTEM MODU
// ============================================================

enum SystemMode {
  AUTO,
  MANUAL
};

SystemMode currentMode = AUTO;


// ============================================================
// DURUMLAR
// ============================================================

bool pumpPhysicalState = false;
bool pumpDesiredState  = false;

bool alarmPhysicalState = false;
bool buzzerActiveState  = false;

bool alarmMuted = false;

bool zeroPressureLock   = false;
bool zeroPressureTiming = false;

bool levelAlarmState = false;
bool lowLevelLock    = false;

bool samandiraHata = false;

bool userStopped = false;


// ============================================================
// ZAMANLAYICILAR
// ============================================================

unsigned long pumpStartTimer = 0;
bool pumpStartTiming = false;

unsigned long pumpStopTimer = 0;
bool pumpStopTiming = false;

unsigned long zeroPressureTimer = 0;

unsigned long alarmDebounceTimer = 0;

bool rawAlarmCondition = false;
bool debouncedAlarmState = false;


// ============================================================
// SENSOR DEGERLERI
// ============================================================

float sebekeBar  = 0.0;
float tesisatBar = 0.0;

int tankPercent = 0;
int tankLitre   = 0;


// ============================================================
// TELEFON
// ============================================================

unsigned long sonGonderimZamani = 0;
const unsigned long gonderimPeriyodu = 250UL;

// Komut tamponu (bloklamayan okuma)
const byte CMD_BUF_SIZE = 24;
char cmdBuf[CMD_BUF_SIZE];
byte cmdLen = 0;
bool cmdOverflow = false;


// ============================================================
// OLED ALARM FLAS
// ============================================================

unsigned long oledFlashTimer = 0;
bool oledInvertState = false;


// ============================================================
// BUTON DURUMLARI (DEBOUNCE'LU)
// ============================================================

const byte BTN_RESET = 0;
const byte BTN_MUTE  = 1;
const byte BTN_START = 2;
const byte BTN_STOP  = 3;
const byte BTN_MODE  = 4;
const byte BTN_COUNT = 5;

const byte btnPins[BTN_COUNT] = {
  PIN_RESET,
  PIN_MUTE,
  PIN_START,
  PIN_STOP,
  PIN_AUTO_MANUAL
};

bool btnStable[BTN_COUNT];          // debounce sonrasi kararli durum
bool btnRaw[BTN_COUNT];             // son ham okuma
unsigned long btnTime[BTN_COUNT];   // son degisim zamani



// FONKSIYON PROTOLEPI (Derleyici Hatasi Onleme)
void updateRelays(bool activeAlarm);
void applyPumpState();
void readSensors();
void checkButtons();
void checkPhoneCommands();
void handleCommand(const char* komut);
void processLogic();
void updateOLED();
void sendPhoneTelemetry();
void initButtons();
bool updateButton(byte i);
void setUserStopped(bool v);
void setZeroPressureLock(bool v);
byte remainingSec(unsigned long startTime);
void drawCentered(const char* text, byte y, byte size);
void drawPumpIcon(int cx, int cy);
void drawTankBar();


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  // Sensorler
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_SAMANDIRA, INPUT_PULLUP);

  // Butonlar
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_MUTE, INPUT_PULLUP);
  pinMode(PIN_AUTO_MANUAL, INPUT_PULLUP);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);

  // Roleler
  pinMode(PIN_RELAY_PUMP, OUTPUT);
  pinMode(PIN_RELAY_ALARM, OUTPUT);
  pinMode(PIN_RELAY_BUZZER, OUTPUT);
  pinMode(PIN_RELAY_VALVE, OUTPUT);

  digitalWrite(PIN_RELAY_PUMP, ROLE_OFF);
  digitalWrite(PIN_RELAY_ALARM, ROLE_OFF);
  digitalWrite(PIN_RELAY_BUZZER, ROLE_OFF);
  digitalWrite(PIN_RELAY_VALVE, ROLE_OFF);

  Wire.begin();

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(20, 18);
    display.println("HIDROFOR");
    display.setCursor(35, 34);
    display.println("v5.8");
    display.setCursor(20, 48);
    display.println("BASLATILIYOR");
    display.display();
    delay(1000);
  }

  initButtons();

  currentMode = (btnStable[BTN_MODE] == LOW) ? MANUAL : AUTO;

  // Kayitli STOP durumunu EEPROM'dan oku
  userStopped = (EEPROM.read(EEPROM_ADDR_STOP) == EEPROM_STOP_VAL);

  // Kayitli kuru calisma kilidini EEPROM'dan oku
  zeroPressureLock = (EEPROM.read(EEPROM_ADDR_DRY) == EEPROM_DRY_VAL);

  // MANUAL'de STOP kavrami yok
  if (currentMode == MANUAL) {
    setUserStopped(false);
  }

  readSensors();
  updateRelays(false);
}


// ============================================================
// LOOP
// ============================================================

void loop() {
  readSensors();
  checkButtons();
  checkPhoneCommands();
  processLogic();
  updateOLED();
  sendPhoneTelemetry();
  delay(30);
}



// ============================================================
// STOP DURUMUNU AYARLA (EEPROM'A SADECE DEGISIMDE YAZAR)
// ============================================================

void setUserStopped(bool v) {
  if (userStopped != v) {
    userStopped = v;
    EEPROM.update(EEPROM_ADDR_STOP, v ? EEPROM_STOP_VAL : EEPROM_RUN_VAL);
  }
}


// ============================================================
// KURU CALISMA KILIDINI AYARLA (EEPROM'A SADECE DEGISIMDE YAZAR)
// ============================================================

void setZeroPressureLock(bool v) {
  if (zeroPressureLock != v) {
    zeroPressureLock = v;
    EEPROM.update(EEPROM_ADDR_DRY, v ? EEPROM_DRY_VAL : EEPROM_DRY_OK);
  }
}


// ============================================================
// SEBEKE BASINC OKUMA
// ============================================================

float readSebekePressure() {
  long sum = 0;
  for (byte i = 0; i < 10; i++) {
    sum += analogRead(PIN_SEBEKE);
    delayMicroseconds(50);
  }
  float adc = sum / 10.0;
  float voltage = (adc * 5.0) / 1023.0;
  float bar = (voltage - SEBEKE_MIN_VOLT) / SEBEKE_V_BAR;
  if (bar < 0.0) bar = 0.0;
  return bar;
}


// ============================================================
// TESISAT BASINC OKUMA
// ============================================================

float readTesisatPressure() {
  long sum = 0;
  for (byte i = 0; i < 10; i++) {
    sum += analogRead(PIN_TESISAT);
    delayMicroseconds(50);
  }
  float adc = sum / 10.0;
  float voltage = (adc * 5.0) / 1023.0;
  float bar = ((voltage - TESISAT_MIN_VOLT) / TESISAT_VSPAN) * TESISAT_MAX_BAR;
  if (bar < 0.0) bar = 0.0;
  if (bar > TESISAT_MAX_BAR) bar = TESISAT_MAX_BAR;
  return bar;
}


// ============================================================
// ULTRASONIK MESAFE OKUMA
// ============================================================

float readUltrasonicDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // 30.000 uS timeout (~5 metre)
  unsigned long sure = pulseIn(PIN_ECHO, HIGH, 30000UL);

  if (sure == 0) {
    return 999.0; // Engel yok / Havaya bakiyor
  }

  float mesafe = (sure * 0.0343) / 2.0;

  if (mesafe < 2.0) {
    return TANK_DOLU_MESAFE_CM;
  }

  return mesafe;
}


// ============================================================
// TANK YUZDE HESAPLAMA
// ============================================================

int readTankPercent() {
  float mesafe = readUltrasonicDistance();

  // Tank bossa veya mesafe tank yuksekliginden fazlaysa -> BOS
  if (mesafe >= TANK_BOS_MESAFE_CM || mesafe == 999.0) {
    return 0;
  }

  // Su tam dolu seviyedeyse -> %100
  if (mesafe <= TANK_DOLU_MESAFE_CM) {
    return 100;
  }

  // Linear Yuzde Hesabi
  float yuzde = ((TANK_BOS_MESAFE_CM - mesafe) / (TANK_BOS_MESAFE_CM - TANK_DOLU_MESAFE_CM)) * 100.0;

  if (yuzde < 0.0)   yuzde = 0.0;
  if (yuzde > 100.0) yuzde = 100.0;

  return (int)round(yuzde);
}


// ============================================================
// SENSORLERI OKU
// ============================================================

void readSensors() {
  sebekeBar = readSebekePressure();
  tesisatBar = readTesisatPressure();
  tankPercent = readTankPercent();

  tankLitre = (int)((tankPercent / 100.0) * TANK_KAPASITE_LITRE);

  samandiraHata = (digitalRead(PIN_SAMANDIRA) == HIGH);

  if (currentMode == MANUAL) {
    levelAlarmState = false;
    lowLevelLock = false;
  } else {
    if (samandiraHata) {
      levelAlarmState = true;
    }

    if (tankPercent < TANK_DURDURMA_YUZDE) {
      lowLevelLock = true;
      levelAlarmState = true;
    }

    if (tankPercent >= TANK_BASLATMA_YUZDE) {
      lowLevelLock = false;
    }

    if (!samandiraHata && !lowLevelLock) {
      levelAlarmState = false;
    }
  }
}



// ============================================================
// ANA KONTROL MANTIGI
// ============================================================

void processLogic() {

  // MANUAL MOD
  if (currentMode == MANUAL) {
    zeroPressureTiming = false;
    setZeroPressureLock(false);
    pumpStartTiming = false;
    pumpStopTiming = false;

    // Manuelde alarm yok, durumu temizle
    rawAlarmCondition = false;
    debouncedAlarmState = false;
    alarmDebounceTimer = 0;
    alarmMuted = false;

    applyPumpState();

    digitalWrite(PIN_RELAY_VALVE, ROLE_ON);
    updateRelays(false);
    return;
  }

  // AUTO MOD - KURU CALISMA KONTROLU
  if (pumpPhysicalState && tesisatBar <= KURU_CALISMA_BAR) {
    if (!zeroPressureLock) {
      if (!zeroPressureTiming) {
        zeroPressureTiming = true;
        zeroPressureTimer = millis();
      } else {
        if (millis() - zeroPressureTimer >= ZERO_PRESSURE_TIME) {
          setZeroPressureLock(true);
          zeroPressureTiming = false;
          pumpDesiredState = false;
        }
      }
    }
  } else {
    if (!zeroPressureLock) {
      zeroPressureTiming = false;
    }
  }

  // ALARM DEBOUNCE
  rawAlarmCondition = zeroPressureLock || levelAlarmState;

  if (rawAlarmCondition != debouncedAlarmState) {
    if (alarmDebounceTimer == 0) {
      alarmDebounceTimer = millis();
    } else if (millis() - alarmDebounceTimer >= ALARM_DEBOUNCE_TIME) {
      debouncedAlarmState = rawAlarmCondition;
      alarmDebounceTimer = 0;
    }
  } else {
    alarmDebounceTimer = 0;
  }

  // userStopped aktifse otomatik mantik pompayi baslatmaz
  if (debouncedAlarmState || zeroPressureLock || lowLevelLock || samandiraHata || userStopped) {
    pumpDesiredState = false;
    pumpStartTiming = false;
    pumpStopTiming = false;
  } else {
    // NORMAL AUTO KONTROL
    bool stopCondition = (sebekeBar >= SEBEKE_DURDURMA_BAR) || (tesisatBar >= TESISAT_DURDURMA_BAR);

    if (pumpPhysicalState) {
      pumpStartTiming = false;

      if (stopCondition) {
        if (!pumpStopTiming) {
          pumpStopTiming = true;
          pumpStopTimer = millis();
        } else if (millis() - pumpStopTimer >= PUMP_DELAY) {
          pumpDesiredState = false;
          pumpStopTiming = false;
        }
      } else {
        pumpStopTiming = false;
        pumpDesiredState = true;
      }
    } else {
      pumpStopTiming = false;

      bool startCondition = (sebekeBar < SEBEKE_BASLATMA_BAR) && (tesisatBar <= TESISAT_BASLATMA_BAR);

      if (startCondition) {
        if (!pumpStartTiming) {
          pumpStartTiming = true;
          pumpStartTimer = millis();
        } else if (millis() - pumpStartTimer >= PUMP_DELAY) {
          pumpDesiredState = true;
          pumpStartTiming = false;
        }
      } else {
        pumpStartTiming = false;
        pumpDesiredState = false;
      }
    }
  }

  applyPumpState();

  digitalWrite(PIN_RELAY_VALVE, debouncedAlarmState ? ROLE_OFF : ROLE_ON);

  updateRelays(debouncedAlarmState);
}


// ============================================================
// POMPA ROLE DURUMU
// ============================================================

void applyPumpState() {
  if (pumpPhysicalState != pumpDesiredState) {
    pumpPhysicalState = pumpDesiredState;
    digitalWrite(PIN_RELAY_PUMP, pumpPhysicalState ? ROLE_ON : ROLE_OFF);
  }
}


// ============================================================
// ALARM ROLELERI
// ============================================================

void updateRelays(bool activeAlarm) {
  bool alarmLightDesired = activeAlarm;

  if (alarmPhysicalState != alarmLightDesired) {
    alarmPhysicalState = alarmLightDesired;
    digitalWrite(PIN_RELAY_ALARM, alarmPhysicalState ? ROLE_ON : ROLE_OFF);
  }

  bool buzzerDesired = activeAlarm && !alarmMuted;

  if (buzzerActiveState != buzzerDesired) {
    buzzerActiveState = buzzerDesired;
    digitalWrite(PIN_RELAY_BUZZER, buzzerActiveState ? ROLE_ON : ROLE_OFF);
  }
}



// ============================================================
// BUTON DEBOUNCE
// ============================================================

// Baslangicta tum butonlarin mevcut durumunu kararli kabul et
void initButtons() {
  for (byte i = 0; i < BTN_COUNT; i++) {
    bool v = digitalRead(btnPins[i]);
    btnStable[i] = v;
    btnRaw[i] = v;
    btnTime[i] = millis();
  }
}

// Kararli durumu gunceller. Basma (HIGH->LOW) aninda true doner.
bool updateButton(byte i) {
  bool reading = digitalRead(btnPins[i]);

  if (reading != btnRaw[i]) {
    btnRaw[i] = reading;
    btnTime[i] = millis();
  }

  if ((millis() - btnTime[i] >= BUTTON_DEBOUNCE_MS) && (reading != btnStable[i])) {
    btnStable[i] = reading;
    if (btnStable[i] == LOW) {
      return true;
    }
  }

  return false;
}


// ============================================================
// FIZIKSEL BUTONLAR
// ============================================================

void checkButtons() {
  // Hepsi her dongude guncellenmeli
  bool resetPressed = updateButton(BTN_RESET);
  bool mutePressed  = updateButton(BTN_MUTE);
  bool startPressed = updateButton(BTN_START);
  bool stopPressed  = updateButton(BTN_STOP);
  updateButton(BTN_MODE);

  if (resetPressed) {
    setZeroPressureLock(false);
    zeroPressureTiming = false;
    alarmMuted = false;
    debouncedAlarmState = false;
    alarmDebounceTimer = 0;
    setUserStopped(false);
  }

  if (mutePressed) {
    alarmMuted = true;
  }

  SystemMode newMode = (btnStable[BTN_MODE] == LOW) ? MANUAL : AUTO;

  if (newMode != currentMode) {
    currentMode = newMode;
    pumpStartTiming = false;
    pumpStopTiming = false;
    zeroPressureTiming = false;

    if (currentMode == MANUAL) {
      setUserStopped(false);
    }
  }

  if (startPressed) {
    setUserStopped(false);
    if (currentMode == MANUAL) {
      pumpDesiredState = true;
    }
  }

  if (stopPressed) {
    pumpDesiredState = false;
    pumpStartTiming = false;
    pumpStopTiming = false;

    setUserStopped(currentMode == AUTO);
  }
}


// ============================================================
// TELEFON KOMUTLARI (BLOKLAMAYAN)
// ============================================================

void checkPhoneCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == ' ' || c == '\t') {
      continue;
    }

    if (c == '\n') {
      if (!cmdOverflow && cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
      }
      cmdLen = 0;
      cmdOverflow = false;
    } else if (cmdLen < CMD_BUF_SIZE - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      // Tampon tasti: bu satiri yoksay
      cmdOverflow = true;
    }
  }
}

void handleCommand(const char* komut) {
  if (strcmp(komut, "$PUMP_START") == 0 || strcmp(komut, "PUMP_START") == 0 || strcmp(komut, "START") == 0) {
    setUserStopped(false);
    if (currentMode == MANUAL) {
      pumpDesiredState = true;
    }
  } else if (strcmp(komut, "$PUMP_STOP") == 0 || strcmp(komut, "PUMP_STOP") == 0 || strcmp(komut, "STOP") == 0) {
    pumpDesiredState = false;
    pumpStartTiming = false;
    pumpStopTiming = false;

    setUserStopped(currentMode == AUTO);
  } else if (strcmp(komut, "$ALARM_RST") == 0 || strcmp(komut, "RESET") == 0 || strcmp(komut, "$RESET") == 0) {
    setZeroPressureLock(false);
    zeroPressureTiming = false;
    alarmMuted = false;
    debouncedAlarmState = false;
    alarmDebounceTimer = 0;
    setUserStopped(false);
  } else if (strcmp(komut, "$MUTE") == 0 || strcmp(komut, "MUTE") == 0) {
    alarmMuted = true;
  } else if (strcmp(komut, "$STATUS?") == 0) {
    // Periyodu beklemeden hemen gonder
    sonGonderimZamani = millis() - gonderimPeriyodu;
    sendPhoneTelemetry();
  }
}


// ============================================================
// TELEFONA VERI GONDER
// ============================================================

void sendPhoneTelemetry() {
  if (millis() - sonGonderimZamani >= gonderimPeriyodu) {
    sonGonderimZamani = millis();

    Serial.print("$HYDRO");
    Serial.print(",P1="); Serial.print(tesisatBar, 2);
    Serial.print(",P2="); Serial.print(sebekeBar, 2);
    Serial.print(",LVL="); Serial.print(tankLitre);
    Serial.print(",LVL_PCT="); Serial.print(tankPercent);
    Serial.print(",PUMP="); Serial.print(pumpPhysicalState ? 1 : 0);
    Serial.print(",MODE="); Serial.print(currentMode == AUTO ? "AUTO" : "MANUAL");
    Serial.print(",ALM_DRY="); Serial.print(zeroPressureLock ? 1 : 0);
    Serial.print(",ALM_LOW="); Serial.print(lowLevelLock ? 1 : 0);
    Serial.print(",ALM_FLT="); Serial.print(samandiraHata ? 1 : 0);
    Serial.print(",ALM_MUTE="); Serial.print(alarmMuted ? 1 : 0);
    Serial.print(",USER_STOP="); Serial.print(userStopped ? 1 : 0);
    Serial.print("\r\n");
    Serial.flush();
  }
}



// ============================================================
// OLED YARDIMCI FONKSIYONLAR
// ============================================================

// Gecikme sayacinda kalan saniye (yukari yuvarlar)
byte remainingSec(unsigned long startTime) {
  unsigned long elapsed = millis() - startTime;
  if (elapsed >= PUMP_DELAY) return 0;
  return (byte)(((PUMP_DELAY - elapsed) + 999UL) / 1000UL);
}

// Yaziyi ekranda ortalar
void drawCentered(const char* text, byte y, byte size) {
  display.setTextSize(size);
  int w = strlen(text) * 6 * size;
  int x = (SCREEN_WIDTH - w) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.print(text);
}

// Pompa simgesi: calisirken donen cizgi, kapaliyken bos daire
void drawPumpIcon(int cx, int cy) {
  display.drawCircle(cx, cy, 4, SSD1306_WHITE);

  if (pumpPhysicalState) {
    switch ((byte)((millis() / 150UL) % 4UL)) {
      case 0:
        display.drawLine(cx - 3, cy, cx + 3, cy, SSD1306_WHITE);
        break;
      case 1:
        display.drawLine(cx - 2, cy - 2, cx + 2, cy + 2, SSD1306_WHITE);
        break;
      case 2:
        display.drawLine(cx, cy - 3, cx, cy + 3, SSD1306_WHITE);
        break;
      default:
        display.drawLine(cx + 2, cy - 2, cx - 2, cy + 2, SSD1306_WHITE);
        break;
    }
  }
}

// Sag tarafta dikey tank cubugu + yuzde + litre
void drawTankBar() {
  // Dis cerceve (x 102..127, y 0..43), ic alan 22x40
  display.drawRect(102, 0, 26, 44, SSD1306_WHITE);

  int fillH = (tankPercent * 40) / 100;
  if (fillH > 0) {
    display.fillRect(104, 42 - fillH, 22, fillH, SSD1306_WHITE);
  }

  // 25 / 50 / 75 isaretleri
  display.drawFastHLine(98, 32, 4, SSD1306_WHITE);
  display.drawFastHLine(98, 22, 4, SSD1306_WHITE);
  display.drawFastHLine(98, 12, 4, SSD1306_WHITE);

  char buf[8];

  display.setTextSize(1);

  snprintf(buf, sizeof(buf), "%d%%", tankPercent);
  display.setCursor(128 - (int)strlen(buf) * 6, 47);
  display.print(buf);

  snprintf(buf, sizeof(buf), "%dL", tankLitre);
  display.setCursor(128 - (int)strlen(buf) * 6, 56);
  display.print(buf);
}


// ============================================================
// OLED EKRAN
// ============================================================

void updateOLED() {
  display.clearDisplay();

  // ---------------- ALARM EKRANI ----------------
  if (debouncedAlarmState) {
    if (millis() - oledFlashTimer >= 800) {
      oledFlashTimer = millis();
      oledInvertState = !oledInvertState;
    }

    display.invertDisplay(oledInvertState);
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
    display.drawRect(2, 2, 124, 60, SSD1306_WHITE);

    const char* l1   = "ALARM";
    const char* l2   = "";
    const char* hint = "OTOMATIK KALKAR";

    if (zeroPressureLock) {
      l1 = "KURU";
      l2 = "CALISMA";
      hint = "RESET GEREKLI";
    } else if (samandiraHata) {
      l1 = "SAMANDIRA";
      l2 = "HATASI";
    } else if (lowLevelLock) {
      l1 = "DUSUK";
      l2 = "SEVIYE";
    }

    drawCentered(l1, 6, 2);
    if (l2[0] != '\0') {
      drawCentered(l2, 24, 2);
    }
    drawCentered(hint, 43, 1);
    drawCentered(alarmMuted ? "SES: SUSTURULDU" : "SES: AKTIF", 52, 1);

    display.display();
    return;
  }

  // ---------------- ANA EKRAN ----------------
  display.invertDisplay(false);
  display.setTextSize(1);

  // Ust satir: mod + pompa simgesi
  display.setCursor(0, 0);
  display.print(currentMode == AUTO ? "[AUTO]" : "[MAN]");

  drawPumpIcon(52, 4);

  display.setTextSize(1);
  display.setCursor(60, 0);
  display.print("POMPA");

  display.drawFastHLine(0, 10, 96, SSD1306_WHITE);

  // Sebeke basinci (sol blok)
  display.setCursor(0, 14);
  display.print("SEBEKE");
  display.setCursor(0, 25);
  display.setTextSize(2);
  display.print(sebekeBar, 1);
  display.setTextSize(1);
  display.print("b");

  // Ayirici
  display.drawFastVLine(49, 13, 30, SSD1306_WHITE);

  // Tesisat basinci (sag blok)
  display.setCursor(52, 14);
  display.print("TESISAT");
  display.setCursor(52, 25);
  display.setTextSize(2);
  display.print(tesisatBar, 1);
  display.setTextSize(1);
  display.print("b");

  display.drawFastHLine(0, 44, 96, SSD1306_WHITE);

  // Durum satiri (sol alan 96 px icinde ortali)
  char sbuf[20];

  if (pumpPhysicalState) {
    if (pumpStopTiming) {
      snprintf(sbuf, sizeof(sbuf), "DURUYOR %ds", (int)remainingSec(pumpStopTimer));
    } else {
      snprintf(sbuf, sizeof(sbuf), "POMPA CALISIYOR");
    }
  } else if (userStopped && currentMode == AUTO) {
    snprintf(sbuf, sizeof(sbuf), "KULLANICI STOP");
  } else if (pumpStartTiming) {
    snprintf(sbuf, sizeof(sbuf), "BASLIYOR %ds", (int)remainingSec(pumpStartTimer));
  } else if (currentMode == MANUAL) {
    snprintf(sbuf, sizeof(sbuf), "MANUEL - KAPALI");
  } else {
    snprintf(sbuf, sizeof(sbuf), "BEKLEMEDE");
  }

  int sx = (96 - (int)strlen(sbuf) * 6) / 2;
  if (sx < 0) sx = 0;
  display.setCursor(sx, 52);
  display.print(sbuf);

  // Sag tarafta dikey tank cubugu
  drawTankBar();

  display.display();
}

