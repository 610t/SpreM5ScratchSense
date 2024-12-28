#include <GNSS.h>

#define STRING_BUFFER_SIZE 128
#define RESTART_CYCLE (60 * 5)
static SpGnss Gnss;

void setup() {
  int result;

  Serial.begin(115200);
  Serial2.begin(115200);  // Connect to M5Stack via Serial2.

  ledOn(PIN_LED0);  // Turn on LED0 :Setup start.

  Gnss.setDebugMode(PrintInfo);  // Set Debug mode to Info

  result = Gnss.begin();

  if (result != 0) {
    Serial.println("Gnss begin error!!");
    ledOff(PIN_LED2);
    exit(0);
  }

  // Select all GPS mode.
  Gnss.select(GPS);
  Gnss.select(GLONASS);
  Gnss.select(QZ_L1CA);

  // Start positioning
  result = Gnss.start(COLD_START);
  if (result != 0) {
    Serial.println("Gnss start error!!");
    ledOff(PIN_LED3);
    exit(0);
  }

  ledOff(PIN_LED0);  /// Turn off LED0 :Setup done.
}

static void print_with_debug(char *strbuf) {
  Serial.print(strbuf);
  Serial2.print(strbuf);
}

static void print_pos(SpNavData *pNavData) {
  char StringBuffer[STRING_BUFFER_SIZE];

  // print date & time via Serial2
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "Date:%04d/%02d/%02d\n", pNavData->time.year, pNavData->time.month, pNavData->time.day);
  print_with_debug(StringBuffer);

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "Time:%02d:%02d:%02d.%06ld\n", pNavData->time.hour, pNavData->time.minute, pNavData->time.sec, pNavData->time.usec);
  print_with_debug(StringBuffer);

  // print satellites count
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d\n", pNavData->numSatellites);
  print_with_debug(StringBuffer);

  // print position data
  if (pNavData->posFixMode == FixInvalid) {
    Serial.printf("Fix:No-Fix\n");
  } else {
    Serial.printf("Fix:Fix\n");
  }
  if (pNavData->posDataExist == 0) {
    Serial.print("Post:No Position\n");
  } else {
    snprintf(StringBuffer, STRING_BUFFER_SIZE, "Lat:%f\nLon:%f\n", pNavData->latitude, pNavData->longitude);
    print_with_debug(StringBuffer);
  }
}

static void print_condition(SpNavData *pNavData) {
  char StringBuffer[STRING_BUFFER_SIZE];
  unsigned long cnt;

  // Print satellite count.
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d\n", pNavData->numSatellites);
  print_with_debug(StringBuffer);

  for (cnt = 0; cnt < pNavData->numSatellites; cnt++) {
    const char *pType = "GPS";
    SpSatelliteType sattype = pNavData->getSatelliteType(cnt);

    // Get print conditions.
    unsigned long Id = pNavData->getSatelliteId(cnt);
    unsigned long Elv = pNavData->getSatelliteElevation(cnt);
    unsigned long Azm = pNavData->getSatelliteAzimuth(cnt);
    float sigLevel = pNavData->getSatelliteSignalLevel(cnt);

    // Print satellite condition.
    snprintf(StringBuffer, STRING_BUFFER_SIZE, "[%2ld] Type:%s, Id:%2ld, Elv:%2ld, Azm:%3ld, CN0:", cnt, pType, Id, Elv, Azm);
    Serial.print(StringBuffer);
    Serial.println(sigLevel, 6);
  }
}

void loop() {
  static int LoopCount = 0;
  static int LastPrintMin = 0;

  // Check update.
  if (Gnss.waitUpdate(-1)) {
    // Get NaviData.
    SpNavData NavData;
    Gnss.getNavData(&NavData);

    // Print satellite information to Serial every minute.
    if (NavData.time.minute != LastPrintMin) {
      print_condition(&NavData);
      LastPrintMin = NavData.time.minute;
    }

    // Send position information via Serial2
    print_pos(&NavData);
  } else {
    // Not update.
    Serial.println("data not update");
  }

  // Check loop count.
  LoopCount++;
  if (LoopCount >= RESTART_CYCLE) {
    int error_flag = 0;

    // Restart GNSS.
    if (Gnss.stop() != 0) {
      Serial.println("Gnss stop error!!");
      error_flag = 1;
    } else if (Gnss.end() != 0) {
      Serial.println("Gnss end error!!");
      error_flag = 1;
    } else {
      Serial.println("Gnss stop OK.");
    }

    if (Gnss.begin() != 0) {
      Serial.println("Gnss begin error!!");
      error_flag = 1;
    } else if (Gnss.start(HOT_START) != 0) {
      Serial.println("Gnss start error!!");
      error_flag = 1;
    } else {
      Serial.println("Gnss restart OK.");
    }

    // If error on LED3 and halt.
    if (error_flag == 1) {
      ledOn(PIN_LED3);
      exit(0);
    }

    LoopCount = 0;
  }
}
