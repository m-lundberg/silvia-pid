#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MenuSystem.h>
#include <max6675.h>

// EEPROM
#define CONFIG_VERSION "sip"
#define CONFIG_START 32

// Settings
struct StoreStruct {
    int brewSetpoint, steamSetpoint;
    double p, i, d;
    char version[4];
}
settings = {
    100, 140,
    40, 15, 10,
    CONFIG_VERSION
};

// LCD
// Pin A4 = SDA
// Pin A5 = SCL
#define I2C_ADDR        0x27
#define BACKLIGHT_PIN   3
#define En_pin          2
#define Rw_pin          1
#define Rs_pin          0
#define D4_pin          4
#define D5_pin          5
#define D6_pin          6
#define D7_pin          7
bool inMenu = false;
uint8_t degreeChar[8] = {0x8,0xf4,0x8,0x43,0x4,0x4,0x43,0x0};
uint8_t cupChar[8] = {0x0,0x4,0x8,0x1f,0xf2,0xc,0x1e,0x0};
uint8_t steamChar[8] = {0x0,0xa,0xa,0xa,0x0,0x15,0xa,0x0};
LiquidCrystal_I2C lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);

// Menu System
MenuSystem ms;
Menu rootMenu("Settings");
Menu setpointMenu("Setpoints");
MenuItem brewItem("Brew setpoint");
MenuItem steamItem("Steam setpoint");
Menu pidMenu("PID Settings");
MenuItem pItem("P Value");
MenuItem iItem("I Value");
MenuItem dItem("D Value");
Menu tuneMenu("Autotune");
MenuItem tuneItem("Activate");
MenuItem cancelItem("Cancel");
MenuItem backItem("Back");

// Rotary encoder
const int rotaryA = 2;
const int rotaryB = 3;
const int rotaryButton = 8;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;

// Thermocouple
const int thermoDO = 4;   // num. 3
const int thermoCS = 5;   // num. 2
const int thermoCLK = 6;  // num. 1
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Temperature data
const int numReadings = 5;
double temps[numReadings];
int tempIndex = 0;
double totalTemp = 0.0;
double averageTemp = 0.0;
unsigned long lastTempRead = 0;

// SSR
const int SSR = 7;

// Switches
const int steamSwitch = 9;
const int brewSwitch = 10;

// PID
double setpoint;
double input, output;
PID pid(&input, &output, &setpoint, settings.p, settings.i, settings.d, REVERSE);
const int windowSize = 1000;
unsigned long windowStartTime;

//PID Autotune
bool tuning = false;
PID_ATune tuner(&input, &output);

void saveConfig() {
    for (unsigned int t = 0; t < sizeof(settings); ++t) {
        EEPROM.update(CONFIG_START + t, *((char*)&settings + t));
    }
}

void loadConfig() {
    if (EEPROM.read(CONFIG_START + sizeof(settings) - 2) == settings.version[2] &&
        EEPROM.read(CONFIG_START + sizeof(settings) - 3) == settings.version[1] &&
        EEPROM.read(CONFIG_START + sizeof(settings) - 4) == settings.version[0]) { 
        for (unsigned int t = 0; t < sizeof(settings); ++t) {
            *((char*)&settings + t) = EEPROM.read(CONFIG_START + t);
        }
    } else {
        // settings invalid, overwrite with default
        saveConfig();
    }
}

void setup() {
    Serial.begin(115200);
    loadConfig();
    
    windowStartTime = millis();
    setpoint = settings.brewSetpoint;

    pid.SetTunings(settings.p, settings.i, settings.d);
    pid.SetOutputLimits(0, windowSize);
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(250); // set pid to compute new output every 250 ms to allow temp readings to happen
    input = thermocouple.readCelsius();
    
    pinMode(SSR, OUTPUT);
    
    lcd.begin(16, 2);
    lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
    lcd.setBacklight(HIGH);
    lcd.home();
    lcd.print(F("Silvia PID"));
    lcd.createChar(0, degreeChar);
    lcd.createChar(1, cupChar);
    lcd.createChar(2, steamChar);
    
    digitalWrite(rotaryA, HIGH);  // enable pull-up resistors for inputs
    digitalWrite(rotaryB, HIGH);
    digitalWrite(rotaryButton, HIGH);
    attachInterrupt(digitalPinToInterrupt(rotaryA), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rotaryB), updateEncoder, CHANGE);
    
    rootMenu.add_menu(&setpointMenu);
    setpointMenu.add_item(&brewItem, &onSelected);
    setpointMenu.add_item(&steamItem, &onSelected);
    setpointMenu.add_item(&backItem, &onSelected);
    rootMenu.add_menu(&pidMenu);
    pidMenu.add_item(&pItem, &onSelected);
    pidMenu.add_item(&iItem, &onSelected);
    pidMenu.add_item(&dItem, &onSelected);
    pidMenu.add_item(&backItem, &onSelected);
    rootMenu.add_menu(&tuneMenu);
    tuneMenu.add_item(&tuneItem, &onSelected);
    tuneMenu.add_item(&cancelItem, &onSelected);
    tuneMenu.add_item(&backItem, &onSelected);
    rootMenu.add_item(&backItem, &onSelected);
    ms.set_root_menu(&rootMenu);
    
    delay(500);
    lcd.clear();
}

void loop() {
    // Brew timer
    static unsigned long timerStart = 0;
    static unsigned long timerSaved = 0;
    static bool timerGoing = false;
    if (timerStart != 0) {
        if (timerGoing) {
            timerSaved = millis()/1000 - timerStart/1000;
        }
        String timerStr = String(timerSaved);
        lcd.setCursor(16 - timerStr.length(), 0);
        lcd.print(timerStr);
    }
    
    // Check if brew switch is pressed
    if (digitalRead(brewSwitch) && !timerGoing) {
        // Start timer
        timerGoing = true;
        timerStart = millis();
    } else if (!digitalRead(brewSwitch)) {
        timerGoing = false;
        timerStart = 0;
    }
    
    // Handle rotary encoder rotation
    byte rotation = encoderRotated();
    if (rotation == 2) {
        // rotated right
        if (inMenu) {
            ms.next();
            displayMenu();
        } else {
            Serial.println("Temp:" + String(input));
            if (digitalRead(brewSwitch)) Serial.println("Brewing");
            lcd.begin(16, 2);
            lcd.clear();
        }
    } else if (rotation == 1) {
        // rotated left
        if (inMenu) {
            ms.prev();
            displayMenu();
        }
    }
    
    // Handle rotary button
    if (buttonPressed()) {
        // button pressed and debounced
        if (!inMenu) {
            inMenu = true;
            displayMenu();
        } else {
            ms.select();
            displayMenu();
        }
    }

    // Update LCD
    static unsigned long lastUpdateLCD = 0;
    static bool lastInMenu = false;
    if (!inMenu && millis() - lastUpdateLCD > 440) {
        if (lastInMenu) lcd.clear();
        lastInMenu = false;

        // print brew or steam indicator
        lcd.setCursor(0, 0);
        if (digitalRead(steamSwitch)) {
            lcd.print((char)2);
        } else {
            lcd.print((char)1);
        }

        lcd.print(' ');
        lcd.print((int)input);
        //lcd.print("100");
        lcd.print((char)0);
        lcd.print('/');
        lcd.print((int)setpoint);
        lcd.print((char)0);
        lcd.print("  ");

        if (tuning) {
            lcd.setCursor(0, 1);
            lcd.print(F("Tuning PID"));
        } else {
            lcd.setCursor(0, 1);
            lcd.print(F("          "));
        }
        
        lastUpdateLCD = millis();
    } else if (inMenu && !lastInMenu) {
        // just switched to menu, display it
        lastInMenu = true;
        displayMenu();
    }

    // Read tempereature data
    input = readTemp();
    //Serial.println("temp = " + String(input));

    // Check brew/steam mode and set setpoint
    if (digitalRead(steamSwitch)) {
      setpoint = settings.steamSetpoint;
    } else {
      setpoint = settings.brewSetpoint;
    }

    // Perform PID autotune
    if (tuning) {
        if (tuner.Runtime() != 0) {
            // Tuning complete
            tuning = false;
            settings.p = tuner.GetKp();
            settings.i = tuner.GetKi();
            settings.d = tuner.GetKd();
            pid.SetTunings(settings.p, settings.i, settings.d);
            pid.Compute();
        }
    } else {
        // Compute new PID output
        pid.Compute();
    }

    // Control SSR using time proporional control
    unsigned long now = millis();
    if (now - windowStartTime > windowSize) {
        windowStartTime += windowSize;
    }
    if (output < now - windowStartTime) {
        digitalWrite(SSR, HIGH);
    } else {
        digitalWrite(SSR, LOW);
    }
}

double readTemp() {
    // Check if enough time has passed to do a new reading from chip
    if (millis()-lastTempRead <= 220) return averageTemp;
    
    double temp = thermocouple.readCelsius();
    lastTempRead = millis();

    totalTemp = (totalTemp + temp) - temps[tempIndex];
    temps[tempIndex] = temp;
    tempIndex++;
    if (tempIndex >= numReadings) tempIndex = 0;
    averageTemp = totalTemp / numReadings;

    return averageTemp;
}

void onSelected(MenuItem* item) {
    if (item->get_name() == backItem.get_name()) {
        if (ms.get_current_menu()->get_name() == "Settings") {
            // in root menu, exit menu
            inMenu = false;
        } else {
            // in sub menu, go back
            ms.back();
        }
    } else if (item->get_name() == brewItem.get_name()) { 
        lcd.clear();
        lcd.home();
        lcd.print(item->get_name());
        lcd.setCursor(0, 1);
        lcd.print(settings.brewSetpoint);
        
        while (true) {
            byte rotation = encoderRotated();
            if (rotation == 2) {
                // rotated right
                if (settings.brewSetpoint < 150) {
                    settings.brewSetpoint++;
                    lcd.setCursor(0, 1);
                    lcd.print(settings.brewSetpoint);
                    lcd.print("    ");
                }
            } else if (rotation == 1) {
                // rotated left
                if (settings.brewSetpoint > 0) {
                    settings.brewSetpoint--;
                    lcd.setCursor(0, 1);
                    lcd.print(settings.brewSetpoint);
                    lcd.print("    ");
                }
            }
            
            if (buttonPressed()) {
                // button pressed and debounced
                setpoint = settings.brewSetpoint;        // testing only
                break;
            }
        }
    } else if (item->get_name() == steamItem.get_name()) { 
        lcd.clear();
        lcd.home();
        lcd.print(item->get_name());
        lcd.setCursor(0, 1);
        lcd.print(settings.steamSetpoint);
        
        while (true) {
            byte rotation = encoderRotated();
            if (rotation == 2) {
                // rotated right
                if (settings.steamSetpoint < 150) {
                    settings.steamSetpoint++;
                    lcd.setCursor(0, 1);
                    lcd.print(settings.steamSetpoint);
                    lcd.print("    ");
                }
            } else if (rotation == 1) {
                // rotated left
                if (settings.steamSetpoint > 0) {
                    settings.steamSetpoint--;
                    lcd.setCursor(0, 1);
                    lcd.print(settings.steamSetpoint);
                    lcd.print("    ");
                }
            }
            
            if (buttonPressed()) {
                // button pressed and debounced
                break;
            }
        }
    } else if (item->get_name() == pItem.get_name()) {
        lcd.clear();
        lcd.home();
        lcd.print(item->get_name());
        lcd.setCursor(0, 1);
        lcd.print(settings.p);
        
        while (true) {
            byte rotation = encoderRotated();
            if (rotation == 2) {
                // rotated right
                settings.p += 0.1;
                lcd.setCursor(0, 1);
                lcd.print(settings.p);
                lcd.print("    ");
            } else if (rotation == 1) {
                // rotated left
                if (settings.p > 0) {
                    settings.p -= 0.1;
                    lcd.setCursor(0, 1);
                    lcd.print(settings.p);
                    lcd.print("    ");
                }
            }
            
            if (buttonPressed()) {
                // button pressed and debounced
                pid.SetTunings(settings.p, settings.i, settings.d);
                break;
            }
        }
    } else if (item->get_name() == iItem.get_name()) {
        lcd.clear();
        lcd.home();
        lcd.print(item->get_name());
        lcd.setCursor(0, 1);
        lcd.print(settings.i);
        
        while (true) {
            byte rotation = encoderRotated();
            if (rotation == 2) {
                // rotated right
                settings.i += 0.1;
                lcd.setCursor(0, 1);
                lcd.print(settings.i);
                lcd.print("    ");
            } else if (rotation == 1) {
                // rotated left
                if (settings.i > 0) {
                    settings.i -= 0.1;
                    lcd.setCursor(0, 1);
                    lcd.print(settings.i);
                    lcd.print("    ");
                }
            }
            
            if (buttonPressed()) {
                // button pressed and debounced
                pid.SetTunings(settings.p, settings.i, settings.d);
                break;
            }
        }
    } else if (item->get_name() == dItem.get_name()) {
        lcd.clear();
        lcd.home();
        lcd.print(item->get_name());
        lcd.setCursor(0, 1);
        lcd.print(settings.d);
        
        while (true) {
            byte rotation = encoderRotated();
            if (rotation == 2) {
                // rotated right
                settings.d += 0.1;
                lcd.setCursor(0, 1);
                lcd.print(settings.d);
                lcd.print("    ");
            } else if (rotation == 1) {
                // rotated left
                if (settings.d > 0) {
                    settings.d -= 0.1;
                    lcd.setCursor(0, 1);
                    lcd.print(settings.d);
                    lcd.print("    ");
                }
            }
            
            if (buttonPressed()) {
                // button pressed and debounced
                pid.SetTunings(settings.p, settings.i, settings.d);
                break;
            }
        }
    } else if (item->get_name() == tuneItem.get_name()) {
        lcd.clear();

        tuning = true;
    } else if (item->get_name() == cancelItem.get_name()) {
        lcd.clear();

        tuner.Cancel();
        tuning = false;
    }
    saveConfig();
}

void displayMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    const Menu* current = ms.get_current_menu();
    
    lcd.print(current->get_name());
    
    lcd.setCursor(0, 1);
    lcd.print(current->get_selected()->get_name());
}

// ISR for rotary encoder
void updateEncoder() {
    int MSB = digitalRead(rotaryA); //MSB = most significant bit
    int LSB = digitalRead(rotaryB); //LSB = least significant bit
    
    int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
    
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
    
    lastEncoded = encoded; //store this value for next time
}

// Returns 1 for left, 2 for right and 0 for no rotation
byte encoderRotated() {
    static long lastEncoderValue = 0;
    if (encoderValue/4 > lastEncoderValue) {
        lastEncoderValue = encoderValue/4;
        return 2;
    } else if (encoderValue/4 < lastEncoderValue) {
        lastEncoderValue = encoderValue/4;
        return 1;
    }
    return 0;
}

bool buttonPressed() {
    static bool pressable = true;
    static unsigned long lastPress;
    
    if (!digitalRead(rotaryButton) && pressable && millis() - lastPress > 200) {
        pressable = false;
        lastPress = millis();
        return true;
    } else if (digitalRead(rotaryButton)) {
        pressable = true;
        return false;
    }
}
