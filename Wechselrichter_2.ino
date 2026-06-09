//**************************************************************************************************
//GaN-Wechselrichter Source-Code
//by Moritz Rambold 2026
//thetrashinventor.de
//**************************************************************************************************

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>
#include <math.h>
#include "driver/mcpwm.h"
#include <ESP32RotaryEncoder.h>

#define configGENERATE_RUN_TIME_STATS 1 //for calculation CPU-Stats
//Parameter

#define TABLE_SIZE 720  //size of sine-lookuptable
#define UPDATE_RATE (SINE_FREQ * TABLE_SIZE)

//Display
#define SCREEN_WIDTH 132  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//RANDNOTIZ:
//Display CH1116-Driver -> deklariert als 128x64 eigentlich aber physisch 132x64
//FIX: in Adafruit_SH110X.cpp -> page_start_offset auf 0 ändern

//Pins
#define PHASE_U 14
#define PHASE_V 12
#define PHASE_W 13

#define OLED_SDA 21
#define OLED_SCL 22

#define BUTTON_UP 34
#define BUTTON_DOWN 35
#define BUTTON_LEFT 32
#define BUTTON_RIGHT 33

#define ENCODER_SW 27
#define ENCODER_A 26
#define ENCODER_B 25

#define I1_PIN 2
#define I2_PIN 4

//variables
float sin_table[TABLE_SIZE];
uint32_t PWM_FREQ = 100000;   // PWM carrier frequency (100 kHz)
uint32_t SINE_FREQ = 50;      // Output sine frequency
float I1 = 0;                 //Stromsensor U -> V
float I2 = 0;                 //Stromsensor V -> W
int contrast = 255;           //Display-Kontrast
float core0 = 0.0;            //Kern-Auslastung Kern 0 in %
float core1 = 0.0;  	        //Kern-Auslastung Kern 1 in %

// MENU TYPES (needed for rendering)
enum VarType {
  TYPE_UINT32,
  TYPE_INT,
  TYPE_FLOAT
};

struct MenuItem {
  const char* label;    //Displayed name
  void* variable;       //pointer to variable to be displayed
  VarType type;         //Variable-Type (uint32_t, int, float)
  const char* unit;     //Displayed unit of value (e.g. current -> "A")
  bool selectable;      //true -> value can be selected to be changed
  bool isPageSwitch;    //true -> selecting item changes the page 
  uint8_t targetPage;   //page to be changed to
};

struct MenuPage {
  const char* header;   //Displayed name of page
  MenuItem* items;      //pointer to displayed items
  uint8_t itemCount;  	//number of displayed items on page
};

// MENU ITEMS
//Homepage
MenuItem homeItems[] = {
  { "f-PWM", &PWM_FREQ, TYPE_UINT32, "Hz", true, false, 0 },
  { "f-SIN", &SINE_FREQ, TYPE_UINT32, "Hz", true, false, 0 },
  { "I1", &I1, TYPE_FLOAT, "A", false, false, 0 },
  { "I2", &I2, TYPE_FLOAT, "A", false, false, 0 },
  { "SETTINGS", nullptr, TYPE_INT, "", true, true, 1 }
};

//Settingspage
MenuItem settingsItems[] = {
  { "Contrast", &contrast, TYPE_INT, "", true, false, 1 },
  { "CPU0", &core0, TYPE_FLOAT, "%", false, false, 0 },
  { "CPU1", &core1, TYPE_FLOAT, "%", false, false, 0 },
  { "BACK", nullptr, TYPE_INT, "", true, true, 0 }
};

//Pages
MenuPage pages[] = {
  { "HOME", homeItems, 5 },
  { "SETTINGS", settingsItems, 4 }
};

// STATE
volatile int8_t encoderDelta = 0;
volatile uint8_t lastA = 0;
uint8_t currentPage = 0;
uint8_t selectedItem = 0;
bool editMode = false;

// shared selected digit
// 0 = ones
// 1 = tens
// 2 = hundreds
int selectedDigit = 0;

// BUTTON STATES
bool lastUp = false;
bool lastDown = false;
bool lastLeft = false;
bool lastRight = false;
bool lastEncoderButton = false;

//Handles
TaskHandle_t UI;            //Handle for UI-Task
TaskHandle_t SPWM_GEN;      //Handle for SPWM-Generation-Task

//Prototypes
void init_sin_table();                                //Calculate Sine-Lookuptable
void init_mcpwm();                                    //Initialize 3-Phase-SPWM Generation
void SetupDisplay();                                  //Initialize OLED-Display
float GetCurrent(int);                                //Calculate Current-value from ADC
void IRAM_ATTR encoderISR();                          //ISR for rotary-encoder
float getStep(MenuItem&);                             //Menusystem: Get stepsize for changing selected value
void changeValue(MenuItem&, int);                     //Menusystem: Change selected value
void drawEditableValue(int, int, MenuItem&, bool);  	//Menusystem: format string for displaying editable value on screen
void moveSelection(int);                              //Menusystem: set Cursor to selected item
void activateItem();                                  //Menusystem: Display activation symbol if value in editmode
void draw();                                          //Menusystem: Rendering of Menu
void updateMenu();                                    //Menusystem: Update Menu
void update_mcpwm_frequency();                        //Reinitialize 3-Phase-SPWM when carrierfrequency changed via menu
float getCoreUsagePercent(BaseType_t coreID);         //Calculate the Usage of the CPU-cores

// 'HS_logo', 64x64px
static const unsigned char PROGMEM HS_logo[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x06, 0x01, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x04, 0x00, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x7e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x7e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x00, 0xc0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x01, 0xc0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x03, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x03, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x3e, 
	0x76, 0x8c, 0xb6, 0x65, 0xad, 0x71, 0xe0, 0x3e, 0x76, 0xab, 0xb5, 0xdd, 0xad, 0x77, 0xe0, 0x3e, 
	0x70, 0xab, 0x86, 0xdc, 0x2d, 0x73, 0xe0, 0x3e, 0x76, 0xab, 0xb7, 0x5d, 0xad, 0x77, 0xe0, 0x3e, 
	0x76, 0x8c, 0xb4, 0xe5, 0xb3, 0x11, 0xe0, 0x3e, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x3e, 0x77, 0x8d, 0xb1, 0xc6, 0xd6, 0x83, 0xe0, 0x3e, 
	0x77, 0xac, 0xb6, 0xbe, 0xd6, 0xef, 0xe0, 0x3e, 0x77, 0x8d, 0x36, 0xce, 0x16, 0xef, 0xe0, 0x3e, 
	0x77, 0x25, 0xb6, 0xf6, 0xd6, 0xef, 0xe0, 0x3e, 0x71, 0x75, 0xb1, 0x8e, 0xd9, 0xef, 0xe0, 0x3e, 
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1e, 
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


void setup() {
  Serial.begin(115200);

  pinMode(PHASE_U, OUTPUT);
  pinMode(PHASE_V, OUTPUT);
  pinMode(PHASE_W, OUTPUT);

  // ENCODER
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  // BUTTONS
  pinMode(BUTTON_UP, INPUT);
  pinMode(BUTTON_DOWN, INPUT);
  pinMode(BUTTON_LEFT, INPUT);
  pinMode(BUTTON_RIGHT, INPUT);
  
  //Create 3-Phase SPWM-Generator Task on core 1
  xTaskCreatePinnedToCore(Generate_SPWM, "3-SPWM-Generator", 10000, NULL, 1, &SPWM_GEN, 1);

  init_sin_table();
  init_mcpwm();
  analogSetAttenuation(ADC_11db);

  // ENCODER INTERRUPT
  lastA = digitalRead(ENCODER_A);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
        
  // I2C
  Wire.begin(21, 22);
  
  //Display Init
  SetupDisplay();   //bevor UI-Task erstellt wird, sonst kein LOGO

  //Create UI Task on core 0
  xTaskCreatePinnedToCore(UserInterface, "UI-Task", 10000, NULL, 1, &UI, 0);
}

//3-Phase SPWM-Generator Task
void Generate_SPWM(void* pvParameters) {
  static uint16_t index = 0;
  static uint32_t lastMicros = 0;
  while (1) {
    uint32_t now = micros();

    if (now - lastMicros >= (1000000UL / UPDATE_RATE)) {
      lastMicros = now;

      // 120° phase shifts
      uint16_t phase_u = index;
      uint16_t phase_v = (index + TABLE_SIZE / 3) % TABLE_SIZE;
      uint16_t phase_w = (index + (2 * TABLE_SIZE / 3)) % TABLE_SIZE;

      float duty_u = sin_table[phase_u];
      float duty_v = sin_table[phase_v];
      float duty_w = sin_table[phase_w];

      // Update phases
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_u);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_v);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_w);

      index++;

      if (index >= TABLE_SIZE) index = 0;
    }
  }
}

//UI-Task
void UserInterface(void* pvParameters){
  while(1){
    I1 = GetCurrent(I1_PIN);
    I2 = GetCurrent(I2_PIN);
    core0 = getCoreUsagePercent(0);
    core1 = getCoreUsagePercent(0);
    updateMenu();
    vTaskDelay(pdMS_TO_TICKS(50));
  }  
}

void loop(){}

//Generate Sine-Lookup-Table
void init_sin_table() {
  for (int i = 0; i < TABLE_SIZE; i++) {
    sin_table[i] = (sinf(2.0f * M_PI * i / TABLE_SIZE) + 1.0f) * 50.0f;
  }
}

//initialize 3-Phase SPWM-System
void init_mcpwm() {
  //Phases
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PHASE_U);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PHASE_V);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PHASE_W);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = PWM_FREQ;
  pwm_config.cmpr_a = 50;
  pwm_config.cmpr_b = 50;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  // 3 independent PWM timers
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
}

void SetupDisplay(){

  display.begin(0x3C,true);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.drawBitmap(34,0,HS_logo,64,64,1); //Hochschul-Logo anzeigen
  display.display();
  delay(2000);
  display.clearDisplay();
  draw();
}

// CURRENT MEASUREMENT
float GetCurrent(int pin) {
  int adc = analogRead(pin);
  float v = (adc * 3.3f) / 4096.0f;
  float vin = v * (47000.0f + 91000.0f) / 91000.0f;
  return 25.0f * (vin - 2.5f);
}

// ENCODER ISR
void IRAM_ATTR encoderISR() {
  uint8_t A = digitalRead(ENCODER_A);
  uint8_t B = digitalRead(ENCODER_B);
  if (A != lastA) {
    if (B != A) encoderDelta--;
    else encoderDelta++;
  }
  lastA = A;
}

// STEP SIZE
float getStep(MenuItem& item) {

  switch (item.type) {
    case TYPE_UINT32:

    case TYPE_INT:
      return pow(10.0f, selectedDigit);

    case TYPE_FLOAT:
      return pow(10.0f, -selectedDigit);

    default:
      return 1.0f;
  }
}

// CHANGE VALUE
void changeValue(MenuItem& item, int dir) {

  if (item.variable == nullptr) return;

  float step = getStep(item);

  switch (item.type) {

    case TYPE_UINT32:
      {
        uint32_t* v = (uint32_t*)item.variable;

        if (dir > 0) {
          *v += (uint32_t)step;
        } else if (*v >= (uint32_t)step) {
          *v -= (uint32_t)step;
        }

        // IF PWM_FREQ CHANGED -> UPDATE MCPWM
        if (v == &PWM_FREQ) {
          update_mcpwm_frequency();
        }

        break;
      }

    case TYPE_INT:
      {
        int* v = (int*)item.variable;
        *v += (int)(dir * step);
        break;
      }

    case TYPE_FLOAT:
      {
        float* v = (float*)item.variable;
        *v += dir * step;
        break;
      }
  }
}

// DRAW VALUE
void drawEditableValue(int x, int y, MenuItem& item, bool selected) {
  char buf[32];

  // FORMAT
  switch (item.type) {

    case TYPE_UINT32:
      sprintf(buf, "%lu", *(uint32_t*)item.variable);
      break;

    case TYPE_INT:
      sprintf(buf, "%d", *(int*)item.variable);
      break;

    case TYPE_FLOAT:
      dtostrf(*(float*)item.variable, 0, 3, buf);
      break;
  }

  while (buf[0] == ' ') {
    memmove(buf, buf + 1, strlen(buf));
  }

  // DRAW TEXT
  display.setCursor(x, y);
  display.print(buf);

  if (item.unit && strlen(item.unit) > 0) {
    display.print(" ");
    display.print(item.unit);
  }

  // CURSOR
  if (selected && editMode) {
    int len = strlen(buf);
    int digitIndex = -1;

    // INTEGER TYPES
    if (item.type == TYPE_UINT32 || item.type == TYPE_INT) {
      int numericDigits = 0;
      for (int i = 0; i < len; i++) {
        if (isDigit(buf[i])) {
          numericDigits++;
        }
      }

      if (selectedDigit < 0) {
        selectedDigit = 0;
      }

      if (selectedDigit >= numericDigits) {
        selectedDigit = numericDigits - 1;
      }

      int count = 0;

      for (int i = len - 1; i >= 0; i--) {
        if (isDigit(buf[i])) {
          if (count == selectedDigit) {
            digitIndex = i;
            break;
          }
          count++;
        }
      }

      if (digitIndex < 0) {
        digitIndex = 0;
      }
    }

    // FLOAT TYPES
    else if (item.type == TYPE_FLOAT) {
      int decimalPos = -1;
      for (int i = 0; i < len; i++) {
        if (buf[i] == '.') {
          decimalPos = i;
          break;
        }
      }

      if (decimalPos >= 0) {
        if (selectedDigit == 0) {
          digitIndex = decimalPos - 1;
        } else {
          digitIndex = decimalPos + selectedDigit;
        }
        if (digitIndex < 0 || digitIndex >= len || !isDigit(buf[digitIndex])) {
          digitIndex = -1;
        }
      }
    }

    // DRAW UNDERLINE
    if (digitIndex >= 0 && isDigit(buf[digitIndex])) {
      int ux = x + (digitIndex * 6);
      if (digitIndex == 0) {
        ux += 1;
      }
      int uy = y + 8;
      display.drawLine(ux, uy, ux + 4, uy, SH110X_WHITE);
    }
  }
}

// MENU HELPERS
void moveSelection(int dir) {

  MenuPage& page = pages[currentPage];
  int next = selectedItem;

  for (uint8_t i = 0; i < page.itemCount; i++) {
    next += dir;
    if (next >= page.itemCount) {
      next = 0;
    }
    if (next < 0) {
      next = page.itemCount - 1;
    }
    if (page.items[next].selectable) {
      selectedItem = next;
      return;
    }
  }
}

void activateItem() {

  MenuPage& page = pages[currentPage];
  MenuItem& item = page.items[selectedItem];

  if (item.isPageSwitch) {
    currentPage = item.targetPage;
    selectedItem = 0;
    editMode = false;
    selectedDigit = 0;
  } else if (item.selectable) {
    editMode = !editMode;
    selectedDigit = 0;
  }
}

// DRAW MENU
void draw() {

  display.clearDisplay();
  contrast = constrain(contrast, 0, 255);
  display.setContrast(contrast);
  MenuPage& page = pages[currentPage];

  // HEADER
  display.setCursor(0, 0);
  display.println(page.header);
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);

  // ITEMS
  for (int i = 0; i < page.itemCount; i++) {
    int y = 14 + i * 10;
    display.setCursor(0, y);

    if (i == selectedItem) {
      display.print(editMode ? "* " : "> ");
    } else {
      display.print("  ");
    }

    display.print(page.items[i].label);

    if (!page.items[i].isPageSwitch) {
      display.print(": ");
      if (page.items[i].variable != nullptr) {
        int valueX = display.getCursorX();
        int valueY = display.getCursorY();
        drawEditableValue(valueX, valueY, page.items[i], i == selectedItem);
      }
    }
  }

  display.display();
}

// UPDATE MENU
void updateMenu() {

  static uint32_t lastButtonTime = 0;
  const uint32_t debounceMs = 80;
  uint32_t now = millis();
  MenuPage& page = pages[currentPage];

  // ENCODER
  int8_t d;
  noInterrupts();
  d = encoderDelta;
  encoderDelta = 0;
  interrupts();

  if (d != 0) {
    int dir = (d > 0) ? 1 : -1;
    if (editMode) {
      changeValue(page.items[selectedItem], dir);
    } else {
      moveSelection(dir);
    }
  }

  // BUTTONS
  bool up = !digitalRead(BUTTON_UP);
  bool down = !digitalRead(BUTTON_DOWN);
  bool left = !digitalRead(BUTTON_LEFT);
  bool right = !digitalRead(BUTTON_RIGHT);
  bool enc = !digitalRead(ENCODER_SW);

  // BUTTON HANDLING
  if (now - lastButtonTime > debounceMs) {

    // UP
    if (up && !lastUp) {
      if (editMode) {
        changeValue(page.items[selectedItem], 1);
      } else {
        moveSelection(-1);
      }
      lastButtonTime = now;
    }

    // DOWN
    if (down && !lastDown) {
      if (editMode) {
        changeValue(page.items[selectedItem], -1);
      } else {
        moveSelection(1);
      }
      lastButtonTime = now;
    }

    // RIGHT BUTTON
    if (right && !lastRight) {
      if (editMode) {
        selectedDigit--;  // move toward smaller digit
        if (selectedDigit < 0) {
          selectedDigit = 0;
        }
      } else {
        activateItem();
      }
      lastButtonTime = now;
    }

    // LEFT BUTTON
    if (left && !lastLeft) {
      if (editMode) {
        MenuItem& item = page.items[selectedItem];
        int maxDigits = 1;
        if (item.type == TYPE_UINT32) {
          maxDigits = String(*(uint32_t*)item.variable).length();
        } else if (item.type == TYPE_INT) {
          maxDigits = String(abs(*(int*)item.variable)).length();
        } else if (item.type == TYPE_FLOAT) {
          maxDigits = 3;
        }
        // already at highest digit?
        // exit edit mode
        if (selectedDigit >= maxDigits - 1) {
          editMode = false;
          selectedDigit = 0;
        }
        // otherwise move left
        else {
          selectedDigit++;
        }

      } else if (currentPage != 0) {
        currentPage = 0;
        selectedItem = 0;
      }
      lastButtonTime = now;
    }

    // ENCODER BUTTON
    if (enc && !lastEncoderButton) {
      activateItem();
      lastButtonTime = now;
    }
  }

  // SAVE STATES
  lastUp = up;
  lastDown = down;
  lastLeft = left;
  lastRight = right;
  lastEncoderButton = enc;

  // DRAW
  static uint32_t lastDraw = 0;
  if (millis() - lastDraw > 30) {
    draw();
    lastDraw = millis();
  }
}
//Reinitialize SPWM-Generator after carrierfrequency is changed
void update_mcpwm_frequency() {

  // optional safety limits
  PWM_FREQ = constrain(PWM_FREQ, 1000, 100000);

  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, PWM_FREQ);
  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_1, PWM_FREQ);
  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_2, PWM_FREQ);
}

//Kern-Auslastung berechnen
float getCoreUsagePercent(BaseType_t coreID) {

  static uint32_t lastIdle0 = 0;
  static uint32_t lastIdle1 = 0;
  static uint32_t lastTime = 0;
  uint32_t now = millis();

  // runtime counters of idle tasks
  TaskStatus_t taskStatusArray[20];
  uint32_t totalRunTime;
  UBaseType_t taskCount = uxTaskGetSystemState(taskStatusArray, 20, &totalRunTime);
  uint32_t idleRunTime = 0;

  for (UBaseType_t i = 0; i < taskCount; i++) {
    // Idle task names:
    // "IDLE0" -> Core 0
    // "IDLE1" -> Core 1
    if (coreID == 0 && strcmp(taskStatusArray[i].pcTaskName, "IDLE0") == 0) {
      idleRunTime = taskStatusArray[i].ulRunTimeCounter;
    }
    if (coreID == 1 && strcmp(taskStatusArray[i].pcTaskName, "IDLE1") == 0) {
      idleRunTime = taskStatusArray[i].ulRunTimeCounter;
    }
  }

  uint32_t* lastIdle = (coreID == 0) ? &lastIdle0 : &lastIdle1;
  uint32_t idleDelta = idleRunTime - *lastIdle;
  uint32_t timeDelta = now - lastTime;
  *lastIdle = idleRunTime;

  if (timeDelta == 0) return 0.0f;

  // convert idle percentage to usage
  float idlePercent = ((float)idleDelta / (float)totalRunTime) * 100.0f;
  float usage = 100.0f - idlePercent;

  if (coreID == 1) {
    lastTime = now;
  }
  usage = constrain(usage, 0.0f, 100.0f);
  return usage;
}