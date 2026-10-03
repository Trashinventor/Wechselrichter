//******************************************************************************************************
// GaN-Wechselrichter Source-Code
// by Moritz Rambold 2026
// thetrashinventor.de
// Version 4
//******************************************************************************************************

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>
#include <math.h>
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include <ESP32RotaryEncoder.h>

//Parameter
#define TABLE_SIZE 1024       //samples lookup table
#define MCPWM_RESOLUTION_HZ 40000000UL  //MCPWM timer resolution

// Display
#define SCREEN_WIDTH 132
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display =  Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// RANDNOTIZ:
// Display CH1116-Driver -> deklariert als 128x64 eigentlich aber physisch 132x64
// FIX:
// in Adafruit_SH110X.cpp -> page_start_offset auf 0 ändern

// Pins
#define PHASE_U 13
#define PHASE_V 12
#define PHASE_W 14

#define OLED_SDA 21
#define OLED_SCL 22

#define BUTTON_UP 34
#define BUTTON_DOWN 35
#define BUTTON_LEFT 32
#define BUTTON_RIGHT 33

#define ENCODER_SW 27
#define ENCODER_A 25
#define ENCODER_B 26

#define I1_PIN 2
#define I2_PIN 4

// Variables
uint32_t sin_table[TABLE_SIZE];
volatile uint32_t PWM_FREQ = 100000;
volatile uint32_t SINE_FREQ = 50;
volatile float I1 = 0;
volatile float I2 = 0;
volatile uint32_t aussteuergrad = 100;

// MCPWM
mcpwm_timer_handle_t mcpwm_timer = NULL;

mcpwm_oper_handle_t mcpwm_oper_u = NULL;
mcpwm_oper_handle_t mcpwm_oper_v = NULL;
mcpwm_oper_handle_t mcpwm_oper_w = NULL;

mcpwm_cmpr_handle_t mcpwm_cmp_u = NULL;
mcpwm_cmpr_handle_t mcpwm_cmp_v = NULL;
mcpwm_cmpr_handle_t mcpwm_cmp_w = NULL;

mcpwm_gen_handle_t mcpwm_gen_u = NULL;
mcpwm_gen_handle_t mcpwm_gen_v = NULL;
mcpwm_gen_handle_t mcpwm_gen_w = NULL;

volatile uint32_t mcpwm_period_ticks = 0; // Current MCPWM period in timer ticks
volatile uint32_t spwmPhaseAccumulator = 0; // Hardware SPWM phase accumulator

// Menu types
enum VarType{TYPE_UINT32, TYPE_INT, TYPE_FLOAT};

struct MenuItem {
  const char* label;
  void* variable;
  VarType type;
  const char* unit;
  bool selectable;
  bool isPageSwitch;
  uint8_t targetPage;
};

struct MenuPage {
  const char* header;
  MenuItem* items;
  uint8_t itemCount;
};

// Homepage
MenuItem homeItems[] = {
  {"f-PWM", (void*)&PWM_FREQ, TYPE_UINT32, "Hz", true, false, 0},
  {"f-SIN", (void*)&SINE_FREQ, TYPE_UINT32, "Hz", true, false, 0},
  {"Ausst.", (void*)&aussteuergrad, TYPE_UINT32, "%", true, false, 0},
  {"I1", (void*)&I1, TYPE_FLOAT, "A", false, false, 0},
  {"I2", (void*)&I2, TYPE_FLOAT, "A", false, false, 0}
};

// Pages
MenuPage pages[] = {
  {"HOME", homeItems, 5}
};

// States
volatile int8_t encoderDelta = 0;
volatile uint8_t lastA = 0;
uint8_t currentPage = 0;
uint8_t selectedItem = 0;
bool editMode = false;

// 0 = ones
// 1 = tens
// 2 = hundreds
int selectedDigit = 0;

// Button states
bool lastUp = false;
bool lastDown = false;
bool lastLeft = false;
bool lastRight = false;
bool lastEncoderButton = false;

// Task handles
TaskHandle_t UI;

// Function prototypes
void init_sin_table();                              //Calculate Sine-Lookuptable
void init_mcpwm();                                  //Initialize 3-Phase-SPWM Generation
void SetupDisplay();                                //Initialize OLED-Display
float GetCurrent(int);                              //Calculate Current-value from ADC
void IRAM_ATTR encoderISR();                        //ISR for rotary-encoder
float getStep(MenuItem&);                           //Menusystem: Get stepsize for changing selected value
void changeValue(MenuItem&, int);                   //Menusystem: Change selected value
void drawEditableValue(int, int, MenuItem&, bool);  //Menusystem: format string for displaying editable value on screen
void moveSelection(int);                            //Menusystem: set Cursor to selected item
void activateItem();                                //Menusystem: Display activation symbol if value in editmode
void draw();                                        //Menusystem: Rendering of Menu
void updateMenu();                                  //Menusystem: Update Menu 
void update_mcpwm_frequency();                      //Reinitialize 3-Phase-SPWM when carrierfrequency changed via menu

// Hardware MCPWM callback
static bool IRAM_ATTR mcpwm_timer_on_empty(
  mcpwm_timer_handle_t timer,
  const mcpwm_timer_event_data_t* edata,
  void* user_data);

// 'HS_logo', 64x64px
static const unsigned char PROGMEM HS_logo[] = {
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

// Setup
void setup() {
  Serial.begin(9600);
  pinMode(PHASE_U, OUTPUT);
  pinMode(PHASE_V, OUTPUT);
  pinMode(PHASE_W, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT);
  pinMode(BUTTON_DOWN, INPUT);
  pinMode(BUTTON_LEFT, INPUT);
  pinMode(BUTTON_RIGHT, INPUT);
  pinMode(I1_PIN, INPUT);
  pinMode(I2_PIN, INPUT);

  init_sin_table();
  init_mcpwm();
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  lastA = digitalRead(ENCODER_A);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  
  Wire.begin(OLED_SDA, OLED_SCL);
  SetupDisplay();
  xTaskCreatePinnedToCore(UserInterface, "UI-Task", 16000, NULL, 1, &UI, 0);
}

void loop() {}

// Generate Sine Lookup Table
void init_sin_table() {
  uint32_t modulation = aussteuergrad;
  // Safety
  if(modulation > 100){modulation = 100;}
  for(int i = 0; i < TABLE_SIZE; i++){
    float sine = sinf(2.0f * M_PI * (float)i / (float)TABLE_SIZE);
    float duty = 50.0f + (sine * 50.0f * (float)modulation / 100.0f);
    // Convert percentage to 1/1000 percentage
    uint32_t dutyFixed = (uint32_t)(duty * 1000.0f);
    if (dutyFixed > 100000){dutyFixed = 100000;}
    sin_table[i] = dutyFixed;
  }
}

// Initialize MCPWM
void init_mcpwm() {

  mcpwm_period_ticks = MCPWM_RESOLUTION_HZ / PWM_FREQ;  // Calculate PWM period
  if(mcpwm_period_ticks < 2){
    mcpwm_period_ticks = 2;
  }

  //Timer config
  mcpwm_timer_config_t timer_config = {};
  timer_config.group_id = 0;
  timer_config.resolution_hz = MCPWM_RESOLUTION_HZ;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.period_ticks = mcpwm_period_ticks;
  timer_config.flags.update_period_on_empty = true;
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &mcpwm_timer));

  //Operator config
  mcpwm_operator_config_t oper_config = {};
  oper_config.group_id = 0;
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &mcpwm_oper_u));
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &mcpwm_oper_v));
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &mcpwm_oper_w));

  //Connect operators to timer
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_oper_u, mcpwm_timer));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_oper_v, mcpwm_timer));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_oper_w, mcpwm_timer));

  // Comparator configuration
  mcpwm_comparator_config_t cmp_config = {};
  cmp_config.flags.update_cmp_on_tez = true;   // New comparator value becomes active at timer-zero.
  ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_oper_u, &cmp_config, &mcpwm_cmp_u));
  ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_oper_v, &cmp_config, &mcpwm_cmp_v));
  ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_oper_w, &cmp_config, &mcpwm_cmp_w));

  // Generator U
  mcpwm_generator_config_t gen_config_u = {};
  gen_config_u.gen_gpio_num = PHASE_U;
  ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_oper_u, &gen_config_u, &mcpwm_gen_u));

  // Generator V
  mcpwm_generator_config_t gen_config_v = {};
  gen_config_v.gen_gpio_num = PHASE_V;
  ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_oper_v, &gen_config_v, &mcpwm_gen_v));

  // Generator W
  mcpwm_generator_config_t gen_config_w = {};
  gen_config_w.gen_gpio_num = PHASE_W;
  ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_oper_w, &gen_config_w, &mcpwm_gen_w));

  // Generator U actions
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(mcpwm_gen_u,
    MCPWM_GEN_TIMER_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      MCPWM_TIMER_EVENT_EMPTY,
      MCPWM_GEN_ACTION_HIGH)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mcpwm_gen_u,
    MCPWM_GEN_COMPARE_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      mcpwm_cmp_u,
      MCPWM_GEN_ACTION_LOW)));

  // Generator V actions
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(mcpwm_gen_v,
    MCPWM_GEN_TIMER_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      MCPWM_TIMER_EVENT_EMPTY,
      MCPWM_GEN_ACTION_HIGH)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mcpwm_gen_v,
    MCPWM_GEN_COMPARE_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      mcpwm_cmp_v,
      MCPWM_GEN_ACTION_LOW)));
  
  // Generator W actions
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(mcpwm_gen_w,
    MCPWM_GEN_TIMER_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      MCPWM_TIMER_EVENT_EMPTY,
      MCPWM_GEN_ACTION_HIGH)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mcpwm_gen_w,
    MCPWM_GEN_COMPARE_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      mcpwm_cmp_w,
      MCPWM_GEN_ACTION_LOW)));

  uint32_t initial_duty = mcpwm_period_ticks / 2;  // Initial duty = 50 %
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_cmp_u, initial_duty));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_cmp_v, initial_duty));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_cmp_w, initial_duty));


  // Register hardware timer callback (callback frequency = PWM_FREQ)
  mcpwm_timer_event_callbacks_t callbacks = {};
  callbacks.on_empty = mcpwm_timer_on_empty;
  ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(mcpwm_timer, &callbacks, NULL));

  ESP_ERROR_CHECK(mcpwm_timer_enable(mcpwm_timer));  // Enable timer
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));  // Start timer
}

// Hardware-Timed 3-Phase SPWM
static bool IRAM_ATTR mcpwm_timer_on_empty(
  mcpwm_timer_handle_t timer,
  const mcpwm_timer_event_data_t* edata,
  void* user_data) {

  // Read current configuration
  uint32_t sineFreq = SINE_FREQ;
  uint32_t pwmFreq = PWM_FREQ;
  uint32_t period = mcpwm_period_ticks;
  if(pwmFreq == 0 || period < 2){return false;}  // Safety



  //****************************************************************************************************
  // Calculate phase increment (phaseIncrement = (SINE_FREQ/PWMFREQU) * 2^32)
  uint32_t phaseIncrement = (uint32_t)(((uint64_t)sineFreq << 32)/ pwmFreq);

  const uint32_t PHASE_SHIFT_120 = 0x55555555UL;  // 120° phase shift

  // Calculate phase positions
  uint32_t phase_u = spwmPhaseAccumulator;
  uint32_t phase_v = phase_u + PHASE_SHIFT_120;
  uint32_t phase_w = phase_u + (2UL * PHASE_SHIFT_120);

  // Convert phase to lookup-table index
  uint16_t index_u = (uint16_t)(((uint64_t)phase_u * TABLE_SIZE) >> 32);
  uint16_t index_v = (uint16_t)(((uint64_t)phase_v * TABLE_SIZE) >> 32);
  uint16_t index_w = (uint16_t)(((uint64_t)phase_w * TABLE_SIZE) >> 32);

  // Read lookup table
  uint32_t duty_u = sin_table[index_u];
  uint32_t duty_v = sin_table[index_v];
  uint32_t duty_w = sin_table[index_w];

  // Convert duty to MCPWM ticks (compare = duty / 100000 * period)
  uint32_t compare_u = (uint32_t)(((uint64_t)duty_u * period)/ 100000ULL);
  uint32_t compare_v = (uint32_t)(((uint64_t)duty_v * period)/ 100000ULL);
  uint32_t compare_w = (uint32_t)(((uint64_t)duty_w * period)/ 100000ULL);

  // Safety clamp
  if(compare_u > period){compare_u = period;}
  if(compare_v > period){compare_v = period;}
  if(compare_w > period){compare_w = period;}

  // Update MCPWM comparators
  mcpwm_comparator_set_compare_value(mcpwm_cmp_u, compare_u);
  mcpwm_comparator_set_compare_value(mcpwm_cmp_v, compare_v);
  mcpwm_comparator_set_compare_value(mcpwm_cmp_w, compare_w);

  spwmPhaseAccumulator += phaseIncrement;   // Advance phase accumulator
  return false;
}

// UI Task
void UserInterface(void* pvParameters){
  while(1){
    I1 = GetCurrent(I1_PIN);
    I2 = GetCurrent(I2_PIN);
    updateMenu();
    vTaskDelay(pdMS_TO_TICKS(50));    // UI refresh interval
  }
}

// Setup Display
void SetupDisplay() {
  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.drawBitmap(34, 0, HS_logo, 64, 64, 1);
  display.display();
  delay(2000);
  display.clearDisplay();
  draw();
}

// Measure Current
float GetCurrent(int pin){
  int adc = analogRead(pin);
  float v = (adc / 4095.0f)* 3.3f;
  // Needs correct sampling + RMS calculation
  //
  // 1.646 V = 0 A
  // 0.327 V = -50 A
  // 2.946 V = +50 A
  return (v - 1.646f) * (50.0f / (2.946f - 1.646f));
}

// Encoder ISR
void IRAM_ATTR encoderISR(){
  uint8_t A = digitalRead(ENCODER_A);
  uint8_t B = digitalRead(ENCODER_B);
  if(A != lastA){
    if(B != A){encoderDelta--;}
    else {encoderDelta++;}
  }
  lastA = A;
}

// Stepsize
float getStep(MenuItem& item){
  switch(item.type){
    case TYPE_UINT32:
    case TYPE_INT: return pow(10.0f, selectedDigit);
    case TYPE_FLOAT: return pow(10.0f, -selectedDigit);
    default: return 1.0f;
  }
}

// Change value
void changeValue(MenuItem& item, int dir){
  if(item.variable == nullptr){return;}
  float step = getStep(item);

  // UINT32
  switch(item.type){
    case TYPE_UINT32:
    {
      uint32_t* v = (uint32_t*)item.variable;
      if(dir > 0){
        *v += (uint32_t)step;
      }
      else{
        if(*v >= (uint32_t)step){
          *v -= (uint32_t)step;
        }
        else{
          *v = 0;
        }
      }

      // PWM frequency
      if(v == (uint32_t*)&PWM_FREQ){
        update_mcpwm_frequency();
      }
  
      // Sine frequency
      if(v == (uint32_t*)&SINE_FREQ) {
        // No MCPWM restart necessary.
      }

        // Modulation
      if(v == (uint32_t*)&aussteuergrad){
        *v = constrain(*v, 0U, 100U);
        init_sin_table();
      }
      break;
    }

    // INT
    case TYPE_INT:
    {
      int* v = (int*)item.variable;
      *v += (int)(dir * step);
      break;
    }

    // FLOAT
    case TYPE_FLOAT:
    {
      float* v = (float*)item.variable;
      *v += dir * step;
      break;
    }
  }
}

// Draw editable value
void drawEditableValue(int x, int y, MenuItem& item, bool selected){
  char buf[32];

  // Format value
  switch(item.type){
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

  // Remove leading spaces
  while(buf[0] == ' '){
    memmove(buf, buf + 1, strlen(buf));
  }

  // Draw value
  display.setCursor(x, y);
  display.print(buf);
  if(item.unit && strlen(item.unit) > 0){
    display.print(" ");
    display.print(item.unit);
  }

  // Draw edit cursor
  if(selected && editMode){
    int len = strlen(buf);
    int digitIndex = -1;

    // Integer types
    if(item.type == TYPE_UINT32 || item.type == TYPE_INT){
      int numericDigits = 0;
      for(int i = 0; i < len; i++){
        if(isDigit(buf[i])){
          numericDigits++;
        }
      }
      if(selectedDigit < 0){
        selectedDigit = 0;
      }
      if(selectedDigit >= numericDigits){
        selectedDigit = numericDigits - 1;
      }
      int count = 0;
      for(int i = len - 1; i >= 0; i--){
        if(isDigit(buf[i])){
          if(count == selectedDigit){
            digitIndex = i;
            break;
          }
          count++;
        }
      }
      if(digitIndex < 0){
        digitIndex = 0;
      }
    }

    // Float
    else if(item.type == TYPE_FLOAT){
      int decimalPos = -1;
      for(int i = 0; i < len; i++){
        if(buf[i] == '.'){
          decimalPos = i;
          break;
        }
      }
      if(decimalPos >= 0){
        if(selectedDigit == 0){
          digitIndex = decimalPos - 1;
        }
        else {
          digitIndex = decimalPos + selectedDigit;
        }
        if(digitIndex < 0 || digitIndex >= len || !isDigit(buf[digitIndex])){
          digitIndex = -1;
        }
      }
    }

    // Draw underline
    if(digitIndex >= 0 && isDigit(buf[digitIndex])){
      int ux = x + (digitIndex * 6);
      if(digitIndex == 0){
        ux += 1;
      }
      int uy = y + 8;
      display.drawLine(ux, uy, ux + 4, uy, SH110X_WHITE);
    }
  }
}

// Move menu selection
void moveSelection(int dir){
  MenuPage& page = pages[currentPage];
  int next = selectedItem;

  for(uint8_t i = 0; i < page.itemCount; i++){
    next += dir;
    if(next >= page.itemCount){
      next = 0;
    }
    if(next < 0){
      next = page.itemCount - 1;
    }
    if(page.items[next].selectable){
      selectedItem = next;
      return;
    }
  }
}

// Activate menu item
void activateItem(){
  MenuPage& page = pages[currentPage];
  MenuItem& item = page.items[selectedItem];

  // Page switch
  if(item.isPageSwitch){
    currentPage = item.targetPage;
    selectedItem = 0;
    editMode = false;
    selectedDigit = 0;
  }

  // Edit value
  else if(item.selectable){
    editMode = !editMode;
    selectedDigit = 0;
  }
}

// Draw menu
void draw(){
  display.clearDisplay();
  MenuPage& page = pages[currentPage];

  // Header
  display.setCursor(0, 0);
  display.println(page.header);
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);

  // Items
  for(int i = 0; i < page.itemCount; i++){
    int y = 14 + i * 10;
    display.setCursor(0, y);

    // Selection marker
    if(i == selectedItem){
      display.print(editMode ? "* " : "> ");
    }
    else{
      display.print("  ");
    }
    display.print(page.items[i].label);

    // Value
    if(!page.items[i].isPageSwitch){
      display.print(": ");
      if(page.items[i].variable != nullptr){
        int valueX = display.getCursorX();
        int valueY = display.getCursorY();
        drawEditableValue(valueX, valueY, page.items[i], i == selectedItem);
      }
    }
  }
  display.display();
}

// Update menu
void updateMenu(){
  static uint32_t lastButtonTime = 0;
  const uint32_t debounceMs = 80;
  uint32_t now = millis();
  MenuPage& page = pages[currentPage];

  // Encoder
  int8_t d;
  noInterrupts();
  d = encoderDelta;
  encoderDelta = 0;
  interrupts();
  if(d != 0){
    int dir = (d > 0) ? 1 : -1;
    if(editMode){
      changeValue(page.items[selectedItem], dir);
    }
    else{
      moveSelection(dir);
    }
  }

  // Button states
  bool up = !digitalRead(BUTTON_UP);
  bool down = !digitalRead(BUTTON_DOWN);
  bool left = !digitalRead(BUTTON_LEFT);
  bool right = !digitalRead(BUTTON_RIGHT);
  bool enc = !digitalRead(ENCODER_SW);

  // Debounce buttons
  if(now - lastButtonTime > debounceMs){

    // UP
    if(up && !lastUp){
      if(editMode){
        changeValue(page.items[selectedItem], 1);
      }
      else {
        moveSelection(-1);
      }
      lastButtonTime = now;
    }

    // DOWN
    if(down && !lastDown){
      if(editMode){
        changeValue(page.items[selectedItem], -1);
      }
      else {
        moveSelection(1);
      }
      lastButtonTime = now;
    }

    // RIGHT
    if(right && !lastRight){
      if(editMode){
        selectedDigit--;
        if(selectedDigit < 0){
          selectedDigit = 0;
        }
      }
      else{
        activateItem();
      }
      lastButtonTime = now;
    }

    // LEFT
    if(left && !lastLeft){
      if(editMode){
        MenuItem& item = page.items[selectedItem];
        int maxDigits = 1;

        if(item.type == TYPE_UINT32){
          maxDigits = String(*(uint32_t*)item.variable).length();
        }
        else if(item.type == TYPE_INT){
          maxDigits = String(abs(*(int*)item.variable)).length();
        }
        else if(item.type == TYPE_FLOAT){
          maxDigits = 3;
        }

        // Exit edit mode when highest digit reached
        if(selectedDigit >= maxDigits - 1){
          editMode = false;
          selectedDigit = 0;
        }
        else{
          selectedDigit++;
        }
      }
      else if(currentPage != 0){
        currentPage = 0;
        selectedItem = 0;
      }
      lastButtonTime = now;
    }

    // Encoder button
    if(enc && !lastEncoderButton){
      activateItem();
      lastButtonTime = now;
    }
  }

  // Save button states
  lastUp = up;
  lastDown = down;
  lastLeft = left;
  lastRight = right;
  lastEncoderButton = enc;
  static uint32_t lastDraw = 0;
  if(millis() - lastDraw > 30){
    draw();
    lastDraw = millis();
  }
}

// Update MCPWM Frequency
void update_mcpwm_frequency() {
  PWM_FREQ = constrain(PWM_FREQ, 1000U, 100000U); //PWM limit 1-100 kHz

  // Calculate new period
  uint32_t new_period = MCPWM_RESOLUTION_HZ / PWM_FREQ;
  if(new_period < 2){
    new_period = 2;
  }

  mcpwm_period_ticks = new_period;  // Update global period
  ESP_ERROR_CHECK(mcpwm_timer_set_period(mcpwm_timer, new_period)); //new period synced to next timer-zero event.
  uint32_t initialDuty = new_period / 2;  // Start at 50 % duty

  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_cmp_u, initialDuty));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_cmp_v, initialDuty));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_cmp_w, initialDuty));
}