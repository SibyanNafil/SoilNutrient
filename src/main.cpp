#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <DallasTemperature.h>
#include <ModbusMaster.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/*
  kode Modbust protokol
  Nitrogen : 01 03 00 1e 00 01 e4 0c
  Phosphor : 01 03 00 1f 00 01 b5 cc
  Potassium: 01 03 00 20 00 01 85 c0
*/ 

// pin definitions
#define ONE_WIRE_BUS 17
#define SOIL_MOISTURE_PIN 16
#define EN_RS485 8
#define PH_RXPIN 9
#define PH_TXPIN 18
#define NPK_RXPIN 39
#define NPK_TXPIN 40
#define EN_NPK 42
// #define TFT_CS   4      10 or 34 (FSPI CS0) 
// #define TFT_MOSI 7      11 or 35 (FSPI D)
// #define TFT_SCLK 15      12 or 36 (FSPI CLK)
// // #define TFT_MISO 37     13 or 37 (FSPI Q)

// // Use pins in range 0-31
// #define TFT_DC    6
// #define TFT_RST   5


/*Change to your screen resolution*/
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

TFT_eSPI tft = TFT_eSPI( screenWidth, screenHeight ); /* TFT instance */

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush (lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap)
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    if (LV_COLOR_16_SWAP) {
        size_t len = lv_area_get_size( area );
        lv_draw_sw_rgb565_swap( pixelmap, len );
    }

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( (uint16_t*) pixelmap, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read (lv_indev_t * indev_driver, lv_indev_data_t * data)
{
    uint16_t touchX = 0, touchY = 0;

    bool touched = false;//tft.getTouch( &touchX, &touchY, 600 );

    if (!touched)
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;

        Serial.print( "Data x " );
        Serial.println( touchX );

        Serial.print( "Data y " );
        Serial.println( touchY );
    }
}

// Initialize oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// initialize RS485 comunnication
ModbusMaster node;
ModbusMaster node2;
#define SLAVE_ADDR ((uint16_t)0x01)

// initialize NPK and PH sensor value
uint16_t nilai_Nitrogen;
uint16_t nilai_Phosphorus;
uint16_t nilai_Potassium;
uint16_t nilai_pH;
float ph = 0;
int nitrogen = 0;
int phosphorous = 0;
int potassium = 0;

// pre-transmision and post-transmision NPK sensor procedure
void preTransmission_NPK()
{
  digitalWrite(EN_NPK, 1);
}
void postTransmission_NPK()
{
  digitalWrite(EN_NPK, 0);
}

// pre-transmision and post-transmission PH sensor procedur
void preTransmission_PH()
{
  digitalWrite(EN_RS485, 1);
}
void postTransmission_PH()
{
  digitalWrite(EN_RS485, 0);
}

BLEServer* pServer = NULL;
BLECharacteristic* pTempCharacteristic = NULL;
BLECharacteristic* pMoistureCharacteristic = NULL;
BLECharacteristic* pPhCharacteristic = NULL;
BLECharacteristic* pNitrogenCharacteristic = NULL;
BLECharacteristic* pPhosphorCharacteristic = NULL;
BLECharacteristic* pPotassiumCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

/*Characteristic UUID Comunication*/ 
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TEMP_CHARACTERISTIC_UUID "68a6e0f2-5429-496c-883a-ac5d8a87de9b"
#define MOISTURE_CHARACTERISTIC_UUID "e54f29d4-1b0e-4d44-bd4e-43f5d5c40f50"
#define PH_CHARACTERISTIC_UUID "551a162b-e2e0-4c3a-ab36-6d6668a6e5ad"
#define NITROGEN_CHARACTERISTIC_UUID "8ffef044-1203-41e2-8281-e65626b6ad97"
#define PHOSPHOR_CHARACTERISTIC_UUID "1195044f-9b1a-4834-a387-2c1bed672102"
#define POTASSIUM_CHARACTERISTIC_UUID "4db8ef1d-dd96-4a9c-89c7-d1338f30372f"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        BLEDevice::startAdvertising();
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

/*Set tick routine needed for LVGL internal timings*/
static uint32_t my_tick_get_cb (void) { return millis(); }


void setup1() {

    pinMode(EN_RS485, OUTPUT);
    pinMode(EN_NPK, OUTPUT);
    
    Serial.begin( 115200 );
    // Initialize sensors
    Serial1.begin(4800, SERIAL_8N1, PH_RXPIN, PH_TXPIN);
    Serial2.begin(4800, SERIAL_8N1, NPK_RXPIN, NPK_TXPIN);
    sensors.begin();

    node.preTransmission(preTransmission_PH);
    node.postTransmission(postTransmission_PH);
    node2.preTransmission(preTransmission_NPK);
    node2.postTransmission(postTransmission_NPK);
    node.begin(SLAVE_ADDR, Serial1);
    node2.begin(SLAVE_ADDR, Serial2);
    // Initialize BLE
    BLEDevice::init("PKM_KC_SOIL_NUTRIENTS");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService( BLEUUID (SERVICE_UUID), 18);

    pTempCharacteristic = pService->createCharacteristic(
        TEMP_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pTempCharacteristic->addDescriptor(new BLE2902());

    pMoistureCharacteristic = pService->createCharacteristic(
        MOISTURE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pMoistureCharacteristic->addDescriptor(new BLE2902());

    pPhCharacteristic = pService->createCharacteristic(
        PH_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pPhCharacteristic->addDescriptor(new BLE2902());

    pNitrogenCharacteristic = pService->createCharacteristic(
        NITROGEN_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pNitrogenCharacteristic->addDescriptor(new BLE2902());

    pPhosphorCharacteristic = pService->createCharacteristic(
        PHOSPHOR_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pPhosphorCharacteristic->addDescriptor(new BLE2902());

    pPotassiumCharacteristic = pService->createCharacteristic(
        POTASSIUM_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pPotassiumCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    Serial.println("Waiting for a client connection to notify...");
}

int baca_sensor_pH(uint16_t alamatpH) {
//   Serial.print("Reading pH from address: ");
//   Serial.println(alamatpH, HEX);
  int Result_pH;
  Result_pH = node.readHoldingRegisters(alamatpH, 1);
  if (Result_pH == node.ku8MBSuccess)
  {
  nilai_pH = node.getResponseBuffer(0);
  } else {
    Serial.println("modbus ph fail");
  }
  return nilai_pH;
}

int baca_Nitrogen(uint16_t alamatN)
{
//   Serial.print("Reading Nitrogen from address: ");
//   Serial.println(alamatN, HEX);
  int Result_Nitrogen;
  Result_Nitrogen = node2.readHoldingRegisters(alamatN, 1);
  if(Result_Nitrogen == node2.ku8MBSuccess)
  {
    nilai_Nitrogen = node2.getResponseBuffer(0);
  }
  else{
    Serial.println("modbus Nitrogen fail");
  }
  return nilai_Nitrogen;
}

int baca_Phosphrus(uint16_t alamatP)
{
//   Serial.print("Reading Phosphorus from address: ");
//   Serial.println(alamatP, HEX);
  int Result_Phosphorus;
  Result_Phosphorus = node2.readHoldingRegisters(alamatP, 1);
  if(Result_Phosphorus == node2.ku8MBSuccess)
  {
    nilai_Phosphorus = node2.getResponseBuffer(0);
  }
  else{
    Serial.println("modbus Phosphorus fail");
  }
  return nilai_Phosphorus;
}

int baca_potassium(uint16_t alamatK)
{
//   Serial.print("Reading Potassium from address: ");
//   Serial.println(alamatK, HEX);
  int Result_Potassium;
  Result_Potassium = node2.readHoldingRegisters(alamatK,1);
  if(Result_Potassium == node2.ku8MBSuccess)
  {
    nilai_Potassium = node2.getResponseBuffer(0);
  }
  else{
    Serial.println("modbus potassium fail");
  }
  return nilai_Potassium;
}

void loop1() {

    
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    int soilMoistureValue = analogRead(SOIL_MOISTURE_PIN);
    int persentase = map(soilMoistureValue, 3010, 0, 0, 100);
    ph = baca_sensor_pH(0) / 10.0;
    nitrogen = baca_Nitrogen(0x1E);
    phosphorous = baca_Phosphrus(0x1f);
    potassium = baca_potassium(0x20);

    Serial.println("Moisture:" + String(persentase) + "| analog : " + String(soilMoistureValue));
    Serial.println("Nitrogen: " + String(nitrogen) + " mg/kg");
    Serial.println("Phosphorous: " + String(phosphorous) + " mg/kg");
    Serial.println("Potassium: " + String(potassium) + " mg/kg");
    Serial.println("pH: " + String(ph));
    
    // update Sensor value to LCD
    lv_label_set_text(ui_TempValueLabel, String(tempC).c_str());
    lv_label_set_text(ui_HumValueLabel, String(persentase).c_str());
    lv_label_set_text(ui_PhValueLabel, String(ph).c_str());
    lv_label_set_text(ui_NValueLabel, String(nitrogen).c_str());
    lv_label_set_text(ui_PValueLabel, String(phosphorous).c_str());
    lv_label_set_text(ui_KValueLabel, String(potassium).c_str());

    // Update BLE characteristics
    if (deviceConnected) {
        pTempCharacteristic->setValue(String(tempC).c_str());
        pTempCharacteristic->notify();

        pMoistureCharacteristic->setValue(String(persentase).c_str());
        pMoistureCharacteristic->notify();

        pPhCharacteristic->setValue(String(ph).c_str());
        pPhCharacteristic->notify();

        pNitrogenCharacteristic->setValue(String(nitrogen).c_str());
        pNitrogenCharacteristic->notify();

        pPhosphorCharacteristic->setValue(String(phosphorous).c_str());
        pPhosphorCharacteristic->notify();

        pPotassiumCharacteristic->setValue(String(potassium).c_str());
        pPotassiumCharacteristic->notify();

    }
    

    // Handle BLE connection status
    if (!deviceConnected && oldDeviceConnected) {
        pServer->startAdvertising(); // restart advertising
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    vTaskDelay(2);
}

TaskHandle_t task_loop1;
void esploop1(void* pvParameters) {
  setup1();

  for (;;)
    loop1();
}


void setup ()
{
    // String LVGL_Arduino = "Hello Arduino! ";
    // LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    // Serial.println( LVGL_Arduino );
    // Serial.println( "I am LVGL_Arduino" );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    tft.setRotation( 3 ); /* Landscape orientation, flipped */

    static lv_disp_t* disp;
    disp = lv_display_create( screenWidth, screenHeight );
    lv_display_set_buffers( disp, buf, NULL, SCREENBUFFER_SIZE_PIXELS * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL );
    lv_display_set_flush_cb( disp, my_disp_flush );

    static lv_indev_t* indev;
    indev = lv_indev_create();
    lv_indev_set_type( indev, LV_INDEV_TYPE_POINTER );
    lv_indev_set_read_cb( indev, my_touchpad_read );

    lv_tick_set_cb( my_tick_get_cb );

    ui_init();
 
    // Serial.println( "Setup done" );
    xTaskCreatePinnedToCore(
    esploop1,               /* Task function. */
    "loop1",                /* name of task. */
    10000,                  /* Stack size of task */
    NULL,                   /* parameter of the task */
    0,                      /* priority of the task */
    &task_loop1,            /* Task handle to keep track of created task */
    ARDUINO_RUNNING_CORE); /* pin task to core */
}

void loop ()
{   
     lv_timer_handler(); /* let the GUI do its work */
    vTaskDelay(50);
}
