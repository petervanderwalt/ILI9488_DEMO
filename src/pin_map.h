// Serial2
#define TXD2 43
#define RXD2 44

// I2C Touch
#define TIRQ_PIN 2
#define I2C_SCL 4
#define I2C_SDA 3

#define VIBRATOR_PIN 45
#define BEEPER_PIN 47
#define MODE_BUTTON_PIN 0

// SD Card
#define SPI_MOSI 35
#define SPI_MISO 37
#define SPI_SCK 36
#define SD_CS 1

// NeoPixels
#define PIXEL_PIN 48

// LCD Backlight
#define LCD_BLK 46

// LCD
#define LCD_CS 42
#define LCD_WR 40
#define LCD_RD 39
#define LCD_RS 41
#define LCD_D0 18  // 20 on v1.1
#define LCD_D1 17  // 19 on v1.1
#define LCD_D2 14  // 14
#define LCD_D3 13  // 13
#define LCD_D4 12  // 12
#define LCD_D5 11  // 11
#define LCD_D6 10  // 10
#define LCD_D7 9   // 9
#define LCD_D8 21  // 21
#define LCD_D9 8   // 8
#define LCD_D10 16 // 16
#define LCD_D11 15 // 15
#define LCD_D12 7  // 7
#define LCD_D13 6  // 6
#define LCD_D14 5  // 5
#define LCD_D15 38 // 38


#define LEDC_BEEPER 0
#define LEDC_BACKLIGHT 5

void init_pins() {
  //pinMode(PIXEL_PIN, OUTPUT);
  
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, LOW);

  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, LOW);
  // use first channel of 16 channels (started from zero)
  #define LCD_BACKLIGHT_CHANNEL LEDC_BACKLIGHT
  // use 12 bit precission for LEDC timer
  #define LEDC_TIMER_12_BIT  12
  // use 5000 Hz as a LEDC base frequency
  #define LEDC_BASE_FREQ     5000
  ledcSetup(LCD_BACKLIGHT_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(LCD_BLK, LCD_BACKLIGHT_CHANNEL);

  pinMode(VIBRATOR_PIN, OUTPUT);
  digitalWrite(LCD_CS, LOW);

  pinMode(BEEPER_PIN, OUTPUT);
  ledcAttachPin(BEEPER_PIN, LEDC_BEEPER);


  pinMode(MODE_BUTTON_PIN, INPUT);
}

void init_serial() {
  // Serial
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    Serial2.setRxBufferSize(512);
}

void vibrate() {
  digitalWrite(VIBRATOR_PIN, HIGH);
}

void stopVibrate() {
  digitalWrite(VIBRATOR_PIN, LOW);

}
