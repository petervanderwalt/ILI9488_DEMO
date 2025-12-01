/**
 * @file main.cpp
 * @brief 3D G-Code Viewer (Watchdog & Overflow Fix)
 * @details
 *   - FIXED: Project() function now clamps coordinates to prevent int16_t overflow artifacts.
 *   - RETAINED: Watchdog/Yield fixes for stability.
 */

#include <Arduino.h>

#include "esp_task_wdt.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "soc/soc.h"

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <vector>
#include <math.h>
#include <FS.h>
#include <SPIFFS.h>

#include <Adafruit_NeoPixel.h>

// Fetch Online files
#include <WiFi.h>
#include <HTTPClient.h>
String fetchURL = "https://pastebin.com/raw/vtZVze7H";


#define DEBUG_POINT(msg) Serial.printf("[%lu] %s\n", millis(), msg); Serial.flush();
#define DEBUG_VAR(name, val) Serial.printf("[%lu] %s = %d\n", millis(), name, val); Serial.flush();


// =======================================================================================
// CONFIGURATION
// =======================================================================================

const char* WIFI_SSID = "30BIRD-YARD";
const char* WIFI_PASS = "oysterbox";

#define MAX_POINTS 200000
#define MIN_SEG_LEN 0.05f
#define ARC_RES 2.0f

#define NETWORK_TIMEOUT_MS 8000

#define DRAG_SKIP_STEPS 1
#define IDLE_SKIP_STEPS 1
#define GESTURE_LOCKOUT_MS 350

// --- CUSTOM COLORS (RGB565) ---
#define C_BG      0x10C4
#define C_RAPID   0x2AF1
#define C_HUD_BG  0x0861
#define C_HUD_TXT 0xFFFF
#define C_LOG_TXT 0x07E0
#define C_ERR_TXT 0xF800

// UI Colors
#define C_BTN     0x2124
#define C_BTN_ACT 0xF800
#define C_TXT     0xFFFF
#define C_HL      0x07E0

// Tool Color Palette
uint16_t TOOL_COLORS[] = {
    0xF800, 0x07E0, 0xFFE0, 0xF81F,
    0x07FF, 0xFD20, 0x780F, 0xF810
};
#define TOOL_PALETTE_SIZE 8

// =======================================================================================
// HARDWARE PINS
// =======================================================================================
#define LCD_BLK 46
#define LCD_CS 42
#define LCD_WR 40
#define LCD_RD 39
#define LCD_RS 41
#define LCD_D0 18
#define LCD_D1 17
#define LCD_D2 14
#define LCD_D3 13
#define LCD_D4 12
#define LCD_D5 11
#define LCD_D6 10
#define LCD_D7 9
#define LCD_D8 21
#define LCD_D9 8
#define LCD_D10 16
#define LCD_D11 15
#define LCD_D12 7
#define LCD_D13 6
#define LCD_D14 5
#define LCD_D15 38

#define TOUCH_SDA 3
#define TOUCH_SCL 4
#define TOUCH_I2C_ADDR 0x38

#define VIBRATOR_PIN 45
#define BEEPER_PIN 47
#define LEDC_BEEPER_CH 0
#define LEDC_BACKLIGHT_CH 5

#define LCD_W 480
#define LCD_H 320

// NeoPixels
#define PIXEL_PIN 48



// =======================================================================================
// GLOBALS
// =======================================================================================
Arduino_DataBus *bus = new Arduino_ESP32PAR16(
    LCD_RS, LCD_CS, LCD_WR, LCD_RD,
    LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7,
    LCD_D8, LCD_D9, LCD_D10, LCD_D11, LCD_D12, LCD_D13, LCD_D14, LCD_D15);

Arduino_GFX *gfx = new Arduino_ILI9488(bus, -1, 1, false);
Arduino_Canvas *canvas;

struct Vertex { float x, y, z; uint8_t type; };
std::vector<Vertex> modelVertices;
std::vector<String> fileList;

// View State
float camRotY = 0.78f, camRotX = -0.78f, modelScale = 1.0f;
float panX = 0, panY = 0;
float modelCenterX = 0, modelCenterY = 0, modelCenterZ = 0;
float minX, maxX, minY, maxY, minZ, maxZ;
int lastDetectedTool = 0;

// Interaction State
bool inFileMenu = true, memoryFull = false;
bool isInteracting = false;

// Gesture State
struct TouchPoint { int16_t x, y; };
TouchPoint tp[2];
uint8_t touchCount = 0;
unsigned long lastMultiTouchTime = 0;

float startRotX, startRotY, startScale, startPanX, startPanY;
float startDist = 0;
int16_t startX1, startY1, startCX, startCY;

struct Button { int x, y, w, h; String label; int id; };

Adafruit_NeoPixel pixels(6, PIXEL_PIN, NEO_GRB + NEO_KHZ800);


// =======================================================================================
// LOGGING HELPER
// =======================================================================================
int logY = 0;
void resetLog() {
    gfx->fillScreen(C_BG);
    logY = 10;
}

void log(String msg, uint16_t color = C_LOG_TXT) {
    if (logY > 300) {
        gfx->fillRect(0, 0, LCD_W, 320, C_BG);
        logY = 10;
    }
    gfx->setCursor(10, logY);
    gfx->setTextColor(color);
    gfx->setTextSize(1);
    gfx->println(msg);
    logY += 15;
    Serial.println(msg);
}

// =======================================================================================
// HARDWARE
// =======================================================================================
void init_hardware() {

    pinMode(LCD_CS, OUTPUT); digitalWrite(LCD_CS, LOW);
    pinMode(LCD_BLK, OUTPUT); digitalWrite(LCD_BLK, LOW);
    pinMode(VIBRATOR_PIN, OUTPUT); digitalWrite(VIBRATOR_PIN, LOW);

    ledcSetup(LEDC_BACKLIGHT_CH, 5000, 12);
    ledcAttachPin(LCD_BLK, LEDC_BACKLIGHT_CH);
    ledcWrite(LEDC_BACKLIGHT_CH, 0);

    ledcSetup(LEDC_BEEPER_CH, 2000, 8);
    ledcAttachPin(BEEPER_PIN, LEDC_BEEPER_CH);
    ledcWrite(LEDC_BEEPER_CH, 0);
}

void feedback() {
    digitalWrite(VIBRATOR_PIN, HIGH);
    ledcWriteTone(LEDC_BEEPER_CH, 2000);
    delay(20);
    digitalWrite(VIBRATOR_PIN, LOW);
    ledcWriteTone(LEDC_BEEPER_CH, 0);
}

// =======================================================================================
// MULTI-TOUCH DRIVER
// =======================================================================================
void readTouch() {
    touchCount = 0;
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(TOUCH_I2C_ADDR, 1);
    if (!Wire.available()) return;
    uint8_t pts = Wire.read() & 0x0F;
    if (pts == 0 || pts > 2) return;

    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(TOUCH_I2C_ADDR, 14);
    if (Wire.available() < 14) return;

    uint8_t xh1 = Wire.read(); uint8_t xl1 = Wire.read();
    uint8_t yh1 = Wire.read(); uint8_t yl1 = Wire.read();
    Wire.read(); Wire.read();
    uint8_t xh2 = Wire.read(); uint8_t xl2 = Wire.read();
    uint8_t yh2 = Wire.read(); uint8_t yl2 = Wire.read();

    int16_t rawX1 = ((xh1 & 0x0F) << 8) | xl1;
    int16_t rawY1 = ((yh1 & 0x0F) << 8) | yl1;
    tp[0].x = rawY1;
    tp[0].y = 320 - rawX1;
    touchCount = 1;

    if (pts == 2) {
        int16_t rawX2 = ((xh2 & 0x0F) << 8) | xl2;
        int16_t rawY2 = ((yh2 & 0x0F) << 8) | yl2;
        tp[1].x = rawY2;
        tp[1].y = 320 - rawX2;
        touchCount = 2;
    }
}


// =======================================================================================
// WIFI
// =======================================================================================
bool wifiConnect() {
    resetLog();
    log("Initializing WiFi...");
    log("SSID: " + String(WIFI_SSID));

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
        delay(100);
        if((millis() - t0) % 1000 == 0) log("Waiting...");
    }

    if (WiFi.status() == WL_CONNECTED) {
        log("WiFi Connected!");
        log("IP: " + WiFi.localIP().toString());
        delay(500);
        return true;
    }

    log("WiFi Failed!", C_ERR_TXT);
    delay(2000);
    return false;
}

// =======================================================================================
// UI LAYOUT
// =======================================================================================
#define ID_EXIT 0
#define ID_ISO 1
#define ID_TOP 2
#define ID_FRT 3

Button btnExit={10,  5, 70, 30, "BACK", ID_EXIT};
Button btnIso ={90,  5, 70, 30, "ISO",  ID_ISO};
Button btnTop ={170, 5, 70, 30, "TOP",  ID_TOP};
Button btnFrt ={250, 5, 70, 30, "FRT",  ID_FRT};

Button* allBtns[]={&btnExit,&btnIso,&btnTop,&btnFrt};
int btnCount=4;

// =======================================================================================
// 3D ENGINE (FIXED PROJECT FUNCTION)
// =======================================================================================
inline void project(float inX, float inY, float inZ, float cosX, float sinX, float cosY, float sinY, int16_t &outX, int16_t &outY) {
    float x = inX - modelCenterX;
    float y = inY - modelCenterY;
    float z = inZ - modelCenterZ;

    // 3D Rotation
    float y1 = y * cosX - z * sinX;
    float z1 = y * sinX + z * cosX;
    float x2 = x * cosY - z1 * sinY;

    // Project to Screen Space using float/long to prevent immediate overflow
    float projX = (LCD_W / 2.0f) + panX + (x2 * modelScale);
    float projY = (LCD_H / 2.0f) + panY - (y1 * modelScale);

    // FIX: Clamp coordinates to safe int16_t range to prevent wrap-around artifacts
    // 32000 is safely within int16_t (32767) but far enough off-screen for culling
    if (projX > 32000) projX = 32000;
    if (projX < -32000) projX = -32000;
    if (projY > 32000) projY = 32000;
    if (projY < -32000) projY = -32000;

    outX = (int16_t)projX;
    outY = (int16_t)projY;
}

void renderModel() {
    // 1. Sanity Check
    if (modelVertices.size() < 2) return;

    // 2. Pre-calculate rotation math
    float cosX = cos(camRotX), sinX = sin(camRotX);
    float cosY = cos(camRotY), sinY = sin(camRotY);

    // 3. Determine Step Size (Detail level)
    const size_t step = isInteracting ? DRAG_SKIP_STEPS : IDLE_SKIP_STEPS;

    int16_t x1, y1, x2, y2;
    Vertex &vStart = modelVertices[0];

    // Project first point
    project(vStart.x, vStart.y, vStart.z, cosX, sinX, cosY, sinY, x1, y1);

    unsigned long lastYield = millis();

    // LOOP THROUGH VERTICES
    for (size_t i = step; i < modelVertices.size(); i += step) {

        Vertex &v2 = modelVertices[i];
        project(v2.x, v2.y, v2.z, cosX, sinX, cosY, sinY, x2, y2);

        // If project() clamped the values to +/- 32000, the line is infinitely long
        // or way off screen. Drawing a 60,000px line freezes the ESP32.
        // We skip drawing if either point hit the safety clamp limits.
        bool clamped = (x1 <= -32000 || x1 >= 32000 || y1 <= -32000 || y1 >= 32000 ||
                        x2 <= -32000 || x2 >= 32000 || y2 <= -32000 || y2 >= 32000);

        if (!clamped) {
            // Standard Screen Bounds Check (Culling)
            // Only draw if at least one part of the line touches the screen area
            if(!((x1 < -50 && x2 < -50) || (x1 > LCD_W+50 && x2 > LCD_W+50) ||
                 (y1 < -50 && y2 < -50) || (y1 > LCD_H+50 && y2 > LCD_H+50)))
            {
                uint16_t color = (v2.type == 0) ? C_RAPID : TOOL_COLORS[(v2.type - 1) % TOOL_PALETTE_SIZE];
                canvas->drawLine(x1, y1, x2, y2, color);
            }
        }

        // Cycle coordinates
        x1 = x2; y1 = y2;

        // --- FIX 3: BETTER YIELDING ---
        // Don't count segments. Use time. Yield only once every 100ms.
        // This prevents the "5 seconds of sleep" issue.
        if ((millis() - lastYield) > 100) {
            lastYield = millis();
            // Since we disabled the Hardware Watchdog, we don't strictly need this
            // for the dog, but we need it to keep WiFi alive.
            vTaskDelay(1);
        }
    }
}


void drawUI(int activeBtnID = -1) {
    canvas->setTextSize(1);
    for(int i=0; i<btnCount; i++) {
        Button* b = allBtns[i];
        uint16_t bg = (b->id == activeBtnID) ? C_BTN_ACT : C_BTN;
        canvas->fillRoundRect(b->x, b->y, b->w, b->h, 4, bg);
        canvas->drawRoundRect(b->x, b->y, b->w, b->h, 4, WHITE);
        canvas->setTextColor(C_TXT);
        int16_t x1, y1; uint16_t w, h;
        canvas->getTextBounds(b->label, 0, 0, &x1, &y1, &w, &h);
        canvas->setCursor(b->x+(b->w-w)/2, b->y+(b->h-h)/2);
        canvas->print(b->label);
    }

    // Debug HUD
    int hudX = 5, hudY = 40, hudW = 140, hudH = 110;
    canvas->fillRect(hudX, hudY, hudW, hudH, C_HUD_BG);
    canvas->drawRect(hudX, hudY, hudW, hudH, 0x52AA);
    canvas->setTextColor(C_HUD_TXT);
    canvas->setCursor(hudX + 4, hudY + 4);
    canvas->printf("HEAP : %d KB\n", ESP.getFreeHeap() / 1024);
    canvas->setCursor(hudX + 4, canvas->getCursorY() + 2);
    canvas->printf("PSRAM: %.1f MB\n", (float)ESP.getFreePsram() / 1048576.0);
    canvas->drawLine(hudX, canvas->getCursorY()+2, hudX+hudW, canvas->getCursorY()+2, 0x52AA);
    canvas->setCursor(hudX + 4, canvas->getCursorY() + 6);
    canvas->printf("Rot : %.2f, %.2f\n", camRotX, camRotY);
    canvas->setCursor(hudX + 4, canvas->getCursorY() + 2);
    canvas->printf("Pan : %.0f, %.0f\n", panX, panY);
    canvas->setCursor(hudX + 4, canvas->getCursorY() + 2);
    canvas->printf("Zoom: %.2fx\n", modelScale);
    canvas->drawLine(hudX, canvas->getCursorY()+2, hudX+hudW, canvas->getCursorY()+2, 0x52AA);
    canvas->setCursor(hudX + 4, canvas->getCursorY() + 6);
    canvas->printf("Pts : %d\n", modelVertices.size());
    canvas->setCursor(hudX + 4, canvas->getCursorY() + 2);
    float dimW = maxX - minX; if(dimW<0) dimW=0;
    float dimH = maxY - minY; if(dimH<0) dimH=0;
    float dimD = maxZ - minZ; if(dimD<0) dimD=0;
    canvas->printf("Dim : %.0f x %.0f\n", dimW, dimH);
    canvas->setCursor(hudX + 4, canvas->getCursorY() + 2);
    canvas->printf("Z-Dp: %.0f mm\n", dimD);
    canvas->setCursor(hudX + 4, canvas->getCursorY() + 2);
    canvas->setTextColor(YELLOW);
    canvas->printf("Tool: T%d\n", lastDetectedTool);
}

void safeCanvasFillScreen(uint16_t color) {
    Serial.printf("[%lu] safeCanvasFillScreen START\n", millis());

    const int stripHeight = 20;

    for(int y = 0; y < LCD_H; y += stripHeight) {
        int h = min(stripHeight, LCD_H - y);
        canvas->fillRect(0, y, LCD_W, h, color);
        delay(1);
    }

    Serial.printf("[%lu] safeCanvasFillScreen COMPLETE\n", millis());
}

void renderFull() {
    Serial.printf("[%lu] >>> renderFull START\n", millis());
    unsigned long t0 = millis();

    // 1. Clear Screen
    canvas->fillScreen(C_BG);

    // 2. Draw Model & UI
    renderModel();
    drawUI();

    // 3. Flush to LCD
    // With WDT disabled in setup(), this slow operation is now safe.
    Serial.printf("[%lu] canvas->flush START\n", millis());
    canvas->flush();

    Serial.printf("[TIMING] <<< renderFull COMPLETE (%lums)\n", millis() - t0);
}


void handleButton(int id) {
    switch(id) {
        case ID_EXIT: inFileMenu=true; modelVertices.clear(); modelVertices.shrink_to_fit(); break;
        case ID_ISO: camRotX = -0.78; camRotY = 0.78; panX=0; panY=0; modelScale=1.0; break;
        case ID_TOP: camRotX = 0.00; camRotY = 0.00; panX=0; panY=0; modelScale=1.0; break;
        case ID_FRT: camRotX = -1.57;  camRotY = 0.00; panX=0; panY=0; modelScale=1.0; break;
    }
}

// =======================================================================================
// LOADER & PARSER
// =======================================================================================
struct MachineState { float x,y,z; bool abs; bool mm; int plane; int mode; int tool; };

void updateBounds(float x, float y, float z) {
    if(x<minX)minX=x; if(x>maxX)maxX=x; if(y<minY)minY=y; if(y>maxY)maxY=y; if(z<minZ)minZ=z; if(z>maxZ)maxZ=z;
}

float getGVal(String &s, char c, float def) {
    int i=s.indexOf(c); if(i<0) return def;
    int e=i+1; while(e<s.length() && (isdigit(s[e])||s[e]=='.'||s[e]=='-')) e++;
    return s.substring(i+1, e).toFloat();
}

void addPoint(float x, float y, float z, uint8_t type) {
    if(modelVertices.size()>=MAX_POINTS) { memoryFull=true; return; }
    if (modelVertices.size()>0) {
        Vertex &last=modelVertices.back();
        float distSq=pow(x-last.x,2)+pow(y-last.y,2)+pow(z-last.z,2);
        if (distSq<(MIN_SEG_LEN*MIN_SEG_LEN)) return;
    }
    modelVertices.push_back({x, y, z, type});
    updateBounds(x,y,z);
}

void parseArc(float tx, float ty, float tz, float i, float j, float k, MachineState &st) {
    float sx=st.x, sy=st.y, sz=st.z;
    float a1s, a2s, a1e, a2e, off1, off2;
    if(st.plane==1) { a1s=sz; a2s=sx; a1e=tz; a2e=tx; off1=k; off2=i; }
    else if(st.plane==2) { a1s=sy; a2s=sz; a1e=ty; a2e=tz; off1=j; off2=k; }
    else { a1s=sx; a2s=sy; a1e=tx; a2e=ty; off1=i; off2=j; }
    float cenX=a1s+off1; float cenY=a2s+off2;
    float rad=sqrt(off1*off1+off2*off2);
    float angS=atan2(a2s-cenY, a1s-cenX); float angE=atan2(a2e-cenY, a1e-cenX);
    float diff=angE-angS; if(st.mode==2 && diff>=0)diff-=2*PI; if(st.mode==3 && diff<=0)diff+=2*PI;
    int segs=(int)(abs(diff)*rad/ARC_RES); if(segs<1)segs=1;
    float angStep=diff/segs; float linStep=((st.plane==1?ty:st.plane==2?tx:tz)-(st.plane==1?sy:st.plane==2?sx:sz))/segs;
    uint8_t type = (st.tool > 0) ? (st.tool) : 1;
    for(int s=1; s<=segs; s++) {
        float ang=angS+angStep*s; float lin=(st.plane==1?sy:st.plane==2?sx:sz)+linStep*s;
        float c1=cenX+cos(ang)*rad; float c2=cenY+sin(ang)*rad;
        if(st.plane==1) addPoint(c2,lin,c1,type); else if(st.plane==2) addPoint(lin,c1,c2,type); else addPoint(c1,c2,lin,type);
    }
}

void processLine(String &l, MachineState &st, int &nextTool, int &activeTool) {
    l.trim(); l.toUpperCase();
    if (l.startsWith(";") || l.length() == 0) return;
    int tIdx = l.indexOf('T');
    if (tIdx >= 0) nextTool = getGVal(l, 'T', nextTool);
    if (l.indexOf("M6") >= 0) {
        activeTool = nextTool;
        if(activeTool > 250) activeTool = 250;
        if(activeTool < 1) activeTool = 1;
        st.tool = activeTool;
        lastDetectedTool = activeTool;
    }
    if (l.indexOf("G90") >= 0) st.abs = true;
    if (l.indexOf("G91") >= 0) st.abs = false;
    if (l.indexOf("G21") >= 0) st.mm = true;
    if (l.indexOf("G20") >= 0) st.mm = false;
    if (l.indexOf("G17") >= 0) st.plane = 0;
    if (l.indexOf("G18") >= 0) st.plane = 1;
    if (l.indexOf("G19") >= 0) st.plane = 2;
    if (l.indexOf("G0") >= 0)  st.mode = 0;
    if (l.indexOf("G1") >= 0)  st.mode = 1;
    if (l.indexOf("G2") >= 0)  st.mode = 2;
    if (l.indexOf("G3") >= 0)  st.mode = 3;

    if (l.indexOf('X') >= 0 || l.indexOf('Y') >= 0 || l.indexOf('Z') >= 0 ||
        (st.mode > 1 && (l.indexOf('I') >= 0 || l.indexOf('J') >= 0))) {
        float tx = getGVal(l, 'X', st.abs ? st.x : 0) + (st.abs ? 0 : st.x);
        float ty = getGVal(l, 'Y', st.abs ? st.y : 0) + (st.abs ? 0 : st.y);
        float tz = getGVal(l, 'Z', st.abs ? st.z : 0) + (st.abs ? 0 : st.z);
        float sc = st.mm ? 1.0f : 25.4f;
        uint8_t vType = 0;
        if (st.mode > 0) {
            vType = (uint8_t)st.tool;
            if(vType == 0) vType = 1;
        }
        if (st.mode <= 1) {
            addPoint(tx * sc, ty * sc, tz * sc, vType);
        } else {
            float i = getGVal(l, 'I', 0) * sc;
            float j = getGVal(l, 'J', 0) * sc;
            float k = getGVal(l, 'K', 0) * sc;
            MachineState tmp = st;
            tmp.x *= sc; tmp.y *= sc; tmp.z *= sc;
            parseArc(tx * sc, ty * sc, tz * sc, i, j, k, tmp);
        }
        st.x = tx; st.y = ty; st.z = tz;
    }
}

// --- FETCH FROM SERVER ---
void fetchFromServer() {
    DEBUG_POINT("fetchFromServer START");
    unsigned long fetchStart = millis();

    if (!wifiConnect()) {
        DEBUG_POINT("WiFi connect failed");
        return;
    }

    log("Connecting to host...");
    HTTPClient http;
    http.begin(fetchURL);
    http.setConnectTimeout(10000);
    http.setTimeout(10000);

    DEBUG_POINT("Sending GET request");
    log("Sending GET Request...");
    unsigned long requestStart = millis();
    int code = http.GET();
    Serial.printf("[TIMING] GET request took %lums\n", millis() - requestStart);

    if (code > 0) {
        log("HTTP Code: " + String(code));
        int len = http.getSize();
        log("Content-Len: " + String(len));

        if (code == HTTP_CODE_OK) {
            DEBUG_POINT("HTTP OK - starting stream");
            WiFiClient *stream = http.getStreamPtr();

            if (psramFound()) {
                size_t freeMem = ESP.getFreePsram();
                log("Free PSRAM: " + String(freeMem));
                if (freeMem > (MAX_POINTS * sizeof(Vertex) + 100000))
                    modelVertices.reserve(MAX_POINTS);
            }

            camRotX = -0.78; camRotY = 0.78; panX = 0; panY = 0;
            modelVertices.clear(); memoryFull = false;
            minX = 1e9; maxX = -1e9; minY = 1e9; maxY = -1e9; minZ = 1e9; maxZ = -1e9;
            lastDetectedTool = 0;

            MachineState st = {0, 0, 0, false, true, 0, 0, 1};
            int nextTool = 1;
            int activeTool = 1;

            modelVertices.push_back({0, 0, 0, 0});

            int linesProcessed = 0;
            unsigned long lastUpdate = millis();
            unsigned long lastReadTime = millis();
            unsigned long streamStart = millis();

            log("Starting Stream...");

            unsigned long lastData = millis();

            while (true) {
                vTaskDelay(pdMS_TO_TICKS(2));

                if (memoryFull) {
                    log("ERR: Memory Full!", C_ERR_TXT);
                    break;
                }

                size_t avail = stream->available();

                if (avail) {
                    String l = stream->readStringUntil('\n');
                    processLine(l, st, nextTool, activeTool);

                    linesProcessed++;
                    lastData = millis();

                    if (len > 0) len -= (l.length() + 1);
                }
                else {
                    // If no data for 1000ms → treat as END OF FILE (not a timeout)
                    if (millis() - lastData > 1000) {
                        log("Stream ended normally.");
                        break;
                    }
                }

                if (millis() - lastUpdate > 250) {
                    lastUpdate = millis();
                    gfx->fillRect(0, 300, LCD_W, 20, C_BG);
                    gfx->setCursor(10, 315);
                    gfx->printf("Lines: %d  Pts: %d", linesProcessed, modelVertices.size());
                    gfx->setTextColor(YELLOW);
                }
            }


            Serial.printf("[TIMING] Stream download took %lums\n", millis() - streamStart);
            Serial.printf("[TIMING] Lines processed: %d, Vertices: %d\n", linesProcessed, modelVertices.size());

            DEBUG_POINT("Stream complete");
            log("Download Complete.", C_LOG_TXT);
            delay(500);
        } else {
            log("HTTP Error: " + String(code), C_ERR_TXT);
            delay(3000);
            http.end();
            return;
        }
    } else {
        log("Connection Failed!", C_ERR_TXT);
        log("Err: " + http.errorToString(code), C_ERR_TXT);
        delay(3000);
        http.end();
        return;
    }

    unsigned long httpEndStart = millis();
    DEBUG_POINT("http.end()");
    http.end();
    Serial.printf("[TIMING] http.end() took %lums\n", millis() - httpEndStart);

    unsigned long wifiDisconnectStart = millis();
    DEBUG_POINT("WiFi.disconnect()");
    WiFi.disconnect();
    Serial.printf("[TIMING] WiFi.disconnect() took %lums\n", millis() - wifiDisconnectStart);

    unsigned long calcStart = millis();
    modelCenterX = (minX + maxX) / 2;
    modelCenterY = (minY + maxY) / 2;
    modelCenterZ = (minZ + maxZ) / 2;
    float maxD = max(maxX - minX, max(maxY - minY, maxZ - minZ));
    modelScale = (maxD > 0) ? (280.0f / maxD) : 1.0f;
    Serial.printf("[TIMING] Bounds calculation took %lums\n", millis() - calcStart);

    DEBUG_POINT("Starting render after download");
    log("Rendering...", C_LOG_TXT);

    unsigned long renderStart = millis();
    delay(50);

    inFileMenu = false;
    renderFull();  // This has its own timing inside

    Serial.printf("[TIMING] Total render sequence took %lums\n", millis() - renderStart);
    Serial.printf("[TIMING] ==== TOTAL fetchFromServer took %lums ====\n", millis() - fetchStart);

    DEBUG_POINT("fetchFromServer COMPLETE");
}

// --- LOAD FROM SPIFFS ---
void loadAndParseFile(String path) {
    DEBUG_POINT("loadAndParseFile START");
    Serial.println("File: " + path);

    gfx->fillScreen(C_BG);
    gfx->setCursor(20, 150); gfx->setTextSize(2); gfx->setTextColor(GREEN); gfx->print("Loading...");

    if(psramFound()) {
        if(ESP.getFreePsram() > (MAX_POINTS * sizeof(Vertex) + 50000))
            modelVertices.reserve(MAX_POINTS);
    }

    camRotX=-0.78; camRotY=0.78; panX=0; panY=0;
    modelVertices.clear(); memoryFull=false;
    minX=1e9; maxX=-1e9; minY=1e9; maxY=-1e9; minZ=1e9; maxZ=-1e9;
    lastDetectedTool = 0;

    if (!path.startsWith("/")) path = "/" + path;

    DEBUG_POINT("Opening file");
    File f=SPIFFS.open(path, FILE_READ);
    if(!f) {
        DEBUG_POINT("File open FAILED");
        return;
    }
    long fSize=f.size();
    Serial.printf("File size: %ld bytes\n", fSize);

    MachineState st={0,0,0,false,true,0,0,1};
    int nextTool = 1;
    int activeTool = 1;

    modelVertices.push_back({0,0,0,0});
    uint32_t lastYield=millis();
    int lineCount = 0;

    DEBUG_POINT("Parsing file");
    while(f.available()){
        if(memoryFull) break;

        lineCount++;

        if(lineCount % 100 == 0) {
            Serial.printf("[%lu] Parsed %d lines, %d vertices\n", millis(), lineCount, modelVertices.size());
        }

        if(millis()-lastYield > 100) {
            vTaskDelay(pdMS_TO_TICKS(1));
            lastYield=millis();
        }

        String l=f.readStringUntil('\n');
        processLine(l, st, nextTool, activeTool);

        if(f.position()%8192<100) {
            int w=(f.position()*460)/fSize;
            gfx->fillRect(10,200,w,10,GREEN);
        }
    }
    f.close();

    DEBUG_POINT("File parsing complete");
    Serial.printf("Total: %d lines, %d vertices\n", lineCount, modelVertices.size());

    modelCenterX=(minX+maxX)/2; modelCenterY=(minY+maxY)/2; modelCenterZ=(minZ+maxZ)/2;
    float maxD=max(maxX-minX,max(maxY-minY,maxZ-minZ));
    modelScale=(maxD>0)?(280.0f/maxD):1.0f;

    DEBUG_POINT("Starting render after load");
    inFileMenu=false;
    renderFull();

    DEBUG_POINT("loadAndParseFile COMPLETE");
}

void createSampleFile() {
    if(!SPIFFS.exists("/sample.gcode")) {
        File f = SPIFFS.open("/sample.gcode", FILE_WRITE);
        if(f) {
            f.println("G21\nG90\nG0 Z10");
            f.println("T1 M6");
            for(int z=0; z<10; z+=2) {
                f.printf("G1 Z%d F500\n", z);
                f.println("G1 X0 Y0\nG1 X20 Y0\nG1 X20 Y20\nG1 X0 Y20\nG1 X0 Y0");
            }
            f.println("T2 M6");
            f.println("G0 X30 Y30");
            for(int z=0; z<10; z+=2) {
                f.printf("G1 Z%d F500\n", z);
                f.println("G1 X30 Y30\nG1 X50 Y30\nG1 X50 Y50\nG1 X30 Y50\nG1 X30 Y30");
            }
            f.close();
        }
    }
}

// =======================================================================================
// SETUP & LOOP
// =======================================================================================
void setup() {
    Serial.begin(115200);

    // =============================================================
    // WATCHDOG FIX (Based on your provided headers)
    // =============================================================

    // 1. Configure the high-level software watchdog to 60s (prevents software panic)
    esp_task_wdt_init(60, false);

    // 2. Disable Hardware Watchdog for Timer Group 1 (The one causing the crash)
    // We use the exact member names found in your timer_group_struct.h
    TIMERG1.wdtwprotect.wdt_wkey = 0x50D83AA1; // Unlock using the magic number
    TIMERG1.wdtconfig0.wdt_en = 0;             // Disable bit (wdt_en)
    TIMERG1.wdtwprotect.wdt_wkey = 0;          // Lock

    // 3. Disable Timer Group 0 as well (Safety measure)
    TIMERG0.wdtwprotect.wdt_wkey = 0x50D83AA1;
    TIMERG0.wdtconfig0.wdt_en = 0;
    TIMERG0.wdtwprotect.wdt_wkey = 0;

    // =============================================================

    init_hardware();

    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.show();

    gfx->begin();
    gfx->fillScreen(C_BG);
    ledcWrite(LEDC_BACKLIGHT_CH, 4095);

    if(psramFound()) {
        Serial.printf("PSRAM: %d\n", ESP.getFreePsram());
        canvas = new Arduino_Canvas(LCD_W, LCD_H, gfx);
        if(!canvas->begin()) {
            Serial.println("Canvas alloc failed!");
            while(1);
        }
    } else {
        Serial.println("PSRAM ERROR!");
        while(1);
    }

    Wire.begin(TOUCH_SDA, TOUCH_SCL);

    if(SPIFFS.begin(true)) {
        createSampleFile();
        File root = SPIFFS.open("/");
        File f = root.openNextFile();
        while(f){
            String n = String(f.name());
            if (!n.startsWith("/")) n = "/" + n;
            if(n.endsWith(".gcode")) fileList.push_back(n);
            f = root.openNextFile();
        }
    }
}


void loop() {

    static unsigned long lastHeartbeat = 0;

    // Heartbeat every 10 seconds to show we're alive
    if(millis() - lastHeartbeat > 10000) {
        Serial.printf("[%lu] HEARTBEAT - Heap: %d, PSRAM: %d\n",
                      millis(), ESP.getFreeHeap(), ESP.getFreePsram());
        lastHeartbeat = millis();
    }
    readTouch();

    static uint8_t prevTouchCount = 0;

    if (touchCount != prevTouchCount) {
        if (prevTouchCount == 2 && touchCount == 1) {
            lastMultiTouchTime = millis();
        }
        isInteracting = false;
    }
    prevTouchCount = touchCount;

    if (touchCount > 0) {
        if (inFileMenu) {
            if (touchCount == 1) {
              int16_t tx = tp[0].x;
              int16_t ty = tp[0].y;
              int y = 40;
              if (ty > y && ty < y + 25) {
                feedback();
                fetchFromServer();
                return;
              }
              y += 25;
              for (int i = 0; i < fileList.size(); i++) {
                if (ty > y && ty < y + 25) {
                    feedback();
                    loadAndParseFile(fileList[i]);
                    return;
                }
                y += 25;
              }
            }
        } else {
            if (touchCount == 1) {
                int16_t tx = tp[0].x;
                int16_t ty = tp[0].y;
                for(int i=0; i<btnCount; i++) {
                    Button* b = allBtns[i];
                    if(tx>=b->x && tx<=(b->x+b->w) && ty>=b->y && ty<=(b->y+b->h)) {
                        feedback();
                        handleButton(b->id);
                        renderFull();
                        return;
                    }
                }
            }
            if (touchCount == 1 && (millis() - lastMultiTouchTime < GESTURE_LOCKOUT_MS)) {
                renderFull();
                return;
            }
            if (!isInteracting) {
                isInteracting = true;
                startX1 = tp[0].x; startY1 = tp[0].y;
                startRotX = camRotX; startRotY = camRotY;
                if (touchCount == 2) {
                    float dx = tp[0].x - tp[1].x;
                    float dy = tp[0].y - tp[1].y;
                    startDist = sqrt(dx*dx + dy*dy);
                    startScale = modelScale;
                    startCX = (tp[0].x + tp[1].x) / 2;
                    startCY = (tp[0].y + tp[1].y) / 2;
                    startPanX = panX;
                    startPanY = panY;
                }
            } else {
                if (touchCount == 1) {
                    int16_t dx = tp[0].x - startX1;
                    int16_t dy = tp[0].y - startY1;
                    camRotY = startRotY - (dx * 0.01f);
                    camRotX = startRotX + (dy * 0.01f);
                } else if (touchCount == 2) {
                    float dx = tp[0].x - tp[1].x;
                    float dy = tp[0].y - tp[1].y;
                    float currDist = sqrt(dx*dx + dy*dy);
                    if (startDist > 10) modelScale = startScale * (currDist / startDist);
                    int16_t currCX = (tp[0].x + tp[1].x) / 2;
                    int16_t currCY = (tp[0].y + tp[1].y) / 2;
                    panX = startPanX + (currCX - startCX);
                    panY = startPanY + (currCY - startCY);
                }
                renderFull();
            }
        }
    } else {
        if(isInteracting) {
            isInteracting = false;
            if(!inFileMenu) renderFull();
        }
        if (inFileMenu) {
             static long lastMenu = 0;
             if (millis() - lastMenu > 500) {
                lastMenu = millis();
                gfx->fillScreen(C_BG);
                gfx->setCursor(0,0); gfx->setTextColor(ORANGE); gfx->setTextSize(1);
                gfx->println(" Select File:");
                int y = 30;
                gfx->setCursor(10, y);
                gfx->setTextColor(C_HL);
                gfx->println("[Fetch from server]");
                y += 25;
                for (int i=0; i<fileList.size(); i++) {
                    gfx->setCursor(10, y);
                    gfx->setTextColor(WHITE);
                    gfx->print(fileList[i].substring(1));
                    y += 25;
                }
             }
        }
    }
}
