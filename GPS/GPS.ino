/*
Aaron Becker 2.671 Project
March 2024
*/

/****** ENABLE RTK ON THE LINE BELOW (comment out to disable) */
#define RTK_ENABLED_D

#ifdef RTK_ENABLED_D
bool RTK_ENABLED = true;
#else
bool RTK_ENABLED = false;
#endif

#include <SerLCD.h>
#include <WiFi.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_MCP4728.h>
#include <esp32-hal-timer.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

#include "bigFontChars.h"
#include "secrets.h"

SerLCD lcd;
SFE_UBLOX_GNSS GNSS;
Adafruit_MCP4728 dac;
WiFiClient ntripClient;

#define MTOV 1.0 // 1 meter = 1 volt (scaling).
#define VMAX 5.0 // Maximum voltage at the output of the ADC

#define DISP_BOUND (VMAX / MTOV) // Bounded maximunm of sensor value

#define UPDATE_RATE_DISPLAY 2     // Hz
#define UPDATE_RATE_GNSS_STATUS 5 // Hz
#define UPDATE_RATE_ADC 50        // Hz

enum State
{
    GPS_INIT,
    GPS_NORM_FIX,
    GPS_RTK_ACQUIRED,
    GPS_RTK_FIX
};

State currentState = GPS_INIT;

// ECEF position reported by the GPS
struct ECEFStateVector_t
{
    // All positions in meters
    double x = 0;
    double y = 0;
    double z = 0;

    // All velocities in meters/second
    double vx = 0;
    double vy = 0;
    double vz = 0;

    // Accuracy in meters
    float posAcc = -1;
    float velAcc = -1;

    // Compute magnitude of the position vector
    double posMagnitude() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    // Compute magnitude of the velocity vector
    double velMagnitude() const
    {
        return sqrt(vx * vx + vy * vy + vz * vz);
    }

    // Overload the subtraction operator
    ECEFStateVector_t operator-(const ECEFStateVector_t &other) const
    {
        ECEFStateVector_t result;

        // Subtract positions
        result.x = x - other.x;
        result.y = y - other.y;
        result.z = z - other.z;

        // Subtract velocities
        result.vx = vx - other.vx;
        result.vy = vy - other.vy;
        result.vz = vz - other.vz;

        // Take the worst accuracy from the two initial results
        result.posAcc = max(posAcc, other.posAcc); // (posAcc + other.posAcc) / 2.0;
        result.velAcc = max(velAcc, other.velAcc); // (velAcc + other.velAcc) / 2.0;

        return result;
    }
};

volatile ECEFStateVector_t currentPosition;
volatile ECEFStateVector_t startPosition;

struct GNSSState_t
{
    bool isECEFInBounds = 0;
    bool isFixOK = 0;
    bool isRTCMFresh = 0;
    bool isReady = 0; // IsReady is updated dynamically based on whether RTK is enabled or not
    uint8_t siv = 0;  // Satellites In View
    uint8_t fixType = 0;
    // GNSSfix Type:
    // 0: no fix
    // 1: dead reckoning only
    // 2: 2D-fix
    // 3: 3D-fix
    // 4: GNSS + dead reckoning combined
    // 5: time only fix

    uint8_t carrierSolution = 0;
    // Carrier phase range solution status:
    // 0: no carrier phase range solution
    // 1: carrier phase range solution with floating ambiguities
    // 2: carrier phase range solution with fixed ambiguities
}

volatile GNSSState_t GNSSState;
volatile unsigned long lastRTCMDataTime; // Last time we received RTCM data
volatile unsigned int RTCMDataCount;     // Used for counting RTCM update rate
volatile unsigned int GNSSDataCount;     // Used for counting GNSS update rate

// Forward declarations of the ISR functions
void IRAM_ATTR displayUpdateISR();
void IRAM_ATTR GNSSStateUpdateISR();
void IRAM_ATTR ADCUpdateISR();

// Timer handles for each ISR
hw_timer_t *timerDisplayUpdate = NULL;
hw_timer_t *timerGNSSStateUpdate = NULL;
hw_timer_t *timerADCUpdate = NULL;

// Mutex lock for ISRs
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/*
GPS stuff

Line 1:
GPS | (FAIL PONLY RTCM) | (WARM YDAC)
SIV: N, FT: T (RTK)

Things I want to display
- SIV
- Pos accuracy, vel accuracy
- Fix type, RTK fix type
- RTCM datarate and connection status
- GPS solution input frequency
- Screen updates once a second
*/

/****** CALLBACKS AND POINTER FUNCTIONS */

// High precision ECEF data callback
void callback_HPPOSECEF(UBX_NAV_HPPOSEECEF_data_t *pack)
{
    // Check to make sure pointer is valid and position is not invalid
    if (pack == NULL)
        return;

    if (pack->flags.bits.invalidECEF)
    {
        currentPosition->posAcc = -1;
        return;
    }

    // Get accuracy and convert to m
    float accuracy = (float)pack->pAcc;
    accuracy /= 1000.0; // Convert from mm to m

    // Collect the position data
    int32_t ECEFX = pack->ecefX;
    int8_t ECEFXHp = pack->ecefXHp;
    int32_t ECEFY = pack->ecefY;
    int8_t ECEFYHp = pack->ecefYHp;
    int32_t ECEFZ = pack->ecefZ;
    int8_t ECEFZHp = pack->ecefZHp;

    // Assemble the high precision coordinates
    double d_ECEFX = ((double)ECEFX) / 100.0 + ((double)ECEFXHp) / 10000.0;
    double d_ECEFY = ((double)ECEFY) / 100.0 + ((double)ECEFYHp) / 10000.0;
    double d_ECEFZ = ((double)ECEFZ) / 100.0 + ((double)ECEFZHp) / 10000.0;

    currentPosition->x = d_ECEFX;
    currentPosition->y = d_ECEFY;
    currentPosition->z = d_ECEFZ;
    currentPosition->posAcc = accuracy;

    GNSSDataCount++; // Add one to data count
}

// Velocity ECEF data callback
void callback_VELECEF(UBX_NAV_VELECEF_data_t *pack)
{
    // Check to make sure pointer is valid
    if (pack == NULL)
        return;

    // Get accuracy and convert to m
    float accuracy = (float)pack->sAcc;
    accuracy /= 100.0; // Convert from cm to m
    double ecefVX = (double)pack->ecefVX / 100.0;
    double ecefVY = (double)pack->ecefVY / 100.0;
    double ecefVZ = (double)pack->ecefVZ / 100.0;

    currentPosition->vx = ecefVX;
    currentPosition->vy = ecefVY;
    currentPosition->vz = ecefVZ;
    currentPosition->velAcc = accuracy;
}

// PVT data callback (for fixType, carrierType, and SIV)

void callback_PVT(UBX_NAV_PVT_data_t *pack)
{
    // We don't want the low-accuracy NED data, just want the SIV and fix parameters
    GNSSState->siv = pack->numSV;
    GNSSState->isFixOK = pack->flags.bits.gnssFixOK;
    GNSSState->carrierSolution = (uint8_t)pack->flags.bits.carrSoln;
    GNSSState->fixType = pack->fixType;
}

// Timed interrupt for display update (2 Hz)

void IRAM_ATTR displayUpdateISR()
{
    portENTER_CRITICAL_ISR(&timerMux);

    // Line 1: GPS Status, GPS update rate, RTCM update rate
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    lcd.setCursor(0, 0);
    char buf[40]; // Buffer for formatted string
    // Calculating update rates in Hz, assuming counts are reset after each function call
    float gpsUpdateRate = (float)UPDATE_RATE_DISPLAY / GNSSDataCount;
    float rtcmUpdateRate = (float)UPDATE_RATE_DISPLAY / RTCMDataCount;
    // Reset data counts
    GNSSDataCount = 0;
    RTCMDataCount = 0;

    sprintf(buf, "GPS-%s G:%.1fR:%.1fHz",
            (GNSSState->isReady) ? ((RTK_ENABLED) ? "RTK" : "3DO") : "INI", // GPS status
            gpsUpdateRate,                                                  // GPS update rate in Hz
            rtcmUpdateRate);                                                // RTCM update rate in Hz
    buf[20] = '\0';
    lcd.print(buf);

    // Line 2: SIV, fixType, carrierFixType, RTK last packet time (s)
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(0, 1);
    char buf[40]; // Buffer for formatted string
    char ft[3];   // Buffer for fix type
    char cst[3];  // Buffer for carrier solution type
    switch (GNSSState->fixType)
    {
    case 0:
        strcopy(ft, "NF"); // No fix
        break;
    case 1:
        strcopy(ft, "DR"); // Dead reckoning
        break;
    case 2:
        strcopy(ft, "2D");
        break;
    case 3:
        strcopy(ft, "3D");
        break;
    case 4:
        strcopy(ft, "GD"); // GNSS + dead reckoning
        break;
    case 5:
        strcopy(ft, "TO"); // Time only
        break;
    default:
        strcopy(ft, "UN"); // Unknown
        break;
    }

    switch (GNSSState->carrierSolution)
    {
    case 0:
        strcopy(cst, "NS"); // No carrier solution
        break;
    case 1:
        strcopy(cst, "FL"); // Floating solution
        break;
    case 2:
        strcopy(cst, "FX"); // Fixed solution
        break;
    default:
        strcopy(cst, "UN"); // Unknown solution type
        break;
    }

    if (RTK_ENABLED)
    {
        sprintf(buf, "SIV%d FT:%s CST:%s LRD%01.2f",
                GNSSState->siv,
                ft,
                cst,
                ((float)millis() - lastRTCMDataTime) / 1000.0);
    }
    else
    {
        sprintf(buf, "SIV%d FT:%s RTKDIS",
                GNSSState->siv,
                ft);
    }
    buf[20] = '\0';
    lcd.print(buf);

    // Line 3: Accuracies, and velocity magnitude
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(0, 2);
    char buf[40]; // Buffer for formatted string
    double d_vel = currentPosition.velMagnitude();
    sprintf(buf, "hA%01.2f vA%01.2f V%02d.%02d",
            currentPosition->posAcc,
            currentPosition->velAcc,
            (int)d_vel % 100, abs((int)(d_vel * 100) % 100));
    buf[20] = '\0';
    lcd.print(buf);

    // Line 4: Ready state (as first char) & XYZ ECEF Position
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    lcd.setCursor(0, 3);
    double d_ECEFX = currentPosition->x;
    double d_ECEFY = currentPosition->y;
    double d_ECEFZ = currentPosition->z;
    char buf[40]; // Buffer for formatted string
    sprintf(buf, "%cX%02d.%02dY%02d.%02dZ%02d.%02d",
            (GNSSState->isReady) ? 'R' : 'N',
            (int)d_ECEFX % 100, abs((int)(d_ECEFX * 100) % 100),  // X: Last two digits of integer part, first two digits after decimal
            (int)d_ECEFY % 100, abs((int)(d_ECEFY * 100) % 100),  // Y: Same for Y
            (int)d_ECEFZ % 100, abs((int)(d_ECEFZ * 100) % 100)); // Z: Same for Z
    buf[20] = '\0';                                               // Only print first 20 chars
    lcd.print(buf);

    portEXIT_CRITICAL_ISR(&timerMux);
}

// Timed interrupt for GNSS state update (5 Hz)

void IRAM_ATTR GNSSStateUpdateISR()
{
    portENTER_CRITICAL_ISR(&timerMux);

    // Note that SIV, fixType and the like are set in the PVT callback

    // Sanity check the ECEF data of the most recent fix
    GNSSState->isECEFInBounds = currentPosition->posAcc <= 5.0 && currentPosition->velAcc <= 5.0;

    GNSSState->isRTCMFresh = (lastRTCMDataTime - millis()) <= 10000;

    bool oldReady = GNSSState->isReady;
    if (RTK_ENABLED)
    {
        // We care about carrier solution being >0 to indicate RTCM received, and fresh data
        GNSSState->isReady = GNSSState->isFixOk && GNSSState->isECEFInBounds && GNSSState->fixType == 3 && GNSSState->carrierSolution > 0 && GNSSState->isRTCMFresh;
    }
    else
    {
        GNSSState->isReady = GNSSState->isFixOk && GNSSState->isECEFInBounds && GNSSState->fixType == 3;
    }

    // First time entering ready state, reset experiment position
    if (!oldReady && GNSSState->isReady)
    {
        startPosition = currentPosition;
    }

    oldReady = GNSSState->isReady;

    portEXIT_CRITICAL_ISR(&timerMux);
}

// Timed interrupt for ADC update (50 Hz)

void IRAM_ATTR ADCUpdateISR()
{
    portENTER_CRITICAL_ISR(&timerMux);

    // If GNSS is ready & positions are valid
    if (GNSSState->isReady && currentPosition->posAcc != -1 && startPosition->posAcc != -1)
    {
        // dPX, dPY, dPZ, Vel -> A, B, C, D
        ECEFStateVector_t diff = currentPosition - startPosition; // Find difference in positions

        double mappedX = mapDouble(constrainDouble(abs(diff.x), 0, DISP_BOUND), 0, DISP_BOUND, 0, 4095);
        double mappedY = mapDouble(constrainDouble(abs(diff.y), 0, DISP_BOUND), 0, DISP_BOUND, 0, 4095);
        double mappedZ = mapDouble(constrainDouble(abs(diff.z), 0, DISP_BOUND), 0, DISP_BOUND, 0, 4095);

        mcp.setChannelValue(MCP4728_CHANNEL_A, mappedX);
        mcp.setChannelValue(MCP4728_CHANNEL_B, mappedY);
        mcp.setChannelValue(MCP4728_CHANNEL_C, mappedZ);

        double constrainedVel = constrainDouble(currentPosition.velMagnitude(), 0, DISP_BOUND);
        uint16_t mappedVel = mapDouble(constrainedVel, 0, DISP_BOUND, 0, 4095);
        mcp.setChannelValue(MCP4728_CHANNEL_D, mappedVel);
    }
    else
    {
        // Output test pattern on DAC if GPS is not ready (triangle wave)
        mcp.setChannelValue(MCP4728_CHANNEL_A, millis() % 4095);
        mcp.setChannelValue(MCP4728_CHANNEL_B, (millis() + 1024) % 4095);
        mcp.setChannelValue(MCP4728_CHANNEL_C, (millis() + 2048) % 4095);
        mcp.setChannelValue(MCP4728_CHANNEL_D, (millis() + 3072) % 4095);
    }

    portEXIT_CRITICAL_ISR(&timerMux);
}

// Function to map double values
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to constrain double values
double constrainDouble(double value, double min, double max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Aaron Becker 2.671");
    Wire.begin();          // Join I2C bus
    Wire.setClock(400000); // set I2C SCL to High Speed Mode of 400kHz

    pinMode(18, OUTPUT);

    // Interrogate one device at a time, and check whether they have initialized correctly
    lcd.begin(Wire);
    digitalWrite(18, HIGH); // Status LED on
    delay(1000);
    lcd.setBacklight(50, 50, 50); // Set backlight to bright white
    lcd.setContrast(5);           // Set contrast. Lower to 0 for higher contrast.

    lcd.clear(); // Clear the display - this moves the cursor to home position as well
    lcd.setCursor(0, 0);
    lcd.print("Aaron Becker 2.671");
    lcd.setCursor(0, 1);
    lcd.print("System Starting...");

    lcd.saveSplash(); // Save this current text as the splash screen at next power on

    lcd.enableSplash();          // This will cause the splash to be displayed at power on
    lcd.disableSystemMessages(); // Disable system messages like contrast

    // Assign bigFont segments write numbers
    lcd.createChar(0, C_LT);
    lcd.createChar(1, C_UB);
    lcd.createChar(2, C_RT);
    lcd.createChar(3, C_LL);
    lcd.createChar(4, C_LB);
    lcd.createChar(5, C_LR);
    lcd.createChar(6, C_UMB);
    lcd.createChar(7, C_LMB);

    // Now, proceed to hardware check
    lcd.setCursor(0, 0);
    lcd.print("Hardware Check.");

    if (!GNSS.begin(Wire))
        error("GPS Fail");
    lcd.print(".");
    delay(100);

    if (sizeof(double) != 8) // Check double is 64-bit
        Serial.println(F("double is not 64-bit. ECEF resolution may be limited!"));

    GNSS.setI2COutput(COM_TYPE_UBX);                                                // Turn off NMEA noise
    GNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.

    GNSS.setNavigationFrequency(20); // Set output in Hz (Max navigation frequency)

    GNSS.saveConfiguration();

    // Setup callback functions
    GNSS.setAutoHPPOSECEFcallbackPtr(&callback_HPPOSECEF);
    GNSS.setAutoVELECEFcallbackPtr(&callback_VELECEF);
    GNSS.setAutoPVTcallbackPtr(&callback_PVT);

    if (!mcp.begin())
        error("MCP4728 DAC Fail");
    lcd.print(".");
    delay(100);
    // Initial voltage spread on DAC
    mcp.setChannelValue(MCP4728_CHANNEL_A, 4095);
    mcp.setChannelValue(MCP4728_CHANNEL_B, 2048);
    mcp.setChannelValue(MCP4728_CHANNEL_C, 1024);
    mcp.setChannelValue(MCP4728_CHANNEL_D, 0);

    // Assuming CPU frequency is 240MHz
    // Prescaler calculation: We choose a prescaler to achieve a 1MHz timer frequency (1 tick = 1us)
    // For a 240MHz CPU, prescaler = 240
    unsigned int prescaler = getCpuFreqMHz();

    // Timer 0 for display update at 2Hz (0.5 seconds)
    timerDisplayUpdate = timerBegin(0, prescaler, true);
    timerAttachInterrupt(timerDisplayUpdate, &displayUpdateISR, true);
    timerAlarmWrite(timerDisplayUpdate, (1000000 / UPDATE_RATE_DISPLAY), true); // 0.5 seconds in microseconds
    timerAlarmEnable(timerDisplayUpdate);

    // Timer 1 for GNSS state update at 5Hz (0.2 seconds)
    timerGNSSStateUpdate = timerBegin(1, prescaler, true);
    timerAttachInterrupt(timerGNSSStateUpdate, &GNSSStateUpdateISR, true);
    timerAlarmWrite(timerGNSSStateUpdate, (1000000 / UPDATE_RATE_GNSS_STATUS), true); // 0.2 seconds in microseconds
    timerAlarmEnable(timerGNSSStateUpdate);

    // Timer 2 for ADC update at 50Hz (0.02 seconds)
    timerADCUpdate = timerBegin(2, prescaler, true);
    timerAttachInterrupt(timerADCUpdate, &ADCUpdateISR, true);
    timerAlarmWrite(timerADCUpdate, (1000000 / UPDATE_RATE_ADC), true); // 0.02 seconds in microseconds
    timerAlarmEnable(timerADCUpdate);

    // Immediately grab mutex so it does't get overwritten
    portENTER_CRITICAL(&timerMux);

    // Start wifi connection
    if (RTK_ENABLED)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi connecting...");

        if (ntripClient.connected() == false)
        {
            if (ntripClient.connect(casterHost, casterPort) == false)
            {
                error("WiFi FailConn");
            }
            else
            {
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Connected to:");
                lcd.setCursor(0, 1);
                lcd.print(casterHost);
                lcd.print(":");
                lcd.print(casterPort);
                lcd.setCursor(0, 2);
                lcd.print("Requesting NTRIP...");
                const int SERVER_BUFFER_SIZE = 512;
                char serverRequest[SERVER_BUFFER_SIZE];

                snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
                         mountPoint);

                char credentials[512];
                if (strlen(casterUser) == 0)
                {
                    strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
                }
                else
                {
                    // Pass base64 encoded user:pw
                    char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; // The ':' takes up a spot
                    snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

                    lcd.setCursor(0, 3);
                    lcd.print("Sending credentials");

                    // Encode with nfriendly library
                    int encodedLen = base64_enc_len(strlen(userCredentials));
                    char encodedCredentials[encodedLen];                                         // Create array large enough to house encoded data
                    base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); // Note: Input array is consumed
                }

                strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
                strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);
                ntripClient.write(serverRequest, strlen(serverRequest));
            }

            // Wait for response
            unsigned long timeout = millis();
            while (ntripClient.available() == 0)
            {
                if (millis() - timeout > 5000)
                {
                    ntripClient.stop();
                    error("Ntrip RTimeout");
                    return;
                }
                delay(10);
            }

            // Check reply
            bool connectionSuccess = false;
            char response[512];
            int responseSpot = 0;
            while (ntripClient.available())
            {
                if (responseSpot == sizeof(response) - 1)
                    break;

                response[responseSpot++] = ntripClient.read();
                if (strstr(response, "200") > 0) // Look for 'ICY 200 OK'
                    connectionSuccess = true;
                if (strstr(response, "401") > 0) // Look for '401 Unauthorized'
                {
                    Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
                    connectionSuccess = false;
                }
            }
            response[responseSpot] = '\0';

            lcd.clear();
            lcd.setCursor(0, 0);
            if (connectionSuccess == false)
            {
                lcd.print(F("Failed to connect to "));
                lcd.setCursor(0, 1);
                lcd.print(casterHost);
                lcd.print(F(": "));
                lcd.setCursor(0, 2);
                lcd.print(response);
                delay(5000);
                error("Ntrip BadCred") return;
            }
            else
            {
                lcd.print(F("Connected! To:"));
                lcd.setCursor(0, 1);
                Serial.println(casterHost);
                delay(1000);
                lastRTCMDataTime = millis(); // Reset timeout
            }
        }
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    BFwriteString("2.671", 0, 0);
    lcd.setCursor(0, 2);
    lcd.print("By Aaron Becker");
    lcd.setCursor(0, 4);
    lcd.print("Hware Check OK.");
    delay(2000);
    lcd.clear();
    portEXIT_CRITICAL(&timerMux); // Allow interrupts to start running
}

void loop()
{
    static unsigned long lastStatusCheckTime = 0;
    static unsigned long lastPosCheckTime = 0;
    unsigned long currentMillis = millis();

    // Prevent interrupts while running these functions
    portENTER_CRITICAL(&timerMux);
    GNSS.checkUblox(); // Check for new data data
    GNSS.checkCallbacks();
    portEXIT_CRITICAL(&timerMux);

    if (RTK_ENABLED)
    {
        if (ntripClient.connected() == true)
        {
            uint8_t rtcmData[512 * 4]; // Most incoming data is around 500 bytes but may be larger
            rtcmCount = 0;

            // Print any available RTCM data
            while (ntripClient.available())
            {
                // Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
                rtcmData[rtcmCount++] = ntripClient.read();
                if (rtcmCount == sizeof(rtcmData))
                    break;
            }

            if (rtcmCount > 0)
            {
                lastRTCMDataTime = millis();
                RTCMDataCount++;

                // Push RTCM to GNSS module over I2C
                GNSS.pushRawData(rtcmData, rtcmCount, false);
            }
        }

        // Close socket if we don't have new data for 10s
        if (millis() - lastRTCMDataTime > 10000)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("RTCM Timeout!");
            lcd.setCursor(0, 1);
            lcd.print("RTK Disabling");
            lcd.setCursor(0, 2);
            lcd.print("Reset to reconnect");
            delay(5000);

            if (ntripClient.connected() == true)
                ntripClient.stop();
            return;
            RTK_ENABLED = false;
        }
    }

    delay(20); // let's not hammer too hard on the I2C bus
}

void hang()
{
    while (true)
    {
        digitalWrite(18, HIGH);
        delay(1000);
        digitalWrite(18, LOW);
        delay(1000);
    }
}

void error(String message)
{
    lcd.setFastBacklight(0xFF0000);
    lcd.setCursor(0, 0);
    lcd.print("Fatal Error:");
    lcd.setCursor(0, 1);
    lcd.print(message);
    hang();
}

// BIGFONT CODE
int BFwriteString(String str, int xPos, int yPos)
{
    int strLen = str.length() + 1; // optimize gang!

    char charBuffer[strLen];
    str.toCharArray(charBuffer, strLen);

    for (int i = 0; i < strLen; i++)
    {
        // Serial.println(charBuffer[i]);
        xPos = BFwriteChar(charBuffer[i], xPos, yPos);
    }

    return xPos; // returns new pos
}

int BFwriteChar(char tW, int offsetX, int offsetY)
{
    if (tW >= 65 && tW <= 90)
    {
        tW = tolower(tW);
    }
    boolean halfWidthChar = false;
    switch (tW)
    { // lower case char, will only affect letters
    case '0':
    case 'o':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(1);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(3);
        lcd.writeChar(4);
        lcd.writeChar(5);
        break;
    case '1':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(1);
        lcd.writeChar(2);
        lcd.setCursor(offsetX + 1, offsetY + 1);
        lcd.writeChar(255);
        break;
    case '2':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(6);
        lcd.writeChar(6);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(3);
        lcd.writeChar(7);
        lcd.writeChar(7);
        break;
    case '3':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(6);
        lcd.writeChar(6);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(7);
        lcd.writeChar(7);
        lcd.writeChar(5);
        break;
    case '4':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(3);
        lcd.writeChar(4);
        lcd.writeChar(2);
        lcd.setCursor(offsetX + 2, offsetY + 1);
        lcd.writeChar(255);
        break;
    case '5':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(6);
        lcd.writeChar(6);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(7);
        lcd.writeChar(7);
        lcd.writeChar(5);
        break;
    case '6':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(6);
        lcd.writeChar(6);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(3);
        lcd.writeChar(7);
        lcd.writeChar(5);
        break;
    case '7':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(1);
        lcd.writeChar(1);
        lcd.writeChar(2);
        lcd.setCursor(offsetX + 1, offsetY + 1);
        lcd.writeChar(0);
        break;
    case '8':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(6);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(3);
        lcd.writeChar(7);
        lcd.writeChar(5);
        break;
    case '9':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(6);
        lcd.writeChar(2);
        lcd.setCursor(offsetX + 2, offsetY + 1);
        lcd.writeChar(255);
        break;
    case 'a':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(6);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(254);
        lcd.writeChar(255);
        break;
    case 'b':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(6);
        lcd.writeChar(5);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(7);
        lcd.writeChar(2);
        break;
    case 'c':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(1);
        lcd.writeChar(1);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(3);
        lcd.writeChar(4);
        lcd.writeChar(4);
        break;
    case 'd':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(1);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(4);
        lcd.writeChar(5);
        break;
    case 'e':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(6);
        lcd.writeChar(6);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(7);
        lcd.writeChar(7);
        break;
    case 'f':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(6);
        lcd.writeChar(6);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        break;
    case 'g':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(1);
        lcd.writeChar(1);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(3);
        lcd.writeChar(4);
        lcd.writeChar(2);
        break;
    case 'h':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(4);
        lcd.writeChar(255);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(254);
        lcd.writeChar(255);
        break;
    case 'i':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(1);
        lcd.writeChar(255);
        lcd.writeChar(1);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(4);
        lcd.writeChar(255);
        lcd.writeChar(4);
        break;
    case 'j':
        lcd.setCursor(offsetX + 2, offsetY);
        lcd.writeChar(255);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(4);
        lcd.writeChar(4);
        lcd.writeChar(5);
        break;
    case 'k':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(4);
        lcd.writeChar(5);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(254);
        lcd.writeChar(2);
        break;
    case 'l':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(4);
        lcd.writeChar(4);
        break;
    case 'm':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(3);
        lcd.writeChar(5);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(254);
        lcd.writeChar(254);
        lcd.writeChar(255);
        break;
    case 'n':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(2);
        lcd.writeChar(254);
        lcd.writeChar(255);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(254);
        lcd.writeChar(3);
        lcd.writeChar(5);
        break;
    case 'p':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(6);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        break;
    case 'q':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(1);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(3);
        lcd.writeChar(4);
        lcd.writeChar(255);
        lcd.writeChar(4);
        break;
    case 'r':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(6);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(255);
        lcd.writeChar(254);
        lcd.writeChar(2);
        break;
    case 's':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(0);
        lcd.writeChar(6);
        lcd.writeChar(6);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(7);
        lcd.writeChar(7);
        lcd.writeChar(5);
        break;
    case 't':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(1);
        lcd.writeChar(255);
        lcd.writeChar(1);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(254);
        lcd.writeChar(255);
        break;
    case 'u':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(254);
        lcd.writeChar(255);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(3);
        lcd.writeChar(4);
        lcd.writeChar(5);
        break;
    case 'v':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(3);
        lcd.writeChar(254);
        lcd.writeChar(254);
        lcd.writeChar(5);
        lcd.setCursor(offsetX + 1, offsetY + 1);
        lcd.writeChar(2);
        lcd.writeChar(0);
        break;
    case 'w':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.writeChar(254);
        lcd.writeChar(254);
        lcd.writeChar(255);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(3);
        lcd.writeChar(0);
        lcd.writeChar(2);
        lcd.writeChar(5);
        break;
    case 'x':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(3);
        lcd.writeChar(4);
        lcd.writeChar(5);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(0);
        lcd.writeChar(254);
        lcd.writeChar(2);
        break;
    case 'y':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(3);
        lcd.writeChar(4);
        lcd.writeChar(5);
        lcd.setCursor(offsetX + 1, offsetY + 1);
        lcd.writeChar(255);
        break;
    case 'z':
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(1);
        lcd.writeChar(6);
        lcd.writeChar(5);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(0);
        lcd.writeChar(7);
        lcd.writeChar(4);
        break;
    case '?': // question mark?
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(1);
        lcd.writeChar(6);
        lcd.writeChar(2);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(254);
        lcd.writeChar(7);
        break;
    case '!': // Exclamation mark?
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(255);
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(7);
        break;
    // HALF WIDTH CHARACTERS
    case '.':
        halfWidthChar = true;
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(4); // CLB
        break;
    case ':':
        halfWidthChar = true;
        lcd.setCursor(offsetX, offsetY);
        lcd.writeChar(1); // CUB
        lcd.setCursor(offsetX, offsetY + 1);
        lcd.writeChar(4); // CLB
        break;
    }
    return offsetX + ((halfWidthChar) ? 2 : 4); // return new xPos
}
