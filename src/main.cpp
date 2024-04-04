/*
Gamepad project code
Dependancies:


1/20/2024
Author: Quinlan Stewart
Built upon "test"

*/

#include <BleConnectionStatus.h>
#include <BleCompositeHID.h>
#include <XboxGamepadDevice.h>
#include <esp_timer.h>
#include <esp_bt.h>

//ABXY BUTTONS
#define A_BUTTON 23 // A
#define B_BUTTON 21  // B
#define Y_BUTTON 22  // Y
#define X_BUTTON 19  // X

//TRIGGERS AND BUMPERS
#define RT_BUTTON 32 // Right trigger
#define RB_BUTTON 33 // Right bumper
#define LT_BUTTON 13 // Left trigger
#define LB_BUTTON 12 // Left bumper

//DPAD BUTTONS
#define DPAD_NORTH 17
#define DPAD_SOUTH 15
#define DPAD_EAST 4
#define DPAD_WEST 16

//MENU BUTTONS
#define START_BUTTON_PIN 26
#define SELECT_BUTTON_PIN 14
#define XB_BUTTON 27

//JOYSTICKS BUTTONS
#define R3_BUTTON 18 // Right joystick button
#define L3_BUTTON 5 // Left joystick button

//JOYSTICKS
#define LEFT_VRX_JOYSTICK 39
#define LEFT_VRY_JOYSTICK 36
#define RIGHT_VRX_JOYSTICK 35
#define RIGHT_VRY_JOYSTICK 34

uint16_t ledPin = 2; // LED connected to digital pin 13

uint16_t joypins[4] = {
        LEFT_VRX_JOYSTICK, 
        LEFT_VRY_JOYSTICK, 
        RIGHT_VRX_JOYSTICK, 
        RIGHT_VRY_JOYSTICK,
    };
#define num_joypins 4

uint16_t dpadpins[4] = {
        DPAD_NORTH, 
        DPAD_SOUTH, 
        DPAD_EAST, 
        DPAD_WEST,
    };
#define num_dpadpins 4

//array of button pin numbers
uint16_t buttonsPins[11] = {
        A_BUTTON, 
        B_BUTTON, 
        X_BUTTON, 
        Y_BUTTON,
        RB_BUTTON, 
        LB_BUTTON, 
        START_BUTTON_PIN, 
        SELECT_BUTTON_PIN,
        XB_BUTTON, 
        R3_BUTTON, 
        L3_BUTTON,
    };

//array of codes for which buttons are pressed
//IMPORTANT: this array must be in the same order as "buttonsPins" array
uint16_t buttonsCodes[11] = {
        XBOX_BUTTON_A, 
        XBOX_BUTTON_B, 
        XBOX_BUTTON_X, 
        XBOX_BUTTON_Y, 
        XBOX_BUTTON_RB,
        XBOX_BUTTON_LB, 
        XBOX_BUTTON_START,
        XBOX_BUTTON_SELECT,
        XBOX_BUTTON_HOME,
        XBOX_BUTTON_RS,
        XBOX_BUTTON_LS, 
    };

#define num_buttons 11


//Joystick analog values, use type "int" to store large signed numbers
int leftJoyXraw = 0;  
int leftJoyYraw = 0;
int rightJoyXraw = 0;
int rightJoyYraw = 0;

int leftJoyX = 0;
int leftJoyY = 0;
int rightJoyX = 0;
int rightJoyY = 0;

#define offset_samples 50
#define deadzone 800
int leftXoffset = 0;
int leftYoffset = 0;
int rightXoffset = 0;
int rightYoffset = 0;

float stickangle = 0.00;
long magnitude = 0.00;
int startflag = 0;

double begin = 0.00;
double end = 0.00;
double time_spent = 0.00;


//used to calculate cardinal direction
short d_north = 0;
short d_south = 0;
short d_east = 0;
short d_west = 0;
int pressed;
int last_pressed;

//For blinking led
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;   
unsigned long interval = 250;


XboxGamepadDevice *gamepad;
BleCompositeHID compositeHID("Solar Gamepad", "Quinlan-S");

void OnVibrateEvent(XboxGamepadOutputReportData data)
{
    if(data.weakMotorMagnitude > 0 || data.strongMotorMagnitude > 0){
        digitalWrite(ledPin, LOW);
    } else {
        digitalWrite(ledPin, HIGH);
    }
    Serial.println("Vibration event. Weak motor: " + String(data.weakMotorMagnitude) + " Strong motor: " + String(data.strongMotorMagnitude));
}

void testButtons();
void testPads();
void testTriggers();
void testThumbsticks();


void setup()
{
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT); // sets the digital pin as output

    // Uncomment one of the following two config types depending on which controller version you want to use
    // The XBox series X controller only works on linux kernels >= 6.5
    
    //XboxOneSControllerDeviceConfiguration* config = new XboxOneSControllerDeviceConfiguration();
    XboxSeriesXControllerDeviceConfiguration* config = new XboxSeriesXControllerDeviceConfiguration();

    // The composite HID device pretends to be a valid Xbox controller via vendor and product IDs (VID/PID).
    // Platforms like windows/linux need this in order to pick an XInput driver over the generic BLE GATT HID driver. 
    BLEHostConfiguration hostConfig = config->getIdealHostConfiguration();
    //hostConfig.setVid(0x02045E);
    hostConfig.setPid(0x0B13);
    Serial.println("Using VID source: " + String(hostConfig.getVidSource(), HEX));
    Serial.println("Using VID: " + String(hostConfig.getVid(), HEX));
    Serial.println("Using PID: " + String(hostConfig.getPid(), HEX));
    Serial.println("Using GUID version: " + String(hostConfig.getGuidVersion(), HEX));
    Serial.println("Using serial number: " + String(hostConfig.getSerialNumber()));
    // Set up gamepad
    gamepad = new XboxGamepadDevice(config);

    // Set up vibration event handler
    FunctionSlot<XboxGamepadOutputReportData> vibrationSlot(OnVibrateEvent);
    gamepad->onVibrate.attach(vibrationSlot);

    // Add all child devices to the top-level composite HID device to manage them
    compositeHID.addDevice(gamepad);
    //esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12);
    Serial.println("POWER LEVEL: " + String(esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV)));
    
    // Start the composite HID device to broadcast HID reports
    Serial.println("begining bluetooth device...");
    compositeHID.begin(hostConfig);
    

    //setup pins
    for(int i = 0; i < num_buttons; i++){
    pinMode(buttonsPins[i], INPUT_PULLUP);
    };

    //setup joystick pins
    for(int i = 0; i < num_joypins; i++){
    pinMode(joypins[i], INPUT_PULLUP);
    };

    //setup DPAD pins
    for(int i = 0; i < num_dpadpins; i++){
    pinMode(dpadpins[i], INPUT_PULLUP);
    };

    pinMode(RT_BUTTON, INPUT_PULLUP);
    pinMode(LT_BUTTON, INPUT_PULLUP);

    //Get analog trigger offset to handle stick drift    

    for( int i = 0; i < offset_samples; i++){
        leftXoffset  += analogRead(LEFT_VRX_JOYSTICK) - 2048;
        leftYoffset  += analogRead(LEFT_VRY_JOYSTICK) - 2048;
        rightXoffset += analogRead(RIGHT_VRX_JOYSTICK) - 2048;
        rightYoffset += analogRead(RIGHT_VRY_JOYSTICK) - 2048;
    }

    leftXoffset  /= offset_samples;
    leftYoffset  /= offset_samples;
    rightXoffset /= offset_samples;
    rightYoffset /= offset_samples;

    
    Serial.println("LX: " + String(leftXoffset));
    Serial.println("LY: " + String(leftYoffset));
    Serial.println("RX: " + String(rightXoffset));
    Serial.println("RY: " + String(rightYoffset));

    while(!compositeHID.isConnected()){
        if (millis() - previousMillis > interval) 
        {
            // save the last time you blinked the LED
            previousMillis += interval;  

            // if the LED is off turn it on and vice-versa:
            if (ledState == LOW)
            {
            ledState = HIGH;
            }
            else
            {
            ledState = LOW;
            }

            // set the LED with the ledState of the variable:
            digitalWrite(ledPin, ledState);
        }
    }
    digitalWrite(ledPin, LOW);
}

void loop()
{
    if(compositeHID.isConnected()){

        //*=========LATENCY TIMER============//
            //comment this out unless debugging
            begin = esp_timer_get_time();
        //==================================//*/

        //=======================ANALOG TRIGGER HANDLING================================//
        //TIME KEEPING FOR LATENCY CALCULATING
        

        leftJoyXraw = analogRead(LEFT_VRX_JOYSTICK) - leftXoffset;
        leftJoyYraw = analogRead(LEFT_VRY_JOYSTICK) - leftYoffset;
        rightJoyXraw = analogRead(RIGHT_VRX_JOYSTICK) - rightXoffset;
        rightJoyYraw = analogRead(RIGHT_VRY_JOYSTICK)- rightYoffset;

        if(leftJoyXraw <= 0 - leftXoffset){ leftJoyXraw = 0;}
        if(leftJoyYraw <= 0 - leftYoffset){ leftJoyYraw = 0;}
        if(rightJoyXraw <= 0 - rightXoffset){ rightJoyXraw = 0;}
        if(rightJoyYraw <= 0 - rightYoffset){ rightJoyYraw = 0;}
        if(leftJoyXraw >= 4095 - leftXoffset){ leftJoyXraw = 4095;}
        if(leftJoyYraw >= 4095 - leftYoffset){ leftJoyYraw = 4095;}
        if(rightJoyXraw >= 4095 - rightXoffset){ rightJoyXraw = 4095;}
        if(rightJoyYraw >= 4095 - rightYoffset){ rightJoyYraw = 4095;}


        /*
        MOVED TO MAGNITUDE CALCULATIONS
        //Code for enforcing deadzone for triggers (helps reduce constant packet updates when triggers are idle)
        if((leftJoyXraw > 2048 - deadzone && leftJoyXraw < 2048 + deadzone) && (leftJoyYraw > 2048 - deadzone && leftJoyYraw < 2048 + deadzone ) )
        {   leftJoyXraw = 2048;
            leftJoyYraw = 2048;}
    
        if(rightJoyXraw > 2048 - deadzone && rightJoyXraw < 2048 + deadzone && (rightJoyYraw > 2048 - deadzone && rightJoyYraw < 2048 + deadzone ) )
        {   rightJoyXraw = 2048;
            rightJoyYraw = 2048;}
        */
        
        //Compute joysticks mapped value
        leftJoyX = map(leftJoyXraw, 0, 4095, 32768, -32737);
        leftJoyY = map(leftJoyYraw, 0, 4095, 32768, -32737);
        rightJoyX = map(rightJoyXraw, 0, 4095, 32768, -32737);
        rightJoyY = map(rightJoyYraw, 0, 4095, 32768, -32737);

        //Use trig to map maximum bounds to circle of radius 32737

        //Mapping Left stick
        stickangle = atan2( leftJoyY, leftJoyX);                 // in radians
        magnitude = sqrt( leftJoyX*leftJoyX + leftJoyY*leftJoyY - 1);    // Pythagoras
        if(magnitude > 32737) magnitude = 32737;
        //enforcing deadzone
        if(magnitude < deadzone) magnitude = 0;
        int leftJoyXmapped = (float)magnitude * cos( stickangle );
        int leftJoyYmapped = (float)magnitude * sin( stickangle );

        //Mapping right stick 
        stickangle = atan2( rightJoyY, rightJoyX);                 // in radians
        magnitude = sqrt( rightJoyX*rightJoyX + rightJoyY*rightJoyY - 1);    // Pythagoras
        if(magnitude > 32737) magnitude = 32737;
        //enforcing deadzone
        if(magnitude < deadzone) magnitude = 0;
        int rightJoyXmapped = (float)magnitude * cos( stickangle );
        int rightJoyYmapped = (float)magnitude * sin( stickangle );

        if(startflag == 0){
            gamepad->setLeftThumb(0, 0);
            gamepad->setRightThumb(0, 0);
            startflag = 1;
        }
        
        gamepad->setLeftThumb(leftJoyXmapped, leftJoyYmapped);
        gamepad->setRightThumb(rightJoyXmapped, rightJoyYmapped);
        
    //===========================SIMPLE BUTTON HANDLING===============================//
        //read simple buttons
        for(int i = 0; i < num_buttons; i++){
            if(!digitalRead(buttonsPins[i])){
                gamepad->press(buttonsCodes[i]);
            }
            else{
                gamepad->release(buttonsCodes[i]);
            }
        };
    
    //===========================DPAD HANDLING========================================//
        d_north = 0;
        d_east = 0;
        d_south = 0;
        d_west = 0;

        if(!digitalRead(DPAD_NORTH)){d_north = 1;}
        if(!digitalRead(DPAD_SOUTH)){d_south = -1;}
        if(!digitalRead(DPAD_EAST)){d_east = -1;}
        if(!digitalRead(DPAD_WEST)){d_west = 1;}

        d_north += d_south;
        d_west += d_east;
        last_pressed = pressed;
        pressed = 0;

        if(d_north == 1 && d_west == 1) { pressed = XBOX_BUTTON_DPAD_NORTHWEST; }
        if(d_north == 1 && d_west == 0) { pressed = XBOX_BUTTON_DPAD_NORTH; }
        if(d_north == 1 && d_west == -1) { pressed = XBOX_BUTTON_DPAD_NORTHEAST; }

        if(d_north == 0 && d_west == 1) { pressed = XBOX_BUTTON_DPAD_WEST; }
        if(d_north == 0 && d_west == -1) { pressed = XBOX_BUTTON_DPAD_EAST; }

        if(d_north == -1 && d_west == 1) { pressed = XBOX_BUTTON_DPAD_SOUTHWEST; }
        if(d_north == -1 && d_west == 0) { pressed = XBOX_BUTTON_DPAD_SOUTH; }
        if(d_north == -1 && d_west == -1) { pressed = XBOX_BUTTON_DPAD_SOUTHEAST; }

        
        if(d_north == 0 && d_west == 0) {
            if(last_pressed != 0){
            gamepad->releaseDPadDirection(last_pressed);
            }
        }
        else if(last_pressed != pressed) {
            if(last_pressed != 0){
            gamepad->releaseDPadDirection(last_pressed);
            }
            gamepad->pressDPadDirection(pressed);

        }

    //===========================TRIGGER HANDLING========================================//

        //currently only coded for digital action
        if(!digitalRead(LT_BUTTON)){gamepad->setLeftTrigger(XBOX_TRIGGER_MAX);}
        else{gamepad->setLeftTrigger(XBOX_TRIGGER_MIN);}

        if(!digitalRead(RT_BUTTON)){gamepad->setRightTrigger(XBOX_TRIGGER_MAX);}
        else{gamepad->setRightTrigger(XBOX_TRIGGER_MIN);}

    //===========================SEND GAMEPAD REPORT=================================//
        gamepad->sendGamepadReport();
      
    //*//=========LATENCY TIMER AND DEBUG============//
        //Comment this entire block out unless debugging
        end = esp_timer_get_time();
        time_spent = end - begin;

        if(millis() % 100 < 3){
            //Serial.println("MAG: " + String(magnitude));
            //Serial.println("XMapped: " + String(rightJoyXmapped));
            //Serial.println("YMapped: " + String(rightJoyYmapped));
            //Serial.println("X: " + String(rightJoyX));
            //Serial.println("Y: " + String(rightJoyY));
            //Serial.print("TIME SPENT: ");
            //Serial.println(time_spent);
            //Serial.println("POWER LEVEL: " + String(esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV)));
    
        } 

    //=====================================//*/
        delay(1);   
        //testButtons();
        //testPads();
        //testTriggers();
        //testThumbsticks();
    }
}







//TEST CODE FOR EACH INPUT TYPE: TAKEN FROM EXAMPLE CODE FROM LIBRARY


void testButtons(){
    // Test each button
    uint16_t buttons[] = {
        XBOX_BUTTON_A, 
        XBOX_BUTTON_B, 
        XBOX_BUTTON_X, 
        XBOX_BUTTON_Y, 
        XBOX_BUTTON_LB, 
        XBOX_BUTTON_RB, 
        XBOX_BUTTON_START,
        XBOX_BUTTON_SELECT,
        XBOX_BUTTON_HOME,   // Uncomment this to test the hom/guide button. Steam will flip out and enter big picture mode when running this sketch though so be warned!
        XBOX_BUTTON_LS, 
        XBOX_BUTTON_RS
    };
    for (uint16_t button : buttons)
    {
        Serial.println("Pressing button " + String(button));
        gamepad->press(button);
        gamepad->sendGamepadReport();
        delay(500);
        gamepad->release(button);
        gamepad->sendGamepadReport();
        delay(100);
    }

    // The share button is a seperate call since it doesn't live in the same 
    // bitflag as the rest of the buttons

    //
    //gamepad->pressShare();
    //gamepad->sendGamepadReport();
    //delay(500);
    //gamepad->releaseShare();
    //gamepad->sendGamepadReport();
    //delay(100);
}

void testPads(){
    uint16_t directions[] = {
        XBOX_BUTTON_DPAD_NORTH,
        XBOX_BUTTON_DPAD_NORTHEAST,
        XBOX_BUTTON_DPAD_EAST,
        XBOX_BUTTON_DPAD_SOUTHEAST,
        XBOX_BUTTON_DPAD_SOUTH,
        XBOX_BUTTON_DPAD_SOUTHWEST,
        XBOX_BUTTON_DPAD_WEST,
        XBOX_BUTTON_DPAD_NORTHWEST
    };

    for (uint16_t direction : directions)
    {
        Serial.println("Pressing DPad: " + String(direction));
        gamepad->pressDPadDirection(direction);
        gamepad->sendGamepadReport();
        delay(500);
        gamepad->releaseDPadDirection(direction);
        gamepad->sendGamepadReport();
        delay(100);
    }
}

void testTriggers(){
    for(int16_t val = XBOX_TRIGGER_MIN; val <= XBOX_TRIGGER_MAX; val++){
        if(val % 8 == 0)
            Serial.println("Setting trigger value to " + String(val));
        gamepad->setLeftTrigger(val);
        gamepad->setRightTrigger(val);
        gamepad->sendGamepadReport();
        delay(10);
    }
}

void testThumbsticks(){
    int startTime = millis();
    int reportCount = 0;
    while(millis() - startTime < 8000){
        reportCount++;
        int16_t x = cos((float)millis() / 1000.0f) * XBOX_STICK_MAX;
        int16_t y = sin((float)millis() / 1000.0f) * XBOX_STICK_MAX;

        gamepad->setLeftThumb(x, y);
        gamepad->setRightThumb(x, y);
        gamepad->sendGamepadReport();
        
        if(reportCount % 8 == 0)
            Serial.println("Setting left thumb to " + String(x) + ", " + String(y));
            
        delay(10);
    }
}