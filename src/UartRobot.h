#ifndef FIVEBARROBOT_UART_H
#define FIVEBARROBOT_UART_H

#include <Arduino.h>
#include <math.h> 
#include <WiFi.h>
#include <SPIFFS.h>
#include "ScaraRobot.h"

enum eResponse
{   SUC_TASK,
    ERR_PARSE,
	ERR_TASK,
};

typedef union convertFloatUnion
{
    int16_t arr[2];
    float f;
} convertFloatUnion;

typedef union convertInt16Union
{
    int8_t arrint8[2];
    int16_t int16;
} convertInt16Union;

class UartRobot
{
    private:
    bool _stringComplete = false;
    String _stringCommand;
    String _stringResponse;
    bool _isStartRobot;
    uint8_t _resCode;
    protected:
    public:

    void begin(uint8_t tx, uint8_t rx, unsigned long baudrate);
    bool checkString();
    bool handleStringCommand();
    bool handleRobotTask(); 
    void respondToRequest();
    void respondToPC();
    //void clearStringCommand();
    
    void loop();

};
extern UartRobot HandleRobot;

#endif