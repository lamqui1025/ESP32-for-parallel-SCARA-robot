#include "UartRobot.h"

UartRobot HandleRobot;

String inputString = "";
bool stringStart = false;
bool stringComplete = false;
//ScaraRobot Robot;
point P[10];
bool zP[10];
bool eP[10];


String pulToString(int pul)
{
    char res[5];
    int pulConvert;
    res[0] = (pul >= 0)? '+' : '-';
    pulConvert = fabs(pul);
    res[1] = (int)(pulConvert/1000) + '0';
    pulConvert = pulConvert - 1000*res[1];
    res[2] = (int)(pulConvert/100) + '0';
    pulConvert = pulConvert - 100*res[2];
    res[3] = (int)(pulConvert/10) + '0';
    pulConvert = pulConvert - 10*res[2];
    res[4] = (int)pulConvert + '0';
    String result = res;
    return result;
}

String degToString(float deg)
{
    char res[5];
    int pulConvert;
    res[0] = (deg >= 0)? '+' : '-';
    //Serial2.print(res[0]);
    pulConvert = fabs(10*deg);
    res[1] = (int)(pulConvert/1000) + '0';
    //Serial2.print(res[1]);
    pulConvert = pulConvert - 1000*(res[1]-'0');
    res[2] = (int)(pulConvert/100) + '0';
    //Serial2.print(res[2]);
    pulConvert = pulConvert - 100*(res[2]-'0');
    res[3] = (int)(pulConvert/10) + '0';
    //Serial2.print(res[3]);
    pulConvert = pulConvert - 10*(res[3]-'0');
    res[4] = (int)pulConvert + '0';
    //Serial2.print(res[4]);
    String result = res;
    return result;
}

float charToFloat(int16_t highWord, int16_t lowWord)
{
    convertFloatUnion data;
    data.arr[0] = lowWord;
    data.arr[1] = highWord;
    float floatData = data.f;
    return floatData;
}

int16_t int8ToInt16(int8_t highByte, int8_t lowByte)
{
    convertInt16Union data;
    data.arrint8[1] = lowByte;
    data.arrint8[0] = highByte;
    int16_t int16Data = data.int16;
    return int16Data;
}

float fourCharToFloat(char hun, char ten, char unit, char dec)
{
    float result = (hun - '0')*100.0 + (ten - '0')*10.0 + (unit - '0')*1.0 + (dec - '0')*0.1;
    return result;
}

float fiveCharToFloat(char sign, char hun, char ten, char unit, char dec)
{
    float result = (hun - '0')*100.0 + (ten - '0')*10.0 + (unit - '0')*1.0 + (dec - '0')*0.1;
    if (sign == '+')
    {
        return result;
    }
    else if (sign == '-')
    {
        return (0.0 - result);
    }
}

uint8_t twoCharToUint8(char ten, char unit)
{
    uint8_t result = (ten - '0')*10 + (unit - '0');
    return result;
}

void UartRobot::begin(uint8_t tx = 17, uint8_t rx = 16, unsigned long baudrate = 115200)
{
    // Serial2.begin(baudrate, SERIAL_8N1, rx, tx);
}

void serialEvent() {
    while (Serial.available()) {
        // get the new byte:
        char inChar = Serial.read();
        // if the incoming character is a STX, set a flag so the main loop can do something about it:
        if((int)inChar == 2)
        {
            stringStart = true;
        }
        if(stringStart)
        {
            //Serial2.print(inChar);
            inputString += inChar;
        }     
        // if the incoming character is a ETX, set a flag so the main loop can do something about it:
        if ((int)inChar == 3) 
        {
            stringComplete = true;
            stringStart = false;
        }
    }
    
}

bool UartRobot::checkString()
{
    serialEvent();
    this->_stringComplete = stringComplete;
    if(stringComplete == true)
    {
        inputString = inputString.substring(1, inputString.length()-1);
        this->_stringCommand = inputString;
    }   
    if((this->_stringCommand != "") && (stringComplete == true))
    {
        // Serial2.printf("\nString Command is: \n");
        // Serial2.print(this->_stringCommand);
        inputString = "";
    }
    stringComplete = false;
    return this->_stringComplete;
}

bool UartRobot::handleStringCommand()
{
    Robot.setTaskToDo((this->_stringCommand[0]-'0'));
    switch (this->_stringCommand[0]-'0')
    {
    case START:
        Robot.startRobot();
        // Serial2.println("Start Robot");
        this->_isStartRobot = true;
        return true;        
        break;
    case STOP:
        Robot.stopRobot();
        // Serial2.println("Stop Robot");
        this->_isStartRobot = false;
        return true;
        break;
    case RESET:
        // Serial2.println("Reset Robot");

        return true;
    case SWITCH_WORKSPACE:
        // Serial2.println("Switch Workspace");

        return true;
    case KINETIC:
        // Serial2.println("Kinetic");   
        return true;
        break;
    case MOVL:

        // Serial2.println("MOVL");
        return true;
        break;
    case MOVC:
        // Serial2.println("MOVC");
        return true;
        break;
    case MOVJ:
        // Serial2.println("MOVJ");
        return true;
        break;
    case CIRCULAR:
        // Serial2.println("Circular mode");
        return true;
        break;
    case MOVQ:
        return true;
        break;
    default:
        break;
    }
    // Serial2.println("No task match the eTask enum");
    return false;
}

bool UartRobot::handleRobotTask()
{    
    switch(Robot.getTaskToDo())
    {
    case START:
        return true;        
        break;
    case STOP:
        return true;
        break;
    case RESET:
        return Robot.reset();
        break;
    case SWITCH_WORKSPACE:
        return Robot.switchWorkSpace();
        break;
    case KINETIC:
    {
        point A;
        float xA = fiveCharToFloat(this->_stringCommand[2],this->_stringCommand[3], this->_stringCommand[4],this->_stringCommand[5], this->_stringCommand[6]);
        float yA = fiveCharToFloat(this->_stringCommand[7],this->_stringCommand[8], this->_stringCommand[9],this->_stringCommand[10], this->_stringCommand[11]);
        float *pos;
        //Serial.printf("\nPosition before Kinetics XY(%f, %f), Pul(%d,%d)\n", Robot.getRealX(), Robot.getRealY(), Robot.getRealPul1(), Robot.getRealPul4());
        if (((int)(this->_stringCommand[1]) - '0') == 0)
        {
            //Serial.printf("\nTheta1: %f, Theta4: %f\n", xA, yA);
            //Serial.printf("\nWorkspace to forward kinetics: %d, present pul: Pul1: %d, Pul4: %d\n", Robot.getWorkSpace(), Robot.getRealPul1(), Robot.getRealPul4());
            pos = (Robot.getWorkSpace() == true)? Robot.angleToPoint1(xA, yA) : Robot.angleToPoint0(xA, yA);
            A.setPoint(pos[0], pos[1]);
            return Robot.rawMoveToPoint(A);
        }
        else if (((int)(this->_stringCommand[1]) - '0') == 1)
        {
            // Serial2.printf("\nxA: %f, yA: %f\n", xA, yA);
            A.setPoint(xA, yA);
            return Robot.rawMoveToPoint(A);
        }
        else if (((int)(this->_stringCommand[1]) - '0') == 2)
        {
            return Robot.handleZ();
        }
        else if (((int)(this->_stringCommand[1]) - '0') == 3)
        {
            return Robot.onEndEffector();
        }
        else if (((int)(this->_stringCommand[1]) - '0') == 4)
        {
            return Robot.offEndEffector();
        }
        break;
    }
    case MOVL:
    {
        point A1, B;
        float xA1 = fiveCharToFloat(this->_stringCommand[2],this->_stringCommand[3], this->_stringCommand[4], this->_stringCommand[5], this->_stringCommand[6]);
        float yA1 = fiveCharToFloat(this->_stringCommand[7],this->_stringCommand[8], this->_stringCommand[9], this->_stringCommand[10], this->_stringCommand[11]);
        A1.setPoint(xA1, yA1);
        float xB = fiveCharToFloat(this->_stringCommand[12],this->_stringCommand[13], this->_stringCommand[14], this->_stringCommand[15], this->_stringCommand[16]);
        float yB = fiveCharToFloat(this->_stringCommand[17],this->_stringCommand[18], this->_stringCommand[19], this->_stringCommand[20], this->_stringCommand[21]);
        B.setPoint(xB, yB);
        int vL = round(twoCharToUint8(this->_stringCommand[22],this->_stringCommand[23]));
        return Robot.moveInLine(A1, B, vL);
        break;
    }
    case MOVC:
    {
        point CO;
        float xCO = fiveCharToFloat(this->_stringCommand[2],this->_stringCommand[3], this->_stringCommand[4], this->_stringCommand[5], this->_stringCommand[6]);
        float yCO = fiveCharToFloat(this->_stringCommand[7],this->_stringCommand[8], this->_stringCommand[9], this->_stringCommand[10], this->_stringCommand[11]);
        CO.setPoint(xCO, yCO);
        float R = fiveCharToFloat(this->_stringCommand[12],this->_stringCommand[13], this->_stringCommand[14], this->_stringCommand[15], this->_stringCommand[16]);
        float start = fiveCharToFloat(this->_stringCommand[17],this->_stringCommand[18], this->_stringCommand[19], this->_stringCommand[20], this->_stringCommand[21]);
        float start_rad = (float)(start*PI/180);
        float end = fiveCharToFloat(this->_stringCommand[22],this->_stringCommand[23], this->_stringCommand[24], this->_stringCommand[25], this->_stringCommand[26]);
        float end_rad = (float)(end*PI/180);
        int vC = round(twoCharToUint8(this->_stringCommand[27],this->_stringCommand[28]));
        return Robot.moveInCircle(CO,R,start_rad,end_rad, vC);
        break;
    }
    case MOVJ:
    {
        point A2;
        float xA2 = fiveCharToFloat(this->_stringCommand[2],this->_stringCommand[3], this->_stringCommand[4], this->_stringCommand[5], this->_stringCommand[6]);
        float yA2 = fiveCharToFloat(this->_stringCommand[7],this->_stringCommand[8], this->_stringCommand[9], this->_stringCommand[10], this->_stringCommand[11]);
        A2.setPoint(xA2, yA2);
        int vMax = round(twoCharToUint8(this->_stringCommand[12],this->_stringCommand[13]) * Robot.getVMax()/100);
        int aMax = round(twoCharToUint8(this->_stringCommand[14],this->_stringCommand[15]) * Robot.getAMax()/100);
        //Serial.printf("\n xA,yA %f,%f vMax, aMax, %d, %d", xA2,yA2, vMax,aMax);
        return Robot.handleToPointCirculation(A2, vMax, aMax);
        break;
    }
    case CIRCULAR:
    {
        if (((int)(this->_stringCommand[1]) - '0') == 0)
        {
            int noP = (int)this->_stringCommand[2] - '0';
            int vcMax = round(twoCharToUint8(this->_stringCommand[3],this->_stringCommand[4]) * Robot.getVMax()/100);
            int acMax = round(twoCharToUint8(this->_stringCommand[5],this->_stringCommand[6]) * Robot.getAMax()/100);
            for(int i = 0; i<noP; i++)
            {
                float xAc = fiveCharToFloat(this->_stringCommand[11*i + 7],this->_stringCommand[11*i + 8], this->_stringCommand[11*i + 9], this->_stringCommand[11*i + 10], this->_stringCommand[11*i + 11]);
                float yAc = fiveCharToFloat(this->_stringCommand[11*i + 12],this->_stringCommand[11*i + 13], this->_stringCommand[11*i + 14], this->_stringCommand[11*i + 15], this->_stringCommand[11*i + 16]);
                uint8_t ze = (uint8_t)((this->_stringCommand[11*i + 17]- '0'));
                P[i].setPoint(xAc, yAc);
                zP[i] = (int)(floor(ze/2));
                eP[i] = (int)(ze - 2*zP[i]);
                // Serial2.printf("ze: %d, zP: %d, eP: %d",ze, zP[i], eP[i]);
                if(!Robot.handleToPointCirculation(P[i],vcMax,acMax))
                {
                    return false;
                }
                else
                {
                    zP[i] = (zP[i])? Robot.downZ() : false;
                    eP[i] = (eP[i])? Robot.onEndEffector() : Robot.offEndEffector();

                    if(zP[i])
                    {
                        delay(200);
                    }
                    Robot.upZ();
                }
            }
        }
        else if (((int)(this->_stringCommand[1]) - '0') == 1)
        {
            return true;

        }
            return true;
        

        // point Ac, Bc, Cc;
        // float xAc = fiveCharToFloat(this->_stringCommand[2],this->_stringCommand[3], this->_stringCommand[4], this->_stringCommand[5], this->_stringCommand[6]);
        // float yAc = fiveCharToFloat(this->_stringCommand[7],this->_stringCommand[8], this->_stringCommand[9], this->_stringCommand[10], this->_stringCommand[11]);
        // Ac.setPoint(xAc, yAc);
        // float xBc = fiveCharToFloat(this->_stringCommand[12],this->_stringCommand[13], this->_stringCommand[14], this->_stringCommand[15], this->_stringCommand[16]);
        // float yBc = fiveCharToFloat(this->_stringCommand[17],this->_stringCommand[18], this->_stringCommand[19], this->_stringCommand[20], this->_stringCommand[21]);
        // Bc.setPoint(xBc, yBc);
        // float xCc = fiveCharToFloat(this->_stringCommand[22],this->_stringCommand[23], this->_stringCommand[24], this->_stringCommand[25], this->_stringCommand[26]);
        // float yCc = fiveCharToFloat(this->_stringCommand[27],this->_stringCommand[28], this->_stringCommand[29], this->_stringCommand[30], this->_stringCommand[31]);
        // Cc.setPoint(xCc, yCc);
        // int vMaxc = round(twoCharToUint8(this->_stringCommand[32],this->_stringCommand[33]) * Robot.getVMax()/100);
        // int aMaxc = round(twoCharToUint8(this->_stringCommand[34],this->_stringCommand[35]) * Robot.getAMax()/100);
        // bool zA, zB, zC;
        // uint8_t z = (uint8_t)((this->_stringCommand[36]- '0') <<5);
        // zA = (z>>7)? true : false;
        // zB = ((z<<1)>>7)? true : false;
        // zC = ((z<<2)>>7)? true : false;

        // bool eA, eB, eC;
        // uint8_t e = (uint8_t)((this->_stringCommand[37] - '0') <<5);
        // eA = (e>>7)? true : false;
        // eB = ((e<<1)>>7)? true : false;
        // eC = ((e<<2)>>7)? true : false;
        // if(Robot.handleToPointCirculation(Ac, vMaxc, aMaxc))
        // {
        //     zA = zA? Robot.downZ() : false;
        //     eA = eA? Robot.onEndEffector() : Robot.offEndEffector();
        //     Robot.upZ();
        //     if(Robot.handleToPointCirculation(Bc, vMaxc, aMaxc))
        //     {
        //         zB = zB? Robot.downZ() : false;
        //         eB = eB? Robot.onEndEffector() : Robot.offEndEffector();
        //         Robot.upZ();
        //         if(Robot.handleToPointCirculation(Cc, vMaxc, aMaxc))
        //         {
        //             zC = zC? Robot.downZ() : false;
        //             eC = eC? Robot.onEndEffector() : Robot.offEndEffector();
        //             Robot.upZ();
        //             return Robot.handleToPointCirculation(Ac, vMaxc, aMaxc);
        //         }
        //         else return false;
        //     }
        //     else return false;
        // }
        // else return false;
        break;
    }
    case MOVQ:
    {
        point A2;
        float xA2 = fiveCharToFloat(this->_stringCommand[2],this->_stringCommand[3], this->_stringCommand[4], this->_stringCommand[5], this->_stringCommand[6]);
        float yA2 = fiveCharToFloat(this->_stringCommand[7],this->_stringCommand[8], this->_stringCommand[9], this->_stringCommand[10], this->_stringCommand[11]);
        A2.setPoint(xA2, yA2);
        point A3;
        float xA3 = fiveCharToFloat(this->_stringCommand[12],this->_stringCommand[13], this->_stringCommand[14], this->_stringCommand[15], this->_stringCommand[16]);
        float yA3 = fiveCharToFloat(this->_stringCommand[17],this->_stringCommand[18], this->_stringCommand[19], this->_stringCommand[20], this->_stringCommand[21]);
        A3.setPoint(xA3, yA3);
        int vMax = round(twoCharToUint8(this->_stringCommand[12],this->_stringCommand[22]) * Robot.getVMax()/100);
        int aMax = round(twoCharToUint8(this->_stringCommand[24],this->_stringCommand[23]) * Robot.getAMax()/100);

        if (Robot.handleToPointCirculation(A2, vMax, aMax))
        {
            Robot.onEndEffector();
            Robot.downZ();
            delay(300);
            Robot.upZ();
            if(Robot.handleToPointCirculation(A3, vMax, aMax))
            {
                delay(100);
                Robot.offEndEffector();
                return true;
            }
        }
        return false;
        break;
    }
    default:
        break;

    }
    return false;
}

void UartRobot::respondToRequest()
{
    // Serial2.println("String Respond:");
    //this->_stringResponse = char(2) + this->_resCode + char(Robot.getWorkSpace()+'0') + degToString(Robot.getRealTheta1()) + degToString(Robot.getRealTheta4()) + char(3);
    Serial.print(char(2));
    Serial.print(this->_resCode);
    Serial.print(Robot.getWorkSpace());
    Serial.print(degToString(Robot.getRealTheta1()));
    Serial.print(degToString(Robot.getRealTheta4()));
    Serial.print(char(3));
    //Serial2.printf("%s%s%s%s%s%s",char(2), this->_resCode,Robot.getWorkSpace()+'0',degToString(Robot.getRealTheta1()), degToString(Robot.getRealTheta4()), char(3)), 
    Serial.flush();
    //delay(100);
}
// ===========================QUI WRITED============================

void valueTofiveBytes(uint8_t* res, int value)
{
    res[0] = (value >= 0)? '+' : '-';
    value = fabs(value);
    for(int i = 4; i > 0; i--)
    {
        res[i] = int(value%10) + '0';
        value = int(value/10);
    }
}

void UartRobot::respondToPC()
{
    int realpul1 = Robot.getRealPul1();
    int realpul4 = Robot.getRealPul4();
    uint8_t res[14];
    res[0] = 2;
    res[1] = this->_resCode;
    res[2] = uint8_t(Robot.getWorkSpace());
    valueTofiveBytes(&res[3], realpul1);
    // res[3] = (realpul1 >=0)? 1 : 0;
    // res[4] = uint8_t(fabs(realpul1/256));
    // res[5] = uint8_t(fabs(realpul1%256));
    valueTofiveBytes(&res[8], realpul4);
    // res[6] = (realpul4 >=0)? 1 : 0;
    // res[7] = uint8_t(fabs(realpul4/256));
    // res[8] = uint8_t(fabs(realpul4%256));
    res[13] = 3;
    // Serial.print(res);
    Serial.write(res, 14);
    // Serial.flush();
}
// ===========================QUI WRITED============================
void UartRobot::loop()
{
    if((this->checkString()) || (Robot.getTaskToDo() == CIRCULAR))
    {  
        if(this->handleStringCommand())
        {
            if((this->_isStartRobot))
            {
                if(this->handleRobotTask())
                {
                    this->_resCode = SUC_TASK;
                    // this->respondToRequest();
                    this->respondToPC();

                }
                else
                {
                    this->_resCode = ERR_TASK;
                    // this->respondToRequest();  
                    this->respondToPC();
                }

            }

        }
        else
        {
            this->_resCode = ERR_PARSE;
            // this->respondToRequest();
            this->respondToPC();

        }
    }
    if(Robot.getTaskToDo() != CIRCULAR)
    {
        for (uint8_t i = 0; i < this->_stringCommand.length(); i++)
        {
            this->_stringCommand[i] = NULL;
        }
        
    }
}