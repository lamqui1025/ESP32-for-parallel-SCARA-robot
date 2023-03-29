#include "ScaraRobot.h"

ScaraRobot Robot;

void ScaraRobot::begin()
{
    pinMode(ENA_ROBOT,OUTPUT);
    digitalWrite(ENA_ROBOT,LOW);
    pinMode(END_EFFECTOR_PIN,OUTPUT);
    digitalWrite(END_EFFECTOR_PIN,LOW);
    pinMode(DIR_PIN_1,OUTPUT);
    digitalWrite(DIR_PIN_1,LOW);
    pinMode(DIR_PIN_2,OUTPUT);
    digitalWrite(DIR_PIN_2,LOW);
    pinMode(PUL_PIN_1,OUTPUT);
    digitalWrite(PUL_PIN_1,LOW);    
    pinMode(PUL_PIN_2,OUTPUT);
    digitalWrite(PUL_PIN_2,LOW);
    pinMode(Z_PIN,OUTPUT);
    digitalWrite(Z_PIN,LOW);
    pinMode(CHECK_PIN_1,INPUT_PULLUP);
    pinMode(CHECK_PIN_2,INPUT_PULLUP);
    this->_realPul1 = 3200;
    this->_realPul4 = 0;
    this->_NO.setPoint(0, -50);
    this->_PO.setPoint(0, 50);
    return;
}

void ScaraRobot::startRobot()
{
    digitalWrite(ENA_ROBOT, HIGH);
    this->reset();
    this->calibZero();
}

void ScaraRobot::stopRobot()
{
    this->reset();
    this->calibZero();
    digitalWrite(ENA_ROBOT, LOW);
}

void ScaraRobot::setDisBase(int disBase)
{
    this->_disBase = disBase;
}

void ScaraRobot::setLongLink(int longLink)
{
    this->_longLink = longLink;
}

void ScaraRobot::setShortLink(int shortLink)
{
    this->_shortLink = shortLink;
}

void ScaraRobot::setVMax(int vMax)
{
    this->_vmax = vMax;
}

int ScaraRobot::getVMax()
{
    return this->_vmax;
}

void ScaraRobot::setAMax(int aMax)
{
    this->_amax = aMax;
}

int ScaraRobot::getAMax()
{
    return this->_amax;
}


void ScaraRobot::setTaskToDo(uint8_t task)
{
    this->_taskToDo = task;
}

uint8_t ScaraRobot::getTaskToDo()
{
    return this->_taskToDo;
}
void ScaraRobot::setSpd1(float spd1)
{
    if(spd1 == 0.0F)
    {
        delayMicroseconds(10000);
        return;
    }
    else
    {
        int Th = (int)round(10000/(2*fabs(spd1)));
        int spd_int = (int)round(fabs(spd1));
        int pul;
        if (spd1 < 0.0F)
        {
            digitalWrite(DIR_PIN_1,LOW); // CW
            pul = -1;
        }
        else
        {
            digitalWrite(DIR_PIN_1,HIGH); // CCW
            pul = 1;
        }
        

        for(int i = 0; i < spd_int; i++)
        {
            digitalWrite(PUL_PIN_1,HIGH);
            delayMicroseconds(Th);
            digitalWrite(PUL_PIN_1,LOW);
            delayMicroseconds(Th);
            this->_realPul1 = this->_realPul1 + pul;
            this->forwardKinetic();
        }
        return;

    }
}

void ScaraRobot::setSpd1_1ms(float spd1)
{
    if(spd1 == 0.0F)
    {
        delayMicroseconds(1000);
        return;
    }
    else
    {
        int Th = (int)round(1000/(2*fabs(spd1)));
        int spd_int = (int)round(fabs(spd1));
        int pul;
        if (spd1 < 0.0F)
        {
            digitalWrite(DIR_PIN_1,LOW); // CW
            pul = -1;
        }
        else
        {
            digitalWrite(DIR_PIN_1,HIGH); // CCW
            pul = 1;
        }
        

        for(int i = 0; i < spd_int; i++)
        {
            digitalWrite(PUL_PIN_1,HIGH);
            delayMicroseconds(Th);
            digitalWrite(PUL_PIN_1,LOW);
            delayMicroseconds(Th);
            this->_realPul1 = this->_realPul1 + pul;
            this->forwardKinetic();
        }
        return;

    }
}

void ScaraRobot::setSpd4(float spd2)
{
    if(spd2 == 0.0F)
    {
        delayMicroseconds(10000);
        return;
    }
    else
    {
        int Th = (int)round(10000/(2*fabs(spd2)));
        int spd_int = (int)round(fabs(spd2));
        int pul;
        if (spd2 < 0.0F)
        {
            digitalWrite(DIR_PIN_2,LOW); // CW
            pul = -1;
        }
        else
        {
            digitalWrite(DIR_PIN_2,HIGH); // CCW
            pul = 1;
        }
        

        for(int i = 0; i < spd_int; i++)
        { 
            digitalWrite(PUL_PIN_2,HIGH);
            delayMicroseconds(Th);
            digitalWrite(PUL_PIN_2,LOW);
            delayMicroseconds(Th);
            this->_realPul4 = this->_realPul4 + pul;
            this->forwardKinetic();          
        }
        return;

    }
}

void ScaraRobot::setSpd4_1ms(float spd2)
{
    if(spd2 == 0.0F)
    {
        delayMicroseconds(1000);
        return;
    }
    else
    {
        int Th = (int)round(1000/(2*fabs(spd2)));
        int spd_int = (int)round(fabs(spd2));
        int pul;
        if (spd2 < 0.0F)
        {
            digitalWrite(DIR_PIN_2,LOW); // CW
            pul = -1;
        }
        else
        {
            digitalWrite(DIR_PIN_2,HIGH); // CCW
            pul = 1;
        }
        

        for(int i = 0; i < spd_int; i++)
        { 
            digitalWrite(PUL_PIN_2,HIGH);
            delayMicroseconds(Th);
            digitalWrite(PUL_PIN_2,LOW);
            delayMicroseconds(Th);
            this->_realPul4 = this->_realPul4 + pul;
            this->forwardKinetic();          
        }
        return;

    }
}

bool ScaraRobot::switchWorkSpace()
{
    bool curWorkspace = (this->_workspace)? moveToPoint(this->_PO) : moveToPoint(this->_NO);
    bool swWorkspace = false;
    if(curWorkspace)
    {
        swWorkspace = this->switchO(round(this->_vmax/3), round(this->_amax/3));
    }
    return swWorkspace;
}

float ScaraRobot::calVmax(float s_max, float a_max, float v_max)
{
    return min(min((float)(sqrt(fabs(s_max*a_max))), v_max), this->_vmax);
}

void ScaraRobot:: velocityProfile1(float s1_max, float a1_max, float tf1)
{
    int sgn = (int)(s1_max/fabs(s1_max));
    int pulFull = (int)round(fabs(s1_max/this->_degPerStep));
    float a_max = sgn*a1_max;
    float v_max = sgn*(tf1-sqrt(pow(tf1,2)-4*fabs(s1_max/a1_max)))*a1_max/2;
    float tf = round(tf1);
    float tc = round(v_max/a_max);
    float v = 0;
    for (float t = 0; t < tf; t = t + 0.01)
    {
        if(t < tc)
        {
            v = a_max*t;
        }
        else if(t < (tf - tc))
        {
            v = a_max*tc;
        }
        else if(t < tf)
        {
            v = a_max*(tc - (tf-t));
        }
        else
        {
            v = 0;
        }
        this->setSpd1(v);
        this->forwardKinetic();
    }
    float pulNeed = (float)(pulFull - this->_realPul1);
    this->setSpd1(pulNeed);
    return;
}

void ScaraRobot:: velocityProfile4(float s2_max, float a2_max, float tf2)
{
    int sgn = (int)(s2_max/fabs(s2_max));
    int pulFull = (int)round(fabs(s2_max/this->_degPerStep));
    float a_max = sgn*a2_max;
    float v_max = sgn*(tf2-sqrt(pow(tf2,2)-4*fabs(s2_max/a2_max)))*a2_max/2;
    float tf = round(tf2);
    float tc = round(v_max/a_max);
    float v = 0;
    for (float t = 0; t < tf; t = t + 0.01)
    {
        if(t < tc)
        {
            v = a_max*t;
        }
        else if(t < (tf - tc))
        {
            v = a_max*tc;
        }
        else if(t < tf)
        {
            v = a_max*(tc - (tf-t));
        }
        else
        {
            v = 0;
        }
        this->setSpd4(v);
        this->forwardKinetic();
    }
    float pulNeed = (float)(pulFull - this->_realPul4);
    this->setSpd4(pulNeed);
    return;
}

void ScaraRobot::checkOMotor1()
{
    while(digitalRead(CHECK_PIN_1) == HIGH)
    {
        for (int i = 1; i < 20; i++)
        {
            int spd = (int)round(15*i*this->_stepPerDeg);
            int th = (int)round(150000*i/spd);
            if(i%2 == 0)
            {
                digitalWrite(DIR_PIN_1,LOW);
                for(int t = 0; t < spd; t++)
                {
                    if(digitalRead(CHECK_PIN_1) == LOW)
                    {
                        return;
                    }
                    digitalWrite(PUL_PIN_1,HIGH);
                    delayMicroseconds(th);
                    digitalWrite(PUL_PIN_1,LOW);
                    delayMicroseconds(th);
                    if(digitalRead(CHECK_PIN_1) == LOW)
                    {
                        return;
                    }
                }

            }
            else
            {
                digitalWrite(DIR_PIN_1,HIGH);
                for(int t = 0; t < spd; t++)
                {
                    if(digitalRead(CHECK_PIN_1) == LOW)
                    {
                        return;
                    }
                    digitalWrite(PUL_PIN_1,HIGH);
                    delayMicroseconds(th);
                    digitalWrite(PUL_PIN_1,LOW);
                    delayMicroseconds(th);
                    if(digitalRead(CHECK_PIN_1) == LOW)
                    {
                        return;
                    }
                }
            }

        }
    }

}

void ScaraRobot::checkOMotor2()
{
    while(digitalRead(CHECK_PIN_2) == HIGH)
    {
        for (int i = 1; i < 20; i++)
        {
            int spd = (int)round(15*i*this->_stepPerDeg);
            int th = (int)round(150000*i/spd);
            if(i%2 == 0)
            {
                digitalWrite(DIR_PIN_2,HIGH);
                for(int t = 0; t < spd; t++)
                {
                    if(digitalRead(CHECK_PIN_2) == LOW)
                    {
                        return;
                    }
                    digitalWrite(PUL_PIN_2,HIGH);
                    delayMicroseconds(th);
                    digitalWrite(PUL_PIN_2,LOW);
                    delayMicroseconds(th);
                    if(digitalRead(CHECK_PIN_2) == LOW)
                    {
                        return;
                    }
                }

            }
            else
            {
                digitalWrite(DIR_PIN_2,LOW);
                for(int t = 0; t < spd; t++)
                {
                    if(digitalRead(CHECK_PIN_2) == LOW)
                    {
                        return;
                    }
                    digitalWrite(PUL_PIN_2,HIGH);
                    delayMicroseconds(th);
                    digitalWrite(PUL_PIN_2,LOW);
                    delayMicroseconds(th);
                    if(digitalRead(CHECK_PIN_2) == LOW)
                    {
                        return;
                    }
                }
            }

        }
    }

}

bool ScaraRobot:: switchO(int vMax, int aMax)
{
    bool moveToO = (this->_workspace)? moveToPointCirculation(this->_PO, vMax, aMax) : moveToPointCirculation(this->_NO, vMax, aMax);
    if(!moveToO)
    {
        return false;
    }
    float* theta;
    theta = (this->_workspace)? pointToAngle0(this->_NO.x,this->_NO.y): pointToAngle1(this->_PO.x,this->_PO.y);
    float s1 = theta[0] - (this->_realPul1)*(this->_degPerStep);
    float s4 = theta[1] - (this->_realPul4)*(this->_degPerStep);
    float amax = aMax;
    float vmax1 = calVmax(s1,amax,vMax), vmax4 = calVmax(s4,amax,vMax);
    float tf = max(fabs(s1/vmax1) + vmax1/amax, fabs(s4/vmax4) + vmax4/amax);
    this->velocityProfile(s1, amax, s4, amax, tf);
    this->_workspace = !this->_workspace;
    return true;

}

void ScaraRobot:: velocityProfile(float s1_max, float a1_max, float s2_max, float a2_max, float tf)
{
    int sgn1 = (s1_max>=0)? 1:(-1);
    int sgn2 = (s2_max>=0)? 1:(-1);
    int pulFull1 = sgn1*(int)round(fabs(s1_max/this->_degPerStep));
    int pulFull2 = sgn2*(int)round(fabs(s2_max/this->_degPerStep));
    float a1_max_f = sgn1*a1_max;
    float sqrtDelta1 = sqrt(pow(tf,2)-4*fabs(s1_max/a1_max));
    if(isnan(sqrtDelta1))
    {
        sqrtDelta1 = 0;
    }
    float v1_max_f = sgn1*(tf-sqrtDelta1)*a1_max/2;
    float a2_max_f = sgn2*a2_max;
    float sqrtDelta2 = sqrt(pow(tf,2)-4*fabs(s2_max/a2_max));
    if(isnan(sqrtDelta2))
    {
        sqrtDelta2 = 0;
    }
    float v2_max_f = sgn2*(tf-sqrtDelta2)*a2_max/2;
    float tc1 = round(1000*v1_max_f/a1_max_f);
    float tc2 = round(1000*v2_max_f/a2_max_f);
    float v1 = 0;
    float v2 = 0;
    tf = 1000*tf;
    // Serial2.printf("Trapezoidal profile: tf: %f, tc1: %f, tc2: %f, p1: %d, p2: %d, s1: %f, s2: %f, a1: %f, a2: %f, v1: %f, v2: %f", tf, tc1, tc2, pulFull1, pulFull2,s1_max, s2_max, a1_max_f, a2_max_f, v1_max_f, v2_max_f);
    int curPul1 = this->_realPul1;
    int curPul2 = this->_realPul4;
    for (float t = 0; t < tf; t = t + 20)
    {
        if(t < tc1)
        {
            v1 = 2*a1_max_f*t;
        }
        else if(t < (tf - tc1))
        {
            v1 = 2*a1_max_f*tc1;
        }
        else if(t < tf)
        {
            v1 = 2*a1_max_f*(tf-t);
        }
        else
        {
            v1 = 0;
        }
        v1 = v1*this->_stepPerDeg/pow(10,5);
        this->setSpd1(v1);

        if(t < tc2)
        {
            v2 = 2*a2_max_f*t;
        }
        else if(t < (tf - tc2))
        {
            v2 = 2*a2_max_f*tc2;
        }
        else if(t < tf)
        {
            v2 = 2*a2_max_f*(tf-t);
        }
        else
        {
            v2 = 0;
        }
        v2 = v2*this->_stepPerDeg/pow(10,5);
        this->setSpd4(v2);
        this->forwardKinetic();
        //Serial.printf("\nloop at t: %f, v1: %f, v4:%f, pul1: %d, pul4: %d\n", t, v1, v2, this->_realPul1, this->_realPul4);
    }
    int actPul1 = this->_realPul1 - curPul1;
    int actPul2 = this->_realPul4 - curPul2;
    //Serial.printf("pulNeed1: %d, pulNeed2: %d", pulFull1 - actPul1, pulFull2 - actPul2);

    this->setSpd1(pulFull1 - actPul1);
    this->setSpd4(pulFull2 - actPul2);
    return;
}

void ScaraRobot::setWorkingMode(bool mode)
{
    _workingmode = mode;
}

void ScaraRobot::setWorkSpace(bool space)
{
    _workspace = space;
}

bool ScaraRobot::getWorkingMode()
{
    return this->_workingmode;
}

bool ScaraRobot::getWorkSpace()
{
    return this->_workspace;
}

int ScaraRobot::getRealPul1()
{
    return _realPul1;
}

int ScaraRobot::getRealPul4()
{
    return _realPul4;
}

float ScaraRobot::getRealX()
{
    return _realX;
}

float ScaraRobot::getRealY()
{
    return _realY;
}

float ScaraRobot::getRealTheta1()
{
    return ((float)this->_realPul1*this->_degPerStep);
}

float ScaraRobot::getRealTheta4()
{
    return ((float)this->_realPul4*this->_degPerStep);
}

bool ScaraRobot::calibZero()
{
    this->checkOMotor1();
    this->checkOMotor2();
    return true;
}

bool ScaraRobot::reset()
{
    bool reset = true;
    if((this->_realPul1 != 3200)||(this->_realPul4))
    {
        point O;
        O.setPoint(0,0);
        O.workspace = this->_workspace;
        if(!moveToPoint(O))
        {
            // Serial2.println("fail to reset");
            return false;
        }
        //this->_workspace = true;
        this->checkOMotor1();
        this->checkOMotor2();
    }
    return reset;
}
// Function caculate position in workspace -
float* ScaraRobot::angleToPoint0(float theta1, float theta4)
{
    static float pos[2];
    static float B, C, A, D, q;
    int a1 = this->_disBase, a2 = this->_shortLink, a3 = this->_longLink;
    theta1 = theta1*PI/180;
    theta4 = theta4*PI/180;
    B = pow(a1,2) + 2*pow(a2,2)*(1 - cos(theta4 - theta1)) + 2*a1*a2*(cos(theta4) - cos(theta1));
    C = 2*a2*a3*(cos(theta4) - cos(theta1)) + 2*a1*a3;
    A = -2*a2*a3*(sin(theta4) - sin(theta1));
    D = (A + sqrt(pow(A,2) - pow(B,2) + pow(C,2)))/(B - C);
    q = 2*atan(D);
    pos[0] = a2*cos(theta4) + a3*cos(q) + a1/2;
    pos[1] = a2*sin(theta4) + a3*sin(q);
    return pos;
}

// Function caculate position in workspace +
float* ScaraRobot::angleToPoint1(float theta1, float theta4)
{
    static float pos[2];
    static float B, C, A, D, q;
    int a1 = this->_disBase, a2 = this->_shortLink, a3 = this->_longLink;
    theta1 = theta1*PI/180;
    theta4 = theta4*PI/180;
    B = pow(a1,2) + 2*pow(a2,2)*(1 - cos(theta4 - theta1)) + 2*a1*a2*(cos(theta4) - cos(theta1));
    C = 2*a2*a3*(cos(theta4) - cos(theta1)) + 2*a1*a3;
    A = -2*a2*a3*(sin(theta4) - sin(theta1));
    D = (A - sqrt(pow(A,2) - pow(B,2) + pow(C,2)))/(B - C);
    q = 2*atan(D);
    pos[0] = a2*cos(theta4) + a3*cos(q) + a1/2;
    pos[1] = a2*sin(theta4) + a3*sin(q);
    return pos;
}

// Function caculate angle in workingmode +-
float* ScaraRobot::pointToAngle0(float x, float y)
{
    static float theta0[2];
    float Be1, Ce1, Ae1, De12, theta12, Be4, Ce4, Ae4, De42, theta42;
    int a1 = this->_disBase, a2 = this->_shortLink, a3 = this->_longLink;
    // caculate theta1
    Be1 = (pow((x + a1/2),2) + pow(y,2) + pow(a2,2) - pow(a3,2))/(2*a2);
    Ce1 = -(x + a1/2);
    Ae1 = y;
    De12 = (Ae1 - sqrt(pow(Ae1,2) - pow(Be1,2) + pow(Ce1,2)))/(Be1 - Ce1);
    theta12 = 2*atan(De12);
    if(isnan(theta12))
    {
        theta12 = -PI;
    }
    
    theta0[0] = theta12*180/PI;
    if(theta0[0]>0)
    {
        theta0[0] = theta0[0] - 360;
    }
    theta0[0] = theta0[0] + 360;
    // caculate theta4
    Be4 = (pow((x - a1/2),2) + pow(y,2) + pow(a2,2) - pow(a3,2))/(2*a2);
    Ce4 = -(x - a1/2);
    Ae4 = y;
    De42 = (Ae4 + sqrt(pow(Ae4,2) - pow(Be4,2) + pow(Ce4,2)))/(Be4 - Ce4);
    theta42 = 2*atan(De42);
    theta0[1] = theta42*180/PI;
    return theta0;
}

// Function caculate angle in workingmode -+
float* ScaraRobot::pointToAngle1(float x, float y)
{
    static float theta1[2];
    float Be1, Ce1, Ae1, De11, theta11, Be4, Ce4, Ae4, De41, theta41;
    int a1 = this->_disBase, a2 = this->_shortLink, a3 = this->_longLink;
    // caculate theta1
    Be1 = (pow((x + a1/2),2) + pow(y,2) + pow(a2,2) - pow(a3,2))/(2*a2);
    Ce1 = -(x + a1/2);
    Ae1 = y;
    De11 = (Ae1 + sqrt(pow(Ae1,2) - pow(Be1,2) + pow(Ce1,2)))/(Be1 - Ce1);
    theta11 = 2*atan(De11);
    if(isnan(theta11))
    {
        theta11 = PI;
    }
    
    theta1[0] = theta11*180/PI;
    if(theta1[0] < 0)
    {
        theta1[0] = theta1[0] + 360;
    }
    // caculate theta4
    Be4 = (pow((x - a1/2),2) + pow(y,2) + pow(a2,2) - pow(a3,2))/(2*a2);
    Ce4 = -(x - a1/2);
    Ae4 = y;
    De41 = (Ae4 - sqrt(pow(Ae4,2) - pow(Be4,2) + pow(Ce4,2)))/(Be4 - Ce4);
    theta41 = 2*atan(De41);
    theta1[1] = theta41*180/PI;
    return theta1;
}

void ScaraRobot:: forwardKinetic()
{
    float theta1 = (this->_realPul1)*(this->_degPerStep);
    float theta4 = (this->_realPul4)*(this->_degPerStep);

    float* pos;
    pos = (this->_workspace)? (this->angleToPoint1(theta1, theta4)) : (this->angleToPoint0(theta1, theta4));
    this->_realX = pos[0];
    this->_realY = pos[1];
}

void ScaraRobot::inverseKinetic()
{
    float* theta;
    theta = (this->_workingmode)? (this->pointToAngle1(this->_realX, this->_realY)) : (this->pointToAngle0(this->_realX, this->_realY));
    this->_realPul1 = (int)(theta[0]*(this->_stepPerDeg));
    this->_realPul4 = (int)(theta[1]*(this->_stepPerDeg));
}

bool ScaraRobot::isInWorkSpace(point A)
{
    int R = this->_shortLink + this->_longLink;
    int a = this->_disBase;
    if (A.x>=0)
    {
        return ((pow((A.x+a/2),2) + pow(A.y,2) <= pow(R,2))? true : false);


    }
    else 
    {
        return ((pow((A.x-a/2),2) + pow(A.y,2) <= pow(R,2))? true : false);

    }

}

bool ScaraRobot::isInCurrentWorkSpace(point B)
{
    if(B.workspace == this->_workspace)
    {
        return true;
    }
    else
    {
        if(this->_workspace)
        {
            if (B.x > -100)
            {
                // Serial2.println("Not in current Workspace, TH1");
                return false;
            }
            else if (B.y < -100)
            {
                // Serial2.println("Not in current Workspace, TH2");
                return false;
            }
            else
            {
                B.workspace == this->_workspace;
                return true;
            }

        }
        else if(!(this->_workspace))
        {
            if (B.x > -100)
            {
                // Serial2.println("Not in current Workspace, TH3");
                return false;
            }
            else if (B.y > 100)
            {
                // Serial2.println("Not in current Workspace, TH4");
                return false;
            }
            else
            {
                B.workspace == this->_workspace;
                return true;
            }

        }
    }
}

bool ScaraRobot::moveToPoint(point D)
{
    if(!(this->isInWorkSpace(D)))
    {
        // Serial2.printf("\nThe destination (%f, %f) is not in the Workspace\n", D.x, D.y);
        return false;
    }
    else if(!(this->_workspace == D.workspace))
    {
        // Serial2.printf("\nThe destination (%f, %f) is not in the same Workspace\n", D.x, D.y);
        return false;
    }
    else if ((abs(this->_realX - D.x) <= 0.05)&&(abs(this->_realY - D.y) <= 0.05))
    {
        // Serial2.printf("/n Already at the destination, curren pos: %f, %f", this->_realX, this->_realY);
        return true;
    }
    else
    {
        float* theta;
        // Serial2.printf("Present Workspace is: %d", int(this->_workspace));
        theta = (this->_workspace)? pointToAngle1(D.x,D.y): pointToAngle0(D.x,D.y);
        // Serial2.printf("\nAngle of destination (%f, %f)\n", theta[0], theta[1]);
        float s1 = theta[0] - (this->_realPul1)*(this->_degPerStep);
        float s4 = theta[1] - (this->_realPul4)*(this->_degPerStep);
        float amax = (this->_amax)/2;
        float vmax1 = calVmax(s1,amax,this->_vmax/2), vmax4 = calVmax(s4,amax,this->_vmax/2);
        //Serial.printf("vmax1: %f, vmax4: %f", vmax1, vmax4);
        float tf = max(fabs(s1/vmax1) + vmax1/amax, fabs(s4/vmax4) + vmax4/amax);
        if(1000*tf >= 50)
        {
            this->velocityProfile(s1, amax, s4, amax, tf);
        }
        else
        {
            this->rawMoveToPoint(D);
        }
        //Serial.printf("\nMoved to (%f, %f)\n", D.x, D.y);
        //Serial.printf("\nCurrent Position XY(%f, %f), Pul(%d,%d)\n", this->_realX, this->_realY, this->_realPul1, this->_realPul4);
        return true;
    }

}

bool ScaraRobot::rawMoveToPoint(point D)
{
    if(!(this->isInWorkSpace(D)))
    {
        // Serial2.printf("\nThe destination (%f, %f) is not in the Workspace\n", D.x, D.y);
        return false;
    }
    else if(!isInCurrentWorkSpace(D))
    {
        // Serial2.printf("\nThe destination (%f, %f) is not in the same Workspace\n", D.x, D.y);
        return false;
    }
    else if ((fabs(this->_realX - D.x) <= 0.05)&&(fabs(this->_realY - D.y) <= 0.05))
    {
        // Serial2.printf("/n Already at the destination, curren pos: %f, %f", this->_realX, this->_realY);
        return true;
    }
    else
    {
        float* theta;
        theta = (this->_workspace)? pointToAngle1(D.x,D.y): pointToAngle0(D.x,D.y);
        float s1_pul = theta[0]*(this->_stepPerDeg) - (this->_realPul1);
        float s4_pul = theta[1]*(this->_stepPerDeg) - (this->_realPul4);
        for(int i = 0; i<4; i++)
        {
            this->setSpd1(s1_pul/4);
            this->setSpd4(s4_pul/4);
        }
        s1_pul = theta[0]*(this->_stepPerDeg) - (this->_realPul1);
        s4_pul = theta[1]*(this->_stepPerDeg) - (this->_realPul4);
        this->setSpd1(s1_pul);
        this->setSpd4(s4_pul);
        // Serial2.printf("\nRaw moved to (%f, %f)\n", D.x, D.y);
        // Serial2.printf("\nCurrent Position XY(%f, %f), Pul(%d,%d)\n", this->_realX, this->_realY, this->_realPul1, this->_realPul4);
        return true;
    }

}


bool ScaraRobot::moveToPointCirculation(point D, int vMax, int aMax)
{
    if(!(this->isInWorkSpace(D)))
    {
        // Serial2.printf("\nThe destination (%f, %f) is not in the Workspace\n", D.x, D.y);
        return false;
    }
    else if(!isInCurrentWorkSpace(D))
    {
        // Serial2.printf("\nThe destination (%f, %f) is not in the same Workspace\n", D.x, D.y);
        return false;
    }
    else if ((fabs(this->_realX - D.x) <= 0.05)&&(fabs(this->_realY - D.y) <= 0.05))
    {
        // Serial2.println("Already at the destination");
        return true;
    }
    else
    {
        // Serial2.println("Start moving");
        float* theta;
        theta = (this->_workspace)? pointToAngle1(D.x,D.y): pointToAngle0(D.x,D.y);
        // Serial2.printf("Current Workspace: %d, xD: %f, yD: %f, Theta1:%f Theta4:%f", this->_workspace, D.x, D.y, theta[0], theta[1]);
        float s1 = theta[0] - (this->_realPul1)*(this->_degPerStep);
        float s4 = theta[1] - (this->_realPul4)*(this->_degPerStep);
        float amax = aMax;
        float vmax1 = calVmax(s1,amax,vMax), vmax4 = calVmax(s4,amax,vMax);
        float tf = max(fabs(s1/vmax1) + vmax1/amax, fabs(s4/vmax4) + vmax4/amax);
        if(1000*tf >= 60)
        {
            this->velocityProfile(s1, amax, s4, amax, tf);
        }
        else
        {
            this->rawMoveToPoint(D);
        }
        // Serial2.printf("\nMoved to (%f, %f), Real position (%f,%f)\n", D.x, D.y, this->_realX, this->_realY);
        return true;
    }

}

bool ScaraRobot::handleToPoint(point E)
{
    if(!(this->isInWorkSpace(E)))
    {
        // Serial2.printf("\nThe destination (%f, %f) is not in the Workspace\n", E.x, E.y);
        return false;
    }
    else
    {
        if(!isInCurrentWorkSpace(E))
        {
            this->switchO(round(this->_vmax)/2, round(this->_amax)/2);
            return this->moveToPoint(E);
        }

    }
    return this->moveToPoint(E);

}

bool ScaraRobot::handleToPointCirculation(point E, int vMax, int aMax)
{
    if(!(this->isInWorkSpace(E)))
    {
        //Serial.printf("\nThe destination (%f, %f) is not in the Workspace\n", E.x, E.y);
        return false;
    }
    else
    {
        if(!isInCurrentWorkSpace(E))
        {
            //Serial.println("The destination is in another Workspace");
            this->switchO(vMax, aMax);
            return this->moveToPointCirculation(E, vMax, aMax);
        }

    }
    //Serial.println("The destination is in same Workspace");
    return this->moveToPointCirculation(E, vMax, aMax);

}

bool ScaraRobot::moveInLine(point A1, point A2, int spd)
{
    if(!isInWorkSpace(A1)||!isInWorkSpace(A2))
    {
        // Serial2.printf("\n(%f, %f) and (%f, %f) are not in Workspace\n", A1.x, A1.y, A2.x, A2.y);
        return false;
    }
    else if (!isInCurrentWorkSpace(A1) || !isInCurrentWorkSpace(A2))
    {
        // Serial2.printf("\n(%f, %f) and (%f, %f) are not in the same Workspace\n", A1.x, A1.y, A2.x, A2.y);
        return false;
    }
    else if(!(this->moveToPoint(A1)))
    {
        // Serial2.println("fail move to A1");
        return false;
    }
    else
    {
        float d = sqrt(pow((A2.x-A1.x),2) + pow((A2.y-A1.y),2));
        int td = (int)round((d/spd))*100; 
        // Serial2.printf("\nStart moving from (%f, %f) to (%f, %f), d: %f, td: %d\n", this->_realX, this->_realY, A2.x, A2.y,d,td);
        float preTheta0, preTheta1;
        //preTheta = (this->_workingmode)? pointToAngle1(A1.x, A1.y) : pointToAngle0(A1.x, A1.y);
        float *theta;
        for(int t = 0; t < td; t = t + 2)
        {
            preTheta0= this->_realPul1 * this->_degPerStep;
            preTheta1= this->_realPul4 * this->_degPerStep;
            //Serial.printf("\npretheta1 & pretheta4 (%f, %f)x & y (%f, %f)\n", preTheta[0], preTheta[1], this->_realX, this->_realY);
            theta = (this->_workspace)? pointToAngle1((A1.x+(A2.x-A1.x)*(t+2)/td ), (A1.y+ (A2.y-A1.y)*(t+2)/td)) : pointToAngle0((A1.x+(A2.x-A1.x)*(t+2)/td), (A1.y+ (A2.y-A1.y)*(t+2)/td));
            // Serial2.printf("\nt: %d, next angle (%f, %f), real angle (%f,%f)\n", t,  theta[0], theta[1], preTheta0, preTheta1);
            this->setSpd1_1ms((theta[0]-preTheta0)*this->_stepPerDeg);
            this->setSpd4_1ms((theta[1]-preTheta1)*this->_stepPerDeg);
            
        }
        // Serial2.printf("\nReal current x,y (%f, %f)\n", this->_realX, this->_realY);
        //rawMoveToPoint(A2);
        // Serial2.printf("\nMoved from (%f, %f) to (%f, %f)\n", A1.x, A1.y, A2.x, A2.y);
        return true;
    }    
}

bool ScaraRobot::moveInCircle(point O, float R, float start, float finish, int spd)
{
    point O1, O2, O3, O4, Os, Of;
    O1.setPoint(O.x,(O.y+R));
    O2.setPoint(O.x,(O.y-R));
    O3.setPoint((O.x+R),O.y);
    O4.setPoint((O.x-R),O.y);
    Os.setPoint((O.x+R*cos(start)), (O.y+R*sin(start)));
    Of.setPoint((O.x+R*cos(finish)), (O.y+R*sin(finish)));
    // Serial2.printf("\nStart ((%f,%f), %f) to ((%f,%f), %f) R: %f\n", Os.x, Os.y, start, Of.x, Of.y, finish, R);

    if(!isInWorkSpace(O)||!isInWorkSpace(O1)||!isInWorkSpace(O2)||!isInWorkSpace(O3)||!isInWorkSpace(O4))
    {
        return false;
    }
    else if(!isInCurrentWorkSpace(O) || !isInCurrentWorkSpace(O1) || !isInCurrentWorkSpace(O2) || !isInCurrentWorkSpace(O3) || !isInCurrentWorkSpace(O4))
    {
        // Serial2.printf("\nAll parts of Circle ((%f,%f), %f) are not in the same Workspace\n", O.x, O.y, R);
        return false;
    }
    else if(!(this->moveToPoint(Os)))
    {
        return false;
    }
    else
    {
        float d = R*(finish-start);
        int td = (int)round((d/spd))*100; 
        float preTheta0, preTheta1;
        float* theta;
        // Serial2.printf("\nStart ((%f,%f), %f) to ((%f,%f), %f) R: %f\n", Os.x, Os.y, start, Of.x, Of.y, finish, R);
        for(int t = 0; t < td; t = t + 2)
        {
            preTheta0= this->_realPul1 * this->_degPerStep;
            preTheta1= this->_realPul4 * this->_degPerStep;
            theta = (this->_workspace)? pointToAngle1((O.x+R*cos(start+ (finish-start)/td * (t+2))), (O.y+R*sin(start+ (finish-start)/td * (t+2)))) : pointToAngle0((O.x+R*cos(start+ (finish-start)/td * (t+2))), (O.y+R*sin(start+ (finish-start)/td * (t+2))));
            // Serial2.printf("\nt: %d, next angle (%f, %f), real angle (%f,%f)\n", t,  theta[0], theta[1], preTheta0, preTheta1);
            this->setSpd1_1ms((theta[0]-preTheta0)*this->_stepPerDeg);
            this->setSpd4_1ms((theta[1]-preTheta1)*this->_stepPerDeg);
        }
        // Serial2.printf("\nReal position x,y (%f, %f)\n", this->_realX, this->_realY);
        //rawMoveToPoint(Of);
        // Serial2.printf("\nMoved in circle ((%f,%f), %f) from %f(rad) to %f(rad)\n", O.x, O.y, R, start, finish);
        return true;
    }

    

}

bool ScaraRobot::moveFromAToB(point A, point B)
{
    moveToPoint(A);
    if(A.workspace == B.workspace)
    {
        return this->moveToPoint(B);
    }
    else
    {
        this->switchWorkSpace();
        return this->moveToPoint(B);
    }
}

bool ScaraRobot::handleZ()
{
    digitalWrite(Z_PIN, HIGH);
    delay(500);
    digitalWrite(Z_PIN, LOW);
    return true;
}

bool ScaraRobot::downZ()
{
    // Serial2.println("down Z");
    digitalWrite(Z_PIN, HIGH);
    delay(200);
    return true;
}

bool ScaraRobot::upZ()
{
    // Serial2.println("up Z");
    digitalWrite(Z_PIN, LOW);
    return true;
}

bool ScaraRobot::onEndEffector()
{
    // Serial2.println("on effector");
    digitalWrite(END_EFFECTOR_PIN, HIGH);
    return true;
}

bool ScaraRobot::offEndEffector()
{   
    // Serial2.println("off effector");
    digitalWrite(END_EFFECTOR_PIN, LOW);
    return false;
}
;