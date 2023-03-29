#ifndef FIVE_BAR_ROBOT_H
#define FIVE_BAR_ROBOT_H

#include <Arduino.h>
#include <math.h> 

//define struct of Robot


// define the pin to send signal to driver
#define ENA_ROBOT 2
#define DIR_PIN_1 22
#define PUL_PIN_1 23
#define DIR_PIN_2 19
#define PUL_PIN_2 18
#define END_EFFECTOR_PIN 5
#define Z_PIN 4
#define CHECK_PIN_1 32
#define CHECK_PIN_2 33

enum eRequest
{   START,
    STOP,
	RESET,
    SWITCH_WORKSPACE,
    KINETIC,
	MOVL,
	MOVC,
	MOVJ,
    CIRCULAR,
    MOVQ
};

struct point
{
    float x = 0;
    float y = 10;
    bool workspace = true;
    void setPoint(float xA, float yA)
    {
        this->x = xA;
        this->y = yA;
        this->workspace = ((yA>=0)? true : false);
    }

};


class ScaraRobot
{
private:

    int _longLink = 225; //mm
    int _shortLink = 150;
    int _disBase = 150;

    int _realPul1 = 3200;
    int _realPul4 = 0;
    float _realX = 0;
    float _realY = 0;
    bool _workspace = true;
    bool _workingmode = true;
    uint8_t _taskToDo = STOP;
    uint8_t _microStep =  32; //step of stepper motor
    float _degPerStep = 1.8/_microStep;
    float _stepPerDeg = _microStep/1.8;
    float _vmax = 1080; //deg/second
    float _amax = 3*_vmax; //deg/second^2
    point _PO;
    point _NO;

public:
    void begin();       //khoi dong cac in out
    bool reset();   //calib tro ve vi tri 0
    void checkOMotor1();    //calib cua motor 1
    void checkOMotor2();    //calib cua motor 2
    void startRobot();  // Enable driver - kich relay de cap nguon cho dong co -- calib vi tri dong co
    void stopRobot();   // calib ve 0 - ngat ngon
    void loop();   // ko dung
    void loopMotor1(); // ko dung
    void loopMotor2(); // ko dung
    bool switchWorkSpace(); //chuyen vung lam viec

    void setLongLink(int longLength);   // Hieu chinh  khch thuoc robot - thanh dai
    void setShortLink(int shortLength); //hieu chinh kich thuoc robot - thanh ngan
    void setDisBase(int disBase);   //hieu chinh kich thuoc robot - khoang cacnh 2 dong co


    void setTaskToDo(uint8_t task); // xac dinh nhiem vu robot can lam - tach cac case
    uint8_t getTaskToDo();      // nhan dien truong hop can thuc hien
    void setSpd1(float spd1); //Set speed of stepper motor 1 in 10ms
    void setSpd1_1ms(float spd1); //Set speed of stepper motor 1 in 1ms
    void setSpd4(float spd2); //Set speed of stepper motor 2 in 10ms
    void setSpd4_1ms(float spd2);   //Set speed of stepper motor 1 in 1ms

    float calVmax(float s_max, float a_max, float v_max);   // tinh van toc toi da cua chu trinh hinh thang

    void velocityProfile1(float s1_max, float a1_max, float tf1);   // ko can dung
    void velocityProfile4(float s2_max, float a2_max, float tf2);   // ko dung
    void velocityProfile(float s1_max, float a1_max, float s2_max, float a2_max, float tf); //Hoach dinh van toc cho 2 dong co

    float* pointToAngle0(float x, float y); //Phan duong - chuyen doi vi tri (x,y) sang goc cua cac dong co
    float* pointToAngle1(float x, float y); // Phan am - chuyen doi vi tri (x,y) sang goc cua cac dong co
    float* angleToPoint0(float theta1, float theta4); // phan duong -  chuyen doi goc sang vi tri
    float* angleToPoint1(float theta1, float theta4);   // phan am -  chuyen doi goc sang vi tri

    

    void handleRequest(enum eRequest);  // thuc hien nhiem vu theo command

    void setWorkingMode(bool mode); // ko dung 
    void setWorkSpace(bool space);  // dat gia tri WS
    bool getWorkingMode();  // ko dung
    bool getWorkSpace();    // check WS 


    float getRealX();   // ly toa do X hien tai
    float getRealY();   // lay toa do Y hien tai
    float getRealTheta1();  //lay goc hien tai
    float getRealTheta4();  // lay goc hien tai
    int getRealPul1();  // lay vi tri theo xung hien tai
    int getRealPul4();  // lay vi tri theo xung hien tai

    void setVMax(int vMax); // set van toc toi da cua dong co
    int getVMax();  // lay gia tri toi da cua dong co
    void setAMax(int aMax); // set gia toc toi da 
    int getAMax();  // lay gia toc toi da


    bool calibZero();  // thuc hien calib zero
    void forwardKinetic();  //  tinh toan dong hoc thuan - angle to point
    void inverseKinetic();  // tinh toan dong hoc nghich - point to angle
    bool isInWorkSpace(point A);    // check 1 diem co trong khong gian lam viec hay ko
    bool isInCurrentWorkSpace(point B); // check 1 diem co the nam trong khong gian hien tai hay ko
    bool switchO(int vMax, int aMax);   //di chuyen tu (0,5) den (0,-5) hoac nguoc lai
    bool moveToPoint(point D);  //di chuyen den diem bat ki theo hinh thang da dat
    bool rawMoveToPoint(point D);   //  chi dung 1 loop di chuyen den diem dat - co khoang cach ngan
    bool moveToPointCirculation(point D, int vMax, int aMax);   // di chuyen den diem theo van toc hinh thang cai dat
    bool handleToPoint(point E);    // di chuyen giua hai diem trong 2 khong gian khac nhau
    bool handleToPointCirculation(point E, int vMax, int aMax); //di chuyen giua hai diem trong 2 khong gian khac nhau-  co cai dat van toc
    bool moveInLine(point A1, point A2, int spd);  // chay theo duong thang
    bool moveInCircle(point O, float R, float start, float finish, int spd);  // chay theo cung tron
    bool onEndEffector();   // bat nam cham
    bool offEndEffector();  // tat nam cham
    bool handleZ(); // cung xuong len cach 500ms
    bool downZ();   // xuong xilanh
    bool upZ(); // len xi lanh
    bool moveFromAToB(point A, point B); // di chuyen tu diem A den diem B


};
extern ScaraRobot Robot;


#endif