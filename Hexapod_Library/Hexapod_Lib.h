/******************************************************************
Created with PROGRAMINO IDE for Arduino
Libraries   : SoftwareSerial.h
Author      : UlliS
******************************************************************/
#ifndef __HEXAPOD_LIB_HEADER__
#define __HEXAPOD_LIB_HEADER__

#include <SoftwareSerial.h>

// DEBUG OUTPUT
//#define DEBUG_OUT

// Is defined in the User-Sketch
//#define ARDUINO
//#define NODEMCU

// UART
#define SERIAL_CMD_BAUD         38400       // Locomotion-Controller to User-Board
#define SERIAL_STD_BAUD         115200      // USB Connector (Arduino-UNO/NodeMCU)

// SERIAL PROTOCOLL
#define CMD_SYNC0               33          // Sync byte 1
#define CMD_SYNC1               42          // Sync byte 2
#define CMD_TERM_BYTE           255         // Termination byte
int cmd_crc     = 0;                        // CRC byte
int cmd         = 0;                        // CMD byte
int status_byte = 0;                        // Status byte
int cmd_param[6];                           // Parameter
int TxErrorCnt = 0;                         // Error counter

// CMD REGISTER [WRITE-VALUES]
#define CMD_REG_POWER           5
#define CMD_REG_SPEED           10
#define CMD_REG_GAINT_MODE      15
#define CMD_REG_BALANCE_MODE    20
#define CMD_REG_BODY_HEIGHT     25
//#define CMD_REG_WALK_MODE       30
#define CMD_REG_TRANSLATE       35
#define CMD_REG_WALK            40
#define CMD_REG_ROTATE          45
#define CMD_REG_DOUBLE_HEIGHT   50
#define CMD_REG_DOUBLE_LENGTH   55
#define CMD_REG_SINGLE_LEG_POS  60
#define CMD_REG_SOUND           65
#define CMD_REG_OUT1            70
#define CMD_REG_STATUS_LED      75

// CMD-REGISTER [READ-VALUES]
#define CMD_REG_SA_LEG          100
#define CMD_REG_AKKU            105
#define CMD_REG_PS2_ACTIVE      110
#define CMD_REG_IS_WALKING      115
#define CMD_REG_IS_POWER_ON     120
#define CMD_REG_READ_PS2_VALUES 125
#define CMD_REG_IN1             130

// CMD-REGISTER [HW-RESET]
#define CMD_REG_RESET           255

// CMD STATUS FEEDBACK
#define STATUS_ACK_OK           64
#define STATUS_ERR_TERM         1
#define STATUS_ERR_STATE        2
#define STATUS_ERR_CRC          3
#define STATUS_ERR_CMD          255

// MOVE MODES
#define WALKMODE                0
#define TRANSLATEMODE           1
#define ROTATEMODE              2
#define SINGLELEGMODE           3

#define BALANCEMODE_ON          1
#define BALANCEMODE_OFF         0

// GAIN MODES
enum GAIN_MODES 
{
    TRIPOD_6,
    TRIPOD_8,
    TRIPPLE_12,
    TRIPPLE_16,
    RIPPLE_12,
    WAVE_24,
};

// USER-BOARD PINS (ARDUINO UNO or compatible)
#ifdef ARDUINO
    #define T1                      A2
    #define T2                      A3
    #define PA_PIN                  3
    #define IR_DAT                  7
    #define SU1                     6
    #define SU2                     5
    #define SU3                     9
    #define RXD_U                   4
    #define TXD_U                   2
#endif

// USER-BOARD PINS (NodeMCU)
#ifdef NODEMCU
    #define T1                      10
    #define T2                      A0
    #define PA_PIN                  14
    #define SU1                     12
    #define SU2                     13
    #define SU3                     15
    #define RXD_U                   0
    #define TXD_U                   2
#endif    

// SOUND FUNCTION
void MSound(byte cNotes, ...);
void SoundNoTimer(unsigned long duration,  unsigned int frequency);

// UART SEND- and READ-COMMANDS
void SendData(byte _cmd, byte _data1, byte _data2, byte _data3, byte _data4);
bool ReceiveAck(void);
void ErrorHandle(void);
bool CheckForSerialData(void);
bool CheckRxCrc(void);

// ROBOT READ-COMMANDS
float ReadVoltage(void);
bool ReadPs2Status(void);
bool ReadIsWalking(void);
bool ReadIsPowerOn(void);
int ReadLegAdc(byte idx);
byte ReadIN1(void);

unsigned int ps2x_ButtonData = 0;
byte ps2x_Analog_PSS_LX = 0;
byte ps2x_Analog_PSS_LY = 0;
byte ps2x_Analog_PSS_RX = 0;
byte ps2x_Analog_PSS_RY = 0;
bool ReadPs2Values(void);

// ROBOT SEND-COMMANDS
void ROBOT_INIT(void);
void ROBOT_RESET(void);
void ROBOT_MOVE(byte _lateral, byte _turn, byte _move);
void ROBOT_WALK_FWD(void);
void ROBOT_WALK_BWD(void);
void ROBOT_WALK_LEFT(void);
void ROBOT_WALK_RIGHT(void);
void ROBOT_STOP(void);
void ROBOT_TURN_LEFT(void);
void ROBOT_TURN_RIGHT(void);
void ROBOT_PWR_ON(void);
void ROBOT_PWR_OFF(void);
void ROBOT_HEIGHT(byte _hight);
void ROBOT_GAINT_MODE(byte gaint_mode);
void ROBOT_ROTATE_MODE(byte _x, byte _y, byte _z, byte _BodyYShift);
void ROBOT_TRANSLATE_MODE(byte _x, byte _y, byte _z, byte _BodyYShift);
void ROBOT_BALANCE_MODE_ON(void);
void ROBOT_BALANCE_MODE_OFF(void);
void ROBOT_DOUBLE_HEIGHT_ON(void);
void ROBOT_DOUBLE_HEIGHT_OFF(void);
void ROBOT_DOUBLE_LENGTH_ON(void);
void ROBOT_DOUBLE_LENGTH_OFF(void);
void ROBOT_PLAY_TONE(byte _duration, byte _frequency);
void ROBOT_SPEED(byte _speed);

/******************************************************************************
SOFT-SERIAL
******************************************************************************/
SoftwareSerial SERIAL_CMD(RXD_U, TXD_U);

/******************************************************************************
SOUND FUNCTION
******************************************************************************/
void MSound(byte cNotes, ...)
{
    va_list  ap;
    unsigned int uDur;
    unsigned int uFreq;
    va_start(ap, cNotes);
    
    while (cNotes > 0)
    {
        uDur  = va_arg(ap, unsigned int);
        uFreq = va_arg(ap, unsigned int);
        SoundNoTimer(uDur, uFreq);
        cNotes--;
    }
    va_end(ap);
    pinMode(PA_PIN, INPUT); // high-Z
}

void SoundNoTimer(unsigned long duration,  unsigned int frequency)
{
    #ifndef __MK20DX256__
    #ifdef __AVR__
        volatile uint8_t *pin_port;
        volatile uint8_t pin_mask;
    #else
        volatile uint32_t *pin_port;
        volatile uint16_t pin_mask;
    #endif
    
    long toggle_count = 0;
    long lusDelayPerHalfCycle;
    
    // set the pinMode as OUTPUT
    pinMode(PA_PIN, OUTPUT);
    
    pin_port = portOutputRegister(digitalPinToPort(PA_PIN));
    pin_mask = digitalPinToBitMask(PA_PIN);
    
    toggle_count         = 2 * frequency * duration / 1000;
    lusDelayPerHalfCycle = 1000000L / (frequency * 2);
    
    // if we are using an 8 bit timer, scan through prescalars to find the best fit
    while(toggle_count--)
    {
        // toggle the pin
        *pin_port ^= pin_mask;
        
        // delay a half cycle
        delayMicroseconds(lusDelayPerHalfCycle);
    }
    *pin_port &= ~(pin_mask);  // keep pin low after stop
#else
    // the tone command does sort of work, but does not play multiple sounds smoothly
    long    toggle_count = 0;
    long    lusDelayPerHalfCycle;
    boolean fHigh = false;
    
    // set the pinMode as OUTPUT
    pinMode(SOUND_PIN, OUTPUT);
    digitalWrite(SOUND_PIN, LOW);
    toggle_count         = 2 * frequency * duration / 1000;
    lusDelayPerHalfCycle = 1000000L /(frequency * 2);
    
    // if we are using an 8 bit timer, scan through prescalars to find the best fit
    while(toggle_count--)
    {
        // toggle the pin
        fHigh = !fHigh;
        digitalWrite(SOUND_PIN, fHigh? LOW : HIGH);
        // delay a half cycle
        delayMicroseconds(lusDelayPerHalfCycle);
    }
    digitalWrite(SOUND_PIN, LOW);
#endif
}

/******************************************************************************
UART SEND- and READ-COMMANDS
******************************************************************************/
void SendData(byte _cmd, byte _data1, byte _data2, byte _data3, byte _data4)
{
    // reset error counter
    TxErrorCnt = 0;
    
    // calc CRC
    byte _crc = CMD_SYNC0 ^ CMD_SYNC1 ^ _cmd ^ _data1 ^ _data2 ^ _data3 ^ _data4;
    
    // send bytes
    do
    {
        SERIAL_CMD.write(CMD_SYNC0);
        SERIAL_CMD.write(CMD_SYNC1);
        SERIAL_CMD.write(_crc);
        
        SERIAL_CMD.write(_cmd);
        SERIAL_CMD.write(_data1);
        SERIAL_CMD.write(_data2);
        SERIAL_CMD.write(_data3);
        SERIAL_CMD.write(_data4);
        SERIAL_CMD.write(CMD_TERM_BYTE);
        
        // increment the error counter
        if(TxErrorCnt > 10)ErrorHandle(); // exit
        TxErrorCnt++;
        
    }while(!ReceiveAck());
    
    #ifdef DEBUG_OUT
        Serial.print("Error: ");
        Serial.println(TxErrorCnt);
    #endif
}

bool ReceiveAck(void)
{
    boolean status = false;
    
    // read serial data
    if(CheckForSerialData())
    {
        if(CheckRxCrc())
        {
            if(status_byte == STATUS_ACK_OK)
            {
                #ifdef DEBUG_OUT
                    Serial.println("> ACK");
                #endif
                status = true;
            } 
            else
            {
                #ifdef DEBUG_OUT
                    if(status_byte == STATUS_ERR_TERM)Serial.println("> STATUS_ERR_TERM");
                    if(status_byte == STATUS_ERR_STATE)Serial.println("> STATUS_ERR_STATE");
                    if(status_byte == STATUS_ERR_CRC)Serial.println("> STATUS_ERR_CRC");
                    if(status_byte == STATUS_ERR_CMD)Serial.println("> STATUS_ERR_CMD");
                #endif
                status = false;
            }
        }
    }  
    
    // clear the serial buffer
    SERIAL_CMD.flush();
    
    return status;
}

void ErrorHandle(void)
{
    #ifdef DEBUG_OUT
        Serial.println("COMMUNICATION ERROR!");
    #endif
    
    // do anything else here :-)
    
    while(1)
    {
        MSound(1, 100, 1000);
        delay(500);
    }
}

bool CheckForSerialData(void)
{
    // received data frame from Locomotion-Controller
    // [SYNC0],[SYNC1],[CRC],[STATUS_BYTE],[CMD],[PARAM0],[PARAM1],[PARAM2],[PARAM3],[PARAM4],[PARAM5],[TERMINATION_BYTE]
    
    enum DATA_STATE
    {
        WAIT_FOR_SYNC_0,
        WAIT_FOR_SYNC_1,
        GET_CRC,
        GET_STATUS,
        GET_CMD,
        GET_PARAM0,
        GET_PARAM1,
        GET_PARAM2,
        GET_PARAM3,
        GET_PARAM4,
        GET_PARAM5,
        GET_TERM_CHAR,
    };
    
    // first state
    static int SerialState = WAIT_FOR_SYNC_0;
    byte _temp;
    byte _serialTimout = 0;
    
    // wait for data... 
    // the complete frame has 11 bytes
    while(SERIAL_CMD.available() < 10)
    {
        delay(10);
        _serialTimout++;
        if(_serialTimout>50)break; // max. 500ms -> timeout
    }
    
    // now, read data
    while(SERIAL_CMD.available() > 0)
    {
        byte cTemp = SERIAL_CMD.read();
        switch(SerialState)
        {
          case WAIT_FOR_SYNC_0:
            // looking for first sync byte
            if(cTemp == CMD_SYNC0)
            {        
                SerialState++; // first sync byte found, go to next state
            }
            break;
            
          case WAIT_FOR_SYNC_1:
            // looking for second sync byte
            if( cTemp == CMD_SYNC1)
            {
                SerialState++; // second sync byte found
            } 
            else 
            {
                SerialState = WAIT_FOR_SYNC_0; // failed sync
                while(SERIAL_CMD.available()>0)_temp = SERIAL_CMD.read(); // clear the buffer
                SERIAL_CMD.flush();
            }
            break;
            
          case GET_CRC:
            // get crc byte
            cmd_crc = cTemp;
            SerialState++;
            break;
            
          case GET_STATUS:
            // get status byte
            status_byte = cTemp;
            SerialState++;
            break;
            
          case GET_CMD:
            // get CMD byte
            cmd = cTemp;
            SerialState++;
            break;
            
          case GET_PARAM0:
            // get first Parameter
            cmd_param[0] = cTemp;
            SerialState++;
            break;
            
          case GET_PARAM1:
            cmd_param[1] = cTemp;
            SerialState++;
            break;
            
          case GET_PARAM2:
            cmd_param[2] = cTemp;
            SerialState++;
            break;
            
          case GET_PARAM3:
            cmd_param[3] = cTemp;
            SerialState++;
            break;
            
          case GET_PARAM4:
            cmd_param[4] = cTemp;
            SerialState++;
            break;
            
          case GET_PARAM5:
            cmd_param[5] = cTemp;
            SerialState++;
            break;
            
          case GET_TERM_CHAR:
            // check for terminator char
            if(cTemp == CMD_TERM_BYTE) 
            {
                // got the full command!
                SerialState = WAIT_FOR_SYNC_0; // ready to get the next command
                return true;                   // done!
            } 
            else 
            {
                // error
                SERIAL_CMD.write(STATUS_ERR_TERM);
                SerialState = WAIT_FOR_SYNC_0;
                while(SERIAL_CMD.available()>0)_temp = SERIAL_CMD.read(); // clear buffer
                SERIAL_CMD.flush();
            }
            break;
            
          default:
            // error
            SERIAL_CMD.write(STATUS_ERR_STATE);
            SerialState = WAIT_FOR_SYNC_0;      // abort and get next command
            while(SERIAL_CMD.available()>0)_temp = SERIAL_CMD.read(); // clear buffer
            SERIAL_CMD.flush();
        }
    }    
    
    return false;
    
}

bool CheckRxCrc(void)
{
    byte _crc = CMD_SYNC0 ^ CMD_SYNC1 ^ status_byte ^ cmd ^ cmd_param[0] ^ cmd_param[1] ^ cmd_param[2] ^ cmd_param[3] ^ cmd_param[4] ^ cmd_param[5];
    if(_crc == cmd_crc)return true;
    return false;
}

/******************************************************************************
ROBOT READ-COMMANDS
******************************************************************************/
float ReadVoltage(void)
{
    float volt         = 0.0;
    float voltPerDigit = (5.0/1023.0);  // (Aref / ADC-Resolution)
    float mul          = 4.30;          // (R1 / R2) + 1 -> adjust the value for your R1 and R2
    
    // send request
    SendData(CMD_REG_AKKU, 0, 0, 0, 0);
    
    // evaluate data and pass the variables
    if(cmd == CMD_REG_AKKU) 
    {
        // cmd_param[0] = hight byte, cmd_param[1] = low byte
        volt = (((cmd_param[0] << 8) + cmd_param[1]) * voltPerDigit) * mul;
    }    
    return volt;
}

bool ReadPs2Status(void)
{
    // send request
    SendData(CMD_REG_PS2_ACTIVE, 0, 0, 0, 0);
    
    // cmd_param[0] is status byte (0 = not active, 1 = active)
    if(cmd == CMD_REG_PS2_ACTIVE)
    {
        return cmd_param[0];
    }
    return false;
}

bool ReadPs2Values(void)
{
    // send request
    SendData(CMD_REG_READ_PS2_VALUES, 0, 0, 0, 0);
    
    // cmd_param[0] = ButtonData hight byte
    // cmd_param[1] = ButtonData low byte
    // cmd_param[2] = Joystick left X
    // cmd_param[3] = Joystick left Y
    // cmd_param[4] = Joystick right X
    // cmd_param[5] = Joystick right Y
    if(cmd == CMD_REG_READ_PS2_VALUES) 
    {
        ps2x_ButtonData    = (cmd_param[0] << 8) + cmd_param[1];
        ps2x_Analog_PSS_LX = cmd_param[2];
        ps2x_Analog_PSS_LY = cmd_param[3];
        ps2x_Analog_PSS_RX = cmd_param[4];
        ps2x_Analog_PSS_RY = cmd_param[5];
    } 
    else 
    {  
        return false;
    }
    
    return true;
}

bool ReadIsWalking(void)
{
    // send request
    SendData(CMD_REG_IS_WALKING, 0, 0, 0, 0);
    
    // cmd_param[0] is status byte (0 = stop, 1 = walking)
    if(cmd == CMD_REG_IS_WALKING)
    {
        return cmd_param[0];
    }
    
    return false;
}

bool ReadIsPowerOn(void)
{
    // send request
    SendData(CMD_REG_IS_POWER_ON, 0, 0, 0, 0);
    
    // cmd_param[0] is status byte (0 = off, 1 = on)
    if(cmd == CMD_REG_IS_POWER_ON)
    {
        return cmd_param[0];
    }
    
    return false;
}

int ReadLegAdc(byte idx)
{
    // send request
    // idx is SA0 to SA5 (SA0 = 0 and SA5 = 5)
    SendData(CMD_REG_SA_LEG, idx, 0, 0, 0);
    
    // cmd_param[0] = hight byte, cmd_param[1] = low byte
    if(cmd == CMD_REG_SA_LEG)
    {
        return (cmd_param[0] << 8) + cmd_param[1];
    }
    
    return 0;
}

byte ReadIN1(void)
{
    // send request
    SendData(CMD_REG_IN1, 0, 0, 0, 0);
    
    // cmd_param[0] is status byte (0 = pin to ground, 1 = open)
    if(cmd == CMD_REG_IN1)
    {
        return cmd_param[0];
    }
    return 0;
}

/******************************************************************************
ROBOT SEND-COMMANDS
******************************************************************************/
void ROBOT_INIT(void)
{
    ROBOT_STOP();
    ROBOT_SPEED(100);
    ROBOT_HEIGHT(0);
    ROBOT_GAINT_MODE(TRIPOD_6);
    ROBOT_PWR_OFF();
}

void ROBOT_RESET(void)
{
    // send data w/o check the ack or other tings!
    // brute force :-)
    byte _cmd   = 255;
    byte _data1 = 100;
    byte _data2 = 100;
    byte _data3 = 100;
    byte _data4 = 0;
    
    // calc CRC
    byte _crc = CMD_SYNC0 ^ CMD_SYNC1 ^ _cmd ^ _data1 ^ _data2 ^ _data3 ^ _data4;
    
    SERIAL_CMD.write(CMD_SYNC0);
    SERIAL_CMD.write(CMD_SYNC1);
    SERIAL_CMD.write(_crc);
    
    SERIAL_CMD.write(_cmd);
    SERIAL_CMD.write(_data1);
    SERIAL_CMD.write(_data2);
    SERIAL_CMD.write(_data3);
    SERIAL_CMD.write(_data4);
    SERIAL_CMD.write(CMD_TERM_BYTE);
}

void ROBOT_MOVE(byte _lateral, byte _turn, byte _move)
{
    //-----------------------
    // lateral [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    //-----------------------
    // turn [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    //-----------------------
    // move [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed forward
    // 255  = full speed backward
    
    SendData(CMD_REG_WALK, _lateral, _move, _turn, 0);
}

void ROBOT_WALK_FWD(void)
{
    // [lateral],[move],[turn]
    // move = 0 -> max speed forward
    
    //-----------------------
    // lateral [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    //-----------------------
    // move [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed forward
    // 255  = full speed backward
    
    //-----------------------
    // turn [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    SendData(CMD_REG_WALK, 128, 0, 128, 0);
}

void ROBOT_WALK_BWD(void)
{
    // [lateral],[move],[turn]
    // move = 255 -> max speed backward
    
    //-----------------------
    // lateral [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    //-----------------------
    // move [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed forward
    // 255  = full speed backward
    
    //-----------------------
    // turn [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    SendData(CMD_REG_WALK, 128, 255, 128, 0);
}

void ROBOT_WALK_LEFT(void)
{
    // [lateral],[move],[turn]
    // lateral = 0 -> max speed
    
    //-----------------------
    // lateral [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    //-----------------------
    // move [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed forward
    // 255  = full speed backward
    
    //-----------------------
    // turn [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    SendData(CMD_REG_WALK, 0, 128, 128, 0);
}

void ROBOT_WALK_RIGHT(void)
{
    // [lateral],[move],[turn]
    // lateral = 255 -> max speed
    
    //-----------------------
    // lateral [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    //-----------------------
    // move [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed forward
    // 255  = full speed backward
    
    //-----------------------
    // turn [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    SendData(CMD_REG_WALK, 255, 128, 128, 0);
}

void ROBOT_STOP(void)
{
    // [lateral],[move],[turn]
    // all = 128 -> stop
    
    //-----------------------
    // lateral [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    //-----------------------
    // move [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed forward
    // 255  = full speed backward
    
    //-----------------------
    // turn [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    SendData(CMD_REG_WALK, 128, 128, 128, 0);
}

void ROBOT_TURN_LEFT(void)
{
    // [lateral],[move],[turn]
    // turn = 0 -> max speed left
    
    //-----------------------
    // lateral [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    //-----------------------
    // move [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed forward
    // 255  = full speed backward
    
    //-----------------------
    // turn [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    SendData(CMD_REG_WALK, 128, 128, 0, 0);
}

void ROBOT_TURN_RIGHT(void)
{
    // [lateral],[move],[turn]
    // turn = 255 -> max speed right
    
    //-----------------------
    // lateral [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    //-----------------------
    // move [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed forward
    // 255  = full speed backward
    
    //-----------------------
    // turn [0 to 255]
    //-----------------------
    // 128  = stop
    // 0    = full speed left
    // 255  = full speed right
    
    SendData(CMD_REG_WALK, 128, 128, 255, 0);
}

void ROBOT_PWR_ON(void)
{
    // [CMD_REG_POWER],[1 = on, 0 = off],[0],[0],[0]
    SendData(CMD_REG_POWER, 1, 0, 0, 0);
}

void ROBOT_PWR_OFF(void)
{
    // [CMD_REG_POWER],[1 = on, 0 = off],[0],[0],[0]
    SendData(CMD_REG_POWER, 0, 0, 0, 0);
}

void ROBOT_HEIGHT(byte _hight)
{
    // [CMD_REG_HEIGHT],[hight as mm],[0],[0],[0]
    SendData(CMD_REG_BODY_HEIGHT, _hight, 0, 0, 0);
}

void ROBOT_GAINT_MODE(byte gaint_mode)
{
    // [CMD_REG_GAINT_MODE],[gaint_mode],[0],[0],[0]
    
    // TRIPOD_6     = 0
    // TRIPOD_8     = 1
    // TRIPPLE_12   = 2
    // TRIPPLE_16   = 3
    // RIPPLE_12    = 4
    // WAVE_24      = 5
    
    SendData(CMD_REG_GAINT_MODE, gaint_mode, 0, 0, 0);
}

void ROBOT_ROTATE_MODE(byte _x, byte _y, byte _z, byte _BodyYShift)
{
    //-----------------------
    // x [0 to 255]
    //-----------------------
    // 128  = center
    // 0    = body tilt to the right
    // 255  = body tilt to the left
    
    //-----------------------
    // y [0 to 255]
    //-----------------------
    // 128  = center
    // 0    = body turn to left 
    // 255  = body turn to right 
    
    //-----------------------
    // z [0 to 255]
    //-----------------------
    // 128  = center
    // 0    = body tilt backward (up)
    // 255  = body tilt forward (down)
    
    //-----------------------
    // BodyYShift [0 to 255]
    //-----------------------
    // 128  = center
    // 0    = max offset
    // 255  = min offset
    
    SendData(CMD_REG_ROTATE, _x, _y, _z, _BodyYShift);
}

void ROBOT_TRANSLATE_MODE(byte _x, byte _y, byte _z, byte _BodyYShift)
{
    //-----------------------
    // x [0 to 255]
    //-----------------------
    // 128  = center
    // 0    = body tilt to the left
    // 255  = body tilt to the right
    
    //-----------------------
    // y [0 to 255]
    //-----------------------
    // 128  = center
    // 0    = body turn to left 
    // 255  = body turn to right 
    
    //-----------------------
    // z [0 to 255]
    //-----------------------
    // 128  = center
    // 0    = body tilt backward (up)
    // 255  = body tilt forward (down)
    
    //-----------------------
    // BodyYShift [0 to 255]
    //-----------------------
    // 128  = center
    // 0    = max offset
    // 255  = min offset
    SendData(CMD_REG_TRANSLATE, _x, _y, _z, _BodyYShift);
}

void ROBOT_SINGLE_LEG(byte _idx, byte _x, byte _y, byte _z)
{
    //-----------------------
    // idx [0 to 5]
    //-----------------------
    
    //-----------------------
    // x [0 to 255]
    //-----------------------
    
    //-----------------------
    // y [0 to 255]
    //-----------------------
    
    //-----------------------
    // z [0 to 255]
    //-----------------------
    
    SendData(CMD_REG_SINGLE_LEG_POS, _idx, _x, _y, _z);
}

void ROBOT_BALANCE_MODE_ON(void)
{
    // [CMD_REG_BALANCE_MODE],[1 = on, 0 = off],[0],[0],[0]
    SendData(CMD_REG_BALANCE_MODE, 1, 0, 0, 0);
}

void ROBOT_BALANCE_MODE_OFF(void)
{
    // [CMD_REG_BALANCE_MODE],[1 = on, 0 = off],[0],[0],[0]
    SendData(CMD_REG_BALANCE_MODE, 0, 0, 0, 0);
}

void ROBOT_DOUBLE_HEIGHT_ON(void)
{
    // [CMD_REG_DOUBLE_HEIGHT],[1 = on, 0 = off],[0],[0],[0]
    SendData(CMD_REG_DOUBLE_HEIGHT, 0, 0, 0, 0);
}

void ROBOT_DOUBLE_HEIGHT_OFF(void)
{
    // [CMD_REG_DOUBLE_HEIGHT],[1 = on, 0 = off],[0],[0],[0]
    SendData(CMD_REG_DOUBLE_HEIGHT, 0, 0, 0, 0);
}

void ROBOT_DOUBLE_LENGTH_ON(void)
{
    // [CMD_REG_DOUBLE_LENGTH],[1 = on, 0 = off],[0],[0],[0]
    SendData(CMD_REG_DOUBLE_LENGTH, 1, 0, 0, 0);
}

void ROBOT_DOUBLE_LENGTH_OFF(void)
{
    // [CMD_REG_DOUBLE_LENGTH],[1 = on, 0 = off],[0],[0],[0]
    SendData(CMD_REG_DOUBLE_LENGTH, 0, 0, 0, 0);
}

void ROBOT_PLAY_TONE(byte _duration, byte _frequency)
{
    // [CMD_REG_SOUND],[duration],[frequency],[0],[0]
    // duration     = 1 to 255
    // frequency    = 1 to 255
    
    SendData(CMD_REG_SOUND, _duration, _frequency, 0, 0);
}

void ROBOT_SPEED(byte _speed)
{
    // [CMD_REG_SPEED],[speed],[0],[0],[0]
    // Speed = 10 to 255 (10 is max speed -> is fast)
    SendData(CMD_REG_SPEED, _speed, 0, 0, 0);
}

void WriteOUT1(byte _state)
{
    // [CMD_REG_OUT1],[1 = on, 0 = off],[0],[0],[0]
    SendData(CMD_REG_OUT1, _state, 0, 0, 0);
}

#endif
