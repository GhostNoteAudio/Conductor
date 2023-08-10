#include <Adafruit_TinyUSB.h>
#include "EEPROM.h"

Adafruit_USBD_MIDI usb_midi;

#define Serial SerialTinyUSB

int SliderMinimum = 3;
int SliderSections[8] = 
{
  27,
  56,
  103,
  166,
  252,
  439,
  730,
  1023
};

double SliderSlope[8] = 
{
  5.333333333,
  4.413793103,
  2.723404255,
  2.031746032,
  1.488372093,
  0.684491979,
  0.439862543,
  0.436860068
};

int midiCC[3] = {0};
int midiChannel[3] = {0};
double slider[3] = {0};
int midiVal[3] = {0};
double lastTransmitFloat[3] = {-1};
int lastTransmitInt[3] = {-1};

uint8_t sysexData[32];
int sysexIdx;

int TranslateLog2lin(int value)
{
    if (value < SliderMinimum)
        return 0;
    else if (value < SliderSections[0])
        return SliderSlope[0] * (value - SliderMinimum);
    else if (value < SliderSections[1])
        return SliderSlope[1] * (value - SliderSections[0]) + 128;
    else if (value < SliderSections[2])
        return SliderSlope[2] * (value - SliderSections[1]) + 256;
    else if (value < SliderSections[3])
        return SliderSlope[3] * (value - SliderSections[2]) + 384;
    else if (value < SliderSections[4])
        return SliderSlope[4] * (value - SliderSections[3]) + 512;
    else if (value < SliderSections[5])
        return SliderSlope[5] * (value - SliderSections[4]) + 640;
    else if (value < SliderSections[6])
        return SliderSlope[6] * (value - SliderSections[5]) + 768;
    else if (value < SliderSections[7])
        return SliderSlope[7] * (value - SliderSections[6]) + 896;
    else
        return 1023;
}

void printSettings()
{
    Serial.print("Midi CC#: ");
    Serial.print(midiCC[0]);
    Serial.print(", ");
    Serial.print(midiCC[1]);
    Serial.print(", ");
    Serial.print(midiCC[2]);
    Serial.println("");

    Serial.print("Midi Channel: ");
    Serial.print(midiChannel[0]);
    Serial.print(", ");
    Serial.print(midiChannel[1]);
    Serial.print(", ");
    Serial.print(midiChannel[2]);
    Serial.println("");
}

void loadSettings()
{
    Serial.println("Loading settings from flash memory...");
    EEPROM.begin(512);
    midiCC[0] = EEPROM.read(0);
    midiCC[1] = EEPROM.read(1);
    midiCC[2] = EEPROM.read(2);
    midiChannel[0] = EEPROM.read(3);
    midiChannel[1] = EEPROM.read(4);
    midiChannel[2] = EEPROM.read(5);
    printSettings();
}

void storeSettings()
{
    Serial.println("Storing settings in flash memory...");
    Serial.flush();
    TinyUSB_Device_FlushCDC();
    TinyUSB_Device_Task();

    printSettings();
    Serial.flush();
    TinyUSB_Device_FlushCDC();
    TinyUSB_Device_Task();

    EEPROM.write(0, midiCC[0]);
    EEPROM.write(1, midiCC[1]);
    EEPROM.write(2, midiCC[2]);

    EEPROM.write(3, midiChannel[0]);
    EEPROM.write(4, midiChannel[1]);
    EEPROM.write(5, midiChannel[2]);

    EEPROM.commit();

    delay(1000);
    Serial.flush();
    TinyUSB_Device_FlushCDC();
    TinyUSB_Device_Task();

    midiCC[0] = EEPROM.read(0);
    midiCC[1] = EEPROM.read(1);
    midiCC[2] = EEPROM.read(2);
    midiChannel[0] = EEPROM.read(3);
    midiChannel[1] = EEPROM.read(4);
    midiChannel[2] = EEPROM.read(5);

    printSettings();
    Serial.flush();
    TinyUSB_Device_FlushCDC();
    TinyUSB_Device_Task();
}

void setup() 
{
    TinyUSB_Device_Init(0);
    Serial.begin(115200);
    Serial1.begin(31250);
    usb_midi.setStringDescriptor("Ghost Note Conductor");
    bool res = usb_midi.begin();
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    loadSettings();
}

void beginSysex()
{
    Serial.println("Reset sysex message");
    sysexIdx = 0;
}

void appendSysex(uint8_t value)
{
    Serial.print("Appending sysex value: ");
    Serial.println(value);
    // example message (use as default values for programmed units):
    // F0  7E 67 68 6F 73 74 6E 6F 74 65 00 01  01  0B  07  00  00  00  F7
    // SYX ID g  h  o  s  t  n  o  t  e  DEV-ID CC0 CC1 CC2 CH0 CH1 CH2 SYX
    if (sysexIdx >= 31)
        return;

    sysexData[sysexIdx] = value;
    sysexIdx++;
}

bool processSysex()
{
    Serial.println("Processing sysex...");
    int dataLen = sysexIdx;
    sysexIdx = 0;
    
    if (dataLen != 20)
        return false;

    for (int i = 0; i < 20; i++)
    {
        Serial.print(sysexData[i], 16);
        Serial.print(" ");
    }
    Serial.println("");
    
    bool valid = true;
    valid &= sysexData[0] == 0xF0;
    valid &= sysexData[1] == 0x7E;
    valid &= sysexData[2] == 0x67;
    valid &= sysexData[3] == 0x68;
    valid &= sysexData[4] == 0x6F;
    valid &= sysexData[5] == 0x73;
    valid &= sysexData[6] == 0x74;
    valid &= sysexData[7] == 0x6E;
    valid &= sysexData[8] == 0x6F;
    valid &= sysexData[9] == 0x74;
    valid &= sysexData[10] == 0x65;
    valid &= sysexData[11] == 0x00;
    valid &= sysexData[12] == 0x01;
    valid &= sysexData[19] == 0xF7;
    if (!valid)
        return false;

    midiCC[0] = sysexData[13];
    midiCC[1] = sysexData[14];
    midiCC[2] = sysexData[15];

    midiChannel[0] = sysexData[16];
    midiChannel[1] = sysexData[17];
    midiChannel[2] = sysexData[18];

    storeSettings();
}

void writeCc(int sliderNum)
{
    uint8_t data[3];
    data[0] = midiChannel[sliderNum] | 0xB0;
    data[1] = midiCC[sliderNum];
    data[2] = midiVal[sliderNum];
    usb_midi.write(data, 3);
    Serial1.write(data, 3);
}

int iterations = 0;

void loop() 
{
    iterations++;
    bool warmup = iterations < 100;
    
    int s3 = analogRead(A0);
    int s2 = analogRead(A1);
    int s1 = analogRead(A2);

    s3 = TranslateLog2lin(s3);
    s2 = TranslateLog2lin(s2);
    s1 = TranslateLog2lin(s1);

    slider[0] = slider[0] * 0.9 + s1 * 0.1;
    slider[1] = slider[1] * 0.9 + s2 * 0.1;
    slider[2] = slider[2] * 0.9 + s3 * 0.1;

    midiVal[0] = slider[0]/8;
    midiVal[1] = slider[1]/8;
    midiVal[2] = slider[2]/8;

    if (abs(slider[0] - lastTransmitFloat[0]) >= 3 && lastTransmitInt[0] != midiVal[0])
    {
        if (!warmup)
        {
            Serial.print("Slider 0: "); 
            Serial.println(midiVal[0]);
            writeCc(0);
        }
        lastTransmitFloat[0] = slider[0];
        lastTransmitInt[0] = midiVal[0];
    }

    if (abs(slider[1] - lastTransmitFloat[1]) >= 3 && lastTransmitInt[1] != midiVal[1])
    {
        if (!warmup)
        {
            Serial.print("Slider 1: ");
            Serial.println(midiVal[1]);
            writeCc(1);
        }
        lastTransmitFloat[1] = slider[1];
        lastTransmitInt[1] = midiVal[1];
    }

    if (abs(slider[2] - lastTransmitFloat[2]) >= 3 && lastTransmitInt[2] != midiVal[2])
    {
        if (!warmup)
        {
            Serial.print("Slider 2: ");
            Serial.println(midiVal[2]);
            writeCc(2);
        }
        lastTransmitFloat[2] = slider[2];
        lastTransmitInt[2] = midiVal[2];
    }

    while (usb_midi.available() > 0)
    {
        uint8_t value = usb_midi.read();

        if (value == 0xF0)
        {
            beginSysex();
            appendSysex(value);
        }
        else if (value == 0xF7)
        {
            appendSysex(value);
            processSysex();
        }
        else if (sysexIdx > 0)
        {
            appendSysex(value);
        }
    }

    Serial.flush();
    TinyUSB_Device_FlushCDC();
    TinyUSB_Device_Task();
    delayMicroseconds(100);
}
