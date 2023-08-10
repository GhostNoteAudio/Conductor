#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <EEPROM.h>

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;


class PotUpdate
{
public:
    float Value;
    bool IsNew;

    PotUpdate(float val, bool isNew)
    {
        Value = val;
        IsNew = isNew;
    }
};

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

int midiCC[3] = {0};
int midiChannel[3] = {0};
uint8_t sysexData[32];
int sysexIdx;

int sliderData[9] = 
{
  0,
  27,
  56,
  103,
  166,
  252,
  439,
  730,
  1023
};

float TranslateLog2lin(int x)
{
    if (x >= 1023)
        return 1023.0f;
    
    for (int i=0; i < 8; i++)
    {
        if (x >= sliderData[i] && x < sliderData[i+1])
        {
            float base_val = i * 128;
            float residual = x - sliderData[i];
            float frac = residual / (sliderData[i+1] - sliderData[i]);
            float output = base_val + (int)(frac * 128);
            return output;
        }
    }

    return 0.0f;
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
    TinyUSB_Device_FlushCDC();
    TinyUSB_Device_Task();

    printSettings();

    TinyUSB_Device_FlushCDC();
    TinyUSB_Device_Task();

    EEPROM.write(0, midiCC[0]);
    EEPROM.write(1, midiCC[1]);
    EEPROM.write(2, midiCC[2]);

    EEPROM.write(3, midiChannel[0]);
    EEPROM.write(4, midiChannel[1]);
    EEPROM.write(5, midiChannel[2]);

    EEPROM.commit();

    delay(200);
    loadSettings();
}

void setup()
{
  //TinyUSB_Device_Init(0);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  usb_midi.setStringDescriptor("Ghost Note Conductor");
  usb_midi.begin();
  Serial.begin(115200);

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);

  delay(100);
  EEPROM.begin(512);
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

void processSysex()
{
    Serial.println("Processing sysex...");
    int dataLen = sysexIdx;
    sysexIdx = 0;
    
    if (dataLen != 20)
    {
        return;
    }

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
    {
        Serial.println("Invalid Sysex message received");
        return;
    }

    midiCC[0] = sysexData[13];
    midiCC[1] = sysexData[14];
    midiCC[2] = sysexData[15];

    midiChannel[0] = sysexData[16];
    midiChannel[1] = sysexData[17];
    midiChannel[2] = sysexData[18];

    storeSettings();
}

void writeCc(int sliderNum, int midiValue)
{
    uint8_t data[3];
    data[0] = midiChannel[sliderNum] | 0xB0;
    data[1] = midiCC[sliderNum];
    data[2] = midiValue;
    usb_midi.write(data, 3);
}

int iterations = 0;
int adcMin = 3;
float submitThreshold = 5;
float adcValues[3] = {0};
float filteredValues[3] = {0};
float submittedValues[3] = {0};
int submittedMidi[3] = {0};

void loop() 
{
    iterations++;
    bool warmup = iterations < 100;
    
    int s2 = analogRead(A0) - adcMin;
    int s1 = analogRead(A1) - adcMin;
    int s0 = analogRead(A2) - adcMin;
    adcValues[0] = TranslateLog2lin(s0);
    adcValues[1] = TranslateLog2lin(s1);
    adcValues[2] = TranslateLog2lin(s2);

    for (int i=0; i<3; i++)
    {
        float alpha;
        float val = adcValues[i];
        if (fabsf(val - filteredValues[i]) > 10)
          alpha = 0.05;
        else
          alpha = 0.005;

        filteredValues[i] = (1-alpha) * filteredValues[i] + alpha * val;

        if (fabsf(filteredValues[i] - submittedValues[i]) >= submitThreshold)
        {
            int midiValue = ((int)filteredValues[i]) >> 3;
            if (submittedMidi[i] != midiValue)
            {
                submittedValues[i] = filteredValues[i];
                submittedMidi[i] = midiValue;
                Serial.printf("Submitting %d (%.2f)\n", midiValue, submittedValues[i]);
                writeCc(i, midiValue);
            }
        }
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

    delayMicroseconds(100);
}


