#include <Arduino.h> // MIDI Library by Francois Best, lathoub
#include <Adafruit_TinyUSB.h> // Included in the Pi Pico Arduino board setup, select from Tools->USB Stack // Don't install this library separately
#include <MIDI.h>
#include <EEPROM.h>

// Change if using log slider
#define LINEAR_SLIDER 1
#define MAX_SYX_LEN 150
#define EXPECTED_SYX_LEN 95 // 13 + 1 + 9 * 9
#define BYTES_PER_MAPPING 9

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);


class Mapping
{
public:
    uint8_t Channel = 0;
    uint8_t SubmitThreshold = 5;

    uint16_t ParamNumber = 1; // CC or nrpn number
    uint16_t MinVal = 0;
    uint16_t MaxVal = 127;
    
    bool SendPitchbend = false;
    bool SendNrpn = false;
    bool SendCC = true;

    void ProcessUpdate(float filteredValue, bool dryrun)
    {
        if (SendCC)
            SendCCUpdate(filteredValue, dryrun);
    }

    void PackData(uint8_t* buffer)
    {
        buffer[0] = Channel;
        buffer[1] = SubmitThreshold;
        
        buffer[2] = (ParamNumber >> 7) & 0x7F;
        buffer[3] = (ParamNumber) & 0x7F;

        buffer[4] = (MinVal >> 7) & 0x7F;
        buffer[5] = (MinVal) & 0x7F;

        buffer[6] = (MaxVal >> 7) & 0x7F;
        buffer[7] = (MaxVal) & 0x7F;
        
        buffer[8] = SendPitchbend * 4;
        buffer[8] |= SendNrpn * 2;
        buffer[8] |= SendCC * 1;
    }

    void LoadData(uint8_t* buffer)
    {
        Channel = buffer[0];
        SubmitThreshold = buffer[1];

        ParamNumber = (buffer[2] << 7) | buffer[3];
        MinVal = (buffer[4] << 7) | buffer[5];
        MaxVal = (buffer[6] << 7) | buffer[7];

        SendPitchbend = (buffer[8] & 0x04) > 0;
        SendNrpn = (buffer[8] & 0x02) > 0;
        SendCC = (buffer[8] & 0x01) > 0;
    }

    void Print()
    {
        Serial.printf("Channel: %d, SubmitThreshold: %d, Param Number (CC/NRPN): %d, MinVal: %d, MaxVal: %d, Pitchbend: %d, NRPN: %d, CC: %d\n", 
            Channel, SubmitThreshold, ParamNumber, MinVal, MaxVal, SendPitchbend, SendNrpn, SendCC);
    }

private:
    float lastTransmitFloat = 0.0f;
    int lastTransmitCC = 0;
    int lastTransmit14bit = 0;

    void SendCCUpdate(float filteredValue, bool dryrun)
    {
        if (fabsf(filteredValue - lastTransmitFloat) >= SubmitThreshold)
        {
            int midiValue = ((int)roundf(filteredValue)) >> 3;
            if (lastTransmitCC != midiValue)
            {
                lastTransmitFloat = filteredValue;
                lastTransmitCC = midiValue;
                Serial.printf("Submitting %d (%.2f)\n", midiValue, lastTransmitFloat);
                if (!dryrun)
                    WriteCc(midiValue);
            }
        }
    }

    void WriteCc(int midiValue)
    {
        uint8_t data[3];
        data[0] = Channel | 0xB0;
        data[1] = ParamNumber;
        data[2] = midiValue;
        usb_midi.write(data, 3);
    }
};

class FaderProcessor
{
public:
    int FaderMinVal = 5;
    int FaderMaxVal = 1020;

    int adcValues[3];
    float floatValues[3];
    float filteredValues[3];

    void ProcessUpdate(int adcValue, int idx)
    {
        adcValues[idx] = adcValue;
        adcValue = adcValue - FaderMinVal;
        float range = FaderMaxVal - FaderMinVal;
        float fVal = adcValue / range;
        if (fVal < 0) fVal = 0.0f;
        if (fVal > 1) fVal = 1.0f;
        floatValues[idx] = fVal * 1023.0f;

        FilterValue(idx);
    }

    float GetFilteredValue(int idx)
    {
        if (idx < 0 || idx >= 3)
            return -1;
        return filteredValues[idx];
    }

private:
    void FilterValue(int i)
    {
        float alpha;
        float val = floatValues[i];

        // larger alpha if new value is far away, so we seek the correct location quicker
        // Slower alpha when nearby to smooth noise
        if (fabsf(val - filteredValues[i]) > 10)
          alpha = 0.05;
        else
          alpha = 0.005;

        filteredValues[i] = (1-alpha) * filteredValues[i] + alpha * val;
    }
};

class ButtonProcessor
{
    int btnCounter = 0;
    int btnState = 0;
    int page = 0;

public:
    int GetPage() { return page; }

    // returns true if page has changed
    bool ProcessPageButton(int btn)
    {
        bool pageChanged = false;
        if (btn == 1)
            btnCounter++;
        else
            btnCounter--;

        if (btnCounter > 100)
            btnCounter = 100;
        if (btnCounter < 0)
            btnCounter = 0;

        if (btnCounter == 100 && btnState == 0)
        {
            btnState = 1;
            page = (page + 1) % 3;
            pageChanged = true;
        }

        if (btnCounter == 0 && btnState == 1)
        {
            btnState = 0;
        }

        SetPageLed();
        return pageChanged;
    }

private:
    void SetPageLed()
    {
        // Control LED colour
        Serial.printf("Current Page: %d", page);

        if (page == 0)
        {
            digitalWrite(D6, 1);
            digitalWrite(D7, 0);
            digitalWrite(D8, 0);
        }
        else if (page == 1)
        {
            digitalWrite(D6, 0);
            digitalWrite(D7, 1);
            digitalWrite(D8, 0);
        }
        else if (page == 2)
        {
            digitalWrite(D6, 0);
            digitalWrite(D7, 0);
            digitalWrite(D8, 1);
        }
    }
};

class ConfigManager
{
    uint8_t sysexData[MAX_SYX_LEN] = {0};
    int sysexCount = 0;
    Mapping* mappings;

public:
    ConfigManager(Mapping* mappings)
    {
        this->mappings = mappings;
    }

    void PrintSettings()
    {
        for (int i=0; i<9; i++)
            mappings->Print();
    }

    void LoadSettingsFromEEPROM()
    {
        Serial.println("Loading settings from flash memory...");
        for (int i = 0; i < 9; i++)
            mappings[i].LoadData(&EEPROM[i*BYTES_PER_MAPPING]);
        PrintSettings();
    }

    void LoadSettingsFromSysexData()
    {
        Serial.println("Loading settings from Sysex buffer...");
        for (int i = 0; i < 9; i++)
            mappings[i].LoadData(&sysexData[i*BYTES_PER_MAPPING]);
        PrintSettings();
    }

    void StoreSettings()
    {
        Serial.println("Storing settings in flash memory...");
        
        // Only calling this manually because this operation can take a while and we don't want to fill the buffer
        TinyUSB_Device_FlushCDC();
        TinyUSB_Device_Task();

        PrintSettings();

        for (int i = 0; i < 9; i++)
        {
            mappings[i].PackData(&EEPROM[i*BYTES_PER_MAPPING]);
        }

        // Only calling this manually because this operation can take a while and we don't want to fill the buffer
        TinyUSB_Device_FlushCDC();
        TinyUSB_Device_Task();

        EEPROM.commit();

        delay(200);
        LoadSettingsFromEEPROM();
    }

    void ListenForSysexInput()
    {
        while (usb_midi.available() > 0)
        {
            uint8_t value = usb_midi.read();

            if (value == 0xF0)
            {
                BeginSysex();
                AppendSysex(value);
            }
            else if (value == 0xF7)
            {
                AppendSysex(value);
                ProcessSysex();
            }
            else if (sysexCount > 0)
            {
                AppendSysex(value);
            }
        }
    }

    void BeginSysex()
    {
        Serial.println("Reset sysex message");
        sysexCount = 0;
    }

    void AppendSysex(uint8_t value)
    {
        Serial.print("Appending sysex value: ");
        Serial.println(value);
        // example message (use as default values for programmed units):
        // F0  7E 67 68 6F 73 74 6E 6F 74 65 00 02  ... F7
        // SYX ID g  h  o  s  t  n  o  t  e  DEV-ID ... SYX
        if (sysexCount >= MAX_SYX_LEN)
            return;

        sysexData[sysexCount] = value;
        sysexCount++;
    }

    bool ValidateSysex()
    {
        bool valid = true;
        valid &= sysexData[0] == 0xF0; // SYX start
        valid &= sysexData[1] == 0x7E; // dummy Manufacturer Id
        valid &= sysexData[2] == 0x67; // g
        valid &= sysexData[3] == 0x68; // h
        valid &= sysexData[4] == 0x6F; // o
        valid &= sysexData[5] == 0x73; // s
        valid &= sysexData[6] == 0x74; // t
        valid &= sysexData[7] == 0x6E; // n
        valid &= sysexData[8] == 0x6F; // o
        valid &= sysexData[9] == 0x74; // t
        valid &= sysexData[10] == 0x65; // e
        valid &= sysexData[11] == 0x00; // DEV-ID byte0
        valid &= sysexData[12] == 0x02; // DEV-ID byte1 - changed from previous Conductor version
        valid &= sysexData[EXPECTED_SYX_LEN-1] == 0xF7; // Sysex End
        return valid;
    }

    void ProcessSysex()
    {
        Serial.println("Processing sysex...");
        int dataLen = sysexCount;
        sysexCount = 0;

        if (dataLen != EXPECTED_SYX_LEN)
        {
            Serial.println("Received incomplete Sysex Message. Not processing.");
            return;
        }

        // Print the message
        for (int i = 0; i < 21; i++)
        {
            Serial.print(sysexData[i], 16);
            Serial.print(" ");
        }
        Serial.println("");

        bool valid = ValidateSysex();
        if (!valid)
        {
            Serial.println("Invalid Sysex message received");
            return;
        }

        LoadSettingsFromSysexData();
    }
};

ButtonProcessor button;
FaderProcessor faders;
Mapping mappings[9];
ConfigManager configManager(mappings);
int iterations;

void setup()
{
    iterations = 0;
    //TinyUSB_Device_Init(0);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(D2, INPUT);

    pinMode(D6, OUTPUT);
    pinMode(D7, OUTPUT);
    pinMode(D8, OUTPUT);

    TinyUSBDevice.setID(0xFB83, 0x5000); // 0xFB83 is an unused USB manufacturer ID.
    usb_midi.setStringDescriptor("Ghost Note Audio Conductor");
    TinyUSBDevice.setManufacturerDescriptor("Ghost Note Audio");
    TinyUSBDevice.setProductDescriptor("Conductor");
    TinyUSBDevice.setSerialDescriptor("Ghost Note Conductor");

    usb_midi.begin();
    Serial.begin(115200);

    // wait until device mounted
    while( !TinyUSBDevice.mounted() ) delay(1);

    delay(100);
    EEPROM.begin(512);
    configManager.LoadSettingsFromEEPROM();
}

void loop()
{
    iterations++;
    bool changePage = button.ProcessPageButton(digitalRead(D2));
    if (changePage) iterations = 0;
    bool dryrun = iterations < 100;
    
    faders.ProcessUpdate(analogRead(A2), 0);
    faders.ProcessUpdate(analogRead(A1), 1);
    faders.ProcessUpdate(analogRead(A0), 2);

    int page = button.GetPage();
    mappings[page * 3 + 0].ProcessUpdate(faders.GetFilteredValue(0), dryrun);
    mappings[page * 3 + 1].ProcessUpdate(faders.GetFilteredValue(1), dryrun);
    mappings[page * 3 + 2].ProcessUpdate(faders.GetFilteredValue(2), dryrun);

    configManager.ListenForSysexInput();
    delayMicroseconds(100);
}
