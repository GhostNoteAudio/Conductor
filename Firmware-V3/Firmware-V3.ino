#include <Arduino.h> // MIDI Library by Francois Best, lathoub
#include <Adafruit_TinyUSB.h> // Included in the Pi Pico Arduino board setup, select from Tools->USB Stack // Don't install this library separately
#include <MIDI.h>
#include <EEPROM.h>

// Change if using log slider
#define LINEAR_SLIDER 1
#define MAX_SYX_LEN 500
#define BYTES_PER_MAPPING 9
#define MAGIC_CONSTANT 3788608666 // Used to check whether the unit has been programmed or not

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

#define MODE_CC 1
#define MODE_PITCHBEND 2
#define MODE_NRPN 3
#define MODE_RPN 4
#define MODE_14BIT 5

const float inv1023 = 1.0f / 1023.0f;

class Mapping
{
public:
    uint8_t Channel = 0;
    uint8_t SubmitThreshold = 5;

    uint8_t FaderBottomDeadZone = 5;
    uint8_t FaderTopDeadZone = 5;

    uint16_t ParamNumber = 1; // CC, nrpn number or pitchbend deadzone
    uint16_t MinVal = 0;
    uint16_t MaxVal = 127;
    
    uint8_t Mode = MODE_CC;

private:
    float PitchbendMultiplierUpper = 1.0;
    float PitchbendMultiplierLower = 1.0;

public:
    void ProcessUpdate(float filteredValue, bool dryrun)
    {
        if (Mode == MODE_CC)
            SendCCUpdate(filteredValue, dryrun);
        if (Mode == MODE_PITCHBEND)
            SendPitchbendUpdate(filteredValue, dryrun);
        if (Mode == MODE_NRPN || Mode == MODE_RPN || Mode == MODE_14BIT)
            Send14BitUpdate(filteredValue, dryrun);
    }

    void PackData(uint8_t* buffer)
    {
        buffer[0] = Channel;
        buffer[1] = SubmitThreshold;
        
        buffer[2] = (uint8_t)((ParamNumber >> 7) & 0x7F);
        buffer[3] = (ParamNumber) & 0x7F;

        buffer[4] = (uint8_t)((MinVal >> 7) & 0x7F);
        buffer[5] = (uint8_t)((MinVal) & 0x7F);

        buffer[6] = (uint8_t)((MaxVal >> 7) & 0x7F);
        buffer[7] = (uint8_t)((MaxVal) & 0x7F);
        
        buffer[8] = Mode;
    }

    void LoadData(uint8_t* buffer)
    {
        Channel = buffer[0];
        SubmitThreshold = buffer[1];

        ParamNumber = (buffer[2] << 7) | buffer[3];
        MinVal = (buffer[4] << 7) | buffer[5];
        MaxVal = (buffer[6] << 7) | buffer[7];
        Mode = buffer[8];

        int deadspace = ParamNumber;
        PitchbendMultiplierUpper = 8191.0f / (511.0f - deadspace / 2.0f);
        PitchbendMultiplierLower = 8192.0f / (512.0f - deadspace / 2.0f);
    }

    void Print()
    {
        Serial.printf("Channel: %d, SubmitThreshold: %d, Param Number (CC/NRPN/Deadzone): %d, MinVal: %d, MaxVal: %d, Mode: %d\n", 
            Channel, SubmitThreshold, ParamNumber, MinVal, MaxVal, Mode);
    }

private:
    float lastTransmitFloat = 0.0f;
    int lastTransmitCC = 0;
    int lastTransmit14bit = 0;

    void SendCCUpdate(float filteredValue, bool dryrun)
    {
        if (fabsf(filteredValue - lastTransmitFloat) >= SubmitThreshold)
        {
            int midiValue = (int)(filteredValue * inv1023 * (MaxVal - MinVal + 0.999) + MinVal);
            if (lastTransmitCC != midiValue)
            {
                lastTransmitFloat = filteredValue;
                lastTransmitCC = midiValue;
                if (!dryrun)
                {
                    //Serial.printf("Submitting midiValue %d\n", midiValue);
                    WriteCc(midiValue);
                }
            }
        }
    }

    void SendPitchbendUpdate(float filteredValue, bool dryrun)
    {
        bool edgeValue = false;
        // deal with very close to top - pull to max value
        if (1023 - lastTransmitFloat <= SubmitThreshold && 1023 - filteredValue < SubmitThreshold)
        {
            filteredValue = 1023;
            edgeValue = true;
        }

        // deal with very close to zero - pull to min
        if (lastTransmitFloat <= SubmitThreshold && filteredValue < SubmitThreshold)
        {
            filteredValue = 0;
            edgeValue = true;
        }

        if (fabsf(filteredValue - lastTransmitFloat) >= SubmitThreshold || edgeValue)
        {
            int pitchValue = 0x2000;
            int deadspace = ParamNumber;
            int deadspaceUpper = 512 + deadspace / 2;
            int deadspaceLower = 512 - deadspace / 2;
            
            if (filteredValue > deadspaceUpper)
                pitchValue += (filteredValue - deadspaceUpper) * PitchbendMultiplierUpper;
            if (filteredValue < deadspaceLower)
                pitchValue += (filteredValue - deadspaceLower) * PitchbendMultiplierLower;

            if (pitchValue > 0x3FFF) pitchValue = 0x3FFF;
            if (pitchValue < 0) pitchValue = 0;

            if (lastTransmit14bit != pitchValue)
            {
                lastTransmitFloat = filteredValue;
                lastTransmit14bit = pitchValue;
                if (!dryrun)
                {
                    //Serial.printf("Submitting pitch value %d\n", pitchValue);
                    WritePitchbend(pitchValue);
                }
            }
        }
    }

    void Send14BitUpdate(float filteredValue, bool dryrun)
    {
        bool edgeValue = false;
        // deal with very close to top - pull to max value
        if (1023 - lastTransmitFloat <= SubmitThreshold && 1023 - filteredValue < SubmitThreshold)
        {
            filteredValue = 1023;
            edgeValue = true;
        }

        // deal with very close to zero - pull to min
        if (filteredValue < SubmitThreshold)
        {
            filteredValue = 0;
            edgeValue = true;
        }

        if (fabsf(filteredValue - lastTransmitFloat) >= SubmitThreshold || edgeValue)
        {
            int x14bitValue = (int)(filteredValue * inv1023 * (MaxVal - MinVal + 0.999) + MinVal);
            if (lastTransmit14bit != x14bitValue)
            {
                lastTransmitFloat = filteredValue;
                lastTransmit14bit = x14bitValue;
                if (!dryrun)
                {
                    if (Mode == MODE_14BIT)
                        Write14Bit(x14bitValue);
                    else
                        WriteNrpn(x14bitValue);
                }
            }
        }
    }

    void WriteCc(int midiValue, int ccNumber = -1)
    {
        if (ccNumber == -1) ccNumber = ParamNumber;
        uint8_t data[3];
        data[0] = 0xB0 | Channel;
        data[1] = ccNumber;
        data[2] = midiValue;
        usb_midi.write(data, 3);
        delayMicroseconds(500); // Seems like you can overload the buffer and crash the program
    }

    void WritePitchbend(int value)
    {
        uint8_t data[3];
        data[0] = 0xE0 | Channel;
        data[1] = (value) & 0x7F;
        data[2] = (value >> 7) & 0x7F;
        usb_midi.write(data, 3);
        delayMicroseconds(500); // Seems like you can overload the buffer and crash the program
    }

    void Write14Bit(int value14bit)
    {
        int param1 = ParamNumber;
        int param2 = ParamNumber + 32;
        int dataMsb = (value14bit >> 7) & 0x7F;
        int dataLsb = (value14bit) & 0x7F;

        uint8_t data[3];
        data[0] = 0xB0 | Channel;
        data[1] = param1;
        data[2] = dataMsb;
        usb_midi.write(data, 3);

        data[0] = 0xB0 | Channel;
        data[1] = param2;
        data[2] = dataLsb;
        usb_midi.write(data, 3);

        delayMicroseconds(500); // Seems like you can overload the buffer and crash the program
    }

    void WriteNrpn(int nrpnValue)
    {
        int addressMsb = (ParamNumber >> 7) & 0x7F;
        int addressLsb = (ParamNumber) & 0x7F;
        int dataMsb = (nrpnValue >> 7) & 0x7F;
        int dataLsb = (nrpnValue) & 0x7F;
        bool rpn = Mode == MODE_RPN;
        WriteCc(addressMsb, rpn ? 101 : 99);
        WriteCc(addressLsb, rpn ? 100 : 98);
        WriteCc(dataMsb, 6);
        WriteCc(dataLsb, 38);
    }
};

class FaderProcessor
{
    Mapping* mappings;

public:
    FaderProcessor(Mapping* mappings)
    {
        this->mappings = mappings;
    }

    int adcValues[3];
    float filteredValues[3];
    float clippedValues[3];

    void ProcessUpdate(int adcValue, int idx, int page)
    {
        auto mapping = mappings[page * 3 + idx];
        adcValues[idx] = adcValue;
        
        // ---- Filter ----
        float alpha;
        float val = adcValue;
        // larger alpha if new value is far away, so we seek the correct location quicker
        // Slower alpha when nearby to smooth noise
        if (fabsf(val - filteredValues[idx]) > 10)
          alpha = 0.05;
        else
          alpha = 0.005;
        filteredValues[idx] = (1-alpha) * filteredValues[idx] + alpha * val;

        // ---- Clipping ----
        float clipVal = filteredValues[idx] - mapping.FaderBottomDeadZone;
        float range = (1023 - mapping.FaderTopDeadZone) - mapping.FaderBottomDeadZone;
        float fVal = clipVal / range;
        if (fVal < 0) fVal = 0.0f;
        if (fVal > 1) fVal = 1.0f;;
        clippedValues[idx] = fVal * 1023.0f;
    }

    float GetProcessedValue(int idx)
    {
        if (idx < 0 || idx >= 3)
            return -1;
        return clippedValues[idx];
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

        if (pageChanged)
            Serial.printf("New Page: %d\n", page);

        SetPageLed();
        return pageChanged;
    }

private:
    void SetPageLed()
    {
        // Control LED colour
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
            mappings[i].Print();
    }

    bool IsProgrammed()
    {
        uint8_t b0 = (MAGIC_CONSTANT >> 24) & 0xFF;
        uint8_t b1 = (MAGIC_CONSTANT >> 16) & 0xFF;
        uint8_t b2 = (MAGIC_CONSTANT >> 8) & 0xFF;
        uint8_t b3 = (MAGIC_CONSTANT >> 0) & 0xFF;

        bool output = EEPROM[0] == b0 && EEPROM[1] == b1 && EEPROM[2] == b2 && EEPROM[3] == b3;
        Serial.printf("Unit is programmed: %d\n", output ? 1 : 0);
        return output;
    }

    void SetProgrammed(bool commit = false)
    {
        uint8_t b0 = (MAGIC_CONSTANT >> 24) & 0xFF;
        uint8_t b1 = (MAGIC_CONSTANT >> 16) & 0xFF;
        uint8_t b2 = (MAGIC_CONSTANT >> 8) & 0xFF;
        uint8_t b3 = (MAGIC_CONSTANT >> 0) & 0xFF;

        EEPROM[0] = b0;
        EEPROM[1] = b1;
        EEPROM[2] = b2;
        EEPROM[3] = b3;
        Serial.println("Setting IsProgrammed Flag magic constants");
        if (commit)
            EEPROM.commit();
    }

    void LoadDefaultSettings(bool storeSettings = true)
    {
        Serial.println("Loading default settings programmatically...");

        for (int i = 0; i < 9; i++)
        {
            mappings[i].Channel = 0;
            mappings[i].SubmitThreshold = 1;
            mappings[i].ParamNumber = 1 + i;
            mappings[i].MinVal = 0;
            mappings[i].MaxVal = 127;
            mappings[i].Mode = 1 + i % 4;
        }

        mappings[2].Channel = 0;
        mappings[2].SubmitThreshold = 1;
        mappings[2].ParamNumber = 14; // 14+46
        mappings[2].MinVal = 0;
        mappings[2].MaxVal = 16383;
        mappings[2].Mode = MODE_14BIT;

        if (storeSettings)
            StoreSettings();
    }

    void LoadSettingsFromEEPROM()
    {
        uint8_t temp[256];
        for (int i = 0; i < 256; i++)
            temp[i] = EEPROM[4+i];
        
        Serial.println("Loading settings from flash memory...");
        for (int i = 0; i < 9; i++)
            mappings[i].LoadData(&temp[i*BYTES_PER_MAPPING]);
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

        SetProgrammed();

        for (int i = 0; i < 9; i++)
        {
            mappings[i].PackData(&EEPROM[4+i*BYTES_PER_MAPPING]);
        }

        // Only calling this manually because this operation can take a while and we don't want to fill the buffer
        TinyUSB_Device_FlushCDC();
        TinyUSB_Device_Task();

        Serial.println("Commit eeprom");
        EEPROM.commit();

        delay(200);
        Serial.println("Reload stored settings from eeprom");
        LoadSettingsFromEEPROM();
    }

    void ListenForSysexInput()
    {
        //Serial.println("Listen for Sysex input...");
        while (usb_midi.available() > 0)
        {
            Serial.println("Data available");
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

        //Serial.println("Exiting sysex listener function");
    }

    void BeginSysex()
    {
        Serial.println("Reset sysex message");
        sysexCount = 0;
    }

    void AppendSysex(uint8_t value)
    {
        Serial.printf("Appending sysex value: %d\n", value);
        // example message (use as default values for programmed units):
        // F0  7E 67 68 6F 73 74 6E 6F 74 65 00 02  ... F7
        // SYX ID g  h  o  s  t  n  o  t  e  DEV-ID ... SYX
        if (sysexCount >= MAX_SYX_LEN)
            return;

        sysexData[sysexCount] = value;
        sysexCount++;
    }

    bool ValidateSysex(int dataLen)
    {
        Serial.println("Validating Sysex message");

        if (dataLen < 14)
        {
            Serial.println("Received incomplete Sysex Message.");
            return false;
        }

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
        valid &= sysexData[dataLen-1] == 0xF7; // Sysex End
        return valid;
    }

    void ProcessSysex()
    {
        Serial.println("Processing sysex...");
        int dataLen = sysexCount;
        sysexCount = 0;


        // Print the message
        for (int i = 0; i < 21; i++)
        {
            Serial.print(sysexData[i], 16);
            Serial.print(" ");
        }
        Serial.println("");

        bool valid = ValidateSysex(dataLen);
        if (!valid)
        {
            Serial.println("Invalid Sysex message received");
            return;
        }

        Serial.println("Sysex valid, proceeding to load settings from buffer...");
        LoadSettingsFromSysexData();
    }
};

ButtonProcessor button;
Mapping mappings[9];
FaderProcessor faders(mappings);
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

    delay(300);

    TinyUSBDevice.setID(0xFB83, 0x5000); // 0xFB83 is an unused USB manufacturer ID.
    usb_midi.setStringDescriptor("Ghost Note Audio Conductor");
    TinyUSBDevice.setManufacturerDescriptor("Ghost Note Audio");
    TinyUSBDevice.setProductDescriptor("Conductor");
    TinyUSBDevice.setSerialDescriptor("Ghost Note Conductor");

    usb_midi.begin();
    delay(100);
    Serial.begin(115200);
    delay(100);

    // wait until device mounted
    while( !TinyUSBDevice.mounted() ) delay(1);

    // Todo: REMOVE before release
    //while(!Serial) {}

    delay(100);
    EEPROM.begin(512);
    delay(100);

    if (configManager.IsProgrammed())
        configManager.LoadSettingsFromEEPROM();
    else
        configManager.LoadDefaultSettings();

    delay(100);
}

void loop()
{
    iterations++;
    bool changePage = button.ProcessPageButton(!digitalRead(D2)); // button pulls low
    if (changePage) iterations = 0;
    bool dryrun = iterations < 1000;
    int page = button.GetPage();

    faders.ProcessUpdate(analogRead(A2), 0, page);
    faders.ProcessUpdate(analogRead(A1), 1, page);
    faders.ProcessUpdate(analogRead(A0), 2, page);
    
    mappings[page * 3 + 0].ProcessUpdate(faders.GetProcessedValue(0), dryrun);
    mappings[page * 3 + 1].ProcessUpdate(faders.GetProcessedValue(1), dryrun);
    mappings[page * 3 + 2].ProcessUpdate(faders.GetProcessedValue(2), dryrun);
    configManager.ListenForSysexInput();
    delayMicroseconds(100);
}
