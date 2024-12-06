#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // Included in the Pi Pico Arduino board setup, select from Tools->USB Stack // Don't install this library separately
#include <EEPROM.h>

/*
IMPORTANT

The Serial interface can interfere with the Midi interface and cause deadlocks. We want to completely disable the serial interface
for the production build of the firmware.


Patches and changes in the Adafruit_TinyUSB_Arduino Library
Path: \AppData\Local\Arduino15\packages\rp2040\hardware\rp2040\4.3.1\libraries\Adafruit_TinyUSB_Arduino\
Modify the following:

src\arduino\Adafruit_USBD_Device.cpp
  Comment out the call to .begin - which causes the device to always create a serial interface
  //SerialTinyUSB.begin(115200);

src\arduino\ports\rp2040\tusb_config_rp2040.h
  Change port numbers and buffers as follows:

        #ifndef CFG_TUD_CDC
        #define CFG_TUD_CDC 1
        #endif
        #ifndef CFG_TUD_MSC
        #define CFG_TUD_MSC 0
        #endif
        #ifndef CFG_TUD_HID
        #define CFG_TUD_HID 0
        #endif
        #ifndef CFG_TUD_MIDI
        #define CFG_TUD_MIDI 1
        #endif
        #ifndef CFG_TUD_VENDOR
        #define CFG_TUD_VENDOR 0
        #endif
        #ifndef CFG_TUD_VIDEO
        #define CFG_TUD_VIDEO 0 // number of video control interfaces
        #endif
        #ifndef CFG_TUD_VIDEO_STREAMING
        #define CFG_TUD_VIDEO_STREAMING 0 // number of video streaming interfaces
        #endif

        // video streaming endpoint buffer size
        #define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE 256

        // CDC FIFO size of TX and RX
        #define CFG_TUD_CDC_RX_BUFSIZE 256
        #define CFG_TUD_CDC_TX_BUFSIZE 256

        // MSC Buffer size of Device Mass storage
        #define CFG_TUD_MSC_EP_BUFSIZE 512

        // HID buffer size Should be sufficient to hold ID (if any) + Data
        #define CFG_TUD_HID_EP_BUFSIZE 64

        // MIDI FIFO size of TX and RX
        #define CFG_TUD_MIDI_RX_BUFSIZE 512
        #define CFG_TUD_MIDI_TX_BUFSIZE 512

        // Vendor FIFO size of TX and RX
        #define CFG_TUD_VENDOR_RX_BUFSIZE 64
        #define CFG_TUD_VENDOR_TX_BUFSIZE 64

src\tusb.c

  // You should apply the patch described here: https://github.com/adafruit/Adafruit_TinyUSB_Arduino/issues/293
  // There is an issue which can cause the controller to lock up after an indeterminate amount of time on *some* systems.
  // Patch (make sure you apply to the correct instance of tusb.h, you may have several):

   // pre-check to help reducing mutex lock
   TU_VERIFY((ep_state->busy == 0) && (ep_state->claimed == 0));
-  (void) osal_mutex_lock(mutex, OSAL_TIMEOUT_WAIT_FOREVER);
+  (void) osal_mutex_lock(mutex, OSAL_TIMEOUT_NORMAL);

*/

// Change if using log slider
#define LINEAR_SLIDER 1
#define MAX_SYX_LEN 500
#define BYTES_PER_MAPPING 11
#define MAGIC_CONSTANT 3788608666 // Used to check whether the unit has been programmed or not

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

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
    uint8_t MovementThreshold = 1;

    uint8_t LowerThreshold = 5;
    uint8_t UpperThreshold = 5;

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
        buffer[1] = MovementThreshold;

        buffer[2] = LowerThreshold;
        buffer[3] = UpperThreshold;
        
        buffer[4] = (uint8_t)((ParamNumber >> 7) & 0x7F);
        buffer[5] = (uint8_t)((ParamNumber) & 0x7F);

        buffer[6] = (uint8_t)((MinVal >> 7) & 0x7F);
        buffer[7] = (uint8_t)((MinVal) & 0x7F);

        buffer[8] = (uint8_t)((MaxVal >> 7) & 0x7F);
        buffer[9] = (uint8_t)((MaxVal) & 0x7F);
        
        buffer[10] = Mode;
    }

    void LoadData(uint8_t* buffer)
    {
        this->Channel = buffer[0];
        this->MovementThreshold = buffer[1];

        this->LowerThreshold = buffer[2];
        this->UpperThreshold = buffer[3];

        this->ParamNumber = (buffer[4] << 7) | buffer[5];
        this->MinVal = (buffer[6] << 7) | buffer[7];
        this->MaxVal = (buffer[8] << 7) | buffer[9];
        this->Mode = buffer[10];

        int deadspace = ParamNumber; // for pitchbend only
        this->PitchbendMultiplierUpper = 8191.0f / (511.0f - deadspace / 2.0f);
        this->PitchbendMultiplierLower = 8192.0f / (512.0f - deadspace / 2.0f);
    }

private:
    float lastTransmitFloat = 0.0f;
    int lastTransmitCC = 0;
    int lastTransmit14bit = 0;

    void SendCCUpdate(float filteredValue, bool dryrun)
    {
        if (fabsf(filteredValue - lastTransmitFloat) >= MovementThreshold)
        {
            int midiValue = (int)(filteredValue * inv1023 * (MaxVal - MinVal + 0.999) + MinVal);
            if (lastTransmitCC != midiValue)
            {
                lastTransmitFloat = filteredValue;
                lastTransmitCC = midiValue;
                if (!dryrun)
                {
                    WriteCc(midiValue);
                }
            }
        }
    }

    void SendPitchbendUpdate(float filteredValue, bool dryrun)
    {
        bool edgeValue = false;
        // deal with very close to top - pull to max value
        if (filteredValue > 1023 - MovementThreshold)
        {
            filteredValue = 1023;
            edgeValue = true;
        }

        // deal with very close to zero - pull to min
        if (filteredValue < MovementThreshold)
        {
            filteredValue = 0;
            edgeValue = true;
        }

        if (fabsf(filteredValue - lastTransmitFloat) >= MovementThreshold || edgeValue)
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
                    WritePitchbend(pitchValue);
                }
            }
        }
    }

    void Send14BitUpdate(float filteredValue, bool dryrun)
    {
        bool edgeValue = false;
        // deal with very close to top - pull to max value
        if (filteredValue > 1023 - MovementThreshold)
        {
            filteredValue = 1023;
            edgeValue = true;
        }

        // deal with very close to zero - pull to min
        if (filteredValue < MovementThreshold)
        {
            filteredValue = 0;
            edgeValue = true;
        }

        if (fabsf(filteredValue - lastTransmitFloat) >= MovementThreshold || edgeValue)
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
        TinyUSB_Device_Task();
        delayMicroseconds(500); // Seems like you can overload the buffer and crash the program
    }

    void WritePitchbend(int value)
    {
        uint8_t data[3];
        data[0] = 0xE0 | Channel;
        data[1] = (value) & 0x7F;
        data[2] = (value >> 7) & 0x7F;
        usb_midi.write(data, 3);
        TinyUSB_Device_Task();
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
        TinyUSB_Device_Task();
        delayMicroseconds(500); // Seems like you can overload the buffer and crash the program

        data[0] = 0xB0 | Channel;
        data[1] = param2;
        data[2] = dataLsb;
        usb_midi.write(data, 3);
        TinyUSB_Device_Task();
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
        float clipVal = filteredValues[idx] - mapping.LowerThreshold;
        float range = (1023 - mapping.UpperThreshold) - mapping.LowerThreshold;
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

    bool IsProgrammed()
    {
        uint8_t b0 = (MAGIC_CONSTANT >> 24) & 0xFF;
        uint8_t b1 = (MAGIC_CONSTANT >> 16) & 0xFF;
        uint8_t b2 = (MAGIC_CONSTANT >> 8) & 0xFF;
        uint8_t b3 = (MAGIC_CONSTANT >> 0) & 0xFF;

        bool output = EEPROM[0] == b0 && EEPROM[1] == b1 && EEPROM[2] == b2 && EEPROM[3] == b3;
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
        if (commit)
            EEPROM.commit();
    }

    void LoadDefaultSettings(bool storeSettings = true)
    {
        for (int i = 0; i < 9; i++)
        {
            int pn = 0;
            if (i == 0) pn = 1;
            else if (i == 1) pn = 11;
            else if (i == 2) pn = 7;
            else if (i == 3) pn = 17;
            else if (i == 4) pn = 21;
            else if (i == 5) pn = 74;
            else if (i == 6) pn = 2;
            else if (i == 7) pn = 3;
            else if (i == 8) pn = 4;

            mappings[i].Channel = 0;
            mappings[i].LowerThreshold = 5;
            mappings[i].UpperThreshold = 5;
            mappings[i].MovementThreshold = 1;
            mappings[i].ParamNumber = pn;
            mappings[i].MinVal = 0;
            mappings[i].MaxVal = 127;
            mappings[i].Mode = MODE_CC;
        }

        if (storeSettings)
            StoreSettings();
    }

    void LoadSettingsFromEEPROM()
    {
        uint8_t temp[256];
        for (int i = 0; i < 256; i++)
            temp[i] = EEPROM[4+i];
        
        for (int i = 0; i < 9; i++)
            mappings[i].LoadData(&temp[i*BYTES_PER_MAPPING]);
    }

    void LoadSettingsFromSysexData()
    {
        for (int i = 0; i < 9; i++)
            mappings[i].LoadData(&sysexData[13 + i*BYTES_PER_MAPPING]);
    }

    void StoreSettings()
    {
        // Only calling this manually because this operation can take a while and we don't want to fill the buffer
        TinyUSB_Device_Task();
        SetProgrammed();
        TinyUSB_Device_Task();
        delay(10);
        
        for (int i = 0; i < 9; i++)
        {
            mappings[i].PackData(&EEPROM[4+i*BYTES_PER_MAPPING]);
        }

        // Only calling this manually because this operation can take a while and we don't want to fill the buffer
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
        sysexCount = 0;
    }

    void AppendSysex(uint8_t value)
    {
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
        if (dataLen < 14)
        {
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

    bool IsRequestForDump(int dataLen)
    {
        // Magic request key = 0x7F 0x01 0x7F 0x02
        if (dataLen != 18) return false;
        if (sysexData[13] != 0x7F) return false;
        if (sysexData[14] != 0x01) return false;
        if (sysexData[15] != 0x7F) return false;
        if (sysexData[16] != 0x02) return false;
        return true;
    }

    void DumpCurrentSettings()
    {
        TinyUSB_Device_Task();

        uint8_t startByte[1];
        uint8_t stopByte[1];
        startByte[0] = 0xF0;
        stopByte[0] = 0xF7;
        int i = 0;

        usb_midi.write(startByte, 1);
        TinyUSB_Device_Task();
        delay(1);

        for (int k = 0; k < 9 * BYTES_PER_MAPPING; k+=BYTES_PER_MAPPING)
        {
            usb_midi.write(&EEPROM[4+k], BYTES_PER_MAPPING);
            TinyUSB_Device_Task();
            delay(6);
        }
        
        usb_midi.write(stopByte, 1);
        TinyUSB_Device_Task();
        delay(50);
    }

    void ProcessSysex()
    {
        int dataLen = sysexCount;
        sysexCount = 0;
        TinyUSB_Device_Task();

        bool valid = ValidateSysex(dataLen);
        if (!valid)
            return;

        TinyUSB_Device_Task();

        if (IsRequestForDump(dataLen))
        {
            DumpCurrentSettings();
        }
        else
        {
            LoadSettingsFromSysexData();
            StoreSettings();
        }
    }
};

ButtonProcessor button;
Mapping mappings[9];
FaderProcessor faders(mappings);
ConfigManager configManager(mappings);
int iterations;

void setup()
{
    // Initial delay ensures successful startup
    delay(100);
    
    iterations = 0;
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(D2, INPUT);
    pinMode(D6, OUTPUT);
    pinMode(D7, OUTPUT);
    pinMode(D8, OUTPUT);

    // the timing of the USB stack initialisation is critical.
    // Extra delays or operations in this area can cause the stack to not start up correctly.
    TinyUSBDevice.setID(0xFB83, 0x5000); // 0xFB83 is an unused USB manufacturer ID.
    usb_midi.setStringDescriptor("Ghost Note Audio Conductor Mk2");
    TinyUSBDevice.setManufacturerDescriptor("Ghost Note Audio");
    TinyUSBDevice.setProductDescriptor("Conductor Mk2");
    TinyUSBDevice.setSerialDescriptor("Ghost Note Conductor Mk2");
    usb_midi.begin();
    // wait until device mounted
    while( !TinyUSBDevice.mounted() ) delay(1);

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
