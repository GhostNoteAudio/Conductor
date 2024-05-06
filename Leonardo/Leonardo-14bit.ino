#include <MIDIUSB.h>
#include <EEPROM.h>

class PotUpdate {
public:
    float Value;
    bool IsNew;

    PotUpdate(float val, bool isNew) {
        Value = val;
        IsNew = isNew;
    }
};

int midiCC[3] = {0};
int midiChannel[3] = {0};
int minVal = 5;
uint8_t sysexData[32];
int sysexIdx;

int sliderData[9] = {
    0, 27, 56, 103, 166, 252, 439, 730, 1023
};

#define LINEAR_SLIDER 1

float TranslateLog2lin(float x) {
    if (x <= 0) x = 0;
    if (x >= 1023) x = 1023;
    if (LINEAR_SLIDER) return x;

    for (int i=0; i < 8; i++) {
        if (x >= sliderData[i] && x < sliderData[i+1]) {
            // Scale the input value to fit within the range of 0-16383
            float base_val = i * 2047; // 2047 is (16383 / 8)
            float residual = x - sliderData[i];
            float frac = residual / (sliderData[i+1] - sliderData[i]);
            float output = base_val + frac * 2047; // 2047 is (16383 / 8)
            return output;
        }
    }

    return 0.0f;
}

void printSettings() {
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

    Serial.print("Min Value: ");
    Serial.print(minVal);
    Serial.println("");
}

void loadSettings() {
    Serial.println("Loading settings from flash memory...");
    EEPROM.get(0, midiCC);
    EEPROM.get(sizeof(midiCC), midiChannel);
    EEPROM.get(sizeof(midiCC) + sizeof(midiChannel), minVal);
    printSettings();
}

void storeSettings() {
    Serial.println("Storing settings in flash memory...");
    printSettings();

    EEPROM.put(0, midiCC);
    EEPROM.put(sizeof(midiCC), midiChannel);
    EEPROM.put(sizeof(midiCC) + sizeof(midiChannel), minVal);

    delay(200);
    loadSettings();
}

void setup() {
    Serial.begin(115200);
    delay(100);
    loadSettings();
}

void beginSysex() {
    Serial.println("Reset sysex message");
    sysexIdx = 0;
}

void appendSysex(uint8_t value) {
    Serial.print("Appending sysex value: ");
    Serial.println(value);

    if (sysexIdx >= 31)
        return;

    sysexData[sysexIdx] = value;
    sysexIdx++;
}

void processSysex() {
    Serial.println("Processing sysex...");
    int dataLen = sysexIdx;
    sysexIdx = 0;

    if (dataLen != 21)
        return;

    for (int i = 0; i < 21; i++) {
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
    valid &= sysexData[20] == 0xF7;
    if (!valid) {
        Serial.println("Invalid Sysex message received");
        sysexIdx = 0;
        return;
    }

    midiCC[0] = sysexData[13];
    midiCC[1] = sysexData[14];
    midiCC[2] = sysexData[15];
    midiChannel[0] = sysexData[16];
    midiChannel[1] = sysexData[17];
    midiChannel[2] = sysexData[18];
    minVal = sysexData[19];

    storeSettings();
}

void writeCc(int sliderNum, int midiValue) {
    uint8_t msb = midiValue >> 7;
    uint8_t lsb = midiValue & 0x7F;

    // Construct the MIDI event packets
    midiEventPacket_t eventMSB = {0x0B, 0xB0 | midiChannel[sliderNum], midiCC[sliderNum], msb};
    midiEventPacket_t eventLSB = {0x0B, 0xB0 | midiChannel[sliderNum], midiCC[sliderNum] + 32, lsb};

    // Send MSB
    MidiUSB.sendMIDI(eventMSB);

    // Send LSB
    MidiUSB.sendMIDI(eventLSB);
}

int iterations = 0;
float submitThreshold = 5;
float adcValues[3] = {0};
float filteredValues[3] = {0};
float submittedValues[3] = {0};
int submittedMidi[3] = {0};

void loop() {
    iterations++;
    bool warmup = iterations < 100;

    int s2 = analogRead(A0) - minVal;
    int s1 = analogRead(A1) - minVal;
    int s0 = analogRead(A2) - minVal;
    float scaler = 1023.0f / (1023.0f - minVal);
    adcValues[0] = TranslateLog2lin(s0 * scaler);
    adcValues[1] = TranslateLog2lin(s1 * scaler);
    adcValues[2] = TranslateLog2lin(s2 * scaler);

    for (int i=0; i<3; i++) {
        float alpha;
        float val = adcValues[i];
        if (fabsf(val - filteredValues[i]) > 10)
          alpha = 0.05;
        else
          alpha = 0.005;

        filteredValues[i] = (1-alpha) * filteredValues[i] + alpha * val;

        if (fabsf(filteredValues[i] - submittedValues[i]) >= submitThreshold) {
            int midiValue = ((int)filteredValues[i]) >> 3;
            if (submittedMidi[i] != midiValue) {
                submittedValues[i] = filteredValues[i];
                submittedMidi[i] = midiValue;
                Serial.print("Submitting ");
                Serial.print(midiValue);
                Serial.print(" (");
                Serial.print(submittedValues[i], 2);
                Serial.println(")");
                writeCc(i, midiValue);
            }
        }
    }

    while (MidiUSB.available() > 0) {
        midiEventPacket_t event = MidiUSB.read();
        uint8_t value = event.byte1;

        if (value == 0xF0) {
            beginSysex();
            appendSysex(value);
        } else if (value == 0xF7) {
            appendSysex(value);
            processSysex();
        } else if (sysexIdx > 0) {
            appendSysex(value);
        }
    }

    delayMicroseconds(100);
}
