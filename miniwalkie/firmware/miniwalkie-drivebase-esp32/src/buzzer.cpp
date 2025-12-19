#include "buzzer.h"
#include "pitches.h"

// Define the static melodies
const int Buzzer::bootMelody[] = {
    NOTE_D4, NOTE_DS4, NOTE_F4, NOTE_F4, NOTE_F4,
    NOTE_AS4, NOTE_AS4, NOTE_D5, NOTE_D5, NOTE_C5,
    NOTE_AS4, REST, REST, REST, REST,
    NOTE_G4, NOTE_AS4, NOTE_C5,
    REST, REST,
    NOTE_D5, NOTE_C5,
    NOTE_AS4,
};

const int Buzzer::bootDurations[] = {
    4, 4, 4, 4, 2,
    4, 2, 4, 2, 4,
    2, 4, 4, 4, 4,
    2, 4, 2,
    2, 2,
    2, 4,
    4,
};

const int Buzzer::connectedMelody[] = {
    NOTE_C5, NOTE_E5, NOTE_G5
};

const int Buzzer::connectedDurations[] = {
    8, 8, 4
};

const int Buzzer::disconnectedMelody[] = {
    NOTE_G5, NOTE_E5, NOTE_C5
};

const int Buzzer::disconnectedDurations[] = {
    8, 8, 4
};

Buzzer::Buzzer(uint8_t pin) : _pin(pin) {
    pinMode(_pin, OUTPUT);
}

void Buzzer::playNote(int note, int duration) {
    int noteDuration = 800 / duration;
    tone(_pin, note, noteDuration);
    
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(_pin);
}

void Buzzer::playBootMusic() {
    for (int i = 0; i < BOOT_MELODY_SIZE; i++) {
        playNote(bootMelody[i], bootDurations[i]);
    }
}

void Buzzer::playConnectedSound() {
    for (int i = 0; i < CONNECTED_MELODY_SIZE; i++) {
        playNote(connectedMelody[i], connectedDurations[i]);
    }
}

void Buzzer::playDisconnectedSound() {
    for (int i = 0; i < DISCONNECTED_MELODY_SIZE; i++) {
        playNote(disconnectedMelody[i], disconnectedDurations[i]);
    }
}

void Buzzer::stop() {
    noTone(_pin);
}