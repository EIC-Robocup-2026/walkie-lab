#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

class Buzzer {
public:
    Buzzer(uint8_t pin);
    void playBootMusic();
    void playConnectedSound();
    void playDisconnectedSound();
    void playNote(int note, int duration);
    void stop();

private:
    uint8_t _pin;
    
    // Erika's boot music
    static const int BOOT_MELODY_SIZE = 23;
    static const int bootMelody[BOOT_MELODY_SIZE];
    static const int bootDurations[BOOT_MELODY_SIZE];
    
    // Connected sound (short ascending notes)
    static const int CONNECTED_MELODY_SIZE = 3;
    static const int connectedMelody[CONNECTED_MELODY_SIZE];
    static const int connectedDurations[CONNECTED_MELODY_SIZE];
    
    // Disconnected sound (short descending notes)
    static const int DISCONNECTED_MELODY_SIZE = 3;
    static const int disconnectedMelody[DISCONNECTED_MELODY_SIZE];
    static const int disconnectedDurations[DISCONNECTED_MELODY_SIZE];
};

#endif