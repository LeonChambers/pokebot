#include "helper.h"

SerialComm serialComm;
RobotPose robotPose;
unsigned long prevTime = 0;

void setup() {
    Serial.begin(115200);
}

void loop() {
    unsigned long currentTime = micros();

    if (currentTime - prevTime >= PERIOD_MICROS) {
        robotPose.update(
            serialComm.desiredWV_R*PERIOD, serialComm.desiredWV_R*PERIOD
        );
        serialComm.send(robotPose);
        serialComm.receiveSerialData();
        prevTime = prevTime + PERIOD_MICROS;
    }
}
