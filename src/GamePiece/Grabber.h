#pragma once

#include <Basic/Mechanism.h>

class Grabber : public Mechanism {
public:
    Grabber();
    ~Grabber();

    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;

    enum class Action {
        IDLE,
        INTAKE,
        OUTTAKE,
    };

    void setAction(Action action);

    enum class Position {
        OPEN, //All the way open - cube intake, placement
        AGAPE, //Semi open - cone intake
        AJAR, //Not open - carrying cone tightly
    };

    void setPosition(Position position);

    bool hasGamePiece();
    void overrideHasGamePiece(bool hasGamePiece);
    void placeGamePiece();
private:

};
