#pragma once

#include <Basic/Mechanism.h>

class Drive;
class GamePiece;

class Controls : public Mechanism {
public:
    Controls(Drive* drive, GamePiece* gamePiece);
    ~Controls();

    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;

    void processInDisabled();
    bool getShouldPersistConfig();

private:
    Drive* drive;
    GamePiece* gamePiece;
};
