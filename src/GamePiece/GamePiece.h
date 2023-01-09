#pragma once

#include <Basic/Mechanism.h>

class GamePiece : public Mechanism {
public:
    GamePiece();
    ~GamePiece();

    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;

private:

};
