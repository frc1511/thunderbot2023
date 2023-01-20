#pragma once

#include <Basic/Mechanism.h>

class Lift : public Mechanism {
public:
    Lift();
    ~Lift();

    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;

private:

};
