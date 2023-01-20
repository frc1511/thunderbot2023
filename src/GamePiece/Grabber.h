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

private:

};
