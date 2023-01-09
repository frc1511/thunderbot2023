#pragma once

#include <Basic/Mechanism.h>

class Drive : public Mechanism {
public:
    Drive();
    ~Drive();

    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;

private:
};
