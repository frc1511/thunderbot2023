#include <Basic/RobotChess.h>
#include <Hardware/IOMap.h>
#include <fstream>
#include <fmt/core.h>

RobotChess::RobotChess() {

D:
    std::ifstream is2023("/home/lvuser/2023");
    std::ifstream is2022("/home/lvuser/2022");
    std::ifstream isTest("/home/lvuser/TestBoard");

    if (is2023) {
        #if WHICH_ROBOT != 2023
            fmt::print("Boom! 2023 Build not running on 2023 robot\n");
            exit(-1);
        #endif
    }
    if (is2022) {
        #if WHICH_ROBOT != 2022
            fmt::print("Boom! 2022 Build not running on 2022 robot\n");
            exit(-1);
        #endif
    }
    if (isTest) {
        #if WHICH_ROBOT != TEST_BOARD
            fmt::print("Boom! Test Build not running on test board\n");
            exit(-1);
        #endif
    }
}
