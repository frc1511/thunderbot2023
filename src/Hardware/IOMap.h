#pragma once

/*
  ██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
  ██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
  ██║██║   ██║    ██╔████╔██║███████║██████╔╝
  ██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
  ██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
  ╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     v2023
*/

// #define TEST_BOARD

#ifdef TEST_BOARD
#   include <Hardware/_TestBoardIOMap.h>
#else
#   include <Hardware/_MainIOMap.h>
#endif
