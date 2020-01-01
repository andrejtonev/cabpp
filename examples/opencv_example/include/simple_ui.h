/**
 * @brief Header implementing a simple UI.
 *
 * @author Andreja Tonev
 */
#ifndef _CABPP_SIMPLE_UI_H_
#define _CABPP_SIMPLE_UI_H_

#include <iostream>
#include <limits>
#include <string>
#include <thread>

namespace ui {
const std::string kEraseInLine = "\033[0K";  //!< Command to erase the line after current cursor position

/**
 * Clear screen using an escape sequence.
 */
inline void ClearScreen(void) { std::cout << "\033[2J" << std::endl; }

/**
 * Print string on a defined terminal row and return to the last UI line.
 */
inline void PrintOnLine(const std::string& str, uint8_t row) {
  std::cout << "\033[" + std::to_string(row) + ";1H" << str << kEraseInLine
            << "\033[13;1H" << std::endl;
}

/**
 * Handle the UI of the OpenCV example.
 */
void RunUI(std::atomic_bool& use_cab, uint16_t& step_ms) {
  std::string line;
  ClearScreen();
  uint16_t row = 1;
  PrintOnLine("***************************"
              "CABpp OpenCV example"
              "***************************",
              row++);
  PrintOnLine("", row++);
  PrintOnLine("The example showcases the benefit of using CAB for inter-thread "
              "communication.",
              row++);
  PrintOnLine("A black and white 800x1500 frame is generated. "
              "The frame is later used by 2 threads.",
              row++);
  PrintOnLine("One simply displays it while the other processes the frame and "
              "displays the result.",
              row++);
  PrintOnLine("Below are the current measured speeds of 3 active threads.",
              row++);
  PrintOnLine("GEN - frame generating thread; "
              "DRAW - live feed thread; "
              "PROC - frame processing thread.",
              row++);
  PrintOnLine("Enter \'h\' for further help.", row++);
  while (1) {
    PrintOnLine(std::string("CAB ") + (use_cab ? "ON " : "OFF"), row + 4);
    PrintOnLine("-", row + 5);
    std::getline(std::cin, line);
    for (auto& c : line) {
      switch (c) {
        case 'H':
        case 'h':
          PrintOnLine("Enter \'c\' to toggle CAB; "
                      "\'f\'/\'s\' to increase/decrease the maximum speed; "
                      "\'q\' to quit.",
                      row + 6);
          break;
        case 'C':
        case 'c':
          use_cab = !use_cab;
          break;
        case 'F':
        case 'f':
          if (step_ms > 0) step_ms--;
          break;
        case 'S':
        case 's':
          if (step_ms < std::numeric_limits<uint16_t>::max()) step_ms++;
          break;
        case 'Q':
        case 'q':
          return;
        default:
          break;
      }
    }
    std::this_thread::yield();
  }
}

}  // namespace ui
#endif  //_CABPP_SIMPLE_UI_H_
