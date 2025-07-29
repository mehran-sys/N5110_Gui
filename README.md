A simple, menu-driven Graphical User Interface (GUI) for the Nokia 5110 LCD screen, implemented on an STM32F103C8T6 (Blue Pill) microcontroller.
This project is developed using STM32CubeIDE with HAL libraries and leverages FreeRTOS for task management.

Overview:
This project provides a lightweight and responsive GUI on the popular and inexpensive Nokia 5110 display. It's designed to be a starting point for more complex embedded projects requiring user interaction. The interface is navigated using push buttons. The use of FreeRTOS allows for modular and non-blocking operation, ensuring that UI responsiveness, sensor reading, and other tasks run smoothly alongside each other.

Features:
The GUI is organized into several distinct menus, each handled by its own FreeRTOS task:
Main Menu: The central navigation hub to access all other features.
Clock: Displays the current time. (Set the time through push buttons)
Timer: A simple stopwatch function with minutes, seconds, and centiseconds. (Stop, start and reset the timer using push buttons)
Backlight Control: Adjust the LCD backlight brightness.
Contrast Control: Adjust the LCD contrast for optimal viewing.
MCU Temperature: Reads and displays the internal temperature of the STM32 microcontroller. (The integarted temperature sensor worked out terrible for me, I put it there just for fun!) 
Help Screen: Provides basic instructions or information about the device.
Reboot: A software-triggered system reset.

Independent Watchdog (IWDG) is implemented to ensure system stability and automatically recover from potential software freezes.

Hardware Requirements:
Microcontroller: STM32F103C8T6 "Blue Pill" development board.
Display: Nokia 5110 (PCD8544) 84x48 LCD.
Input: Push buttons for navigation (Up, Down, Select/Enter).
Wiring: Jumper wires to connect the components.

The pinout is specified in the code, change it as you wish.

Software & Tools:
IDE: STM32CubeIDE
Framework: STM32Cube HAL (Hardware Abstraction Layer)
RTOS: FreeRTOS

Feel free to contribute to this project by forking the repository and submitting a pull request. For major changes, please open an issue first to discuss what you would like to change.
Known Issues & Areas for Improvement:
Font System: The current font system is very basic and needs a rework. Contributions to integrate a more flexible and efficient font library or to clean up the existing implementation would be a great help.
