#ifndef PINOUT_H
#define PINOUT_H

//Buttons for input(0 and 1 are swapped from pcb labeling)
#define BTN_0 2 //INIT | x
#define BTN_1 1 //RUN | TIME ADJUST
#define BTN_2 3 //ADJUSTMENTS | X ADJUST
#define BTN_3 4 //LASERS | Y ADJUST

//Rotary Encoder
#define INCR_BTN 5
#define INCR_A 15
#define INCR_B 16

//Indicatiors
#define BUZZER 8
#define LED_0 6
#define LED_1 7

//Motors
#define STEP_L 21
#define DIR_L 14
#define DIAG_L 48
#define INDEX_L 47

#define STEP_R 11
#define DIR_R 10
#define DIAG_R 13
#define INDEX_R 12

#define STEP_EN 9
#define SPREAD 45   

//Laser
#define LASER 42

//SD Card
#define SD_CS 41
#define SD_CLK 38
#define SD_MOSI 40
#define SD_MISO 39

#endif
