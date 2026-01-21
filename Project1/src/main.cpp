/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kai-s                                                     */
/*    Created:      1/20/2026, 1:03:35 PM                                     */
/*    Description:  EXP project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

brain Brain;

int main() {

    Brain.Screen.printAt( 2, 30, "Hello EXP" );

    while(1) {
       
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }

}



