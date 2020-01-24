#ifndef KEYBOARD_H
#define KEYBOARD_H

#include "keyboard.h"
#include<iostream>
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include<thread>
#include<mutex>
namespace now{
using namespace std;

class keyboard
{
public:
    keyboard(volatile const bool&, mutex&);
    ~keyboard();
    void state(volatile char&);
    thread stateThread(volatile char& statChar){return thread([&statChar, this]{this->state(statChar);});}
private:
    mutex& _kbdMutex;
    volatile const bool& _quitter;
    char _state;
    char keys_return[32];
    Display* dpy;
    bool prA, prS, prD, prW;
    KeyCode kca, kcA, kcw, kcW, kcs, kcS, kcd, kcD;
};
}
#endif // KEYBOARD_H
