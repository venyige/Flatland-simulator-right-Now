#include "keyboard.h"
#include <chrono>
#include <X11/Xlib.h>
#include <X11/keysym.h>
namespace now{

void keyboard::state(volatile char& mState)
{
    while(!_quitter){
        _state=0;
        XQueryKeymap( dpy, keys_return );
        prW = (!!(keys_return[kcw>>3]&(1<<(kcw&7))))||(!!(keys_return[kcW>>3]&(1<<(kcW&7))));
        prA = (!!(keys_return[kca>>3]&(1<<(kca&7))))||(!!(keys_return[kcA>>3]&(1<<(kcA&7))));
        prS = (!!(keys_return[kcs>>3]&(1<<(kcs&7))))||(!!(keys_return[kcS>>3]&(1<<(kcS&7))));
        prD = (!!(keys_return[kcd>>3]&(1<<(kcd&7))))||(!!(keys_return[kcD>>3]&(1<<(kcD&7))));
        if(prW)
            _state=1;
        if(prA)
            _state|=2;
        if(prS)
            _state|=4;
        if(prD)
            _state|=8;
        while(_kbdMutex.try_lock());
        mState=_state;
        _kbdMutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(interface_d::tSlot));
    }
}

keyboard::~keyboard()
{
    XCloseDisplay(dpy);
}
keyboard::keyboard(volatile const bool& quitter,
                   mutex& kbdMutex):
    _kbdMutex(kbdMutex),
    _quitter(quitter)
{
    dpy = XOpenDisplay(nullptr);
    kcw = XKeysymToKeycode(dpy, XK_w);
    kcW = XKeysymToKeycode(dpy, XK_W);
    kca = XKeysymToKeycode(dpy, XK_a);
    kcA = XKeysymToKeycode(dpy, XK_A);
    kcs = XKeysymToKeycode(dpy, XK_s);
    kcS = XKeysymToKeycode(dpy, XK_S);
    kcd = XKeysymToKeycode(dpy, XK_d);
    kcD = XKeysymToKeycode(dpy, XK_D);
}
}
