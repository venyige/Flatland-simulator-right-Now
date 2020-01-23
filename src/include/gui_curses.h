#ifndef GUI_CURSES_H
#define GUI_CURSES_H
#include<iostream>
#include<curses.h>
#include<mutex>
#include<memory>
using namespace std;
#define KEY_ESC 27

class gui_curses
{
public:
    gui_curses( const volatile char& kbState,
                mutex& sceneMutex,
                const shared_ptr<array<array<char, 80>, 25>>& sceneMat,
                const size_t& rowC,
                const size_t& colC);
    ~gui_curses();
    void test();
    void disp();
private:
    const size_t& _rowC, _colC;
    const volatile char& _kbState;
    mutex& _sceneMutex;
    const shared_ptr<array<array<char, 80>, 25>>& _sceneMat;
};

#endif // GUI_CURSES_H
