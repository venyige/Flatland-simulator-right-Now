#ifndef GUI_CURSES_H
#define GUI_CURSES_H
#include<iostream>
#include<curses.h>
#include<mutex>
#include<memory>
namespace now{
using namespace std;
#define KEY_ESC 27
template<size_t rowC, size_t colC>
class gui_curses
{
public:
    gui_curses( const volatile char&,
                mutex&,
                const shared_ptr<array<array<char, colC>,
                rowC>>&,
                volatile const bool&);
    ~gui_curses();
    void test();
    void disp();
private:
    const size_t _rowC=rowC;
    const size_t  _colC=colC;
    const volatile char& _kbState;
    const volatile bool& _quitter;
    mutex& _sceneMutex;
    const shared_ptr<array<array<char, colC>, rowC>>& _sceneMat;
};
}
#include "../gui_curses.cpp"
#endif // GUI_CURSES_H
