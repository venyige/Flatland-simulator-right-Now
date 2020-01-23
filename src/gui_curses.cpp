#include "gui_curses.h"
using namespace std;
gui_curses::gui_curses( const volatile char& kbState,
                        mutex& sceneMutex,
                        const shared_ptr<array<array<char, 80>, 25>>& sceneMat,
                        const size_t& rowC,
                        const size_t& colC):
    _rowC(rowC),_colC(colC),
    _kbState(kbState),
    _sceneMutex(sceneMutex),
    _sceneMat(sceneMat)
{
    initscr();
    keypad(stdscr, true);
    noecho();///< don't print out the "wasd"
    curs_set(0);///< remove the cursor
    nodelay(stdscr, TRUE);
}
gui_curses::~gui_curses()
{
    endwin();
}
void gui_curses::disp()
{
    int curChar;
    while ((curChar = getch()) != KEY_ESC)
    {
//        clear();
        while(_sceneMutex.try_lock());
        for (size_t iii=0; iii<_rowC; iii++) {
            for (size_t jjj=0; jjj<_colC; jjj++) {
                mvaddch(iii,jjj,(*_sceneMat)[iii][jjj]);
            }
        }
        _sceneMutex.unlock();
//        refresh();
        napms(100);
    }


}
void gui_curses::test()
{
    int curChar;
    while ((curChar = getch()) != KEY_ESC)
    {
        clear();
        while(_kbState){
            if(_kbState&1)
                printw("w");
            if(_kbState&2)
                printw("a");
            if(_kbState&4)
                printw("s");
            if(_kbState&8)
                printw("d");
            refresh();
            napms(10);
        }
        refresh();
        napms(10);
    }
}
