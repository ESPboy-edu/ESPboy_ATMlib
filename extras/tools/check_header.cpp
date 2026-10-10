// chk.cpp
#include <stdio.h>
#define PROGMEM
#include "quest.h"
int main(){ fwrite(questTheme, 1, sizeof(questTheme), stdout); return 0; }
