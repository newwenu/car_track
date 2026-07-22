#include "board.h"
#include "sys.h"
#include "../Src/APP/app.h"

int main(void)
{
    all_init();
    app_init();

    while (1)
    {
        app_update();
    }
}


