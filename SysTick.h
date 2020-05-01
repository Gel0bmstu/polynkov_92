void initSysTick() {
    SysTick->LOAD = 0xFFFFFF;
    SysTick->CTRL = 1 << SysTick_CTRL_CLKSOURCE_Pos |
                    1 << SysTick_CTRL_ENABLE_Pos;
}

int SysTickGetElapsed() {
    int buf = 0xFFFFFF - SysTick->VAL;
    SysTick->VAL = 0; 
    return buf;
    
}