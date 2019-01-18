/*****************APP******************/
/* main.c	*/			system(company): a functional system that performs a task
/* app1.c, app2.c, app3.c */		APPn(task):specific task, system decomposition. CEO assigns a task to manager



/**************+APP API+***************/
/* led.c, button, lcd, eepram, 'communication(wrap)'  */	Modules(warp)(department):do a task in a particular area.



/*************+Components+************/ Dedicated hardward(outside SOC) support the Modules function
/* ili9341.c, STM811.c */		need to LINK to xx_BSP(SOC)	


/*************+Communication+***************/ CMSIS Communication driver
/* i2c.c, spi.c,			need to Link to xx_BSP(SOC)


/****************BSP******************/ Link to LL Driver(SOC)
/* xx_BSP.c */	


/***************+Device+**********/
/* LL Drivers, StartUp, Register Map */			