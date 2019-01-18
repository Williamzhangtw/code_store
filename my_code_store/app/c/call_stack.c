
/*
 * belief:
 *  This code is used to show what process that MCU does when calling a function.
 *  You should follow the following steps to get the result
 *    - set a break-point just right before the call_stack()
 *    - open the disassembly window
 */

int CallStack(int param)
{
  int a[3];
  param = 1;
  (void) param;
  (void) a;

  return 0xff;

}
