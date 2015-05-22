SharkDrv - multi group firmware for 105d
-----------------------------------------------

License: Released to the Public Domain.

For use in 105d type drivers, no support for stars.

##### Programming

1. Group (10 short presses)

 To set group you make 10 short presses and after that you will get a few seconds short flashes.
After that you count all long flashes and switch off when you reach your group.

 1. 10, 50, 100
 2. 10, 50, 100, Strobe
 3. Ramping, 10, 50, 100
 4. Ramping, 7, 20, 60, 100
 5. 100, 50, 10
 6. 100, 50, 10, Strobe
 7. 100

2. Mode memory (11 short presses)
 To set memory you make 10 short presses and after that you will get a few seconds short flashes.
The first long flash after that means memory off. All other flashes mean memory on.

3. Full mode timer (12 short presses)

 To set turbo timer you make 11 short presses and after that you will get a few seconds short flashes.
The first long flash after that means no timer. All flashes after that mean one second each. Max is 254.

4. Strobe frequency (13 short presses)

 To set strobe frequency you make 12 short presses and after that you will get a few seconds short flashes.
All long flashes after that mean 5 ms delay each. Max is 255 (1275ms).
So i.e 10 flashes mean 50 ms delay which is 10 hz.
Tactical strobe should be 10-20 hz which mean 5-10 flashes.

5. Ramping output level (14 short presses)

 To set ramping you make 13 short presses and after that you will get a few seconds short flashes.
All long flashes after that mean one step (1-255).

