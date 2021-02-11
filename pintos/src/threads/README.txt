
To add the mega alarm you had to change these files:

- Rubric.alarm
- Make.tests
- tests.c
- alarm-wait.c

Each file had to have changes made that are similar to alarm-multiple, but use 70 instead of 7. You must make
sure that each file edited has a reference to alarm-mega. "alarm-wait.c" has to be edited to create the alarms.

A file named alarm-mega.ck also had to be created. This was the PERL file that calls the alarm. 
