# Change log for OdorChoice Pico program
// Version 1.0 5/3/24  Outline procs needed and initial trial code, test hardware
// Version 2.0 5/31/24
//     use 2 74HC595 for 16 expansion bits
// Version 3.0 6/5/24
//    Split tasks between both processors
//    Processor 0 run task, Processor 1 communicate with MatLab
// 3.1 Moved to Mouse computer MouseRig3
//   Println does CR/LF terminator so set MatLab to know when to process a line
//   Add '#' char at start of every Debug text so MatLab can display it
// Reward every lick  mode E1 reward A  E2 reward B E0 off
//  Version 4.0 
//  Fix lick count for bursts: ( see V8 update)
// Number of licks set for a reward; must be close enough together MinNumLicks 'L'
// Note there is also a MinLickInterval (5) and MaxLkTm (var) to get into a lick burst
// Version 5.0
// Change flash code if not connected
// Version 6.0
// Swap loop and loop1 so most of com happens in the loop
// Work on locking up after a lick
// LED_BUILTIN codes
// Waiting for serial to connect 10Hz flash
// In CheckID() 25 Hz flash
// Connected waiting for trial 0.5 Hz (1 sec on 1 sec off)
// During trial LED is on, end of trial back to slow flash
// V7 Still working on ISR - shorten them
// Switching them to Proc1 didn't help
// Version 8
//  Doh! ReportData not needed so who cares if it's broken! REMOVE table and report
// Clean out extra debug stuff
// fix GivReward so max drops time is used in reward
// No error buzz for ignored trials
// Do Air for either Odorx is 0 or >10 from ML, change to 0 for Pico Odorx
// add 'L' Minimum number of licks
// add MxLkTm 'X' MaxLick Interval between licks in a burst
// Version 9
// Remove RepCor, MatLab handles it
// Change '0' end of trial to 'z', as I need to filter out numeric type errors
// Version 10 (1.0)
// remember ULN2803 are inverting! Output pulls low (hardware issue)
// GP0 is Left Odor 1 on PCB had to switch Left #s
// thus fix OpenOdorValves and CloseOdorValves
// Fix crossed wires on PCB 1.0
// 9/14/24
// Getting occasional 0 times for licks. Suspect crashing messages from Pico
// converted print("c"); println(val); to printf("c%iu",val); to see if L 0 and R 0 stop happening

9/18/24
// for ML v16
//  Remove bool Rewarding, make all prints atomc printf()
//  Shorten Serial Timeout to 20ms
//  Fixed Fail (Error) trials to proper # of licks
9/22/24
// Added e Error Buzz GUI button and command
9/24
// Fixed NoCountErr problem - Does that also fix occasional -1 on error?
9/27
// Change delay(DropTm) to delayMicroseonds(DropTm*1000) to improve time accuracy
10/14  V33 fixed GiveReward for faster drops when manual as well as lick every
10/20 v34 removed extra return; repace with break; as needed
Ignore v35 it's a mistake to treat AB as error on either side
10/20  v36 to sync with new ML v36
10/23 added individual odor valve opening for testing 
Use v1 to v20 for L1-10, R11-20, 21=Air/Vac v0 is close all
Added 21 AirL 22 AirR 23 Air/Vac
10/28  v39 
Pico Added single valve closing with wN command (N=1-23) 24 or greater closes all as before with v0
ML update 1: fixed popup queries (and app) to stay in front
Fixed TRAINING to not need Levels file to be correct.
Added Stop button to "paused" button click response options
10/30 V40 Fix Manual reward variable
11/15 V41 Fix Air Vac Order to match PCB, CloseAllValves to do L then R
11/20 V42-v43 rush fix various
11/27 v44 add NewTrain Mode
11/30 reset totaling vars at end of FinishSession
    add Corrections for Reward valves CVO(A/B L/R)
    
12/7
Add IsDone break between ChkLeft and ChkRight in CWT

