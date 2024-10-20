// Odor Choice Trial  (C) Gavin Perry May 2024 - Oct 2024

// Program to take serial input parameters from a Matlab program,
// Perform the requested trial while collecting data
// Return real time data to the Matlab program, no Report, let MatLab handle that
// Wait for the next trial
// Passing parameters without spaces or linefeeds is fine
// Example: (defines vars with default values, then starts trial )
//   default:     I2500O1P1A0B1D70T2400W2000U70S1G
//   other:         I2100O2P3A3B4D4T2300W2600U65G
//
// Immediate commands
//           a b c d  - Give reward now LA LB RA RB
//           n play note (test)
//           r  (DEBUG only report parameters)
//           s Resync to microscope
//           x STOP!  Pause at end of trial
//           z  Buzzer
// Parameter Codes from Matlab used (details in ReadInputs)
// com handshake i d  (During setup)
// A B C D E F G H I M N O P S T U X W Y Z
// Available letters J K Q R V
//
// Codes to MatLab
// #1 ID needs reply,  #2 ID running needs no reply
// E F L O R S s U V W X x Y Z z  (S start is ITI start)
// x is stopping a trial

// Moved older issue tracking to GitHub (ssh) git@github.com:Gavin-Perry/OdorChoice.git
// Version 28 -- aligning Pico code program numbers with ML code
// V29 aligning with ML
// Temporary difference between left and right for when to allow more licks
// V30 faster RewEvery lick rate, clean up burst counting
// Stable version 
// V31 start 10/12
// V33 fixed GivReward for faster drops when manual as well as lick every

//=================================
// TO DO
// Simulation Test parts of code here https://wokwi.com/projects/405222044520961025
// or make a new one here   https://wokwi.com/projects/new/pi-pico
//
#include "pico/stdlib.h"
#include "pico/multicore.h"

/* Some pico specific functions that may be useful
//    void rp2040.reboot() // Forces a hardware reboot of the Pico. (Use on error? Watchdog TO?)
//    void rp2040.idleOtherCore() // pause the other core until..
//    void rp2040.resumeOtherCore()
//    void rp2040.restartCore1() // Hard resets Core1 from Core 0 and restarts its operation from setup1().
//  If I can't do it the other way restartCore0() I'll need to swap which processor does what
*/
// IF NO HEARTBEAT AT START, PROBABLY NO SERIAL COM CONNECTED!!!
// CLOSE THEN REOPEN THE SERIAL MONITOR IF DEBUGGING

//================================ ATTENTION!!=========================
// loop (0) to do most of USB communication (Core zero is connected to USB hardware)
// loop 1 does the trials and some quick com. (Passed to Core0)
//====================================================================================

// #include <Libraries>  use "MyLib.h" if any headers in the local dir
#include <ShiftRegister74HC595.h>  // Shifter lib
// #include <Arduino.h>               // in the above lib
#define PCBv1  // Work around code for version 1 PCB. Undefine for PCBv2

// Debugging levels set here
#define DEBUG  // if defined, debug code is compiled in -- Comment out for no debug code
// #define DEBUG2 // verbose, mostly for testing handshake to ML
// Define default values for any params that need them for user convenience

#define dfCWT 2500         // default Choice Wait Time (W)
#define dfITI 2000         // (I)
#define dfMinNumLicks 2    // set with param (L) Keep small for debugging
#define dfMxLkTm 500       // Maximum time between licks to stay in burst
#define dfDropTm 25        // msec set by param (U)
#define dfDropDelay 150    // InterDrop Interval  Minimum value is 50 see ReadCmd 'H'
#define dfRewEvSpaceTm 20  // Delay before next freebie lick with RewEvery
#define dfOdorTm 2000      // ms of odor on (T)
#define dfSyncPol 1        // Low to high (S)
#define dfToneTm 200       // Set default tone (Note) duration (Y)
#define dfBuzzTm 1000      // how long for error buzz sound (Z)
#define dfMaxDrops 7       // default maximum # of drops thus sets total reward time
// Constant values that have to change here
//     no variable for live changes unless requested??
#define SynPulseTm 10      // Sync out to microscope duration (fixed but can change)
#define MinLickInterval 5  // minimum ms between licks FIXED
#define MaxLickList 100    // end of Lick lists, 2 lists, one each Left and Right
#define PostSync 2000      // Time to wait after sync out before start of trial
#define EndSync 1000       // wait at end of trial brefore giving the sync (mscope off)??
// Set notes for error and go tones. Mice hear 1kHz to >80kHz
// Use sounds in human hearing range too. Notes set to minimize dissonance for user
#define Buzz 2489  // D7 near resonance of piezo speaker
#define Note 1865  // A#6  mice hear above 1kHz, Want this higher than buzz or lower OK ??

#define ClickTm 25  // ms for extra click to be sure valves close

//----------------- Pin map using GPIO#s not pin#s ----------------------------

//  GP 0-9 for left, expansion sr. 1-10 for right
#define OL1 0   //  Not changed with V2 board
#define OL2 1   //
#define OL3 2   //
#define OL4 3   //
#define OL5 4   //
#define OL6 5   //
#define OL7 6   //
#define OL8 7   //
#define OL9 8   //
#define OL10 9  // Last odor Left - direct pins
// #define OR1     // Shift reg output1 (skipped zero)
#define VAC 10   //
#define AirL 11  // (Odor 0 Left)
#define AirR 12  //  (Odor 0 Right)

// Right valves use '595 expansion chip with ShiftReg lib included
//        Name      GP#       pin#   pico pin#
const int dataPin = 13;   // 595 p14 pico p17
const int clockPin = 14;  // 595 p11 pico p19
const int latchPin = 15;  // 595 p12 pico p20

// create 16 bit shift reg with 2 'HC595nICs in series
ShiftRegister74HC595<2> sr(dataPin, clockPin, latchPin);  // create a shift register pair <2>
// The right valves are mapped into the '595 shifter pins (SR1 to SR10)
// Reserved/Spare: SR0, SR11-SR15

#define SyncIn 16   //  TTL in from Microscope (Uses OptoIso) Gives a timestamp at start of pulse
#define SyncOut 17  //  TTT to Microscope (Opto?)
// 4 reward solenoids
#define RewLeftA 18    // Pin 24
#define RewLeftB 19    // pin 25
#define RewRightA 20   // pin 26
#define RewRightB 21   // pin 27
#define BUZZER_PIN 22  // pin 29
// SMPS Powersave 23 // dedicated pins
// VBUS in sense 24
// LED_BUILTIN 25
// Lick detect inputs ref to AGND pin 33 for lower noise?
#define LickLeft 26   // input ADC0
#define LickRight 27  // input ADC1

// Comm chars from MatLab
#define idChar 'i'
#define ackChar 'd'
// (ID code to matlab #1 )
//----------------------------------------------------------------------------------------
// Variables
// Event timing
int OdorTm = dfOdorTm;  // Set to defaults defined above 'T'
int DropTm = dfDropTm;
int DropDelay = dfDropDelay;
int MinDD = 50;  // fastest possible time to next reward (ms) in RewEvery case
int ITI = dfITI;  // Wait time before trial

unsigned long  // milli() Times
  RunTime,     // Loop (0) time check
  StartTm,
  EndTime,                        // End of current trial
  EndTm,                          // End of CWT
  RewEvSpaceTm = dfRewEvSpaceTm,  // Minimum time between free Rewards
  HBeat;                          // Tick Builtin LED in loop 1
                                  // debugging defaults:
byte OdorL = 1,                   // "O" value Which Odor on the left 1-5 is A
  OdorR = 6,                      // "P" value  6-10 is B
  RewLA = 1,                      // Rew Left A number of drops
  RewLB = 0,                      // Rew Left B
  RewRA = 0,                      // Rew Right A
  RewRB = 1;                      // Rew Right B

int ToneTm = dfToneTm;  // Set default tone (Note) duration
int BuzzTm = dfBuzzTm;  // how long for error buzz sound
int CWT = dfCWT;        // Choice Wait Time
char Cmd = 0;           // Make command global so other routines can check it
// For bools, 1= true, yes, on; 0= false no off
bool isMatLabPresent = false;
bool NoCountErr = false;  // Not counting errors
// Is Error only needed for debugging - Give Error code ASAP
bool IsDone = false;          // Trial is complete (escape CWT)
bool RewEvery = false;         // Reward Every Lick
bool RewEvA = false;           // for EewEvery true: the juice A, false B
bool NewLickLeft = false;      // Unreported lick on the left?
bool NewLickRight = false;     // Unreported lick on the right?
bool IsFirstLick = true;       // No licks yet, first of burst
bool WasLastLickLeft = false;  // true if the last lick was on the left
bool SyncPol = dfSyncPol;      // Sync polarity
bool Stress = false;           // running stress test
// Trial managment flags to communicate between processors
// The goal with so many vars is to allow some overlap
// Probably a waste of time and can be simplified ???
// e.g. reporting data from last trial while ITI can start in the next
//  VERIFY Did I get this right?
bool GoTime = false;        // It's time to start a trial
bool TrialRunning = false;  // true while a trial is running, to the end
// Lick stuff
int MxLkTm = dfMxLkTm;
unsigned long NewLickLeftTm = 0;    // Save time of most recent L lick in ISR, process in CheckNow
unsigned long NewLickRightTm = 0;   // Save time of most recent R lick in ISR, process in CheckNow
unsigned long LastLickLeftTm = 0;   // Use to check Minimum spacing between licks
unsigned long LastLickRightTm = 0;  // Use to check Minimum spacing between licks
int LickCountL = 0;                 // Counting licks for reward
int LickCountR = 0;
int MinNumLicks = dfMinNumLicks;  // Minimum Licks for reward 'L'
int MaxDrops = dfMaxDrops;

bool first = true;  // for debugging setup vs loop

// Declare procs
bool CheckID();
void CheckNow();
void ToggleLED();
bool ReadInput();             // Parse inputs from MatLab, set Cmd value, handle immediate commands
bool ReadCmds(char Cmd);      // Parse and execute Trial commands
void LowerCaseCmd(char Cmd);  // Called in ReadInput and on error in ReadCmds with lc letter
bool ExecuteTrial();          // True if successful false on timeout or other failure
void OpenAirVac();
void CloseAirVac();
void OpenOdorValves(byte Lft, byte Rt);
void CloseOdorValves(byte Lft, byte Rt);
void DoSyncPulse();
void ChkLeft();     // if burst of left licks process response
void ChkRight();    // if burst of right licks process response
void GiveReward(byte RewLoc, int Drops);  //  RewPin, # of drops (RewLA RewLB RewRA or RewRB)
void LickLISR();
void LickRISR();
void SyncInISR();
void ErrorBuzz(int ErrNum);
int isAABB();  // New 10/3 for checking AA (1) or BB (2) choices AB==0
//
void setup() {
  unsigned long runTime = millis();  // Use a local version for timing in setup
  pinMode(LED_BUILTIN, OUTPUT);      // make sure LED_PIN is output
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(BUZZER_PIN, OUTPUT_12MA);  // Output drive can be 2 4 8 or 12 mA
  for (byte i = 0; i <= 22; i++) {   // set all output pins on and LOW
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  sr.setAllLow();  // set all ShiftReg pins LOW

  pinMode(LickLeft, INPUT);  // Licking inputs
  pinMode(LickRight, INPUT);
  pinMode(SyncIn, INPUT_PULLUP);  // Sync input - opto will pull down when it's input is high
  pinMode(SyncOut, OUTPUT);       // Sync output


  Serial.begin(57600);  // Serial monitor (115200 is maximum rate but it doesn't really matter with USB)
  while (!Serial) {     // Wait for the serial port to connect
    delay(100);         // While waiting for Serial port to connect - fast(ish) toggle LED
    ToggleLED();
  }
  Serial.flush();  // OK, we can talk to PC now
#ifdef DEBUG2
  Serial.println("#Setup");
#endif
  // Make sure a connection is established with MatLab Program- Do Handshake
  isMatLabPresent = CheckID();  // first check for ID
#ifdef DEBUG2
  // Extra flash
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
#endif
  //*/ This code runs when CheckID times out
  if (!isMatLabPresent) {         //
    isMatLabPresent = CheckID();  // Retry again each CheckID is 10 sec
    while (!isMatLabPresent) {    // Still missing?
#ifdef DEBUG
      Serial.println("#Missed CheckID ");
#endif
      isMatLabPresent = CheckID();  // Retry for 20 seconds (each CheckID is 5 sec?)
                                    // different flash code for this problem
      if (digitalRead(LED_BUILTIN))
        delay(400);  // dash on, dot off
      else delay(100);
      ToggleLED();  // Toggle LED
      if (millis() > (runTime + 20000)) {
        if (!isMatLabPresent)                    // This took too long
          Serial.println("#Help! COM failure");  // But no one is listening!
        runTime = millis();
      }
    }  // while no Matlab present - semi patient.
  } else
#ifdef DEBUG2
    Serial.println("#MatLab is Present");
#endif
    //*/
    // Check that pins can interrupt and attach them to proc0 (or proc1 in Setup1)
#ifdef DEBUG2
  Serial.println("#Attach Interrupts  Core0");
#endif  // DEBUG2
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);  // Need delay before attaching ISRs?
  if ((digitalPinToInterrupt(LickLeft) < 0) || (digitalPinToInterrupt(LickRight) < 0) || (digitalPinToInterrupt(SyncIn) < 0)) {
    Serial.println("#Warning NOT an interrupt pin for lick or sync");  // Error message to be NEVER seen unless MPU is changed
  } else {
    attachInterrupt(digitalPinToInterrupt(LickLeft), LickLISR, FALLING);  // Count licks
    attachInterrupt(digitalPinToInterrupt(LickRight), LickRISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(SyncIn), SyncInISR, RISING);  // _-_ microscope sync received
  }
  // test buzzer sounds (takes 500ms)
#ifdef DEBUG2
  Serial.println("#Test tones ");
  digitalWrite(LED_BUILTIN, LOW);
#endif
  tone(BUZZER_PIN, Buzz, 100);     // generates a 100ms beep note D5
  delay(200);                      // tone is Non-blocking, so wait
  tone(BUZZER_PIN, Note, 100);     // generates a 100ms  beep note A6
                                   // Need this when MatLab runs it?  (Should always give a value)
  Serial.setTimeout(20);           // Doing Serial.Available so only times out when no val sent
  digitalWrite(LED_BUILTIN, LOW);  // Setup complete
#ifdef DEBUG
  delay(100);
  Serial.print("#Setup done at ");
  Serial.println(millis());
#endif
}  // end setup()

// Start second processor
void setup1() {  // nothing needed to set up loop1??
                 // Init some default variables for re-boot
  ITI = dfITI;
  CWT = dfCWT;
  OdorTm = dfOdorTm;
  DropTm = dfDropTm;
  DropDelay = dfDropDelay;
  MxLkTm = dfMxLkTm;
  LickCountL = 0;  // Counting licks for reward
  LickCountR = 0;
  MinNumLicks = dfMinNumLicks;  // Minimum Licks for reward
}

//*****************************************
// Loop for first processor,  Watch for commands and set params
// does the heartbeat when trial is NOT running
//    handle Serial IO with Matlab, that's a USB function)
void loop() {                       // Need this loop fast as it process completion routines for licks
  CheckNow();                       // Look for commands and interrupts (licks) completed
                                    // CheckNow calls ReadInput which will (via ReadCmds) set GoTime
                                    // Check the side effects (from other processor) as to what to do next here
                                    // if it's time to do a trial.  loop() does the trials
                                    // Heartbeat between trials (during trial should be steady on)
  if ((millis() - HBeat) > 1000) {  //HBeat rate one sec on, one off
    if (!GoTime && !TrialRunning) {
      HBeat = millis();
      ToggleLED();  // Toggle it
    }
  }
  if (first) {  // tell proc1 we've started, done one pass
    first = false;
  }
}  // end loop

//==================Loop1 stops heartbeat, runs the trials ======================
void loop1() {
  while (first) {  // Waiting for Proc0
    delay(10);
  }
  //main code to run the trials
  if (GoTime && !TrialRunning) {      // Wait for it if there is already a trial running
                                      // Probably towards the end so catch it in a future loop
    digitalWrite(LED_BUILTIN, HIGH);  // LED on during a trial
    ExecuteTrial();                   // Can Pay attention to Error trials??? (returns false)
    digitalWrite(LED_BUILTIN, LOW);   // Trial finished, LED off
  }
  // Nothing to do except run the trials
  // Do some more error checking??? of what?
}  // end loop1

//======================================================
void ToggleLED() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Toggle LED
}

// ---------------------- for setup() and loop()  ----------------
// Wait for a char from ML, read it, if it's idChar reply #1, wait for ackChar
// Keep checking for 10 seconds before complaining.
// We know there is a serial connection so this shouldn't be needed more than once
bool CheckID() {
  char inByte = ' ';  // empty not allowed, use space (or zero / null)
  unsigned long timeout;
  // in case lost sync or matLab restarted while Pico running
  // Should I handle other async cases?
  if (isMatLabPresent) {
    return true;  // No need to check again
  }
  timeout = millis();
  do {
    ToggleLED();                   // Toggle LED
    delay(40);                     // Fast LED flicker while waiting
    if (Serial.available() > 0) {  // keep reading input
      inByte = Serial.read();
      if (inByte == idChar) {
        Serial.println("#1");  // Announce to Matlab we are here
      }
    }
    /*/  TOO CONFUSING    
    if (((millis() - timeout) > 10000) && !(inByte == idChar)) {  // This took too long
      Serial.println("#Help: ML late");
      return false;
    }
//*/
  } while (inByte != ackChar);  // Waiting for the 'd' from ML
#ifdef DEBUG
  Serial.println("#Pico Connected");  // tell of success
#endif
  Serial.flush();  // getrid of any extra chars in buffer
  return true;
}  // end CheckID

//----------------------- Running Trial routines for Loop1() ---------------------------------

bool ExecuteTrial() {  // MatLab RunTrial
  IsFirstLick = true;  // No licks yet
  GoTime = false;      // Started
  TrialRunning = true;
#ifdef DEBUG
  Serial.println("#GO");
#endif
  StartTm = millis();  // Start of this trial. Does anyone care later?
  Serial.printf("%c%u\r\n",'S', StartTm);
  delay(5);  // 5ms to separate S from U
  OpenAirVac();              // Clear last odors
  if ((ITI - PostSync) > 0)  // if ITI < 2 sec, skip it
    delay(ITI - PostSync);   // wait out the ITI time (minus 2 sec)
  DoSyncPulse();
  Serial.printf("Y%u\r\n", millis());  // tell Sync
  delay(PostSync);  // wait for microscope to start
  CloseAirVac();
  // Open odor valves
  OpenOdorValves(OdorL, OdorR);
  // Checking for licks is done by interrupt
  delay(OdorTm);
  // Close odor valves
  CloseOdorValves(OdorL, OdorR);
  // Announce licking time start
  tone(BUZZER_PIN, Note, ToneTm);  // Go time
  delay(ToneTm);                   // Tone is async so have to wait
  // What about licks of Air (Errors?) during or before Odors
  IsDone = false;  // A fresh start at CWT
  Serial.printf("%c%u\r\n", 'W', millis());
#ifdef DEBUG
  Serial.printf("#CWT %u\r\n", millis());
#endif
  // check if licks added to list since odors presented
  // Do any count as errors?
  LickCountL = 0;  // re-start count now
  LickCountR = 0;
  EndTm = millis();  // End of tone, count licks  EndTm needed (for debug only?)
                     // Lick counting is done in CheckNow on the other processor that has the interrupts

  // New V11 - An error is only counted when the lick count threshold is passed on the wrong side.
  while ((millis() - EndTm) < CWT) {
    // Count Licks during CWT with LickCountL and LickCountR updates from CheckNow()
    // Enough Licks to count as a decision? (MinNumLicks)
    // Check if a reward should be given or an error counted
//======================== Left Lick burst? ====================================
    if (LickCountL >= MinNumLicks)  { // enough licks on Left to classify as choice
      ChkLeft();
      // after a good burst always reset the lick counts  
      LickCountL = 0; // Reset licks 
    }
    // ========================== Now check right side ==========================
    if (LickCountR >= MinNumLicks) {   // enough licks on Right to classify as choice
      ChkRight();
      LickCountR = 0; // Reset licks 
    }
    if (IsDone) break;
  }  // end of while CWT

#ifdef DEBUG
  Serial.printf("#End CWT %u  %u\r\n", millis(), IsDone, !IsDone);
#endif
  if (!IsDone) {
  // If timed out it's an ignored trial
  // No reward or error is ignored trial X -> (-1)
      Serial.printf("%c%u\r\n", 'X', millis());  // Report trial ignored, and time
  }
  delay(EndSync);  // shorter time than before trial, but some wait before m'scope off??
  DoSyncPulse();
  Serial.printf("%c%u\r\n", 'Z', millis());  // Sync for microscope off report
  delay(20);                                 // Why not give a bit of time after the pulse (Is microscope going to pulse again?)
  TrialRunning = false;                      // Pico knows Trial is finished
  Serial.printf("%c%u\r\n", 'z', millis());  //  Tell Matlab to save data and go to next trial.
  return true;                               //
}  // True if successful, possible False on timeout or other failure, not implemented that way
   // end ExecuteTrial

//=================================================

int isAABB() {                   // Check if it's an AA, A0, BB, or B0 trial
  if (OdorL < 6 && OdorR < 6) {  // it's AA or A0
    return 1;
  } else if (((OdorL > 5) || (OdorL == 0)) &&  // BB or B0
     ((OdorR > 5) || (OdorR == 0))) {
    return 2;
  } else {
    return 0;  // false, normal AB trial
  }
}

void ChkLeft() {
  if (OdorL == 0) {      // wrong choice Air on Left
    if (NoCountErr) {    // Keep going for another chance         
#ifdef DEBUG
      Serial.printf("#NCErr L0 %u\r\n", millis());
#endif
    } else {  // An error, that counts as a fail
      IsDone = true;
      ErrorBuzz(1);  // Error Left on Air
    }
  } else {  // Not 0 odor; Good choice? check for larger on AA and BB offers
    switch (isAABB()) {                    // What trial type
      case 0:                              //  Normal AB choice is OK
        if (RewLA > 0)                     // Give A if that's the one
          GiveReward(RewLeftA, RewLA);     //
        else GiveReward(RewLeftB, RewLB);  // OK give B
        IsDone = true;
        break;
      case 1:                           // AA trial, chose L
        if (RewLA > RewRA) {   // L more drops
          GiveReward(RewLeftA, RewLA);  //
          IsDone = true;
        } else {                          // oops
          if (NoCountErr) {           
#ifdef DEBUG
      Serial.printf("#NCErr LA< %u\r\n", millis());
#endif              
          } else {
              IsDone = true;
              ErrorBuzz(2);  // Fail
          }
        }  // error
          break;  
        case 2:                           // BB trial Chose R
        if (RewLB > RewRB) { // More L drops = stronger odor
          GiveReward(RewLeftB, RewLB);  //
          IsDone = true;
        } else {
          if (NoCountErr) {
#ifdef DEBUG
            Serial.printf("#NCErr LB< %u\r\n", millis());
#endif              
          } else {          
            IsDone = true;
            ErrorBuzz(3);  //  BB smaller picked
          }
        }  // error
    }  // End switch
  } //  end Odor !0
}  // Chk Left

void ChkRight() {
  if (OdorR == 0) {               // wrong choice it's Air
    if (NoCountErr) {            // and keep going
#ifdef DEBUG
      Serial.printf("#NCErr R0 %u\r\n", millis());
#endif
    } else {  // An error that counts
        IsDone = true;
        ErrorBuzz(4);  // Error Right on Air
    }
  } else {   // Not Air trial
    switch (isAABB()) {                     // What trial type
      case 0:                               //  Normal AB
        if (RewRA > 0)                      // Give A if that's the one
          GiveReward(RewRightA, RewRA);     //
        else GiveReward(RewRightB, RewRB);  // OK give B
        IsDone = true;
        break;
      case 1:                            // AA trial, chose R
                                          // Which is it if it matters ????
        if (RewRA > RewLA) { // R more
          GiveReward(RewRightA, RewRA);  //
          IsDone = true;
        } else {
          if (NoCountErr) {
#ifdef DEBUG
            Serial.printf("#NCErr RA< %u\r\n", millis());
#endif              
          } else {          
          IsDone = true;
          ErrorBuzz(5);  //
          }
        }                // error
        break;  // from switch
      case 2:  // BBB trial Chose R
        if (RewRB > RewLB) {
          GiveReward(RewRightB, RewRB);  //
          IsDone = true;
        } else {
          if (NoCountErr) {
#ifdef DEBUG
            Serial.printf("#NCErr RB< %u\r\n", millis());
#endif 
          } else {
            IsDone = true;
            ErrorBuzz(6);  // 
          }
        }                // error
    }  // End switch
  }  // else no Air
}  // Check Right
  

void DoSyncPulse() {  // Tell about it
  digitalWrite(SyncOut, SyncPol);
  delay(SynPulseTm);
  digitalWrite(SyncOut, !SyncPol);
}

void OpenAirVac() {
  digitalWrite(AirL, HIGH);
  digitalWrite(AirR, HIGH);
  digitalWrite(VAC, HIGH);
  Serial.printf("%c%u\r\n", 'U', millis());  //  U open; V close
}

void CloseAirVac() {
  // 10/1  added extra clicks at end of close to be sure valves close
  digitalWrite(AirL, LOW);
  digitalWrite(AirR, LOW);
  delay(ClickTm);
  digitalWrite(VAC, LOW);
  digitalWrite(AirL, HIGH);
  digitalWrite(AirR, HIGH);
  delay(ClickTm);
  digitalWrite(AirL, LOW);
  digitalWrite(AirR, LOW);
  digitalWrite(VAC, HIGH);
  delay(ClickTm);
  digitalWrite(VAC, LOW);
  Serial.printf("%c%u\r\n", 'V', millis());  //
}

void OpenOdorValves(byte Lft, byte Rt) {
  if (Rt == 0)
    digitalWrite(AirR, HIGH);
  else {
    // Open Right mapped to '595

    // work around wiring bug in board PCB v1.0
#ifdef PCBv1
    if (Rt < 8)
      sr.set(Rt, HIGH);  // Shifter calls the bits 0-7 we skipped 0 pin
    else
      sr.set(Rt + 1, HIGH);  // Missed QA on the right side of chip2
#else                        // PCB V2
    sr.set(Rt, HIGH);  // PCB v2 QA corrected
#endif
  }
  if (Lft == 0)
    digitalWrite(AirL, HIGH);
  else {
    // Open left valve map 1-10 to GP0-GP9 Version 10
    digitalWrite(Lft - 1, HIGH);
  }
#ifdef DEBUG
  Serial.printf("%c%u\r\n", 'O', millis());  //
#endif
}

void CloseOdorValves(byte Lft, byte Rt) {
  if (Rt == 0) {
    digitalWrite(AirR, LOW);
    delay(ClickTm);
    digitalWrite(AirR, HIGH);
    delay(ClickTm);
    digitalWrite(AirR, LOW);
  } else {
#ifdef PCBv1  // work around wiring bug in boards 1.0
    if (Rt < 8) {
      sr.set(Rt, LOW);  // Shifter calls the bits 0-7 we skipped 0 pin
      delay(ClickTm);
      sr.set(Rt, HIGH);  // Shifter calls the bits 0-7 we skipped 0 pin
      delay(ClickTm);
      sr.set(Rt, LOW);  // Shifter calls the bits 0-7 we skipped 0 pin
    } else {
      sr.set(Rt + 1, LOW);  // Missed QA on the right side of chip2
      delay(ClickTm);
      sr.set(Rt + 1, HIGH);  // Shifter calls the bits 0-7 we skipped 0 pin
      delay(ClickTm);
      sr.set(Rt + 1, LOW);  // Shifter calls the bits 0-7 we skipped 0 pin
    }
#else

    sr.set(Rt, LOW);  // PCB v2 QA corrected
    delay(ClickTm);
    sr.set(Rt, HIGH);  // Shifter calls the bits 0-7 we skipped 0 pin
    delay(ClickTm);
    sr.set(Rt, LOW);  // Shifter calls the bits 0-7 we skipped 0 pin
#endif
  }
  if (Lft == 0) {
    digitalWrite(VAC, LOW);
    delay(ClickTm);
    digitalWrite(VAC, HIGH);
    delay(ClickTm);
    digitalWrite(VAC, LOW);
  } else {
    digitalWrite(Lft - 1, LOW);  // GP0 - 9  Version 10
    delay(ClickTm);
    digitalWrite(Lft - 1, HIGH);  // GP0 - 9  Version 10
    delay(ClickTm);
    digitalWrite(Lft - 1, LOW);  // GP0 - 9  Version 10
  }
  Serial.printf("%c%u\r\n", 'F', millis());  // Odors off
}  // End Close Odor Valves

void ValveTest(int rate) {  // click each valve at rate (0.1 sec units)
#ifdef DEBUG
  Serial.println("#Valve Test");
#endif
  int ratems = rate * 100;         // Convert to msec
  for (byte i = 1; i < 11; i++) {  // each of 10 valves 1 - 10
    OpenOdorValves(i, i);
    delay(ratems);
    CloseOdorValves(i, i);
    delay(ratems / 2);
  }
  tone(BUZZER_PIN, 2000, 500);
  delay(600);
  OpenAirVac();  // All 3 together
  delay(ratems);
  CloseAirVac();
  delay(ratems / 2);
  RewVTest(rate);
}  // end Valve Test

// TTest just for reward valves 10/4 added stress test
// Version 10/1/24 has added a final pulse after the open time
// to be sure the valves close
void RewVTest(int rate) {  // rate is really a period in ms
  if (rate == 1000) {      //Stress test
    Serial.println("#Stress test");
    Stress = true;
    long count = 0;
    while (Stress) {  // Runs until k pressed
      for (int i = 0; i < rate; i++) {
        digitalWrite(RewLeftA, HIGH);
        digitalWrite(RewLeftB, HIGH);
        digitalWrite(RewRightA, HIGH);
        digitalWrite(RewRightB, HIGH);
        delay(ClickTm);
        digitalWrite(RewLeftA, LOW);
        digitalWrite(RewLeftB, LOW);
        digitalWrite(RewRightA, LOW);
        digitalWrite(RewRightB, LOW);
        delay(ClickTm);
        if (Serial.available() > 0) {
          Stress = false;
          Serial.println("#End Stress Test");
          break;
        }
      }  // end for
      count++;
      Serial.printf("#Total: %u,000\r\n", count);
      delay(500);
    }                      // while Stress
  } else if (rate > 30) {  // do all together so it won't take so long
    Serial.println("#Cleaning");
    digitalWrite(RewLeftA, HIGH);
    digitalWrite(RewLeftB, HIGH);
    digitalWrite(RewRightA, HIGH);
    digitalWrite(RewRightB, HIGH);
    delay(rate);
    digitalWrite(RewLeftA, LOW);
    digitalWrite(RewLeftB, LOW);
    digitalWrite(RewRightA, LOW);
    digitalWrite(RewRightB, LOW);
    delay(ClickTm);
    digitalWrite(RewLeftA, HIGH);
    digitalWrite(RewLeftB, HIGH);
    digitalWrite(RewRightA, HIGH);
    digitalWrite(RewRightB, HIGH);
    delay(ClickTm);
    digitalWrite(RewLeftA, LOW);
    digitalWrite(RewLeftB, LOW);
    digitalWrite(RewRightA, LOW);
    digitalWrite(RewRightB, LOW);
    // Plus extra click to be sure they close
  } else {  // One at a time for "normal rewards"
    digitalWrite(RewLeftA, HIGH);
    delay(rate);
    digitalWrite(RewLeftA, LOW);
    delay(rate / 2);
    digitalWrite(RewLeftB, HIGH);
    delay(rate);
    digitalWrite(RewLeftB, LOW);
    delay(rate / 2);
    digitalWrite(RewRightA, HIGH);
    delay(rate);
    digitalWrite(RewRightA, LOW);
    delay(rate / 2);
    digitalWrite(RewRightB, HIGH);
    delay(rate);
    digitalWrite(RewRightB, LOW);
  }
}  // End Rew V Test

void GiveReward(byte RewLoc, int Drops) {  //  RewPin, # of drops
// Coded with delays so this core will be here until the reward perios id complete
// New in v2.8 clear licks from count that were during reward
  // Report which reward and which side with A B C D
  // Tell to ML for Trial List Results Using L and R for licks
  unsigned long RwTm = millis();  // Rew Takes time, so mark time now
                                  // Uses A B C D (ML sends a b c d for manual)
  // RewLoc is pin #
  // Need to use microseconds for better accuracy than single ms
  for (int drp = 0; drp < Drops; drp++) {
#ifdef DEBUG2            // This breaks MatLab
    Serial.print(" .");  // Dots to count at valve open
#endif
    digitalWrite(RewLoc, HIGH);
    delayMicroseconds(DropTm * 1000);
    digitalWrite(RewLoc, LOW);
    delayMicroseconds((DropDelay - DropTm) * 1000);  // Whole loop time is DropDelay
  }
  if ((DropDelay < 1000) && (RewEvery == 0)) {  
  // Probably a cleaning if > 1 sec so no wait also no waiting for reward every
    for (int i = Drops; i < MaxDrops; i++) {
      delay(DropDelay);
#ifdef DEBUG2              // This breaks MatLab
      Serial.print(" x");  // # of waits to Max Drops
#endif
    }
  }
    // v2.8 Reset lick counts to avoid rewarding extra rewards for licking rewards
  LickCountL = 0; // Reset licks 
  LickCountR = 0; // Reset licks 

  // Tell which reward was given
  switch (RewLoc) {  //
    case RewLeftA:
      Serial.printf("A%u\r\n", RwTm);
      break;
    case RewLeftB:
      Serial.printf("B%u\r\n", RwTm);
      break;
    case RewRightA:
      Serial.printf("C%u\r\n", RwTm);
      break;
    case RewRightB:
      Serial.printf("D%u\r\n", RwTm);
      break;
  }  // end switch

}  // end Give Reward

void ErrorBuzz(int ErrNum) {       // actually only 1 kind of error in this program
  tone(BUZZER_PIN, Buzz, BuzzTm);  //
  // No reporting of Error number currently
  // for Buzz test can use 0 to get no report
  if (ErrNum > 0) {                            // Buzz without E code, for z
    Serial.printf("%c%u\r\n", 'E', millis());  // T Logging
#ifdef DEBUG // info to debugger
    Serial.print("#Err ");
    switch (ErrNum) {
    case 1 :
      Serial.print("L0 ");
      break;
    case 2 :
      Serial.print("A L< ");
      break;
    case 3 :
      Serial.print("B L< ");
      break;
    case 4 :
      Serial.print("R0 ");
      break;
    case 5 :
      Serial.print("A R< ");
      break;
    case 6 :
      Serial.print("B R< ");
    }
    Serial.println(millis());
#endif
  }
  delay(BuzzTm); // wait for buzz to finish
}

// ISRs // Falling voltage on pin check if minimum interval has passed to count another lick
// Legit lick, save it for CheckNow to finish up
// Later (v7) check if it counts for the Licks for reward
// V7 only reporting problems on the right for debug!!!!??
void LickLISR() {
  if (NewLickLeft) {  // Should never happen next lick before last one processed (noise?)
                      // Core must process the lick soon, so ignore extra too soon licks
                      // With debounced switch all working OK
    return;           //  // Not finished processing the last lick so this one is too soon
  }
  if ((millis() - LastLickLeftTm) > MinLickInterval) {  // legit lick
                                                        // V7 save it for Completion routine
    NewLickLeftTm = millis();                           // time stamp
    WasLastLickLeft = true;
    NewLickLeft = true;  // Trigger completion in CheckNow()
  }
}

void LickRISR() {
  if (NewLickRight) {  // Should never happen next lick before last one processed
    return;            // Not finished processing the last lick so this one is too soon
  }
  if ((millis() - LastLickRightTm) > MinLickInterval) {  // legit lick
                                                         // V7 save it for Completion routine
    NewLickRightTm = millis();
    WasLastLickLeft = false;  // This one on the right
    NewLickRight = true;      // There is a new lick to be processed
  }
  // else too fast, ignore it.
}

void SyncInISR() {  // Sync from microscope, time stamp it
  Serial.printf("s%lu\r\n", millis());
}

void SetALLlow() {                  //Set all output pins low (when forcing stop)
  for (byte i = 0; i <= 22; i++) {  // set all output pins on and LOW
    digitalWrite(i, LOW);
  }
  sr.setAllLow();  // set all ShiftReg pins LOW
}
//========================================================
// Procedures for LOOP Communicating with MatLab (and lick interrupts)
void CheckNow() {
  // Check for results of interrupts
  // Gives rewards if reward every
  // Check what needs to be done in real time (lower Case commands)
  //  handle immediate status updates to MatLab
  // Keep track of both L&R lick counts, check inter lick interval to find a burst of N licks
  // ExecuteTrial during CWT decides what to do about a burst and resets counts as needed.
  if (NewLickLeft) {                                                   // See if it's legit and put it in the list
                                                                       // Check for lick count for reward, NoCountErr means wrong side is OK
                                                                       // RewEvery rewards every lick
                                                                       // and also not insist on fast bursts, this is for training more rewards is better
// One more question: ??? Does lick on wrong side reset count even if NCE = YES                                                                       
    if (IsFirstLick || (WasLastLickLeft &&                            // on the same side
                         ((NewLickLeftTm - LastLickLeftTm) < MxLkTm)))  //and in time
    {                                                                  // still in the burst OR not counting errors, add 1
      LickCountL++;
    } else {  // Nope it's wrong side or too long since last lick
      LickCountL = 1;  // start over but this one counts as first
    }
    IsFirstLick = false; // a lick that counted so not first next time
    WasLastLickLeft = true;  // This one was left side, not right
    LickCountR = 0;  // start over right licks
// Get set for next lick timing for in burst or not
    LastLickLeftTm = NewLickLeftTm;  // Saved NewLick so can allow next ISR
    // pause the other processor???
    NewLickLeft = false;  // clear it, allow ISR now Here or after reward?

    Serial.printf("L%lu\r\n", LastLickLeftTm);  // report the lick to MatLab
                                                //  when it happened (a while ago!)
    if (RewEvery) {                             // Every lick is a winner!
      if (RewEvA)                               // reward is A
        GiveReward(RewLeftA, RewLA);
      else  // RewB
        GiveReward(RewLeftB, RewLB);
      delay(RewEvSpaceTm);  // Minimum time to next freebie reward
    }
  }                       //End NLLeft processing

  if (NewLickRight) {
    // v7 See if it's legit and put it in the list
    // Check for lick count for reward NoCountErr means wrong side is OK
    // and also not insist on fast bursts, this is for training more rewards is better
    if (IsFirstLick || (!WasLastLickLeft &&                             // on the same side
                         ((NewLickRightTm - LastLickRightTm) < MxLkTm)))  //and in time
    {                                                                    // still in the burst or not counting errors
      LickCountR++;
    } else {  // Nope it's wrong side or too long and errors count
        LickCountR = 1;  // start over but this one counts as first
      }
    IsFirstLick = false; // No longer first in burst
    LickCountL = 0;  // start over for Left
    WasLastLickLeft = false;                     // This one was right side
    LastLickRightTm = NewLickRightTm;            // Saved NewLick so can allow next ISR
    NewLickRight = false;  // clear it, allow ISR now
    Serial.printf("R%lu\r\n", LastLickRightTm);  // report the lick to MatLab
    if (RewEvery) {                              // Every lick is a winner! But don't overlap them,
                                                 // Wait reward time before next lick, i.e. ignoring licks during reward of course
                                                 // But what about last lick? Keep going or have a pause
      if (RewEvA)                                // reward is A
        GiveReward(RewRightA, RewRA);
      else  // RewB
        GiveReward(RewRightB, RewRB);
      // end reward every. Take a break? MinRewSpacing between reward every's
      delay(RewEvSpaceTm);  // Minimum time to next freebie reward
    }
  }                        // end NLRight processing


  // Look at immediate commands like reward now (any LowerCase),
  //  else pass on to ReadCmds for parameter changes (for next trial)
  ReadInput();  // Do we care here if a valid trial command given? 
                // Probably not, I think everything was handled
                // Either something was taken care of now or a trial was triggered

}  // end CheckNow

bool ReadInput() {  // Check for input char from MatLab
                    // returns false for an error but it's probably been handled already
  Cmd = 0;          // Clear the Command variable
  // Handle immediate commands with no value added
  // BUT what if they want to give longer drops need to set D??
  if (Serial.available() > 0) {
    // Play it safe, look for alpha only as a command
    if (isAlpha(Serial.peek())) {  // Something there, if not try again later
                                   // Special case need to pass unread char
                                   // See below
                                   //      if (Serial.peek() == idChar) {  // lost comms, MatLab checking for ID?
                                   //        isMatLabPresent = CheckID();  // CheckID want's to read the char itself
                                   //      }
      // OK, now can Get the character out of the buffer
      Cmd = Serial.read();     // get Command
                               // check if it is a lower case "immediate command"
      if (isLowerCase(Cmd)) {  // Read a lower case command to do now
        LowerCaseCmd(Cmd);
      } else {  // Not lower case so get next trial specs
        if (isUpperCase(Cmd)) {
          if (ReadCmds(Cmd))
            return true;  // successful trial spec
        }

      }  // UpperCase
    }    // is Alpha
    /* else Do any special char (not upper or lower) commands??? (CR or LF??)
      e.g. case '+': // Open valves for cleaning
           case '-'; // Close valves
      //*/
    // else
    Serial.read();  // just eat it?
    return false;
  }  // any char ready?
  // No char
  return false;
}  // end ReadInput

void LowerCaseCmd(char Cmd) {  // Handle lower case (immediate) commands
  int val;
  unsigned long FinTrlTm = millis();
  if (isLowerCase(Cmd)) {
    if (Serial.available())
      val = Serial.parseInt(SKIP_WHITESPACE);  // get (optional) value
    else val = 1;                              // default value for LC commands that need a value
    switch (Cmd) {                             // do as commanded
      case 'a':
#ifdef DEBUG
        Serial.printf("#Manual Left A %u\r\n", val);
#endif
        GiveReward(RewLeftA, val);
        break;
      case 'b':
#ifdef DEBUG
        Serial.printf("#Manual Left B %u\r\n", val);
#endif
        GiveReward(RewLeftB, val);
        break;
      case 'c':
#ifdef DEBUG
        Serial.printf("#Manual Right A %u\r\n", val);
#endif
        GiveReward(RewRightA, val);
        break;
      case 'd':
#ifdef DEBUG
        Serial.printf("#Manual Right B %u\r\n", val);
#endif
        GiveReward(RewRightB, val);
        break;
      case idChar:  // 'i' Just in case out of sync and missed it elsewhere
        if (!isMatLabPresent)
          isMatLabPresent = CheckID;
        else {                           // MatLab wants to resync
          Serial.println("#2");          // Announce to Matlab we are still here
          delay(20);                     // #2 is to prevent the 'd' coming back
          if (Serial.available() > 0) {  // keep reading input
            val = Serial.read();         // There shouldn't be anything but this didn
          }
        }
        break;
      case 'k':  // Stop the stress test
        Stress = false;
        Serial.printf("#End Stress Test %u", val);  // fall thru to note
      case 'n':                                     // play note as defined for go time
        tone(BUZZER_PIN, Note, ToneTm);
        break;
#ifdef DEBUG
      case 'r':  // Tell all param values (I had to get matlab to check for multiple line responses)
        Serial.printf(
          "#params: (I)TI %u, Odor(T)m %u, C(W)Tm %u, (O)dorL %u, OdorR(P) %u, RL(A) %u, RL(B) %u, RRA(C) %u, RRB(D) %u\r\n",
          ITI, OdorTm, CWT, OdorL, OdorR, RewLA, RewLB, RewRA, RewRB);
        if (val >= 2) {  // Tell me more
          Serial.printf(
            "#(M)xDrp %u, DropTm(U) %u, M(X)Lk %u, Rw(E)v %u, (N)oCErr %u, TnTm(Y) %u, B(Z)Tm %u\r\n",
            MaxDrops, DropTm, MxLkTm, (RewEvery + RewEvA), NoCountErr, ToneTm, BuzzTm);
        }
        if (val >= 3) {  // Even more debugging details
          Serial.printf(
            "#Licks: NLLTm %u, NLRTm %u, LLTm %u, LRTm %u, WasLLL %u, LCt %u, RCt %u, Min(L) %u\r\n",
            NewLickLeftTm, NewLickRightTm, LastLickLeftTm, LastLickRightTm, WasLastLickLeft,
            LickCountL, LickCountR, MinNumLicks);
        }
        if (val >= 4) {
          Serial.printf(
            "#Trl  GoTm %u, EndTm %u, TRun %u, TDone %u, DropIntv(H) %u, (S)yncPol %u\r\n",
            GoTime, EndTm, TrialRunning, IsDone, DropDelay, SyncPol);
        }
        break;
#endif
      case 's':  // Sync pulse (to resync if out of sync with microscope)
        digitalWrite(SyncOut, SyncPol);
        delay(SynPulseTm);  // Sync duration
        digitalWrite(SyncOut, !SyncPol);
        break;
      case 't':         // Play a tone where val is frequency if over 200
        if (val < 200)  // play default tone
          tone(BUZZER_PIN, Note, ToneTm);
        else
          tone(BUZZER_PIN, val, ToneTm);  // Play specified frequency
        break;
      case 'v':        // Valve check Open each odor 1 by one then Air and Vac and rewards
        if (val == 1)  // Test all valves
          ValveTest(val);
        else {  // Allow slow or fast testing of Rewards only
          RewVTest(val);
        }
        break;
      case 'x':  // STOP button pressed, stop the trial, but not during reward I hope
                 // need to check and close valves, Tell MatLab to do it
                 // Emergency stop with reboot is problematic as USB connection can be lost due to Windoze way of retermining if the connection is there.
                 // Trial will end itself,so wait, then do ValveCheck
        FinTrlTm = millis();
#ifdef DEBUG
        Serial.printf("#Stopping at %u\r\n", FinTrlTm);
#endif
        if (!TrialRunning) {
          Serial.printf("z%u\r\n", millis());  //  Tell Matlab done even though no trial
        }
        while (TrialRunning) {               //waiting for trial to finish... but not forever
          if ((millis() - FinTrlTm) > 8000)  // is 8 seconds long enough to finish any trial???
            break;                           // call it
        }
        if (TrialRunning) {                   // Timed out, not complete
          Serial.println("#Aborting trial");  // Tell MatLab if we quit mid trial
          Serial.printf("x%u\r\n", millis());
          Serial.printf("z%u\r\n", millis());  //  Tell Matlab done (if needed)
        }                                      // if trial finished already got the 'z'
        break;                                               
      case 'z':                                // Do Buzzer
        ErrorBuzz(0);                          // Use Error# 0 - no E reported
        break;
      default:  // got a different lower case letter
#ifdef DEBUG
        Serial.printf("#Bad Command %c dec:%u \r\n", Cmd, int(Cmd));
#endif
        break;

    }  // end switch  - for a lower case letter
  }    // end if LC
}  // End LowerCaseCmd

// Read commands until we get the Go signal (so might hang if G is missing from MatLab)
bool ReadCmds(char Cmd) {                      // Keep reading rest of command string until G is given or time out
                                               // Command letters used: A B C D E F G H I L M N O P S T U W X Y Z
                                               // Available letters J K Q R V
  int val = 0;                                 // local value to be set
  do {                                         // get all parameters
    if (Cmd == 0) Cmd = Serial.read();         // Get next command
    if (isUpperCase(Cmd)) {                    // need to check after first one
      delay(10);                               // let all the chars of a number arrive into the buffer
      val = Serial.parseInt(SKIP_WHITESPACE);  // get value
      // Validate value??? (for each parameter)
      switch (Cmd) {  // Put value into the variable commanded
        case 'A':
          RewLA = val;
          if (RewEvery == 0)  // unless RewEvery is on
            RewLB = 0;        // can only have A or B not both (Last given is kept)
          break;
        case 'B':
          RewLB = val;
          if (RewEvery == 0)  // unless RewEvery is on
            RewLA = 0;        // can only have A or B not both
          break;
          // Right side
        case 'C':
          RewRA = val;
          if (RewEvery == 0)  // unless RewEvery is on
            RewRB = 0;        // can only have A or B not both
          break;
        case 'D':
          RewRB = val;
          if (RewEvery == 0)  // unless RewEvery is on
            RewRA = 0;        // can only have A or B not both
          break;
        case 'E':  // Reward Every Lick (1=A, 2=B)  off (0)
          if (val == 2) {
            RewEvA = false;  // It's B
            val = 1;         // Back to normal true/false value
          } else if (val == 1)
            RewEvA = true;  // Yes, it's A (1)
          else RewEvA = 0;  // aka false
          // val is now 1 or 0
          RewEvery = val;  // true or false
          break;
        case 'F':  // Minimum time between Freebie Licks (when E>0)
          if (val > 0)
            RewEvSpaceTm = val;
          else RewEvSpaceTm = dfRewEvSpaceTm;
        case 'G':  // Go time, end of variables to change
          GoTime = true;
          break;
        case 'H':        // set DropDelay 
          if (val > MinDD)  // Minimum value
            DropDelay = val;
          else DropDelay = dfDropDelay;
          break;
        case 'I':
          if (val > 0)
            ITI = val;
          else ITI = dfITI;
          break;
        case 'L':  // Minimum number of licks
          if (val > 0) MinNumLicks = val;
          else MinNumLicks = dfMinNumLicks;
          break;
        case 'M':  // Max drops for constant total reward timing
          if (val > 0) MaxDrops = val;
          else MaxDrops = dfMaxDrops;
          break;
        case 'N':  // Not counting errors (1)  off (0)
          NoCountErr = val;
          break;
        case 'O':        // Odor Left: 1-5 is A 6-10 is B  0 is Air, if >10 change to Air
          if (val > 10)  // e.g. Air pin direct, or out of range
            OdorL = 0;
          else
            OdorL = val;
          break;
        case 'P':        // odor right
          if (val > 10)  // e.g. Air pin direct, or out of range
            OdorR = 0;
          else
            OdorR = val;
          break;
          //        case 'R':  // was Repeat until correct, now Handled by MatLab
          //          RepCor = val;
          //          break;
        case 'S':  // Sync polarity 0 = -; 1 = + (Low to High)
          SyncPol = val;
          break;
        case 'T':
          if (val > 0)
            OdorTm = val;
          else OdorTm = dfOdorTm;
          break;
        case 'U':  // Drop time in ms
          if ((val > 0) && (val < (DropDelay - 10)))
            DropTm = val;
          else DropTm = dfDropTm;
          break;
        case 'W':  //Choice Wait Time
          if (val > 0)
            CWT = val;
          else CWT = dfCWT;
          break;
        case 'X':  // maXimum interval between licks
          if (val > 0)
            MxLkTm = val;
          else
            MxLkTm = dfMxLkTm;
          break;
        case 'Y':  // End offer sound duration, time for choice
          if (val > 0)
            ToneTm = val;
          else ToneTm = dfToneTm;
          break;
        case 'Z':  // Buzz time
          if (val > 0)
            BuzzTm = val;
          else BuzzTm = dfBuzzTm;
          break;
        default:
          Serial.printf("#Bad Command: %c %u\r\n", Cmd, int(Cmd));
          //  return false; keep going? Break out and ignore???
          break;
      }  // end switch
    }    // end Is upper case

    else {                  // Bad command
                            // Manage corrupted USB comm
      if (isLowerCase) {    // Hmmm, got a lower case in the middle of a trial command???
                            // This shouldn't happen but let's deal with it
        LowerCaseCmd(Cmd);  // deal with it and leave
        Cmd = 0;            // Clear it
        return false;
      }
      // At this point it's probably a number but check everything printable
      if (isPrintable(Cmd)) {  //  tell me about it, what did I miss?
        Serial.printf("#Input error: %c %u \r\n", Cmd, val);
        Cmd = 0;       // Clear it
        return false;  // not a valid trial get out of here
      }
      // end corrupted data check
    }
    Cmd = 0;        // Clear it
                    // Serial chars available (otherwise waiting for G)
  }                 //do
  while (!GoTime);  // until GoTime
  return true;
}  // end ReadCmds()
