// ---------------------- includes & config ----------------------
#include "vex.h"
using namespace vex;

brain Brain;
inertial BrainInertial = inertial();
motor MotorLeft      = motor(PORT3,  false);
motor MotorRight     = motor(PORT6,  true);
motor MotorDispense  = motor(PORT1,  false);
optical OpticalSensor = optical(PORT4);  
touchled TouchLED = touchled(PORT5);

void initializeRandomSeed()
{
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  int seed = int(xAxis + yAxis + zAxis);
  srand(seed);
}

void vexcodeInit() 
{
  initializeRandomSeed();
}

void configureAllSensors()
{
  BrainInertial.calibrate();
  wait(2, seconds);
  BrainInertial.setHeading(0, degrees);
  BrainInertial.setRotation(0, degrees);
  MotorLeft.setPosition(0, turns);
  MotorRight.setPosition(0, turns);
  MotorDispense.setPosition(0, deg);
  OpticalSensor.setLight(ledState::on);
}

// ---------------------- pid rotation functions
double clamp(double power, double minPower, double maxPower)
{
  double temp = fabs(power);
  if (temp < minPower) temp = minPower; // corrects if the current power is
  if (temp > maxPower) temp = maxPower; // outside of our limits
  return copysign(temp, power);
}

double convertAngle(double angle)
{
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0; // makes a within -180 to 180 
  return angle;
}

bool rotateToHeadingPID(double target)
{
  const double maxPower = 70.0;        // max motor power
  const double minPower = 7.0;         // min motor power
  const double integralLimit = 20.0;   // minimum degrees away from target
                                       // to start calculating integral

  const int loopTime = 15;             // update frequency (in ms)

  double kp = 1.25;
  double ki = 0.02;
  double kd = 0.15;
  int timeout = 2000;
  double tolerance = 1.0;
  // old values: 1.25, 0.0, 0.12, 2000, 2.0

  MotorLeft.setStopping(brake);
  MotorRight.setStopping(brake);
  MotorLeft.spin(forward);
  MotorRight.spin(forward);

  timer t; // initializes timer t
  
  double error = convertAngle(target - BrainInertial.heading(degrees));
  // converts target angle to a lowest angle to target
  // IN DEGREES
  // IN DEGREES

  double prevError  = error;  // for derivative calc
  double integral = 0.0;      // accumulated error over time (integral)

  // continue until within tolerance or timeouts
  while (fabs(error) > tolerance && t.time(msec) < timeout)
  {
    // INTEGRAL: RIEMANN'S SUM OF ERROR * TIME ELAPSED
    if (fabs(error) < integralLimit)
    // only start calculating integral as it approaches the limit
    {
        integral += error * (loopTime/1000.0);
    }
    else
    {
        integral = 0.0; // Reset integral when far from target
    }

    // DERIVATIVE: RATE OF CHANGE OF ERROR
    double derivative = (error - prevError) / (loopTime/1000.0);

    // PID CALCULATION
    // pos u = cw rotation, neg u = ccw rotation
    double u = kp*error + ki*integral + kd*derivative;

    // uses clamp() to make sure u is within min max values for power
    double leftPower  = clamp( u, minPower, maxPower);
    double rightPower = clamp(-u, minPower, maxPower);

    MotorLeft.setVelocity(leftPower,  percent);
    MotorRight.setVelocity(rightPower, percent);

    wait(loopTime, msec); // waits until next loop of while loop, based on
    //                     set value for loopDt

    prevError = error; // stores current error for next derivative calculation
    error = convertAngle(target - BrainInertial.heading(degrees)); 
    // get new error
  }

  MotorLeft.stop();
  MotorRight.stop();

  // returns false if it failed to reach the target within timeout time
  return fabs(error) <= tolerance;
}

// ---------------------- dispense one card
const double DEG_PER_CARD = 240.0;    // max degrees of rotation per card
const int    MAX_MS = 240;            // max time for motor to run
void dispenseOneCard()
{
  double startDispense = MotorDispense.position(deg);
  MotorDispense.setVelocity(90, percent);
  MotorDispense.spin(forward);

  timer t;
  while ((MotorDispense.position(deg) - startDispense) < DEG_PER_CARD
         && t.time(msec) < MAX_MS) 
  {}

  MotorDispense.stop(brake);

  int dispenseTime = t.time(msec);  // how long the motor ran fro

  MotorDispense.setVelocity(90, percent);  // same speed as forward
  MotorDispense.spin(reverse);             // spin backwards

  wait(dispenseTime + 200, msec);  // run backwards for the same time + extra

  MotorDispense.stop(brake);
}

// deal a set number of cards to a specific position, using rotation function 
// + dispensing function()
void dealCardsToPosition(double heading, int numCards) 
{
  rotateToHeadingPID(heading);
  for (int i = 0; i < numCards; i++) 
  {
    dispenseOneCard();
    wait(80, msec);
  }
}

bool allDealt(int seats[],int numplayers, int req) 
{
  for (int i = 0; i < numplayers; i ++) 
  {
    if (seats[i] != req) return false;
  }

  return true;
}


// ---------------------- random shuffling algorithm ----------------------
void shuffleDeal(int numSeats, int totalCardsPerSeat) 
{
  // tracks how many cards each seat has received
  int cardsDealt[numSeats];

  for (int i = 0; i < numSeats; i++) 
  {
    cardsDealt[i] = 0;
  }

  while (!allDealt(cardsDealt,numSeats,totalCardsPerSeat)) 
  {
    int index = rand() % numSeats;

    int pileSize = cardsDealt[index];

    if (pileSize!=totalCardsPerSeat) 
    {
      dealCardsToPosition(360.0/numSeats*index,1);
      cardsDealt[index] ++;
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    for (int n = 0; n < numSeats; n++) 
    {
      Brain.Screen.print("%d ", cardsDealt[n]);
    }
  }
}

// mode constants
const int MODE_DEAL = 0;
const int MODE_SHUFFLE = 1;
const int MODE_SORT = 2;
const int MODE_EXIT = 3;

/*
method runs user interface for selecting which process to run
returns an integer indicating mode (deal, shuffle, or sort)
*/

int selectMode() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  
  wait(1,seconds);

  // mode is set to deal as default
  int i = 0; // stores current mode selected 0 indicates top option

  // runs ui process until return statement
  while (true) 
  {
    Brain.Screen.setCursor(1,1);

    // displays the mode options
    // & *** next to the mode selected
    Brain.Screen.print("DEAL");
    if (i == MODE_DEAL) 
    {
      Brain.Screen.print(" ***");
    }
    Brain.Screen.newLine();
    Brain.Screen.print("SHUFFLE");
    if (i == MODE_SHUFFLE) 
    {
      Brain.Screen.print(" ***");
    }
    Brain.Screen.newLine();
    Brain.Screen.print("SORT");
    if (i == MODE_SORT) 
    {
      Brain.Screen.print(" ***");
    }
    Brain.Screen.newLine();
    Brain.Screen.print("EXIT");
    if (i == MODE_EXIT) 
    {
      Brain.Screen.print(" ***");
    }
    Brain.Screen.newLine();

    // waits until any button is pressed
    while(!Brain.buttonLeft.pressing() 
          && !Brain.buttonRight.pressing() 
          && !Brain.buttonCheck.pressing()) 
    {}

    Brain.Screen.clearScreen();

    // if the checkmark is pressed
    if (Brain.buttonCheck.pressing()) 
    {
      // wait until button released and return selected mode
      while (Brain.buttonCheck.pressing()) {}
      return i;
    }

    // changes the current mode based on input
    if (Brain.buttonLeft.pressing()) 
    {
      i --; 
    } 
    else if (Brain.buttonRight.pressing()) 
    {
      i ++;
    }

    // waits until buttons are no longer being pressed
    while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing()) 
    {}

    // overflow conditions
    if (i == -1) 
    {
      i = MODE_EXIT;
    } 
    else if (i==4) 
    {
      i = MODE_DEAL;
    }
  }
}

/*
method runs user interface for choosing number of players
taking part in the game. returns a number between 2 and max
*/ 
int getNumPlayers(int max) 
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  
  wait(1,seconds);

  // number of players starts at 2
  int i = 2;

  // runs ui process until return statement
  while (true) 
  {
    // prompts the user and displays selected number of players
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("within [1,%d]",max);
    Brain.Screen.newLine();
    Brain.Screen.print("%d",i);

    // waits until any button is pressed
    while(!Brain.buttonLeft.pressing() 
          && !Brain.buttonRight.pressing() 
          && !Brain.buttonCheck.pressing()) 
    {}

    Brain.Screen.clearScreen();

    // checks if the checkmark was pressed
    if (Brain.buttonCheck.pressing()) 
    {
        // waits until button is released and then returns # players
      while (Brain.buttonCheck.pressing()) 
      {}
      return i;
    }

    // ensures # players selected stays within bounds
    if (i > 2 && Brain.buttonLeft.pressing()) 
    {
      i --; 
    } 
    else if (i < max && Brain.buttonRight.pressing()) 
    {
      i ++;
    }

    // waits until buttons are released before proceeding
    while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing()) 
    {}

  }
}

int getCardsPer(int max) 
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);

  wait(1,seconds);

  // cards per player starts at 1
  int i = 1;

  // runs ui process until return statement
  while (true) 
  {
    // prompts the user and displays selected number of cards per player
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("within [1,%d]",max);
    Brain.Screen.newLine();
    Brain.Screen.print("%d",i);

    // waits until any button is pressed
    while(!Brain.buttonLeft.pressing()
          && !Brain.buttonRight.pressing()
          && !Brain.buttonCheck.pressing()) 
    {}

    Brain.Screen.clearScreen();

    // checks if the checkmark was pressed
    if (Brain.buttonCheck.pressing()) 
    {
        // waits until button is released and then returns # cards
      while (Brain.buttonCheck.pressing()) {}
      return i;
    }

    // ensures # cards selected stays within bounds
    if (i > 1 && Brain.buttonLeft.pressing()) 
    {
      i --;
    } 
    else if (i < max && Brain.buttonRight.pressing()) 
    {
      i ++;
    }
    else if (i <= 1 && Brain.buttonLeft.pressing())
    {
        i = max;
    }
    else if (i > max && Brain.buttonRight.pressing())
    {
        i = 1;
    }

    // waits until buttons are released before proceeding
    while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing()) 
    {}

  }
}

void dispenseIndividualCardsUI(int numplayers) {
  double currentHeading = 0;

  rotateToHeadingPID(currentHeading);

  
  bool running = true;
  while (running) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("<> to rotate");
    Brain.Screen.newLine();
    Brain.Screen.print("check to dispense");
    Brain.Screen.newLine();
    Brain.Screen.print("use touchled to exit");
    Brain.Screen.newLine();

    TouchLED.setColor(color::blue);

    while (!Brain.buttonRight.pressing() &&
           !Brain.buttonLeft.pressing() &&
           !Brain.buttonCheck.pressing() &&
           !TouchLED.pressing()) 
    {}
    
    if (TouchLED.pressing()) {
      while (TouchLED.pressing()) {}
      running = false;
    }
    else if (Brain.buttonRight.pressing()) 
    { 
      while (Brain.buttonRight.pressing()) {}
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("turning right");
      rotateToHeadingPID(currentHeading+360.0/numplayers);
      currentHeading += 360.0/numplayers;
    } else if (Brain.buttonLeft.pressing()) 
    {
      while (Brain.buttonLeft.pressing()) {}
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("turning right");
      rotateToHeadingPID(currentHeading-360.0/numplayers);
      currentHeading -= 360.0/numplayers;
    } else if (Brain.buttonCheck.pressing())
    {
      while (Brain.buttonCheck.pressing()) {}
      dispenseOneCard();
    }
  }

  Brain.Screen.clearScreen();
}

// prompts user with whether to continue another deal cycle
// used for MODE_DEAL and MODE_SHUFFLE
bool askContinue(int numplayers) 
{
	wait(1, seconds);
	Brain.Screen.clearScreen();

	wait(1, seconds);

	int selection = 0;

	// runs ui process until return statement
  bool running = true;
	while (running) // runs until user confirms
  {
		Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);
		Brain.Screen.print("Continue?");
		Brain.Screen.newLine();

		// displays the options
		Brain.Screen.print("YES");
		if (selection == 0)
    {
			Brain.Screen.print(" ***");
    }
		Brain.Screen.newLine();
		Brain.Screen.print("NO");
		if (selection == 1)
    {
			Brain.Screen.print(" ***");
    }
		Brain.Screen.newLine();
    Brain.Screen.print("RE-DISPENSE");
		if (selection == 2)
    {
			Brain.Screen.print(" ***");
    }
		Brain.Screen.newLine();

		while(!Brain.buttonLeft.pressing()
		      && !Brain.buttonRight.pressing()
		      && !Brain.buttonCheck.pressing()) 
    {}

		Brain.Screen.clearScreen(); // reloads screen when button pressed

        // if checkmark/user confirms
		if (Brain.buttonCheck.pressing()) 
    {
			while (Brain.buttonCheck.pressing()) 
      {}
      if (selection != 2) {
        return (selection == 0); 
      }
      else {
        // run dispense individual cards ui
        dispenseIndividualCardsUI(numplayers);
      }
      // if selection is 0 (yes), true - if not 0, false
		}


        // if other button is pressed, alter selection -> reload screen
		if (Brain.buttonLeft.pressing()) 
    {
			selection--;
		} 
    else if (Brain.buttonRight.pressing()) 
    {
			selection++;
		}
		while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing()) 
    {}

		// overflow conditions (wrap around)
		if (selection == -1) 
    {
			selection = 2;
		} 
    else if (selection == 3) 
    {
			selection = 0;
		}
        // end of while loop - if checkmark not pressed, goes back to top
	}
  return false;
}

// get current color of card in tray
int getCardColor() 
{
  double hue = OpticalSensor.hue();
  // hearts red
  if (hue >= 0 && hue < 20) 
  {
    return 0;
  }

  // spades orange/ brown
  else if (hue >= 20 && hue < 45) 
  {
    return 1;
  }

  // diamonds blue purple pink
  else if (hue >= 215 && hue < 360) 
  {
    return 2;
  }

  // clubs yellow and green
  else if (hue >= 45 && hue <= 150)
  {
    return 3;
  }

  // if cyan is seen
  else if (hue >= 180 && hue < 205)
  {
    return 4;
  }

  else 
  {
    return 5; // incase something weird happens
  }
}

// sort cards into 4 suits
void colorSort() 
{
  const int piles = 6;
  // int cardsPerPile[4] = { 0, 0, 0, 0 };
  int cardsPerPile[piles] = {0, 0, 0, 0, 0,0};

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Sorting cards by color");

  int colorPile = 0;
  // 4 is cyan
  while (colorPile != 4)
  //for (int i = 0; i < 52; i++) 
  {
    colorPile = getCardColor();

    if (colorPile == 0) // red
    {
      rotateToHeadingPID(0);
      wait(200, msec);
    }
    else if (colorPile == 1) // blue
    {
      rotateToHeadingPID(90);
      wait(200, msec);
    }
    else if (colorPile == 2) // green
    {
      rotateToHeadingPID(180);
      wait(200, msec);
    }
    else if (colorPile == 3) // yellow
    {
      rotateToHeadingPID(270);
      wait(200, msec);
    } 
    else if (colorPile == 5)
    {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("unrecognized color %f", OpticalSensor.hue());
      wait(5000, msec);
      
    }

    dispenseOneCard();
    wait(100, msec);

    cardsPerPile[colorPile]++;
  }

  Brain.Screen.clearScreen();
  for (int i = 0; i < 4; i++) 
  { // displays results of sorting
    Brain.Screen.setCursor(i + 1, 1);
    Brain.Screen.print("pile %d has %d cards", i, cardsPerPile[i]);
  }
  
  wait(1,seconds);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("cyan: %d", cardsPerPile[4]);
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("NA: %d", cardsPerPile[5]);

}

// ---------------------- main: random shuffle dealing ----------------------
int main()
{
	vexcodeInit();
	configureAllSensors();

	srand(Brain.Timer.time(msec));

  // while (true) {
  //   wait(1,seconds);

  //   Brain.Screen.clearScreen();
  //   Brain.Screen.setCursor(1,1);
  //   Brain.Screen.print("hue: %.2f", OpticalSensor.hue());
  // }

	// gets deal mode
	int mode = selectMode();
  int players = 0;
  int max = 0;
  int cardsPer = 0;
  while(mode != MODE_EXIT)
  {
    // displaying selected mode/asking for values
    Brain.Screen.clearScreen();
	  Brain.Screen.setCursor(1,1);
	  if (mode == MODE_DEAL)
    {
		  Brain.Screen.print("deal selected");
      wait(1,seconds);

	    // gets how many players tehre are
	    players = getNumPlayers(10);
	    Brain.Screen.clearScreen();
	    Brain.Screen.setCursor(1,1);
  	  Brain.Screen.print("%d players", players);

	    wait(1, seconds);

	    // gets cards per person
      max = 52/players;
	    cardsPer = getCardsPer(max);
	    Brain.Screen.clearScreen();
	    Brain.Screen.setCursor(1,1);
	    Brain.Screen.print("%d cards per", cardsPer);
	  }
    else if (mode == MODE_SHUFFLE)
    {
		  Brain.Screen.print("shuffle selected");
      wait(1,seconds);

	    // gets how many players tehre are
	    players = getNumPlayers(10);
	    Brain.Screen.clearScreen();
	    Brain.Screen.setCursor(1,1);
  	  Brain.Screen.print("%d players", players);

	    wait(1, seconds);

	    // gets cards per person
      max = 52/players;
	    cardsPer = getCardsPer(max);
	    Brain.Screen.clearScreen();
	    Brain.Screen.setCursor(1,1);
	    Brain.Screen.print("%d cards per", cardsPer);
	  }
    else if (mode == MODE_SORT)
    {
		  Brain.Screen.print("sort selected");
      wait(1, seconds);
	  }


    // looping code
	  if (mode == MODE_DEAL)
    {

		  bool keepDealing = true;
		  int cycle = 0;

		  // infinite loop for dealing cycles
		  while (keepDealing)
      {
            int cyanCheck = getCardColor();
            while (cyanCheck == 4)
            {
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1,1);
                Brain.Screen.print("no cards detected");
                Brain.Screen.newLine();
                Brain.Screen.print("put in cards and");
                Brain.Screen.newLine();
                Brain.Screen.print("press check to continue.");
                while(!Brain.buttonCheck.pressing())
                {}
                while(Brain.buttonCheck.pressing())
                {}
                cyanCheck = getCardColor();
            }
			  cycle++;

			  // deals cards to each player, based on how many cards per player
			  wait(100, msec);
			  Brain.Screen.clearScreen();
			  Brain.Screen.setCursor(1,1);
			  Brain.Screen.print("dealing cards (cycle %d)", cycle);

			  for (int i = 0; i < cardsPer; i++)
        {
				  for (int j = 0; j < players; j++)
          {
					  double heading = 360.0 / players * j;
					  // "divides" 360 degrees into angles based on how many players, then multiplies by j for the current player
					  dealCardsToPosition(heading, 1);
				  }
			  }

			  Brain.Screen.clearScreen();
			  Brain.Screen.setCursor(1,1);
			  Brain.Screen.print("dealt %d cards to %d players", cardsPer, players);

			  wait(1, seconds);

			  // ask user if they want another cycle
			  keepDealing = askContinue(players);
        
        wait(1, seconds);
		  } 

		  Brain.Screen.clearScreen();
		  Brain.Screen.setCursor(1,1);
		  Brain.Screen.print("completed %d cycles", cycle);
	  }
    else if (mode == MODE_SHUFFLE)
    {
		  // infinite loop for shuffle dealing cycles
		  bool keepDealing = true;
		  int cycle = 0;

		  while (keepDealing)
      {
            
            int cyanCheck = getCardColor();
            while (cyanCheck == 4)
            {
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1,1);
                Brain.Screen.print("no cards detected");
                Brain.Screen.newLine();
                Brain.Screen.print("put in cards and");
                Brain.Screen.newLine();
                Brain.Screen.print("press check to continue.");
                while(!Brain.buttonCheck.pressing())
                {}
                while(Brain.buttonCheck.pressing())
                {}
                cyanCheck = getCardColor();
            }
            
			  cycle++;

			  // runs shuffle dealing function
			  wait(1, seconds);
			  Brain.Screen.clearScreen();
			  Brain.Screen.setCursor(1,1);
			  Brain.Screen.print("shuffle dealing (cycle %d)", cycle);

			  shuffleDeal(players, cardsPer);

			  Brain.Screen.clearScreen();
			  Brain.Screen.setCursor(1,1);
			  Brain.Screen.print("dealt %d cards to %d players", cardsPer, players);

			  wait(1, seconds);

			  // ask user if they want another cycle
			  keepDealing = askContinue(players);

        wait(1, seconds);
		  }

		  Brain.Screen.clearScreen();
		  Brain.Screen.setCursor(1,1);
		  Brain.Screen.print("Completed %d cycles", cycle);
	  }
    else if (mode == MODE_SORT)
    {
        int cyanCheck = getCardColor();
            while (cyanCheck == 4)
            {
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1,1);
                Brain.Screen.print("no cards detected");
                Brain.Screen.newLine();
                Brain.Screen.print("put in cards and");
                Brain.Screen.newLine();
                Brain.Screen.print("press check to continue.");
                while(!Brain.buttonCheck.pressing())
                {}
                while(Brain.buttonCheck.pressing())
                {}
                cyanCheck = getCardColor();
            }
		  // runs colorSort()
		  wait(1, seconds);
		  colorSort();
	  }

	  wait(5,seconds);
	  mode = selectMode();
  }
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("returning to start position");
  rotateToHeadingPID(0);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("exiting.");
  wait(1,seconds);

  Brain.programStop();
}