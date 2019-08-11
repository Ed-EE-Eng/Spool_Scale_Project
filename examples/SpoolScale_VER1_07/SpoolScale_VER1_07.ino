#define ProgName "SpoolScale_VER1.07"
//	LOOP and SETUP Revision History
//	Note: Developed and tested using Arduino IDE ver 1.8.9 
//
//	20190105	0.00	Development begins -- Many on going revisions
//	20190325	1.00	[DB REv Code: AMC0] Fully functional - Known Problems:
//						1) Filament-Diameter value does not save to EEPROM properly.
//						2) WT vale from sensor needs filtering adjusted...Bounces +/- 1 to 2g
//	20190401	1.05	[DB Rev Code: AMC1]  
//						Add FILTER parameter to CalCfg DB
//	20190401	1.06	[DB Rev Code: AMC1]  General code clean up prior to Github Upload
//	20190730	1.07	[DB Rev Code: AMC1]  Fix calibration error.			
//
 
#include <LiquidCrystal_I2C.h>

#include <Bounce2.h>			//This is a general use switch debouncing library
								//https://github.com/thomasfredericks/Bounce2
#include <HX711.h>

extern void SaveAllToEEPROM(uint8_t SaveWithAsk=true); //Req'd to resolve this compile Error: 'SaveAllToEEPROM' was not declared in this scope
const uint8_t GBL_AutoSaveFlag=false;	//Set true to force oper to confirm all EEPROM writes
										//Set to false skip oper confirmation and ALWAYS approve EEPROM writes


//Common I2C lcd address option are typically 0x38 or 0x27.  Set the value to one that matches your particular LCD display module
#define I2C_ADDR    0x27  // Define I2C Address for 20x4 LCD module
//#define I2C_ADDR    0x38  // Define I2C Address for 20x4 LCD module

// The following Define pins to I2C version of display
//#define BACKLIGHT_PIN     13	//Alternate value used by some LCDs
#define BACKLIGHT_PIN	3	
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
//=== Define variables & create an instance of the I2C LCD display driver
LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
//I2C Liquid Crystal NANO pin assignments are as follows: SCLK=A5, SDATA=A4

//=== Assign pins used for HX711 LOAD CELL Interface
const int LOADCELL_DOUT_PIN = A0;
const int LOADCELL_SCK_PIN = A1;
HX711 scale;

//============= DEFINE GLOBAL VARIABLES
#define Tab	Serial.print("\t")	//Just a handy substitution for a "Tab" character
const int StatusLedPin=13;
int Fnum =0;
const uint8_t FnumMax=16;		//Defines the maximum number of Filaments supported in the Filament DataBase.
								//Nano doesn't have much RAM; If this is set too big, you can run out of RAM!
float WtKg,NewWtKg,RawWtKg;

char TypeName[5];				//each filament type can have up to 4 characters lomg (+ terminator)

float GBL_Diam_mm,GBL_Dens_gcm3;	//Global Variables to hold values for the currently selected filament


//A few constants for decision making...
const uint8_t Yes = 1;		//Yes
const uint8_t No = 0;		//No
const uint8_t Cancel = 0;	//CANCEL
const uint8_t Accept = 1;	//ACCEPT
	
//Uncomment one of the following two lines to define nominal filament diameter
float const Default_Fil_Diam = 1.75;
//float const Default_Fil_Diam = 3.00;

//Master Menu State Machine 'State-number' variable
uint8_t GBL_menuNum=0;
//float SpoolWtG=125.0;		//powerup default SpoolWt value - No longer used; Left over from initial, non-data base version.

//Include basic eePROM library
#include <EEPROM.h>



struct CAL_CONFIG_STRUCT
{
	char DB_RevCode[5];			//		5 Bytes - A/N code for the Cal & Config DB_Rev
	float LdCell_Scale;			//		4 Bytes - Calibration factor
	float LdCell_Offset;		//		4 Bytes - Zero offset value
	float LdCell_Filter;		//		4 Bytes - Filter Factor (0=No Filter, .99 = Max Filter)
	float Last_SpoolWtG;		//		4 Bytes - Default SpoolWt (g)
	uint8_t Last_Fnum;			//		1 Byte  - Last active filament number (valid values: 1 to FnumMax)
	uint8_t Gbl_LenUnits;		//		1 Byte  - Preferred "length units" (0=m, 1=cm, 2=mm) 
	uint8_t	Gbl_WtUnits;		//		1 Byte  - Desired output Wt Units (0=Kg, 1=g)	
								//	  ---------	  ^==== Reserved for future use ====^ (Current design only outputs Kg!)
								//Tot: 24 Bytes
};
//Create and define our 'CalCfg' defaults
CAL_CONFIG_STRUCT CalCfg_DEFAULT = {
	"AMC1",			//		5 Bytes - 4-character string that defines Config & data base version
	392582.4176,	//		4 Bytes - Load-Cell Units Gain Calibration value. This is updated by the CALIBRATION routine.
	 -157616.,		//		4 Bytes - Load-Cell Tare value. This is Measured & updated each time the load cell is ZERO'D
	 .80,			//		4 Bytes - Display Filter Factor (0=No Filter, .99 = Max Filter)
	 125.,			//		4 Bytes - Active Spool Wt (g). Can be Entered/updated/saved by operator 
	 1,				//		1 Byte  - Last Filament Number selected and in use; Can be Entered/updated/saved by operator
	 0,				//		1 Byte  - Length units (0=m,1=cm,2=mm) <-Recommended:'m'.  Can be Entered/updated.saved by operator
	 0				//		1 Byte  - Wt units (0=Kg,1=g).  Can be Entered/updated/saved by operator
					//	  ---------	
					//Tot: 24 Bytes
};
long CalCfg_CRC_DEFAULT=0;
CAL_CONFIG_STRUCT CalCfg_ACTIVE;
long CalCfg_CRC_ACTIVE=0;
//CAL_CONFIG_STRUCT CalCfg_EEPROM;
long CalCfg_CRC_EEPROM=0;

//Define Filament DataBase Structure
struct FILAMENT_DATABASE
{
	char Name[5];	//		5 Bytes - Filament Name that shows up on LCD
	float Dia;		//		4 Bytes - Filament Diameter in mm
	float Dens;		//		4 Bytes - Filament Density in g/cm^3
					//	  ---------
					//Tot: 13 Bytes per member of this structure
};


// Ram & EEPROM is Tight! With FnumMax = 16, we 
// use-up 208 Bytes of RAM & EEPROM (16 x 13 = 208)
// For each FILAMENT_DATABASE structure we define.
struct FILAMENT_DATABASE FilType_DEFAULT[FnumMax] = {
	//Name ,Diameter_mm     ,Density_g/cm^3
	{"PLA ",Default_Fil_Diam,1.24},	//00
	{"ABS ",Default_Fil_Diam,1.04},	//01
	{"ASA ",Default_Fil_Diam,1.07},	//02
	{"PETG",Default_Fil_Diam,1.27},	//03
	{"NYLN",Default_Fil_Diam,1.08},	//04
	{"PlyC",Default_Fil_Diam,1.20},	//05
	{"HIPS",Default_Fil_Diam,1.07},	//06
	{"PVA ",Default_Fil_Diam,1.19},	//07
	{"TPU ",Default_Fil_Diam,1.20},	//08
	{"TPE ",Default_Fil_Diam,1.20},	//09
	{"PMMA",Default_Fil_Diam,1.18},	//10
	{"Copr",Default_Fil_Diam,3.90},	//11
	{"????",Default_Fil_Diam,0.0},	//12
	{"????",Default_Fil_Diam,0.0},	//13
	{"????",Default_Fil_Diam,0.0},	//14
	{"????",Default_Fil_Diam,0.0}	//15
};
long FilType_CRC_DEFAULT=0;
struct FILAMENT_DATABASE FilType_ACTIVE[FnumMax];	//Define up to FnumMax filament types 
													// Ram is Tight! FnumMax is usually set to 16 
													// which uses-up 208 Bytes of RAM (16 x 13 = 208).
long FilType_CRC_ACTIVE=0;											
//struct FILAMENT_DATABASE FilType_EEPROM[FnumMax];	//Define up to FnumMax filament types 
													// Ram is Tight! FnumMax is usually set to 16 
													// which uses-up 208 Bytes of RAM (16 x 13 = 208).

//Define eeprom start address for CalCfg structure at start of eeprom
const int eeprom_CalCfg_Adr = 0;
//Put the CRC right after the CalCfg data.
const int eeprom_CalCfg_CRC_Adr=eeprom_CalCfg_Adr+sizeof(CalCfg_ACTIVE);

//Define eeprom start address for CalCfg structure, arbitrarily set to '50'
const int eeprom_FilTyp_Adr = 50;
//Put the CRC right after the CalCfg data.
const int eeprom_FilTyp_CRC_Adr= eeprom_FilTyp_Adr +sizeof(FilType_ACTIVE);
//Define Config & Calibration Data Structure
											
long FilType_CRC_EEPROM=0;

//====CUSTOM LCD SYMBOLS & CHARACTERS===================================================
// Create some custom characters

const uint8_t SYM_Bitmap[][8]={
{0x06,0x08,0x08,0x06,0x00,0x1A,0x15,0x15} ,  // char(0) = cm
{0x1A,0x15,0x15,0x00,0x1A,0x15,0x15,0x00} ,  // char(1) = mm
{0x08,0x14,0x0C,0x04,0x19,0x02,0x04,0x08} ,  // char(2) = g/ (GramPer…)
{0x0E,0x01,0x06,0x01,0x0E,0x00,0x00,0x00} ,  // char(3) = ^3
{0x00,0x00,0x04,0x0E,0x1F,0x00,0x00,0x00} ,  // char(4) = Up Arrow
{0x00,0x00,0x1F,0x0E,0x04,0x00,0x00,0x00} ,  // char(5) = Down Arrow
{0x00,0x08,0x0C,0x0E,0x0C,0x08,0x00,0x00} ,  // char(6) = Next
{0x00,0x01,0x01,0x05,0x0D,0x1F,0x0C,0x04}    // char(7) = Enter
};
#define SYM_cm 0
#define SYM_mm 1
#define SYM_GramPer 2
#define SYM_Cubed 3
#define SYM_UpArrow 4
#define SYM_DwnArrow 5
#define SYM_NxtArrow 6
#define SYM_Enter 7

//Macro that will print custom character symbols (such as "g/cm^3") at current LCD location
#define lcd_print_cm lcd.print(char(SYM_cm));
#define lcd_print_mm lcd.print(char(SYM_mm));
#define lcd_print_G_Per_CM3 lcd.print(char(SYM_GramPer));lcd.print(char(SYM_cm));lcd.print(char(SYM_Cubed))
#define lcd_print_Cubed lcd.print(char(SYM_Cubed));
#define lcd_print_UpArrow lcd.print(char(SYM_UpArrow));
#define lcd_print_DwnArrow lcd.print(char(SYM_DwnArrow));
#define lcd_print_NxtArrow lcd.print(char(SYM_NxtArrow));
#define lcd_print_Enter lcd.print(char(SYM_Enter));
//========== BEGIN ROTARY ENCODER DEFINITIONS ====================================================
#include <Encoder.h>
#define EncPin_A 2
#define EncPin_B 3
#define EncPB_Pin 4
//#define PB_Debug
uint8_t Enc_PB_value=HIGH;
uint8_t Enc_PB_DownEdge=HIGH;
uint8_t Enc_PB_UpEdge=HIGH;
uint8_t Enc_PB_Last_value=HIGH;

//Instantiate an 'Encoder object' for the rotary encoder control
Encoder Enc_Knob(EncPin_A,EncPin_B);		//Wire Rotary encoder to pins 2 & 3 (These are Nano Interrupt Pins!)
//Instantiate a 'Bounce object' for the Encoder Push Button
Bounce Enc_PB_debounce = Bounce(); 
//========== END ROTARY ENCODER DEFINITIONS ======================================================

void Read_Enc_Push_Button(){
	//	Routine to read Encoder Push-Button and set global push button variable.
	//
	//	Passed Parameters	NONE
	//
	//	Returns Via Global Variables:  
	//			Enc_PB_value,Enc_PB_DownEdge, Enc_PB_UpEdge, Enc_PB_Last_value
	//
	//	20190409 Ver 0.1	E.Andrews	Adapted from prior project.
	//
	Enc_PB_debounce.update();
	Enc_PB_value = Enc_PB_debounce.read();	
	if (Enc_PB_value==HIGH && Enc_PB_Last_value==HIGH) {	//Has value remained HI (aka: not pressed) since last pass?
		//No button push is in process...
		//Enc_PB_DownEdge=LOW;	//Comment out if you want to let application code reset this value
		//Enc_PB_UpEdge=LOW;		//Comment out if you want to let application code reset this value
	}		
	if (Enc_PB_value==LOW && Enc_PB_Last_value==HIGH) {
		//Detect a DOWN Edge...(Button just being depressed)
		Enc_PB_DownEdge=HIGH;
		Enc_PB_UpEdge=LOW;
		#ifdef PB_Debug
			Serial.println (F("==== ENC_PB Pressed (DOWN Edge)"));
		#endif
	}
	if (Enc_PB_value==HIGH && Enc_PB_Last_value==LOW) {
		//Detect an UP Edge...(Button being released)
		Enc_PB_DownEdge=LOW;
		Enc_PB_UpEdge=HIGH;
		#ifdef PB_Debug
			Serial.println (F("==== ENC_PB Released (UP Edge)"));
		#endif
	}		
	Enc_PB_Last_value=Enc_PB_value;
}
void ResetEncPushButton(){
	//	Routine to reset/initialize global Encoder Push-button variables.
	//
	//	Passed Parameters	NONE
	//
	//	Returns Via Global Variables:  RESET values for all buttons
	//
	//	20181005 Ver 0.0	E.Andrews	First cut - (Make button update a subroutine that can be called from anywhere)
	//	20190409 Ver 0.1	E.Andrews	Adapted from prior project.
	//
		
		const bool ResetState=LOW;
		Enc_PB_DownEdge=ResetState;
		Enc_PB_UpEdge=ResetState;
		Enc_PB_Last_value=Enc_PB_value=LOW;
}



//#define PushButtonsInMainline	//  [Comment-out this line if no descrete Push buttons are connected so that PB code will be omitted from build]

#ifdef PushButtonsInMainline	//Button constants and variables must be defined here as they are GLOBAL in nature!

	//======= BEGIN 4-BUTTON DEFINITIONS ========================================================== 
	int const PB1_Pin=10;	//Use D8,9,10,11,12 when plugging 4-position membrane switch on to NANO board
	int const PB2_Pin=11;	//as ribbon connector from the button array is easily plugged onto these 5 adjacent pins 
	int const PB3_Pin=8;	// Note: Port assignments depends on wiring of membrane switch
	int const PB4_Pin=9;	//       and is NOT "Sequential" as you might guess! Use an ohm-meter
							//       to decipher & check your particular button array wiring.
	int const PBcomPin=12;	//This pin is 'COMMON' for 4-position membrane button array
							//and must be set LOW in software for button sensing to work.
	//======= END 4-BUTTON PIN DEFINITIONS

	//.........DEFINE 4-BUTTON CONTROL PANEL - Push Buttons for clock-set & other functions
	//	These buttons use <bounce2.h)> library
	//								//	Button Array when mounted horizontally and viewed from FRONT panel
	//								//	+-------------------------------------------+
	//								//	| 	LCD Row 0, (20 Characters each row)		|
	//								//	|	LCD Row 2								|
	//								//	|	LCD Row 3								|
	//								//	|	LCD Row 4								|
	//								//	+-------------------------------------------+

	//.........Declare GLOBAL PUSH BUTTON variables
	//	These are Global PUSH BUTTON variables that enable the button scanning routines to be called
	//	from anywhere in the mainline or it's subroutines.  

	unsigned long PB_RepeatMs;
	unsigned long PB_LastPushedMs;						//Define "LastPushedMs" TimeStamp

	 
	const long PB_RepeatInitialDelayMs=900;	//Initial Delay in milliseconds (900 means an initial delay of 0.9 Second before repeat begins)
	const long PB_RepeatRatePeriodMs=330;	//Repeat Rate Period in milliseconds (330 means a repeat rate about 3 x per second)

	int PB1_Last_value=HIGH, PB1_value=HIGH;
	int PB1_UpEdge=LOW;
	int PB1_DownEdge=LOW;

	int PB2_Last_value=HIGH, PB2_value=HIGH; 
	int PB2_UpEdge=LOW; 
	int PB2_DownEdge=LOW;

	int PB3_Last_value=HIGH, PB3_value=HIGH; 
	int PB3_UpEdge=LOW; 
	int PB3_DownEdge=LOW;

	int PB4_Last_value=HIGH, PB4_value=HIGH; 
	int PB4_UpEdge=LOW; 
	int PB4_DownEdge=LOW;

	//======= END 4-BUTTON DEFINITIONS ==========================================================
#endif

 
 //==== BEGIN LCD PRINT ROUTINES===================================================
void lcd_printMenu(int MenuNumber,uint8_t lcdRow=3){
	// Routine to print a the Menu items to Line 3 of the display
	//
	//	INPUTS	
	//	int		MenuNumber	The menu number to actually print to Line 3 of LCD
	//	uint8_t	lcdRow		LCD Row # to print the menu item.  If omitted, row # =3
	//
	//	RETURNS	Nothing
	//
	//	20190410	E.Andrews	Rev 0	Initial cut
	//
	
	lcd.setCursor(0,lcdRow);
	lcd_print_NxtArrow;
	if (MenuNumber<10)lcd.print(F(" "));
	lcd.print(MenuNumber);lcd.print(F(")"));
	switch (MenuNumber){
	case 1:
		lcd.print (F("ZERO scale"));
	break;
	case 2:
		lcd.print (F("Chng SPOOL WT"));
	break;
	case 3:
		lcd.print (F("Select Filament"));
	break;
	case 4:
		lcd.print (F("Chng FIL DENS."));
	break;
	case 5:
		lcd.print (F("Chng FIL DIAM."));
	break;
	case 6:
		lcd.print (F("Chng FIL NAME"));
	break;
	case 7:
		lcd.print (F("Save to EEPROM"));
	break;
	case 8:
		lcd.print (F("Calibrate Scale"));
	break;
	case 9:
		lcd.print (F("Set Scale Filter"));
	break;
	case 10:
		lcd.print (F("Serial DB Dump"));
	break;
	case 11:
		lcd.print (F("Erase EEPROM"));
	break;
	case 12:
		lcd.print (F("EXIT Menu"));
	break;
	default:	//Print this message only if MenuNumber is not defined within this routine!
		lcd.print (F("***Undefined# "));lcd.print(MenuNumber); lcd.print(F("***"));
	break;
	}
}
void lcd_print_2d(uint8_t v,char LeadCharacter='0'){
	// Routine to print a 2 digit positive decimal number to LCD  
	//  Will insert leading 'LeadCharacter' (default=zero) when needed.
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//	20190427	E.Andrews	Rev 1	Added 'LeadCharacter'All for caller to specify the leading character
	//

  if ( v < 10 ) lcd.print(LeadCharacter);
  lcd.print(v, DEC);
}
void lcd_print_3d(uint16_t v,char LeadCharacter='0'){
	// Routine to print a 3 digit positive decimal number to LCD  
	//  Will insert leading 'LeadCharacter' (default=zero) when needed.
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//	20190427	E.Andrews	Rev 1	Added 'LeadCharacter'All for caller to specify the leading character
	//
  if ( v < 10 )		lcd.print(LeadCharacter);	
  if ( v < 100 )	lcd.print(LeadCharacter);  
  lcd.print(v, DEC);
}
void lcd_print_4d(uint16_t v,char LeadCharacter='0'){
	// Routine to print a 4 digit positive decimal number to LCD  
	//  Will insert leading 'LeadCharacter' (default=zero) when needed.
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//	20190427	E.Andrews	Rev 1	Added 'LeadCharacter'All for caller to specify the leading character
	//
	if ( v < 10  )	lcd.print(LeadCharacter);
	if ( v < 100 )	lcd.print(LeadCharacter);
	if ( v < 1000)	lcd.print(LeadCharacter);  
	lcd.print(v, DEC);
}
void lcd_print_5d(uint16_t v,char LeadCharacter='0'){
	// Routine to print a 5 digit positive decimal number to LCD  
	//  Will insert leading 'LeadCharacter' (default=zero) when needed.
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//
	if ( v < 10 )   	lcd.print(LeadCharacter);
	if ( v < 100 )  	lcd.print(LeadCharacter);
	if ( v < 1000 ) 	lcd.print(LeadCharacter);
	if ( v < 10000 )	lcd.print(LeadCharacter);  
	lcd.print(v, DEC);
}
void lcd_print_date(uint16_t y, uint8_t m, uint8_t d){
	//	Routine to print 2 digit month to LCD
	//	in a yyyy-mm-dd format
	//	Will insert leading zeros when needed
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//

	lcd.print(y, DEC);
	lcd.print(F("-"));
	lcd_print_2d(m);
	lcd.print(F("-"));
	lcd_print_2d(d);
}
void lcd_printInteger(int v, uint8_t Ndigits=6,char LeadCharacter='0') {
	// 	Routine to print a Ndigits decimal number to LCD  
	//  Will insert leading 'LeadCharacter' (default=zero) when needed.
	//INPUT
	//	int 		v				Data to be formatted & Printed to lcd
	//	uint8_t		Ndigits			Number of digit	places to print, range: 1 to 6
	//	char		LeadCharacter 	The character that will be inserted as needed (Default: '0')
	//RETURNS 
	//   Nothing
	//
	//	NOTE: Minimal range checking is done; If a negative number is passed into routine,
	//	a minus-sign will precede the numbers AND an Ndigits will be reduced by one position.
	//	Caller must be sure not to send a value > than the number of
	//	places specified or incorrect print out will result!
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//	20190427	E.Andrews	Rev 1	Allow call to specify LeadCharacter & will accept negative nums...
	//
	if (v<0){
		lcd.print(F("-"));	//Print 'negative sign' for minus numbers
		Ndigits--;			//Decrease Ndigits by one position due to injections of the minus sign...
	}
	if ( abs(v) < 10 && Ndigits> 1)   		lcd.print(LeadCharacter);	//0-9
	if ( abs(v) < 100 && Ndigits> 2)  		lcd.print(LeadCharacter);	//00-99
	if ( abs(v) < 1000 && Ndigits> 3) 		lcd.print(LeadCharacter);	//000-999
	if ( abs(v) < 10000 && Ndigits> 4)		lcd.print(LeadCharacter);  //0000-9999
	if ( abs(v) < 100000 && Ndigits> 5)		lcd.print(LeadCharacter);  //0000-9999
	if ( abs(v) < 1000000  && Ndigits> 6)	lcd.print(LeadCharacter);  //00000-99999
	lcd.print(abs(v), DEC);
}
void lcd_printFloat(float value, uint8_t places) {
	// 	Routine that prints out the float 'value' rounded to 'places' places to the right of the decimal point
	//	Formats and sends Floating Point data to LCD display with "places" level of precision.
	//	Note: Routine will precede values with a minus sign for negative numbers.  A Plus sign DOES NOT appear for positive value.	
	//INPUT
	//   value =  Data to be formatted
	//   places = number of places to print
	//RETURNS 
	//   Nothing
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//	
	uint8_t digit;
	float tens = 0.1;
	uint8_t tenscount= 0;
	uint8_t i;
	float tempfloat = value;

	// Make sure we round properly. This could use pow from <math.h>, but doesn't seem worth the import
	// however, if this rounding step isn't here, for example, the value  54.321 prints as 54.3209

	// calculate rounding term d:   0.5/pow(10,places)  
	float d = 0.5;
	if (value < 0) d *= -1.0;
	
	// divide by ten for each decimal place
	for (i = 0; i < places; i++) d/= 10.0;    
	
	// this small addition, combined with truncation will round our values properly 
	tempfloat +=  d;

	// first get value tens to be the large power of ten less than value
	// tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

	if (value < 0)
		tempfloat *= -1.0;
	while ((tens * 10.0) <= tempfloat)
	{
		tens *= 10.0;
		tenscount += 1;
	}
	// write out the negative if needed
	if (value < 0)
		lcd.print('-');

	if (tenscount == 0)
		lcd.print(0, DEC);

	for (i=0; i< tenscount; i++) 
	{
		digit = (int) (tempfloat/tens);
		lcd.print(digit, DEC);
		tempfloat = tempfloat - ((float)digit * tens);
		tens /= 10.0;
	}

	// if no places after decimal, stop now and return
	if (places <= 0)
		return;

	// otherwise, write the point and continue on
	lcd.print('.');  

	// now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
	for (i = 0; i < places; i++) 
	{
		tempfloat *= 10.0; 
		digit = (int) tempfloat;
		lcd.print(digit,DEC);  
		// once written, subtract off that digit
		tempfloat = tempfloat - (float) digit; 
	}
}
void lcd_clearRows(uint8_t StartRow,uint8_t NumRows) {
	//	Routine to blank out 'NumRows' starting at 'StartRow' on display and then
	//	leaves cursor to Left most position of spec'd StartRow.
	//	Note: This routine is setup to function with a 4x20 A/N LCD Display
	//INPUT
	//  StartRow 	The Row to be cleared. Valid row numbers are 0-3 for 4x24 display.
	//	NumRows		The number of rows to blank out; valid numbers are 1-4 for 4x24 display
	//RETURNS 
	//   Nothing
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//

	for (uint8_t i=StartRow; i <= (StartRow+(NumRows-1)); i++)
	{
		lcd.setCursor(0,i);
		lcd.print(F("                    "));
	}
	lcd.setCursor(0,StartRow);		//Leave cursor at start of erase zone...
}
void lcd_clearRow(uint8_t StartRow) {
	//	Routine to blank out 'NumRows' starting at 'StartRow' on display and then
	//	leaves cursor to Left most position of spec'd StartRow.
	//	Note: This routine is setup to function with a 4x20 A/N LCD Display
	//INPUT
	//  StartRow 	The Row to be cleared. Valid row numbers are 0-3 for 4x24 display.
	//RETURNS 
	//   Nothing
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//
	const uint8_t NumRows=1;
	for (uint8_t i=StartRow; i <= (StartRow+(NumRows-1)); i++)
	{
		lcd.setCursor(0,i);
		lcd.print(F("                    "));
	}
	lcd.setCursor(0,StartRow);		//Leave cursor at start of erase zone...
}
//===== END LCD ROUTINES ===========================================================

//==== BEGIN MENU & RELATED HELPER ROUTINES ========================================
float ZeroScale(){
	// Routine to print perform the scale ZERO function
	//
	//	INPUTS	None
	//
	//	RETURNS	This routine will output directly into 
	//		the CalCfg database and write a new value into
	//		CalCfg_ACTIVE.LdCell_Offset.  It will update the value
	//		to EEPROM if const GBL_AutoSaveFlag configured to be true.
	//
	//	20190410	E.Andrews	Rev 0	Initial cut
	//	20190421	E.Andrews	Rev	1	Changed to AUTO UPDATE CalCfg_ACTIVE.LdCell_Offset
	//									and autosave to EEPROM.
	//	
	lcd_clearRows(0,3);
	lcd.setCursor(0,0);
	lcd.print(F("1.REMOVE ALL LOADS!"));
	delay(3000);
	lcd.setCursor(0,1);
	lcd.print(F("2.TAKING READINGS..."));
	scale.tare(20);
	delay(1000);
	lcd.setCursor(0,2);
	lcd.print(F("3.ZEROING DONE! "));						
	delay(1000);
	
	//CalCfg_ACTIVE.LdCell_Offset=scale.get_offset();  //Load new ZERO value into CalCfg DB.
	//SaveAllToEEPROM(GBL_AutoSaveFlag);	
	return scale.get_offset();  //Return new ZERO value into CalCfg DB.

	delay(1000);
}

float SetSpoolWt(float SpoolWtG){
	// Routine to enter enter new spool wt value
	//
	//	INPUTS	None
	//
	//	RETURNS		Changes currently active SpoolWtG variable.
	//
	//	20190410	E.Andrews	Rev 0	Initial cut
	//		
	int NewSpoolWtG = int(SpoolWtG);
	const int MaxSpoolWt=300;
	const int MinSpoolWt=0;
	Enc_Knob.write(0);		//Reset Encoder value back to Zero
	lcd_clearRows(0,4);
	lcd.setCursor(0,0);
	//           012345678901235456789
	lcd.setCursor(0,0);	lcd.print(F("Set value with knob."));
	lcd.setCursor(0,1); lcd.print(F("Push knob when done.")); 
	lcd.setCursor(0,2);lcd.print(F("New Spool Wt "));lcd_print_NxtArrow; lcd_print_3d(NewSpoolWtG); lcd.print (F(" g"));

	uint8_t Done=0;
	while (Done==0){
		Read_Enc_Push_Button();							//Read Encoder Push Button
		if((Enc_Knob.read()>2||Enc_Knob.read()<-2) && Enc_PB_DownEdge==LOW){
			NewSpoolWtG=NewSpoolWtG+(Enc_Knob.read()/2);
			Enc_Knob.write(0);		//Reset Encoder value back to Zero
		}
		
		if (NewSpoolWtG>MaxSpoolWt) NewSpoolWtG=MaxSpoolWt;		//Range limit Number value
		if (NewSpoolWtG<MinSpoolWt) NewSpoolWtG=MinSpoolWt;		
		lcd.setCursor(0,2);lcd.print(F("New Spool Wt "));lcd_print_NxtArrow; lcd_print_3d(NewSpoolWtG); lcd.print (F(" g"));		
		if (Enc_PB_DownEdge==HIGH) {	//Is Oper Done?? Has Opr Pressed the button terminating the operation?
			Enc_PB_DownEdge=LOW;		//Reset edge detect
			lcd_clearRows(0,3);
			if(NewSpoolWtG!=int(SpoolWtG)){
				//           012345678901235456789
				lcd.setCursor(0,0);lcd.print(F("New Spool Wt  ")); lcd_print_3d(NewSpoolWtG); lcd.print (F(" g"));	
				lcd.setCursor(0,1);	lcd.print(F("ACCEPT this change?"));
				const uint8_t YN_row =3;			//This is the LCD row we want to use for the Y/N question
				if (Get_Opr_YN(Yes,YN_row)==Yes) {	//Stays in YN routine until Oper makes a selection
					SpoolWtG=float(NewSpoolWtG);	//Update SpoolWtG only if change accepted by Operator
					lcd_clearRow(3);
					lcd.setCursor(0,3);	lcd.print(F("  Change Accepted!"));
					delay(1000);
					//SaveAllToEEPROM(GBL_AutoSaveFlag);
				}else{
					lcd_clearRows(0,4);
					lcd.setCursor(0,1);	lcd.print(F("  Change Cancelled "));lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
					delay(2000);
				}
			} else {
				lcd_clearRows(0,4);
				lcd.setCursor(0,1);	lcd.print(F("    Spool Weight"));lcd.setCursor(0,2);lcd.print(F("     Unchanged!"));
				delay(3000);
			}
			Done=1;
		}
	}
		
	lcd_clearRows(0,3);
	GBL_menuNum=0;	//Return to main level	lcd_clearRows(0,3);	
	return SpoolWtG;	//Return NewSpoolWt -OR- Orig SpoolWG if unchanged or changes were rejected)
}
float SetLdCellFilter(float Filter){
	// Routine to enter a new filament diameter value
	//
	//	INPUTS		Filter	Current (starting) value of Filter
	//
	//	RETURNS		Updated value Filter variable.
	//
	//	20190501	E.Andrews	Rev 0	Initial cut
	//		
	float New_Filter =  Filter;	//Initialize New value to Current Value
	const float knobScaleFactor = 100.;
	int knobFilter=int(New_Filter*knobScaleFactor);	//Make an integer 'knobDens' value to get oper input
	const float MaxFilter=.99;	//Set Upper Limit (.99 = Max Filter)
	const float MinFilter=0.0;		//Set Lower Limit (0.0 = No Filter)
	Enc_Knob.write(0);				//Reset Encoder value back to Zero
	lcd_clearRows(0,4);
	lcd.setCursor(0,0);
	//           					 012345678901235456789
	lcd.setCursor(0,0);	lcd.print(F(" Set value with knob."));
	lcd.setCursor(0,1); lcd.print(F(" Push knob when done.")); 
	lcd.setCursor(0,2);lcd.print(F("New Filter = "));lcd_print_NxtArrow; lcd_printFloat(New_Filter,2); 

	uint8_t Done=0;
	while (Done==0){
		Read_Enc_Push_Button();							//Read Encoder Push Button
		if((Enc_Knob.read()>1||Enc_Knob.read()<-1) && Enc_PB_DownEdge==LOW){
			knobFilter=knobFilter+Enc_Knob.read();
			New_Filter=float(knobFilter)/knobScaleFactor;
			Enc_Knob.write(0);		//Reset Encoder value back to Zero
		}
		if (New_Filter>MaxFilter) New_Filter=MaxFilter;		//Range limit Number value
		if (New_Filter<MinFilter) New_Filter=MinFilter;	
		knobFilter=int(New_Filter*knobScaleFactor);		
		
		lcd.setCursor(0,2);lcd.print(F("New Filter = "));lcd_print_NxtArrow; lcd_printFloat(New_Filter,2);
		if (Enc_PB_DownEdge==HIGH) {	//Is Oper Done?? Has Opr Pressed the button terminating the operation?
			Enc_PB_DownEdge=LOW;		//Reset edge detect
			lcd_clearRows(0,3);
			if(New_Filter!=Filter){
				//           012345678901235456789
				lcd.setCursor(0,0);lcd.print(F("New Filter = "));lcd_print_NxtArrow; lcd_printFloat(New_Filter,2);
				lcd.setCursor(0,1);	lcd.print(F("ACCEPT this change?"));
				const uint8_t YN_row =3;			//This is the LCD row we want to use for the Y/N question
				if (Get_Opr_YN(Yes,YN_row)==Yes) {	//Stays in YN routine until Oper makes a selection
					Filter=New_Filter;
					lcd_clearRow(3);
					lcd.setCursor(0,3);	lcd.print(F("  Change Accepted!"));
					delay(1000);
					//SaveAllToEEPROM(GBL_AutoSaveFlag);
				}else{
					lcd_clearRows(0,4);
					lcd.setCursor(0,1);	lcd.print(F("  Change Cancelled "));lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
				}
				delay(2000);
			} else {
				lcd_clearRows(0,4);
				lcd.setCursor(0,1);	lcd.print(F(" LdCell_Filter Value"));lcd.setCursor(0,2);lcd.print(F("     Unchanged!"));
				delay(3000);
			}
			Done=1;
		}
	}
		
	lcd_clearRows(0,3);
	GBL_menuNum=0;	//Return to main level	lcd_clearRows(0,3);	
	return Filter;	//Return Changed Diameter
}
float SetFilamentDiam(float Diam_mm){
	// Routine to enter a new filament diameter value
	//
	//	INPUTS		Diam_mm	Current (starting) value of Diam_mm
	//
	//	RETURNS		Updated value DiamMm variable.
	//
	//	20190415	E.Andrews	Rev 0	Initial cut
	//	20190418	E.Andrews	Rev 0.1	Add integer treatment and know scaling on knob to improve control precision
	//		
	float New_Diam_mm =  Diam_mm;	//Initialize New value to Current Value
	const float knobScaleFactor = 256.;
	int knobDia=int(New_Diam_mm*knobScaleFactor);	//Make an integer 'knobDens' value to get oper input
	const float MaxDiam_mm=5.0;		//Set Upper Limit
	const float MinDiam_mm=1.0;		//Set Lower Limit
	Enc_Knob.write(0);		//Reset Encoder value back to Zero
	lcd_clearRows(0,4);
	lcd.setCursor(0,0);
	//           					 012345678901235456789
	lcd.setCursor(0,0);	lcd.print(F(" Set value with knob."));
	lcd.setCursor(0,1); lcd.print(F(" Push knob when done.")); 
	lcd.setCursor(0,2);lcd.print(F("New Fil Dia. "));lcd_print_NxtArrow; lcd_printFloat(New_Diam_mm,2); lcd_print_mm;

	uint8_t Done=0;
	while (Done==0){
		Read_Enc_Push_Button();							//Read Encoder Push Button
		if((Enc_Knob.read()>1||Enc_Knob.read()<-1) && Enc_PB_DownEdge==LOW){
			knobDia=knobDia+Enc_Knob.read();
			New_Diam_mm=float(knobDia)/knobScaleFactor;//			New_Diam_mm=New_Diam_mm + (float(Enc_Knob.read())/200.);
			Enc_Knob.write(0);		//Reset Encoder value back to Zero
		}
		if (New_Diam_mm>MaxDiam_mm) New_Diam_mm=MaxDiam_mm;		//Range limit Number value
		if (New_Diam_mm<MinDiam_mm) New_Diam_mm=MinDiam_mm;	
		knobDia=int(New_Diam_mm*knobScaleFactor);		
		
		lcd.setCursor(0,2);lcd.print(F("New Fil Dia. "));lcd_print_NxtArrow; lcd_printFloat(New_Diam_mm,2); lcd_print_mm;		
		if (Enc_PB_DownEdge==HIGH) {	//Is Oper Done?? Has Opr Pressed the button terminating the operation?
			Enc_PB_DownEdge=LOW;		//Reset edge detect
			lcd_clearRows(0,3);
			if(New_Diam_mm!=Diam_mm){
				//           012345678901235456789
				lcd.setCursor(0,0);lcd.print(F("Old Fil Dia: "));lcd_printFloat(Diam_mm,2); lcd_print_mm;
				lcd.setCursor(0,1);lcd.print(F("New Fil Dia: "));lcd_printFloat(New_Diam_mm,2); lcd_print_mm;
				lcd.setCursor(0,2);	lcd.print(F("ACCEPT this change?"));
				const uint8_t YN_row =3;			//This is the LCD row we want to use for the Y/N question
				if (Get_Opr_YN(Yes,YN_row)==Yes) {	//Stays in YN routine until Oper makes a selection
					Diam_mm=New_Diam_mm;
					lcd_clearRow(3);
					lcd.setCursor(0,3);	lcd.print(F("  Change Accepted!"));
					delay(1000);
					//SaveAllToEEPROM(GBL_AutoSaveFlag);
				}else{
					lcd_clearRows(0,4);
					lcd.setCursor(0,1);	lcd.print(F("  Change Cancelled "));lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
				}
				delay(2000);
			} else {
				lcd_clearRows(0,4);
				lcd.setCursor(0,1);	lcd.print(F(" Filament Diameter"));lcd.setCursor(0,2);lcd.print(F("     Unchanged!"));
				delay(3000);
			}
			Done=1;
		}
	}
		
	lcd_clearRows(0,3);
	GBL_menuNum=0;	//Return to main level	lcd_clearRows(0,3);	
	return Diam_mm;	//Return Changed Diameter
}
float SetFilamentDens(float Dens){
	// Routine to enter a new filament density value
	//
	//	INPUTS		Dens	Current (starting) value of Dens
	//
	//	RETURNS		Updated value Dens variable.
	//
	//	20190415	E.Andrews	Rev 0	Initial cut
	//	20190418	E.Andrews	Rev 0.1	Add integer treatment and know scaling on knob to improve control precision
	//		
	float New_Dens =  Dens;	//Initialize New value to Current Value
	const float knobScaleFactor = 256.;
	int knobDens=int(New_Dens*knobScaleFactor);	//Make an integer 'knobDens' value to get oper input
	const float MaxDens=10.0;		//Set Upper Limit
	const float MinDens=.20;		//Set Lower Limit
	Enc_Knob.write(0);		//Reset Encoder value back to Zero
	lcd_clearRows(0,4);
	lcd.setCursor(0,0);
	//           012345678901235456789
	lcd.setCursor(0,0);	lcd.print(F("Set value with knob."));
	lcd.setCursor(0,1); lcd.print(F("Push knob when done.")); 
	lcd.setCursor(0,2);lcd.print(F("New Fil Dens"));lcd_print_NxtArrow; lcd_printFloat(New_Dens,2); lcd_print_G_Per_CM3;

	uint8_t Done=0;
	while (Done==0){
		Read_Enc_Push_Button();							//Read Encoder Push Button
		if((Enc_Knob.read()>1||Enc_Knob.read()<-1) && Enc_PB_DownEdge==LOW){
			knobDens=knobDens+Enc_Knob.read();
			New_Dens=float(knobDens)/knobScaleFactor;	//New_Dens + (float(Enc_Knob.read())/200.);
			Enc_Knob.write(0);		//Reset Encoder value back to Zero
		}
		
		if (New_Dens>MaxDens) New_Dens=MaxDens;		//Range limit Number value
		if (New_Dens<MinDens) New_Dens=MinDens;	
		knobDens=int(New_Dens*knobScaleFactor);		//Update 'knob version' of range limited value	
		lcd.setCursor(0,2);lcd.print(F("New Fil Dens"));lcd_print_NxtArrow; lcd_printFloat(New_Dens,2); lcd_print_G_Per_CM3;		
		if (Enc_PB_DownEdge==HIGH) {	//Is Oper Done?? Has Opr Pressed the button terminating the operation?
			Enc_PB_DownEdge=LOW;		//Reset edge detect
			lcd_clearRows(0,3);
			if(New_Dens!=Dens){
				//           012345678901235456789
				lcd.setCursor(0,0);lcd.print(F("Old Fil Dens: "));lcd_printFloat(Dens,2); lcd_print_G_Per_CM3;
				lcd.setCursor(0,1);lcd.print(F("New Fil Dens: "));lcd_printFloat(New_Dens,2); lcd_print_G_Per_CM3;
				lcd.setCursor(0,2);	lcd.print(F("ACCEPT this change?"));
				const uint8_t YN_row =3;			//This is the LCD row we want to use for the Y/N question
				if (Get_Opr_YN(Yes,YN_row)==Yes) {	//Stays in YN routine until Oper makes a selection
					Dens=New_Dens;
					lcd_clearRow(3);
					lcd.setCursor(0,3);	lcd.print(F("  Change Accepted!"));
					delay(1000);
					//SaveAllToEEPROM(GBL_AutoSaveFlag);
				}else{
					lcd_clearRows(0,4);
					lcd.setCursor(0,1);	lcd.print(F("  Change Cancelled "));lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
				}
				delay(2000);
			} else {
				lcd_clearRows(0,4);
				lcd.setCursor(0,1);	lcd.print(F(" Filament Density"));lcd.setCursor(0,2);lcd.print(F("     Unchanged!"));
				delay(2000);
			}
			Done=1;
		}
	}
		
	lcd_clearRows(0,3);
	GBL_menuNum=0;	//Return to main level	lcd_clearRows(0,3);	
	return Dens;	//Return Changed Diameter
}
void SetFilamentName(char *name){
	// Routine to enter a new filament diameter value
	//
	//	INPUTS		
	//		*name	Pointer to 'name' string to be updated/changed
	//				Routine assumes string is at most 4 characters + null (5 total!)
	//
	//	RETURNS		
	//				Modifies/returns any changes made by Oper back by reference.
	//
	//	20190415	E.Andrews	Rev 0	Initial cut
	//		
	char NewName[5];
	//#define SetFilament_DEBUG
	for(uint8_t i=0;i<5;i++){
		NewName[i]=name[i];
	}
	//strcpy(NewName,name);	//Initialize NewName with current 'name' value	
	#ifdef SetFilament_DEBUG
		Serial.print(F(" SetFilName, INPUT: "));Serial.print(NewName); Serial.print(F(" Size="));Serial.println (sizeof(NewName));
		for (uint8_t i;i<sizeof(NewName);i++){
			Serial.print(i);Serial.print(" ");Serial.print (NewName[i]);Serial.print(" ");Serial.print (NewName[i],DEC);Serial.print(" ");Serial.println (NewName[i],HEX);
		}
		Serial.println();
	#endif
	
	uint8_t	MaxCh=0x7f;		//Set Upper Limit
	uint8_t MinCh=0x20;		//Set Lower Limit
	Enc_Knob.write(0);		//Reset Encoder value back to Zero
	lcd_clearRows(0,4);
	lcd.setCursor(0,0);
	//           012345678901235456789
	lcd.setCursor(0,0);	lcd.print(F("TURN to chang letter"));
	lcd.setCursor(0,1); lcd.print(F("PUSH for next letter"));
	lcd.setCursor(0,2); lcd.print(F("Filament Name "));lcd_print_NxtArrow;
	uint8_t Xptr=15;
	for (uint8_t i=0;i<4;i++){
		lcd.setCursor(Xptr+i,2);
		lcd.print(NewName[i]);
	}
	
	lcd.setCursor(Xptr,3);lcd_print_UpArrow;
	ResetEncPushButton;
	Enc_Knob.write(0);		//Reset Knob
	uint8_t i=0;
	while (i<4) {
		lcd.setCursor(Xptr+i,2);
		lcd.print(NewName[i]);
		Read_Enc_Push_Button();
		if(Enc_PB_DownEdge==HIGH){
			Enc_PB_DownEdge=LOW;		//Reset PB edge
			lcd.setCursor(Xptr+i,3);	//Move on to next character
			lcd.print(F(" "));			//Erase last arrow
			lcd_print_UpArrow;			//Move Arrow over one place
			i++;
		}
		int KnobChange=Enc_Knob.read()/3;
		if (KnobChange!=0){
			Enc_Knob.write(0);						//Reset Knob
			NewName[i]=NewName[i]+KnobChange;
			if (NewName[i]>MaxCh)NewName[i]=MaxCh;	//Range limit
			if (NewName[i]<MinCh)NewName[i]=MinCh;	//Range Limit
			lcd.setCursor(Xptr+i,2);				//Update New Char
			lcd.print(NewName[i]);
		}
	}
	//Make sure null char is present at end of array
	NewName[4]=0;
	
	if(NewName!=name){	//Was name actually changed?
		#ifdef SetFilament_DEBUG
			Serial.print (F("OLD_NAME |"));Serial.print(name);Serial.println("|");
			Serial.print (F("NEW_NAME |"));Serial.print(NewName);Serial.println("| (I think its different...)");
		#endif
		lcd_clearRows(0,4);
		lcd.setCursor(0,0);lcd.print(F("Old Fil Name: ")); lcd.print(name);
		lcd.setCursor(0,1);lcd.print(F("New Fil Name: ")); lcd.print(NewName);
		lcd.setCursor(0,2);	lcd.print(F("ACCEPT this change?"));
		const uint8_t YN_row =3;			//This is the LCD row we want to use for the Y/N question
		if (Get_Opr_YN(Yes,YN_row)==Yes) {	//Stays in YN routine until Oper makes a selection
			strcpy(name,NewName);	//Initialize NewName with current 'name' value
			lcd_clearRow(3);
			lcd.setCursor(0,3);	lcd.print(F("  Change Accepted!"));
			delay(2000);
		}else{
			lcd_clearRows(0,4);
			lcd.setCursor(0,1);	lcd.print(F("  Change Cancelled "));lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
			delay(3000);
		}
	} else {
		lcd_clearRows(0,4);
		lcd.setCursor(0,1);	lcd.print(F("   Filament Name  "));lcd.setCursor(0,2);lcd.print(F("     Unchanged!"));
		delay(3000);
	}
	lcd_clearRows(0,3);
	GBL_menuNum=0;	//Return to main level	lcd_clearRows(0,3);	

}	

void lcd_printFilamentText_1Line(uint8_t Fnum=1,uint8_t RowNum=0){
	//	Routine to send Filament Name, Size, & Density to lcd.	Note: Caller must
	//	clear line where 'printing' is to start prior to calling this function.
	//	One line of LCD is used in the following format:	
	//			01234567890123456789
	//			nn.xxxx d.ddu e.eevv 
	//
	//		where 	nn=Filament Number, xxxx=Filament name, d.dd=Filament Diameter, u=mm, e.ee=Filament Density, vv=g/cm^3
	//
	//	INPUTS
	//	uint8_t	Fnum	Filament number.  Range: one to FnumMax.  Defaults to 1.
	//
	//	OUTPUTS	None
	//
	//	20190413	E.Andrews	Rev 0	Initial cut
	//
	char TypeName[5];
	//Retrieve data from Filament Data Base structure & load into working variables
	strcpy(TypeName,FilType_ACTIVE[Fnum-1].Name);	//Get Filament shortcut name (just 3 or 4 characters), Diam (mm) and Dens (g/cm^3)
	float Diam_mm = FilType_ACTIVE[Fnum-1].Dia;
	float Dens_gcm3=FilType_ACTIVE[Fnum-1].Dens;
	lcd.setCursor(0,RowNum);
	switch (Fnum){
		case 1 ... 9:			//Single digit Filament Number
			lcd.print(Fnum);lcd.print("."),lcd.print(TypeName);
			lcd.print(" ");lcd_printFloat(Diam_mm,2); lcd_print_mm; lcd.print(" "); lcd_printFloat(Dens_gcm3,2);lcd_print_G_Per_CM3;
		break;
		case 10 ... FnumMax:	//Two digit Filament Number...
			lcd.print(Fnum);lcd.print("."),lcd.print(TypeName);
			lcd_printFloat(Diam_mm,2); lcd_print_mm; lcd.print(" "); lcd_printFloat(Dens_gcm3,2);lcd_print_G_Per_CM3;
		break;
		default:
			lcd.print (F("ERR:Fil#=")); lcd.print(Fnum);lcd.print(F(" Unkwn"));
		break;
	}
}
void lcd_printFilamentText_2Line(uint8_t Fnum=1,uint8_t RowNum=0){
	//	Routine to send Filament Name, Mfg, Color, Size, & Density to lcd.	Note: Caller must
	//	clear lines and specify RowNum where 'printing' is to begin.  
	//	Two lines of LCD are used in the following format:	
	//			01234567890123456789
	//			nn.xxxxxx yyyyy cccc
	//			  d.dd    e.ee 
	//		where 	nn=Filament Number, xxxxxx=Filament name, yyyyy=Manufacturer, cccc=Color
	//				d.dd=Filament Diameter_mm, e.ee=Filament Density_g/cm^3
	//
	//
	//	INPUTS
	//	uint8_t	Fnum	Filament number.  Range: one to FnumMax; default=1.
	//	uint9_t RowNum	Row number, Range: 0,1,2,3, specifies line to which printing will start; default=0.
	//			Draws out data from FilType_ACTIVE[Fnum-1] data base.
	//
	//	OUTPUTS	None
	//
	//	20190502	E.Andrews	Rev 0	Initial cut of two line version of display
	//
	char TypeName[5];
	//Retrieve data from Filament Data Base structure & load into working variables
	strcpy(TypeName,FilType_ACTIVE[Fnum-1].Name);	//Get Filament shortcut name (just 3 or 4 characters), Diam (mm) and Dens (g/cm^3)
	float Diam_mm = FilType_ACTIVE[Fnum-1].Dia;
	float Dens_gcm3=FilType_ACTIVE[Fnum-1].Dens;
	lcd.setCursor(0,RowNum);
	switch (Fnum){
		case 1 ... 9:			//Single digit Filament Number
			lcd.print(Fnum);lcd.print("."),lcd.print(TypeName);
			lcd.print(" ");lcd_printFloat(Diam_mm,2); lcd_print_mm; lcd.print(" "); lcd_printFloat(Dens_gcm3,2);lcd_print_G_Per_CM3;
		break;
		case 10 ... FnumMax:	//Two digit Filament Number...
			lcd.print(Fnum);lcd.print("."),lcd.print(TypeName);
			lcd_printFloat(Diam_mm,2); lcd_print_mm; lcd.print(" "); lcd_printFloat(Dens_gcm3,2);lcd_print_G_Per_CM3;
		break;
		default:
			lcd.print (F("ERR:Fil#=")); lcd.print(Fnum);lcd.print(F(" Unkwn"));
		break;
	}
}
uint8_t SelectFilament(uint8_t NewFilNum=1){
	//	Routine to select new Filament Name, Size, & Density from available Data Base.
	//
	//	INPUTS
	//	uint8_t	OldFilNum	Current Filament number.  Range: one to FnumMax.  Defaults to 1.
	//
	//	OUTPUTS
	//	uint8_t	NewFilNum	Operator set value.  
	//
	//	20190413	E.Andrews	Rev 0	Initial cut
	//
	uint8_t StartFilNum=NewFilNum;	//Remember incoming value in case we get an inactivity-time-out...
	Enc_Knob.write(0);		//Reset Knob Rotary Control value
	//Enc_PB_DownEdge=LOW;	//Reset Knob Push Button
	ResetEncPushButton();
	lcd_clearRows(0,3);	//Clear Screen
	lcd.setCursor(0,0);		//Print Operator Instructions to LCD
	lcd.print(F("  Turn & Push Knob"));
	lcd.setCursor(0,1);
	lcd.print (F(" to select Filament"));
	lcd.setCursor(0,3);
	lcd_printFilamentText_1Line(NewFilNum,3);

	uint8_t Done=No;	//Initialize 'Loop-Done-Flag' to 'No'
	//delay (1000);
	while (Done==No){	//Here's operator input loop...
		Read_Enc_Push_Button();
		if((Enc_Knob.read()>3||Enc_Knob.read()<-3) && Enc_PB_DownEdge==LOW){
			if (Enc_Knob.read()>0) NewFilNum++;	else NewFilNum--;
			Enc_Knob.write(0);		//Reset Encoder value back to Zero
			if (NewFilNum>FnumMax) NewFilNum=FnumMax;		//Range limit Filament Number value (must be confined to 1 & 16)
			if (NewFilNum<1) NewFilNum=1;
			lcd_clearRow(3);
			lcd_printFilamentText_1Line(NewFilNum,3);
		}
		if (Enc_PB_DownEdge==HIGH) {
			Enc_PB_DownEdge=LOW;	//Reset PB Press edge detector
			Done=Yes;
		}
	}
	if (NewFilNum!=StartFilNum) {
		SaveAllToEEPROM(GBL_AutoSaveFlag);
	}
	return NewFilNum;
}
float CalibrateScale(float Cur_ScaleFactor){
	// Routine to perform the scale GAIN calibration function.
	//	The operator is asked to accept or reject the new
	//	calibration factor; if accepted, the new calibration factor
	//	is stored into the CalCfg_ACTIVE.LdCell_Scale variable.
	//	Operator can accept/reject saving the CalCfg_ACTIVE to EEPROM.
	//
	//	INPUTS	
	//		float Cur_ScaleFactor	The currently active scale factor
	//
	//	RETURNS	
	//		float New_ScaleFactor	A new scale factor
	//
	//	20190410	E.Andrews	Rev 0	Initial cut
	//	20190730	E.Andrews	Rev 0.1	Minor documentation cleanup
	//	
	//#define CalScale_DEBUG
	lcd_clearRows(0,3);
	lcd.setCursor(0,0);
	//           012345678901235456789
	lcd.print(F("Do you want to ZERO"));
	lcd.setCursor(0,1);
	lcd.print(F("Scale? (recommended)"));
	if (Get_Opr_YN(No,3)==Yes) ZeroScale();
	
	//Go forward with calibration....
	lcd_clearRows(0,4);
	lcd.print(F("1.Turn knob to enter"));
	lcd.setCursor(0,1);
	lcd.print(F(" known Test-Load Wt."));
	lcd.setCursor(0,2);
	lcd.print(F("2.Push=Accept entry."));
	int knobTestWtG=500;
	float TestWtKg;
	const float knobScaleFactor=1000.;
	uint8_t Done=0;
	const float MaxTestKg=2.0;
	const float MinTestKg=.25;
	float NewLdCell_Scale=Cur_ScaleFactor;
	while (Done==0){
		Read_Enc_Push_Button();							//Read Encoder Push Button
		int ER=Enc_Knob.read();
		if((ER>2||ER<-2) && Enc_PB_DownEdge==LOW){
			knobTestWtG=knobTestWtG+ER/2;

			Enc_Knob.write(0);		//Reset Encoder value back to Zero
		}
		TestWtKg=float(knobTestWtG)/knobScaleFactor;	//Create TestWtKg from knobTestWtG value
		if (TestWtKg>MaxTestKg) TestWtKg=MaxTestKg;		//Range limit Number value
		if (TestWtKg<MinTestKg) TestWtKg=MinTestKg;	

		knobTestWtG=int(TestWtKg*knobScaleFactor);		//Update 'knob version' of range limited value	
		//Show test value entry to operator...
		lcd.setCursor(0,3);
		lcd.print(F(" Test Wt "));lcd_print_NxtArrow
		lcd_printFloat(TestWtKg,3);
		lcd.print(F(" Kg"));
		
		if (Enc_PB_DownEdge==HIGH) {	//Is Oper Done?? Has Opr Pressed the button terminating the operation?
			Enc_PB_DownEdge=LOW;		//Reset edge detect
			lcd_clearRows(0,3);
			if(true==true){
				lcd.setCursor(0,0);
				lcd.print(F("Wt. set to "));
				lcd_printFloat(TestWtKg,3);
				lcd.print(F(" Kg"));
				//           012345678901235456789
				lcd.setCursor(0,1);	lcd.print(F("ACCEPT & proceed?"));
				const uint8_t YN_row =3;			//This is the LCD row we want to use for the Y/N question
				if (Get_Opr_YN(Yes,YN_row)==Yes) {	//Stays in YN routine until Oper makes a selection
					//calc new calib factor here!!! 
					lcd_clearRows(0,4);
					scale.set_scale();
					float RawData = scale.get_units(10);
					NewLdCell_Scale = RawData/TestWtKg;
					#ifdef SetFilament_DEBUG
						Serial.print(F("  get_units() = "));Serial.println(RawData,4);
						Serial.print(F("     TestWtKg = "));Serial.println(TestWtKg,4);
						Serial.print(F("CurCal Factor = "));Serial.println(Cur_ScaleFactor,4);
						Serial.print(F("ACTIVE Factor = "));Serial.println(CalCfg_ACTIVE.LdCell_Scale,4);
						Serial.print(F("NewCal Factor = "));Serial.println(NewLdCell_Scale,4);
					#endif
					lcd.setCursor(0,0); lcd.print(F("CurCal: "));lcd.print (Cur_ScaleFactor,4);			
					lcd.setCursor(0,1); lcd.print(F("NewCal: "));lcd.print (NewLdCell_Scale,4);
					lcd.setCursor(0,2);	lcd.print(F("Accept this change?"));
					if (Get_Opr_YN(Yes,YN_row)==Yes) {
						//
						//CalCfg_ACTIVE.LdCell_Scale=NewLdCell_Scale;
						lcd_clearRows(0,4);
						lcd.setCursor(0,1); lcd.print(F(" New Cal. Accepted "));
						lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
						delay(2000);
						//SaveAllToEEPROM(GBL_AutoSaveFlag);
						
					}else{
						NewLdCell_Scale=Cur_ScaleFactor;	//Reset to current factor
						lcd_clearRows(0,4);
						lcd.setCursor(0,1);	lcd.print(F("Calibration Rejected"));
						lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
						delay(2000);
					}
				}else{
					//Oper didn't really want to calibrate the scale...
					NewLdCell_Scale=Cur_ScaleFactor;	//Reset to current factor
					lcd_clearRows(0,4);
					lcd.setCursor(0,1);	lcd.print(F("Calibration CANCELED"));lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
					delay(2000);
				}
				
			} else {
				lcd_clearRows(0,4);
				lcd.setCursor(0,1);	lcd.print(F(" Scale Calibration"));lcd.setCursor(0,2);lcd.print(F("     Unchanged!"));
				delay(2000);
			}
			Done=1;
			
		}
		
	}

	//scale.set_scale(CalCfg_ACTIVE.LdCell_Scale);
	//return CalCfg_ACTIVE.LdCell_Scale;
	return NewLdCell_Scale;
}
uint8_t	Get_Opr_Cncl_Accept(uint8_t OpResponse, uint8_t lcdRow){
	// Routine to Display & Retrieve CANCEL or ACCEPT Oper response using LCD & Rotary Encoder
	//
	//	INPUTS	
	//	uint8_t	OpResponse	Initial startup (aka default) return value
	//						valid values are Yes = 1, No = 0
	//	uint8_t	lcdRow		Row to use for CANCEL/ACCEPT display.  Routine will use the WHOLE ROW!
	//
	//	RETURNS
	//		Yes/No			These values should be declared as global constants
	//						const uint8_t Cancel = 0;	//Cancel
	//						const uint8_t Accept = 1;	//Accept
	//
	//	20190410	E.Andrews	Rev 0	Initial cut
	//	

	lcd_clearRow(lcdRow);
	lcd.setCursor(0,lcdRow);
	//		     01234567890123456789
	lcd.print(F("   CANCEL    ACCEPT "));
	//lcd.print(F("    YES      NO     "));
	uint8_t Done = No;
	Enc_Knob.write(0);
	//Enter into a loop looking for oper input
	while (Done==No){
		if (OpResponse==Cancel) {
			lcd.setCursor(2,lcdRow); lcd_print_NxtArrow;
			lcd.setCursor(12,lcdRow); lcd.print(F(" "));
		} else {
			lcd.setCursor(12,lcdRow); lcd_print_NxtArrow;
			lcd.setCursor(2,lcdRow); lcd.print(F(" "));	
		}
			
		Read_Enc_Push_Button();
		if (Enc_PB_DownEdge==HIGH) {	//Did Opr push button to lock in answer?
			Done=Yes;
			Enc_PB_DownEdge=LOW;	//Reset PB
		}
		if(Enc_Knob.read()>3 || Enc_Knob.read()<-3){	//Did Oper Turn Knob enought? if YES, toggle Cancel<->Accept response
			if (OpResponse==Cancel)  OpResponse=Accept; else OpResponse=Cancel;
			Enc_Knob.write(0);
		}
	}	
	return OpResponse;
}
uint8_t	Get_Opr_YN(uint8_t OpResponse, uint8_t lcdRow){
	// Routine to Display & Retrieve YES or NO Oper response using LCD & Rotary Encoder
	//
	//	INPUTS	
	//	uint8_t	OpResponse	Initial startup (aka default) return value
	//						valid values are Yes = 1, No = 0
	//	uint8_t	lcdRow		Row to use for YES NO display.  Routine will use the WHOLE ROW!
	//
	//	RETURNS
	//		Yes/No			These values should be declared as global constants
	//						const uint8_t Yes = 1;	//Yes
	//						const uint8_t No = 0;	//No
	//
	//	20190410	E.Andrews	Rev 0	Initial cut
	//	

	lcd_clearRow(lcdRow);
	lcd.setCursor(0,lcdRow);
	lcd.print(F("    YES      NO     "));
	uint8_t Done = No;
	Enc_Knob.write(0);
	//Enter into a loop looking for oper input
	while (Done==No){
		//		     01234567890123456789
		//lcd.print(F("    YES      NO     "));
		if (OpResponse==Yes) {
			lcd.setCursor(3,lcdRow); lcd_print_NxtArrow;
			lcd.setCursor(12,lcdRow); lcd.print(F(" "));
		} else {
			lcd.setCursor(12,lcdRow); lcd_print_NxtArrow;
			lcd.setCursor(3,lcdRow); lcd.print(F(" "));	
		}
			
		Read_Enc_Push_Button();
		if (Enc_PB_DownEdge==HIGH) {	//Did Oper push the knob?
			Done=Yes;
			Enc_PB_DownEdge=LOW;	//Reset PB
		}
		if(Enc_Knob.read()>3 || Enc_Knob.read()<-3){	//If Oper turned the knob enought, toggle Y-N response
			if (OpResponse==Yes)  OpResponse=No; else OpResponse=Yes;
			Enc_Knob.write(0);	//Reset rotary knob
		}
	}	

	return OpResponse;
}
//==== END MENU & RELATED HELPER ROUTINES ==========================================


	
float Kg_to_Length(float WtKg,float DiamMm, float Density, uint8_t Units=0){
	// Routine to convert Weight to Length using the following formula
	//	Length = Weight x Area x Density x ScaleFactor
	//	Length = Weight x (PI x R^2) x Density x ScaleFactor
	//  A "ScaleFactor" is also applied so that m, cm, or mm values can be returned.
	//	Note: Returns 0.0  if either DiamMm or Density are Zero.
	//
	//INPUTS
	//	float	Kg		Weight of Just Filament (units: Kg) 
	//					Note: Wt of SPOOL must be subracted from weight measured BEFORE calling this routine!
	//	float	DiamMm	Filament Diameter (units: mm)
	//	float	Density	Filament density (units: g/cm^3) Get this value from Filament Supplier or use generic value from WEB
	//	uint8_t	Units	0=Return value in Meters (m)
	//					1=Return value in Centimeters (cm)
	//					2= Return in Millimeters (mm)
	//					Note, Will return Meters if Units value is not set to 1 or 2!
	//
	//RETURNS
	//	float	Length	Estimated length of filament in units as requested bu "Units" (m,cm,mm)
	//
	//	20190331	E.Andrews	Rev 0	Initial release
	//
	const float Scale_MM = 10;
	const float Scale_CM = 1;
	const float Scale_M = 1/100.;
	//Set appropriate Scale Factor
	float ScaleFactor = Scale_M;			//Return in Meters (default)
	if (Units == 1) ScaleFactor = Scale_CM;	//return in Centimeters
	if (Units == 2) ScaleFactor = Scale_MM;	//return in Millimeters
	if (DiamMm==0 || Density==0) {	//Make sure that DiamMm and Density values are not zero 
		return 0.0;		//No coversion possible; return a length as '0'.
	}else {
		return   ScaleFactor*(WtKg * 1000.)/(PI * pow((DiamMm/20.0),2.)*Density);
	}
}

void lcd_print_WtKg(uint8_t row){
	
	//Refresh Filament Type Text
	lcd.setCursor(0,0);
	lcd_printFilamentText_1Line(Fnum);
	
	//Measure Wt and Display measurements
	NewWtKg=scale.get_units(1);	//Get a new weight reading
	RawWtKg=CalCfg_ACTIVE.LdCell_Filter*RawWtKg + (1.0-CalCfg_ACTIVE.LdCell_Filter) * NewWtKg;	//Create a Running average filter
	RawWtKg = NewWtKg;
	WtKg=RawWtKg-(CalCfg_ACTIVE.Last_SpoolWtG/1000.);
	if (WtKg<0)WtKg=0.;	//Range limit so we don't show NEGATIVE Weights...
	//Display current weight reading in Row 2
	lcd.setCursor(0,row);
	lcd.print(F(" New Wt "));lcd_print_NxtArrow; lcd_printFloat(WtKg,3);lcd.print(F("Kg"));	

}
void MeasurePrint_FilNam_Wt_Length(uint8_t LenUnits){
	// Routine to take new reading and display wt and len to LCD display
	//
	//	INPUTS	
	//	uint8_t	LenUnits 	1=cm, 2=mm, any other value = m
	//
	//	RETURNS		Changes currently active SpoolWtG variable.
	//
	//	20190410	E.Andrews	Rev 0		Initial cut
	//	20190421	E.Andrews	Rev 1		Changed format to show Tot Wt, Spl Wt, Fil Wt, Fil Len
	//	20190501	E.Andrews	Rev 1.01	Changed  Running Avg Filter to use CalCfg_ACTIVE.LdCell_Filter variable.
	//		
	//Refresh Filament Type Text
	lcd.setCursor(0,0);
	lcd_printFilamentText_1Line(Fnum);
	
	//Measure Wt and Display measurements
	NewWtKg=scale.get_units(1);	//Get a new weight reading
	//RawWtKg=.9*RawWtKg + .1 * NewWtKg;	//Create a Running average filter [OLD VERSION, Fixed 10% filter]
	RawWtKg=CalCfg_ACTIVE.LdCell_Filter*RawWtKg + (1.0-CalCfg_ACTIVE.LdCell_Filter)* NewWtKg;	//Create a Running average filter
	//RawWtKg = NewWtKg;
	WtKg=RawWtKg-(CalCfg_ACTIVE.Last_SpoolWtG/1000.);
	if (WtKg<0)WtKg=0.;	//Range limit so we don't show NEGATIVE Weights...
	//Display current weight reading in Row 2
	lcd.setCursor(0,1);
	//lcd.print(F("Tot:")); lcd_printFloat(RawWtKg*1000.,0);
	lcd.print(F("Tot:")); lcd_printInteger(int(RawWtKg*1000.),4,' ');
	lcd.setCursor(8,1);
	lcd.print(F("g Spl:")); lcd_printFloat(CalCfg_ACTIVE.Last_SpoolWtG,0);lcd.print("g ");
	lcd.setCursor(0,2);
	//lcd.print(F("Fil:")); lcd_printFloat(WtKg*1000.,0);lcd.setCursor(8,2);lcd.print(F("g"));	
	lcd.print(F("Fil:")); lcd_print_4d(int(WtKg*1000.),' ');lcd.setCursor(8,2);lcd.print(F("g"));	
	//---Display new LENGTH readings
	lcd.setCursor(10,2);
	//lcd.print(F("Len:"));lcd_printFloat(Kg_to_Length(WtKg,Diam_mm,Dens_gcm3,LenUnits),1);
	//float Leng_m=Kg_to_Length(WtKg,Diam_mm,Dens_gcm3,LenUnits);
	float Leng_m=Kg_to_Length(WtKg,GBL_Diam_mm,GBL_Dens_gcm3,LenUnits);
	lcd.print(F("Len:"));lcd_print_3d(int(Leng_m),' ');lcd.print(".");lcd.print(int(Leng_m*10.-int(Leng_m)*10));
	lcd.setCursor(19,2);
	switch (LenUnits){
		case 1:
			lcd_print_cm;		//Print in cm
			lcd.print(F(""));
		break;
		case 2:
			lcd_print_mm;		//Print in mm
			lcd.print(F(""));
		break;
		default:
			lcd.print(F("m"));	//Print in Meters
			lcd.print(F(""));
	}
}
//=============== BEGIN CRC ROUTINES ==========================================================
/***
    Written by Christopher Andrews.
    CRC algorithm generated by pycrc, MIT licence ( https://github.com/tpircher/pycrc ).

	A CRC is a simple way of checking whether data has changed or become corrupted.
	This example calculates a CRC value directly on the EEPROM values.
	The purpose of this example is to highlight how the EEPROM object can be used just like an array.
***/


const unsigned long crc_table[16] = {
	0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
	0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
	0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
	0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};
unsigned long ComputeEEPROM_CRC(int startAdr, int Nbytes) {
	//CRC code ADAPTED FROM code originally Written by Christopher Andrews.
    //CRC algorithm generated by pycrc, MIT licence ( https://github.com/tpircher/pycrc ).
	//
	//	A CRC is a simple way of checking whether data has changed or become corrupted.
	//	This routine will compute the CRC on a block of EEPROM memory.
	//
	//	INPUTS
	//	int		startAdr	Starting address (in EEPROM) to begin CRC Calculation
	//	int		Nbytes		The total number of bytes to include in CRC calculation.
	//
	//	OUTPUTS
	//		Returns LONG that is a 32-bit  CRC for the specified EEPROM space.
	//
	//	20190419	E.Andrews	Rev 0	Initial release
	//	20190426	E.Andrews	Rev 1	Fix EEPROM addressing problem
	//

	unsigned long crc = ~0L;

	//for (int index = 0 ; index < Nbytes  ; ++index) {
	for (int index = startAdr ; index < Nbytes+startAdr  ; ++index) {
		crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
		crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
		crc = ~crc;
	}
	return crc;
}
unsigned long ComputeMEM_CRC(uint8_t* StartAdr, int Nbytes) {
	//CRC code ADAPTED FROM code originally Written by Christopher Andrews.
    //CRC algorithm generated by pycrc, MIT licence ( https://github.com/tpircher/pycrc ).
	//
	//	A CRC is a simple way of checking whether data has changed or become corrupted.
	//	This routine will compute the CRC on a block of EEPROM memory.
	//
	//	INPUTS
	//	int		startAdr	Starting address (in RAM) to begin CRC Calculation
	//	int		Nbytes		The total number of bytes to include in CRC calculation.
	//
	//	OUTPUTS
	//		Returns LONG that is a 32-bit  CRC for the specified RAM space.
	//
	//	20190419	E.Andrews	Rev 0	Initial release
	//
	//#define ComputeMemCRC_DEBUG
	uint8_t *AdrPtr;
	//uint8_t contents=0;
	unsigned long crc = ~0L;
	//AdrPtr= StartAdr;
	for (int index = 0 ; index < Nbytes  ; ++index) {
		AdrPtr=StartAdr+index;
		#ifdef ComputeMemCRC_DEBUG
			contents = *AdrPtr;
			Serial.print (index);Serial.print(" "); Serial.print((long)AdrPtr);Serial.print(" ");Serial.println( contents);
		#endif
		crc = crc_table[(crc ^ *AdrPtr) & 0x0f] ^ (crc >> 4);
		crc = crc_table[(crc ^ (*AdrPtr >> 4)) & 0x0f] ^ (crc >> 4);
		crc = ~crc;
		
	}
	return crc;
}
//=============== END CRC ROUTINEs ===========================================================

//=============== EEPROM ROUTINES ============================================================
uint8_t read_CalCfg_from_EEPROM(uint8_t CheckOnly=false) {
	//  This routine will read CalCfg data structure from EEPROM.  
	//	1. 	Check EEPROM CRC - If doesn't match, return UNSUCCESSFUL
	//	2. 	If CRC OKAY, then Check DB Configuration Code
	//		Does it Match with this version of CODE?  If doesn't match, return = UNSUCCESSFUL
	//	3.	If step 1 & Step 2 OKAY, Read the CalCfg from EEPROM and load it into
	//		the CalCfg_ACTIVE data structure.  Then return = SUCCESS.
	//
	//	INPUTS	
	//		uint8_t	CheckOnly	Optional variable; if set TRUE the CRC and REV_CODE will be validated by 
	//							BUT actual CalCfg_ACTIVE will NOT BE updated with EEPROM data!
	//							If omitted, defaults to FALSE and CalCfg_ACTIVE will be updated by EEPROM data!
	//  RETURNS
	//		true if read successful  	(CRC=Good & Firmware format code Matches EEPROM, Valid/Good format)
	//  	false if read unsuccessful 	(CRC=Bad -OR- Firmware format code DOES NOT MATCH EEPROM, Invalid/Bad format)
	//
	//	20190416	E.Andrews	Rev 0	Initial release
	//	20190425	E.Andrews	Rev 1	Add CRC check prior to read for valid format code & Optional CheckOnly variable
	//
	#define ReadCalCfgDebug
	long CRC_ComputedFromEEPROM=0;
	long CRC_FromEEPROM=0;
	uint8_t CRC_OK=false;
	uint8_t ReturnValue=false;
	
	//First, Compute CRC for data block we're interested in...
	CRC_ComputedFromEEPROM=ComputeEEPROM_CRC(eeprom_CalCfg_Adr,eeprom_CalCfg_CRC_Adr-eeprom_CalCfg_Adr);
	
	//Now retrieve CRC from EEPROM that was stored right after the end of the Data Block
	EEPROM.get(eeprom_CalCfg_CRC_Adr,CRC_FromEEPROM);

	//Validate EEPROM data quality
	if (CRC_ComputedFromEEPROM==CRC_FromEEPROM) CRC_OK=true; else CRC_OK=false;

	#ifdef ReadCalCfgDebug
		if (CheckOnly==true) Serial.println (F("*************CHECK MODE - Called with CheckOnly = true"));
		else Serial.println (F("****************READ MODE - Called with CheckOnly = false"));
		Serial.print(F("Checking and Fetching CalCfg data from EEPROM...."));
		Serial.print(F("Check #1 - Calc directly from EPROM: "));Serial.println(CRC_ComputedFromEEPROM,HEX);
		Serial.print(F("                CRC Stored in EPROM: "));Serial.println(CRC_FromEEPROM,HEX);
		Serial.print(F("                        CRC Results: "));
		if(CRC_OK==true) Serial.println(F("CRC MATCH - OKAY to get Read Data from EPROM!")); else Serial.println (F("CRC DOES NOT MATCH - Data No Good!"));
	#endif

	//Now read and validate DB_RevCode
	uint8_t EEPROM_DB_RevCode[5];
	uint8_t DB_RevCodeMATCH = false;
	if (CRC_OK){
		//Retrieve DB_RevCode stored in EEPROM
		char EEPROM_DB_RevCode[4];
		EEPROM.get(eeprom_CalCfg_Adr,EEPROM_DB_RevCode);
		//Check char by char that code matches this version of code
		if(EEPROM_DB_RevCode[0] == CalCfg_DEFAULT.DB_RevCode[0]
		&& EEPROM_DB_RevCode[1] == CalCfg_DEFAULT.DB_RevCode[1]
		&& EEPROM_DB_RevCode[2] == CalCfg_DEFAULT.DB_RevCode[2]
		&& EEPROM_DB_RevCode[3] == CalCfg_DEFAULT.DB_RevCode[3]){
			//All is GOOD!  
			DB_RevCodeMATCH=true;
			//Now, Actually read the EEPROM data into CalCfg_Active!
			if (CheckOnly==false) EEPROM.get(eeprom_CalCfg_Adr, CalCfg_ACTIVE);
								
		}else DB_RevCodeMATCH=false; //If No-Match, set flag to FALSE
	}
	
	if (CRC_OK && DB_RevCodeMATCH) ReturnValue=true;else ReturnValue=false;
	
	#ifdef ReadCalCfgDebug
		if (DB_RevCodeMATCH==true){
			Serial.println(F("EEPROM RevCode_Match=OKAY!"));
		}
		else {
			Serial.println(F("EEPROM REV CODE DIDN'T MATCH!!!"));
		}
		Serial.print(F(" RevCode Check - EEPROM/SHOULD BE: "));
		Serial.print(char(EEPROM_DB_RevCode[0]));Serial.print(char(EEPROM_DB_RevCode[1]));Serial.print(char(EEPROM_DB_RevCode[2]));Serial.print(char(EEPROM_DB_RevCode[3]));
		Serial.print(F(" / "));
		Serial.println(CalCfg_DEFAULT.DB_RevCode);
		
	#endif
	
	return ReturnValue;
}
									
uint8_t  read_FilamentDB_from_EEPROM() {
	//  This routine will read Filament data base structure from 
	//	EEPROM.  Caller is responsible to validate returned data.
	//	
	//  RETURNS	Data will be returned to global structure FilType_ACTIVE[].
	//		true if read successful  	(Firmware format code Matches EEPROM, Valid/Good format)
	//  	false if read unsuccessful 	(Firmware format code DOES NOT MATCH EEPROM, Invalid/Bad format)
	//
	//	20190416	E.Andrews	Rev 0	Initial release
	//	20190425	E.Andrews	Rev 1	Remove need for FilType_EEPROM data structure
	//
	uint8_t ReturnValue=false;	//Set default value to FAILED.  Change it downstrem WHEN we complete a valid READ
	uint8_t	CheckOnly = true;	//Don't actually update CalConfig_ACTIVE, just check verify it's the right DB Revision
	long CRC_ComputedFromEEPROM=0;
	long CRC_FromEEPROM=0;
	uint8_t CRC_OK=false;
	//Check DB REV level
	if (read_CalCfg_from_EEPROM(CheckOnly)==true){	//Check CalCfg EEPROM space first; This verifies CalCfg CRC & DB_Rev Level.  True = All OK!
		//DB REV level OKAY...Now verify EEPROM CRC integrity. 
		
		//Compute EEPROM CRC value for data block we're interested in...
		int NumOfBytes = eeprom_FilTyp_CRC_Adr-eeprom_FilTyp_Adr;
		CRC_ComputedFromEEPROM=ComputeEEPROM_CRC(eeprom_FilTyp_Adr,NumOfBytes);
		
		//Now retrieve CRC from EEPROM that was stored right after the end of the Data-Block
		EEPROM.get(eeprom_FilTyp_CRC_Adr,CRC_FromEEPROM);
		
		//If these two CRCs agree, Data Block is VALID...Go ahead and read it!
		if (CRC_ComputedFromEEPROM==CRC_FromEEPROM) {
			EEPROM.get(eeprom_FilTyp_Adr, FilType_ACTIVE);	//Valid, Read eeprom data into FilType_ACTIVE
			ReturnValue=true;		//Return SUCCESS!
		}	
	}
	
	return ReturnValue;
}

void write_CalCfg_to_EEPROM() {
	//	This routine writes the CURRENT DB to EEPROM.
	// 	By using the 'put' function, only changed data is actually
	//	written to EEPROM thus extending EEPROM operating life.
	//
	//  RETURNS	Nothing
	//
	//	20190416	E.Andrews	Rev 0	Initial release
	//
	//#define WriteCalCfgDebug
	#ifdef WriteCalCfgDebug
		Serial.print(F("Saving CalConfig_ACTIVE..."));
	#endif
	//Compute CRC
	long CRC = ComputeMEM_CRC(CalCfg_ACTIVE.DB_RevCode,sizeof(CalCfg_ACTIVE));
	//Save Data
	EEPROM.put(eeprom_CalCfg_Adr, CalCfg_ACTIVE);
	//Save CRC
	EEPROM.put(eeprom_CalCfg_CRC_Adr, CRC);
	#ifdef WriteCalCfgDebug
		Serial.println(F("done."));
	#endif
}

void write_FilamentDB_to_EEPROM() {
	//	This routine writes the FilType_ACTIVE DB to EEPROM.
	// 	By using the 'put' function, only changed data is actually
	//	written to EEPROM thus extending EEPROM operating life.
	//
	//  RETURNS	Nothing
	//
	//	20190416	E.Andrews	Rev 0	Initial release
	//
	//#define WriteEepromDebug
	#ifdef WriteEepromDebug
		Serial.print(F("Saving FilType_ACTIVE..."));
	#endif
	int Fil_Element_Length=sizeof(FilType_DEFAULT[0]);
	int Fil_Structure_Length=sizeof(FilType_DEFAULT);
	int NumOfElements = Fil_Structure_Length/Fil_Element_Length;
	#ifdef WriteEepromDebug
		Serial.print(F("Save Fil_Element_Len = "));Serial.print(Fil_Element_Length);Serial.print(F(", Save_Fil_Structure_Len = "));Serial.println(Fil_Structure_Length);
	#endif
	//Compute CRC
	long CRC = ComputeMEM_CRC(FilType_ACTIVE[0].Name,sizeof(FilType_ACTIVE));
	//Save Data
	for (int i=0;i<NumOfElements;i++){
		EEPROM.put(eeprom_FilTyp_Adr+(i*Fil_Element_Length), FilType_ACTIVE[i]);
	}
	//Save CRC
	EEPROM.put(eeprom_FilTyp_CRC_Adr, CRC);
	#ifdef WriteEepromDebug
		Serial.print(F("done. CRC = "));Serial.println(CRC,HEX);
	#endif
}
void Clear_CalCfg_EEPROM() {
	//	This routine will clear the CalConfg space in EEPROM.
	// 	By using the 'put' function, only changed data is actually
	//	written to EEPROM thus extending EEPROM operating life.
	//
	//  RETURNS	Nothing
	//
	//	20190416	E.Andrews	Rev 0	Initial release
	//
	const uint8_t Blank=0xFF;
	//#define ClrCalCfg_DEBUG
	#ifdef ClrCalCfg_DEBUG
		Serial.print(F("Clearing CalCfg starting at eeprom_adr = 0x"));
		Serial.print(eeprom_CalCfg_Adr,HEX);Serial.print(F(", # bytes = "));
		Serial.print(sizeof(CalCfg_ACTIVE));Serial.println();
	#endif
	//Clear out sizeof CalCfg structure + size of CRC bytes
	for (int i=0;i<sizeof(CalCfg_ACTIVE)+sizeof(CalCfg_CRC_ACTIVE);i++){
		EEPROM.put(eeprom_CalCfg_Adr+i, Blank);	
	}
}
void Clear_FilamentDB_EEPROM() {
	//	This routine will clear the Filament DB space in EEPROM.
	// 	By using the 'put' function, only changed data is actually
	//	written to EEPROM thus extending EEPROM operating life.
	//
	//  RETURNS	Nothing
	//
	//	20190416	E.Andrews	Rev 0	Initial release
	//
	const uint8_t Blank=0xFF;
	//#define ClrFilDB_DEBUG
	#ifdef ClrFilDB_DEBUG
		Serial.print(F("Clearing FilamentDB starting at eeprom_adr = 0x"));
		Serial.print(eeprom_FilTyp_Adr,HEX);Serial.print(F(", # bytes = "));
		Serial.print(sizeof(FilType_ACTIVE));Serial.println();
	#endif
	for (int i=0;i<sizeof(FilType_ACTIVE)+sizeof(FilType_CRC_EEPROM);i++){
		EEPROM.put(eeprom_FilTyp_Adr+i, Blank);	
	}
}
void SaveAllToEEPROM(uint8_t SaveWithAsk=true) {
	//	This routine will write CalCfg_ACTIVE  -> EEPROM.
	//	This routine will write FilType_ACTIVE -> EEPROM.
	// 	By using the 'put' function, only changed data is actually
	//	written to EEPROM thus extending EEPROM operating life.
	//
	//	INPUTS
	//	uint8_t	SaveWithOutAsk	If set false (or omitted), save will Aske for Oper Save-Confirmation
	//							if set true, save will NOT ask for Oper Save-Confirmation 
	//  RETURNS	Nothing
	//
	//	20190417	E.Andrews	Rev 0	Initial release
	//	20190427	E.Andrews	Rev 1	Added 'SaveWithAsk' feature
	//	20190502	E.Andrews	Rev 1.1	Added check to see if "SaveIsNeeded"
	//									This will skip saves when no changes are made to DB
	//	
	//#define SaveToEEPROM_Debug	//Uncomment this line to get debug messages for this routine out to the serial port
	lcd_clearRows(0,4);	//Clear display
	const uint8_t YN_row =3;
	uint8_t SaveApprovedFlag=true;	//Default condition for oper response flag
	uint8_t	SaveIsNeeded=false;		// This flag will be set true if either DB needs to be updated.
	long CRC_FromEEPROM=0;
	long CRC_FromACTIVE=0;

	//Check CalCfg DB to see if anything has changed.
	EEPROM.get(eeprom_CalCfg_CRC_Adr,CRC_FromEEPROM);
	CRC_FromACTIVE=ComputeMEM_CRC(CalCfg_ACTIVE.DB_RevCode,sizeof(CalCfg_ACTIVE));
	if(CRC_FromACTIVE!=CRC_FromEEPROM) {
		SaveIsNeeded=true;	//CalCfg has changed...Set Flag==true
		#ifdef SaveToEEPROM_Debug
			Serial.print(__LINE__);Serial.println(F(": CalCfg DB has Changed.  Save Needed!"));
		#endif
	}
	
	//Check FilType_ACTIVE DB to see if anything has changed.
	EEPROM.get(eeprom_FilTyp_CRC_Adr,CRC_FromEEPROM);
	CRC_FromACTIVE=ComputeMEM_CRC(FilType_ACTIVE[0].Name,sizeof(FilType_ACTIVE));
	if(CRC_FromACTIVE!=CRC_FromEEPROM) {
		SaveIsNeeded=true;	//FilType has changed...Set Flag==true
		#ifdef SaveToEEPROM_Debug
			Serial.print(__LINE__);Serial.println(F(": FilType DB has Changed.  Save Needed!"));
		#endif
	}
	#ifdef SaveToEEPROM_Debug
		if(SaveIsNeeded==false){
			Serial.print(__LINE__);
			Serial.println(F(": No change to DB found!"));
		}
	#endif
	if(SaveIsNeeded==true && SaveWithAsk==true){	//Don't ask Oper for approval if save is not actually needed
		lcd.print(F("Save data to EEPROM?"));
		//This is the LCD row we want to use for the Y/N question
		SaveApprovedFlag=Get_Opr_YN(Yes,YN_row);
	}
	if (SaveIsNeeded==true && SaveApprovedFlag==true) {	
		write_CalCfg_to_EEPROM();
		write_FilamentDB_to_EEPROM();
		lcd_clearRow(3);
		lcd.setCursor(0,3);	lcd.print(F(" EEPROM SAVE Done!"));
		delay(2000);
	}else{
		if (SaveIsNeeded==true){ //If save is needed by NOT performed, report it to operator
			lcd_clearRows(0,4);
			lcd.setCursor(0,1);	lcd.print(F("NOT SAVED to EEPROM!"));lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
		}
	}

	
}

void SaveJustCalCfgToEEPROM() {
	//	This routine will write CalCfg_ACTIVE  -> EEPROM.
	// 	By using the 'put' function, only changed data is actually
	//	written to EEPROM thus extending EEPROM operating life.
	//
	//  RETURNS	Nothing
	//
	//	20190417	E.Andrews	Rev 0	Initial release
	//	
	lcd_clearRows(0,4);	//Clear display
	lcd.print(F("Save Cal/Cfg>EEPROM?"));
	const uint8_t YN_row =3;			//This is the LCD row we want to use for the Y/N question
	if (Get_Opr_YN(Yes,YN_row)==Yes) {	//Stays in YN routine until Oper makes a selection
		write_CalCfg_to_EEPROM();
		lcd_clearRow(3);
		lcd.setCursor(0,3);	lcd.print(F(" Cal/Cfg SAVE Done!"));
	}else{
		lcd_clearRows(0,4);
		lcd.setCursor(0,1);	lcd.print(F("   SAVE Cancelled   "));lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
	}
	delay(3000);
	
}
void ClearAllEEPROM() {
	//	This routine will clear all of EEPROM.
	//
	//  RETURNS	Nothing
	//
	//	20190419	E.Andrews	Rev 0	Initial release
	//	
	lcd_clearRows(0,4);	//Clear display
	lcd.print(F("Erase EEPROM Data?"));
	const uint8_t YN_row =3;			//This is the LCD row we want to use for the Y/N question
	if (Get_Opr_YN(Yes,YN_row)==Yes) {	//Stays in YN routine until Oper makes a selection
		Clear_CalCfg_EEPROM();
		Clear_FilamentDB_EEPROM();
		lcd_clearRow(3);
		lcd.setCursor(0,3);	lcd.print(F("All of EEPROM ERASED"));
	}else{
		lcd_clearRows(0,4);
		lcd.setCursor(0,1);	lcd.print(F("  ERASE Cancelled   "));lcd.setCursor(0,2);lcd.print(F("    by Operator!"));
	}
	delay(3000);
	
}

void dumpCalCfgDB(){
	//	This is a test and diagnostic routine will send the DEFAULT and ACTIVE 
	//	CalCfgDB values out to the serial port.  This is not typically useful but
	//	was handy to have during code development.
	//
	//  RETURNS	Nothing
	//
	//	20190419	E.Andrews	Rev 0	Initial release
	//	20190401	E.Andrews	Rev 0.1	Add AMC1 CalCfg Ld_Cell_Filter value
	//

	#ifndef Dump_eeprom_array_DEBUG
		Serial.println(F("Dump:\tCalCfg_DEFAULT\tCalCfg_ACTIVE"));
	#endif
	Serial.print(F("     StructRev: "));Serial.print(CalCfg_DEFAULT.DB_RevCode);Tab;
									Tab;Serial.println(CalCfg_ACTIVE.DB_RevCode);
									
	Serial.print(F(" Ld_Cell Scale: "));Serial.print(CalCfg_DEFAULT.LdCell_Scale,4);
									Tab;Serial.println(CalCfg_ACTIVE.LdCell_Scale,4);
									
	Serial.print(F("Ld_Cell Offset: "));Serial.print(CalCfg_DEFAULT.LdCell_Offset);
									Tab;Serial.println(CalCfg_ACTIVE.LdCell_Offset);
									
	Serial.print(F("Ld_Cell Filter: "));Serial.print(CalCfg_DEFAULT.LdCell_Filter);
									Tab;Serial.println(CalCfg_ACTIVE.LdCell_Filter);
									
	Serial.print(F(" Last Spool Wt: "));Serial.print(CalCfg_DEFAULT.Last_SpoolWtG);Tab;
									Tab;Serial.println(CalCfg_ACTIVE.Last_SpoolWtG);
									
	Serial.print(F("     Last Fnum: "));Serial.print(CalCfg_DEFAULT.Last_Fnum);Tab;
									Tab;;Serial.println(CalCfg_ACTIVE.Last_Fnum);
									
	Serial.print(F("     Len Units: "));Serial.print(CalCfg_DEFAULT.Gbl_LenUnits);Tab;
									Tab;Serial.println(CalCfg_ACTIVE.Gbl_LenUnits);
									
	Serial.print(F("      Wt Units: "));Serial.print(CalCfg_DEFAULT.Gbl_WtUnits);Tab;
									Tab;Serial.println(CalCfg_ACTIVE.Gbl_WtUnits);
									
									Serial.println();
	Serial.print(F("        #Bytes: "));Serial.print(sizeof(CalCfg_DEFAULT));Tab;
									Tab;Serial.println(sizeof(CalCfg_ACTIVE));
									
	unsigned long CalCfg_CRC = ComputeMEM_CRC(CalCfg_ACTIVE.DB_RevCode,sizeof(CalCfg_ACTIVE));
	Serial.print(F("      CRC(HEX): "));Serial.print(ComputeMEM_CRC(CalCfg_DEFAULT.DB_RevCode,sizeof(CalCfg_DEFAULT)),HEX);Tab;
									Serial.print(ComputeMEM_CRC(CalCfg_ACTIVE.DB_RevCode,sizeof(CalCfg_ACTIVE)),HEX);
									Serial.println();

}

void dumpFilamentDB(){
	//	This is a test and diagnostic routine will send the DEFAULT and ACTIVE 
	//	FilamentDB values out to the serial port.  This is not typically useful but
	//	was handy to have during code development.
	//
	//  RETURNS	Nothing
	//
	//	20190419	E.Andrews	Rev 0	Initial release
	//	
	
	
	for(uint8_t i=0;i<FnumMax;i++){
		if(i+1<10)Serial.print(" ");
		Tab; Serial.print(i+1);Serial.print(":");Tab;
		Serial.print (FilType_DEFAULT[i].Name[0]);Serial.print (FilType_DEFAULT[i].Name[1]);Serial.print (FilType_DEFAULT[i].Name[2]);Serial.print (FilType_DEFAULT[i].Name[3]);
		Serial.print(", ");Serial.print (FilType_DEFAULT[i].Dia);
		Serial.print(", ");Serial.print (FilType_DEFAULT[i].Dens);
		
		#ifdef Dump_eeprom_Fil_array_DEBUG
			Tab;Serial.print (FilType_EEPROM[i].Name[0]);Serial.print (FilType_EEPROM[i].Name[1]);Serial.print (FilType_EEPROM[i].Name[2]);Serial.print (FilType_EEPROM[i].Name[3]);
			Serial.print(", ");Serial.print (FilType_EEPROM[i].Dia);
			Serial.print(", ");Serial.print (FilType_EEPROM[i].Dens);
		#endif
		
		Tab;Serial.print (FilType_ACTIVE[i].Name[0]);Serial.print (FilType_ACTIVE[i].Name[1]);Serial.print (FilType_ACTIVE[i].Name[2]);Serial.print (FilType_ACTIVE[i].Name[3]);
		Serial.print(", ");Serial.print (FilType_ACTIVE[i].Dia);
		Serial.print(", ");Serial.print (FilType_ACTIVE[i].Dens);		
		
		Serial.println();
	}
	Serial.println();
	Serial.print(F("        #Bytes: "));Serial.print(sizeof(FilType_DEFAULT));Tab;
	Tab;Serial.println(sizeof(FilType_ACTIVE));
	
	unsigned long FilType_CRC = ComputeMEM_CRC(FilType_ACTIVE[0].Name,sizeof(FilType_ACTIVE));
	Serial.print(F("      CRC(HEX): "));Serial.print(ComputeMEM_CRC(FilType_DEFAULT[0].Name,sizeof(FilType_DEFAULT)),HEX);Tab;
	Serial.print(ComputeMEM_CRC(FilType_ACTIVE[0].Name,sizeof(FilType_ACTIVE)),HEX);
	Serial.println();
	
}
//============= END EEPROM ROUTINES ==========================================================
void setup()
{
	Serial.begin(115200);		//Open Serial debug port
	//delay(1000);				//Time for Serial Startup?? TODO Remove this line!
	//Set-up Encoder Push Button pin assignment and INPUT with an internal pull-up resistor
	pinMode(EncPB_Pin,INPUT_PULLUP);
	// After setting up the button, set-up the Bounce instance :
	Enc_PB_debounce.attach(EncPB_Pin);
	Enc_PB_debounce.interval(5);	// interval in ms
	ResetEncPushButton();			//Initialize Encoder Push-Button variables
	
	//Initialize Encoder value
	Enc_Knob.write(500);
	Read_Enc_Push_Button;
	GBL_menuNum=0;
	#define SetupPrintDiag
	//#ifdef SetupPrintDiag
		Serial.print("Program Started: ");
		Serial.println(ProgName);
		Tab;Serial.print("  ");Serial.println(__DATE__);
		Tab;Serial.print("  ");Serial.println(__TIME__);
	//#endif
	pinMode(StatusLedPin,OUTPUT);
	digitalWrite(StatusLedPin,HIGH);	//Turn LED ON
	int SYM_BitmapSize = (sizeof(SYM_Bitmap ) / sizeof (SYM_Bitmap[0]));
	// Switch on the backlight
	//pinMode ( BACKLIGHT_PIN, OUTPUT );
	//digitalWrite ( BACKLIGHT_PIN, HIGH );
  
	//lcd.begin(16,2);               // initialize the lcd 
	lcd.begin(20,4);               // initialize the lcd 
	
	// Switch on the backlight  
	lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
	lcd.setBacklight(HIGH);
	lcd.home ();                // go home
	lcd.print(F("  "));
	lcd.print(ProgName);  
	lcd.setCursor ( 0, 1 );     // go to the next line
	lcd.print(F("    "));
	lcd.print (__DATE__);		//Printout Compiled Data and Time
	lcd.setCursor ( 0, 2 );      // go to the next line
	lcd.print(F("     "));
	lcd.print (__TIME__);		//Printout Compiled Data and Time
	
	//Upload custom symbols to LCD display
	for ( int i = 0; i < SYM_BitmapSize; i++ ) {
		lcd.createChar ( i, (uint8_t *)SYM_Bitmap[i] );
	}
	
	#ifdef SetupPrintDiag
		Serial.println(F("At Power-up (BEFORE CalCfg_ACTIVE = CalCfg_DEFAULT)..."));
		dumpCalCfgDB();
	#endif
		
	CalCfg_ACTIVE = CalCfg_DEFAULT;	//Init CalCfg with Defaults in case EEPROM can't be read
	for(int i=0;i<FnumMax;i++){		//Make Filament_DB DEFAULT into ACTIVE
		FilType_ACTIVE[i]=FilType_DEFAULT[i];	
	}
	#ifdef SetupPrintDiag
		Serial.println(F("Before EEPROM Read (AFTER CalCfg_ACTIVE = CalCfg_DEFAULT)......"));
		dumpCalCfgDB();
		Serial.print(F("Reading CalCfg Data from EEPROM...Read "));
		Serial.println(F("AFTER EEPROM READ..."));
	#endif
	//Now READ Put EEPROM data into CalCfg_ACTIVE
	if (read_CalCfg_from_EEPROM()==true) {
		lcd.setCursor(0,3);
		lcd.print(F("EEPROM CalData found"));
		#ifdef SetupPrintDiag
			Serial.println(F("SUCCESSFUL!  1861"));
		#endif
	}
	else {
		#ifdef SetupPrintDiag
			Serial.println(F("FAILED! 1865"));
		#endif
		delay(2000);
		CalCfg_ACTIVE=CalCfg_DEFAULT;	//Make CalConfg DEFAULT settings into ACTIVE
		for(int i=0;i<FnumMax;i++){		//Make Filament_DB DEFAULT into ACTIVE
			FilType_ACTIVE[i]=FilType_DEFAULT[i];	
		}
		lcd_clearRows(0,4);
		lcd.setCursor(0,0);
		lcd.print(F("1.No EEPROM DB found"));
		lcd.print(F("2.Using default data"));
		lcd.print(F("3.Zero & Cal. Req'd."));
		lcd.print(F("4.EEPROM Save Req'd."));
	}
	uint8_t EPROM_FilReadSuccess = read_FilamentDB_from_EEPROM();


	// Initialize 'scale' library with data output pin, clock input pin and gain factor.
	// Channel selection is made by passing the appropriate gain:
	// - With a gain factor of 64 or 128, channel A is selected
	// - With a gain factor of 32, channel B is selected
	// By omitting the gain factor parameter, the library
	// default "128" (Channel A) is used here.
	scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
	//scale.set_scale(2280.f);          // this value is obtained by calibrating the scale with known weights; see the README for details
	//scale.set_scale();
	//const float DefaultScale=392582.4176;
	//const long DefaultOffset = -157616.;
	scale.set_scale(CalCfg_ACTIVE.LdCell_Scale);	//Set Default Scale GAIN as stored in CalCfg EEPROM
													//The value used here is updated in EEPROM every time a CALIBRATION step is performed by the Operator.
										//This value scales output to Kg (Use 3 decimal places when printing Kg measurements)
										//This program always wants to measure in Kg! Use standard Wt conversions if you want to show
										//other units on the display if you want.
										//This value is obtained by calibrating the scale with known weights.
										//SpoolScale INCLUDES a built in CALIBRATION function that can periodically run by the operator
										// using the Rotary Dial control. After calibration, a new value is stored into EEPROM for recall every powerup
										//ALTERNATELY, see the HX711 README file for more details on how to do this using HX711_full_example program.
	
	//scale.tare();				    	//Uncomment this if you want to AUTO TARE every time you power up
										//	|  This is NOT RECOMMENDED as it requires the user to	|
										//	|  remove filament spool from SCALE at every power-up	|
										//INSTEAD,SpoolScale INCLUDES a built-in ZERO function that can be periodically run by the operator using the Rotary Dial control. 
										
	scale.set_offset(CalCfg_ACTIVE.LdCell_Offset);	//Set Default Scale OFFSET (Tare Value) stored in CalCfg
													//The value used here is measured, set, and stored in EEPROM every time a ZERO function is performed by the Operator.
	
	Fnum=CalCfg_ACTIVE.Last_Fnum;		//initalize Filament number from that stored in CalCfg_ACTIVE DB
	
	delay ( 5000 );						//Wait 5 secs for power up messages to be displayed
	lcd_clearRows(0,4);
	#ifdef SetupPrintDiag
		Serial.println(F("Leaving Setup....."));
	#endif
}
uint8_t LenUnits=0;
void loop(){
	
	if (Fnum>FnumMax) Fnum=FnumMax;		//Range check the Filament Number value (must be confined to 1 & 16)
	if (Fnum<1) Fnum=1;

	//Retrieve data for current filament from DataBase and load it into Global variables
	GBL_Diam_mm = FilType_ACTIVE[Fnum-1].Dia;
	GBL_Dens_gcm3=FilType_ACTIVE[Fnum-1].Dens;

	
	//Read & Display load cell in Kg
	MeasurePrint_FilNam_Wt_Length(LenUnits);	//Update Wt measurements & display ()
	Read_Enc_Push_Button();							//Read Encoder Push Button
	 //============= BEGIN MENU LOOP========================================
	if(GBL_menuNum==0 && Enc_PB_DownEdge==HIGH){	//Did Oper Push the Knob?
		//YES!  Enter MENU MODE
		Enc_PB_DownEdge=LOW;
		GBL_menuNum=1;
		Enc_Knob.write(0);	//Reset Encoder-Knob Value...
		lcd_clearRow(3);
		//Go to Menu Mode

		const uint8_t menuMax=12;	//Update this value if new menu options are added
		uint8_t menuDone=0;
		while (menuDone==0){
			//Read & Display load cell in Kg
			#define UpdateDuringMenu	//Comment out this line to make scale inactive in MenuMode (ie: menuDone==0)
			#ifdef UpdateDuringMenu
				MeasurePrint_FilNam_Wt_Length(LenUnits);	//Update Wt measurements & display ()
			#endif
			Read_Enc_Push_Button();
			//uint8_t Prev_menuNum=GBL_menuNum;
			if(Enc_Knob.read()>3){	//Step Forward
				lcd_clearRow(3);
				GBL_menuNum++;
				Enc_Knob.write(0);
			}
			if(Enc_Knob.read()<-3){	//Step Backward
				lcd_clearRow(3);
				GBL_menuNum--;
				Enc_Knob.write(0);
			}		
			if (GBL_menuNum<1)GBL_menuNum=1;			//range limit GBL_menuNum
			if (GBL_menuNum>menuMax)GBL_menuNum=menuMax;
			
			lcd_printMenu(GBL_menuNum);	//This call puts current menu option on bottom row of LCD 

			if(Enc_PB_DownEdge==HIGH){	//Did Oper Push the Knob & select an option?
				Enc_PB_DownEdge=LOW;	//Reset Edge detector
				//=====Here is main menu Decoder!
				switch (GBL_menuNum){
					case 1:	//Zero Scale...
						CalCfg_ACTIVE.LdCell_Offset=ZeroScale();
						SaveAllToEEPROM(GBL_AutoSaveFlag);
						GBL_menuNum=0;
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 2:	//Enter new SPOOL WT
						CalCfg_ACTIVE.Last_SpoolWtG = SetSpoolWt(CalCfg_ACTIVE.Last_SpoolWtG);
						SaveAllToEEPROM(GBL_AutoSaveFlag);
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 3:	//Select a different Filament
						Fnum = SelectFilament(Fnum);
						CalCfg_ACTIVE.Last_Fnum=Fnum;
						SaveAllToEEPROM(GBL_AutoSaveFlag);
						GBL_menuNum=0;
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 4:	//Change Filament Density
						FilType_ACTIVE[Fnum-1].Dens = SetFilamentDens(FilType_ACTIVE[Fnum-1].Dens);
						SaveAllToEEPROM(GBL_AutoSaveFlag);
						GBL_menuNum=0;
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 5:	//Change Filament Diameter
						FilType_ACTIVE[Fnum-1].Dia = SetFilamentDiam(FilType_ACTIVE[Fnum-1].Dia);
						SaveAllToEEPROM(GBL_AutoSaveFlag);
						GBL_menuNum=0;
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 6:	//Change Filament NAME
						SetFilamentName(FilType_ACTIVE[Fnum-1].Name);
						SaveAllToEEPROM(GBL_AutoSaveFlag);
						GBL_menuNum=0;
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 7:	//Save Data to EEPROM - This is a MANUAL-SAVE function
						SaveAllToEEPROM();	//Always ask operator to confirm!
						GBL_menuNum=0;
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 8:	//Calibrate Scale...
						CalCfg_ACTIVE.LdCell_Scale = CalibrateScale(CalCfg_ACTIVE.LdCell_Scale);
						scale.set_scale(CalCfg_ACTIVE.LdCell_Scale);	//20190730: Added this ro update active cal factor value with latest cal factor
						SaveAllToEEPROM(GBL_AutoSaveFlag);
						GBL_menuNum=0;
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 9:	//Change Scale Readout Filter Value
						CalCfg_ACTIVE.LdCell_Filter=SetLdCellFilter(CalCfg_ACTIVE.LdCell_Filter);
						SaveAllToEEPROM(GBL_AutoSaveFlag);
						GBL_menuNum=0;
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 10:	//DataBase Dump to Serial Port
						dumpCalCfgDB();
						dumpFilamentDB();
						lcd.clear();
						lcd.setCursor(0,1);
						lcd.print(F("DB Sent->Serial Port"));
						lcd.clear();
						delay(2000);
						GBL_menuNum=0;
						menuDone=1;
						lcd_clearRows(0,4);
					break;
					case 11:	//ERASE EEPROM space - This will 'kill off' all current calibrations and DB values!
						ClearAllEEPROM();
						menuDone=1;
						GBL_menuNum=0;
						lcd_clearRows(0,4);
					break;
					case menuMax:
						menuDone=1;	//Last option is always Menu-DONE
						GBL_menuNum=0;
						lcd_clearRows(0,4);
					break;
					default:
						menuDone=1;	//Undefined options will report as NOT-AVAIL & then GOTO Menu-DONE
						lcd_clearRows(0,3);
						lcd.setCursor(0,1);
						lcd.print(F("** NOT  AVAILABLE **"));
						delay(3000);
						menuDone=1;
						GBL_menuNum=0;
						lcd_clearRows(0,4);
					break;
					
					
				}

			}
			
		}
		lcd_clearRow(3);
	}	//============= END MENU LOOP ==============================
	
	
	//Just for fun, toggle LED at a "loop Rate" ...The LED will flicker during normal operation...If it is ever OFF or STEADY, main loop is not running!
	if (digitalRead(StatusLedPin)==HIGH) digitalWrite(StatusLedPin,LOW); else digitalWrite(StatusLedPin,HIGH);	//Turn LED ON
}