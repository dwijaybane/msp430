/*
 * messages.c
 *
 *  Created on: Mar 19, 2014
 *      Author: JB ELECTRONICS
 */

const unsigned char *heading[5] = {"**************************************************\n\r",
						           "* Edutech Systems-	Diagnostic Software      *\n\r",
						           "* Developed By  :	PANKIL SHETH             *\n\r",
			     		           "* Board ID: 		EPBMSP430                *\n\r",
						           "**************************************************\n\n\r"
			     		  };
const unsigned char *mainmenu[10] = {"Main Menu\n\n\r",
 		"1.  LED Test\n\r",
 		"2.  Switch Test\n\r",
		"3.  Matrix Keyboard\n\r",
		"4.  Segment LCD\n\r",
		"5.  10 bit ADC\n\r",
		"6.  SD 24\n\r",
		"7.  Hardware Multiplier\n\r",
        "8.  I2C\n\r",
        "9.  PWM Test\n\n\r"
 	      "Select options (1-9):"
		};

void Print_Main_Menu()
{
	int i;
	printf(UARTA1,"\x1b[2J");
	for(i=0;i<5;i++)
	{
		printf(UARTA1,"%s",heading[i]);
	}
	for(i=0;i<10;i++)
	{
		printf(UARTA1,"%s",mainmenu[i]);
	}
}

void Print_Heading()
{
	int i;
	printf(UARTA1,"\x1b[2J");
	for(i=0;i<5;i++)
	{
		printf(UARTA1,"%s",heading[i]);
	}
}

void Print_Esc_Menu()
{
	printf(UARTA1,"\n\n\n\n\n\n\n\n\n\n\rPRESS ESC. KEY TO EXIT\n\r");
}

void Print_Error_Message()
{
	printf(UARTA1,"\n\rINVALID KEY PRESS\n\rPRESS ANOTHER VALID KEY\n\r");
}
