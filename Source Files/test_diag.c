/*
 * test_diag.c
 *
 *  Created on: Mar 12, 2014
 *      Author: JB ELECTRONICS
 */

extern int adcch[6],sd24ch[6];
extern volatile unsigned int ans16u[2];

void Test_Menu()
{
	int get_key;
	while(1)
	{
		Print_Main_Menu();
		get_key = getche(UARTA1,BLOCKING);
		//printf(UARTA1,"%c",get_key);

		switch(get_key)
		{
		case '1' :
			Print_Heading();
			LED_Test();
			break;
		case '2' :
			Print_Heading();
			Switch_Test();
			break;
		case '3' :
			Print_Heading();
			Mat_Kbrd_Test();
			break;
		case '4' :
			Print_Heading();
			Seg_LCD_Test();
			break;
		case '5' :
			Print_Heading();
			ADC_Test();
			break;
		case '6' :
			Print_Heading();
			SD_Test();
			break;
		case '7' :
			MPY_Test();
			break;
		case '8' :
			I2C_Test();
			break;
		case '9' :
			PWM_Test();
			break;
		default:
			Print_Error_Message();
			Delay_Ms(3000);
			break;
		}
	}
}

void LED_Test()
{
	unsigned char dummy[1] = {0};
	printf(UARTA1,"LEDs Will Blink Continuosly\n\r");
	Print_Esc_Menu();
	while(1)
	{
		UART_Recieve(UARTA1,dummy,1,NONE_BLOCKING);

		if(dummy[0] == ESC_KEY)
		{
			GPIO_setOutputHighOnPin(GPIO_PORT_P5,_SBF(1,0x07));
			return;
		}
		else
		{
			GPIO_toggleOutputOnPin(GPIO_PORT_P5,_SBF(1,0x07));
			Delay_Ms(500);
		}
	}
}

void Switch_Test()
{
	unsigned char dummy[1] = {0};
	printf(UARTA1,"Press SW6 on board for Switch Testing and \n\rRespective 2nd LED will get Toggled\n\r");
	Print_Esc_Menu();
	while(1)
	{
		UART_Recieve(UARTA1,dummy,1,NONE_BLOCKING);

		if(dummy[0] == ESC_KEY)
		{
			GPIO_setOutputHighOnPin(GPIO_PORT_P5,_SBF(2,0x03));
			return;
		}
		else
		{
			if(P1IN == 0xB4 || P1IN == 0xB8)						// Check For the Key Press Detection of P1.6
			{
				p_conf++;
				r_conf = 0;
				if(p_conf >= 10)
				{
					if(pressed == 0)
					{
						GPIO_toggleOutputOnPin(GPIO_PORT_P5,GPIO_PIN1);
						GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN6);
						pressed =1;
					}
				}
			}
			else									// If not pressed.
			{
				r_conf++;
				p_conf = 0;
				if(r_conf >= 10)
				{
					pressed =0;
				}
			}
		}
	}
}

void Mat_Kbrd_Test()
{
	unsigned char dummy[1] = {0};
	unsigned int k;
	printf(UARTA1,"Press SW2 ---- LED2\n\rPress SW3 ---- LED3\n\rPress SW4 ---- LED4\n\rPress SW5 ---- LED2\n\r");
	Print_Esc_Menu();

	while(1)
	{
		UART_Recieve(UARTA1,dummy,1,NONE_BLOCKING);

		if(dummy[0] == ESC_KEY)
		{
			GPIO_setOutputHighOnPin(GPIO_PORT_P5,_SBF(1,0x07));
			return;
		}
		else
		{

			k = Detect_Matkb();
			if(k == 2)
			{
				GPIO_toggleOutputOnPin(GPIO_PORT_P5,GPIO_PIN1);
			}
			else if(k == 3)
			{
				GPIO_toggleOutputOnPin(GPIO_PORT_P5,GPIO_PIN2);
			}
			else if(k == 4)
			{
				GPIO_toggleOutputOnPin(GPIO_PORT_P5,GPIO_PIN3);
			}
			else if(k == 5)
			{
				GPIO_toggleOutputOnPin(GPIO_PORT_P5,GPIO_PIN1);
			}
		}
	}
}

void Seg_LCD_Test()
{
	unsigned char dummy[1] = {0};
	printf(UARTA1,"EDUTECH will be displayed on LCD");
	Print_Esc_Menu();
	slprintf("EDUTECH");
	while(1)
	{
		UART_Recieve(UARTA1,dummy,1,NONE_BLOCKING);

		if(dummy[0] == ESC_KEY)
		{
			clearsegment(7);
			return;
		}
	}
}

void ADC_Test()
{
	unsigned char dummy[1] = {0};
	printf(UARTA1,"Set JUMPER J7 at 2-3 shorted condition\n\rGive Analog Input in the range of (0-2.5V) at Channel 2\n\rVary the Potentiometer and Check the output on LCD\n\rFor 0v---0000\n\ror 2.5v---1023\n\r");
	Print_Esc_Menu();
	while(1)
	{
		UART_Recieve(UARTA1,dummy,1,NONE_BLOCKING);
		if(dummy[0] == ESC_KEY)
		{
			ADC10CTL0 &= ~(ADC10SC + ADC10ON);
			clearsegment(7);
			return;
		}
		else
		{
			ADC10_A_ModeInitialization(ADC10CONSEQ_0,ADC10IE,ADC10INCH_2);
			while(ADC10_A_getInterruptStatus(ADC10_A_BASE,0x01) == 0x00);
			clearsegment(7);
			slprintf("2k%d04",ADC10MEM0);
			Delay_Ms(1000);
		}
	}
}

void SD_Test()
{
	unsigned char dummy[1] = {0};
	printf(UARTA1,"Set JUMPER J7 at 2-3 shorted condition\n\rGive Analog Input in the range of (0-1.1V) at Channel 1\n\rVary the Potentiometer and Check the output on LCD\n\rFor 0v---0000\n\ror 1.1v---255\n\r");
	Print_Esc_Menu();
	while(1)
	{
		UART_Recieve(UARTA1,dummy,1,NONE_BLOCKING);
		if(dummy[0] == ESC_KEY)
		{
			SD24BCCTL0 &= ~SD24SC;
			clearsegment(7);
			return;
		}
		else
		{
			SD24_B_ModeInitialization(NO,SD24_B_CONVERTER_1,NO,YES);
			clearsegment(7);
			slprintf("0k%d04",sd24ch[3]);
			Delay_Ms(1000);

		}
	}
}

void MPY_Test()
{
	unsigned char dummy[1] = {0};
	unsigned char st_ad[2] = {0};
	unsigned int op1,op2;
	Print_Heading();
	printf(UARTA1,"ENTER 16-bit OPERAND 1 : ");
	op1 = getline_hex(UARTA1,st_ad,2,BLOCKING);

	printf(UARTA1,"\n\rENTER 16-bit OPERAND 2 : ");
	op2 = getline_hex(UARTA1,st_ad,2,BLOCKING);

	MPY32_ModeInitialization(NO,MPY32_MULTIPLY_UNSIGNED,op1,op2,NO);
	Delay_Ms(1);
	printf(UARTA1,"\n\rANSWER : %x04 %x04 \n\r",ans16u[1],ans16u[0]);
	Print_Esc_Menu();
	printf(UARTA1,"Press N for Again MULTIPLICATION\n\r");
	UART_Recieve(UARTA1,dummy,1,BLOCKING);
	if(dummy[0] == ESC_KEY)
	{
		return;
	}
	if(dummy[0] == 'N')
	{
		MPY_Test();
	}
}

void I2C_Test()
{
	unsigned char TXData[64] = "1234567891011121314151617181920212223242526272829303132333435363";
	unsigned int address = 0x0000,start_add,end_add,i;
	unsigned char RXData;
	unsigned char TxData = '1';
	unsigned char dummy[1] = {0};
	unsigned char st_ad[4] = {0};
	unsigned char end_ad[4] = {0};


	while(1)
	{
		Print_Heading();
		printf(UARTA1,"Press 1 For Display\n\rPress 2 For Byte Write\n\rPress 3 For Writing value From 0-36 in Increamental Form\n\r");
		Print_Esc_Menu();
		UART_Recieve(UARTA1,dummy,1,BLOCKING);
		switch(dummy[0])
		{
		case '1':
			Print_Heading();
			printf(UARTA1,"START ADD : ");
			start_add=getline_hex(UARTA1,st_ad,4,BLOCKING);
			printf(UARTA1,"\n\r");
			printf(UARTA1,"END ADD : ");
			end_add=getline_hex(UARTA1,end_ad,4,BLOCKING);

			WriteDataI2C(address,TxData);							// Writing Data To Specific Address
			Display_I2C_Data(start_add,end_add);								// Displaying EEPROM data
			UART_Recieve(UARTA1,dummy,1,BLOCKING);
			break;
		case '2':
			Print_Heading();
			WriteDataI2C(address,TxData);							// Writing Data To Specific Address
			printf(UARTA1,"DATA WRITTEN.\n\r ");
			Delay_Ms(2000);
			break;
		case '3':
			Print_Heading();
			printf(UARTA1,"START ADD : ");
			p=getline_hex(UARTA1,st_ad,4,BLOCKING);

			end_add = start_add + 64;
			i2c_eeprom_add = start_add;
			for(i=0;i<2;i++)
			{
				I2C_Init();
				WriteDataStringI2C(i2c_eeprom_add,TXData);
				printf(UARTA1,"%d04 written\n\r",i);
			}
			Display_I2C_Data(start_add,end_add);								// Displaying EEPROM data
			UART_Recieve(UARTA1,dummy,1,BLOCKING);
			break;
		case ESC_KEY :
			return;
			break;
		default:
			Print_Error_Message();
			break;
		}
	}
}

void PWM_Test()
{
	unsigned char st_ad[4] = {0};
	unsigned char end_ad[4] = {0};
	unsigned char dummy[1] = {0};
	unsigned int duty,freq;
	Print_Heading();
	printf(UARTA1,"Press 1 For Single Edge PWM\n\rPress 2 For Dual Edge PWM\n\r");
	Print_Esc_Menu();
	UART_Recieve(UARTA1,dummy,1,BLOCKING);
	switch(dummy[0])
	{
	case '1' :
		printf(UARTA1,"Enter Frequency in Hz in Range of(1000-9999)");
		freq=getline_dec(UARTA1,end_ad,4,BLOCKING);
		printf(UARTA1,"\n\rEnter Duty Cycle in Multiples of x10");
		duty=getline_dec(UARTA1,end_ad,2,BLOCKING);

		PWM_Gen_Single(freq,duty);
		while(1)
		{
			UART_Recieve(UARTA1,dummy,1,BLOCKING);
			if(dummy[0] == ESC_KEY)
			{
				PWM_Test();
			}
		}
		break;
	case '2' :
		printf(UARTA1,"Enter Frequency in Hz in Range of(1000-9999)");
		freq=getline_dec(UARTA1,end_ad,4,BLOCKING);
		printf(UARTA1,"\n\rEnter Duty Cycle in Multiples of x10");
		duty=getline_dec(UARTA1,end_ad,2,BLOCKING);

		PWM_Gen_Dual(freq,duty);
		while(1)
		{
			UART_Recieve(UARTA1,dummy,1,BLOCKING);
			if(dummy[0] == ESC_KEY)
			{
				PWM_Test();
			}
		}
		break;
	default :
		break;
	}

}
