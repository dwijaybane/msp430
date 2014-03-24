/*
 * RFID.c
 *
 *  Created on: Mar 11, 2014
 *      Author: JB ELECTRONICS
 */
int prsnt_1 = 0;
int prsnt_2 = 0;
int prsnt_3 = 0;
int prsnt_4 = 0;

void RFID_Str_Cmp(char temp[14])
{
	char temp1[10];
	int status;
	unsigned int j,i;
	j=0;
	for(i=1;i<11;i++)
	{
		temp1[j++] = temp[i];
	}
	temp[10] = '\0';

	status = RFID_Id_Compare(temp1,RFID_1);
	if(status == 1)
	{
		if(prsnt_1 == 0)
		{
			printf(UARTA0,"551       PANKIL    \n\r");
			printf(UARTA0,"\n\rPress ESC To EXIT\n\r");
			status = 0;
			prsnt_1 = 1;
		}
	}

	status = RFID_Id_Compare(temp1,RFID_2);
	if(status == 1)
	{
		if(prsnt_2 == 0)
		{
			printf(UARTA0,"552       NAISHADH   \n\r");
			printf(UARTA0,"\n\rPress ESC To EXIT\n\r");
			status = 0;
			prsnt_2 = 1;
		}
	}

	status = RFID_Id_Compare(temp1,RFID_3);
	if(status == 1)
	{
		if(prsnt_3 == 0)
		{
			printf(UARTA0,"553       MADHAV    \n\r");
			printf(UARTA0,"\n\rPress ESC To EXIT\n\r");
			status = 0;
			prsnt_3 = 1;
		}
	}

	status = RFID_Id_Compare(temp1,RFID_4);
	if(status == 1)
	{
		if(prsnt_4 == 0)
		{
			printf(UARTA0,"554       JAYDEEP   \n\r");
			printf(UARTA0,"\n\rPress ESC To EXIT\n\r");
			status = 0;
			prsnt_4 = 1;
		}
	}
}

void Main_Menu()
{
	int sw_value = 0,i,length;
	Esc_flag = 0;
	char RFID_Data[14]={0};
	printf(UARTA0,"\x1b[2J");
	printf(UARTA0,"          RFID ATTENDENCE SYSTEM             |------------------------|\n\r          Developed For : EDUTECH SYSTEMS    | SW6(ESC) SW3(2) SW2(1) |\n\r                                             |   O       O       O    |\n\r1.ATTENDENCE MODE                            |------------------------|\n\r2.LOG\n\r");
	sw_value = Detect_Matkb();
	if(sw_value == 2)
	{
		if(Esc_flag == 1)
		{
			Esc_flag = 0;
		}
		printf(UARTA0,"\x1b[2J");
		printf(UARTA0,"          RFID ATTENDENCE SYSTEM             |------------------------|\n\r          Developed For : EDUTECH SYSTEMS    | SW6(ESC) SW3(2) SW2(1) |\n\r                                             |   O       O       O    |\n\rID No.    Name                               |------------------------|\n\r");
		for(i=0;i<14;i++)
		{
			RFID_data[i] = 0;
		}
		while(1)
		{
			UART_Recieve(UARTA1,RFID_Data,14,NONE_BLOCKING);
			if(RFID_Data[0] == 2 && RFID_Data[13]!=0)
			{
				printf(UARTA0,"\x1b[2J");
				printf(UARTA0,"          RFID ATTENDENCE SYSTEM             |------------------------|\n\r          Developed For : EDUTECH SYSTEMS    | SW6(ESC) SW3(2) SW2(1) |\n\r                                             |   O       O       O    |\n\rID No.    Name                               |------------------------|\n\r");
				//index = 0;
				RFID_Str_Cmp(RFID_Data);
				for(i=0;i<14;i++)
				{
					RFID_Data[i] = 0;
				}
			}
			if(Esc_flag == 1)
			{
				Main_Menu();
			}

		}
	}
	else if(sw_value == 3)
	{
		if(Esc_flag == 1)
		{
			Esc_flag = 0;
		}
		printf(UARTA0,"\x1b[2J");
		printf(UARTA0,"          RFID ATTENDENCE SYSTEM             |------------------------|\n\r          Developed For : EDUTECH SYSTEMS    | SW6(ESC) SW3(2) SW2(1) |\n\r                                             |   O       O       O    |\n\rPresent List                                 |------------------------|\n\r");
		printf(UARTA0,"ID No.    Name      \n\r\n\r");
		if(prsnt_1 ==1)
		{
			printf(UARTA0,"551       PANKIL    \n\r");
		}
		if(prsnt_2 == 1)
		{
			printf(UARTA0,"552       NAISHADH   \n\r");
		}
		if(prsnt_3 == 1)
		{
			printf(UARTA0,"553       MADHAV    \n\r");
		}
		if(prsnt_4 == 1)
		{
			printf(UARTA0,"554       JAYDEEP   \n\r");
		}
		printf(UARTA0,"Absent List\n\r");
		printf(UARTA0,"ID No.    Name      \n\r\n\r");
		if(prsnt_1 ==0)
		{
			printf(UARTA0,"551       PANKIL    \n\r");
		}
		if(prsnt_2 == 0)
		{
			printf(UARTA0,"552       NAISHADH   \n\r");
		}
		if(prsnt_3 == 0)
		{
			printf(UARTA0,"553       MADHAV    \n\r");
		}
		if(prsnt_4 == 0)
		{
			printf(UARTA0,"554       JAYDEEP   \n\r");
		}
	}
	printf(UARTA0,"Press ESC to Exit\n\r");
	while(Esc_flag == 0);
	Main_Menu();
}
int RFID_Id_Compare(char *no1,char *no2)
{
	int cnt = 0;
	while (cnt !=10)
		{
			cnt++;
			if (*no2==*no1 )
			{
				no1++;
				no2++;
			}
			else
			{
				return(0);
			}
		}
		return(1);
}
