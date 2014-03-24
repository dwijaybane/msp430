/*
 * RFID.h
 *
 *  Created on: Mar 11, 2014
 *      Author: JB ELECTRONICS
 */

#ifndef RFID_H_
#define RFID_H_

extern void RFID_Str_Cmp(char temp[14]);
extern int RFID_Id_Compare(char *no1,char *no2);
extern void Main_Menu();
extern void RFID_Receive();
#endif /* RFID_H_ */
