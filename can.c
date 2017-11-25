
#include "can.h"

CAN_frame trame1;
CAN_frame trame_buf[10];
ErrorFlags errorStatus;
unsigned char i_wr;
unsigned char i_rd;

void CAN_config(void)
{
	C1CTRLbits.REQOP = 0b100;
	while(C1CTRLbits.OPMOD != 0b100);

	C1CTRLbits.CANCKS = 1;				// Fcan = Fcy = Fosc/4
	
	/**** Configuration Bit Timing ****/
	C1CFG1bits.BRP = 2;					// prescaler pour Tq = 0,2µs
	C1CFG2bits.PRSEG = 6;				// segment de propagation = 7.Tq
	C1CFG2bits.SEG1PH = 5;				// segment de phase 1 = 6.Tq
	C1CGF2bits.SEG2PHTS = 1;			// Prgrammation libre du segment de phase 2
	C1CFG2bits.SEG2PH = 5;				// segment de phase 2 = 6.Tq
	C1CFG2bits.SAM = 1;					// 3 échantillons par bits
	C1CFG1bits.SJW = 1;					// saut de resynchronisation = 2.Tq
	C1CFG2bits.WAKFIL = 0;				// Pas de réveil sur filtrage de trame
	
	/**** Configuration filtres et masques ****/
	// Boite RXB0 (filtrage 0x100 -> 0x10F)
	C1RXM0SID = 0x1FC1;   				// 0b 0001 1111 1100 0001
	C1RXF0SID = 0x0400;					// 0b 0000 0100 0000 0000
	// Boite RXB1 (filtrage 0x200 -> 0x21F)
	C1RXM1SID = 0x1F81;					// 0b 0001 1111 1000 0001 
	C1RXF2SID = 0x0800;					// 0b 0000 1000 0000 0000
	
	
	/**** Configuration interruptions ****/
	// Activation interruptions du CAN
	IEC1bits.C1IE = 1;
	
	// Activation interruptions uniquement en réception de trames
	C1INTEbits.RX0IE = 1;
	C1INTEbits.RX1IE = 1;
	C1INTEbits.ERRIE = 1;
	
	
	C1CTRLbits.REQOP = 0b000;
	while(C1CTRLbits.OPMOD != 0b000);
}
	
void envoi_CAN(CAN_frame CAN_mes)
{
	while(C1TX0CONbits.TXREQ);
	C1TX0SID = ((CAN_mes.STDID & 0b0000 0111 1100 0000) << 5) | ((CAN_mes.STDID & 0b111111) << 2) | (CAN_mes.RTR << 1);
	
	C1TX0DLC = CAN_mes.length << 3;		// ou C1TX0DLCbits.DLC = CAN_mes.length;
	
	C1TX0B1 = (CAN_mes.data[1] << 8) | CAN_mes.data[0];
	C1TX0B2 = (CAN_mes.data[3] << 8) | CAN_mes.data[2];
	C1TX0B3 = (CAN_mes.data[5] << 8) | CAN_mes.data[4];
	C1TX0B4 = (CAN_mes.data[7] << 8) | CAN_mes.data[6];
	
	C1TX0CONbits.TXREQ = 1;
}
	
unsigned char envoi_CAN2(CAN_frame CAN_mes)
{
	if(C1TX0CONbits.TXREQ) return 0;
	C1TX0SID = ((CAN_mes.STDID & 0b0000 0111 1100 0000) << 5) | ((CAN_mes.STDID & 0b111111) << 2) | (CAN_mes.RTR << 1);
	
	C1TX0DLC = CAN_mes.length << 3;		// ou C1TX0DLCbits.DLC = CAN_mes.length;
	
	C1TX0B1 = (CAN_mes.data[1] << 8) | CAN_mes.data[0];
	C1TX0B2 = (CAN_mes.data[3] << 8) | CAN_mes.data[2];
	C1TX0B3 = (CAN_mes.data[5] << 8) | CAN_mes.data[4];
	C1TX0B4 = (CAN_mes.data[7] << 8) | CAN_mes.data[6];
	
	C1TX0CONbits.TXREQ = 1;
	return 1;
}
	
unsigned char envoi_CAN3(CAN_frame CAN_mes)
{
	if(!C1TX0CONbits.TXREQ)
	{
		C1TX0SID = ((CAN_mes.STDID & 0b0000 0111 1100 0000) << 5) | ((CAN_mes.STDID & 0b111111) << 2) | (CAN_mes.RTR << 1);
		
		C1TX0DLC = CAN_mes.length << 3;		// ou C1TX0DLCbits.DLC = CAN_mes.length;
		
		C1TX0B1 = (CAN_mes.data[1] << 8) | CAN_mes.data[0];
		C1TX0B2 = (CAN_mes.data[3] << 8) | CAN_mes.data[2];
		C1TX0B3 = (CAN_mes.data[5] << 8) | CAN_mes.data[4];
		C1TX0B4 = (CAN_mes.data[7] << 8) | CAN_mes.data[6];
		
		C1TX0CONbits.TXREQ = 1;
		return 1;
	}
	else if(!C1TX1CONbits.TXREQ)
	{
		C1TX1SID = ((CAN_mes.STDID & 0b0000 0111 1100 0000) << 5) | ((CAN_mes.STDID & 0b111111) << 2) | (CAN_mes.RTR << 1);
		
		C1TX1DLC = CAN_mes.length << 3;		// ou C1TX1DLCbits.DLC = CAN_mes.length;
		
		C1TX1B1 = (CAN_mes.data[1] << 8) | CAN_mes.data[0];
		C1TX1B2 = (CAN_mes.data[3] << 8) | CAN_mes.data[2];
		C1TX1B3 = (CAN_mes.data[5] << 8) | CAN_mes.data[4];
		C1TX1B4 = (CAN_mes.data[7] << 8) | CAN_mes.data[6];
		
		C1TX1CONbits.TXREQ = 1;
		return 2;
	}
	else if(!C1TX2CONbits.TXREQ)
	{
		C1TX2SID = ((CAN_mes.STDID & 0b0000 0111 1100 0000) << 5) | ((CAN_mes.STDID & 0b111111) << 2) | (CAN_mes.RTR << 1);
		
		C1TX2DLC = CAN_mes.length << 3;		// ou C1TX2DLCbits.DLC = CAN_mes.length;
		
		C1TX2B1 = (CAN_mes.data[1] << 8) | CAN_mes.data[0];
		C1TX2B2 = (CAN_mes.data[3] << 8) | CAN_mes.data[2];
		C1TX2B3 = (CAN_mes.data[5] << 8) | CAN_mes.data[4];
		C1TX2B4 = (CAN_mes.data[7] << 8) | CAN_mes.data[6];
		
		C1TX2CONbits.TXREQ = 1;
		return 3;
	}
	else return 0;
}	

/* V.1 
Le mécanisme d'interruption permet de ne pas laisser le CPU en attente d'une trame (scrutation du bit RXFUL de chaque boite). Si on attend cette condition, on ne fait rien d'autre.
L'interruption permet de traiter une tâche principale ou d'autres tâches et de ne traiter la réception d'une trame que lorsqu'elle effective.

  V.2 Organigramme

  V.3 Fonction d'interruption du CAN
*/	

void __attribute__((__interrupt__)) _C1Interrupt(void)
{
	
	IFS1bits.C1IF = 0;
	
	if(C1INTFbits.RXB0IF)
	{
		// Remise à 0 du flag d'interruption
		C1INTFbits.RXB0IF = 0;
		
		// Clignottement LED 0
		LATBbits.LATB0 = ~LATBbits.LATB0;
		
		// Récupération du message
		trame1.STDID = C1RX0SID >> 2;
		trame1.RTR = C1RX0SIDbits.SRR;
		trame1.length = C1RX0DLCbits.DLC;
		trame1.data[0] = C1RX0B1 & 0x00FF;
		trame1.data[1] = C1RX0B1 >> 8;
		trame1.data[2] = C1RX0B2 & 0x00FF;
		trame1.data[3] = C1RX0B2 >> 8;
		trame1.data[4] = C1RX0B3 & 0x00FF;
		trame1.data[5] = C1RX0B3 >> 8;
		trame1.data[6] = C1RX0B4 & 0x00FF;
		trame1.data[7] = C1RX0B4 >> 8;
		
		C1RX0CONbits.RXFUL = 0;
		
		if (i_wr >= 10) i_wr = 0;
		trame_buf[i_wr++] = trame1;
	}
	if(C1INTFbits.RXB1IF)
	{
		// Remise à 0 du flag d'interruption
		C1INTFbits.RXB1IF = 0;
		
		// Clignottement LED 0
		LATBbits.LATB1 = ~LATBbits.LATB1;
		
		// Récupération du message
		trame1.STDID = C1RX1SID >> 2;
		trame1.RTR = C1RX1SIDbits.SRR;
		trame1.length = C1RX1DLCbits.DLC;
		trame1.data[0] = C1RX1B1 & 0x00FF;
		trame1.data[1] = C1RX1B1 >> 8;
		trame1.data[2] = C1RX1B2 & 0x00FF;
		trame1.data[3] = C1RX1B2 >> 8;
		trame1.data[4] = C1RX1B3 & 0x00FF;
		trame1.data[5] = C1RX1B3 >> 8;
		trame1.data[6] = C1RX1B4 & 0x00FF;
		trame1.data[7] = C1RX1B4 >> 8;
		
		C1RX1CONbits.RXFUL = 0;
		
		if (i_wr >= 10) i_wr = 0;
		trame_buf[i_wr++] = trame1;
	}
	if (C1INTFbits.ERRIF)
	{
		C1INTFbits.ERRIF = 0;
		if(C1INTFbits.RX0OVR || C1INTFbits.RX1OVR) errorStatus.overflow = 1;
		if (C1INTFbits.RXEP || C1INTFbits.TXEP) errorStatus.passive = 1;
		if(C1INTFbits.TXWAR || C1INTFbits.RXWAR || C1INTFbits.EWARN) errorStatus.warning = 1;
		if(C1INTFbits.IVRIF) errorStatus.invalid = 1;
		if(C1INTFbits.TXBO) errorStatus.off = 1;
	}
	
}

unsigned char CAN_recu(void)
{
	if(i_wr != i_rd) return 1;
	else return 0;
}


void lecture_CAN(CAN_frame *t)
{
	if(i_rd >= 10) i_rd = 0; 
	t = trame_buf[i_rd++];
}



















	
	
	
	
	
	
	
	
	
	
	