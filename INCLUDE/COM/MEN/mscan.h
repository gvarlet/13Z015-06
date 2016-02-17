/***********************  I n c l u d e  -  F i l e  ************************
 *  
 *         Name: mscan.h
 *
 *       Author: kp
 *        $Date: 2010/02/25 18:04:50 $
 *    $Revision: 2.4 $
 * 
 *  Description: register layout for MSCAN (Motorola Scalable CAN/MEN Boromir)
 *                      
 *     Switches: MSCAN_IS_Z15  	   set if using MEN's FPGA implementation
 *				 MSCAN_IS_ODIN	   set if using MGT5100 implementation	
 *
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: mscan.h,v $
 * Revision 2.4  2010/02/25 18:04:50  amorbach
 * R: driver ported to MDIS5, new MDIS_API and men_typs
 * M: for backward compatibility to MDIS4 optionally define new types
 *
 * Revision 2.3  2004/02/11 17:12:00  UFranke
 * added
 *  - RIER bit definitions
 *
 * Revision 2.2  2003/01/29 14:03:22  kp
 * added MSCAN_RFLG_CSCIF
 *
 * Revision 2.1  2003/01/07 15:33:16  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2002 by MEN mikro elektronik GmbH, Nuernberg, Germany 
 ****************************************************************************/

#ifndef _MSCAN_H 
#define _MSCAN_H 

/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/

#ifdef MSCAN_IS_ODIN
/*--- registers offset for MGT5100 MSCAN implementation ---*/

/* all registers are 8 bit wide */
#define MSCAN_CTL0		0x00	/* Control 0 */
#define MSCAN_CTL1		0x01	/* Control 1 */
#define MSCAN_BTR0		0x04	/* Bustiming 0 */
#define MSCAN_BTR1		0x05	/* Bustiming 1 */
#define MSCAN_RFLG		0x08	/* Rx Flag */
#define MSCAN_RIER		0x09	/* Rx interrupt enable */
#define MSCAN_TFLG		0x0c	/* Tx Flag */
#define MSCAN_TIER		0x0d	/* Tx interrupt enable */
#define MSCAN_TARQ		0x10	/* Transmit abort request */
#define MSCAN_TAAK		0x11	/* Transmit abort ack */
#define MSCAN_BSEL		0x14	/* Tx buffer select */
#define MSCAN_IDAC		0x15	/* ID acceptance control */
#define MSCAN_RXER		0x1c	/* Rx error */
#define MSCAN_TXER		0x1d	/* Tx error */

#define MSCAN_IDAR0		0x20	/* ID acceptance 0 */
#define MSCAN_IDAR1		0x21	/* ID acceptance 1 */
#define MSCAN_IDAR2		0x24	/* ID acceptance 2 */
#define MSCAN_IDAR3		0x25	/* ID acceptance 3 */

#define MSCAN_IDMR0		0x28	/* ID mask 0 */
#define MSCAN_IDMR1		0x29	/* ID mask 1 */
#define MSCAN_IDMR2		0x2c	/* ID mask 2 */
#define MSCAN_IDMR3		0x2d	/* ID mask 3 */

#define MSCAN_IDAR4		0x30	/* ID acceptance 4 */
#define MSCAN_IDAR5		0x31	/* ID acceptance 5 */
#define MSCAN_IDAR6		0x34	/* ID acceptance 6 */
#define MSCAN_IDAR7		0x35	/* ID acceptance 7 */

#define MSCAN_IDMR4		0x38	/* ID mask 4 */
#define MSCAN_IDMR5		0x39	/* ID mask 5 */
#define MSCAN_IDMR6		0x3c	/* ID mask 6 */
#define MSCAN_IDMR7		0x3d	/* ID mask 7 */

#define MSCAN_RXIDR0	0x40	/* Rx ID 0 */
#define MSCAN_RXIDR1	0x41	/* Rx ID 1 */
#define MSCAN_RXIDR2	0x44	/* Rx ID 2 */
#define MSCAN_RXIDR3	0x45	/* Rx ID 3 */

#define MSCAN_RXDSR0	0x48	/* Rx data byte 0 */
#define MSCAN_RXDSR1	0x49	/* Rx data byte 1 */
#define MSCAN_RXDSR2	0x4c	/* Rx data byte 2 */
#define MSCAN_RXDSR3	0x4d	/* Rx data byte 3 */
#define MSCAN_RXDSR4	0x50	/* Rx data byte 4 */
#define MSCAN_RXDSR5	0x51	/* Rx data byte 5 */
#define MSCAN_RXDSR6	0x54	/* Rx data byte 6 */
#define MSCAN_RXDSR7	0x55	/* Rx data byte 7 */

#define MSCAN_RXDLR		0x58	/* Rx data length */
#define MSCAN_RXTIMH	0x5c	/* Rx timestamp high */
#define MSCAN_RXTIML	0x5d	/* Rx timestamp low */

#define MSCAN_TXIDR0	0x60	/* Tx ID 0 */
#define MSCAN_TXIDR1	0x61	/* Tx ID 1 */
#define MSCAN_TXIDR2	0x64	/* Tx ID 2 */
#define MSCAN_TXIDR3	0x65	/* Tx ID 3 */

#define MSCAN_TXDSR0	0x68	/* Tx data byte 0 */
#define MSCAN_TXDSR1	0x69	/* Tx data byte 1 */
#define MSCAN_TXDSR2	0x6c	/* Tx data byte 2 */
#define MSCAN_TXDSR3	0x6d	/* Tx data byte 3 */
#define MSCAN_TXDSR4	0x70	/* Tx data byte 4 */
#define MSCAN_TXDSR5	0x71	/* Tx data byte 5 */
#define MSCAN_TXDSR6	0x74	/* Tx data byte 6 */
#define MSCAN_TXDSR7	0x75	/* Tx data byte 7 */

#define MSCAN_TXDLR		0x78	/* Tx data length */
#define MSCAN_TXBPR		0x79	/* Tx buffer priority */
#define MSCAN_TXTIMH	0x7c	/* Tx timestamp high */
#define MSCAN_TXTIML	0x7d	/* Tx timestamp low */
#endif /* MSCAN_IS_ODIN */

#ifdef MSCAN_IS_Z15
/*--- registers offset for MEN FPGA implementation "boromir" ---*/

/* all registers are 8 bit wide */
#define MSCAN_CTL0		0x00	/* Control 0 */
#define MSCAN_CTL1		0x04	/* Control 1 */
#define MSCAN_BTR0		0x08	/* Bustiming 0 */
#define MSCAN_BTR1		0x0c	/* Bustiming 1 */
#define MSCAN_RFLG		0x10	/* Rx Flag */
#define MSCAN_RIER		0x14	/* Rx interrupt enable */
#define MSCAN_TFLG		0x18	/* Tx Flag */
#define MSCAN_TIER		0x1c	/* Tx interrupt enable */
#define MSCAN_TARQ		0x20	/* Transmit abort request */
#define MSCAN_TAAK		0x24	/* Transmit abort ack */
#define MSCAN_BSEL		0x28	/* Tx buffer select */
#define MSCAN_IDAC		0x2c	/* ID acceptance control */
#define MSCAN_RXER		0x38	/* Rx error */
#define MSCAN_TXER		0x3c	/* Tx error */

#define MSCAN_IDAR0		0x40	/* ID acceptance 0 */
#define MSCAN_IDAR1		0x44	/* ID acceptance 1 */
#define MSCAN_IDAR2		0x48	/* ID acceptance 2 */
#define MSCAN_IDAR3		0x4c	/* ID acceptance 3 */
#define MSCAN_IDAR4		0x50	/* ID acceptance 4 */
#define MSCAN_IDAR5		0x54	/* ID acceptance 5 */
#define MSCAN_IDAR6		0x58	/* ID acceptance 6 */
#define MSCAN_IDAR7		0x5c	/* ID acceptance 7 */

#define MSCAN_IDMR0		0x60	/* ID mask 0 */
#define MSCAN_IDMR1		0x64	/* ID mask 1 */
#define MSCAN_IDMR2		0x68	/* ID mask 2 */
#define MSCAN_IDMR3		0x6c	/* ID mask 3 */
#define MSCAN_IDMR4		0x70	/* ID mask 4 */
#define MSCAN_IDMR5		0x74	/* ID mask 5 */
#define MSCAN_IDMR6		0x78	/* ID mask 6 */
#define MSCAN_IDMR7		0x7c	/* ID mask 7 */

#define MSCAN_RXIDR0	0x80	/* Rx ID 0 */
#define MSCAN_RXIDR1	0x84	/* Rx ID 1 */
#define MSCAN_RXIDR2	0x88	/* Rx ID 2 */
#define MSCAN_RXIDR3	0x8c	/* Rx ID 3 */

#define MSCAN_RXDSR0	0x90	/* Rx data byte 0 */
#define MSCAN_RXDSR1	0x94	/* Rx data byte 1 */
#define MSCAN_RXDSR2	0x98	/* Rx data byte 2 */
#define MSCAN_RXDSR3	0x9c	/* Rx data byte 3 */
#define MSCAN_RXDSR4	0xa0	/* Rx data byte 4 */
#define MSCAN_RXDSR5	0xa4	/* Rx data byte 5 */
#define MSCAN_RXDSR6	0xa8	/* Rx data byte 6 */
#define MSCAN_RXDSR7	0xac	/* Rx data byte 7 */

#define MSCAN_RXDLR		0xb0	/* Rx data length */
/*#define MSCAN_RXTIMH	0x5c	not implemented! */ /* Rx timestamp high */
/*#define MSCAN_RXTIML	0x5d	not implemented! */ /* Rx timestamp low */

#define MSCAN_TXIDR0	0xc0	/* Tx ID 0 */
#define MSCAN_TXIDR1	0xc4	/* Tx ID 1 */
#define MSCAN_TXIDR2	0xc8	/* Tx ID 2 */
#define MSCAN_TXIDR3	0xcc	/* Tx ID 3 */

#define MSCAN_TXDSR0	0xd0	/* Tx data byte 0 */
#define MSCAN_TXDSR1	0xd4	/* Tx data byte 1 */
#define MSCAN_TXDSR2	0xd8	/* Tx data byte 2 */
#define MSCAN_TXDSR3	0xdc	/* Tx data byte 3 */
#define MSCAN_TXDSR4	0xe0	/* Tx data byte 4 */
#define MSCAN_TXDSR5	0xe4	/* Tx data byte 5 */
#define MSCAN_TXDSR6	0xe8	/* Tx data byte 6 */
#define MSCAN_TXDSR7	0xec	/* Tx data byte 7 */

#define MSCAN_TXDLR		0xf0	/* Tx data length */
#define MSCAN_TXBPR		0xf4	/* Tx buffer priority */
/* #define MSCAN_TXTIMH	0x7c	not implemented! */ /* Tx timestamp high */
/* #define MSCAN_TXTIML	0x7d	not implemented! */ /* Tx timestamp low */
#endif /* MSCAN_IS_ODIN */

#ifndef MSCAN_CTL0
# error "mscan.h: must specify MSCAN_IS_xxx macro!"
#endif

/*--- register bits ---*/

#define MSCAN_CTL0_INITRQ	0x01	/* init mode request */

#define MSCAN_CTL1_INITAK	0x01	/* init mode ack */
#define MSCAN_CTL1_LOOPB	0x20	/* loopback mode */
#define MSCAN_CTL1_CANE		0x80 	/* enable CAN */

#define MSCAN_RFLG_RXF		0x01	/* receive fifo not empty */
#define MSCAN_RFLG_OVRIF	0x02	/* receive buffer overrun */
#define MSCAN_RFLG_CSCIF	0x40	/* status change interrupt */

#define MSCAN_RIER_WUPIE					0x80 /* wakeup */
#define MSCAN_RIER_CSCIE					0x40 /* CAN status change */
#define MSCAN_RIER_RSTATE_LEAVE_BOF			0x10 /* leave bus off only */
#define MSCAN_RIER_RSTATE_LEAVE_BOF_RXERR	0x20 /* leave bus off or rx err only */
#define MSCAN_RIER_RSTATE_ALL				0x30 /* all RX changes */
#define MSCAN_RIER_TSTATE_LEAVE_BOF			0x08 /* leave bus off only */          
#define MSCAN_RIER_TSTATE_LEAVE_BOF_RXERR	0x04 /* leave bus off or tx err only */
#define MSCAN_RIER_TSTATE_ALL				0x0C /* all TX changes */              
#define MSCAN_RIER_OVRIE					0x02 /* overrun */
#define MSCAN_RIER_RXFIE					0x01 /* RX full  */

/*-----------------------------------------+
|  BACKWARD COMPATIBILITY TO MDIS4         |
+-----------------------------------------*/
#ifndef U_INT32_OR_64
    /* we have an MDIS4 men_types.h and mdis_api.h included */
    /* only 32bit compatibility needed!                     */
    #define INT32_OR_64     int32
    #define U_INT32_OR_64   u_int32
    typedef INT32_OR_64     MDIS_PATH;
#endif /* U_INT32_OR_64 */


#endif /* _MSCAN_H */
