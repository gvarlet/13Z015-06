/***********************  I n c l u d e  -  F i l e  ************************
 *
 *         Name: mscan_drv.h
 *
 *       Author: kp
 *        $Date: 2003/07/11 09:25:47 $
 *    $Revision: 2.3 $
 *
 *  Description: Header file for MSCAN driver
 *               - MSCAN specific status codes
 *               - MSCAN function prototypes
 *
 *     Switches: _ONE_NAMESPACE_PER_DRIVER_
 *               _LL_DRV_
 *				 B5
 *---------------------------------------------------------------------------
 * (c) Copyright 2002 by MEN mikro elektronik GmbH, Nuernberg, Germany
 ****************************************************************************/
/*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _MSCAN_DRV_H
#define _MSCAN_DRV_H

#ifdef __cplusplus
      extern "C" {
#endif

/*-----------------------------------------+
|  TYPEDEFS                                |
+-----------------------------------------*/
typedef struct {
	MSCAN_FILTER filter1;
	MSCAN_FILTER filter2;
} MSCAN_SETFILTER_PB;

typedef struct {
	u_int32 objNr;
	MSCAN_DIR dir;
	u_int32 qEntries;
	MSCAN_FILTER filter;
} MSCAN_CONFIGMSG_PB;

typedef struct {
	u_int8 brp;
	u_int8 sjw;
	u_int8 tseg1;
	u_int8 tseg2;
	u_int8 spl;
} MSCAN_SETBUSTIMING_PB;

typedef struct {
	MSCAN_BITRATE bitrate;
	u_int32 spl;
} MSCAN_SETBITRATE_PB;

typedef struct {
	u_int32 objNr;
	int32 timeout;
	MSCAN_FRAME msg;			/* out for mscan_read_msg */
} MSCAN_READWRITEMSG_PB;

typedef struct {
	u_int32 errCode;			/* out */
	u_int32 objNr;				/* out */
} MSCAN_READERROR_PB;

typedef struct {
	u_int32 objNr;
	u_int32 signal;
} MSCAN_SIGNAL_PB;

typedef struct {
	u_int32 objNr;
	u_int32 entries;			/* out */
	MSCAN_DIR direction;		/* out */
} MSCAN_QUEUESTATUS_PB;

typedef struct {
	u_int32 objNr;
	int txabort;
} MSCAN_QUEUECLEAR_PB;

typedef struct {
	u_int8 txErrCnt;
	u_int8 rxErrCnt;
} MSCAN_ERRORCOUNTERS_PB;


/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/
/* ICANL2 specific status codes (STD) */	/* G,S: S=setstat, G=getstat */
#define MSCAN_GETCANCLK 	(M_DEV_OF+0x00) /* G  : get CAN clock rate */
#define MSCAN_CLEARBUSOFF	(M_DEV_OF+0x01) /*   S: get clear bus off */
#define MSCAN_ENABLE		(M_DEV_OF+0x02) /*   S: enable/disable CAN */
#define MSCAN_LOOPBACK		(M_DEV_OF+0x03) /*   S: enable/disable loopback */
#define MSCAN_NODESTATUS 	(M_DEV_OF+0x04) /* G  : get node status */
#define MSCAN_MAXIRQTIME 	(M_DEV_OF+0x10) /* G,S: for internal tests */
/* ICANL2 specific status codes (BLK) */		/* S,G: S=setstat, G=getstat */
#define MSCAN_SETFILTER 	(M_DEV_BLK_OF+0x00) /*   S: set filter */
#define MSCAN_CONFIGMSG 	(M_DEV_BLK_OF+0x01) /*   S: config object */
#define MSCAN_SETBUSTIMING 	(M_DEV_BLK_OF+0x02) /*   S: set bustiming */
#define MSCAN_READMSG		(M_DEV_BLK_OF+0x03) /* G  : read single frame */
#define MSCAN_WRITEMSG		(M_DEV_BLK_OF+0x04) /*   S: write single frame */
#define MSCAN_READERROR		(M_DEV_BLK_OF+0x05) /* G  : read error entry */
#define MSCAN_SETRCVSIG		(M_DEV_BLK_OF+0x06) /*   S: install rx signal */
#define MSCAN_SETXMTSIG		(M_DEV_BLK_OF+0x07) /*   S: install tx signal */
#define MSCAN_CLRRCVSIG		(M_DEV_BLK_OF+0x08) /*   S: remove rx signal */
#define MSCAN_CLRXMTSIG		(M_DEV_BLK_OF+0x09) /*   S: remove tx signal */
#define MSCAN_QUEUESTATUS	(M_DEV_BLK_OF+0x0a) /* G  : get queue status */
#define MSCAN_QUEUECLEAR	(M_DEV_BLK_OF+0x0b) /*   S: clear queue */
#define MSCAN_SETBITRATE 	(M_DEV_BLK_OF+0x0c) /*   S: set bitrate */
#define MSCAN_ERRORCOUNTERS	(M_DEV_BLK_OF+0x0d) /* G  : read error counters */
#define MSCAN_DUMPINTERNALS	(M_DEV_BLK_OF+0x0e) /* G  : dump internals */

/*-----------------------------------------+
|  PROTOTYPES                              |
+-----------------------------------------*/
#define _MSCAN_GLOBNAME(var,name) var##name

#ifndef _ONE_NAMESPACE_PER_DRIVER_
# define MSCAN_GLOBNAME(var,name) _MSCAN_GLOBNAME(var,name)
#else
# define MSCAN_GLOBNAME(var,name) _MSCAN_GLOBNAME(MSCAN,name)
#endif

/* variant specific function names */
#define _MSCAN_GetEntry	MSCAN_GLOBNAME(MSCAN_VARIANT,GetEntry)

#ifdef _LL_DRV_
	extern void _MSCAN_GetEntry(LL_ENTRY* drvP);
#endif /* _LL_DRV_ */



#ifdef __cplusplus
      }
#endif

#endif /* _MSCAN_DRV_H */

