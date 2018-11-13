/*********************  P r o g r a m  -  M o d u l e ***********************/
/*!  
 *        \file  mscan_drv.c
 *
 *      \author  klaus.popp@men.de
 *        $Date: 2013/03/27 09:54:36 $
 *    $Revision: 1.19 $
 * 
 *  	 \brief  Low level driver for MSCAN and MEN Boromir controller chips
 *
 *	This driver provides a direct layer 2 access to the CAN
 *	controller. All features of the MSCAN are supported
 *
 *	The user library mscan_api should be used to access this driver
 *			
 *	This driver will not work without interrupts!
 *
 *	The driver uses NON LOCKING mode to allow multiple processes to
 *	wait for messages objects simultanously.
 * 
 *	Supports buffer queues for each of the 9 message objects
 *	plus one virtual "error object".
 *
 *	See mscan_api documentation for further details.
 *
 * 
 *     Switches: see mscan_int.h
 */
/*-------------------------------[ History ]---------------------------------
 *
 * $Log: mscan_drv.c,v $
 * Revision 1.19  2013/03/27 09:54:36  gvarlet
 * R: When more than 1 CAN port is used on a multitask application, some frames
 *    are output several times, due to a bad IRQ masking and unmasking.
 * M: Remove useless and unbalanced OSS_IrqMaskR/OSS_IrqRestore
 *    on RX & TX FIFO managment.
 *
 * Revision 1.18  2011/09/26 11:10:36  gv
 * R : Wrong cast leads to errors under Windows.
 * M : Cast to void* instead of u_int32*
 *
 * Revision 1.17  2011/09/16 15:01:33  gvarlet
 * R : CAN Odin interface for MPC5200 was not outputing frames on the physical
 *     interface.
 * M : Enable the PSC2_CAN bit on GPIOPCR register of MPC5200.
 *
 * Revision 1.16  2010/12/09 09:14:39  CKauntz
 * R: OSS_IrqRestore not called in some conditions
 * M: Moved OSS_IrqRestore out of else condition
 *
 * Revision 1.15  2010/02/25 18:04:21  amorbach
 * R: Porting to MDIS5
 * M: Changed according to MDIS Porting Guide 0.8
 *
 * Revision 1.14  2009/11/09 17:15:58  KSchneider
 * R: MSCAN driver used a baud rate prescaler of 1 for some canclock/baud rate constellations. This is not supported by some cores.
 * M: Use a minimum BRP of 2 per default (configurable)
 *
 * Revision 1.13  2009/01/22 13:34:15  AWanka
 * R: Makro MSSETMASK/MSCLRMASK was expanded by the assembler instruction eieio.    MSSETMASK between a IF/ELSE construct was without curly brackets.
 * M: Added curly brackets at IF/ELSE construct.
 *
 * Revision 1.12  2007/03/01 15:53:32  SYao
 * correct a type mismatch bug
 *
 * Revision 1.11  2007/02/12 15:55:13  SYao
 * add support to SC15 board.
 *
 * Revision 1.10  2005/03/24 10:18:28  kp
 * added possibility to select baudrate codes 20k and 10k (only possible if MSCAN
 * clock <= 16MHz)
 *
 * Revision 1.9  2004/06/14 11:58:06  kp
 * - MDIS4/2004 compliance (use OSS_IrqMaskR/Restore)
 * - eliminated GetTimeBase() calls (OS-9 specific)
 *
 * Revision 1.8  2004/05/24 09:07:06  dpfeuffer
 * some casts
 *
 * Revision 1.7  2003/10/02 14:21:41  kp
 * Bug fix in Tx buffer scheduling algorithm: In certain cases, it could happen
 * that buffer scheduling stops. Write queue got full in this case.
 *
 * Revision 1.6  2003/07/11 09:25:40  kp
 * added support for MSCAN_DUMPINTERNALS
 *
 * Revision 1.5  2003/04/15 12:18:07  kp
 * Compile GetTimeBase only for Ultra-C
 *
 * Revision 1.4  2003/03/18 12:42:20  kp
 * 1) WaitRxFifoEntry/MscanWriteMsg: fixed problem when timeout and
 * rx msg/fifo available conditions arrived at the same time.
 * 2) Allow to read error counters in online mode if MSCAN_IS_Z15
 * 3) added forgotten baudrate 100kBit/s
 *
 * Revision 1.3  2003/02/07 13:16:33  kp
 * SET_BITRATE: use BTR values from table in case of 32MHz clock
 *
 * Revision 1.2  2003/02/03 10:42:48  kp
 * First alpha release to SH Winding
 *
 * Revision 1.1  2003/01/29 14:03:04  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2003 by MEN mikro elektronik GmbH, Nuernberg, Germany 
 ****************************************************************************/

static const char RCSid[]="$Id: mscan_drv.c,v 1.19 2013/03/27 09:54:36 gvarlet Exp $";

#include "mscan_int.h"

/*-----------------------------------------+
|  PROTOTYPES                              |
+-----------------------------------------*/

static int32 MscanWriteMsg( MSCAN_HANDLE *h, MSCAN_READWRITEMSG_PB *pb );
static int32 MscanClearBusOff( MSCAN_HANDLE *h );
static int32 MscanEnable( MSCAN_HANDLE *h, int32 enable );
static int32 MscanLoopback( MSCAN_HANDLE *h, int32 enable );
static int32 MscanSetFilter( MSCAN_HANDLE *h, MSCAN_SETFILTER_PB *pb );
static int32 MscanConfigMsg( MSCAN_HANDLE *h, MSCAN_CONFIGMSG_PB *pb );
static int32 MscanSetBusTiming( MSCAN_HANDLE *h, 
								const MSCAN_SETBUSTIMING_PB *pb );
static int32 MscanSetBitRate( MSCAN_HANDLE *h, MSCAN_SETBITRATE_PB *pb );
static int32 MscanSetSig( MSCAN_HANDLE *h, MSCAN_SIGNAL_PB *pb, MSCAN_DIR dir);
static int32 MscanClrSig( MSCAN_HANDLE *h, MSCAN_SIGNAL_PB *pb, MSCAN_DIR dir);
static int32 MscanQueueClear( MSCAN_HANDLE *h, MSCAN_QUEUECLEAR_PB *pb );
static int32 MscanReadMsg( MSCAN_HANDLE *h, MSCAN_READWRITEMSG_PB *pb );
static int32 MscanReadError( MSCAN_HANDLE *h, MSCAN_READERROR_PB *pb );
static int32 MscanQueueStatus( MSCAN_HANDLE *h, MSCAN_QUEUESTATUS_PB *pb );
static int32 MscanErrorCounters( MSCAN_HANDLE *h, MSCAN_ERRORCOUNTERS_PB *pb );
static int32 MscanDumpInternals( MSCAN_HANDLE *h, char *buffer, int maxLen);
static void IrqRx( MSCAN_HANDLE *h );
static int ScheduleNextTx( MSCAN_HANDLE *h, int txb );
static void IrqOverrun( MSCAN_HANDLE *h );
static void IrqStatus( MSCAN_HANDLE *h );
static MSCAN_NODE_STATUS NodeStatus( MSCAN_HANDLE *h );
static int SwFilter( const MSCAN_FRAME *frm, const MSCAN_FILTER *fspec );
static char* Ident( void );
static int32 Cleanup(MSCAN_HANDLE *h, int32 retCode);
static int32 QueueClear( MSCAN_HANDLE *h, u_int32 nr, u_int32 txabort );
static int32 InitModeEnter( MSCAN_HANDLE *h );
static int32 InitModeLeave( MSCAN_HANDLE *h );
static void SetFilter( MSCAN_HANDLE *h, int fltNum, const MSCAN_FILTER *fspec);
static int32 CalcBustime( MSCAN_HANDLE *h,
						  u_int32 bitrate,
						  u_int32 *calcBrpP,
						  u_int32 *calcTsegP );
static void DumpFilter( MSCAN_HANDLE *h, char *msg, const MSCAN_FILTER *f );
static void DumpFrame( MSCAN_HANDLE *h, char *msg, const MSCAN_FRAME *frm );
static void PutError( MSCAN_HANDLE *h, int nr, MSCAN_ERRENTRY_CODE code );
static int WaitRxFifoEntry( MSCAN_HANDLE *h,int nr,int32 timeout);
static void RecomputeObjLimits( MSCAN_HANDLE *h );

/**********************************************************************/
/** LL-Interface Init: Initialize MSCAN LL driver
 *	
 * Allocate handle, initialize hardware.
 * 
 * Descriptor keys:
 *
 * - DEBUG_LEVEL_DESC [OSS_DBG_DEFAULT]: debug level for
 * descriptor library 
 *
 * - \c DEBUG_LEVEL [OSS_DBG_DEFAULT]: debug level for LL driver code
 *
 * - \c CANCLOCK: must be specified! Specifies the MSCAN input clock in HZ. 
 *
 */
static int32 MSCAN_Init( 
	DESC_SPEC       *descP,
	OSS_HANDLE      *osHdl,
	MACCESS         *ma,
	OSS_SEM_HANDLE  *devSemHdl,
	OSS_IRQ_HANDLE  *irqHdl,
	LL_HANDLE       **llHdlP )
{
	MSCAN_HANDLE *h;	
    u_int32 gotsize;    
    u_int32 value;
    int32 	error;
    int32	i;

	/* Filters to let all IDs pass through */
	const MSCAN_FILTER stdFilter = { 
		0,
		0xffffffff,
		0,
		0 
	};
	const MSCAN_FILTER extFilter = { 
		0,
		0xffffffff,
		MSCAN_EXTENDED,
		0 
	};

	*llHdlP = NULL;
    /*------------------------------+
    |  prepare the handle           |
    +------------------------------*/
	/* alloc handle */
    if ((h = (MSCAN_HANDLE*)
		 OSS_MemGet(osHdl, sizeof(MSCAN_HANDLE), &gotsize)) == NULL)
       return(ERR_OSS_MEM_ALLOC);

	/* clear */
    OSS_MemFill(osHdl, gotsize, (char*)h, 0x00);

	/* init */
    h->memAlloc   	= gotsize;
    h->osHdl      	= osHdl;
    h->irqHdl     	= irqHdl;
	h->devSemHdl  	= devSemHdl;
	h->ma			= ma[0];

    /*------------------------------+
    |  init id function table       |
    +------------------------------*/
	/* drivers ident function */
	h->idFuncTbl.idCall[0].identCall = Ident;
	/* libraries ident functions */
	h->idFuncTbl.idCall[1].identCall = DESC_Ident;
	h->idFuncTbl.idCall[2].identCall = OSS_Ident;
	/* terminator */
	h->idFuncTbl.idCall[3].identCall = NULL;

    /*------------------------------+
    |  prepare debugging            |
    +------------------------------*/
	DBG_MYLEVEL = OSS_DBG_DEFAULT;	/* set OS specific debug level */
	DBGINIT((NULL,&DBH));

    DBGWRT_1((DBH, "LL - MSCAN_Init\n"));

    /*------------------------------+
    |  scan descriptor              |
    +------------------------------*/
	/* prepare access */
    if ((error = DESC_Init(descP, osHdl, &h->descHdl)))
		return( Cleanup( h, error ) );

    /* DEBUG_LEVEL_DESC */
    if ((error = DESC_GetUInt32( h->descHdl, OSS_DBG_DEFAULT, 
								 &value, "DEBUG_LEVEL_DESC")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup( h, error ) );

	DESC_DbgLevelSet(h->descHdl, value);	/* set level */

    /* DEBUG_LEVEL */
    if ((error = DESC_GetUInt32( h->descHdl, OSS_DBG_DEFAULT, 
								 &h->dbgLevel, "DEBUG_LEVEL")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup( h, error ) );

	/*---------------------------------+
    | MSCAN configuration parameters   |
    +---------------------------------*/

    /* CANCLOCK (mandatory) */
    if ((error = DESC_GetUInt32(h->descHdl, 0, 
								&h->canClock, "CANCLOCK"))){
		DBGWRT_ERR((DBH," *** MSCAN_Init: CANCLOCK desc key not found!\n"));
		return( Cleanup( h, error ) );
	}
	
#if defined(  MSCAN_IS_ODIN )
    /* assign CAN pins on MGT5100 for PP01 & MGT5200B for EM1N */
    {
        MACCESS virtGPCRAddr;
        u_int32 tmp;
        OSS_MapPhysToVirtAddr ( 
            osHdl,
            (void*)GPIOPCR,
		    4,
		    OSS_ADDRSPACE_MEM,
		    OSS_BUSTYPE_PCI,
		    0,
		    (void*)&virtGPCRAddr);
        tmp = MREAD_D32(virtGPCRAddr, 0 );
        tmp &= ~PSC2_MASK;
        tmp |=  PSC2_CAN;     /* PSC2: CAN1 CAN2 */
        MWRITE_D32(virtGPCRAddr, 0, tmp);
    }
#endif

	/* MIN_BRP (optional) */
    DESC_GetUInt32(h->descHdl, MSCAN_MIN_BRP, &h->minBrp, "MIN_BRP");

	/*-----------------------+
	|  init message objects  |
	+-----------------------*/
	for( i=0; i<MSCAN_NUM_OBJS; i++ ) {

		h->msgObj[i].nr 	= i;
		h->msgObj[i].q.dir 	= MSCAN_DIR_DIS;
	}
	RecomputeObjLimits( h );

    /*------------------------------+
    |  init hardware                |
    +------------------------------*/
	/* enable CAN module */
	MSWRITE( h->ma, MSCAN_CTL1, MSCAN_CTL1_CANE);

	if( !(MSREAD( h->ma, MSCAN_CTL1 ) & MSCAN_CTL1_CANE) ){
		DBGWRT_ERR((DBH,"*** MSCAN: CANE did not store!\n"));
		return( Cleanup( h, ERR_LL_DEV_NOTRDY ) );
	}

	h->nodeStatus = MSCAN_NS_ERROR_ACTIVE; /* assume good state */
	
	/* put MSCAN in INIT mode (all bus activity is disabled) */
	if( (error = InitModeEnter( h ) ))
		return( Cleanup( h, error ) );
		
    DBGWRT_1((DBH, "LL - MSCAN_Init finished ok\n"));
	*llHdlP = (LL_HANDLE *)h;

	/*
	 * Filter config:
	 * Set filter to ignore, let all messages pass through
	 */
	SetFilter( h, 0, &stdFilter );
	SetFilter( h, 1, &extFilter );

	if (h->descHdl)
		DESC_Exit(&h->descHdl);

	return(ERR_SUCCESS);
}

/**********************************************************************/
/** LL-Interface Exit: De-initialize MSCAN LL driver
 */
static int32 MSCAN_Exit( LL_HANDLE **llHdlP )
{
    MSCAN_HANDLE *h = (MSCAN_HANDLE *)*llHdlP;
	int32 error = 0;
	OSS_IRQ_STATE oldState;

    DBGWRT_1((DBH, "LL - MSCAN_Exit\n"));

    /*------------------------------+
    |  de-init hardware             |
    +------------------------------*/
	oldState = OSS_IrqMaskR( h->osHdl, h->irqHdl );
	InitModeEnter( h );
	OSS_IrqRestore( h->osHdl, h->irqHdl, oldState );

    /*------------------------------+
    |  cleanup memory               |
    +------------------------------*/
	error = Cleanup( h, error);
	*llHdlP = NULL;

	return(error);
}

/**********************************************************************/
/** LL-Interface Read: Not supported by this driver */
static int32 MSCAN_Read( LL_HANDLE *llHdl, int32 ch, int32 *valueP )
{
	return(ERR_LL_ILL_FUNC);
}

/**********************************************************************/
/** LL-Interface Write: Not supported by this driver */
static int32 MSCAN_Write( LL_HANDLE *llHdl, int32 ch, int32 value )
{
	return(ERR_LL_ILL_FUNC);
}

/**********************************************************************/
/** LL-Interface SetStat: Set driver status
 *	
 */
static int32 MSCAN_SetStat( 
	LL_HANDLE *llHdl,
	int32  code,
	int32  ch,
	INT32_OR_64 value32_or_64 )
{
    int32 value = (int32)value32_or_64;	        /* 32bit value     */
    INT32_OR_64 valueP = value32_or_64;			/* stores 32/64bit pointer */
	MSCAN_HANDLE *h = (MSCAN_HANDLE *)llHdl;
	int32 error = ERR_SUCCESS;
	M_SG_BLOCK *blk	= (M_SG_BLOCK*)valueP;

    DBGWRT_1((DBH, "LL - MSCAN_SetStat: ch=%d code=0x%04x value=0x%x\n",
			  ch,code,value));

    switch(code) {

	case MSCAN_WRITEMSG:
		CHK_BLK_SIZE( blk, MSCAN_READWRITEMSG_PB );
		error = MscanWriteMsg( h, (MSCAN_READWRITEMSG_PB*)blk->data );
		break;

	case MSCAN_CLEARBUSOFF:
		error = MscanClearBusOff( h );
		break;

	case MSCAN_ENABLE:
		error = MscanEnable( h, value );
		break;

	case MSCAN_LOOPBACK:
		error = MscanLoopback( h, value );
		break;

	case MSCAN_SETFILTER:
		CHK_BLK_SIZE( blk, MSCAN_SETFILTER_PB );
		error = MscanSetFilter( h, (MSCAN_SETFILTER_PB*)blk->data );
		break;

	case MSCAN_CONFIGMSG:
		CHK_BLK_SIZE( blk, MSCAN_CONFIGMSG_PB );
		error = MscanConfigMsg( h, (MSCAN_CONFIGMSG_PB*)blk->data );
		break;

	case MSCAN_SETBUSTIMING:
		CHK_BLK_SIZE( blk, MSCAN_SETBUSTIMING_PB );
		error = MscanSetBusTiming( h, (MSCAN_SETBUSTIMING_PB*)blk->data );
		break;

	case MSCAN_SETBITRATE:
		CHK_BLK_SIZE( blk, MSCAN_SETBITRATE_PB );
		error = MscanSetBitRate( h, (MSCAN_SETBITRATE_PB*)blk->data );
		break;

	case MSCAN_SETRCVSIG:
		CHK_BLK_SIZE( blk, MSCAN_SIGNAL_PB );
		error = MscanSetSig( h, (MSCAN_SIGNAL_PB*)blk->data, MSCAN_DIR_RCV );
		break;

	case MSCAN_SETXMTSIG:
		CHK_BLK_SIZE( blk, MSCAN_SIGNAL_PB );
		error = MscanSetSig( h, (MSCAN_SIGNAL_PB*)blk->data, MSCAN_DIR_XMT );
		break;

	case MSCAN_CLRRCVSIG:
		CHK_BLK_SIZE( blk, MSCAN_SIGNAL_PB );
		error = MscanClrSig( h, (MSCAN_SIGNAL_PB*)blk->data, MSCAN_DIR_RCV );
		break;

	case MSCAN_CLRXMTSIG:
		CHK_BLK_SIZE( blk, MSCAN_SIGNAL_PB );
		error = MscanClrSig( h, (MSCAN_SIGNAL_PB*)blk->data, MSCAN_DIR_XMT );
		break;

	case MSCAN_QUEUECLEAR:
		CHK_BLK_SIZE( blk, MSCAN_QUEUECLEAR_PB );
		error = MscanQueueClear( h, (MSCAN_QUEUECLEAR_PB*)blk->data );
		break;


	/*--- standard MDIS setstats ---*/
	case M_MK_IRQ_ENABLE:
		DBGWRT_2((DBH, " IRQ_ENABLE %d\n", value));

		/*--- enable/disable all interrupt sources ---*/
		if( value )
			h->irqEnabled = TRUE;
		else {
			/* note: processor interrupts already disabled here */
			h->irqEnabled = FALSE;
			MSWRITE( h->ma, MSCAN_RIER, 0x00 );	
			MSWRITE( h->ma, MSCAN_TIER, 0x00 );	
		}
		break;

	case M_LL_DEBUG_LEVEL:	h->dbgLevel = value; break;
	case M_MK_IRQ_COUNT:	h->irqCount = value; break;
	case MSCAN_MAXIRQTIME:	h->maxIrqTime = value; break;

	default:		
		error = ERR_LL_UNK_CODE;
		break;
    }

    DBGWRT_2((DBH, " SetStat exit error=0x%x\n",error));
	return(error);
}

/**********************************************************************/
/** LL-Interface GetStat: Get driver status
 *	
 */
static int32 MSCAN_GetStat( 
	LL_HANDLE *llHdl,
	int32  code,
	int32  ch,
	INT32_OR_64 *value32_or_64P )
{
    int32 *valueP = (int32*)value32_or_64P;	    /* pointer to 32bit value  */
    INT32_OR_64 *value64P = value32_or_64P;     /* stores 32/64bit pointer  */
	MSCAN_HANDLE *h = (MSCAN_HANDLE *)llHdl;
	M_SG_BLOCK 	*blk = (M_SG_BLOCK*)value32_or_64P;
	int32 error = ERR_SUCCESS;

    DBGWRT_1((DBH, "LL - MSCAN_GetStat: ch=%d code=0x%04x\n", ch,code));

    switch(code){

	case MSCAN_READMSG:
		CHK_BLK_SIZE( blk, MSCAN_READWRITEMSG_PB );
		error = MscanReadMsg( h, (MSCAN_READWRITEMSG_PB*)blk->data );
		break;

	case MSCAN_READERROR:
		CHK_BLK_SIZE( blk, MSCAN_READERROR_PB );
		error = MscanReadError( h, (MSCAN_READERROR_PB*)blk->data );
		break;

	case MSCAN_QUEUESTATUS:
		CHK_BLK_SIZE( blk, MSCAN_QUEUESTATUS_PB );
		error = MscanQueueStatus( h, (MSCAN_QUEUESTATUS_PB*)blk->data );
		break;

	case MSCAN_ERRORCOUNTERS:
		CHK_BLK_SIZE( blk, MSCAN_ERRORCOUNTERS_PB );
		error = MscanErrorCounters( h, (MSCAN_ERRORCOUNTERS_PB*)blk->data );
		break;

	case MSCAN_DUMPINTERNALS:
		error = MscanDumpInternals( h, (char *)blk->data, (int)blk->size );
		break;

	case MSCAN_GETCANCLK:	*valueP = h->canClock; break;
	case MSCAN_NODESTATUS:	*valueP = (int32)NodeStatus( h ); break;
	case MSCAN_MAXIRQTIME:	*valueP = h->maxIrqTime; break;
		
	/*--- standard MDIS getstats ---*/
	case M_LL_DEBUG_LEVEL:	*valueP = h->dbgLevel; break;
	case M_LL_CH_NUMBER:	*valueP = MSCAN_NUM_OBJS; break;
	case M_LL_CH_TYP:		*valueP = M_CH_BINARY; break;
	case M_LL_IRQ_COUNT:	*valueP = h->irqCount; break;
	case M_MK_BLK_REV_ID:	*value64P = (INT32_OR_64)&h->idFuncTbl; break;
	default:
		error = ERR_LL_UNK_CODE;
		break;
    }

    DBGWRT_2((DBH, " GetStat exit error=0x%x\n",error));
	return(error);
}

/**********************************************************************/
/** LL-Interface BlockRead: Read frames from message object
 *	
 */
static int32 MSCAN_BlockRead( 
	LL_HANDLE *llHdl,
	int32     ch,
	void      *buf,
	int32     size,
	int32     *nbrRdBytesP )
{
	MSCAN_HANDLE *h = (MSCAN_HANDLE *)llHdl;
	MSG_OBJ *obj = &h->msgObj[ch];
	int32 availEntries, n;
	MQUEUE_ENT *ent;
	MSCAN_FRAME *frm;
	OSS_IRQ_STATE oldState;

    DBGWRT_1((DBH, "LL - MSCAN_BlockRead: objNr=%d, size=%d\n",ch,size));
	*nbrRdBytesP = 0;

	/* parameter checks */
	if( ch >= MSCAN_NUM_OBJS || ch==0)
		return MSCAN_ERR_BADMSGNUM;

	if( obj->q.dir != MSCAN_DIR_RCV )
		return MSCAN_ERR_BADDIR;

	/* compute number of frames user wants to read */
	size /= sizeof(MSCAN_FRAME);

	/* check for FIFO space */
	availEntries = obj->q.filled;
	if( availEntries > size )
		availEntries = size;

	DBGWRT_2((DBH, " dequeue %d frames\n", availEntries ));

	/*----------------------+
	|  Put frame into FIFO  |
	+----------------------*/
	n = availEntries;
	ent = obj->q.nxtOut;
	frm = (MSCAN_FRAME *)buf;

	while( n-- ){
		*frm++ = ent->d.frm;
		ent = ent->next;
	}
	obj->q.nxtOut = ent;
	
	oldState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

	obj->q.filled -= availEntries;
	obj->q.errSent = FALSE;

	OSS_IrqRestore( h->osHdl, h->irqHdl, oldState );

	/* return nr of read bytes */
	*nbrRdBytesP = availEntries * sizeof(MSCAN_FRAME);

	return( 0 );
}

/**********************************************************************/
/** LL-Interface BlockWrite: Write frames to message object
 *	
 */
static int32 MSCAN_BlockWrite( 
	LL_HANDLE *llHdl,
	int32     ch,
	void      *buf,
	int32     size,
	int32     *nbrWrBytesP )
{
	MSCAN_HANDLE *h = (MSCAN_HANDLE *)llHdl;
	MSG_OBJ *obj = &h->msgObj[ch];
	int32 availEntries, n;
	MQUEUE_ENT *ent;
	MSCAN_FRAME *frm;
	OSS_IRQ_STATE oldState;

    DBGWRT_1((DBH, "LL - MSCAN_BlockWrite: objNr=%d, size=%d\n",ch,size));
	*nbrWrBytesP = 0;

	/* parameter checks */
	if( ch >= MSCAN_NUM_OBJS || ch==0)
		return MSCAN_ERR_BADMSGNUM;

	if( obj->q.dir != MSCAN_DIR_XMT )
		return MSCAN_ERR_BADDIR;

	if( !h->canEnabled )
		return MSCAN_ERR_NOTINIT;

	/* compute number of frames user wants to write */
	size /= sizeof(MSCAN_FRAME);

	/* check for FIFO space */
	availEntries = obj->q.totEntries - obj->q.filled;
	if( availEntries > size )
		availEntries = size;

	DBGWRT_2((DBH, " enqueue %d frames\n", availEntries ));

	/*----------------------+
	|  Put frame into FIFO  |
	+----------------------*/
	n = availEntries;
	ent = obj->q.nxtIn;
	frm = (MSCAN_FRAME *)buf;

	while( n-- ){
		ent->d.frm = *frm++;
		ent = ent->next;
	}
	obj->q.nxtIn = ent;
	
	oldState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

	obj->q.filled += availEntries;

	/* enable all tx interrupts */
	MSWRITE( h->ma, MSCAN_TIER, MSCAN_TXB_MASK );
	
	OSS_IrqRestore( h->osHdl, h->irqHdl, oldState );

	/* return nr of written bytes */
	*nbrWrBytesP = availEntries * sizeof(MSCAN_FRAME);

	return( 0 );
}

/**********************************************************************/
/** LL-Interface Irq: MSCAN interrupt handler
 *	
 * Checks for occurred interrupts:
 * - receive interrupts
 * - transmit interrupts
 * - receive overrun interrupts
 * - status change interrupts
 *
 * Transmit buffer handling:
 * Checks all free tx buffers if they have currently finished a transmission.
 *
 * In addition, check if a new tx frame can be scheduled for any of the
 * three buffers. 
 *
 * It also keeps track of the priorities between the frames to guarantee
 * a chronological transmission of frames from the same FIFO.
 *
 * \sa ScheduleNextTx
 *
 * \return \c LL_IRQ_DEVICE or LL_IRQ_DEV_NOT if no interrupt was pending
 */
static int32 MSCAN_Irq( LL_HANDLE *llHdl )
{
	MSCAN_HANDLE *h = (MSCAN_HANDLE *)llHdl;
	MACCESS ma = h->ma;
	u_int8 rflg, tflg;
	int haveInt=0;
	MSG_OBJ *obj;
	int txb, objNr, nothingToSched=FALSE;
	u_int8 txbMask;
	OSS_IRQ_STATE oldState;

	tflg = MSREAD( ma, MSCAN_TFLG );

	IDBGWRT_2((DBH,">>> MSCAN_Irq rflg=0x%x tflg=0x%x\n", 
			   MSREAD( ma, MSCAN_RFLG ), tflg));

	/* Mask the IRQ to be SMP safe */
	oldState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

	/*-----------------------------------------+
	|  Handle Rx and scheduling of Tx buffers  |
	+-----------------------------------------*/

	/*--- check for received buffers ---*/
	if( (rflg = MSREAD( ma, MSCAN_RFLG )) & MSCAN_RFLG_RXF ){
		IrqRx( h );
		haveInt++;
	}

	/*--- check for completed transmissions ---*/
	for( txb=0, txbMask=0x1; tflg && txb<MSCAN_NTXBUFS; txb++, txbMask<<=1 ){
		if( tflg & txbMask ){

			/*--- buffer is available ---*/
			if( h->txPrio[txb] != MSCAN_UNASSIGNED ){

				/*--- this buffer just completed a transmission ---*/
				objNr = h->txPrio[txb] >> 4;	/* get related obj number */
				obj = &h->msgObj[objNr];
				
				IDBGWRT_2((DBH,"   txed buf %d prio=0x%02x obj %d\n", 
						   txb, h->txPrio[txb], objNr ));

				/* record last sent prio for that object */
				{ 
					u_int8 locPri = h->txPrio[txb] & 0xf;

					if( (locPri > obj->txSentPrio) ||
						((locPri==0) && (obj->txSentPrio==0xf)))
						obj->txSentPrio = locPri;
				}
				obj->txbUsed &= ~txbMask;
				h->txPrio[txb] = MSCAN_UNASSIGNED;
				haveInt++;
			}
		}
	}

	/*--- schedule new transmissions ---*/
	for( txb=0, txbMask=0x1; txb<MSCAN_NTXBUFS; txb++, txbMask<<=1 ){

		if( tflg & txbMask ){

			/*--- schedule next transmission ---*/
			if( (nothingToSched == TRUE) || (ScheduleNextTx( h, txb ) == 0)) {
				/* no new buffer scheduled, disable irq for that tx buf */
				IDBGWRT_2((DBH,"   nothing sched'd for txb %d\n", txb ));
				MSCLRMASK( ma, MSCAN_TIER, txbMask );
				nothingToSched = TRUE;
			}
			/*
			 * check again for received buffers
			 * required for loopback mode - otherwise transmitter may
			 * overrun receiver
			 */
			if( (rflg = MSREAD( ma, MSCAN_RFLG )) & MSCAN_RFLG_RXF ){
				IrqRx( h );
				haveInt++;
			}
		}
	}

	/*--- check for Rx overrun ---*/
	if( rflg & MSCAN_RFLG_OVRIF ){
		IrqOverrun( h );
		haveInt++;
	}

	/*--- check for status change interrupts ---*/
	if( rflg & MSCAN_RFLG_CSCIF ){
		IrqStatus( h );
		/* clear status change interrupt*/
		MSWRITE( ma, MSCAN_RFLG, MSCAN_RFLG_CSCIF );
		haveInt++;
	}

	IDBGWRT_2((DBH,"<<< MSCAN_Irq\n"));
	
	/* Restore IRQ before returning from the ISR */
	OSS_IrqRestore( h->osHdl, h->irqHdl, oldState );

	if( haveInt ){
		h->irqCount += haveInt;
		return LL_IRQ_DEVICE;
	}
	else {
		return LL_IRQ_DEV_NOT;
	}
}

/**********************************************************************/
/** LL-Interface Info: Static driver information
 *	
 */
static int32 MSCAN_Info( int32 infoType, ... )
{
    int32   error = ERR_SUCCESS;
    va_list argptr;

    va_start(argptr, infoType );

    switch(infoType) {
		/*-------------------------------+
        |  hardware characteristics      |
        |  (all addr/data modes OR'ed)   |
        +-------------------------------*/
        case LL_INFO_HW_CHARACTER:
		{
			u_int32 *addrModeP = va_arg(argptr, u_int32*);
			u_int32 *dataModeP = va_arg(argptr, u_int32*);

			*addrModeP = MDIS_MA08;
			*dataModeP = MDIS_MD08 | MDIS_MD16;
			break;
	    }
		/*-------------------------------+
        |  nr of required address spaces |
        |  (total spaces used)           |
        +-------------------------------*/
        case LL_INFO_ADDRSPACE_COUNT:
		{
			u_int32 *nbrOfAddrSpaceP = va_arg(argptr, u_int32*);

			*nbrOfAddrSpaceP = 1;
			break;
	    }
		/*-------------------------------+
        |  address space type            |
        |  (widest used data mode)       |
        +-------------------------------*/
        case LL_INFO_ADDRSPACE:
		{
			u_int32 addrSpaceIndex = va_arg(argptr, u_int32);
			u_int32 *addrModeP = va_arg(argptr, u_int32*);
			u_int32 *dataModeP = va_arg(argptr, u_int32*);
			u_int32 *addrSizeP = va_arg(argptr, u_int32*);

			if (addrSpaceIndex >= 1)
				error = ERR_LL_ILL_PARAM;
			else {
				*addrModeP = MDIS_MA08;
				*dataModeP = MDIS_MD16;
				*addrSizeP = ADDRSPACE_SIZE;
			}

			break;
	    }
		/*-------------------------------+
        |   interrupt required           |
        +-------------------------------*/
        case LL_INFO_IRQ:
		{
			u_int32 *useIrqP = va_arg(argptr, u_int32*);

			*useIrqP = TRUE;
			break;
	    }
		/*-------------------------------+
        |   process lock mode            |
        +-------------------------------*/
        case LL_INFO_LOCKMODE:
		{
			u_int32 *lockModeP = va_arg(argptr, u_int32*);

			*lockModeP = LL_LOCK_NONE; /* driver locks critical sections */
			break;
	    }
		/*-------------------------------+
        |   (unknown)                    |
        +-------------------------------*/
        default:
			error = ERR_LL_ILL_PARAM;
    }

    va_end(argptr);
    return(error);
}

/**********************************************************************/
/** LL-Interface GetEntry: Initialize drivers jump table
 *	
 */
#ifdef _ONE_NAMESPACE_PER_DRIVER_
void LL_GetEntry( LL_ENTRY* drvP )
#else
void _MSCAN_GetEntry( LL_ENTRY* drvP )
#endif
{
    drvP->init        = MSCAN_Init;
    drvP->exit        = MSCAN_Exit;
    drvP->read        = MSCAN_Read;
    drvP->write       = MSCAN_Write;
    drvP->blockRead   = MSCAN_BlockRead;
    drvP->blockWrite  = MSCAN_BlockWrite;
    drvP->setStat     = MSCAN_SetStat;
    drvP->getStat     = MSCAN_GetStat;
    drvP->irq         = MSCAN_Irq;
    drvP->info        = MSCAN_Info;
}
/**********************************************************************/
/* END OF STANDARD MDIS INTERFACE ROUTINES							  */
/**********************************************************************/

/**********************************************************************/
/** Return driver ident string */
static char* Ident( void )
{
    return( "MSCAN - MSCAN low level driver: $Id: mscan_drv.c,v 1.19 2013/03/27 09:54:36 gvarlet Exp $" );
}

/**********************************************************************/
/** Cleanup routine for driver init/exit function 
 * 
 * Close all handles, free memory and return error code. 
 *
 * LL handle cannot be used anymore after this function			   
 *
 * \param h			Low level handle
 * \param retCode	this value is returned by this function
 * \return value passed to \a retCode
 */
static int32 Cleanup( MSCAN_HANDLE *h, int32 retCode ) 
{
	u_int32 nr;
	/*------------------------------+
	|  Free message queues/sems     |
	+------------------------------*/
	for( nr=0; nr<MSCAN_NUM_OBJS; nr++ )
	{
		if( h->msgObj[nr].sig )
			OSS_SigRemove( h->osHdl, &h->msgObj[nr].sig );
		if( h->msgObj[nr].q.sem )
			OSS_SemRemove( h->osHdl, &h->msgObj[nr].q.sem );
	
		if( h->msgObj[nr].q.first )
		{
			OSS_MemFree( h->osHdl, (int8 *)h->msgObj[nr].q.first, 
						 h->msgObj[nr].q.memAlloc );
			h->msgObj[nr].q.first = NULL;
		}
	}

    /*------------------------------+
    |  close handles                |
    +------------------------------*/
	/* clean up desc */
	if (h->descHdl)
		DESC_Exit(&h->descHdl);

	/* cleanup debug */
	DBGEXIT((&DBH));

    /*------------------------------+
    |  free memory                  |
    +------------------------------*/
    /* free my handle */
    OSS_MemFree(h->osHdl, (int8*)h, h->memAlloc);

    /*------------------------------+
    |  return error code            |
    +------------------------------*/
	return(retCode);
}

/**********************************************************************/
/** Handler for API function mscan_config_msg
 */ 
static int32 MscanConfigMsg( MSCAN_HANDLE *h, MSCAN_CONFIGMSG_PB *pb )
{
	MSG_OBJ *obj = &h->msgObj[pb->objNr];
	MQUEUE_ENT *ent;
	u_int32 i;
	int32 error=0;
	OSS_IRQ_STATE oldState;

	DBGWRT_1((DBH,"MscanConfigMsg: nr=%d dir=%d entries=%d\n", 
			  pb->objNr, pb->dir, pb->qEntries ));

	/* sanity checks */
	if( pb->dir > MSCAN_DIR_XMT )
		return MSCAN_ERR_BADDIR;

	if( pb->objNr >= MSCAN_NUM_OBJS )
		return MSCAN_ERR_BADMSGNUM;

	if( (pb->qEntries == 0) && (pb->dir != MSCAN_DIR_DIS) )
		return MSCAN_ERR_BADPARAMETER;

	if( (pb->filter.mflags & MSCAN_USE_ACCFIELD) && 
		(pb->filter.cflags & MSCAN_EXTENDED))
		return MSCAN_ERR_BADPARAMETER;

	/*-------------+
	|  Init queue  |
	+-------------*/	
	obj->q.ready	  = FALSE;

	/*--- realloc memory for queue ---*/
	if( obj->q.first ){
		OSS_MemFree( h->osHdl, (void *)obj->q.first, obj->q.memAlloc );
		obj->q.first = NULL;
	}

	if( pb->dir != MSCAN_DIR_DIS ){

		if( obj->q.sem == NULL ){
			/*--- create wakeup sem for object ---*/
			if( (error = OSS_SemCreate( h->osHdl, OSS_SEM_BIN, 0, 
										&obj->q.sem ))){

				DBGWRT_ERR((DBH,"*** MscanConfigMsg: error 0x%x "
							"creating sem\n",error));
				goto ABORT;
			}
		}
		/*--- allocate new queue entries ---*/

		if( (obj->q.first = (MQUEUE_ENT*)OSS_MemGet( h->osHdl, 
												pb->qEntries * sizeof(MQUEUE_ENT),
												&obj->q.memAlloc )) == NULL){
			DBGWRT_ERR((DBH,"*** MscanConfigMsg: can't alloc queue mem\n"));
			error = ERR_OSS_MEM_ALLOC;
			goto ABORT;
		}

		/*--- init queue entries ---*/
		for( i=0, ent=obj->q.first; i<pb->qEntries; ent++, i++ )
			ent->next = ent+1;

		ent[-1].next = obj->q.first;

		/*--- init queue ---*/
		obj->q.totEntries = pb->qEntries;
		obj->q.filled	  = 0;
		obj->q.dir		  = pb->dir;
		obj->q.filter	  = pb->filter;
		obj->txbUsed	  = 0;
		obj->txNxtPrio	  = 0;
		obj->txSentPrio	  = 0xf;

		DBGWRT_2((DBH,"filter: mask=%x code=%x cf=%x mf=%x\n",
				  obj->q.filter.mask, obj->q.filter.code, 
				  obj->q.filter.cflags, obj->q.filter.mflags ));
		DBGDMP_2((DBH,"accField", (void *)obj->q.filter.accField, 0x100, 1));

		oldState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

		QueueClear( h, pb->objNr, 0 );

		OSS_IrqRestore( h->osHdl, h->irqHdl, oldState );

	}
	else {
		/*--- disable object ---*/
		obj->q.totEntries = 0;
		obj->q.filled	  = 0;
		obj->q.ready	  = FALSE;
		obj->q.dir		  = MSCAN_DIR_DIS;

	}
 ABORT:
	/* recompute first/last Rx/Tx object */
	RecomputeObjLimits( h );
	return error;
}

/**********************************************************************/
/** Handler for API function mscan_enable
 */ 
static int32 MscanEnable( MSCAN_HANDLE *h, int32 enable )
{
	int32 error;

	DBGWRT_1((DBH,"MscanEnable: %sable\n", enable ? "en" : "dis" )); 
	
	if( enable ){
		if( !h->busTimingSet || !h->irqEnabled )
		    error = MSCAN_ERR_NOTINIT;
		else
			error = InitModeLeave( h ); 
	}
	else {
		error = InitModeEnter( h ); 
	}
		
	return error;
}

/**********************************************************************/
/** Handler for API function mscan_loopback
 */ 
static int32 MscanLoopback( MSCAN_HANDLE *h, int32 enable )
{
	DBGWRT_1((DBH,"MscanLoopback: %sable\n", enable ? "en" : "dis" )); 

	if( h->canEnabled )
		return MSCAN_ERR_ONLINE;

	if( enable ){
		MSSETMASK( h->ma, MSCAN_CTL1, MSCAN_CTL1_LOOPB );
	}
	else{
		MSCLRMASK( h->ma, MSCAN_CTL1, MSCAN_CTL1_LOOPB );
	}
		
	return 0;
}

/**********************************************************************/
/** Handler for API function mscan_set_bustiming
 */ 
static int32 MscanSetBusTiming( MSCAN_HANDLE *h, 
								const MSCAN_SETBUSTIMING_PB *pb )
{
	u_int8 btr0, btr1;

	DBGWRT_1((DBH,"MscanSetBusTiming: brp=%d sjw=%d tseg1=%d tseg2=%d "
			  "spl=%d\n", 
			  pb->brp, pb->sjw, pb->tseg1, pb->tseg2, pb->spl ));
	
	if( h->canEnabled )
		return MSCAN_ERR_ONLINE;

	if( (pb->brp < 1) || (pb->brp > 64 ) || 
		(pb->sjw < 1) || (pb->sjw > 4 ) ||
		(pb->tseg1 < 1 ) || (pb->tseg1 > 16 ) ||
		(pb->tseg2 < 1 ) || (pb->tseg2 > 8 )){

		DBGWRT_ERR((DBH, "*** MscanSetBusTiming bad parameter\n"));
		return MSCAN_ERR_BADTMDETAILS;
	}

	
	btr0 = ((pb->sjw - 1) << 6) | (pb->brp - 1);
	btr1 = ((pb->tseg2 - 1) << 4) | (pb->tseg1 - 1) | (pb->spl ? 0x80:0x00);

	DBGWRT_2((DBH, " btr0=0x%02x btr1=0x%02x\n", btr0, btr1 ));

	MSWRITE( h->ma, MSCAN_BTR0, btr0 );
	MSWRITE( h->ma, MSCAN_BTR1, btr1 );

	h->busTimingSet = TRUE;

	return 0;
}

/**********************************************************************/
/** Handler for API function mscan_set_bitrate
 *
 * If CANCLOCK is 32Mhz, use BTR values from table, otherwise
 * compute the BTR values dynamically
 */ 
static int32 MscanSetBitRate( MSCAN_HANDLE *h, MSCAN_SETBITRATE_PB *pb )
{
	u_int32 brp, tseg, bitrate, bestRate;
	MSCAN_SETBUSTIMING_PB bt;
	const struct _BTR{
		int bitrate; 
		MSCAN_SETBUSTIMING_PB bt;
	} brTable32MHz[] = {			/* BTR table for 32MHz */
		{ 1000000, { 4, 1, 5, 2, 0 } },
		{  800000, { 4, 1, 7, 2, 0 } },
		{  500000, { 4, 1, 13, 2, 0 } },
		{  250000, { 8, 1, 13, 2, 0 } },
		{  125000, { 16, 1, 13, 2, 0 } },
		{  100000, { 20, 1, 13, 2, 0 } },
		{   50000, { 40, 1, 13, 2, 0 } },
		{   20000, { 0xff, 1, 13, 2, 0 } },/* not possible,BRP value exceeded*/
		{   10000, { 0xff, 1, 13, 2, 0 } },/* not possible,BRP value exceeded*/
	};
#ifndef MSCAN_SC15
	const struct _BTR brTable16MHz[] = {
		{ 1000000, { 2, 1, 5, 2, 0 } },
		{  800000, { 2, 1, 7, 2, 0 } },
		{  500000, { 2, 1, 13, 2, 0 } },
		{  250000, { 4, 1, 13, 2, 0 } },
		{  125000, { 8, 1, 13, 2, 0 } },
		{  100000, { 10, 1, 13, 2, 0 } },
		{   50000, { 20, 1, 13, 2, 0 } },
		{   20000, { 50, 1, 13, 2, 0 } },
		{   10000, { 64, 1, 13, 3, 0 } },

	};
#else
	const struct _BTST{
		int bitrate;
		u_int32 brp;		
		u_int32 tseg;		
	} btstTable16MHz[] = {			/* BTR table for 16MHz */
		{ 1000000, 2,8 },
		{ 800000, 2,10 },
		{ 500000, 2,16 },
		{ 250000, 4,16 },
		{ 125000, 8,16 },
		{ 83333, 12,16 },
		{ 50000, 20,16 },
		{ 20000, 50,16 },/* not possible,BRP value exceeded*/		
	};	
#endif

	DBGWRT_1((DBH,"MscanSetBitRate: bitrate=%d spl=%d ",
			  pb->bitrate, pb->spl ));

	if( pb->bitrate > sizeof(brTable32MHz)/sizeof(struct _BTR) )
		return MSCAN_ERR_BADSPEED;

	#ifdef MSCAN_SC15
		bitrate = btstTable16MHz[pb->bitrate].bitrate;
	#else
		bitrate = brTable32MHz[pb->bitrate].bitrate;
	#endif

	DBGWRT_2(( DBH, " bitrate=%d bps, canclock=%d\n", bitrate, h->canClock ));

	if( h->canClock == 32000000 ){
		/* take values from table */
		if( brTable32MHz[pb->bitrate].bt.brp == 0xff )
			return MSCAN_ERR_BADSPEED;

		bt = brTable32MHz[pb->bitrate].bt;
		bt.spl = (u_int8)pb->spl;
		return MscanSetBusTiming( h, &bt );
	}

	#ifndef MSCAN_SC15
		/* not 32MHz: check if 16MHz */
		if( h->canClock == 16000000 ){
			/* take values from table */
			if( brTable16MHz[pb->bitrate].bt.brp == 0xff )
				return MSCAN_ERR_BADSPEED;

			bt = brTable16MHz[pb->bitrate].bt;
			bt.spl = (u_int8)pb->spl;
			return MscanSetBusTiming( h, &bt );
		}

		/* neither. compute timing */
		bestRate = CalcBustime( h, bitrate, &brp, &tseg );
	#else
		brp = btstTable16MHz[pb->bitrate].brp;
		tseg = btstTable16MHz[pb->bitrate].tseg;
		bestRate = btstTable16MHz[pb->bitrate].bitrate;
	#endif

	DBGWRT_2(( DBH, " computed brp=%d tseg=%d best=%d\n", brp, tseg, 
			   bestRate));
	
	/* check if req. bitrate can be reached at 0.1% */
	if( (bestRate*1000 < 999 * bitrate) ||
		(bestRate*1000 > 1001 * bitrate)){

#ifndef MSCAN_BRP_ALLOW_1
		u_int32 orig_diff, diff, best_diff;
		u_int32 rate;
		u_int32 bestRateBpr1 = bestRate;
		best_diff = orig_diff = (bestRate > bitrate) ? bestRate - bitrate : bitrate - bestRate;
		for (tseg=6; tseg<=19; tseg++) {
			rate = h->canClock / tseg;
			diff = (bitrate > rate ? bitrate-rate : rate-bitrate);
			if (diff < best_diff){
				best_diff = diff;
				bestRateBpr1 = rate;
			}
		}
		if(best_diff < orig_diff){
			DBGWRT_ERR(( DBH, "The driver is configured not to accept a value of 1 for the "
						"baud rate prescaler (not supported by some cores). It was detected "
						"the CAN core could archive a " ));
			if( (bestRateBpr1*1000 < 999 * bitrate) ||
					(bestRateBpr1*1000 > 1001 * bitrate)){
				DBGWRT_ERR ((DBH, "better, yet still not " ));
			}
			DBGWRT_ERR ((DBH, "sufficient accuracy ",
						"using a prescaler value of 1. Compile with MSCAN_BRP_ALLOW_1 defined "
						"to enable a prescaler value of 1 if you are sure the core supports "
						"this.\n"));
		}

#endif //MSCAN_BRP_ALLOW_1

		DBGWRT_ERR((DBH, "*** MscanSetBusTiming bad parameter\n"));
		return MSCAN_ERR_BADSPEED;
	}

	/* setup sampling point according to CIA DS 102 V2.0 */
	if( bitrate == 1000000 )
		bt.tseg1 = (u_int8)tseg * 75 / 100;
	else if( bitrate == 800000 )
		bt.tseg1 = (u_int8)tseg * 80 / 100;
	else 
		bt.tseg1 = (u_int8)tseg * 875 / 1000;

	bt.brp = (u_int8)brp;
	bt.sjw = 1;
	bt.spl = (u_int8)pb->spl;

	bt.tseg2 = (u_int8)tseg - bt.tseg1;
	bt.tseg1--;					/* subtract SYNC */

	return MscanSetBusTiming( h, &bt );
}

/**********************************************************************/
/** Handler for API function mscan_set_filter
 */ 
static int32 MscanSetFilter( MSCAN_HANDLE *h, MSCAN_SETFILTER_PB *pb )
{
	int canWasEnabled = h->canEnabled;
	int32 error = 0;

	DBGWRT_1((DBH,"MscanSetFilter\n"));
	DumpFilter( h, "Filter1:", &pb->filter1 );
	DumpFilter( h, "Filter2:", &pb->filter2 );

	if( canWasEnabled ){
		if(( error = InitModeEnter( h )))
			goto XIT;
	}

	SetFilter( h, 0, &pb->filter1 );
	SetFilter( h, 1, &pb->filter2 );

	if( canWasEnabled ){
		if(( error = InitModeLeave( h )))
			goto XIT;
	}
 XIT:
	return error;
}

/**********************************************************************/
/** Handler for API function mscan_write_msg
 */ 
static int32 MscanWriteMsg( MSCAN_HANDLE *h, MSCAN_READWRITEMSG_PB *pb )
{
	MSG_OBJ *obj = &h->msgObj[pb->objNr];
	int32 error = 0;
	OSS_IRQ_STATE oldState;

	DBGWRT_1((DBH,"MscanWriteMsg objNr=%d tout=%dms\n", 
			  pb->objNr, pb->timeout));
	DumpFrame( h, " enqueue", &pb->msg );

	/* parameter checks */
	if( pb->objNr >= MSCAN_NUM_OBJS || pb->objNr==0)
		return MSCAN_ERR_BADMSGNUM;

	if( obj->q.dir != MSCAN_DIR_XMT )
		return MSCAN_ERR_BADDIR;

	if( !h->canEnabled )
		return MSCAN_ERR_NOTINIT;

	/*-----------------------+
	|  Check for FIFO space  |
	+-----------------------*/
	if( obj->q.filled == obj->q.totEntries ){

		DBGWRT_2((DBH, " FIFO full\n"));

		/*--- FIFO full ---*/
		if( pb->timeout == -1 ){
			return MSCAN_ERR_QFULL;
		}

		while( error == 0 ){
			obj->q.waiting = TRUE;	/* flag we're waiting for sem */

			DEVSEM_UNLOCK( h );

			/* wait for FIFO space */
			error = OSS_SemWait( h->osHdl, obj->q.sem, 
								 pb->timeout==0 ? 
								 OSS_SEM_WAITFOREVER : pb->timeout );

			DEVSEM_LOCK( h );

			if( obj->q.waiting == FALSE ){
				/*
				 * fifo space available
				 */
				break;
			}
			else if( error ){
				obj->q.waiting = FALSE;
				DBGWRT_ERR((DBH,"*** MscanWriteMsg: error 0x%x waiting for "
							"FIFO\n", error ));
				return error;
			}
			/* no error, but no fifo space, continue waiting */
		}
	}
	
	/*----------------------+
	|  Put frame into FIFO  |
	+----------------------*/
	obj->q.nxtIn->d.frm = pb->msg;
	obj->q.nxtIn = obj->q.nxtIn->next;
	
	oldState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

	obj->q.filled++;
	/* enable all tx interrupts */
	MSWRITE( h->ma, MSCAN_TIER, MSCAN_TXB_MASK );
	
	OSS_IrqRestore( h->osHdl, h->irqHdl, oldState );

	return 0;
}

/**********************************************************************/
/** Handler for API function mscan_read_msg
 */ 
static int32 MscanReadMsg( MSCAN_HANDLE *h, MSCAN_READWRITEMSG_PB *pb )
{
	MSG_OBJ *obj = &h->msgObj[pb->objNr];
	int32 error = 0;
	OSS_IRQ_STATE oldState;

	DBGWRT_1((DBH,"MscanReadMsg objNr=%d tout=%dms\n", 
			  pb->objNr, pb->timeout));

	/* parameter checks */
	if( pb->objNr==0)
		return MSCAN_ERR_BADMSGNUM;

	/* wait until there is at least one entry in FIFO */
	if( (error = WaitRxFifoEntry( h, pb->objNr, pb->timeout )) )
		return error;

	/*----------------------+
	|  Get frame from FIFO  |
	+----------------------*/
	pb->msg = obj->q.nxtOut->d.frm;
	obj->q.nxtOut = obj->q.nxtOut->next;
	
	oldState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

	obj->q.filled--;
	obj->q.errSent = FALSE;

	OSS_IrqRestore( h->osHdl, h->irqHdl, oldState );

	DumpFrame( h, " dequeue", &pb->msg );

	return 0;
}

/**********************************************************************/
/** Handler for API function mscan_read_error
 */ 
static int32 MscanReadError( MSCAN_HANDLE *h, MSCAN_READERROR_PB *pb )
{
	MSG_OBJ *obj = &h->msgObj[MSCAN_ERROR_OBJ];
	int32 error = 0;
	OSS_IRQ_STATE oldState;

	DBGWRT_1((DBH,"MscanReadError\n" ));

	/* wait (forever) until there is at least one entry in FIFO */
	if( (error = WaitRxFifoEntry( h, MSCAN_ERROR_OBJ, 0)) )
		return error;

	/*---------------------------+
	|  Get error info from FIFO  |
	+---------------------------*/
	*pb = obj->q.nxtOut->d.err;
	obj->q.nxtOut = obj->q.nxtOut->next;
	
	oldState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

	obj->q.filled--;
	
	OSS_IrqRestore( h->osHdl, h->irqHdl, oldState );

	DBGWRT_2((DBH," dequeued error info code=%d nr=%d\n",
			  pb->errCode, pb->objNr));

	return 0;
	
}

/**********************************************************************/
/** Handler for API function mscan_clear_busoff
 * \remark not required for MSCAN	
 */ 
static int32 MscanClearBusOff( MSCAN_HANDLE *h )
{
	return 0;
}


/**********************************************************************/
/** Handler for API function mscan_set_rcv_sig and mscan_set_xmt_sig
 */ 
static int32 MscanSetSig( MSCAN_HANDLE *h, MSCAN_SIGNAL_PB *pb, MSCAN_DIR dir)
{
	MSG_OBJ *obj = &h->msgObj[pb->objNr];
	int32 error;

	DBGWRT_1((DBH,"MscanSetSig objNr=%d sigCode=%d\n", 
			  pb->objNr, pb->signal));

	/* parameter checks */
	if( pb->objNr >= MSCAN_NUM_OBJS )
		return MSCAN_ERR_BADMSGNUM;
	
	if( obj->q.dir != dir )
		return MSCAN_ERR_BADDIR;

	if( obj->sig != NULL )
		return MSCAN_ERR_SIGBUSY;

	if( (error = OSS_SigCreate( h->osHdl, pb->signal, &obj->sig )))
		obj->sig = NULL;

	return error;
}

/**********************************************************************/
/** Handler for API function mscan_clr_rcv_sig and mscan_clr_xmt_sig
 */ 
static int32 MscanClrSig( MSCAN_HANDLE *h, MSCAN_SIGNAL_PB *pb, MSCAN_DIR dir)
{
	MSG_OBJ *obj = &h->msgObj[pb->objNr];
	int32 error;

	DBGWRT_1((DBH,"MscanSetSig objNr=%d sigCode=%d\n", 
			  pb->objNr, pb->signal));

	/* parameter checks */
	if( pb->objNr >= MSCAN_NUM_OBJS )
		return MSCAN_ERR_BADMSGNUM;
	
	if( obj->sig == NULL )
		return MSCAN_ERR_SIGBUSY;

	error = OSS_SigRemove( h->osHdl, &obj->sig );

	return error;
}

/**********************************************************************/
/** Handler for API function mscan_queue_clear
 */ 
static int32 MscanQueueClear( MSCAN_HANDLE *h, MSCAN_QUEUECLEAR_PB *pb )
{
	MSG_OBJ *obj = &h->msgObj[pb->objNr];

	DBGWRT_1((DBH,"MscanQueueClear %d\n", pb->objNr ));

	if( pb->objNr >= MSCAN_NUM_OBJS )
		return MSCAN_ERR_BADMSGNUM;

	/* flag object as non-ready */
	obj->q.ready	= FALSE;

	/* reset FIFO counters */
	return QueueClear( h, pb->objNr, pb->txabort );
}

/**********************************************************************/
/** Handler for API function mscan_queue_status
 */ 
static int32 MscanQueueStatus( MSCAN_HANDLE *h, MSCAN_QUEUESTATUS_PB *pb )
{
	MSG_OBJ *obj = &h->msgObj[pb->objNr];

	if( pb->objNr >= MSCAN_NUM_OBJS )
		return MSCAN_ERR_BADMSGNUM;

	if( obj->q.dir == MSCAN_DIR_XMT )
		pb->entries = obj->q.totEntries - obj->q.filled;
	else
		pb->entries = obj->q.filled;

	pb->direction = obj->q.dir;

	return 0;
}

/**********************************************************************/
/** Handler for API function mscan_error_counters
 */ 
static int32 MscanErrorCounters( MSCAN_HANDLE *h, MSCAN_ERRORCOUNTERS_PB *pb )
{
	/* 
	 * standard MSCAN implementation do not allow to read error counters
	 * while online, Z015 does...
	 */
#ifndef MSCAN_IS_Z15
	if( h->canEnabled )
		return MSCAN_ERR_ONLINE;
#endif

	pb->txErrCnt = MSREAD( h->ma, MSCAN_TXER );
	pb->rxErrCnt = MSREAD( h->ma, MSCAN_RXER );

	return 0;
}

/**********************************************************************/
/** IrqRx
 *
 * called from MSCAN_Irq.
 * IrqRx assumes that there is a valid frame into the fifo.
 * Reads out a single frame from the mscan's rx fifo and tries to find
 * a matching Rx object
 */ 
static void IrqRx( MSCAN_HANDLE *h )
{
	MACCESS ma = h->ma;
	MSCAN_FRAME frm;
	u_int32 id, idr1, idr3;
	MSG_OBJ *obj;
	int nr;

	IDBGWRT_2((DBH," CAN Rx irq\n"));
	
	/*----------------------------+
	|  Get frame from CAN's FIFO  |
	+----------------------------*/
	frm.flags = 0;

	if( (idr1 = MSREAD( ma, MSCAN_RXIDR1 )) & 0x8 ){
		/* extended id */
		id = (u_int32)MSREAD( ma, MSCAN_RXIDR0 ) << 21;
		id |= (idr1 & 0x7) << 15;
		id |= (idr1 & 0xe0) << 13;
		id |= (u_int32)MSREAD( ma, MSCAN_RXIDR2 ) << 7;
		idr3 = MSREAD( ma, MSCAN_RXIDR3 );
		id |= idr3 >> 1;

		if (idr3 & 0x1)
			frm.flags |= MSCAN_RTR;

		frm.flags |= MSCAN_EXTENDED;	
	}
	else {
		/* standard ID */
		id = (u_int32)MSREAD( ma, MSCAN_RXIDR0 ) << 3;
		id |= idr1 >> 5;
		if (idr1 & 0x10)
			frm.flags |= MSCAN_RTR;	
	}
	frm.id = id;

	switch( frm.dataLen = (MSREAD( ma, MSCAN_RXDLR ) & 0xf) ){
	case 8:	frm.data[7] = MSREAD( ma, MSCAN_RXDSR7 );
	case 7:	frm.data[6] = MSREAD( ma, MSCAN_RXDSR6 );
	case 6:	frm.data[5] = MSREAD( ma, MSCAN_RXDSR5 );
	case 5:	frm.data[4] = MSREAD( ma, MSCAN_RXDSR4 );
	case 4:	frm.data[3] = MSREAD( ma, MSCAN_RXDSR3 );
	case 3:	frm.data[2] = MSREAD( ma, MSCAN_RXDSR2 );
	case 2:	frm.data[1] = MSREAD( ma, MSCAN_RXDSR1 );
	case 1:	frm.data[0] = MSREAD( ma, MSCAN_RXDSR0 );
	case 0:
	default:
		break;
	}

	/* release Rx buffer */
	MSWRITE( ma, MSCAN_RFLG, MSCAN_RFLG_RXF );

	DumpFrame( h, "   rxfrm", &frm );

	/*----------------------------------------+
	|  Find the corresponding message object  |
	+----------------------------------------*/
	nr = h->firstRxObj;

	for( obj=&h->msgObj[nr]; nr<=h->lastRxObj; nr++, obj++ ){		
		if( !obj->q.ready || (obj->q.dir != MSCAN_DIR_RCV) ){			
			continue;
		}

		if( SwFilter( &frm, &obj->q.filter ) == TRUE ){			
			IDBGWRT_2((DBH, " put frm to msg obj %d\n", nr));

			/* put the received frame into the object's FIFO */
			if( obj->q.filled == obj->q.totEntries ){
				IDBGWRT_ERR((DBH, "*** MSCAN obj %d overrun\n", nr));

				if( ! obj->q.errSent ){
					PutError( h, nr, MSCAN_QOVERRUN );
					obj->q.errSent = TRUE;
				}
			}
			else {				
				obj->q.nxtIn->d.frm = frm;
				obj->q.nxtIn = obj->q.nxtIn->next;
				obj->q.filled++;

				/* wakeup read waiter */
				if( obj->q.waiting ){					
					IDBGWRT_2((DBH, " wake read waiter\n"));
					obj->q.waiting = FALSE;
					OSS_SemSignal( h->osHdl, obj->q.sem );
				}

				/* send signal */
				if( obj->sig ){					
					OSS_SigSend( h->osHdl, obj->sig );
				}
			}
			break;
		}
	}

#ifdef DBG
	if( nr == MSCAN_NUM_OBJS )
		IDBGWRT_2((DBH, " frm discarded\n"));
#endif
}

/**********************************************************************/
/** Schedule next transmit frame to txbuffer \a txb
 * 
 * \return 0=no frame has been scheduled for transmission, 1=scheduled
 */ 
static int ScheduleNextTx( MSCAN_HANDLE *h, int txb )
{
	MSG_OBJ *obj;
	int nr, sched=FALSE;
	u_int8 txbMask = 1<<txb;

	nr = h->firstTxObj;

	for( obj=&h->msgObj[nr]; nr<=h->lastTxObj; nr++, obj++ ){

		if( obj->q.ready && (obj->q.dir == MSCAN_DIR_XMT) &&
			(obj->q.filled != 0 )){
			sched = TRUE;
			break;
		}
	}

	if( !sched )
		return 0;				/* nothing to schedule */
		
	/*
	 * if there are other scheduled frames pending for that 
	 * object and the new (object relative) priority would be 0,
	 * the new frame would be perhaps sent before the previous
	 * frames. 
	 * To avoid this, we keep track of the outstanding frames.
	 */
	if( (obj->txNxtPrio == 0) && (obj->txSentPrio < 0xf) ){
		IDBGWRT_2((DBH,"  SchedNextTx: delay obj %d\n", nr ));
		return 0;
	}

	/* record new priority being scheduled on tx buffer */
	h->txPrio[txb] = obj->txNxtPrio + (nr<<4);

	/* advance local priority for next frame */
	obj->txNxtPrio = (obj->txNxtPrio + 1) & 0xf;

	obj->txbUsed |= txbMask;
		
	/*----------------------------------------+ 
	|  Put frame from FIFO into tx buffer     |
	+----------------------------------------*/
	{
		MACCESS ma = h->ma;
		MSCAN_FRAME *frm = &obj->q.nxtOut->d.frm;
		const u_int8 *dataP = frm->data;
		u_int32 id = frm->id;
	
		IDBGWRT_2((DBH,"   tx buf %d prio=0x%02x obj %d\n",
				   txb, h->txPrio[txb], nr ));
		DumpFrame( h, "   tx", frm );

		MSWRITE( ma, MSCAN_BSEL, txbMask ); /* select tx buffer */

		MSWRITE( ma, MSCAN_TXDSR0, *dataP++ );
		MSWRITE( ma, MSCAN_TXDSR1, *dataP++ );
		MSWRITE( ma, MSCAN_TXDSR2, *dataP++ );
		MSWRITE( ma, MSCAN_TXDSR3, *dataP++ );
		MSWRITE( ma, MSCAN_TXDSR4, *dataP++ );
		MSWRITE( ma, MSCAN_TXDSR5, *dataP++ );
		MSWRITE( ma, MSCAN_TXDSR6, *dataP++ );
		MSWRITE( ma, MSCAN_TXDSR7, *dataP++ );

		MSWRITE( ma, MSCAN_TXDLR, frm->dataLen );

		if( frm->flags & MSCAN_EXTENDED ){
			/* extended message */
			MSWRITE( ma, MSCAN_TXIDR0, id>>21 );
			MSWRITE( ma, MSCAN_TXIDR1, ((id>>13)&0xe0) | 0x18 | 
					 ((id>>15)&0x07));

			MSWRITE( ma, MSCAN_TXIDR2, id>>7 );
			MSWRITE( ma, MSCAN_TXIDR3, (id<<1) | 
					 ((frm->flags & MSCAN_RTR) ? 0x1:0x0));
		}
		else {
			MSWRITE( ma, MSCAN_TXIDR0, id>>3 );
			MSWRITE( ma, MSCAN_TXIDR1, (id<<5) | 
					 ((frm->flags & MSCAN_RTR) ? 0x10:0x0));
		}

		MSWRITE( ma, MSCAN_TXBPR, h->txPrio[txb] );
		
		/* enable irq, start TX */
		MSSETMASK( ma, MSCAN_TIER, txbMask );
		MSWRITE( ma, MSCAN_TFLG, txbMask );
	}


	/* fifo handling */
	obj->q.nxtOut = obj->q.nxtOut->next;
	obj->q.filled--;

	/* wakeup write waiter */
	if( obj->q.waiting ){
		IDBGWRT_2((DBH, " wake write waiter\n"));
		obj->q.waiting = FALSE;
		OSS_SemSignal( h->osHdl, obj->q.sem );
	}

	/* send signal */
	if( obj->sig )
		OSS_SigSend( h->osHdl, obj->sig );

	return 1;		
}


/**********************************************************************/
/** Handle Rx overrun errors
 *
 */ 
static void IrqOverrun( MSCAN_HANDLE *h )
{
	MACCESS ma = h->ma;

	IDBGWRT_ERR((DBH,"*** CAN Rx overrun\n"));
	PutError( h, 0, MSCAN_DATA_OVERRUN );

	MSWRITE( ma, MSCAN_RFLG, MSCAN_RFLG_OVRIF );
}

/**********************************************************************/
/** Check for node status changes.
 * 
 * This is called from MSCAN_Irq() and InitModeLeave().
 *
 * \remark CSCIF is cleared in MSCAN_Irq() routine!
 */ 
static void IrqStatus( MSCAN_HANDLE *h )
{
	MSCAN_NODE_STATUS oldState = h->nodeStatus;
	MSCAN_NODE_STATUS newState;

	IDBGWRT_2((DBH," CAN Status changed\n"));
	/* detemine new status */
	newState = NodeStatus( h );

	/* check for state change */
	if( oldState != MSCAN_NS_BUS_OFF && 
		newState == MSCAN_NS_BUS_OFF )
		PutError( h, 0, MSCAN_BUSOFF_SET );

	if( oldState == MSCAN_NS_BUS_OFF && 
		newState != MSCAN_NS_BUS_OFF )
		PutError( h, 0, MSCAN_BUSOFF_CLR );

	if( oldState != MSCAN_NS_ERROR_PASSIVE && 
		newState == MSCAN_NS_ERROR_PASSIVE )
		PutError( h, 0, MSCAN_WARN_SET );

	if( oldState == MSCAN_NS_ERROR_PASSIVE && 
		newState != MSCAN_NS_ERROR_PASSIVE )
		PutError( h, 0, MSCAN_WARN_CLR );


	IDBGWRT_2((DBH," oldState=%d newState=%d\n", oldState, newState));

	h->nodeStatus = newState;
}

/**********************************************************************/
/** Determine node status (error active, warning, passive or bus off)
 *
 */
static MSCAN_NODE_STATUS NodeStatus( MSCAN_HANDLE *h )
{
	u_int8 rflg = MSREAD( h->ma, MSCAN_RFLG );
	MSCAN_NODE_STATUS nodeStatus;

	/* 
	 * following Bosch CAN Spec V2.0:
	 * "a node is error passive when tx error count or rx error count >=128"
	 * "a node is bus off when tx error count >= 256 
	 */
	if( (rflg & 0xc) == 0xc ){
		/* bus off */
		nodeStatus = MSCAN_NS_BUS_OFF;
	}
	else if( ((rflg & 0xc) == 0x8) || ((rflg & 0x30) == 0x20) ){
		/* error passive */
		nodeStatus = MSCAN_NS_ERROR_PASSIVE;
	}
	else {
		/* error active */
		nodeStatus = MSCAN_NS_ERROR_ACTIVE;
	}
	IDBGWRT_2((DBH," NodeStatus: rlfg=0x%02x node state=%d\n", 
			   rflg, nodeStatus));
	return nodeStatus;		
}

/**********************************************************************/
/** Software filtering. Check if \a frm matches filter \fspec
 *
 * \param frm		frame to compare
 * \param fspec		filter specification (see struct doc)
 * \returns 0=no hit, 1 hit
 */
static int SwFilter( const MSCAN_FRAME *frm, const MSCAN_FILTER *fspec )
{
	if( (frm->flags & MSCAN_EXTENDED) != (fspec->cflags & MSCAN_EXTENDED))
		return 0;

	if( fspec->mflags & MSCAN_RTR ){
		if( (fspec->cflags & MSCAN_RTR) != (frm->flags & MSCAN_RTR))
			return 0;
	}

	if( (fspec->code & ~fspec->mask) != (frm->id & ~fspec->mask) )
		return 0;

	/* individual filter */
	if( (fspec->mflags & MSCAN_USE_ACCFIELD) &&
		! MSCAN_ACCFIELD_GET( fspec->accField, frm->id ))
		return 0;

	return 1;
}

/**********************************************************************/
/** Wait for entry from rx FIFO (either CAN rx or error object)
 *
 * When this function returns without error, caller must get the 
 * frame from FIFO and update FIFO counter and pointer
 *
 * \param nr		message object number
 * \param timeout	-1=don't wait, 0=wait forever, >0=tout in ms
 * \returns error code
 */
static int WaitRxFifoEntry( 
	MSCAN_HANDLE *h,
	int nr,
	int32 timeout)
{
	MSG_OBJ *obj = &h->msgObj[nr];
	int32 error = 0;

	if( nr >= MSCAN_NUM_OBJS )
		return MSCAN_ERR_BADMSGNUM;

	if( obj->q.dir != MSCAN_DIR_RCV )
		return MSCAN_ERR_BADDIR;

    /*--------------------------+
	|  Check for frames in FIFO |
	+--------------------------*/
	if( obj->q.filled == 0 ){

		DBGWRT_2((DBH, " FIFO empty\n"));

		/*--- FIFO empty ---*/
		if( timeout == -1 ){
			return MSCAN_ERR_NOMESSAGE;
		}

		while( error == 0 ){
			obj->q.waiting = TRUE;	/* flag we're waiting for sem */

			DEVSEM_UNLOCK( h );

			/* wait for FIFO entries */
			error = OSS_SemWait( h->osHdl, obj->q.sem, 
								 timeout==0 ? 
								 OSS_SEM_WAITFOREVER : timeout );

			DEVSEM_LOCK( h );


			if( obj->q.waiting == FALSE ){
				/*
				 * got an rx message
				 */
				break;
			}
			else if( error ){
				obj->q.waiting = FALSE;
				DBGWRT_ERR((DBH,"*** MscanReadMsg: error 0x%x waiting for "
							"FIFO\n", error ));
				break;
			}

			/* no error, but no rx msg arrived, continue waiting */
		}
			
	}
	
	return error;
}

/**********************************************************************/
/** Put an entry into error queue
 *
 * \param	h		LL handle
 * \param	nr		related msg obj number (0 for global errors)
 * \param	code	error code to put into error fifo
 */ 
static void PutError( MSCAN_HANDLE *h, int nr, MSCAN_ERRENTRY_CODE code )
{
	MSG_OBJ *obj = &h->msgObj[0];

	IDBGWRT_2((DBH," PutError nr=%d code=%d\n", nr, code ));

	if( !obj->q.ready )
		return;					/* no error object created */

	/* put the error into the error FIFO */
	if( obj->q.filled == obj->q.totEntries ){
		IDBGWRT_ERR((DBH, "*** MSCAN error obj overrun\n", nr));
	}
	else {
		obj->q.nxtIn->d.err.errCode = code;
		obj->q.nxtIn->d.err.objNr	= nr;
		obj->q.filled++;
		obj->q.nxtIn = obj->q.nxtIn->next;

		/* wakeup read waiter */
		if( obj->q.waiting ){
			IDBGWRT_2((DBH, " wake read waiter\n"));
			obj->q.waiting = FALSE;
			OSS_SemSignal( h->osHdl, obj->q.sem );
		}

		/* send signal */
		if( obj->sig )
			OSS_SigSend( h->osHdl, obj->sig );
	}
	
}

/**********************************************************************/
/** Reset queue of message object
 *
 * \param	h		LL handle
 * \param	nr		msg obj number
 * \param	txabort	if non-zero, stop all pending transmissions of
 *					that msg obj
 * \return	error code
 */ 
static int32 QueueClear( MSCAN_HANDLE *h, u_int32 nr, u_int32 txabort ) 
{
	MSG_OBJ *obj = &h->msgObj[nr];

	DBGWRT_2((DBH,"  QueueClear: nr=%d txabort=%d\n", nr, txabort ));

	if( nr >= MSCAN_NUM_OBJS )
		return MSCAN_ERR_BADMSGNUM;

#if 0
	/*???*/
	if( txabort && obj->q.dir == CPL_DIR_XMT )
		_MSCAN_transmit_abort( llHdl, nr );
#endif
	/*--- init queue pointers ---*/
	obj->q.nxtIn 	= obj->q.first;
	obj->q.nxtOut 	= obj->q.first;
	obj->q.filled	= 0;
	obj->q.errSent  = 0;
	obj->q.ready	= TRUE;

	return 0;
}

/**********************************************************************/
/** Set up one of the two 32 bit filters
 *
 * Controller must be in INIT mode
 * 
 * \param fltNum 	filter number to setup (0 or 1)
 * \param fspec		filter specification 
 *
 */
static void SetFilter( MSCAN_HANDLE *h, int fltNum, const MSCAN_FILTER *fspec )
{
	MACCESS ma = h->ma;
	u_int32 id;
	static const int mreg[] = {
		MSCAN_IDMR0, MSCAN_IDMR1, MSCAN_IDMR2, MSCAN_IDMR3, 
		MSCAN_IDMR4, MSCAN_IDMR5, MSCAN_IDMR6, MSCAN_IDMR7 
	};
	static const int areg[] = {
		MSCAN_IDAR0, MSCAN_IDAR1, MSCAN_IDAR2, MSCAN_IDAR3, 
		MSCAN_IDAR4, MSCAN_IDAR5, MSCAN_IDAR6, MSCAN_IDAR7 
	};
	const int *ar, *mr;

	if( fltNum == 0 ){
		ar = &areg[0];
		mr = &mreg[0];
	}
	else {
		ar = &areg[4];
		mr = &mreg[4];
	}

	/*
	 * Filter config:
	 * IDAM=0	- 2*32 bit acceptance filters
	 */
	MSWRITE( ma, MSCAN_IDAC, 0x00 );

	if( fspec->cflags & MSCAN_EXTENDED ){

		/* extended filter */
		id = fspec->mask;

		MSWRITE( ma, mr[0], id>>21 );
		MSWRITE( ma, mr[1], ((id>>13)&0xe0) | 0x00 | ((id>>15)&0x07));
		MSWRITE( ma, mr[2], id>>7 );
		MSWRITE( ma, mr[3], (id<<1) | 
				   ((fspec->mflags & MSCAN_RTR) ? 0x0:0x1));

		id = fspec->code;

		MSWRITE( ma, ar[0], id>>21 );
		MSWRITE( ma, ar[1], ((id>>13)&0xe0) | 0x18 | ((id>>15)&0x07));
		MSWRITE( ma, ar[2], id>>7 );
		MSWRITE( ma, ar[3], (id<<1) | 
				   ((fspec->cflags & MSCAN_RTR) ? 0x1:0x0));
	}
	else {

		/* standard filter */
		id = fspec->mask;

		MSWRITE( ma, mr[0], id>>3 );
		MSWRITE( ma, mr[1], (id<<5) | 0x07 |
				   ((fspec->mflags & MSCAN_RTR) ? 0x0:0x10));
		MSWRITE( ma, mr[2], 0xff ); /* really necessary? */
		MSWRITE( ma, mr[3], 0xff );

		id = fspec->code;

		MSWRITE( ma, ar[0], id>>3 );
		MSWRITE( ma, ar[1], (id<<5) |
				   ((fspec->cflags & MSCAN_RTR) ? 0x10:0x0));

	}
}



/**********************************************************************/
/** Put MSCAN into INIT mode
 *
 * \return	error code if INITAK problem
 */
static int32 InitModeEnter( MSCAN_HANDLE *h )
{
	MACCESS ma = h->ma;
	int timeout = 20000;

	/* INITAK handshake */
	MSSETMASK( ma, MSCAN_CTL0, MSCAN_CTL0_INITRQ );

	while( (MSREAD( ma, MSCAN_CTL1 ) & MSCAN_CTL1_INITAK) == 0  ){
		if( timeout-- == 0 ){
			DBGWRT_ERR((DBH,"*** MSCAN-InitModeEnter: INITAK timeout!\n"));
			return ERR_LL_DEV_NOTRDY;
		}
	}
	h->canEnabled = FALSE;
	return 0;
}

/**********************************************************************/
/** Switch MSCAN from INIT mode to NORMAL mode
 *
 * Enables all receive/status interrupts
 *
 * \return	error code if INITAK problem
 */
static int32 InitModeLeave( MSCAN_HANDLE *h )
{
	MACCESS ma = h->ma;
	int timeout = 20000, i;

	for( i=0; i<MSCAN_NTXBUFS; i++ )
		h->txPrio[i] = MSCAN_UNASSIGNED;

	/* INITAK handshake */
	MSCLRMASK( ma, MSCAN_CTL0, MSCAN_CTL0_INITRQ );

	while( (MSREAD( ma, MSCAN_CTL1 ) & MSCAN_CTL1_INITAK) != 0  ){
		if( timeout-- == 0 ){
			DBGWRT_ERR((DBH,"*** MSCAN-InitModeLeave: INITAK timeout!\n"));
			return ERR_LL_DEV_NOTRDY;
		}
	}
	h->canEnabled = TRUE;

	/* update nodestatus */
	IrqStatus( h );

	/* 
	 * enable interrupts: Rx, Rx overrun, status change for all cases
	 */
	MSWRITE( ma, MSCAN_RIER, MSCAN_RFLG_RXF | MSCAN_RFLG_OVRIF | 
			 MSCAN_RFLG_CSCIF | 0x3c );

	return 0;
}

/**********************************************************************/
/** Calculate BRP and TSEG values (time quantas) for given bitrate
 *
 *               Tries to find out the best combination of BRP and TSEG:
 *			   
 *                   bitrate = canclock / (BRP * (TSEG+1))
 *			   
 *			     (BRP=1..64)
 *			     (TSEG=SYNC+TSEG1+TSEG2=6..19)
 *
 * \param h				LL handle
 * \param bitrate		bus bitrate [bit/s]
 *
 * \return resulting bitrate [bit/s]
 *		   *calcBrpP	best matching BRP
 * 		   *calcTsegP	best matching TSEG1+TSEG2 sum [time quantas]
 */
static int32 CalcBustime( MSCAN_HANDLE * h,
						  u_int32 bitrate,
						  u_int32 *calcBrpP,
						  u_int32 *calcTsegP )
{
	u_int32 brp,best_brp=0,tseg,best_tseg=0;
	u_int32 rate,best_rate=0,diff,best_diff;

	best_diff = h->canClock / 16;			/* max. diff */

	for (brp = h->minBrp; brp<=64; brp++) {
		for (tseg=6; tseg<=19; tseg++) {
			rate = h->canClock / (brp * (tseg));
			diff = (bitrate > rate ? bitrate-rate : rate-bitrate);

			if (diff <= best_diff) {		/* better match ? */
				best_brp = brp;			/* store params */
				best_tseg = tseg;
				best_rate = rate;
				best_diff = diff;
			}

			if (diff == 0) 				/* exact match ? */
				goto ALLDONE;
		}
	}

ALLDONE:	
	*calcBrpP  = best_brp;			/* return best matching params */
	*calcTsegP = best_tseg;
	return(best_rate);
}

/**********************************************************************/
/** recompute first/last Rx/Tx object 
 */
static void RecomputeObjLimits( MSCAN_HANDLE *h )
{
	int32 firstRx=MSCAN_NUM_OBJS, lastRx=0, firstTx=MSCAN_NUM_OBJS, lastTx=0;
	int32 nr;
	OSS_IRQ_STATE oldState;

	for( nr=1; nr<MSCAN_NUM_OBJS; nr++ ){

		if( h->msgObj[nr].q.dir == MSCAN_DIR_RCV ){

			if( nr < firstRx )
				firstRx = nr;
			if( nr > lastRx )
				lastRx = nr;
		}
		else if( h->msgObj[nr].q.dir == MSCAN_DIR_XMT ){

			if( nr < firstTx )
				firstTx = nr;
			if( nr > lastTx )
				lastTx = nr;
		}
	}
	DBGWRT_2((DBH, " RecomputeObjLimits: Rx %d..%d, Tx %d..%d\n",
			  firstRx, lastRx, firstTx, lastTx ));

	oldState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

	h->firstRxObj = firstRx;
	h->lastRxObj  = lastRx;
	h->firstTxObj = firstTx;
	h->lastTxObj  = lastTx;

	OSS_IrqRestore( h->osHdl, h->irqHdl, oldState );
}


/**********************************************************************/
/** dump internals to user
 */
static int32 MscanDumpInternals( MSCAN_HANDLE *h, char *buffer, int maxLen)
{
#define ADDSTR( _x_ ) \
   { \
	 int cl=OSS_Sprintf _x_; \
     if( len+cl+1 < maxLen ) {\
       OSS_StrCpy( h->osHdl, lb, bp );\
       bp += cl; len += cl; \
     }\
   }
 
   int len=0, i;
   char *bp = buffer;
   char lb[80];
   OSS_HANDLE *o = h->osHdl;
   MACCESS ma = h->ma;

   if( maxLen < 1 ) 
	   return MSCAN_ERR_BADPARAMETER;

   *bp = '\0';

   ADDSTR((o,lb, "%s\n", RCSid ));
   ADDSTR((o,lb, "MSCAN REGS:\n"));
   ADDSTR((o,lb, " CTL0=%02x CTL1=%02x\n", MSREAD( ma, MSCAN_CTL0 ),
			MSREAD( ma, MSCAN_CTL1 )));
   ADDSTR((o,lb, " RFLG=%02x", MSREAD( ma, MSCAN_RFLG )));
   ADDSTR((o,lb, " TFLG=%02x TIER=%02x\n", MSREAD( ma, MSCAN_TFLG ), 
			MSREAD( ma, MSCAN_TIER )));

   ADDSTR((o,lb, "MSCAN DRIVER:\n"));
   ADDSTR((o,lb, " txPrio: "));
   for( i=0; i<MSCAN_NTXBUFS; i++ ){
	   ADDSTR((o,lb,"%d ", h->txPrio[i] ));
   }
   
   ADDSTR((o,lb, "\nMESSAGE OBJECTS:\n"));
   for( i=0; i<MSCAN_NUM_OBJS; i++ ){
	   MSG_OBJ *obj = &h->msgObj[i];

	   if( obj->q.dir != MSCAN_DIR_DIS ){
		   ADDSTR((o,lb, " OBJ %d: %s\n", i, obj->q.dir == MSCAN_DIR_RCV ? 
					"rx":"tx"));
   
		   if( obj->q.dir == MSCAN_DIR_XMT ){
			   ADDSTR((o,lb, "  txbUsed: %x txNxtPrio %d txSentPrio %d\n",
						obj->txbUsed, obj->txNxtPrio, obj->txSentPrio ));
		   }
		   ADDSTR((o,lb, "  totEntries: %d filled: %d\n", obj->q.totEntries, 
					obj->q.filled ));
		   
	   }
   }
   return 0;
}


static void DumpFilter( MSCAN_HANDLE *h, char *msg, const MSCAN_FILTER *f )
{
	DBGWRT_2((DBH, " %s: code=%08x%s%s mask=%08x%s\n",
			  msg,
			  f->code, 
			  (f->cflags & MSCAN_EXTENDED) 	? "x" : "",
			  (f->cflags & MSCAN_RTR)		? " RTR" : "",
			  f->mask,
			  (f->mflags & MSCAN_RTR)		? " RTR" : "" ));			  
}

static void DumpFrame( MSCAN_HANDLE *h, char *msg, const MSCAN_FRAME *frm )
{
#ifdef DBG
	int i;
	char buf[100];
	char *s = buf;

	s += OSS_Sprintf( h->osHdl, s, "%s: ID=0x%08lx%s%s data=", 
					  msg,
					  frm->id, 
					  (frm->flags & MSCAN_EXTENDED) ? "x":"", 
					  (frm->flags & MSCAN_RTR) ? " RTR":"");

	for(i=0; i<frm->dataLen; i++ ){
		s += OSS_Sprintf( h->osHdl, s, "%02x ", frm->data[i] );
	}

	DBGWRT_2((DBH, "%s\n", buf ));
#endif /* DBG */
}
