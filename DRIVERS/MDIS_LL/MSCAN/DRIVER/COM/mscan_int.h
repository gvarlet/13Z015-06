/***********************  I n c l u d e  -  F i l e  ************************/
/*!  
 *        \file  mscan_int.h
 *
 *      \author  klaus.popp@men.de
 *        $Date: 2011/09/16 15:01:55 $
 *    $Revision: 1.4 $
 * 
 *  	 \brief  Internal header file for MSCAN driver
 *                      
 *     Switches: MSCAN_IS_Z15 	 	 - for MEN own MSCAN implementation
 *				 MSCAN_IS_ODIN		 - for MSCAN MGT5100 implementation
 *               MSCAN_BRP_ALLOW_1   - allow BRP of 1 (not allowed with some 
 *                                     cores
 */
/*-------------------------------[ History ]---------------------------------
 *
 * $Log: mscan_int.h,v $
 * Revision 1.4  2011/09/16 15:01:55  gvarlet
 * R : CAN Odin interface for MPC5200 was not outputing frames on the physical
 *     interface.
 * M : Enable the PSC2_CAN bit on GPIOPCR register of MPC5200.
 *
 * Revision 1.3  2009/11/09 17:16:01  KSchneider
 * R: MSCAN driver used a baud rate prescaler of 1 for some canclock/baud rate constellations. This is not supported by some cores.
 * M: Use a minimum BRP of 2 per default (configurable)
 *
 * Revision 1.2  2003/02/03 10:42:50  kp
 * First alpha release to SH Winding
 *
 * Revision 1.1  2003/01/29 14:03:05  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2003 by MEN mikro elektronik GmbH, Nuernberg, Germany 
 ****************************************************************************/

#ifdef __cplusplus
	extern "C" {
#endif

#if !defined(MSCAN_IS_Z15) && !defined(MSCAN_IS_ODIN)
# error "must define either MSCAN_IS_Z15 or MSCAN_IS_ODIN"
#endif

#if defined(MSCAN_IS_Z15) && defined(MSCAN_IS_ODIN)
# error "don't define MSCAN_IS_Z15 and MSCAN_IS_ODIN together"
#endif


#include <MEN/men_typs.h>   /* system dependend definitions   */
#include <MEN/maccess.h>    /* hw access macros and types     */
#include <MEN/dbg.h>        /* debug functions                */
#include <MEN/oss.h>        /* oss functions                  */
#include <MEN/desc.h>       /* descriptor functions           */
#include <MEN/mdis_api.h>   /* MDIS global defs               */
#include <MEN/mdis_com.h>   /* MDIS common defs               */
#include <MEN/mdis_err.h>   /* MDIS error codes               */
#include <MEN/ll_defs.h>    /* low level driver definitions   */
#include <MEN/ll_entry.h>   /* low level driver jumptable  	  */

#include <MEN/mscan.h>
#include <MEN/mscan_api.h>
#include <MEN/mscan_drv.h>   /* MSCAN driver header file */

/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/
/* debug settings */
#define DBG_MYLEVEL			h->dbgLevel
#define DBH					h->dbgHdl

/* controller register access macros */
#define MSREAD(ma,offs) 			MREAD_D8(ma,offs)
#define MSWRITE(ma,offs,val) 		MWRITE_D8(ma,offs,val)
#define MSSETMASK(ma,offs,mask) 	MSETMASK_D8(ma,offs,mask) 	
#define MSCLRMASK(ma,offs,mask) 	MCLRMASK_D8(ma,offs,mask) 	

/* general MDIS defs */

/* others */
#define MSCAN_NUM_OBJS		10			/**< number of message objects */
#define	MSCAN_ERROR_OBJ		0			/**< msg obj number of error obj  */

/* address space size occupied by MSCAN registers */
#ifdef MSCAN_IS_Z15
# define ADDRSPACE_SIZE	0x100			/* size of address space */
#endif

#ifdef MSCAN_IS_ODIN
# define ADDRSPACE_SIZE	0x80			/* size of address space */
#endif

#define MSCAN_UNASSIGNED	-1			/**< see txPrio */

#define MSCAN_NTXBUFS		3			/**< number of tx buffers of MSCAN */
#define MSCAN_TXB_MASK		0x7			/**< bitmask for all tx buffers  */

#define MSCAN_MAX_LOC_PRIO	0x0f 		/**< see txNxtPrio  */

/** Macro to check if Setstat/Getstat block sizes match */
#define CHK_BLK_SIZE( blk, type ) \
 if( blk->size != sizeof(type) ){\
    DBGWRT_ERR((DBH,"*** MSCAN: wrong blk->size for %s\n", #type ));\
    error = ERR_LL_ILL_PARAM;\
    break;\
 }

/** Macro to lock device semaphore */
/* ??? while( error == ERR_OSS_SIG_OCCURED ) might be a problem in Linux???*/
#define DEVSEM_LOCK(h) \
 if( h->devSemHdl ){ \
     int32 error;\
     do {\
         error=OSS_SemWait( h->osHdl, h->devSemHdl, OSS_SEM_WAITFOREVER );\
     } while( error == ERR_OSS_SIG_OCCURED );\
 }

/** Macro to unlock device semaphore */
#define DEVSEM_UNLOCK(h) \
 if( h->devSemHdl ) OSS_SemSignal( h->osHdl, h->devSemHdl );


/** Macro to allow/disallow a BRP of 1 */
#ifndef MSCAN_BRP_ALLOW_1
	#define MSCAN_MIN_BRP 2
#else
	#define MSCAN_MIN_BRP 1
#endif //MSCAN_BRP_ALLOW_1

#if defined( MSCAN_IS_ODIN )
    /* #defines for MGT5100 & MGT5200B CAN controllers */
    /*! Port Configuration Register GPIO */
    #define GPIOPCR         (0x0B00 + 0xF0000000UL)
    #define PSC2_MASK       0x00000070UL    /* PSC Pin mask */
    #define PSC2_CAN        0x00000010UL    /* PSC CAN config value */
#endif /* CAN_IS_ODIN */

/*-----------------------------------------+
|  TYPEDEFS                                |
+-----------------------------------------*/

/** queue entry structure */
typedef struct mqueue_ent {
	struct mqueue_ent *next;		/**< ptr to next entry */
	union {
		MSCAN_FRAME frm;			/**< data for rx/tx queues */
		MSCAN_READERROR_PB err;		/**< data for error object */
	} d;							/**< union to hold frm or err */
} MQUEUE_ENT;
			
/** queue header structure */
typedef struct {
	MQUEUE_ENT 	*first,				/**< start of memory used for entries */
		        *nxtIn,				/**< next entry to fill */
		        *nxtOut;			/**< next entry to extract */
	u_int32		memAlloc;			/**< allocated mem for entries */
	u_int32		totEntries;			/**< total number of entries */
	u_int32		filled;				/**< number of filled entries */
	u_int8		ready;				/**< flags if queue is fully initialized */
	u_int8		errSent;			/**< flags if overrun error has been sent*/
	u_int8		waiting;			/**< flags read/write waiter waiting  */
	u_int8		_pad;
	MSCAN_DIR	dir;				/**< direction */
	MSCAN_FILTER filter;			/**< rx: local filter */
	OSS_SEM_HANDLE *sem;			/**< semaphore to wake read/write waiter */
} MQUEUE_HEAD;

/** per message object structure */
typedef struct {
	u_int32			nr;				/**< message object number (redundant) */
	MQUEUE_HEAD		q;				/**< message queue header */
	OSS_SIG_HANDLE	*sig;			/**< signal installed */

	/**********************************************************************/
    /** bitfield to record the tx buffers in use by this message object
	 *	bit 2..0 used only.
	 *  bit=1: a tx frame for this msg obj has been assigned to txbuf
	 *  bit=0: no frame assigned
	 */
	u_int8			txbUsed;

	/**********************************************************************/
    /** records next local priority to assign
	 *	Used in the chronological buffer scheduling algorithm
	 *  It is increased by one with every buffer scheduled for transmit.
	 *  When it would reach 0x10, it is reset to 0
	 */
	u_int8			txNxtPrio;

	/**********************************************************************/
    /** records last local priority that has been sent over CANbus
	 *	Used in the chronological buffer scheduling algorithm
	 */
	u_int8			txSentPrio;	
	
} MSG_OBJ;

/** ll handle */
typedef struct {
	/* general */
    int32           memAlloc;		/**< size allocated for the handle */
    OSS_HANDLE      *osHdl;         /**< oss handle */
    OSS_IRQ_HANDLE  *irqHdl;        /**< irq handle */
    DESC_HANDLE     *descHdl;       /**< desc handle */
    MACCESS         ma;             /**< hw access handle */
	MDIS_IDENT_FUNCT_TBL idFuncTbl;	/**< id function table */
	OSS_SEM_HANDLE	*devSemHdl;		/**< device semaphore handle */
	/* debug */
    u_int32         dbgLevel;		/**< debug level */
	DBG_HANDLE      *dbgHdl;        /**< debug handle */

	MSG_OBJ			msgObj[MSCAN_NUM_OBJS]; /**< message object structures */

	/**********************************************************************/
    /** array to record which priority has been assigned to tx buffers
	 *	
	 *  when a buffer is scheduled for transmission, this field is	
	 *  used to record the pending buffer priority
	 *  
	 *  Once the frame has been transmitted, it is reset to MSCAN_UNASSIGNED.
	 *	The buffer priority is coded as follows:
	 *  upper 4 bits: msg obj number
	 *  lower 4 bits: msg obj relative priority (0=highest)
	 */
	int				txPrio[MSCAN_NTXBUFS];

	int				canEnabled;		/**< CAN bus activity enabled  */
	int				busTimingSet; 	/**< user has setup bustiming  */
	int				irqEnabled;		/**< flags M_MK_IRQ_ENABLE issued  */
	u_int32			canClock;		/**< can clockrate in Hz  */
	u_int32			irqCount;		/**< number of irqs occurred  */
	MSCAN_NODE_STATUS nodeStatus; 	/**< current node status (error act..)  */

	/* used to minimize object search loops */
	int32			firstRxObj;		/**< first object configured for Rx  */
	int32			lastRxObj;		/**< last object configured for Rx  */
	int32			firstTxObj;		/**< first object configured for Tx  */
	int32			lastTxObj;		/**< last object configured for Tx  */

	u_int32			maxIrqTime;

	u_int32			minBrp;			/**< minimum baud rate prescaler  */
} MSCAN_HANDLE;


/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/

#ifdef __cplusplus
	}
#endif


