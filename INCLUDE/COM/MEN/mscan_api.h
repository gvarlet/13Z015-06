/***********************  I n c l u d e  -  F i l e  ************************/
/*!  
 *        \file  mscan_api.h
 *
 *      \author  klaus.popp@men.de
 *        $Date: 2010/02/25 18:04:54 $
 *    $Revision: 2.8 $
 * 
 *  	 \brief  Header file for MSCAN_API
 *                      
 *     Switches: -
 */
/*-------------------------------[ History ]---------------------------------
 *
 * $Log: mscan_api.h,v $
 * Revision 2.8  2010/02/25 18:04:54  amorbach
 * R: driver ported to MDIS5, new MDIS_API and men_typs
 * M: Change return value of mscan_init() to MDIS_PATH
 *
 * Revision 2.7  2008/09/26 12:13:50  ufranke
 * R: diab compiler warnings
 * M: cosmetics
 *
 * Revision 2.6  2005/03/24 10:18:32  kp
 * Added missing baudrate code MSCAN_BR_100K. Added low baudrates 20K/10K.
 * Existing applications using MSCAN_BR_ enum values higher then MSCAN_BR_125K
 * must be recompiled!
 *
 * Revision 2.5  2003/07/11 09:25:49  kp
 * added mscan_dump_internals
 *
 * Revision 2.4  2003/04/02 08:37:15  kp
 * added __MAPILIB to all lib functions
 *
 * Revision 2.3  2003/02/03 10:42:59  kp
 * First alpha release to SH Winding
 *
 * Revision 2.2  2003/01/29 14:03:20  kp
 * many changes for initial implementation
 *
 * Revision 2.1  2002/11/19 16:59:54  kp
 * Checkin to produce doxygen specification from source
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2002 by MEN mikro elektronik GmbH, Nuernberg, Germany 
 ****************************************************************************/

#ifndef _MSCAN_API_H
#define _MSCAN_API_H

#ifdef __cplusplus
	extern "C" {
#endif

/*--------------------------------------+
|   TYPEDEFS                            |
+--------------------------------------*/

/** Bitrate values */
typedef enum {
	MSCAN_BR_1MB=0,				/**< 1 MBit/s */
	MSCAN_BR_800K,				/**< 800 kBit/s */
	MSCAN_BR_500K,				/**< 500 kBit/s */
	MSCAN_BR_250K,				/**< 250 kBit/s */
	MSCAN_BR_125K,				/**< 125 kBit/s */
	MSCAN_BR_100K,				/**< 100 kBit/s (not defined in DS102!) */
	MSCAN_BR_50K,				/**< 50 kBit/s */
	MSCAN_BR_20K,				/**< 20 kBit/s */
	MSCAN_BR_10K				/**< 10 kBit/s */
} MSCAN_BITRATE;

/** Error entry codes */
typedef enum {
	MSCAN_BUSOFF_SET=1,			/**< controller entered bus off state  */
	MSCAN_BUSOFF_CLR,			/**< controller left bus off state  */
	MSCAN_WARN_SET,				/**< controller entered error passive state */
	MSCAN_WARN_CLR,				/**< controller left error passive state  */
	MSCAN_QOVERRUN,				/**< object's receive fifo overflowed  */
	MSCAN_DATA_OVERRUN			/**< controller's FIFO overflowed  */
} MSCAN_ERRENTRY_CODE;

/** Message object direction specifier */
typedef enum {
	MSCAN_DIR_DIS,				/**< object disabled  */
	MSCAN_DIR_RCV,				/**< direction=receive  */
	MSCAN_DIR_XMT				/**< direction=transmit  */
} MSCAN_DIR;

/** CAN message and filter flags */
typedef enum {
	MSCAN_EXTENDED=0x1,			/**< interpret ID as extended ID */
	MSCAN_RTR=0x2,				/**< remote transmit request bit */
	MSCAN_USE_ACCFIELD=0x4		/**< use \em accField filter  */
} MSCAN_FLAGS;

/** CAN frame */
typedef struct{
	u_int32 id;					/**< CAN ID of frame */
	u_int8  flags;				/**< ORed flags: MSCAN_EXTENDED/MSCAN_RTR */
	u_int8  dataLen;			/**< data length (0..8) */
	u_int8 	data[8];			/**< data */
} MSCAN_FRAME;

/** CAN node status */
typedef enum {
    /** node is error active (normal operation)  */
	MSCAN_NS_ERROR_ACTIVE,		
    /** node is error passive */
	MSCAN_NS_ERROR_PASSIVE, 
    /** node is bus off */
	MSCAN_NS_BUS_OFF
} MSCAN_NODE_STATUS;

/** macro to test an ID of the individual ID filter */
#define MSCAN_ACCFIELD_GET(field,id)  (field[(id)>>3] & (0x80>>((id)&7)))

/** macro to set an ID of the individual ID filter */
#define MSCAN_ACCFIELD_SET(field,id)  (field[(id)>>3] |= (0x80>>((id)&7)))

/** macro to clear an ID of the individual ID filter */
#define MSCAN_ACCFIELD_CLR(field,id)  (field[(id)>>3] &= ~(0x80>>((id)&7)))

/** MSCAN filter definition */
typedef struct{

    /** Acceptance code.
	 * Each non-masked bit must match the received CAN frame's ID.
	 *
	 * - For standard IDs: Bits 10..0
	 * - For extended IDs: Bits 28..0
	 */
	u_int32 code;					

    /** Acceptance mask. 
	 * 	Any bit set to zero is compared, any bit set to 1 is ignored
	 *  For bit numbers, see \em code field.
	 */
	u_int32 mask;				

    /** Code flags. 
	 * ORed flags value of MSCAN_EXTENDED or MSCAN_RTR
	 *
	 * - #MSCAN_EXTENDED: setup filter for extended identifiers. Block
	 *   standard identifiers.	
	 *
	 * - #MSCAN_RTR: If mflags.MSCAN_RTR is set, then the received frame's
	 *   RTR bit must be of the same value as this bit.
	 */
	u_int32 cflags;			

	/** Mask flags. 
	 *  
	 * - #MSCAN_RTR: If set, then cflags.MSCAN_RTR bit must 
	 *   match received frame's RTR.
	 *
	 * - #MSCAN_USE_ACCFIELD: If set, perform additional filtering
	 *  with \em accField below.
	 */
	u_int32 mflags;	

	/** acceptance field. 
	 * This is a bitfield to mask any individual ID
	 * 
	 * This field is only used for Rx message objects and for CAN
	 * frames with standard identifier. To enable this additional 
	 * filtering, mflags.MSCAN_USE_ACCFIELD must be set.
	 *
	 * This field is ignored for the global HW filters and for 
	 * Rx objects configured for extended identifiers.
	 *
	 * \em accField[0] contains the bits for IDs 0..7 in ascending order 
	 * (Bit 7=ID0, Bit0=ID7). 
	 *
	 * Any ID whose bit is set in this field, passes the filter, and 
	 * ID whose bit is clear will be blocked by the filter.
	 *
	 * use #MSCAN_ACCFIELD_GET, #MSCAN_ACCFIELD_GET, #MSCAN_ACCFIELD_CLR
	 * to set the individual bits.
	 */
	u_int8 accField[256];

} MSCAN_FILTER;


/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/
/*----- API Error codes -----------------------------------------------------*/
#define MSCAN_ERR_BADSPEED     	(ERR_DEV+1) /**< bitrate not supported */
#define MSCAN_ERR_NOMESSAGE     (ERR_DEV+8)	/**< no message in receive buffer*/
#define	MSCAN_ERR_BADTMDETAILS	(ERR_DEV+12)/**< illegal timing details */
#define	MSCAN_ERR_BADMSGNUM	(ERR_LL_ILL_CHAN)/**< illegal message object nr. */
#define	MSCAN_ERR_BADDIR		(ERR_DEV+14)/**< illegal message direction */
#define	MSCAN_ERR_QFULL			(ERR_DEV+15)/**< transmit queue full */
#define MSCAN_ERR_SIGBUSY		(ERR_DEV+16)/**< signal already installed */
#define	MSCAN_ERR_BADPARAMETER	(ERR_LL_ILL_PARAM) /**< bad parameter */
#define	MSCAN_ERR_NOTINIT		(ERR_DEV+17) /**< controller not completely initialized */
#define	MSCAN_ERR_ONLINE		(ERR_DEV+18) /**< controller not disabled */

/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
MDIS_PATH __MAPILIB mscan_init(char *device);
int32 __MAPILIB mscan_term(MDIS_PATH path);
int32 __MAPILIB mscan_set_filter(
	MDIS_PATH path,
	const MSCAN_FILTER *filter1,
	const MSCAN_FILTER *filter2);
int32 __MAPILIB mscan_config_msg(
	MDIS_PATH path,
	u_int32 nr,	
	MSCAN_DIR dir,
	u_int32 qEntries,
	const MSCAN_FILTER *filter );
int32 __MAPILIB mscan_set_bitrate(
	MDIS_PATH path,
	MSCAN_BITRATE bitrate,
	u_int32 spl );
int32 __MAPILIB mscan_set_bustiming(
	MDIS_PATH path,
	u_int8 brp, 
	u_int8 sjw,
	u_int8 tseg1,
	u_int8 tseg2,
	u_int8 spl );
int32 __MAPILIB mscan_read_msg(
	MDIS_PATH path,
	u_int32 nr,
	int32 timeout,
	MSCAN_FRAME *msg );
int32 __MAPILIB mscan_read_nmsg(
	MDIS_PATH path,
	u_int32 nr,
	int32 nFrames,
	MSCAN_FRAME *msg );
int32 __MAPILIB mscan_write_msg(
	MDIS_PATH path,
	u_int32 nr,
	int32 timeout,
	const MSCAN_FRAME *msg);
int32 __MAPILIB mscan_write_nmsg(
	MDIS_PATH path,
	u_int32 nr,
	int32 nFrames,
	const MSCAN_FRAME *msg);
int32 __MAPILIB mscan_read_error(
	MDIS_PATH path,
	u_int32 *errCodeP,
	u_int32 *nrP);
int32 __MAPILIB mscan_set_rcvsig(
	MDIS_PATH path,
	u_int32 nr,
	int32 signal);
int32 __MAPILIB mscan_set_xmtsig(
	MDIS_PATH path,
	u_int32 nr,
	int32 signal);
int32 __MAPILIB mscan_clr_rcvsig(
	MDIS_PATH path,
	u_int32 nr);
int32 __MAPILIB mscan_clr_xmtsig(
	MDIS_PATH path,
	u_int32 nr);
int32 __MAPILIB mscan_queue_status(
	MDIS_PATH path,
	u_int32 nr,
	u_int32 *entriesP,
	MSCAN_DIR *directionP);
int32 __MAPILIB mscan_queue_clear(
	MDIS_PATH path,
	u_int32 nr,
	int txabort);
int32 __MAPILIB mscan_clear_busoff(
	MDIS_PATH path);
int32 __MAPILIB mscan_enable(
	MDIS_PATH path,
	u_int32 enable );
int32 __MAPILIB mscan_rtr(
	MDIS_PATH path,
	u_int32 nr,
	u_int32 id);
int32 __MAPILIB mscan_set_loopback(
	MDIS_PATH path,
	int enable);
int32 __MAPILIB mscan_node_status(
	MDIS_PATH path,
	MSCAN_NODE_STATUS *statusP );
int32 __MAPILIB mscan_error_counters(
	MDIS_PATH path,
	u_int8 *txErrCntP,
	u_int8 *rxErrCntP);
int32 __MAPILIB mscan_dump_internals(
	MDIS_PATH path,
	char *buffer,
	int maxLen );

/* mscan_strings.c */
char * __MAPILIB mscan_errmsg(int32 error);
const char * __MAPILIB mscan_errobj_msg( u_int32 errCode  );

#ifdef __cplusplus
	}
#endif

#endif	/* _MSCAN_API_H */

