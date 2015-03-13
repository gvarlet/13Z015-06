/*********************  P r o g r a m  -  M o d u l e ***********************/
/*!  
 *        \file  mscan_loopb.c
 *
 *      \author  klaus.popp@men.de
 *        $Date: 2010/02/25 18:04:33 $
 *    $Revision: 1.9 $
 * 
 *  	 \brief  Test tool for MSCAN driver in loopback mode
 *
 *     Since this test runs in loopback mode, no external setup is 
 *	   required (i.e. no second CAN node(
 *
 *     Switches: -
 *     Required: libraries: mdis_api, usr_oss, usr_utl, mscan_api
 */
/*-------------------------------[ History ]---------------------------------
 *
 * $Log: mscan_loopb.c,v $
 * Revision 1.9  2010/02/25 18:04:33  amorbach
 * R: driver ported to MDIS5, new MDIS_API and men_typs
 * M: Change type of path to MDIS_PATH
 *
 * Revision 1.8  2009/05/12 11:50:09  ts
 * R: driver package build failed under windows
 * M: added specifier __MAPILIB to function SigHandler
 *
 * Revision 1.7  2009/05/08 18:12:25  ts
 * R: new CAN IPcore caused error during local loopback test at
 *    high baudrates
 * M: inserted small delay in transmit routine of local loopback test
 *
 * Revision 1.5  2004/06/14 11:58:17  kp
 * cosmetics
 *
 * Revision 1.4  2004/03/19 11:05:45  ub
 * Fixed: Try to terminate non-open path
 *
 * Revision 1.3  2003/07/11 09:25:42  kp
 * added test for mscan_dump_internals
 *
 * Revision 1.2  2003/02/03 10:42:52  kp
 * First alpha release to SH Winding
 *
 * Revision 1.1  2003/01/29 14:03:11  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2003 by MEN mikro elektronik GmbH, Nuernberg, Germany 
 ****************************************************************************/


static const char RCSid[]="$Id: mscan_loopb.c,v 1.9 2010/02/25 18:04:33 amorbach Exp $";

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <MEN/men_typs.h>
#include <MEN/usr_oss.h>
#include <MEN/usr_utl.h>
#include <MEN/mdis_api.h>
#include <MEN/mdis_err.h>
#include <MEN/usr_err.h>
#include <MEN/mscan_api.h>
#include <MEN/mscan_drv.h>		/* only for MSCAN_MAXIRQTIME */

/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/
#define CHK(expression) \
 if( !(expression)) {\
	 printf("\n*** Error during: %s\nfile %s\nline %d\n", \
      #expression,__FILE__,__LINE__);\
      printf("%s\n",mscan_errmsg(UOS_ErrnoGet()));\
     goto ABORT;\
 }

/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
static void usage(void);

static int LoopbBasic( MDIS_PATH path );
static int LoopbTxPrio( MDIS_PATH path );
static int LoopbRxFilter( MDIS_PATH path );
static int LoopbSignals( MDIS_PATH path );
static int LoopbRxOverrun( MDIS_PATH path );

static void DumpFrame( char *msg, const MSCAN_FRAME *frm );
static int CmpFrames( const MSCAN_FRAME *frm1, const MSCAN_FRAME *frm2 );
static void __MAPILIB SigHandler( u_int32 sigCode );

/*--------------------------------------+
|   TYPEDEFS                            |
+--------------------------------------*/
/* test list description */
typedef struct {
	char code;
	char *descr;
	int (*func)(MDIS_PATH path);
} TEST_ELEM;

/*--------------------------------------+
|   GLOBALS                             |
+--------------------------------------*/
/* filters to let everything pass through */
static const MSCAN_FILTER G_stdOpenFilter = { 
	0,
	0xffffffff,
	0,
	0 
};
static const MSCAN_FILTER G_extOpenFilter = { 
	0,
	0xffffffff,
	MSCAN_EXTENDED,
	0 
};

static TEST_ELEM G_testList[] = {
	{ 'a', "Basic Tx/Rx", LoopbBasic },
	{ 'b', "Tx chronological", LoopbTxPrio },
	{ 'c', "Rx filter", LoopbRxFilter },
	{ 'd', "Rx/Tx signals", LoopbSignals },
	{ 'e', "Rx FIFO overrun", LoopbRxOverrun },
	{ 0, NULL, NULL }
};

static int G_sigUos1Cnt, G_sigUos2Cnt;	/* signal counters */
static int G_endMe;

/*
ToDo:
 - read with timeout

Test coverage:

Test:                  A  B  C  D  E	
--------------------   -- -- -- -- --
mscan_init             ALL

mscan_term             ALL  

mscan_set_filter       -  -  *  -  -

mscan_config_msg	   ALL

mscan_set_bitrate	   ALL

mscan_set_bustiming    -  -  -  -  -

mscan_read_msg         *  *  *  -  -

mscan_read_nmsg        -  -  -  -  *

mscan_write_msg        *  -  *  *  *

mscan_write_nmsg       -  *	 -	-  -

mscan_read_error       -  -  -  -  *

mscan_set_rcvsig       -  -  -  *  *

mscan_set_xmtsig       -  -  -  *  -

mscan_clr_rcvsig       -  -  -  *  *

mscan_clr_xmtsig       -  -  -  *  -

mscan_queue_status     -  -  *  *  *

mscan_queue_clear      -  -  -  -  *
 txabort               -  -  -  -  -

mscan_clear_busoff     -  -  -  -  -

mscan_enable		   ALL
 disable               -  -  *  -  -

mscan_rtr              -  -  *  -  -

mscan_set_loopback	   ALL	

mscan_node_status      -  -  -  -  -

mscan_error_counters   -  -  -  -  -

mscan_errmsg           ALL

mscan_errobj_msg       -  -  -  -  *

*/


/**********************************************************************/
/** Print program usage
 */
static void usage(void)
{
	TEST_ELEM *te=G_testList;

	printf(
		"usage: mscan_loopb [<opts>] <device> [<opts>]\n"
		"Options:\n"
		"  -b=<code>    bitrate code (0..8)              [0]\n"
		"                  0=1MBit 1=800kbit 2=500kbit 3=250kbit 4=125kbit\n"
		"                  5=100kbit 6=50kbit 7=20kbit 8=10kbit\n"
		"  -n=<runs>    number of runs through all tests [1]\n"
		"  -s           stop on first error ............ [no]\n"
		"  -t=<list>    perform only those tests listed: [all]\n");

	while( te->func ){
		printf("    %c: %s\n", te->code, te->descr );
		te++;
	}

	printf("(c) 2003 by MEN Mikro Elektronik GmbH\n%s\n", RCSid );
}

/**********************************************************************/
/** Program entry point
 * \return success (0) or error (1)
 */
int main( int argc, char *argv[] )
{
	int32 ret=1, n, error;
    MDIS_PATH path=-1;
	int stopOnFirst, runs, run, errCount=0;
	u_int32 bitrate, spl=0;
	char	*device,*str,*errstr,buf[40],*testlist;
	TEST_ELEM *te;
	char *tCode;

	G_endMe = FALSE;
	/*--------------------+
    |  check arguments    |
    +--------------------*/
	if ((errstr = UTL_ILLIOPT("n=sb=t=?", buf))) {	/* check args */
		printf("*** %s\n", errstr);
		return(1);
	}

	if (UTL_TSTOPT("?")) {						/* help requested ? */
		usage();
		return(1);
	}

	/*--------------------+
    |  get arguments      |
    +--------------------*/
	for (device=NULL, n=1; n<argc; n++)
		if (*argv[n] != '-') {
			device = argv[n];
			break;
		}

	if (!device) {
		usage();
		return(1);
	}

	bitrate  = ((str = UTL_TSTOPT("b=")) ? atoi(str) : 0);
	runs	 = ((str = UTL_TSTOPT("n=")) ? atoi(str) : 1);
	stopOnFirst = !!UTL_TSTOPT("s");

	UOS_SigInit( SigHandler );

	/*--------------------+
    |  open path          |
    +--------------------*/
	CHK( (path = mscan_init(device)) >= 0 );

	CHK( M_setstat( path, MSCAN_MAXIRQTIME, 0 ) == 0 );
	/*--------------------+
    |  config             |
    +--------------------*/
	CHK( mscan_set_bitrate( path, (MSCAN_BITRATE)bitrate, spl ) == 0 );

	/*--- config error object ---*/
	CHK( mscan_config_msg( path, 0, MSCAN_DIR_RCV, 10, NULL ) == 0 );

	/*--- put CAN in loopback mode ---*/
	CHK( mscan_set_loopback( path, TRUE ) == 0 );

	/*--- enable bus ---*/
	CHK( mscan_enable( path, TRUE ) == 0 );

	/*-------------------+
	|  Perform tests     |
	+-------------------*/
	testlist  = ((str = UTL_TSTOPT("t=")) ? 
				 str : "abcde"/*mnopqrstuvxyz"*/);

	for( tCode=testlist; *tCode; tCode++ ){

		for( te=G_testList; te->func; te++ )
			if( *tCode == te->code )
				break;

		if( te->func == NULL ){
			printf("Unknown test: %c\n", *tCode );
			goto ABORT;
		}

		for( run=1; run<=runs; run++ ){
			if( G_endMe )
				goto ABT1;

			printf("=== Performing test %c: %-43s (Run %d/%d) ===\n",
				   te->code, te->descr, run, runs );

			error = te->func( path );
			if( error )
				errCount++;

			printf( "Test %c: ", te->code);
			printf( "%s\n", error ? "FAILED" : "ok" );

			if( error && stopOnFirst )
				goto ABT1;
		}
	}
 ABT1:
	printf("------------------------------------------------\n");
	printf("TEST RESULT: %d errors\n", errCount );

	{
		u_int32 maxIrqTime;

		CHK( M_getstat( path, MSCAN_MAXIRQTIME, (int32*)&maxIrqTime ) == 0 );
		printf("Max irqtime=%d (internal ticks)\n", maxIrqTime );
	}

	ret = 0;
	CHK( mscan_enable( path, FALSE ) == 0 );
	CHK( mscan_term(path) == 0 );
	path=-1;

 ABORT:
	UOS_SigExit();

	if( path != -1 ) {
        mscan_enable( path, FALSE );
		mscan_term(path);
    }

	return(ret);
}

static void __MAPILIB SigHandler( u_int32 sigCode )
{
	switch( sigCode ){
	case UOS_SIG_USR1:
		G_sigUos1Cnt++;	
		break;
	case UOS_SIG_USR2:
		G_sigUos2Cnt++;	
		break;
	default:
		G_endMe = TRUE;
	}
}


/**********************************************************************/
/** Test a: Basic Tx/Rx test 
 * 
 * Configures:
 * - one tx object
 * - two rx objects (one for standard, one for extended IDs)
 *
 * Global filter are configured to let all messages pass through
 * Rx object filters are configured to let all messages pass through
 *
 * Each frame of table \em txFrm is sent and it is checked if the frame
 * could be received correctly on the expected Rx object.
 *
 * \return 0=ok, -1=error
 */
static int LoopbBasic( MDIS_PATH path )
{
	int i, rxObj, rv = -1;
	const int txObj = 5;
	const int rxObj1 = 1;
	const int rxObj2 = 2;

	/* frames to send */
	static const MSCAN_FRAME txFrm[] = {
		/* ID,  flags,          dlen, data */
		{ 0x12, 0,				1,   { 0xa5 } },
		{ 0x45, 0,				8,   { 0x01, 0x02, 0x03, 0x04, 0x05, 
									   0x06, 0x07, 0x08 } },
		{ 0x13218765,  MSCAN_EXTENDED, 2, { 0x99, 0xcc } },
		{ 0x55, 0,				4,   { 0xff, 0x00, 0x7f, 0x1e } },
		{ 0x124, 0,				0,   { 0 } }
	};
	MSCAN_FRAME rxFrm;

	/* Tx object */
	CHK( mscan_config_msg( path, txObj, MSCAN_DIR_XMT, 10, NULL ) == 0 );

	/* Rx object for standard messages */
	CHK( mscan_config_msg( path, rxObj1, MSCAN_DIR_RCV, 20, 
						   &G_stdOpenFilter ) == 0 );

	/* Rx object for extended messages */
	CHK( mscan_config_msg( path, rxObj2, MSCAN_DIR_RCV, 20, 
						   &G_extOpenFilter ) == 0 );


	for( i=0; i<sizeof(txFrm)/sizeof(MSCAN_FRAME); i++ ){

		/* send one frame */
		CHK( mscan_write_msg( path, txObj, 1000, &txFrm[i] ) == 0 );

		/*
		 * with the reworked CAN core (Rx FIFO treshold reduced) the time to
		 * process Interrupts is reduced so that in high baudrates the CAN
		 * local loopback causes an error (lost Rx Objects). However, this can
		 * occur here only since only at local loopback theres such tight
		 * coupling between Tx and Rx Interrupt. So leave a small delay
		 * between sent Frames.
		 */
		UOS_Delay(2);

		/* wait for frame on correct object */
		rxObj = txFrm[i].flags & MSCAN_EXTENDED ? rxObj2 : rxObj1;

		CHK( mscan_read_msg( path, rxObj, 1000, &rxFrm ) == 0 );

		/* check if received correctly */
		if( CmpFrames( &rxFrm, &txFrm[i] ) != 0 ){
			printf("Incorrect Frame received\n");
			DumpFrame( "Sent", &txFrm[i] );
			DumpFrame( "Recv", &rxFrm );
			CHK(0);
		}
	}

    rv = 0;

 ABORT:
	mscan_config_msg( path, txObj, MSCAN_DIR_DIS, 0, NULL );
	mscan_config_msg( path, rxObj1, MSCAN_DIR_DIS, 0, NULL );
	mscan_config_msg( path, rxObj2, MSCAN_DIR_DIS, 0, NULL );

	return rv;

}

/**********************************************************************/
/** Test b: Tx priority test
 * 
 * Verifies that the tx messages are sent in the chronological order (on the
 * same tx object)
 *
 * \return 0=ok, -1=error
 */
static int LoopbTxPrio( MDIS_PATH path )
{
	int i, rv = -1;
	const int txObj = 1;
	const int rxObj = 2;
	/*const int nFrames = 100;*/
	#define nFrames 100			/* Ultra-C didn't like line above */
	MSCAN_FRAME txFrm[nFrames], rxFrm;

	/* Tx object */
	CHK( mscan_config_msg( path, txObj, MSCAN_DIR_XMT, nFrames, NULL ) == 0 );

	/* Rx object for standard messages */
	CHK( mscan_config_msg( path, rxObj, MSCAN_DIR_RCV, nFrames, 
						   &G_stdOpenFilter ) == 0 );

	
	/* build frames */
	for( i=0; i<nFrames; i++ ){
		txFrm[i].id = i;
		txFrm[i].dataLen = 0;
		txFrm[i].flags = 0;				
	}

	/* send frames at once */
	CHK( mscan_write_nmsg( path, txObj, nFrames, txFrm ) == nFrames );
		
	for( i=0; i<nFrames; i++ ){
		
		/* wait for frame on correct object */
		CHK( mscan_read_msg( path, rxObj, 1000, &rxFrm ) == 0 );

		/* check if received correctly */
		CHK( rxFrm.id == i );
	}

    rv = 0;

 ABORT:
	{
		char buf[512];
		mscan_dump_internals( path, buf, sizeof(buf));
		printf("DRIVER INTERNALS:\n%s\n", buf );
	}
	mscan_config_msg( path, txObj, MSCAN_DIR_DIS, 0, NULL );
	mscan_config_msg( path, rxObj, MSCAN_DIR_DIS, 0, NULL );

	return rv;

}

static int RxFilterChkObj( 
	MDIS_PATH path, 
	u_int32 obj, 
	MSCAN_FRAME *expObj,
	u_int32 firstId, 
	u_int32 lastId)
{
	u_int32 id;
	MSCAN_FRAME rxFrm;

	for( id=firstId; id<=lastId; id++ ){
		expObj->id = id;
		
		CHK( mscan_read_msg( path, obj, -1, &rxFrm ) == 0 );

		/* check if received correctly */
		if( CmpFrames( &rxFrm, expObj ) != 0 ){
			printf("Rx object %d: Incorrect Frame received\n", obj);
			DumpFrame( "Exp.", expObj );
			DumpFrame( "Recv", &rxFrm );
			CHK(0);
		}
	}
	return 0;

 ABORT:
	return -1;
}
						   
/**********************************************************************/
/** Test c: Rx filter
 * 
 * Configures:
 * - one tx object
 *
 * Two global filters are applied:
 * - Filter 1: Pass through Std IDs 0x000..0x3ff, RTR ignored
 * - Filter 2: Pass through Ext IDs 0x20000..0x3FFFF, RTR ignored
 *
 * Rx objects are setup:
 * - Obj 1: Std Id 	0x25e
 * - Obj 2: Std Id  0x030..0x033
 * - Obj 3: Std Id  0x200..0x3ff
 * - Obj 4: Std Id  0x400..0x7ff
 * - Obj 5: Ext Id  0x20200..0x202ff
 * - Obj 6: Std Id  0x25e RTR!
 * - Obj 7: Std Id  Individual IDs 0x008, 0x022, 0x100
 * 
 * Tx frames are sent:
 * - all Std Ids 0x000..0x7ff
 * -     Ext Ids 0x1ff00..0x20500
 * -     Std Ids 0x25e.. RTR
 * -     Ext Ids 0x20200 RTR
 *
 * \return 0=ok, -1=error
 */
static int LoopbRxFilter( MDIS_PATH path )
{
	static const MSCAN_FILTER hwflt[] = { 
		/* code, mask, cflags, mflags */
		{ 0x000, 0x3ff, 0, 0 },							/* flt1 */
		{ 0x20000, 0x1ffff, MSCAN_EXTENDED, 0 }			/* flt2 */
	};

	static const MSCAN_FILTER flt[] = { 
		/* code, mask, cflags, mflags */
		{ 0x25e, 0x000, 0, MSCAN_RTR },					/* obj1 */
		{ 0x030, 0x003, 0, 0 },							/* obj2 */
		{ 0x200, 0x1ff, 0, MSCAN_RTR },					/* obj3 */
		{ 0x400, 0x3ff, 0, 0 },							/* obj4 */
		{ 0x20200, 0x000ff, MSCAN_EXTENDED, MSCAN_RTR },/* obj5 */
		{ 0x25e, 0x000, MSCAN_RTR, MSCAN_RTR }			/* obj6 */
	};
	MSCAN_FILTER flt7;

	const int txObj=8;
	int i, rv = -1;
	MSCAN_FRAME txFrm;
	u_int32 id;
	u_int32 entries;

	CHK( mscan_enable( path, FALSE ) == 0 ); /* disable CAN */

	CHK( mscan_set_filter( path, &hwflt[0], &hwflt[1] ) == 0 );

	/* create Rx objects */
	for( i=1; i<7; i++ )
		CHK( mscan_config_msg( path, i, MSCAN_DIR_RCV, 0x400, &flt[i-1] )==0 );

	flt7.code = 0x00000000;
	flt7.mask = 0xFFFFFFFF;
	flt7.cflags = 0;
	flt7.mflags = MSCAN_USE_ACCFIELD;
	  
	memset( &flt7.accField, 0, sizeof(flt7.accField) );
	MSCAN_ACCFIELD_SET( flt7.accField, 0x008 );
	MSCAN_ACCFIELD_SET( flt7.accField, 0x022 );
	MSCAN_ACCFIELD_SET( flt7.accField, 0x100 );

	CHK( mscan_config_msg( path, 7, MSCAN_DIR_RCV, 10, &flt7 )==0 );
	

	/* Tx object */
	CHK( mscan_config_msg( path, txObj, MSCAN_DIR_XMT, 10, NULL ) == 0 );

	CHK( mscan_enable( path, TRUE ) == 0 ); /* enable CAN */
	
	/*--- send standard frames ---*/
	txFrm.dataLen = 0;
	txFrm.flags = 0;

	for( id=0; id<0x7ff; id++ ){
		txFrm.id = id;
		CHK( mscan_write_msg( path, txObj, 1000, &txFrm ) == 0 );
	}
	/*--- send extended frames ---*/
	txFrm.dataLen = 0;
	txFrm.flags = MSCAN_EXTENDED;

	for( id=0x1ff00; id<0x20500; id++ ){
		txFrm.id = id;
		CHK( mscan_write_msg( path, txObj, 1000, &txFrm ) == 0 );
	}

	/*--- send RTR frames ---*/
	txFrm.dataLen = 0;
	txFrm.flags = MSCAN_RTR;

	for( id=0x25e; id<0x25f; id++ ){
		CHK( mscan_rtr( path, txObj, id ) == 0 );
	}

	txFrm.flags = MSCAN_EXTENDED | MSCAN_RTR;

	for( id=0x202000; id<0x202001; id++ ){
		txFrm.id = id;
		CHK( mscan_write_msg( path, txObj, 1000, &txFrm ) == 0 );
	}

	/* wait until everything transmitted */
	printf(" Waiting transmit complete\n");
	do {
		CHK( mscan_queue_status( path, txObj, &entries, NULL ) == 0 );
	} while( entries != 10 );
			
	UOS_Delay(200);
	printf(" Verifying frames\n");
	/*------------------------------------------------------------+
	|  Check if frames are correctly received on every Rx object  |
	+------------------------------------------------------------*/
	/* obj 1:  */
	txFrm.dataLen = 0;
	txFrm.flags = 0;

	CHK( RxFilterChkObj( path, 1, &txFrm, 0x25e, 0x25e ) == 0 );
	mscan_queue_status( path, 1, &entries, NULL );
	CHK( entries == 0 );

	/* obj 2:  */
	CHK( RxFilterChkObj( path, 2, &txFrm, 0x030, 0x033 ) == 0 );
	mscan_queue_status( path, 2, &entries, NULL );
	CHK( entries == 0 );

	/* obj 3:  */
	CHK( RxFilterChkObj( path, 3, &txFrm, 0x200, 0x25d ) == 0 );
	CHK( RxFilterChkObj( path, 3, &txFrm, 0x25f, 0x3ff ) == 0 );
	txFrm.flags = 0;
	mscan_queue_status( path, 3, &entries, NULL );
	CHK( entries == 0 );

	/* obj 4: (no frames), blocked by global filter */
	mscan_queue_status( path, 4, &entries, NULL );
	CHK( entries == 0 );

	/* obj 5:  */
	txFrm.flags = MSCAN_EXTENDED;
	CHK( RxFilterChkObj( path, 5, &txFrm, 0x20200, 0x202ff ) == 0 );
	mscan_queue_status( path, 5, &entries, NULL );
	CHK( entries == 0 );

	/* obj 6: */
	txFrm.flags = MSCAN_RTR;
	CHK( RxFilterChkObj( path, 6, &txFrm, 0x25e, 0x25e ) == 0 );
	mscan_queue_status( path, 6, &entries, NULL );
	CHK( entries == 0 );

	/* obj 7: */
	txFrm.flags = 0;
	CHK( RxFilterChkObj( path, 7, &txFrm, 0x008, 0x008 ) == 0 );
	CHK( RxFilterChkObj( path, 7, &txFrm, 0x022, 0x022 ) == 0 );
	CHK( RxFilterChkObj( path, 7, &txFrm, 0x100, 0x100 ) == 0 );
	mscan_queue_status( path, 7, &entries, NULL );
	CHK( entries == 0 );

    rv = 0;

 ABORT:
	mscan_enable( path, FALSE );
	mscan_config_msg( path, txObj, MSCAN_DIR_DIS, 0, NULL );
	for( i=1; i<8; i++ )
		mscan_config_msg( path, i, MSCAN_DIR_DIS, 0, NULL );
	mscan_set_filter( path, &G_stdOpenFilter, &G_extOpenFilter );
	mscan_enable( path, TRUE );

	return rv;

}


/**********************************************************************/
/** Test d: Rx/Tx signals
 * 
 * Configures Tx objects:
 * - Obj 3: UOS_SIG_USR1
 * - Obj 4: no signal
 *
 * Rx objects are setup, on both a signal is installed:
 * - Obj 1: Std Id 	ALL, no signal
 * - Obj 2: Ext Id  ALL, UOS_SIG_USR2
 *
 * \return 0=ok, -1=error
 */
static int LoopbSignals( MDIS_PATH path )
{
	int rv = -1, i;
	const int txObj1=3, txObj2=4;
	const int rxObj1=1, rxObj2=2;
	MSCAN_FRAME txFrm;
	u_int32 entries;

	G_sigUos1Cnt = 0;
	G_sigUos2Cnt = 0;

	UOS_SigInstall( UOS_SIG_USR1 );
	UOS_SigInstall( UOS_SIG_USR2 );
	
	CHK( mscan_config_msg( path, txObj1, MSCAN_DIR_XMT, 10, NULL ) == 0 );
	CHK( mscan_config_msg( path, txObj2, MSCAN_DIR_XMT, 10, NULL ) == 0 );
	CHK( mscan_config_msg( path, rxObj1, MSCAN_DIR_RCV, 10, 
						   &G_stdOpenFilter ) == 0 );
	CHK( mscan_config_msg( path, rxObj2, MSCAN_DIR_RCV, 10, 
						   &G_extOpenFilter ) == 0 );

	CHK( mscan_set_rcvsig( path, rxObj2, UOS_SIG_USR2 ) == 0 );
	CHK( mscan_set_rcvsig( path, rxObj2, UOS_SIG_USR2 ) == -1 );
	CHK( UOS_ErrnoGet() == MSCAN_ERR_SIGBUSY );

	CHK( mscan_set_rcvsig( path, txObj1, UOS_SIG_USR1 ) == -1 );
	CHK( UOS_ErrnoGet() == MSCAN_ERR_BADDIR );
	CHK( mscan_set_xmtsig( path, txObj1, UOS_SIG_USR1 ) == 0 );


	/* send std/ext frames */
	for( i=0; i<10; i++ ){		
		txFrm.id = i;
		txFrm.flags = 0;
		txFrm.dataLen = 0;
		CHK( mscan_write_msg( path, txObj1, -1, &txFrm ) == 0 );

		if( i<7 ){
			txFrm.id = i+10;
			txFrm.flags = MSCAN_EXTENDED;
			txFrm.dataLen = 0;
			CHK( mscan_write_msg( path, txObj2, -1, &txFrm ) == 0 );
		}
	}

	UOS_Delay( 500 );			/* be sure all frames sent */

	mscan_queue_status( path, rxObj1, &entries, NULL );
	CHK( entries == 10 );

	mscan_queue_status( path, rxObj2, &entries, NULL );
	CHK( entries == 7 );

	CHK( G_sigUos1Cnt == 10 );
	CHK( G_sigUos2Cnt == 7 );


	rv = 0;
 ABORT:
	mscan_clr_rcvsig( path, rxObj2 );
	mscan_clr_xmtsig( path, txObj1 );
	mscan_config_msg( path, txObj1, MSCAN_DIR_DIS, 0, NULL );
	mscan_config_msg( path, txObj2, MSCAN_DIR_DIS, 0, NULL );
	mscan_config_msg( path, rxObj1, MSCAN_DIR_DIS, 0, NULL );
	mscan_config_msg( path, rxObj2, MSCAN_DIR_DIS, 0, NULL );

	UOS_SigRemove( UOS_SIG_USR1 );
	UOS_SigInstall( UOS_SIG_USR2 );
	return rv;
}

/**********************************************************************/
/** Test d: Rx overruns
 * 
 * - Install signal for error object
 * - Overruns receiver object. 
 * - Checks if correct number of frames in FIFO
 * - Clears Rx FIFO
 * - 
 *
 * \return 0=ok, -1=error
 */
static int LoopbRxOverrun( MDIS_PATH path )
{
	int rv = -1, i;
	const int txObj=3;
	const int rxObj=1;
	MSCAN_FRAME txFrm, rxFrm[30];
	u_int32 entries, errCode, objNr;

	G_sigUos1Cnt = 0;
	CHK( mscan_set_rcvsig( path, 0, UOS_SIG_USR1 ) == 0 );

	UOS_SigInstall( UOS_SIG_USR1 );

	CHK( mscan_config_msg( path, txObj, MSCAN_DIR_XMT, 15, NULL ) == 0 );
	CHK( mscan_config_msg( path, rxObj, MSCAN_DIR_RCV, 10, 
						   &G_stdOpenFilter ) == 0 );


	/* send 13 frames... */
	for( i=0; i<13; i++ ){
		txFrm.id = i;
		txFrm.flags = 0;
		txFrm.dataLen = 1;
		txFrm.data[0] = i & 0xff;
		CHK( mscan_write_msg( path, txObj, -1, &txFrm ) == 0 );
	}

	UOS_Delay( 200 );			/* be sure all frames sent */

	CHK( mscan_queue_status( path, rxObj, &entries, NULL ) == 0 );
	CHK( entries == 10 );

	/* verify error object */
	CHK( mscan_queue_status( path, 0, &entries, NULL ) == 0 );
	CHK( entries == 1 );
	CHK( mscan_read_error( path, &errCode, &objNr ) == 0 );
	CHK( errCode == MSCAN_QOVERRUN );
	CHK( objNr == rxObj );
	CHK( G_sigUos1Cnt == 1 );

	/* read some frames... */
	i = mscan_read_nmsg( path, rxObj, 30, rxFrm );
	CHK( i == 10 );

	/* overrun again receiver... */
	for( i=0; i<13; i++ ){
		txFrm.id = i;
		txFrm.flags = 0;
		txFrm.dataLen = 1;
		txFrm.data[0] = i & 0xff;
		CHK( mscan_write_msg( path, txObj, -1, &txFrm ) == 0 );
	}

	UOS_Delay( 200 );			/* be sure all frames sent */

	/* verify error object */
	CHK( mscan_queue_status( path, 0, &entries, NULL ) == 0 );
	CHK( entries == 1 );
	CHK( mscan_read_error( path, &errCode, &objNr ) == 0 );
	CHK( errCode == MSCAN_QOVERRUN );
	CHK( objNr == rxObj );
	CHK( G_sigUos1Cnt == 2 );
	CHK( strcmp( mscan_errobj_msg( errCode ), 
				 "object's receive fifo overflowed") == 0 );

	/* clear queue */
	CHK( mscan_queue_clear( path, rxObj, FALSE ) == 0 );
	CHK( mscan_queue_status( path, 0, &entries, NULL ) == 0 );
	CHK( entries == 0 );
	

	rv = 0;
 ABORT:
	mscan_clr_rcvsig( path, 0 );
	mscan_config_msg( path, txObj, MSCAN_DIR_DIS, 0, NULL );
	mscan_config_msg( path, rxObj, MSCAN_DIR_DIS, 0, NULL );
	UOS_SigRemove( UOS_SIG_USR1 );

	return rv;
}


static int CmpFrames( const MSCAN_FRAME *frm1, const MSCAN_FRAME *frm2 )
{
	int i;

	if( frm1->id != frm2->id )
		return -1;

	if( frm1->flags != frm2->flags )
		return -1;

	if( frm1->dataLen != frm2->dataLen )
		return -1;

	for( i=0; i<frm1->dataLen; i++ )
		if( frm1->data[i] != frm2->data[i] )
			return -1;

	return 0;
}

static void DumpFrame( char *msg, const MSCAN_FRAME *frm )
{
	int i;
	printf("%s: ID=0x%08lx%s%s data=", 
		   msg,
		   frm->id, 
		   (frm->flags & MSCAN_EXTENDED) ? "x":"", 
		   (frm->flags & MSCAN_RTR) ? "RTR":"");

	for(i=0; i<frm->dataLen; i++ ){
		printf("%02x ", frm->data[i] );
	}
	printf("\n");
}

