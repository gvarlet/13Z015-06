/*********************  P r o g r a m  -  M o d u l e ***********************/
/*!  
 *        \file  mscan_alyzer.c
 *
 *      \author  klaus.popp@men.de
 *        $Date: 2010/02/25 18:04:27 $
 *    $Revision: 1.9 $
 * 
 *  	 \brief  Test tool for MSCAN driver against CANalyzer
 *
 *     CANalyzer must run mscan_veri.can CAPL program
 *
 * CANalyzer test communication:
 * 
 * CANalyzer listens for frames on ID 07ffx. This frame has 3 bytes:
 * - Byte 0: Test number to execute:
 *			 0x00 = stop running test
 *			 0x01 = verify predefined sequence of frames sent from MSCAN
 *			 0x02 = send predefined sequence of frames (1ms rate)	
 *			 0x03 = send 5 frames (for Overrun test part1)
 *			 0x04 = send 4 frames (for Overrun test part2)
 *			 0x05 = test 0x01 and 0x02 together
 *
 * - Byte 1/2: additional parameters for test (not yet used)
 *
 * CANalyzer stops on error and outputs any error to its "write" window. 
 * CANalyzer does not respond to commands.
 *
 *     Switches: -
 *     Required: libraries: mdis_api, usr_oss, usr_utl, mscan_api
 */
/*-------------------------------[ History ]---------------------------------
 *
 * $Log: mscan_alyzer.c,v $
 * Revision 1.9  2010/02/25 18:04:27  amorbach
 * R: driver ported to MDIS5, new MDIS_API and men_typs
 * M1: Change type of path to MDIS_PATH
 * M2: Compiler warnings removed
 *
 * Revision 1.8  2007/04/10 17:21:43  SYao
 * Correct output message's format
 *
 * Revision 1.7  2006/02/22 12:04:32  ub
 * added options to set timing parameters
 *
 * Revision 1.6  2005/03/24 10:18:30  kp
 * hide error if mscan_error_counters() fails (not possible with MPC5200 CAN)
 *
 * Revision 1.5  2004/06/14 11:58:21  kp
 * cosmetics
 *
 * Revision 1.4  2004/03/19 11:02:04  ub
 * Fixed: Try to close non-open path
 *
 * Revision 1.3  2003/08/27 08:13:28  kp
 * changed for test with Schlafhorst Winding CAN repeater and PP01:
 * 1) don't use COB IDs 0x000..0x2ff
 * 2) filter out COB IDs sent by CAN repeater
 * 3) now Tx/Rx COB IDs are always different
 *
 * Revision 1.2  2003/03/18 12:40:13  kpftp
 * Slightly modified:
 * 1) Ignore frames sent by MGT5100EVB (0x40000x)
 * 2) Sent IDs start with an offset of 10 now
 * 3) Check & display node state
 *
 * Revision 1.1  2003/02/07 13:16:37  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2003 by MEN mikro elektronik GmbH, Nuernberg, Germany 
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


static const char RCSid[]="$Id: mscan_alyzer.c,v 1.9 2010/02/25 18:04:27 amorbach Exp $";

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

static int AlyzerTx( MDIS_PATH path );
static int AlyzerRx( MDIS_PATH path );
static int AlyzerRxTx( MDIS_PATH path );
static int AlyzerRxTxTest( MDIS_PATH path );
static void DumpFrame( char *msg, const MSCAN_FRAME *frm );
static int CmpFrames( const MSCAN_FRAME *frm1, const MSCAN_FRAME *frm2 );

/*--------------------------------------+
|   TYPEDEFS                            |
+--------------------------------------*/
/** CANalyzer communication commands */
typedef enum {
	AcStop=0,
	AcTx=1,
	AcRx=2,
	AcRxOver=3,
	AcRxOver2=4,
	AcRxTx=5
} AC_CMD;

/* test list description */
typedef struct {
	char code;
	char *descr;
	int (*func)(MDIS_PATH path);
} TEST_ELEM;

/*--------------------------------------+
|   GLOBALS                             |
+--------------------------------------*/
#if 0
/* filters to let everything pass through */
static const MSCAN_FILTER G_stdOpenFilter = { 
	0,
	0xffffffff,
	0,
	0 
};
#endif
static const MSCAN_FILTER G_extOpenFilter = { 
	0,
	0xff3fffff,					/* filter out frames sent by MGT5100EVB */
	MSCAN_EXTENDED,
	0 
};

static TEST_ELEM G_testList[] = {
	{ 'a', "Basic Tx", AlyzerTx },
	{ 'b', "Basic Rx", AlyzerRx },
	{ 'c', "Simultaneous Rx/Tx", AlyzerRxTx },
	{ 'd', "Test", AlyzerRxTxTest },
	{ 0, NULL, NULL }
};

static int G_endMe;
static const int G_txObj=1;
static const int G_rxObjStd=2;
static const int G_rxObjExt=3;


/**********************************************************************/
/** Print program usage
 */
static void usage(void)
{
	TEST_ELEM *te=G_testList;

	printf(
		"usage: mscan_alyzer [<opts>] <device> [<opts>]\n"
		"Options:\n"
		"  -b=<code>    bitrate code (0..8)              [0]\n"
		"                  0=1MBit 1=800kbit 2=500kbit 3=250kbit 4=125kbit\n"
		"                  5=100kbit 6=50kbit 7=20kbit 8=10kbit\n"
		"  -r=<brp>     contents of BRP register\n"
		"  -j=<sjw>     SJW register\n"
		"  -1=<tseg1>   TSEG1 register\n"
		"  -2=<tseg1>   TSEG2 register\n"
        "                  Timing parameters disable bitrate selection !\n"
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
	int32	ret=1, n, error;
    MDIS_PATH path=-1;
	int stopOnFirst, runs, run, errCount=0;
    int brp, sjw, tseg1, tseg2;
	u_int32 bitrate, spl=0;
	char	*device,*str,*errstr,buf[40],*testlist;
	TEST_ELEM *te;
	char *tCode;

	G_endMe = FALSE;
	/*--------------------+
    |  check arguments    |
    +--------------------*/
	if ((errstr = UTL_ILLIOPT("n=sb=t=r=j=1=2=?", buf))) {	/* check args */
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

    brp       = ((str = UTL_TSTOPT("r=")) ? atoi(str) : 0);
    sjw       = ((str = UTL_TSTOPT("j=")) ? atoi(str) : 1);
    tseg1     = ((str = UTL_TSTOPT("1=")) ? atoi(str) : 5);
    tseg2     = ((str = UTL_TSTOPT("2=")) ? atoi(str) : 2);

	/*--------------------+
    |  open path          |
    +--------------------*/
	CHK( (path = mscan_init(device)) >= 0 );

	CHK( M_setstat( path, MSCAN_MAXIRQTIME, 0 ) == 0 );
	/*--------------------+
    |  config             |
    +--------------------*/
    if( brp == 0 ) {
        CHK( mscan_set_bitrate( path, (MSCAN_BITRATE)bitrate, spl ) == 0 );  
    }
    else {
        printf("Bustiming: BRP=%d SJW=%d TSEG1=%d TSEG2=%d\n",brp, sjw, tseg1,tseg2);
        CHK( mscan_set_bustiming( path, (u_int8)brp, (u_int8)sjw, (u_int8)tseg1, (u_int8)tseg2, 0 ) == 0 );
    }

	/*--- config error object ---*/
	CHK( mscan_config_msg( path, 0, MSCAN_DIR_RCV, 10, NULL ) == 0 );

	/*--- config Tx object ---*/
	CHK( mscan_config_msg( path, G_txObj, MSCAN_DIR_XMT, 100, NULL ) == 0 );

	/*--- config Rx objects ---*/
#if 0
	CHK( mscan_config_msg( path, G_rxObjStd, MSCAN_DIR_RCV, 500, 
						   &G_stdOpenFilter ) == 0 );
#endif
    {
		MSCAN_FILTER flt;
		int i;

		flt.code = 0x000;
		flt.mask = 0xfff;
		flt.cflags = 0;
		flt.mflags = MSCAN_USE_ACCFIELD;
		
		memset( &flt.accField, 0, sizeof( flt.accField ));

		for( i=0; i<0x7ff; i++ ){
			/* filter out frames generated by Sh Winding CAN repeater */
			if( i!=0x36 && i!=0x5e && i!=0x86 ){
				MSCAN_ACCFIELD_SET( flt.accField, i );
			}
		}

		CHK( mscan_config_msg( path, G_rxObjStd, MSCAN_DIR_RCV, 500, 
							   &flt ) == 0 );
	}
	CHK( mscan_config_msg( path, G_rxObjExt, MSCAN_DIR_RCV, 500, 
						   &G_extOpenFilter ) == 0 );

	/*--- make sure CAN not in loopback mode ---*/
	CHK( mscan_set_loopback( path, FALSE ) == 0 );

	/*--- enable bus ---*/
	CHK( mscan_enable( path, TRUE ) == 0 );

	/*-------------------+
	|  Perform tests     |
	+-------------------*/
	testlist  = ((str = UTL_TSTOPT("t=")) ? 
				 str : "abc");

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
#if 0
	{
		u_int32 maxIrqTime;

		CHK( M_getstat( path, MSCAN_MAXIRQTIME, (int32*)&maxIrqTime ) == 0 );
		printf("Max irqtime=%d (internal ticks)\n", maxIrqTime );
	}
#endif
	ret = 0;
	UOS_Delay( 100 );
	CHK( mscan_enable( path, FALSE ) == 0 );
	CHK( mscan_term(path) == 0 );
	path=-1;

 ABORT:
	
	if( path != -1 ) {
        mscan_enable( path, FALSE );
		mscan_term(path);
    }

	return(ret);
}

/**********************************************************************/
/** Send command to CANalyzer CAPL script
 * 
 * \return 0=ok, -1=error
 */
static int AlyzerCmd( MDIS_PATH path, u_int8 cmd, u_int8 p1, u_int8 p2 )
{
	MSCAN_FRAME frm;

	frm.id = 0x7ff;
	frm.flags = MSCAN_EXTENDED;
	frm.data[0] = cmd;
	frm.data[1] = p1;
	frm.data[2] = p2;
	frm.dataLen = 3;

	CHK( mscan_write_msg( path, G_txObj, 1000, &frm ) == 0 );
	return 0;
 ABORT:
	return -1;
}

/**********************************************************************/
/** Query teststatus from CANalyzer CAPL script
 * 
 * \return 0=ok, 0xff=test still running, >0 error code
 */
static int AlyzerStatus( MDIS_PATH path )
{
	MSCAN_FRAME frm;

	frm.id = 0x7ff;
	frm.flags = MSCAN_EXTENDED|MSCAN_RTR;
	frm.dataLen = 3;

	CHK( mscan_write_msg( path, G_txObj, 1000, &frm ) == 0 );
	CHK( mscan_read_msg( path, G_rxObjExt, 1000, &frm ) == 0 );
	
	CHK( frm.id == 0x7fe );
	CHK( frm.dataLen == 3 );
	return frm.data[1];
		
 ABORT:
	return -1;
}

/**********************************************************************/
/** Build frames for Rx/Tx tests
 * 
 */
static void AlyzerMkFrame( MSCAN_FRAME *frm, int state, int id )
{
	int i;
	u_int8 d;

	/* 
	 * states: 
	 * 0: std, no rtr	ID 0..0x7ff
	 * 1: std, rtr		ID 0..0x7ff
	 * 2: ext, no rtr	ID 0x10000..0x100ff
	 * 3: ext, rtr		ID 0x1000000..0x100000ff
	 *
	 * DLC: 	id % 9 (0 for RTR frames)
	 * data[0]:	id&0xff
	 * data[1]:	(id&0xff)+1
	 * ...
	 */
	frm->id = id;
	frm->flags = ((state & 0x1) ? MSCAN_RTR : 0) | 
		((state & 0x2) ? MSCAN_EXTENDED : 0 );
	frm->dataLen = id % 9;

#if 0
	if( frm->flags & MSCAN_RTR )
		frm->dataLen = 0;		/* ??? */
#endif
	d = id & 0xff;

	for( i=0; i<frm->dataLen; i++ )
		frm->data[i] = d++;
	for( ; i<8; i++ )
		frm->data[i] = 0;
}

/**********************************************************************/
/** Build next ID/state for frames send by MSCAN
 * state:
 * 0: std, no rtr	ID 0x090..0x3ff
 * 1: std, rtr		ID 0x090..0x3ff
 * 2: ext, no rtr	ID 0x10000..0x100ff
 * 3: ext, rtr		ID 0x1000000..0x100000ff
 */
static void NxtTxId( int *stateP, u_int32 *idP )
{
	u_int32 id = *idP;
	int state = *stateP;

	id++;
	switch( state ){
	case -1:
		state = 0;
		id = 0x300;
		break;
	case 0: 
		if( id==0x400 ){
			state++;
			id=0x300;
		}
		break;
	case 1: 
		if( id==0x400 ){
			state++;
			id=0x10000;
		}
		break;
	case 2: 
		if( id==0x10100 ){
			state++;
			id=0x10000000;
		}
		break;
	case 3: 
		if( id==0x10000100 ){
			state++;
		}
		break;
	}
	*stateP = state;
	*idP = id;
}

/**********************************************************************/
/** Build next ID/state for frames sent by CANalyzer
 * state:
 * 0: std, no rtr	ID 0x400..0x7ff
 * 1: std, rtr		ID 0x400..0x7ff
 * 2: ext, no rtr	ID 0x10100..0x101ff
 * 3: ext, rtr		ID 0x1000100..0x100001ff
 */
static void NxtRxId( int *stateP, u_int32 *idP )
{
	u_int32 id = *idP;
	int state = *stateP;

	id++;
	switch( state ){
	case -1:
		state = 0;
		id = 0x400;
		break;
	case 0: 
		if( id==0x7ff ){
			state++;
			id=0x400;
		}
		break;
	case 1: 
		if( id==0x7ff ){
			state++;
			id=0x10100;
		}
		break;
	case 2: 
		if( id==0x10200 ){
			state++;
			id=0x10000100;
		}
		break;
	case 3: 
		if( id==0x10000200 ){
			state++;
		}
		break;
	}
	*stateP = state;
	*idP = id;
}

static void ChkNodeState( MDIS_PATH path )
{
	static MSCAN_NODE_STATUS oldStatus = MSCAN_NS_ERROR_ACTIVE;
	static u_int8 oldTxErr=0, oldRxErr=0;
	MSCAN_NODE_STATUS status;
	u_int8 txErr, rxErr, haveErrCounters=FALSE;

	CHK( mscan_node_status( path, &status ) == 0 );
	if( mscan_error_counters( path, &txErr, &rxErr ) == 0 )
		haveErrCounters = TRUE;

	if( status != oldStatus ){
		printf("*** Node status changed %s\n",
			   status == MSCAN_NS_ERROR_ACTIVE ? "Error active" : 
			   ( status == MSCAN_NS_ERROR_PASSIVE ? "Error passive" : 
				 "Bus off"));
		oldStatus = status;
	}

	if( haveErrCounters && ( (rxErr != oldRxErr) || (txErr != oldTxErr)) ){
		printf("*** MSCAN chip: txErr=%d rxErr=%d\n", txErr, rxErr );
		oldRxErr = rxErr;
		oldTxErr = txErr;
	}
	
 ABORT:
	return;
}

/**********************************************************************/
/** Test a: Basic Tx
 * 
 * \return 0=ok, -1=error
 */
static int AlyzerTx( MDIS_PATH path )
{
	int rv = -1;
	u_int32 id;
	int state=-1;
	int timeout=2000;
	MSCAN_FRAME txFrm;
	
	CHK( AlyzerCmd( path, AcTx, 0, 0) == 0 );
	
	NxtTxId( &state, &id );

	while( state < 4 ){
		
		AlyzerMkFrame( &txFrm, state, id );
		/*DumpFrame( "Tx  ", &txFrm );*/
		ChkNodeState( path );
		CHK( mscan_write_msg( path, G_txObj, timeout, &txFrm ) == 0 );

		NxtTxId( &state, &id );
	}

	ChkNodeState( path );
	CHK( AlyzerStatus( path ) == 0 );
    rv = 0;

 ABORT:
	AlyzerCmd( path, AcStop, 0, 0);

	return rv;
}

/**********************************************************************/
/** Test b: Basic Rx
 * 
 * \return 0=ok, -1=error
 */
static int AlyzerRx( MDIS_PATH path )
{
	int rv = -1;
	u_int32 id;
	int timeout=2000;
	int state=-1;
	MSCAN_FRAME rxFrm, sbFrm;

	CHK( AlyzerCmd( path, AcRx, 0, 0) == 0 );

	NxtRxId( &state, &id );

	while( state < 4 ){
		
		int newState = state;

		ChkNodeState( path );

		AlyzerMkFrame( &sbFrm, state, id );

		/* wait until received */
		CHK( mscan_read_msg( path, state < 2 ? G_rxObjStd : G_rxObjExt, 
							 timeout, &rxFrm ) == 0 );

		if( CmpFrames( &rxFrm, &sbFrm ) != 0 ){
			DumpFrame( "Sb  ", &sbFrm );
			DumpFrame( "Recv", &rxFrm );
			CHK(0);
		}

		NxtRxId( &newState, &id );

		if( newState != state ){
			state = newState;
			printf("   Now in state %d\n", state );
		}
	}

    rv = 0;

 ABORT:
	AlyzerCmd( path, AcStop, 0, 0);

	return rv;
}

/**********************************************************************/
/** Test c: Tx and Rx together
 * 
 * \return 0=ok, -1=error
 */
static int AlyzerRxTx( MDIS_PATH path )
{
	int rv = -1, i;
	u_int32 txId, rxId;
	int txState=-1, rxState=-1;
	MSCAN_FRAME rxFrm, sbFrm, txFrm;

	CHK( AlyzerCmd( path, AcRxTx, 0, 0) == 0 );

	NxtRxId( &rxState, &rxId );
	NxtTxId( &txState, &txId );

	while( (txState < 4) || (rxState < 4) ){

		ChkNodeState( path );

		if( (rxState < 4) && 
			(mscan_read_msg( path, rxState < 2 ? G_rxObjStd : G_rxObjExt, 
							 50, &rxFrm ) == 0) ){
			/* received something */
			int newRxState = rxState;

			AlyzerMkFrame( &sbFrm, rxState, rxId );
			
			if( CmpFrames( &rxFrm, &sbFrm ) != 0 ){
				DumpFrame( "Sb  ", &sbFrm );
				DumpFrame( "Recv", &rxFrm );
				CHK(0);
			}

			NxtRxId( &newRxState, &rxId );

			if( newRxState != rxState ){
				rxState = newRxState;
				printf("   Rx now in state %d\n", rxState );
			}

		}


		/* send burst */
		for( i=0; i<2 && txState<4; i++ ){
			int newTxState = txState;

			AlyzerMkFrame( &txFrm, txState, txId );
			if( mscan_write_msg( path, G_txObj, -1, &txFrm ) == 0 ){

				NxtTxId( &newTxState, &txId );

				if( newTxState != txState ){
					txState = newTxState;
					printf("   Tx now in state %d\n", txState );
				}
			}
		}

	}

	CHK( AlyzerStatus( path ) == 0 );

    rv = 0;

 ABORT:
	AlyzerCmd( path, AcStop, 0, 0);

	return rv;
}

/**********************************************************************/
/** Test d: TEST
 * 
 * \return 0=ok, -1=error
 */
static int AlyzerRxTxTest( MDIS_PATH path )
{
	int rv = -1, i;
	u_int32 txId, rxId;
	int txState=0, rxState=0;
	MSCAN_FRAME rxFrm, sbFrm, txFrm;

	CHK( AlyzerCmd( path, AcTx, 0, 0) == 0 );
	NxtRxId( &rxState, &rxId );
	NxtTxId( &txState, &txId );

	while( (txState < 4) || (rxState < 4) ){

		ChkNodeState( path );

		if( (rxState < 4) && 
			(mscan_read_msg( path, rxState < 2 ? G_rxObjStd : G_rxObjExt, 
							 50, &rxFrm ) == 0) ){
			/* received something */
			int newRxState = rxState;

			AlyzerMkFrame( &sbFrm, rxState, rxId );
			
			if( CmpFrames( &rxFrm, &sbFrm ) != 0 ){
				DumpFrame( "Sb  ", &sbFrm );
				DumpFrame( "Recv", &rxFrm );
				CHK(0);
			}

			NxtRxId( &newRxState, &rxId );

			if( newRxState != rxState ){
				rxState = newRxState;
				printf("   Rx now in state %d\n", rxState );
			}

		}


		/* send burst */
		for( i=0; i<2 && txState<4; i++ ){
			int newTxState = txState;

			AlyzerMkFrame( &txFrm, txState, txId );
			if( mscan_write_msg( path, G_txObj, -1, &txFrm ) == 0 ){

				NxtTxId( &newTxState, &txId );

				if( newTxState != txState ){
					txState = newTxState;
					printf("   Tx now in state %d\n", txState );
				}
			}
		}

	}

	CHK( AlyzerStatus( path ) == 0 );

    rv = 0;

 ABORT:
	AlyzerCmd( path, AcStop, 0, 0);

	return rv;
}

static int CmpFrames( const MSCAN_FRAME *frm1, const MSCAN_FRAME *frm2 )
{
	int i;

	if( frm1->id != frm2->id )
		return -1;

	if( frm1->flags != frm2->flags )
		return -1;

	if( !(frm1->flags & MSCAN_RTR)){
		/* Note: CANalyzer sends remote frames always with DLC=0 */
		if( frm1->dataLen != frm2->dataLen )
			return -1;

		for( i=0; i<frm1->dataLen; i++ )
			if( frm1->data[i] != frm2->data[i] )
				return -1;
	}

	return 0;
}

static void DumpFrame( char *msg, const MSCAN_FRAME *frm )
{
	int i;
	printf("%s: ID=0x%08lx%s", 
		   msg,
		   frm->id, 
		   (frm->flags & MSCAN_EXTENDED) ? "x":"");

	if( frm->flags & MSCAN_RTR )
		printf(" RTR dataLen=%d", frm->dataLen );
	else {
		printf(" data=");

		for(i=0; i<frm->dataLen; i++ ){
			printf("%02x ", frm->data[i] );
		}
	}
	printf("\n");
}

