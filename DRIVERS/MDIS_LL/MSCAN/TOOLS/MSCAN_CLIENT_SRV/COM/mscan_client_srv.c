/*********************  P r o g r a m  -  M o d u l e ***********************/
/*!
 *        \file  mscan_client_srv.c
 *
 *      \author  michael.roth@men.de
 *        $Date: 2013/11/27 15:35:16 $
 *    $Revision: 1.4 $
 *
 *  	 \brief  Test tool for MSCAN controller interface
 *
 *               Tests MSCAN controller interfaces (e.g. SA08) by transmitting
 *               and receiving CAN frames between the client (DUT) and the
 *               server system.
 *
 *     Switches: -
 *     Required: libraries: mdis_api, usr_oss, usr_utl, mscan_api
 */
/*-------------------------------[ History ]---------------------------------
 *
 * $Log: mscan_client_srv.c,v $
 * Revision 1.4  2013/11/27 15:35:16  MRoth
 * R: 1) recurring test errors, test frame objects were not reset correctly
 *    2) Signal handler not used
 *    3) cosmetics
 * M: 1) fixed loop in LoopbBasic function
 *       to reset the objects on every test cycle
 *    2) removed all Signal handler components
 *    3) reworked the usage and user outputs
 *
 * Revision 1.3  2013/09/19 10:54:53  MRoth
 * R: 1) error counter not working
 *    2) option missing to delay the mscan write/read commands
 * M: 1) implemented new error handling
 *    2) added timeout parameter
 *
 * Revision 1.2  2010/05/19 14:57:32  MRoth
 * R: 1) no inversion of frames (server)
 *    2) no error message if physical interface (e.g. SA8) is missing (client)
 * M: 1) implemented InvertFrame() function
 *    2) implemented checking of FPGA global error fifo
 *
 * Revision 1.1  2010/01/26 18:19:40  MRoth
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2010 by MEN mikro elektronik GmbH, Nuernberg, Germany
 ****************************************************************************/
static const char RCSid[]="$Id: mscan_client_srv.c,v 1.4 2013/11/27 15:35:16 MRoth Exp $";

/*--------------------------------------+
|   INCLUDES                            |
+--------------------------------------*/
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
	ret=1;\
	goto ABORT;\
  }

#define SERVER		1
#define CLIENT		0

/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
static void usage( void );
static int  LoopbBasic( MDIS_PATH path1, MDIS_PATH path2, int32 timeout, int32 type, int read );
static void DumpFrame ( char *msg, const MSCAN_FRAME *frm );
static int  CmpFrames ( const MSCAN_FRAME *frm1, const MSCAN_FRAME *frm2 );
static void InvertFrame( MSCAN_FRAME *frm );

/*--------------------------------------+
|   TYPEDEFS                            |
+--------------------------------------*/
/* (none) */

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

/********************************** usage **********************************/
/** Print program usage
 */
static void usage( void )
{
	printf
	(
		"\nUsage: mscan_client_srv dev1 [dev2] [<opts>]             \n"
		"Function: Tool to test single CAN interfaces               \n"
		"\nOptions:                                                 \n"
		"  dev1         device name (e.g. boromir_1)                \n"
		"  dev2         only for XC10 tests!                        \n"
		"  -b=<code>    bitrate code (0..6)..................[0]    \n"
		"                 0=1MBit 1=800kbit 2=500kbit 3=250kbit     \n"
		"                 4=125kbit 5=100kbit 6=50kbit              \n"
		"  -t=<type>    client(0) or server(1) mode..........[0]    \n"
		"  -o=<timeout> max. allowed time difference between        \n"
		"                 sent and received frames (in ms)...[1000] \n"
		"  -n=<runs>    test cycles..........................[0]    \n"
		"                 0 means endless until keypress            \n"
		"  -s           stop on first error .................[no]   \n"
		"  -r           just listen..........................[no]   \n"
		"                 - only for XC10 client tests!             \n"
        "\nCalling examples:\n"
        "\n - run application in server mode (1MBit, endless): \n"
        "     mscan_client_srv boromir_1 -t=1 \n"
        "\n - run application in client mode (1MBit, endless): \n"
        "     mscan_client_srv boromir_1 \n"
        "\n - run application in client mode (500kBit, 50 cycles): \n"
        "     mscan_client_srv boromir_1 -b=2 -n=50 \n"
        "\n(c)Copyright 2013 by MEN Mikro Elektronik GmbH\n%s\n\n", RCSid
	);
}

/***************************************************************************/
/** Program main function
 *
 *  \param argc       \IN  argument counter
 *  \param argv       \IN  argument vector
 *
 *  \return	          success (0) or error (1)
 */
int main( int argc, char *argv[] )
{
	MDIS_PATH path1=(-1), path2=(-1);
	int32 ret=0, n, error;
	int stopOnFirst, read, type, runs, run=0, errCount=0;
	u_int32 bitrate, timeout, spl=0;
	char *device1=NULL, *device2=NULL;
	char *str,*errstr,buf[40];

	/*--------------------+
	|  check arguments    |
	+--------------------*/
	if( (errstr = UTL_ILLIOPT("srb=o=t=n=?", buf)) ) {	/* check args */
		printf("*** %s\n", errstr);
		return(1);
	}

	if( UTL_TSTOPT("?") ) {						/* help requested ? */
		usage();
		return(1);
	}

	/*--------------------+
	|  get arguments      |
	+--------------------*/
	for( n=1; n<argc; n++ ) {
		if( *argv[n] != '-' ) {
			if( device1 == NULL ) {
				device1 = argv[n];
			}
			else if( device2 == NULL ) {
				device2 = argv[n];
			}
		}
	}
	if( !device1 ) {
		usage();
		return(1);
	}

	bitrate = ( (str = UTL_TSTOPT("b=")) ? atoi(str) : 0 );
	runs    = ( (str = UTL_TSTOPT("n=")) ? atoi(str) : 0 );
	type    = ( (str = UTL_TSTOPT("t=")) ? atoi(str) : CLIENT );
	timeout = ( (str = UTL_TSTOPT("o=")) ? atoi(str) : 1000);
	stopOnFirst = !!UTL_TSTOPT("s");
	read = !!UTL_TSTOPT("r");

	/*--------------------+
	|  open path          |
	+--------------------*/
	CHK( (path1 = mscan_init(device1)) >= 0 );

	CHK( M_setstat( path1, MSCAN_MAXIRQTIME, 0 ) == 0 );

	/*--------------------+
	|  config             |
	+--------------------*/
	CHK( mscan_set_bitrate( path1, (MSCAN_BITRATE)bitrate, spl ) == 0 );
	/*--- config error object ---*/
	CHK( mscan_config_msg( path1, 0, MSCAN_DIR_RCV, 10, NULL ) == 0 );
	/*--- enable bus ---*/
	CHK( mscan_enable( path1, TRUE ) == 0 );

	if( device2 ) {
		CHK( (path2 = mscan_init(device2)) >= 0 );
		CHK( M_setstat( path2, MSCAN_MAXIRQTIME, 0 ) == 0 );
		CHK( mscan_set_bitrate( path2, (MSCAN_BITRATE)bitrate, spl ) == 0 );
		CHK( mscan_config_msg( path2, 0, MSCAN_DIR_RCV, 10, NULL ) == 0 );
		CHK( mscan_enable( path2, TRUE ) == 0 );
	}

	if( type == SERVER )
		printf( "Performing CAN test as server...\n\n" );
	else
		printf( "Performing CAN test as client...\n\n" );

	do {
		error = LoopbBasic( path1, path2, timeout, type, read );
		if( error ) {
			errCount++;
		}

		if( runs == 0 )
			run = -1;
		else
			run++;

		if( error && stopOnFirst )
			break;

		/* little break after one run */
		if( type == CLIENT )
			UOS_Delay(500);

	}while( (run < runs) && (UOS_KeyPressed() == -1) );

	if( type == CLIENT ){
		printf( "\n------------------------------------------------\n" );
		printf( "TEST RESULT: %d errors\n", errCount );
	}

	if (errCount > 0)
		ret = 1;

	CHK( mscan_enable( path1, FALSE ) == 0 );
	CHK( mscan_term(path1) == 0 );
	path1=-1;

	if( path2 != (-1) ) {
		CHK( mscan_enable( path2, FALSE ) == 0 );
		CHK( mscan_term(path2) == 0 );
		path2=-1;
	}

ABORT:
	if( path1 != -1 ){
		mscan_enable( path1, FALSE );
		mscan_term(path1);
	}

	if( path2 != -1 ){
		mscan_enable( path2, FALSE );
		mscan_term(path2);
	}

	return(ret);
}


/******************************** LoopbBasic *******************************/
/** Basic Tx/Rx test
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
 * \return 0=ok, 1=error
 */
static int LoopbBasic( MDIS_PATH path1, MDIS_PATH path2, int32 timeout, int32 type, int read )
{
	int i=0, rxObjrd1, rxObjrd2, ret=0;
	u_int32 errCode, nr, entries;
	const int txObj  = 5;
	const int rxObj1 = 1;
	const int rxObj2 = 2;
	const int rxObj3 = 1;
	const int rxObj4 = 2;

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
	MSCAN_FRAME txFrmTmp;


	/* config tx/rx objects */
	/* Tx object */
	CHK( mscan_config_msg( path1, txObj, MSCAN_DIR_XMT, 10, NULL ) == 0 );

	/* Rx object for standard messages */
	CHK( mscan_config_msg( path1, rxObj1, MSCAN_DIR_RCV, 20,
							&G_stdOpenFilter ) == 0 );
	/* Rx object for extended messages */
	CHK( mscan_config_msg( path1, rxObj2, MSCAN_DIR_RCV, 20,
							&G_extOpenFilter ) == 0 );
	if( path2 != (-1) ) {
		/* Rx object for standard messages */
		CHK( mscan_config_msg( path2, rxObj3, MSCAN_DIR_RCV, 20,
							&G_stdOpenFilter ) == 0 );
		/* Rx object for extended messages */
		CHK( mscan_config_msg( path2, rxObj4, MSCAN_DIR_RCV, 20,
							&G_extOpenFilter ) == 0 );
	}

	for( i=0; i<sizeof(txFrm)/sizeof(MSCAN_FRAME); i++ ) {

		if( type == CLIENT ) {

			if( !read ) {
				/* send one frame */
				CHK( mscan_write_msg( path1, txObj, timeout, &txFrm[i] ) == 0 );
				DumpFrame( "Sent", &txFrm[i] );

				/* read errors from global error fifo */
				CHK( mscan_queue_status( path1, 0, &entries, NULL ) == 0 );
				if( entries > 0 ) {
					CHK( mscan_read_error( path1, &errCode, &nr ) == 0 );
					printf( "Error code %ld (%s), obj Nr %ld\n",
						errCode, mscan_errobj_msg(errCode), nr );
					ret = 1;
					break;
				}
			}

			/* wait for frame on correct object */
			rxObjrd1 = txFrm[i].flags & MSCAN_EXTENDED ? rxObj2 : rxObj1;
			CHK( mscan_read_msg( path1, rxObjrd1, timeout, &rxFrm ) == 0 );

			DumpFrame( "Recv(inverted)", &rxFrm );

			txFrmTmp = txFrm[i];
			InvertFrame( &txFrmTmp );

			/* check if received correctly */
			if( CmpFrames( &rxFrm, &txFrmTmp ) != 0 ) {
				printf("FAIL -> Frame mismatch!\n\n");
				CHK(0);
			}
			else
				printf("OK\n\n");

			/* XC10 CAN port (read only! No verify of frame receive possible!)*/
			if( path2 != (-1) ) {
				rxObjrd2 = txFrm[i].flags & MSCAN_EXTENDED ? rxObj3 : rxObj4;
				CHK( mscan_read_msg( path2, rxObjrd2, timeout, &rxFrm ) == 0 );

				DumpFrame( "Recv(inverted)", &rxFrm );

				txFrmTmp = txFrm[i];
				InvertFrame( &txFrmTmp );

				/* check if received correctly */
				if( CmpFrames( &rxFrm, &txFrmTmp ) != 0 ) {
					printf("FAIL -> Frame mismatch!\n\n");
					CHK(0);
				}
				else
					printf("OK\n\n");
			}
		} /* if CLIENT */

		else {  /* type == SERVER */

			/* wait for frame on correct object */
			rxObjrd1 = txFrm[i].flags & MSCAN_EXTENDED ? rxObj2 : rxObj1;

			/* wait max. 10 seconds */
			if( mscan_read_msg( path1, rxObjrd1, 10000, &rxFrm ) == 0 ) {
				DumpFrame( "Recv", &rxFrm );

				UOS_Delay(100);

				InvertFrame( &rxFrm );

				/* send it back */
				CHK( mscan_write_msg( path1, txObj, timeout, &rxFrm ) == 0 );
				DumpFrame( "Sent(inverted)", &rxFrm );
				printf("\n");
			}
			else {
				printf("*** Timeout after 10 seconds - frame restored.\n");
				break;
			}
		} /* SERVER */
	}

 ABORT:
 	/* reset objects */
	mscan_config_msg( path1, txObj, MSCAN_DIR_DIS, 0, NULL );
	mscan_config_msg( path1, rxObj1, MSCAN_DIR_DIS, 0, NULL );
	mscan_config_msg( path1, rxObj2, MSCAN_DIR_DIS, 0, NULL );

	if( path2 != (-1) ) {
		mscan_config_msg( path2, rxObj3, MSCAN_DIR_DIS, 0, NULL );
		mscan_config_msg( path2, rxObj4, MSCAN_DIR_DIS, 0, NULL );
	}

	return ret;
}

/********************************* CmpFrames *******************************/
/** Routine to compare the attributes of two CAN frames.
 *
 *  \param frm1       \IN  Frame one
 *  \param frm2       \IN  Frame two
 *
 *  \return 0=ok, -1=error
 */
static int CmpFrames( const MSCAN_FRAME *frm1, const MSCAN_FRAME *frm2 )
{
	int i;

	if( frm1->id != frm2->id )
		return -1;

	if( frm1->flags != frm2->flags )
		return -1;

	if( frm1->dataLen != frm2->dataLen )
		return -1;

	for( i=0; i<frm1->dataLen; i++ ){
		if( frm1->data[i] != frm2->data[i] )
			return -1;
	}

	return 0;
}

/********************************* InvertFrames *******************************/
/** Routine to invert the CAN data frames.
 *
 *  \param frm       \IN  Frame
 *
 */
static void InvertFrame( MSCAN_FRAME *frm )
{
	int i;

	for( i=0; i<frm->dataLen; i++ ) {
		 frm->data[i] = ~(frm->data[i]);
	}
}

/********************************* DumpFrame *******************************/
/** Routine to dump the content of a given CAN frame.
 *
 *  \param msg       \IN  message string
 *  \param frm       \IN  CAN frame
 */
static void DumpFrame( char *msg, const MSCAN_FRAME *frm )
{
	int i;
	printf( "%s: ID=0x%08lx%s data=",
		    msg,
		    frm->id,
		   (frm->flags & MSCAN_EXTENDED) ? "x":"");

	for( i=0; i<frm->dataLen; i++ ){
		printf( "%02x ", frm->data[i] );
	}
	printf( "\n" );
}

