/*********************  P r o g r a m  -  M o d u l e ***********************/
/*!  
 *        \file  mscan_menu.c
 *
 *      \author  klaus.popp@men.de
 *        $Date: 2010/02/25 18:04:38 $
 *    $Revision: 1.5 $
 * 
 *  	 \brief  Menu driven test tool for MSCAN driver
 *
 *     Switches: -
 *     Required: libraries: mdis_api, usr_oss, usr_utl, mscan_api
 */
/*-------------------------------[ History ]---------------------------------
 *
 * $Log: mscan_menu.c,v $
 * Revision 1.5  2010/02/25 18:04:38  amorbach
 * R: driver ported to MDIS5, new MDIS_API and men_typs
 * M1: Change type of G_path to MDIS_PATH
 * M2: Compiler warnings removed
 *
 * Revision 1.4  2006/07/21 09:18:18  ufranke
 * cosmetics
 *
 * Revision 1.3  2004/06/14 11:58:13  kp
 * cosmetics
 *
 * Revision 1.2  2003/03/18 12:40:56  kpftp
 * fixed read of error counters
 *
 * Revision 1.1  2003/01/29 14:03:08  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2003 by MEN mikro elektronik GmbH, Nuernberg, Germany 
 ****************************************************************************/
 
static const char RCSid[]="$Id: mscan_menu.c,v 1.5 2010/02/25 18:04:38 amorbach Exp $";

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <MEN/men_typs.h>
#include <MEN/usr_oss.h>
#include <MEN/usr_utl.h>
#include <MEN/mdis_api.h>
#include <MEN/mdis_err.h>
#include <MEN/usr_err.h>
#include <MEN/mscan_api.h>

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
|   TYPDEFS                             |
+--------------------------------------*/

/*--------------------------------------+
|   GLOBALS                             |
+--------------------------------------*/
static MDIS_PATH 	G_path;					/* path to device */


/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
static void usage(void);
static char GetChoice( char *str, char def, char *choices );
static void GetInteger( char *prompt, int32 *val );
static void GetHex( char *prompt, u_int32 *val );
static char *GetLine(void);
static void DumpFrame( char *msg, const MSCAN_FRAME *frm );

/**********************************************************************/
/** Print program usage
 */
static void usage(void)
{
	printf(	"usage: mscan_menu <device>\n");

	printf("(c) 2003 by MEN Mikro Elektronik GmbH\n%s\n", RCSid );
}


static void SetBitRate(void)
{
	static int32 bitrateCode=3;
	static char c='F';

	printf(" Bitrate codes: 0=1MBit 1=800kbit 2=500kbit 3=250kbit 4=125kbit\n"
		   "                5=100kbit 6=50kbit 7=20kbit 8=10kbit\n");
	GetInteger("Bitrate code", &bitrateCode );
	
	c = GetChoice( "Sampling mode: (S)low (F)ast", c, "SF" );
	CHK( mscan_set_bitrate( G_path, (MSCAN_BITRATE)bitrateCode, 
							c=='F'? 0:1 ) == 0 );
 ABORT:
	return;
}

#define NYI printf("not yet implemented\n")

static void SetBusTiming(void)
{
	static int32 brp, sjw, tseg1, tseg2, spl;
	
	GetInteger("brp   ", &brp );
	GetInteger("sjw   ", &sjw );
	GetInteger("tseg1 ", &tseg1 );
	GetInteger("tseg2 ", &tseg2 );
	GetInteger("spl   ", &spl );

	CHK( mscan_set_bustiming( G_path, (u_int8)brp, (u_int8)sjw, (u_int8)tseg1, (u_int8)tseg2, (u_int8)spl ) == 0 );
 ABORT:
	return;	
}

static void SetGlobalFilter(void)
{
	int i;

	static MSCAN_FILTER flt[] = { 
		{ 0,
		  0xffffffff,
		  0,
		  0 
		},
		{ 
			0,
			0xffffffff,
			MSCAN_EXTENDED,
			0 
		}
	};

	for( i=0; i<2; i++ ){
		printf("Filter %d settings:\n", i );
		GetHex(" Code", &flt[i].code );
		GetHex(" Mask", &flt[i].mask );
		GetHex(" Cflags", &flt[i].cflags );
		GetHex(" Mflags", &flt[i].mflags );
	}

	CHK( mscan_set_filter( G_path, &flt[0], &flt[1] ) == 0 );
 ABORT:
	return;	
}

static void ConfigMsgObj(void)
{
	static int32 objNr=1, qEntries=10;
	static char dir = 'R';
	static MSCAN_FILTER flt = { 
		0,
		0xffffffff,
		0,
		0 
	};
	MSCAN_DIR d=MSCAN_DIR_DIS;

	GetInteger( "Mesage object number", &objNr );
	dir = GetChoice( "direction: (D)isable (R)x (T)x", dir, "DRT" );
	GetInteger( "FIFO entries", &qEntries );
	if( dir == 'R' ){
		GetHex(" Filter code", &flt.code );
		GetHex(" Filter mask", &flt.mask );
		GetHex(" Filter cflags", &flt.cflags );
		GetHex(" Filter mflags", &flt.mflags );
	}
	switch(dir){
	case 'R': d=MSCAN_DIR_RCV; break;
	case 'T': d=MSCAN_DIR_XMT; break;
	case 'D': d=MSCAN_DIR_DIS; break;
	}

	CHK( mscan_config_msg( G_path, objNr, d, qEntries, &flt ) == 0 );
 ABORT:
	return;		
}

static void SetLoopback(void)
{
	static char c='D';

	c = GetChoice( "Loopback Mode: (E)nable (D)isable", c, "ED" );
	CHK( mscan_set_loopback( G_path, c=='E'? 1:0 ) == 0 );
 ABORT:
	return;	
}

static void EnableCan(void)
{
	static char c='D';

	c = GetChoice( "Enable CAN: (E)nable (D)isable", c, "ED" );
	CHK( mscan_enable( G_path, c=='E'? 1:0 ) == 0 );
 ABORT:
	return;	
}

static void ReadErrorCounters(void)
{
	u_int8 rxErrCnt, txErrCnt;

	CHK( mscan_error_counters( G_path, &txErrCnt, &rxErrCnt ) == 0 );
	printf("CAN rx error count: %d, tx error count: %d\n",
		   rxErrCnt, txErrCnt );
 ABORT:
	return;
}

/**********************************************************************/
/** Configuration submenu
 * 
 * \return 0=ok, -1=error
 */
static void Configure(void)
{
	char selection=0;

	while( selection != 'q' ){
		printf("\n\n==== MSCAN configuration menu ====\n");
		printf("b - set bitrate\n");
		printf("t - set bustiming directly\n");
		printf("f - set global filter\n");
		printf("c - configure message object\n");
		printf("l - set loopback mode\n");
		printf("e - enable/disable CAN\n");
		printf("q - back to main menu\n");

		printf("CONFIG_MENU -> "); fflush(stdout);
		selection = *GetLine();	/* prompt user for input */

		switch( selection ){
		case 'b': SetBitRate(); break;
		case 't': SetBusTiming(); break;
		case 'f': SetGlobalFilter(); break;
		case 'c': ConfigMsgObj(); break;
		case 'l': SetLoopback(); break;
		case 'e': EnableCan(); break;
		case 'q': break;
		default:  printf("Illegal Input. Try again...\n");
		}
	}
}


static void ReadFrame( u_int32 objNr, int reconf )
{
	static int32 timeout=0;
	MSCAN_FRAME frm;

	if( reconf ){
		GetInteger("Read timeout [ms]: (0=wait forever, -1=don't wait)", 
				   &timeout);
	}
	CHK( mscan_read_msg( G_path, objNr, timeout, &frm ) == 0 );
	DumpFrame("rxed", &frm );

 ABORT:
	return;
}

static void WriteFrame( u_int32 objNr, int reconf )
{
	static int32 timeout=0;
	static u_int32 id=0;
	static char rtrMode='N', extMode='S';
	static MSCAN_FRAME frm;	
	char *line;
	u_int8 i;

	if( reconf ){
		GetHex("Identifier", &id );
		rtrMode = GetChoice( "(R)TR frame (N)ormal frame", rtrMode, "RN" );
		extMode = GetChoice( "(E)xt frame (S)tandard frame", extMode, "ES" );
	
		frm.id = id;
		frm.flags = ((rtrMode=='R') ? MSCAN_RTR : 0) |
			((extMode=='E') ? MSCAN_EXTENDED : 0);

		for( i=0; i<9; i++ ){
			printf("Data byte %d [q to quit] [0x%02x]: ", i, frm.data[i] );
			fflush(stdout);
	
			line = GetLine();
			if( line[0] == 'q' )
				break;

			if( line[0] != '\0' )
				frm.data[i] = (u_int8)strtoul( line, NULL, 16 );
		}
		if( i==9 ) i=8;
		frm.dataLen = i;

		GetInteger("Write timeout [ms]: (0=wait forever, -1=don't wait)", 
				   &timeout);
	}
	DumpFrame("Tx", &frm );


	CHK( mscan_write_msg( G_path, objNr, timeout, &frm ) == 0 );
 ABORT:
	return;
}

static void FifoStatus( u_int32 objNr )
{
	u_int32 entries;
	MSCAN_DIR direction;

	CHK( mscan_queue_status( G_path, objNr, &entries, &direction ) == 0 );

	switch( direction ){
	case MSCAN_DIR_RCV:
		printf("rx entries=%ld\n", entries );
		break;
	case MSCAN_DIR_XMT:
		printf("free tx entries=%ld\n", entries );
		break;
	case MSCAN_DIR_DIS:
		printf("object disabled\n");
		break;
	}
 ABORT:
	return;
}

static void FifoClear( u_int32 objNr )
{
	CHK( mscan_queue_clear( G_path, objNr, FALSE ) == 0 );
 ABORT:
	return;
}

static void ClearBusOff( void )
{
	CHK( mscan_clear_busoff( G_path ) == 0 );
 ABORT:
	return;
}

static void NodeStatus( void )
{
	MSCAN_NODE_STATUS status;

	CHK( mscan_node_status( G_path, &status ) == 0 );

	switch( status ){
	case MSCAN_NS_ERROR_ACTIVE:	printf("Error active\n"); break;
	case MSCAN_NS_ERROR_PASSIVE:printf("Error passive\n"); break;
	case MSCAN_NS_BUS_OFF:		printf("Bus off\n"); break;
	default: printf("Unknown status\n"); break;
	}

 ABORT:
	return;
}

static void ReadError( void )
{
	u_int32 errCode, nr, entries;

	CHK( mscan_queue_status( G_path, 0, &entries, NULL ) == 0 );

	if( entries > 0 ){
		CHK( mscan_read_error( G_path, &errCode, &nr ) == 0 );
		printf("Error code %ld (%s), obj Nr %ld\n",
			   errCode, mscan_errobj_msg(errCode), nr );
	}
	else
		printf("no entries in error fifo\n");
 ABORT:
	return;
}

/**********************************************************************/
/** Program entry point
 *
 * \return success (0) always
 */
int main( int argc, char *argv[] )
{
	int32	n;
	char	*device,*errstr,buf[40],selection=0, lastSelection=0;
	int32 objNr = 1;
	static MSCAN_FILTER defaultFlt = { 
		0,
		0xffffffff,
		0,
		0 
	};

	/*--------------------+
    |  check arguments    |
    +--------------------*/
	if ((errstr = UTL_ILLIOPT("?", buf))) {	/* check args */
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

	/*--------------------+
    |  open path & config |
    +--------------------*/
	CHK( (G_path = mscan_init(device)) >= 0 );

	/*--- create default objects ---*/
	CHK( mscan_config_msg( G_path, 0, MSCAN_DIR_RCV, 10, NULL ) == 0 );
	CHK( mscan_config_msg( G_path, 1, MSCAN_DIR_XMT, 10, NULL ) == 0 );
	CHK( mscan_config_msg( G_path, 2, MSCAN_DIR_RCV, 10, &defaultFlt) == 0 );

	printf("Default message objects created:\n");
	printf(" obj 0: (error object) 10 entries\n");
	printf(" obj 1: (Tx) 10 entries\n");
	printf(" obj 2: (Rx, all std frames) 10 entries\n");

	/*--------------------+
	|  Show Menu          |
	+--------------------*/
	while( selection != 'q' ){
		
		printf("\n\n==== MSCAN - Main Menu ====\n");
		printf("c - configuration...\n\n");

		printf("o - set current message object number (used by [rwfx])\n");
		printf("r - read one frame from rx fifo\n");
		printf("w - write one frame to tx fifo\n");
		printf("a - repeat last [r/w] command\n");
		printf("f - get FIFO status\n");
		printf("x - clear FIFO\n\n");

		printf("b - clear bus off\n");
		printf("n - get node status\n");
		printf("C - read error counters\n");
		printf("e - read from error object\n");
		printf("q - quit\n");

		printf("MSCAN_MENU -> "); fflush(stdout);
		selection = *GetLine();	/* prompt user for input */

		switch( selection ){
		case 'c': Configure(); 		break;

		case 'o': GetInteger( "Message object number", &objNr );
			break;

		case 'r': ReadFrame(objNr, TRUE); 		break;

		case 'w': WriteFrame(objNr, TRUE);		break;

		case 'a':
			switch( lastSelection ){
			case 'r': ReadFrame(objNr, FALSE); 		break;
			case 'w': WriteFrame(objNr, FALSE);		break;
			}
			break;

		case 'f': FifoStatus(objNr);		break;
		case 'x': FifoClear(objNr);			break;

		case 'b': ClearBusOff();			break;
		case 'n': NodeStatus();				break;
		case 'C': ReadErrorCounters(); 		break;
		case 'e': ReadError();				break;

		case 'q': break;
		default:  printf("Illegal Input. Try again...\n");
		}

		if( selection == 'r' || selection == 'w' )
			lastSelection = selection;
	}
 ABORT:
	mscan_term( G_path );
	return 0;
}


static char *GetLine(void)
{
	static char line[80];
	clearerr(stdin);
	
	if( fgets( line, sizeof(line), stdin )) {
		if( line[strlen(line)-1] == '\n' )
			line[strlen(line)-1] = '\0';
	}
	else 
		printf("*** ERROR in fgets!\n");

	return line;
}

static void GetInteger( char *prompt, int32 *val )
{
	char *line;

	printf("%s [%ld]: ", prompt, *val );
	fflush(stdout);
	
	line = GetLine();
	if( line[0] == '\0' )
		return;

	*val = atoi(line);
}

static void GetHex( char *prompt, u_int32 *val )
{
	char *line;

	printf("%s [0x%08lx]: ", prompt, *val );
	fflush(stdout);
	
	line = GetLine();
	if( line[0] == '\0' )
		return;

	*val = strtoul( line, NULL, 16 );
}


static char GetChoice( char *str, char def, char *choices )
{
	char c;

	do {
		printf("%s [%c] ", str, def);
		fflush(stdout);
		c = *GetLine();
		if( c=='\0' )
			c=def;
		c = (char)toupper( (int)c );
		if( strchr(choices,c) )
			break;
		printf("\n");
	} while(1);
	return c;
}

static void DumpFrame( char *msg, const MSCAN_FRAME *frm )
{
	int i;
	printf("%s: ID=0x%lx%s%s data=", 
		   msg,
		   frm->id, 
		   (frm->flags & MSCAN_EXTENDED) ? "x":"", 
		   (frm->flags & MSCAN_RTR) ? " RTR":"");

	for(i=0; i<frm->dataLen; i++ ){
		printf("%02x ", frm->data[i] );
	}
	printf("\n");
}

