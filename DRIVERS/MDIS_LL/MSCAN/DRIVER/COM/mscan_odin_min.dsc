#************************** MDIS5 device descriptor *************************
#
#        Author: kp
#         $Date: 2010/02/25 18:05:34 $
#     $Revision: 1.2 $
#
#   Description: Metadescriptor for M59
#
#****************************************************************************

CANODIN_1  {
	#------------------------------------------------------------------------
	#	general parameters (don't modify)
	#------------------------------------------------------------------------
    DESC_TYPE        = U_INT32  1           # descriptor type (1=device)
    HW_TYPE          = STRING   CANODIN     # hardware name of device

	#------------------------------------------------------------------------
	#	reference to base board
	#------------------------------------------------------------------------
    BOARD_NAME       = STRING   ISA_1     # device name of baseboard
    DEVICE_SLOT      = U_INT32  0           # used slot on baseboard (0..n)

	#------------------------------------------------------------------------
	#	device parameters
	#------------------------------------------------------------------------
	CANCLOCK		 = U_INT32	33000000

	DEBUG_LEVEL = U_INT32 0xc0008007
	#DEBUG_LEVEL_OSS = U_INT32 0xc0008003
}
