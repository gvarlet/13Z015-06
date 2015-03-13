#************************** MDIS5 device descriptor *************************
#
#        Author: kp
#         $Date: 2010/02/25 18:05:23 $
#     $Revision: 1.2 $
#
#   Description: Metadescriptor for MSCAN on EM04
#
#****************************************************************************

BOROMIR_1  {
	#------------------------------------------------------------------------
	#	general parameters (don't modify)
	#------------------------------------------------------------------------
    DESC_TYPE        = U_INT32  1           # descriptor type (1=device)
    HW_TYPE          = STRING   BOROMIR     # hardware name of device

	#------------------------------------------------------------------------
	#	reference to base board
	#------------------------------------------------------------------------
    BOARD_NAME       = STRING   CHAMEM04AD66  # device name of baseboard
    DEVICE_SLOT      = U_INT32  0           # used slot on baseboard (0..n)

	#------------------------------------------------------------------------
	#	device parameters
	#------------------------------------------------------------------------
	CANCLOCK		 = U_INT32	32000000
}
