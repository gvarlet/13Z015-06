#***************************  M a k e f i l e  *******************************
#
#         Author: kp
#          $Date: 2004/06/14 11:58:11 $
#      $Revision: 1.2 $
#
#    Description: Makefile definitions for the MSCAN (BOROMIR) driver
#                 
#
#---------------------------------[ History ]---------------------------------
#
#   ts 17.02.2016   changed naming to z15 as with UART Z25 driver
#
#   ----- end of cvs maintenance ------
#   $Log: driver_boromir.mak,v $
#   Revision 1.2  2004/06/14 11:58:11  kp
#   cosmetics
#
#   Revision 1.1  2003/02/07 13:16:35  kp
#   Initial Revision
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2003 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************

MAK_NAME=z15

MAK_SWITCH= $(SW_PREFIX)MAC_MEM_MAPPED 	\
	    $(SW_PREFIX)MSCAN_IS_Z15 		\
	    $(SW_PREFIX)MSCAN_VARIANT=BOROMIR

MAK_LIBS=$(LIB_PREFIX)$(MEN_LIB_DIR)/desc$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/oss$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/dbg$(LIB_SUFFIX)	\

MAK_INCL=$(MEN_INC_DIR)/mscan.h \
		 $(MEN_INC_DIR)/mscan_drv.h	\
		 $(MEN_INC_DIR)/mscan_api.h	\
		 $(MEN_MOD_DIR)/mscan_int.h	\
         $(MEN_INC_DIR)/men_typs.h	\
         $(MEN_INC_DIR)/oss.h		\
         $(MEN_INC_DIR)/mdis_err.h	\
         $(MEN_INC_DIR)/maccess.h	\
         $(MEN_INC_DIR)/desc.h		\
         $(MEN_INC_DIR)/mdis_api.h	\
         $(MEN_INC_DIR)/mdis_com.h	\
         $(MEN_INC_DIR)/ll_defs.h	\
         $(MEN_INC_DIR)/ll_entry.h	\
         $(MEN_INC_DIR)/dbg.h		\

MAK_INP1=mscan_drv$(INP_SUFFIX)


MAK_INP=$(MAK_INP1) 



