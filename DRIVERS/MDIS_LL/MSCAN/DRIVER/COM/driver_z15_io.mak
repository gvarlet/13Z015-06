#***************************  M a k e f i l e  *******************************
#
#         Author: michael.roth@men.de
#          $Date: 2013/11/27 15:35:15 $
#      $Revision: 1.1 $
#
#    Description: Makefile definitions for the MSCAN (BOROMIR) driver for 
#                 IO mapped (x86) FPGAs
#
#---------------------------------[ History ]---------------------------------
#
#   ts 17.02.2016   changed naming to z15 as with UART Z25 driver
#
#   ----- end of cvs maintenance ------
#   $Log: driver_boromir_io.mak,v $
#   Revision 1.1  2013/11/27 15:35:15  MRoth
#   Initial Revision
#
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2011 by MEN Mikro Elektronik GmbH, Nuremberg, Germany
#*****************************************************************************

MAK_NAME=z15_io

MAK_SWITCH= $(SW_PREFIX)MAC_IO_MAPPED \
			$(SW_PREFIX)MSCAN_IS_Z15 \
			$(SW_PREFIX)MSCAN_VARIANT=Z15

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

