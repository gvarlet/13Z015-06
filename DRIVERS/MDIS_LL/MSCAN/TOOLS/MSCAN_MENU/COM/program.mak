#***************************  M a k e f i l e  *******************************
#
#         Author: kp
#          $Date: 2004/06/14 11:58:15 $
#      $Revision: 1.2 $
#
#    Description: Makefile definitions for MSCAN_MENU
#
#---------------------------------[ History ]---------------------------------
#
#   $Log: program.mak,v $
#   Revision 1.2  2004/06/14 11:58:15  kp
#   cosmetics
#
#   Revision 1.1  2003/01/29 14:03:09  kp
#   Initial Revision
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2003 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************

MAK_NAME=mscan_menu

MAK_LIBS=$(LIB_PREFIX)$(MEN_LIB_DIR)/mscan_api$(LIB_SUFFIX)	\
		 $(LIB_PREFIX)$(MEN_LIB_DIR)/mdis_api$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/usr_oss$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/usr_utl$(LIB_SUFFIX)	\

MAK_INCL=$(MEN_INC_DIR)/mscan_api.h	\
         $(MEN_INC_DIR)/men_typs.h	\
         $(MEN_INC_DIR)/mdis_api.h	\
         $(MEN_INC_DIR)/mdis_err.h	\
         $(MEN_INC_DIR)/usr_oss.h	\
         $(MEN_INC_DIR)/usr_err.h	\
         $(MEN_INC_DIR)/usr_utl.h	\

MAK_INP1=mscan_menu$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
