#***************************  M a k e f i l e  *******************************
#
#         Author: uf
#          $Date: 2008/10/16 11:41:36 $
#      $Revision: 1.1 $
#
#    Description: Makefile definitions for MSCAN tool
#
#---------------------------------[ History ]---------------------------------
#
#   $Log: program.mak,v $
#   Revision 1.1  2008/10/16 11:41:36  ufranke
#   Initial Revision
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2008 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************

MAK_NAME=mscan_pingpong

MAK_LIBS=$(LIB_PREFIX)$(MEN_LIB_DIR)/mscan_api$(LIB_SUFFIX)     \
		 $(LIB_PREFIX)$(MEN_LIB_DIR)/mdis_api$(LIB_SUFFIX)    \
         $(LIB_PREFIX)$(MEN_LIB_DIR)/usr_oss$(LIB_SUFFIX)     \
         $(LIB_PREFIX)$(MEN_LIB_DIR)/usr_utl$(LIB_SUFFIX)     \

MAK_INCL=$(MEN_INC_DIR)/mscan_api.h     \
         $(MEN_INC_DIR)/mscan_drv.h    \
         $(MEN_INC_DIR)/men_typs.h    \
         $(MEN_INC_DIR)/mdis_api.h    \
         $(MEN_INC_DIR)/mdis_err.h    \
         $(MEN_INC_DIR)/usr_oss.h     \
         $(MEN_INC_DIR)/usr_err.h     \
         $(MEN_INC_DIR)/usr_utl.h     \

MAK_INP1=mscan_pingpong$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)



