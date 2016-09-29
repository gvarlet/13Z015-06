#***************************  M a k e f i l e  *******************************
#
#         Author: michael.roth@men.de
#          $Date: 2010/01/26 18:19:42 $
#      $Revision: 1.1 $
#
#    Description: Makefile definitions for MSCAN_CLIENT_SRV
#
#---------------------------------[ History ]---------------------------------
#
#   $Log: program.mak,v $
#   Revision 1.1  2010/01/26 18:19:42  MRoth
#   Initial Revision
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2010 by MEN Mikro Elektronik GmbH, Nuremberg, Germany
#*****************************************************************************

MAK_NAME=mscan_client_srv

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

MAK_INP1=mscan_client_srv$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
