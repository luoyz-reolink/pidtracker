#created by xzyang[2013-08-27]


#############################################################
ifndef _CBB_PATH_
$(error "You must give the environment config[source m.env]!")
endif
ifndef _PUBLIC_PATH_
$(error "You must give _PUBLIC_PATH_ path[source m.env]!")
endif 
include ${_PUBLIC_PATH_}/script/base.mk
#############################################################



#############################################################
_LIB_NAME_:=pidtracker
_VERSION_FILE_DIR_:=${_PWD_}/include
_INCLUDE_LIST_+=-I${_CBB_BASIC_LIB_DIR_}/libtinyxml/include
_LIB_DIR_LIST_+=-L${_CBB_BASIC_LIB_DIR_}/libtinyxml
_LIB_FILE_LIST_+=-ltinyxml

_SRC_LIST_+=$(wildcard ${_PWD_}/src/*.cpp)

_INCLUDE_LIST_+=-I${_PWD_}/include
_INCLUDE_LIST_+=-I${_PUBLIC_PATH_}/pub


_RELEASE_HEADER_FILE_LIST_+=$(wildcard ${_PWD_}/include/*.h)
#############################################################

#############################################################
include ${_PUBLIC_PATH_}/script/basic_module.mk
#############################################################


