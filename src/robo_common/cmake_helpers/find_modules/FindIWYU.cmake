 ################################################################################
 #    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    #
 #                                                                              #
 #              This software is distributed under the terms of the             # 
 #         GNU Lesser General Public Licence version 3 (LGPL) version 3,        #  
 #                  copied verbatim in the file "LICENSE"                       #
 ################################################################################
# - Try to find the include-what-you-use (IWYU) instalation
# - This tool is based on llvm and will check the #include statements
# - Once done this will define
#
# - Define macros to run the tool
#
#  IWYU_FOUND - system has include-what-you-use

Message(STATUS "Looking for IWYU...")

Find_File(IWYU_BINARY_PATH NAMES include-what-you-use PATHS ENV PATH)

If(IWYU_BINARY_PATH)
  Set(IWYU_FOUND TRUE)
EndIf(IWYU_BINARY_PATH)

If (IWYU_FOUND)
  If (NOT IWYU_FIND_QUIETLY)
    MESSAGE(STATUS "Looking for IWYU... - found ${IWYU_BINARY_PATH}")
    SET(ENV{ALL_HEADER_RULES} "")
  endif (NOT IWYU_FIND_QUIETLY)
else (IWYU_FOUND)
    message(STATUS "Looking for IWYU... - Not found")
endif (IWYU_FOUND)

#########

Macro(CHECK_HEADERS INFILES INCLUDE_DIRS_IN HEADER_RULE_NAME)

#  Message("Create Header Checker for ${HEADER_RULE_NAME}")

  Set(_INCLUDE_DIRS)
  Set(_all_files)

  ForEach(_current_FILE ${INCLUDE_DIRS_IN})
    Set(_INCLUDE_DIRS ${_INCLUDE_DIRS} -I${_current_FILE})   
  EndForEach(_current_FILE ${INCLUDE_DIRS_IN})

  ForEach (_current_FILE ${INFILES})

    Get_Filename_Component(file_name ${_current_FILE} NAME_WE)
    Get_Filename_Component(path ${_current_FILE} PATH)
    
    Set(_current_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${_current_FILE}")
    Set(headerfile "${CMAKE_CURRENT_SOURCE_DIR}/${path}/${file_name}.h")
   
    If(NOT EXISTS ${headerfile})
      Set(headerfile)
    EndIf(NOT EXISTS ${headerfile})
  
    Set(outfile "${CMAKE_CURRENT_BINARY_DIR}/${file_name}.iwyu")

    ADD_CUSTOM_COMMAND(OUTPUT ${outfile} 
      COMMAND ${IWYU_BINARY_PATH} ${_current_FILE} ${_INCLUDE_DIRS} 2> ${outfile}
      DEPENDS  ${_current_FILE} ${headerfile}
   )

    set(_all_files ${_all_files}  ${outfile})

  endforeach (_current_FILE ${INFILES})

  Add_CUSTOM_TARGET(${HEADER_RULE_NAME} 
     COMMAND touch ${CMAKE_BINARY_DIR}/${RULE_NAME}
     DEPENDS ${_all_files}
  )

  SET(BLA $ENV{ALL_HEADER_RULES})
  SET(BLA ${BLA} ${HEADER_RULE_NAME})
  SET(ENV{ALL_HEADER_RULES} "${BLA}")

EndMacro(CHECK_HEADERS INFILES INCLUDE_DIRS_IN HEADER_RULE_NAME)


