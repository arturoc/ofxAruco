# -helper macro to add a "doc" target with CMake build system. 
# and configure doxy.config.in to doxy.config
#
# target "doc" allows building the documentation with doxygen/dot on WIN32 and Linux
# Creates .chm windows help file if MS HTML help workshop 
# (available from http://msdn.microsoft.com/workshop/author/htmlhelp)
# is installed with its DLLs in PATH.
#
#
# Please note, that the tools, e.g.:
# doxygen, dot, latex, dvips, makeindex, gswin32, etc.
# must be in path.
#
# Note about Visual Studio Projects: 
# MSVS hast its own path environment which may differ from the shell.
# See "Menu Tools/Options/Projects/VC++ Directories" in VS 7.1
#
# author Jan Woetzel 2004-2006
# www.mip.informatik.uni-kiel.de/~jw
#
# Modified by Luis Díaz 2009 http://plagatux.es

MACRO(GENERATE_DOCUMENTATION DOX_CONFIG_FILE)
FIND_PACKAGE(Doxygen)
IF (DOXYGEN_FOUND)
	#Define variables
	SET(SRCDIR "${PROJECT_SOURCE_DIR}/src")
	SET(TAGFILE "${PROJECT_BINARY_DIR}/doc/${PROJECT_NAME}.tag")

	IF (USE_CHM AND WIN32)
		SET(WIN_CHM "YES")
		SET(CHM_FILE "${PROJECT_SOURCE_DIR}/doc/help.chm")
		SET (BINARY_TOC "YES")
		SET (TOC_EXPAND "YES")
	ELSE()
		SET(WIN_CHM "NO")
		SET (BINARY_TOC "NO")
		SET (TOC_EXPAND "NO")
	ENDIF()

	IF (USE_LATEX)
		SET(GENERATE_PDF "YES")
		SET(GENERATE_LATEX "YES")
		SET(LATEXOUT "latex")
	ELSE()
		SET(GENERATE_PDF "NO")
		SET(GENERATE_LATEX "NO")
	ENDIF()

	IF (NOT USE_DOT)
		SET(DOXYGEN_DOT_FOUND "NO")
	ENDIF()

	#click+jump in Emacs and Visual Studio (for doxy.config) (jw)
	IF    (CMAKE_BUILD_TOOL MATCHES "(msdev|devenv)")
		SET(DOXY_WARN_FORMAT "\"$file($line) : $text \"")
	ELSE  (CMAKE_BUILD_TOOL MATCHES "(msdev|devenv)")
		SET(DOXY_WARN_FORMAT "\"$file:$line: $text \"")
	ENDIF (CMAKE_BUILD_TOOL MATCHES "(msdev|devenv)")

	# we need latex for doxygen because of the formulas
	FIND_PACKAGE(LATEX)
	IF    (NOT LATEX_COMPILER)
		MESSAGE(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user inetraction on doxy run.")
	ENDIF (NOT LATEX_COMPILER)
	IF    (NOT MAKEINDEX_COMPILER)
		MESSAGE(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
	ENDIF (NOT MAKEINDEX_COMPILER)
	IF    (NOT DVIPS_CONVERTER)
		MESSAGE(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
	ENDIF (NOT DVIPS_CONVERTER)
	  
	# Check config file
	IF   (EXISTS "${DOX_CONFIG_FILE}")
		CONFIGURE_FILE(${DOX_CONFIG_FILE} ${CMAKE_CURRENT_BINARY_DIR}/doxy.config @ONLY ) #OUT-OF-PLACE LOCATION
		SET(DOXY_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/doxy.config")
	ELSE () 
		MESSAGE(SEND_ERROR "Please create configuration file for doxygen in ${CMAKE_CURRENT_SOURCE_DIR}")
	ENDIF()

	# Add target
	ADD_CUSTOM_TARGET(doc ${DOXYGEN_EXECUTABLE} ${DOXY_CONFIG})

	IF (WIN32 AND GENERATE_WIN_CHM STREQUAL "YES")
		FIND_PACKAGE(HTMLHelp)
		IF   (HTML_HELP_COMPILER)      
			ADD_CUSTOM_TARGET(winhelp ${HTML_HELP_COMPILER} ${HHP_FILE})
			ADD_DEPENDENCIES (winhelp doc)
			IF   (EXISTS ${CHM_FILE})
				IF   (PROJECT_NAME)
					SET(OUT "${PROJECT_NAME}")
				ELSE ()
					SET(OUT "Documentation") # default
				ENDIF()
				IF   (${PROJECT_NAME}_VERSION_MAJOR)
					SET(OUT "${OUT}-${${PROJECT_NAME}_VERSION_MAJOR}")
					IF   (${PROJECT_NAME}_VERSION_MINOR)
						SET(OUT  "${OUT}.${${PROJECT_NAME}_VERSION_MINOR}")
						IF   (${PROJECT_NAME}_VERSION_PATCH)
							SET(OUT "${OUT}.${${PROJECT_NAME}_VERSION_PATCH}")      
						ENDIF()
					ENDIF()
				ENDIF()
				SET(OUT  "${OUT}.chm")
				INSTALL(FILES ${CHM_FILE} DESTINATION "doc" RENAME "${OUT}")
			ENDIF()
		ELSE()
			MESSAGE(FATAL_ERROR "You have not Microsoft Help Compiler")
		ENDIF()
	ENDIF () 

	INSTALL(DIRECTORY "${PROJECT_BINARY_DIR}/html/" DESTINATION "share/doc/lib${PROJECT_NAME}")

ENDIF(DOXYGEN_FOUND)
ENDMACRO(GENERATE_DOCUMENTATION)
