include(SIPMacros)

message( ${SIP_VERSION} " - The version of SIP found expressed as a 6 digit hex number suitable for comparison as a string.")
message( ${SIP_VERSION_STR} " - The version of SIP found as a human readable string.")
message( ${SIP_EXECUTABLE} " - Path and filename of the SIP command line executable.")
message( ${SIP_INCLUDE_DIR} " - Directory holding the SIP C++ header file.")
message( ${SIP_DEFAULT_SIP_DIR} " - default SIP dir" )

include_directories(${SIP_INCLUDE_DIR})

set(SIP_INCLUDES
    ${SIP_DEFAULT_SIP_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR})

set(SIP_CONCAT_PARTS 1)
set(SIP_TAGS ALL)

set(PYTHON_SITE_PACKAGES_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_SHORT_VERSION}/site-packages/)

file(GLOB_RECURSE AIR_NAVIGATION_SIP_FILES *.sip)

set(SIP_EXTRA_FILES_DEPEND ${AIR_NAVIGATION_SIP_FILES})
add_sip_python_module(air_navigation_ ./air_navigation/mod.sip air_navigation)

install(FILES
    air_navigation/__init__.py
    DESTINATION ${PYTHON_SITE_PACKAGES_INSTALL_DIR}/air_navigation)

