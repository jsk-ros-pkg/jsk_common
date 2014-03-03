cmake_minimum_required (VERSION 2.4.6) # need INSTALL(DIRECTORY ...)
project (crlibm)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

include(CheckIncludeFile)
include(CheckTypeSize)

# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0005
cmake_policy(SET CMP0005 NEW)
add_definitions(-DPACKAGE=\"crlibm\")
add_definitions(-DVERSION=\"1.0beta4\")

# Compile the filib-compatible interval functions
# BUILD_INTERVAL_FUNCTIONS

# architecture- and system-specific FPU control header file
check_include_file(fpu_control.h HAS_FPU_CONTROL_H)
if( HAS_FPU_CONTROL_H )
  add_definitions("-DCRLIBM_HAS_FPU_CONTROL=1")
endif()

if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )
  add_definitions("-DCRLIBM_TYPECPU_AMD64=1")
elseif( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "i486" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "i686" )
  add_definitions("-DCRLIBM_TYPECPU_X86=1")
elseif( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "ppc" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "powerpc" )
  add_definitions("-DCRLIBM_TYPECPU_POWERPC=1")
  add_definitions("-DWORDS_BIGENDIAN=1")
elseif( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "sparc" )
  add_definitions("-DCRLIBM_TYPECPU_SPARC=1")
endif()

if( BSD )
  add_definitions("-DCRLIBM_TYPEOS_BSD=1")
endif()
if( CYGWIN )
  add_definitions("-DCRLIBM_TYPEOS_CYGWIN")
endif()
if( HPUX )
  add_definitions("-DCRLIBM_TYPEOS_HPUX")
endif()

check_include_file(fenv.h HAVE_FENV_H)
if(HAVE_FENV_H)
  add_definitions("-DHAVE_FENV_H=1")
endif()

check_include_file(inttypes.h HAVE_INTTYPES_H)
if(HAVE_INTTYPES_H)
  add_definitions("-DHAVE_INTTYPES_H=1")
endif()

# Size of an SCS digit
add_definitions("-DSCS_NB_BITS=30")

# Number of digits in the SCS structure
add_definitions("-DSCS_NB_WORDS=8")

set(source_dir build/crlibm-1.0beta4)

check_type_size("long double" HAVE_LONG_DOUBLE BUILTIN_TYPES_ONLY)
if( HAS_FPU_CONTROL_H AND HAVE_LONG_DOUBLE EQUAL 16 )
  message(STATUS "compiling with double extended (long double) floating point")
  set(crlibm_SOURCES
    ${source_dir}/crlibm_private.c
    ${source_dir}/triple-double.c
    ${source_dir}/exp-td.c
    ${source_dir}/exp-td-standalone.c
    ${source_dir}/expm1-standalone.c
    ${source_dir}/expm1.c
    ${source_dir}/log-de.c
    ${source_dir}/log1p.c
    ${source_dir}/log10-td.c
    ${source_dir}/log2-td.c
    ${source_dir}/rem_pio2_accurate.c
    ${source_dir}/trigo_fast.c
    ${source_dir}/trigo_accurate.c
    ${source_dir}/trigpi.c
    ${source_dir}/asincos.c
    ${source_dir}/pow.c
    ${source_dir}/atan_fast.c
    ${source_dir}/atan_accurate.c
    ${source_dir}/csh_fast.c
    ${source_dir}/scs_lib/addition_scs.c
    ${source_dir}/scs_lib/double2scs.c
    ${source_dir}/scs_lib/print_scs.c
    ${source_dir}/scs_lib/scs2mpfr.c
    ${source_dir}/scs_lib/zero_scs.c
    ${source_dir}/scs_lib/division_scs.c
    ${source_dir}/scs_lib/multiplication_scs.c
    ${source_dir}/scs_lib/scs2double.c
    ${source_dir}/scs_lib/scs_private.c
    )
else()
  set(crlibm_SOURCES
    ${source_dir}/crlibm_private.c
    ${source_dir}/triple-double.c
    ${source_dir}/exp-td.c
    ${source_dir}/exp-td-standalone.c
    ${source_dir}/expm1-standalone.c
    ${source_dir}/expm1.c
    ${source_dir}/log.c
    ${source_dir}/log1p.c
    ${source_dir}/rem_pio2_accurate.c
    ${source_dir}/trigo_fast.c 
    ${source_dir}/trigo_accurate.c
    ${source_dir}/trigpi.c
    ${source_dir}/asincos.c
    ${source_dir}/pow.c
    ${source_dir}/atan_fast.c
    ${source_dir}/atan_accurate.c
    ${source_dir}/csh_fast.c
    ${source_dir}/scs_lib/addition_scs.c
    ${source_dir}/scs_lib/double2scs.c
    ${source_dir}/scs_lib/print_scs.c
    ${source_dir}/scs_lib/scs2mpfr.c
    ${source_dir}/scs_lib/zero_scs.c
    ${source_dir}/scs_lib/division_scs.c
    ${source_dir}/scs_lib/multiplication_scs.c
    ${source_dir}/scs_lib/scs2double.c
    ${source_dir}/scs_lib/scs_private.c
    )
endif()

add_library(crlibm_s STATIC ${crlibm_SOURCES})
add_library(crlibm SHARED ${crlibm_SOURCES})

install(FILES ${source_dir}/crlibm.h DESTINATION include)
install(TARGETS crlibm crlibm_s DESTINATION lib)
