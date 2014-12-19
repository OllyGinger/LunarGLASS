cmake_minimum_required(VERSION 2.8)

include_directories(. ../.. ../../Core ../GLSL ${GLSLANGINCLUDES} ${LLVMINCLUDES})

set(SOURCES
    BottomToMetal.cpp)

set(HEADERS
    MetalManager.h
    MetalTarget.h)

add_library(MetalBackend STATIC ${SOURCES} ${HEADERS})

if(WIN32)
    source_group("Source" REGULAR_EXPRESSION "[^/]*")
endif(WIN32)

install(TARGETS MetalBackend
        ARCHIVE DESTINATION lib)