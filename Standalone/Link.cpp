//===- Link.cpp - Translate GLSL IR to LunarGLASS Top IR -----------------===//
//
// Copyright (C) 2010-2014 LunarG, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//     Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
// 
//     Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
// 
//     Neither the name of LunarG Inc. nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//===----------------------------------------------------------------------===//
//
// Author: John Kessenich, LunarG
//
// Stubs for deprecated glslang link step.
//
//===----------------------------------------------------------------------===//

#include "glslang/Include/Common.h"
#include "glslang/Include/ShHandle.h"

// These stubs use a now deprecated interface to the glslang front end.
// Intra-stage linking should be done before using LunarGLASS to compile a stage.

#ifdef USE_DEPRECATED_GLSLANG

//
// Actual link object, derived from the shader handle base classes.
//
class TGenericLinker : public TLinker {
public:
    TGenericLinker(EShExecutable e, int dOptions) : TLinker(e, infoSink) { }
    bool link(TCompilerList&, TUniformMap*) { return true; }
	void getAttributeBindings(ShBindingTable const **t) const { }
    TInfoSink infoSink;
};

//
// The internal view of a uniform/float object exchanged with the driver.
//
class TUniformLinkedMap : public TUniformMap {
public:
    TUniformLinkedMap() { }
    virtual int getLocation(const char* name) { return 0; }
};

#endif

TShHandleBase* ConstructLinker(EShExecutable executable, int debugOptions)
{
    return 0;
}

void DeleteLinker(TShHandleBase* linker)
{
    delete linker;
}

TUniformMap* ConstructUniformMap()
{
    return 0;
}

void DeleteUniformMap(TUniformMap* map)
{
    delete map;
}

TShHandleBase* ConstructBindings()
{
    return 0;
}

void DeleteBindingList(TShHandleBase* bindingList)
{
    delete bindingList;
}
