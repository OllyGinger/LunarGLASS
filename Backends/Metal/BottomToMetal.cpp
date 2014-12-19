//===- BottomToMetal.cpp - Translate bottom IR to GLSL ---------------------===//
//
// LunarGLASS: An Open Modular Shader Compiler Architecture
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
// A Metal back end for LunarGLASS.
//
// Inherits from 
//   - BackEnd to supply back end information to LunarGLASS.
//   - BackEndTranslator to translate to GLSL, by implementing the methods called
//     by the bottom translator.
// Factories for each of these appear below.
//
// It writes the resulting GLSL shader to standard output.
//
//===----------------------------------------------------------------------===//

#define _CRT_SECURE_NO_WARNINGS
#ifdef _WIN32
#define snprintf _snprintf
#endif

#include <cstdio>
#include <string>
#include <sstream>
#include <map>
#include <set>
#include <vector>
#include <cstdio>

// LunarGLASS includes
#include "Core/Revision.h"
#include "Core/Exceptions.h"
#include "Core/Util.h"
#include "Core/BottomIR.h"
#include "Core/Backend.h"
#include "Core/PrivateManager.h"
#include "Core/TopBuilder.h"
#include "Core/metadata.h"
#include "Core/Passes/Util/ConstantUtil.h"
#include "MetalTarget.h"

// glslang includes
#include "MetalTarget.h"

// LLVM includes
#pragma warning(push, 1)
#include "llvm/IR/Module.h"
#pragma warning(pop)

namespace {
    bool UseLogicalIO = true;

    bool ValidIdentChar(int c)
    {
        return c == '_' || 
               (c >= 'a' && c <= 'z') ||
               (c >= 'A' && c <= 'Z') ||
               (c >= '0' && c <= '9');
    }

    // Taken from VS <function> hash:
    unsigned int _Hash_seq(const unsigned char *_First, unsigned int _Count)
    {	// FNV-1a hash function for bytes in [_First, _First+_Count)
        const unsigned int _FNV_offset_basis = 2166136261U;
        const unsigned int _FNV_prime = 16777619U;

        unsigned int _Val = _FNV_offset_basis;
        for (unsigned int _Next = 0; _Next < _Count; ++_Next)
        {	// fold in another byte
            _Val ^= (unsigned int)_First[_Next];
            _Val *= _FNV_prime;
        }

        return (_Val);
    }

    void IntToString(unsigned int i, std::string& string)
    {
        char buf[2];
        buf[1] = 0;
        int radix = 36;
        while (i > 0) {
            unsigned int r = i % radix;
            if (r < 10) 
                buf[0] = '0' + r;
            else
                buf[0] = 'a' + r - 10;
            string.append(buf);
            i = i / radix;
        }
    }

};

//
// Implement the GLSL backend
//
class MetalBackEnd : public gla::BackEnd {
public:
    MetalBackEnd()
    {
        //decompose[gla::EDiClamp] = true;
        //decompose[gla::EDiMax] = true;
        //decompose[gla::EDiMin] = true;
        //decompose[gla::EDiSmoothStep] = true;
        //decompose[gla::EDiFilterWidth] = true;
    }
    virtual ~MetalBackEnd() { }

    virtual void getRegisterForm(int& outerSoA, int& innerAoS)
    {
        gla::BackEnd::getRegisterForm(outerSoA, innerAoS);
    }

    virtual void getControlFlowMode(gla::EFlowControlMode& flowControlMode,
                                    bool& breakOp, bool& continueOp,
                                    bool& earlyReturnOp, bool& discardOp)
    {
        gla::BackEnd::getControlFlowMode(flowControlMode, breakOp, continueOp,
                                         earlyReturnOp, discardOp);
    }

    virtual bool preferRegistersOverMemory()
    {
        return true;
    }

    virtual bool getRemovePhiFunctions()
    {
        return true;
    }

    virtual bool getDeclarePhiCopies()
    {
        return true;
    }

    virtual bool decomposeNaNCompares()
    {
        return true;
    }

    virtual bool hoistDiscards()
    {
        return false;
    }

    virtual bool useColumnBasedMatrixIntrinsics()
    {
        return true;
    }

    virtual bool useLogicalIo()
    {
        return UseLogicalIO;
    }
};

//
// factory for the GLSL backend
//
gla::BackEnd* gla::GetMetalBackEnd()
{
    return new MetalBackEnd();
}

void gla::ReleaseMetalBackEnd(gla::BackEnd* backEnd)
{
    delete backEnd;
}

class Assignment;

//
// To build up and hold an assignment until the entire statement is known, so it
// can be emitted all at once, and the left-hand side can be given a name then
// based on the entire statement.  Or, the entire right-hand side could just be 
// mapped as a forward substitution.
//
class Assignment : public std::ostringstream {
public:
    Assignment(gla::MetalTarget* target, const llvm::Instruction* instruction) : target(*target), instruction(instruction), lvalue(true) { }

    void emit()
    {
        if (member.size() > 0) {
            // emit a declaration, no assignment
            if (target.valueMap.find(instruction) == target.valueMap.end()) {
                target.newLine();
                target.emitGlaValue(target.shader, instruction, str().c_str());
                target.shader << ";";
            }
        }
        if (! lvalue && str().size() == 0)
            return;

        // emit the l-value
        target.newLine();
        if (lvalue)
            target.emitGlaValue(target.shader, instruction, str().c_str());
        if (member.size() > 0)
            target.shader << member;
        if (lvalue && str().size() > 0)
            target.shader << " = ";

        // emit the expression
        target.shader << str() << ";";
    }

    bool cantMap()
    {
        return member.size() > 0 || ! lvalue;
    }

    void map(bool needsParens)
    {
        if (cantMap()) {
            emit();
            return;
        }

        std::string parenthesized;
        if (needsParens)
            parenthesized.append("(");
        parenthesized.append(str());
        if (needsParens)
            parenthesized.append(")");
        target.mapExpressionString(instruction, parenthesized);
    }

    // TODO: generated-code performance: pass in false here in more places where
    // it can be known there is not an expansion of data.  E.g., sum of two
    // variables that die.
    // increasesData: means it is definitely known there is more data made, so always better to forward substitute (map)
    void mapOrEmit(bool increasesData, bool needsParens)
    {
        if (target.getSubstitutionLevel() == 0 || cantMap()) {
            emit();
            return;
        }

        bool doSubstitute = true;

        if (! target.cheapExpression(str())) {

            if (target.getSubstitutionLevel() < 2)
                doSubstitute = false;

            if (doSubstitute && str().size() > 120)
                doSubstitute = false;

            if (doSubstitute && target.modifiesPrecision(instruction))
                doSubstitute = false;

            if (doSubstitute && target.valueMap.find(instruction) != target.valueMap.end())
                doSubstitute = false;

            if (doSubstitute && ! increasesData && ! target.shouldSubstitute(instruction))
                doSubstitute = false;
        }

        if (doSubstitute)
            map(needsParens);
        else
            emit();
    }

    void setMember(const char* m) { member = m; }
    void setNoLvalue() { lvalue = false; }

protected:
    const llvm::Instruction* instruction;
    gla::MetalTarget& target;
    std::string member;
    bool lvalue;
};

//
// Factory for GLSL back-end translator
//

gla::GlslTranslator* gla::GetMetalTranslator(Manager* manager, bool obfuscate, bool filterInactive, int substitutionLevel)
{
    return new gla::MetalTarget(manager, obfuscate, filterInactive, substitutionLevel);
}

void gla::ReleaseMetalTranslator(gla::BackEndTranslator* target)
{
    delete target;
}

//
// File-local helpers
//

namespace {

using namespace gla;

bool IsIdentitySwizzle(int glaSwizzle, int width)
{
    for (int i = 0; i < width; ++i) {
        if (GetSwizzle(glaSwizzle, i) != i)
            return false;
    }

    return true;
}

bool IsIdentitySwizzle(const llvm::SmallVectorImpl<llvm::Constant*>& elts)
{
    for (int i = 0; i < (int)elts.size(); ++i) {
        if (IsUndef(elts[i]) || i != GetConstantInt(elts[i])) {
            return false;
        }
    }

    return true;
}

// Create the start of a scalar/vector conversion, but not for matrices.
void ConversionStart(std::ostringstream& out, llvm::Type* type, bool toIO)
{
    if (! IsInteger(type))
        return;

    if (GetComponentCount(type) == 1)
        out << (toIO ? "uint(" : "int(");
    else {
        out << (toIO ? "uvec" : "ivec");
        switch (GetComponentCount(type)) {
        case 2:  out << "2(";    break;
        case 3:  out << "3(";    break;
        case 4:  out << "4(";    break;
        default: UnsupportedFunctionality("conversion wrapper");  break;
        }
    }
}

void ConversionStop(std::ostringstream& out, llvm::Type* type)
{
    if (! IsInteger(type))
        return;

    out << ")";
}

// Either convert to (from) an unsigned int or to (from) a matrix
// toIO means converting from an internal variable to an I/O variable, if false, it's the other direction
// integer means doing unsigned/signed conversion, if false, then doing matrix/array conversion
void ConversionWrap(std::string& name, llvm::Type* type, bool toIO)
{
    std::ostringstream wrapped;

    if (IsInteger(type)) {
        ConversionStart(wrapped, type, toIO);
        wrapped << name;
        ConversionStop(wrapped, type);
    } else if (CouldBeMatrix(type)) {
        llvm::ArrayType* array = llvm::dyn_cast<llvm::ArrayType>(type);
        llvm::VectorType* vector = llvm::dyn_cast<llvm::VectorType>(array->getContainedType(0));
        if (toIO)
            wrapped << "mat" << array->getNumElements() << "x" << vector->getNumElements();
        else
            wrapped << "vec" << vector->getNumElements() << "[" << array->getNumElements() << "]";
        wrapped << "(";
        if (toIO)
            wrapped << name;
        else {
            for (int e = 0; e < array->getNumElements(); ++e) {
                if (e > 0)
                    wrapped << ", ";
                wrapped << name << "[" << e << "]";
            }
        }
        wrapped << ")";
    } else
        return;

    name = wrapped.str();
}

//
// Xor is a strange thing:
//  - for scalar Booleans, it looks like "^^"
//  - for vector Booleans, GLSL doesn't have one
//  - for scalar and vector integers, it looks like "^"
//  - if an integer operand is all 1s, it can be represented as unary "~" on the other operand
//  - if a scalar Boolean operand is true, it can be represented as unary "!" on the other operand
//  - if a vector Boolean operand is all true, it can be represented as "not(...)"
//
const char* MapGlaXor(const llvm::Instruction* llvmInstruction, int& unaryOperand)
{
    bool scalar = IsScalar(llvmInstruction->getType());
    bool boolean = IsBoolean(llvmInstruction->getType());

    bool op0AllSet = HasAllSet(llvmInstruction->getOperand(0));
    bool op1AllSet = HasAllSet(llvmInstruction->getOperand(1));

    // first, see if it could be unary

    if (op0AllSet || op1AllSet) {
        // unary; set which operand is the real one to operate on
        if (op0AllSet)
            unaryOperand = 1;
        else
            unaryOperand = 0;

        if (scalar && boolean)
            return "!";

        if (!scalar && boolean)
            return "not";

        if (!boolean)
            return "~";

        UnsupportedFunctionality("xor", EATContinue);

        return "!";
    }

    // now go for binary

    if (scalar && boolean)
        return "^^";

    if (!boolean)
        return "^";

    UnsupportedFunctionality("xor", EATContinue);

    return "^";
}

EVariableQualifier MapGlaAddressSpace(const llvm::Value* value)
{
    if (const llvm::PointerType* pointer = llvm::dyn_cast<llvm::PointerType>(value->getType())) {
        switch (pointer->getAddressSpace()) {
        case ResourceAddressSpace:
            return EVQUniform;
        case GlobalAddressSpace:
            return EVQGlobal;
        default:
            if (pointer->getAddressSpace() >= ConstantAddressSpaceBase)
                return EVQUniform;

            UnsupportedFunctionality("Address Space in Bottom IR: ", pointer->getAddressSpace());
            break;
        }
    }

    if (llvm::isa<llvm::Instruction>(value))
        return EVQTemporary;

    // Check for an undef before a constant (since Undef is a
    // subclass of Constant)
    if (AreAllUndefined(value))
        return EVQUndef;

    if (llvm::isa<llvm::Constant>(value))
        return EVQConstant;

    return EVQTemporary;
}

const char* MapGlaToQualifierString(int version, EShLanguage stage, EVariableQualifier vq)
{
    switch (vq) {
    case EVQUniform:                      return "uniform";

    case EVQInput:
        if (version >= 130)               return "in";
        else if (stage == EShLangVertex)  return "attribute";
        else                              return "varying";

    case EVQOutput:
        if (version >= 130)               return "out";
        else                              return "varying";

    case EVQConstant:                     return "const";

    case EVQNone:
    case EVQGlobal:
    case EVQTemporary:
    case EVQUndef:                        return "";

    default:
        UnsupportedFunctionality("Unknown EVariableQualifier", EATContinue);
        return "";
    }
}

const char* MapGlaToPrecisionString(EMdPrecision precision)
{
    switch (precision) {
    case EMpLow:      return "lowp";
    case EMpMedium:   return "mediump";
    case EMpHigh:     return "highp";
    default:          return "";
    }
}

const char* MapComponentToSwizzleChar(int component)
{
    switch (component) {
    case 0:   return "x";
    case 1:   return "y";
    case 2:   return "z";
    case 3:   return "w";
    default:  assert(! "Vector too large"); break;
    }

    return "x";
}

std::string MapGlaStructField(const llvm::Type* structType, int index, const llvm::MDNode* mdAggregate = 0)
{
    std::string name;

    if (mdAggregate) {
        int aggOp = GetAggregateMdNameOp(index);
        if ((int)mdAggregate->getNumOperands() > aggOp) {
            name = mdAggregate->getOperand(aggOp)->getName();

            return name;
        } else
            UnsupportedFunctionality("missing name in aggregate", EATContinue);
    }

    name.append("member");
            
    const size_t bufSize = 10;
    char buf[bufSize];
    snprintf(buf, bufSize, "%d", index);
    name.append(buf);

    return name;
}

// Write the string representation of an operator. If it's an xor of some
// register and true, then unaryOperand will be set to the index for the
// non-true operand, and s will contain "!".
void GetOp(const llvm::Instruction* llvmInstruction, bool allowBitwise, std::string& s, int& unaryOperand, bool& nested, bool& emulateBitwise)
{
    nested = false;
    emulateBitwise = false;

    //
    // Look for binary ops, where the form would be "operand op operand"
    //

    switch (llvmInstruction->getOpcode()) {
    case llvm::Instruction:: Add:
    case llvm::Instruction::FAdd:
        s = "+";
        break;
    case llvm::Instruction:: Sub:
    case llvm::Instruction::FSub:
        s = "-";
        break;
    case llvm::Instruction:: Mul:
    case llvm::Instruction::FMul:
        s = "*";
        break;
    case llvm::Instruction::UDiv:
    case llvm::Instruction::SDiv:
    case llvm::Instruction::FDiv:
        s = "/";
        break;
    case llvm::Instruction::URem:
    case llvm::Instruction::SRem:
        s = "%";
        break;
    case llvm::Instruction::Shl:
        if (allowBitwise)
            s = "<<";
        else {
            emulateBitwise = true;
            s = "*";
        }
        break;
    case llvm::Instruction::LShr:
    case llvm::Instruction::AShr:
        if (allowBitwise)
            s = ">>";
        else {
            emulateBitwise = true;
            s = "/";
        }
        break;
    case llvm::Instruction::And:
        if (IsBoolean(llvmInstruction->getOperand(0)->getType())) {
            s = "&&";
        } else {
            if (allowBitwise)
                s = "&";
            else {
                emulateBitwise = true;
                s = "-";
            }
        }
        break;
    case llvm::Instruction::Or:
        if (IsBoolean(llvmInstruction->getOperand(0)->getType())) {
            s = "||";
        } else {
            if (allowBitwise)
                s = "|";
            else {
                emulateBitwise = true;
                s = "+";
            }
        }
        break;
    case llvm::Instruction::Xor:
        s = MapGlaXor(llvmInstruction, unaryOperand);
        break;
    case llvm::Instruction::ICmp:
    case llvm::Instruction::FCmp:
        if (! llvm::isa<llvm::VectorType>(llvmInstruction->getOperand(0)->getType())) {

            const llvm::Type* type = llvmInstruction->getOperand(0)->getType();
            if (type != type->getFloatTy(llvmInstruction->getContext()) &&
                type != type->getDoubleTy(llvmInstruction->getContext()) &&
                type != type->getInt32Ty(llvmInstruction->getContext())) {

                UnsupportedFunctionality("Can only compare integers and floats");
                return;
            }

            // Handle float and integer scalars
            // (Vectors are handled as built-in functions)
            if (const llvm::CmpInst* cmp = llvm::dyn_cast<llvm::CmpInst>(llvmInstruction)) {
                switch (cmp->getPredicate()) {
                case llvm::FCmpInst::FCMP_OEQ:
                case llvm::FCmpInst::FCMP_UEQ: // Possibly revise later
                case llvm::ICmpInst::ICMP_EQ:   s = "==";  break;

                case llvm::FCmpInst::FCMP_ONE:
                case llvm::FCmpInst::FCMP_UNE: // Possibly revise later
                case llvm::ICmpInst::ICMP_NE:   s = "!=";  break;

                case llvm::FCmpInst::FCMP_OGT:
                case llvm::FCmpInst::FCMP_UGT: // Possibly revise later
                case llvm::ICmpInst::ICMP_UGT:
                case llvm::ICmpInst::ICMP_SGT:  s = ">";   break;

                case llvm::FCmpInst::FCMP_OGE:
                case llvm::FCmpInst::FCMP_UGE: // Possibly revise laterw
                case llvm::ICmpInst::ICMP_UGE:
                case llvm::ICmpInst::ICMP_SGE:  s = ">=";  break;

                case llvm::FCmpInst::FCMP_OLT:
                case llvm::FCmpInst::FCMP_ULT: // Possibly revise later
                case llvm::ICmpInst::ICMP_ULT:
                case llvm::ICmpInst::ICMP_SLT:  s = "<";   break;

                case llvm::FCmpInst::FCMP_OLE:
                case llvm::FCmpInst::FCMP_ULE: // Possibly revise later
                case llvm::ICmpInst::ICMP_ULE:
                case llvm::ICmpInst::ICMP_SLE:  s = "<=";  break;
                default:
                    s = "==";
                    UnsupportedFunctionality("Comparison Operator in Bottom IR: ", cmp->getPredicate(), EATContinue);
                    break;
                }
            } else {
                assert(! "Cmp instruction found that cannot dyncast to CmpInst");
            }
        }
        break;

    case llvm::Instruction::FPToUI:
        switch (GetComponentCount(llvmInstruction)) {
        case 1: s = "int(uint";  break;
        case 2: s = "ivec2(uvec2"; break;
        case 3: s = "ivec3(uvec3"; break;
        case 4: s = "ivec4(uvec4"; break;
        default: UnsupportedFunctionality("Can only convert scalars and vectors"); break;
        }
        unaryOperand = 0;
        nested = true;
        break;
    case llvm::Instruction::ZExt:
    case llvm::Instruction::FPToSI:
        switch (GetComponentCount(llvmInstruction)) {
        case 1: s = "int";   break;
        case 2: s = "ivec2"; break;
        case 3: s = "ivec3"; break;
        case 4: s = "ivec4"; break;
        default: UnsupportedFunctionality("Can only convert scalars and vectors"); break;
        }
        unaryOperand = 0;
        break;
    case llvm::Instruction::UIToFP:
    case llvm::Instruction::SIToFP:
        switch (GetComponentCount(llvmInstruction)) {
        case 1: s = "float"; break;
        case 2: s = "vec2";  break;
        case 3: s = "vec3";  break;
        case 4: s = "vec4";  break;
        default: UnsupportedFunctionality("Can only convert scalars and vectors"); break;
        }
        unaryOperand = 0;
        break;

    default:
        break;
    }
}

void InvertOp(std::string& op)
{
    if (op == "==")
        op = "!=";
    else if (op == "!=")
        op = "==";
    else if (op == ">")
        op = "<=";
    else if (op == "<=")
        op = ">";
    else if (op == "<")
        op = ">=";
    else if (op == ">=")
        op = "<";
    else
        gla::UnsupportedFunctionality("unknown op to invert", EATContinue);
}

EMdPrecision GetPrecision(const llvm::Value* value)
{
    EMdPrecision precision = EMpNone;

    if (const llvm::Instruction* instr = llvm::dyn_cast<const llvm::Instruction>(value))
        CrackPrecisionMd(instr, precision);

    return precision;
}

bool NeedsShadowRefZArg(const llvm::IntrinsicInst* llvmInstruction)
{
    // Check flags for RefZ
    int texFlags = GetConstantInt(llvmInstruction->getOperand(GetTextureOpIndex(ETOFlag)));

    return (texFlags & ETFRefZArg) != 0;
}

bool NeedsLodArg(const llvm::IntrinsicInst* llvmInstruction)
{
    // Check flags for bias/lod
    int texFlags = GetConstantInt(llvmInstruction->getOperand(GetTextureOpIndex(ETOFlag)));

    return (texFlags & ETFLod) != 0;
}

bool NeedsBiasArg(const llvm::IntrinsicInst* llvmInstruction)
{
    // Check flags for bias/lod
    int texFlags = GetConstantInt(llvmInstruction->getOperand(GetTextureOpIndex(ETOFlag)));

    return (texFlags & ETFBiasLodArg) != 0 && (texFlags & ETFLod) == 0;
}

bool NeedsOffsetArg(const llvm::IntrinsicInst* llvmInstruction, bool& offsets)
{
    // Check flags for offset arg
    int texFlags = GetConstantInt(llvmInstruction->getOperand(GetTextureOpIndex(ETOFlag)));

    offsets = ((texFlags & ETFOffsets) != 0);
    return (texFlags & ETFOffsetArg) != 0;
}

bool NeedsComponentArg(const llvm::IntrinsicInst* llvmInstruction)
{
    // Check flags for component arg
    int texFlags = GetConstantInt(llvmInstruction->getOperand(GetTextureOpIndex(ETOFlag)));

    return (texFlags & ETFComponentArg) != 0;
}

void MakeParseable(std::string& name)
{
    // LLVM uses "." for phi'd symbols, change to _ so it's parseable by GLSL
    // Also, glslang uses @ for an internal name.
    // If the name changes, add a "__goo" so that it's not coincidentally a user name.

    bool changed = false;
    // bool hasDoubleUnderscore = false; // use this once "__" is accepted everywhere
    for (int c = 0; c < (int)name.length(); ++c) {
        if (name[c] == '.' || name[c] == '-' || name[c] == '@') {
            name[c] = '_';
            changed = true;
        }

        if (c > 0 && name[c-1] == '_' && name[c] == '_') {
            // TODO: cleanliness: want to only say:  hasDoubleUnderscore = true;
            // but, for now change things because not all compilers accept "__".
            // Use "_" when possible, because it is much more readable.
            name[c] = 'u';
        }
    }

    // use this once "__" is accepted everywhere
    //if (changed && ! hasDoubleUnderscore)
    //    name.append("_goo");
}

void MakeNonbuiltinName(std::string& name)
{
    // TODO: cleanliness: switch to using "__" when all compilers accept it.

    if (name.compare(0, 3, "gl_") == 0)
        name[0] = 'L';
}

// Whether the given intrinsic's specified operand is the same as the passed
// value, and its type is a vector.
bool IsSameSource(llvm::Value *source, const llvm::IntrinsicInst *inst, int operand)
{
    return (inst->getOperand(operand) == source)
        && (source->getType()->getTypeID() == llvm::Type::VectorTyID);
}

// Returns a pointer to the common source of the multiinsert if they're all
// the same, otherwise returns null.
llvm::Value* GetCommonSourceMultiInsert(const llvm::IntrinsicInst* inst)
{
    llvm::Value* source = NULL;
    bool sameSource = true;
    int wmask = GetConstantInt(inst->getOperand(1));

    for (int i = 0; i < 4; ++i) {
        if (wmask & (1 << i)) {
            int operandIndex = (i+1) * 2;
            if (source)
                sameSource = sameSource && IsSameSource(source, inst, operandIndex);
            else
                source = inst->getOperand(operandIndex);
        }
    }

    return sameSource ? source : NULL;
}

//
// Figure out how many I/O slots 'type' would fill up.
//
int CountSlots(const llvm::Type* type)
{
    if (type->getTypeID() == llvm::Type::VectorTyID)
        return 1;
    else if (type->getTypeID() == llvm::Type::ArrayTyID) {
        const llvm::ArrayType* arrayType = llvm::dyn_cast<const llvm::ArrayType>(type);
        return (int)arrayType->getNumElements() * CountSlots(arrayType->getContainedType(0));
    } else if (type->getTypeID() == llvm::Type::StructTyID) {
        const llvm::StructType* structType = llvm::dyn_cast<const llvm::StructType>(type);
        int slots = 0;
        for (unsigned int f = 0; f < structType->getStructNumElements(); ++f)
            slots += CountSlots(structType->getContainedType(f));

        return slots;
    }

    return 1;
}

//
// *Textually* dereference a name string down to a single I/O slot.
//
void DereferenceName(std::string& name, const llvm::Type* type, const llvm::MDNode* mdAggregate, int slotOffset, EMdTypeLayout& mdTypeLayout)
{
    // Operates recursively...

    if (type->getTypeID() == llvm::Type::PointerTyID) {
        type = type->getContainedType(0);

        DereferenceName(name, type, mdAggregate, slotOffset, mdTypeLayout);
    } else if (type->getTypeID() == llvm::Type::StructTyID) {
        int field = 0;
        int operand;
        const llvm::StructType* structType = llvm::dyn_cast<const llvm::StructType>(type);
        const llvm::Type* fieldType;
        do {
            operand = GetAggregateMdSubAggregateOp(field);
            if (operand >= (int)mdAggregate->getNumOperands()) {
                assert(operand < (int)mdAggregate->getNumOperands());
                return;
            }
            fieldType = structType->getContainedType(field);
            int fieldSize = CountSlots(fieldType);
            if (fieldSize > slotOffset)
                break;
            slotOffset -= fieldSize;
            ++field;
        } while (true);
        if (name.size() > 0)
            name = name + ".";
        name = name + std::string(mdAggregate->getOperand(GetAggregateMdNameOp(field))->getName());
        const llvm::MDNode* subMdAggregate = llvm::dyn_cast<const llvm::MDNode>(mdAggregate->getOperand(operand));
        DereferenceName(name, fieldType, subMdAggregate, slotOffset, mdTypeLayout);
    } else if (type->getTypeID() == llvm::Type::ArrayTyID) {
        const llvm::ArrayType* arrayType = llvm::dyn_cast<const llvm::ArrayType>(type);
        int elementSize = CountSlots(arrayType->getContainedType(0));
        int element = slotOffset / elementSize;
        slotOffset = slotOffset % elementSize;

        char buf[11];
        snprintf(buf, sizeof(buf), "%d", element);
        name = name + "[" + buf + "]";
        
        DereferenceName(name, arrayType->getContainedType(0), mdAggregate, slotOffset, mdTypeLayout);
    } else if (mdAggregate)
        mdTypeLayout = GetMdTypeLayout(mdAggregate);
}

void StripSuffix(std::string& name, const char* suffix)
{
    std::size_t newSize = name.length() - strlen(suffix);
    if (newSize < 0)
        return;

    if (name.compare(newSize, strlen(suffix), suffix) == 0)
        name.resize(newSize);
}

}; // end anonymous namespace






void gla::MetalTarget::buildFullShader()
{
	// Comment line about LunarGOO
	fullShader << "// LunarGOO Metal output" << std::endl;
	
	// Add standard includes that are used in all metal shaders
	fullShader << "#include <metal_stdlib>" << std::endl;
	fullShader << "#include <simd/simd.h>" << std::endl;
	fullShader << "using namespace metal;" << std::endl << std::endl;
	
	// Body of shader
	fullShader << globalStructures.str().c_str() << globalDeclarations.str().c_str() << shader.str().c_str();
}