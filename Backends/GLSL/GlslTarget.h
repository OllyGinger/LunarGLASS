//===- GslsTarget.h - Implementation of Backend.h -------------------------===//
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
//===----------------------------------------------------------------------===//
#pragma once

#include "Core/PrivateManager.h"
#include "GlslTranslator.h"

// glslang includes
#include "glslang/Public/ShaderLang.h"
#include "glslang/MachineIndependent/Versions.h"

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


// LLVM includes
#pragma warning(push, 1)
#include "llvm/IR/Module.h"
#pragma warning(pop)

namespace gla {

    GlslTranslator* GetGlslTranslator(Manager*, bool obfuscate, bool filterInactive = false, int substitutionLevel = 1);
    void ReleaseGlslTranslator(gla::BackEndTranslator*);

    gla::BackEnd* GetGlslBackEnd();
    void ReleaseGlslBackEnd(gla::BackEnd*);

	class GlslTarget;

	class MetaType {
	public:
		MetaType() : precision(gla::EMpNone), matrix(false), notSigned(false), block(false), mdAggregate(0), mdSampler(0) { }
		std::string name;
		gla::EMdPrecision precision;
		bool matrix;
		bool notSigned;
		bool block;
		const llvm::MDNode* mdAggregate;
		const llvm::MDNode* mdSampler;
	};

	class Assignment;

	class GlslTarget : public GlslTranslator{
	public:
		GlslTarget(Manager* m, bool obfuscate, bool filterInactive, int substitutionLevel) :
			GlslTranslator(m, obfuscate, filterInactive, substitutionLevel),
			appendInitializers(false),
			indentLevel(0), lastVariable(0)
		{
#ifdef _WIN32
			unsigned int oldFormat = _set_output_format(_TWO_DIGIT_EXPONENT);
#endif
			indentString = "\t";
		}

		~GlslTarget()
		{
			while (!toDelete.empty()) {
				delete toDelete.back();
				toDelete.pop_back();
			}

			delete generatedShader;
			delete indexShader;

			for (std::map<const llvm::Value*, std::string*>::const_iterator it = nonConvertedMap.begin(); it != nonConvertedMap.end(); ++it)
				delete it->second;
			for (std::map<const llvm::Value*, std::string*>::const_iterator it = valueMap.begin(); it != valueMap.end(); ++it)
				delete it->second;
			for (std::map<const std::string, const std::string*>::const_iterator it = constMap.begin(); it != constMap.end(); ++it)
				delete it->second;
		}

		virtual void start(llvm::Module& module)
		{
			// Call this before doing actual translation.

			// The following information wasn't available at construct time:
			version = manager->getVersion();
			profile = (EProfile)manager->getProfile();
			stage = (EShLanguage)manager->getStage();
			usingSso = version >= 410 || manager->getRequestedExtensions().find("GL_ARB_separate_shader_objects") != manager->getRequestedExtensions().end();

			// Set up noStaticUse() cache
			const llvm::NamedMDNode* mdList = module.getNamedMetadata(NoStaticUseMdName);
			if (mdList) {
				for (unsigned int m = 0; m < mdList->getNumOperands(); ++m)
					noStaticUseSet.insert(mdList->getOperand(m));
			}

			// Get the top-levels modes for this shader.
			int mdInt;
			switch (stage) {
			case EShLangVertex:
				break;

			case EShLangTessControl:
				// output vertices is not optional
				globalStructures << "layout(vertices = " << GetMdNamedInt(module, gla::NumVerticesMdName) << ") out;" << std::endl;
				break;

			case EShLangTessEvaluation:
				// input primitives are not optional
				globalStructures << "layout(";
				switch (GetMdNamedInt(module, gla::InputPrimitiveMdName)) {
				case EMlgTriangles:
					globalStructures << "triangles";
					switch (GetMdNamedInt(module, gla::VertexOrderMdName)) {
					case EMvoNone:
						// ordering is optional
						break;
					case EMvoCw:
						globalStructures << ", cw";
						break;
					case EMvoCcw:
						globalStructures << ", ccw";
						break;
					default:
						UnsupportedFunctionality("tess eval input ordering", EATContinue);
						break;
					}
					break;
				case EMlgQuads:
					globalStructures << "quads";
					break;
				case EMlgIsolines:
					globalStructures << "isolines";
					break;
				default:
					UnsupportedFunctionality("tess eval input primitive", EATContinue);
					break;
				}
				globalStructures << ") in;" << std::endl;

				// vertex spacing is optional
				mdInt = GetMdNamedInt(module, gla::VertexSpacingMdName);
				if (mdInt) {
					globalStructures << "layout(";
					switch (mdInt) {
					case EMvsEqual:
						globalStructures << "equal_spacing";
						break;
					case EMvsFractionalEven:
						globalStructures << "fractional_even_spacing";
						break;
					case EMvsFractionalOdd:
						globalStructures << "fractional_odd_spacing";
						break;
					default:
						UnsupportedFunctionality("tess eval vertex spacing", EATContinue);
						break;
					}
					globalStructures << ") in;" << std::endl;
				}

				if (GetMdNamedInt(module, gla::PointModeMdName))
					globalStructures << "layout(point_mode) in;" << std::endl;
				break;

			case EShLangGeometry:
				// input primitives are not optional
				globalStructures << "layout(";
				switch (GetMdNamedInt(module, gla::InputPrimitiveMdName)) {
				case EMlgPoints:
					globalStructures << "points";
					break;
				case EMlgLines:
					globalStructures << "lines";
					break;
				case EMlgLinesAdjacency:
					globalStructures << "lines_adjacency";
					break;
				case EMlgTriangles:
					globalStructures << "triangles";
					break;
				case EMlgTrianglesAdjacency:
					globalStructures << "triangles_adjacency";
					break;
				default:
					UnsupportedFunctionality("geometry input primitive", EATContinue);
					break;
				}
				globalStructures << ") in;" << std::endl;

				// invocations is optional
				mdInt = GetMdNamedInt(module, gla::InvocationsMdName);
				if (mdInt)
					globalStructures << "layout(invocations = " << mdInt << ") in;" << std::endl;

				// output primitives are not optional
				globalStructures << "layout(";
				switch (GetMdNamedInt(module, gla::OutputPrimitiveMdName)) {
				case EMlgPoints:
					globalStructures << "points";
					break;
				case EMlgLineStrip:
					globalStructures << "line_strip";
					break;
				case EMlgTriangleStrip:
					globalStructures << "triangle_strip";
					break;
				default:
					UnsupportedFunctionality("geometry output primitive", EATContinue);
					break;
				}
				globalStructures << ") out;" << std::endl;

				// max_vertices is not optional
				globalStructures << "layout(max_vertices = " << GetMdNamedInt(module, gla::NumVerticesMdName) << ") out;" << std::endl;
				break;

			case EShLangFragment:
				if (GetMdNamedInt(module, PixelCenterIntegerMdName))
					globalStructures << "layout(pixel_center_integer) in;" << std::endl;

				if (GetMdNamedInt(module, OriginUpperLeftMdName))
					globalStructures << "layout(origin_upper_left) in;" << std::endl;

				break;

			case EShLangCompute:
				break;

			default:
				UnsupportedFunctionality("shader stage", EATContinue);
				break;
			}
		}
		virtual void end(llvm::Module&);

		void addGlobal(const llvm::GlobalVariable* global);
		void addGlobalConst(const llvm::GlobalVariable* global);
		void addIoDeclaration(gla::EVariableQualifier qualifier, const llvm::MDNode* mdNode);
		void startFunctionDeclaration(const llvm::Type* type, llvm::StringRef name);
		void addArgument(const llvm::Value* value, bool last);
		void endFunctionDeclaration();
		void startFunctionBody();
		void endFunctionBody();
		void addInstruction(const llvm::Instruction* llvmInstruction, bool lastBlock, bool referencedOutsideScope = false);
		bool needCanonicalSwap(const llvm::Instruction* instr) const;

		void declarePhiCopy(const llvm::Value* dst);
		void declarePhiAlias(const llvm::Value* dst) { }  // since we will do aliasing, there is no need to declare the intermediate variable
		void addPhiCopy(const llvm::Value* dst, const llvm::Value* src);
		void addPhiAlias(const llvm::Value* dst, const llvm::Value* src);
		void addIf(const llvm::Value* cond, bool invert = false);
		void addElse();
		void addEndif();
		void beginConditionalLoop();
		void beginSimpleConditionalLoop(const llvm::CmpInst* cmp, const llvm::Value* op1, const llvm::Value* op2, bool invert = false);
		void beginForLoop(const llvm::PHINode* phi, llvm::ICmpInst::Predicate, unsigned bound, unsigned increment);
		void beginSimpleInductiveLoop(const llvm::PHINode* phi, const llvm::Value* count);
		void beginInductiveLoop();
		void beginLoop();
		void endLoop();
		void addLoopExit(const llvm::Value* condition = 0, bool invert = false);
		void addLoopBack(const llvm::Value* condition = 0, bool invert = false);
		void addDiscard();
		void print();

		// protected:
		bool filteringIoNode(const llvm::MDNode*);

		void newLine();
		void newScope();
		void leaveScope();

		void addStructType(std::ostringstream& out, std::string& name, const llvm::Type* structType, const llvm::MDNode* mdAggregate, bool block);
		void mapVariableName(const llvm::Value* value, std::string& name);
		void mapExpressionString(const llvm::Value* value, const std::string& name);
		bool getExpressionString(const llvm::Value* value, std::string& name) const;
		bool samplerIsUint(llvm::Value* sampler) const;
		void makeNewVariableName(const llvm::Value* value, std::string& name, const char* rhs);
		void makeNewVariableName(const char* base, std::string& name);
		void makeHashName(const char* prefix, const char* key, std::string& name);
		void makeObfuscatedName(std::string& name);
		void canonicalizeName(std::string& name);
		void makeExtractElementStr(const llvm::Instruction* llvmInstruction, std::string& str);
		void mapPointerExpression(const llvm::Value* ptr, const llvm::Value* additionalToMap = 0);

		void emitGlaIntrinsic(std::ostringstream&, const llvm::IntrinsicInst*);
		void emitGlaCall(std::ostringstream&, const llvm::CallInst*);
		void emitGlaPrecision(std::ostringstream&, EMdPrecision precision);
		void emitComponentCountToSwizzle(std::ostringstream&, int numComponents);
		void emitComponentToSwizzle(std::ostringstream&, int component);
		void emitMaskToSwizzle(std::ostringstream&, int mask);
		void emitGlaSamplerFunction(std::ostringstream&, const llvm::IntrinsicInst* llvmInstruction, int texFlags);
		void emitNamelessConstDeclaration(const llvm::Value*, const llvm::Constant*);
		void emitVariableDeclaration(EMdPrecision precision, llvm::Type* type, const std::string& name, EVariableQualifier qualifier,
			const llvm::Constant* constant = 0, const llvm::MDNode* mdIoNode = 0);
		int emitGlaType(std::ostringstream& out, EMdPrecision precision, EVariableQualifier qualifier, llvm::Type* type,
			bool ioRoot = false, const llvm::MDNode* mdNode = 0, int count = -1, bool araryChild = false);
		bool decodeMdTypesEmitMdQualifiers(std::ostringstream& out, bool ioRoot, const llvm::MDNode* mdNode, llvm::Type*& type, bool arrayChild, MetaType&);
		void emitGlaArraySize(std::ostringstream&, int arraySize);
		void emitGlaSamplerType(std::ostringstream&, const llvm::MDNode* mdSamplerNode);
		void emitGlaInterpolationQualifier(EVariableQualifier qualifier, EInterpolationMethod interpMethod, EInterpolationLocation interpLocation);
		void emitGlaLayout(std::ostringstream&, gla::EMdTypeLayout layout, int location, bool binding);
		void emitGlaConstructor(std::ostringstream&, llvm::Type* type, int count = -1);
		void emitGlaValueDeclaration(const llvm::Value* value, const char* rhs, bool forceGlobal = false);
		void emitGlaValue(std::ostringstream&, const llvm::Value* value, const char* rhs);
		void emitGlaOperand(std::ostringstream&, const llvm::Value* value);
		void emitNonconvertedGlaValue(std::ostringstream&, const llvm::Value* value);
		void propagateNonconvertedGlaValue(const llvm::Value* dst, const llvm::Value* src);
		std::string* mapGlaValueAndEmitDeclaration(const llvm::Value* value);
		void emitFloatConstant(std::ostringstream& out, float f);
		void emitConstantInitializer(std::ostringstream&, const llvm::Constant* constant, llvm::Type* type);
		void emitInitializeAggregate(std::ostringstream&, std::string name, const llvm::Constant* constant);
		void emitGlaSwizzle(std::ostringstream&, int glaSwizzle, int width, llvm::Value* source = 0);
		void emitGlaSwizzle(std::ostringstream&, const llvm::SmallVectorImpl<llvm::Constant*>& elts);
		void emitGlaWriteMask(std::ostringstream&, const llvm::SmallVectorImpl<llvm::Constant*>& elts);
		int getDefinedCount(const llvm::SmallVectorImpl<llvm::Constant*>& elts);
		void emitVectorArguments(std::ostringstream&, bool &firstArg, const llvm::IntrinsicInst *inst, int operand);
		void emitGlaMultiInsertRHS(std::ostringstream& out, const llvm::IntrinsicInst* inst);
		void emitGlaMultiInsert(std::ostringstream& out, const llvm::IntrinsicInst* inst);
		void emitMapGlaIOIntrinsic(const llvm::IntrinsicInst* llvmInstruction, bool input);
		void emitInvariantDeclarations(llvm::Module&);
		virtual void buildFullShader();

		// 'gep' is potentially a gep, either an instruction or a constantExpr.
		// See which one, if any, and return it.
		// Return 0 if not a gep.
		const llvm::GetElementPtrInst* getGepAsInst(const llvm::Value* gep)
		{
			const llvm::GetElementPtrInst* gepInst = llvm::dyn_cast<const llvm::GetElementPtrInst>(gep);
			if (gepInst)
				return gepInst;

			const llvm::ConstantExpr *constantGep = llvm::dyn_cast<const llvm::ConstantExpr>(gep);
			if (constantGep) {
				// seems LLVM's "Instruction *ConstantExpr::getAsInstruction()" is declared wrong that constantGEP can't be const
				llvm::Instruction *instruction = const_cast<llvm::ConstantExpr*>(constantGep)->getAsInstruction();
				toDelete.push_back(instruction);
				gepInst = llvm::dyn_cast<llvm::GetElementPtrInst>(instruction);
			}

			return gepInst;
		}

		std::string traverseGep(const llvm::Instruction* instr, EMdTypeLayout* mdTypeLayout = 0);
		void dereferenceGep(const llvm::Type*& type, std::string& name, llvm::Value* operand, int index, const llvm::MDNode*& mdAggregate, EMdTypeLayout* mdTypeLayout = 0);

		bool cheapExpression(const std::string&);
		bool shouldSubstitute(const llvm::Instruction*);
		bool modifiesPrecision(const llvm::Instruction*);
		bool writeOnceAlloca(const llvm::Value*);
		int getSubstitutionLevel() const { return substitutionLevel; }

		// set of all IO mdNodes in the noStaticUse list
		std::set<const llvm::MDNode*> noStaticUseSet;

		// list of llvm Values to free on exit
		std::vector<llvm::Value*> toDelete;

		// mapping from LLVM values to Glsl variables
		std::map<const llvm::Value*, std::string*> valueMap;

		// mapping of the string representation of a constant's initializer to a const variable name;
		std::map<std::string, const std::string*> constMap;

		// all names that came from hashing, to ensure uniqueness
		std::set<std::string> hashedNames;

		std::map<std::string, int> canonMap;

		// Map from IO-related global variables, by name, to their mdNodes describing them.
		std::map<std::string, const llvm::MDNode*> mdMap;

		// For uint/matrix I/O names that need conversion, preserve the non-converted expression.
		std::map<const llvm::Value*, std::string*> nonConvertedMap;

		// map to track block names tracked in the module
		std::map<const llvm::Type*, std::string> blockNameMap;

		// Map to track structure names tracked in the module.  This could 
		// replicate a block entry if the block's type was reused to declare
		// a shadow variable.
		std::map<const llvm::Type*, std::string> structNameMap;

		// Shadowed variables for a block will have the same type as the block,
		// but the block name cannot be used to declare the shadow and the member
		// names are likely not reusable.  So, use this to track such types
		// so that at the right points, legal GLSL can be produced.
		// Note: these only matter for non-logical IO mode
		std::set<const llvm::Type*> shadowingBlock;

		// map from llvm type to the mdAggregate nodes that describe their types
		std::map<const llvm::Type*, const llvm::MDNode*> typeMdAggregateMap;

		// set to track what globals are already declared;
		// it potentially only includes globals that are in danger of multiple declaration
		std::set<std::string> globallyDeclared;

		std::ostringstream globalStructures;
		std::ostringstream globalDeclarations;
		std::ostringstream globalInitializers;
		std::ostringstream fullShader;
		bool appendInitializers;
		std::ostringstream shader;
		int indentLevel;
		int lastVariable;
		int version;
		EProfile profile;
		EShLanguage stage;
		bool usingSso;
		const char* indentString;
		friend class Assignment;
	};
};
