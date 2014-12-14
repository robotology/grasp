/* TypeSafeBottle.h
 * Copyright (C) 2013, King's College London
 * Authors: Kris De Meyer
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later
 */

#pragma once

#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>
#include <typeinfo>
#include <stdexcept>
#include <string>
#include <sstream>

///File containing functionality to make yarp::os::Bottle type-safe.
/** TypeSafeBottle, the parent class of all other type-safe Bottles
* is implemented as a yarp::os::Bottle, but hides the type-unsafe members from
* the user of derived classes. 
* There are 3 types of TypeSafeBottle: VectorBottle, StructBottle and WrapperBottle.
* VectorBottle and WrapperBottle are implemented using templates. No further code
* needs to be written to instantiate these classes, other than a few typedefs.
* The third type, StructBottle, is a base class for Bottles with named variables 
* and access methods. StructBottle contains a number of protected utility functions to
* make it easier to generate the code for types derived from StructBottle.*/

namespace darwin {
	namespace msg {

class VectorBottleRangeError;
class WrapperBottleIDError;

/// A helper class used to specialize functions in VectorBottle<int>
///(template meta-programming - see Modern C++ Design by Alexandrescu)
template<int v>
struct Int2Type {
	enum { value = v };
};

/// A helper class used to specialize functions in WrapperBottles
///(template meta-programming - see Modern C++ Design by Alexandrescu)
template<typename T,int v>
struct SubID2Type {
	enum { value = v };
	typedef T OriginalType;
};

/// Base class for all TypeSafeBottle formats.
class TypeSafeBottle : public yarp::os::Bottle {
protected:
	TypeSafeBottle(); ///< Protected default constructor.
	TypeSafeBottle(const TypeSafeBottle&); ///< Protected copy constructor.
	TypeSafeBottle(const char*); ///< Protected from-string constructor.
	void addInt(int);
	void addVocab(int);
	void addDouble(double);
	void addString(const char*);
	void addString(const yarp::os::ConstString&);
	void add(const yarp::os::Value&);
	void add(yarp::os::Value* value);
	yarp::os::Bottle& addList();
	yarp::os::Property& addDict();
	yarp::os::Value pop();
	yarp::os::Value& get(int) const;
	void fromString(const char*);
	void fromBinary(const char*,int);
	const char* toBinary(size_t*);
	virtual bool check(const char*);
	virtual yarp::os::Value& find(const char*);
	yarp::os::Bottle& findGroup(const char*);
	virtual bool isNull() const;
	void copy(const yarp::os::Bottle&,int,int);
	void append(const yarp::os::Bottle&);
	yarp::os::Bottle tail() const;
	void hasChanged();
private:
	void specialize(int);
	int getSpecialization();
	void setNested(bool);
};

/// A TypeSafeBottle that behaves as a vector. Typename T can only be a TypeSafeBottle.
template<typename T,bool = false>
class VectorBottle : public TypeSafeBottle {
public:
	/// Indexing operator. Performs range-check.
	T& operator[](int i) const {
		range_check(i);
	    return static_cast<T&>(*get(i).asList());
	}
	/// Add element v to the end of the vector.
	VectorBottle<T>& add(const T& v) {
		Bottle& b = addList();
		b = v;
		hasChanged();
		return *this;
	}
	/// Set index i to a copy of element v. Performs range-check.
	VectorBottle<T>& set(int i,const T& v) {
		range_check(i);
		*get(i).asList() = v;
		hasChanged();
		return *this;
	}
private:
	/// Range-check function.
	void range_check(int i) const {
		if (i >= size()) {
			throw VectorBottleRangeError(typeid(T).name(),i,size());
		}
	}
};

/// Template specialization of VectorBottle for elements of type int.
template<bool isVocab>
class VectorBottle<int,isVocab> : public TypeSafeBottle {
public:
	int operator[](int i) const {
		range_check(i);
	    return index(i,Int2Type<isVocab>());
	}
	VectorBottle<int,isVocab>& add(int v) {
		add(v,Int2Type<isVocab>());
		hasChanged();
		return *this;
	}
	VectorBottle<int,isVocab>& set(int i,int v) {
		range_check(i);
		set(i,v,Int2Type<isVocab>());
		hasChanged();
		return *this;
	}
private:
	int index(int i,const Int2Type<false>&) const {
		return get(i).asInt();
	}
	int index(int i,const Int2Type<true>&) const {
		return get(i).asVocab();
	}
	void add(int v,const Int2Type<false>&) {
		addInt(v);
	}
	void add(int v,const Int2Type<true>&) {
		addVocab(v);
	}
	void set(int i,int v,const Int2Type<false>&) {
		get(i) = yarp::os::Value(v,false);
	}
	void set(int i,int v,const Int2Type<true>&) {
		get(i) = yarp::os::Value(v,true);
	}
	void range_check(int i) const {
		if (i >= size()) {
			throw VectorBottleRangeError("int",i,size());
		}
	}
};

/// Template specialization of VectorBottle for elements of type double.
template<>
class VectorBottle<double,false> : public TypeSafeBottle {
public:
	double operator[](int i) const;
	VectorBottle<double>& add(double v); 
	VectorBottle<double>& set(int i,double v);
private:
	void range_check(int i) const;
};

/// Template specialization of VectorBottle for elements of type yarp::os::ConstString.
template<>
class VectorBottle<yarp::os::ConstString,false> : public TypeSafeBottle {
public:
	yarp::os::ConstString operator[](int i) const;
	VectorBottle<yarp::os::ConstString>& add(const yarp::os::ConstString& v); 
	VectorBottle<yarp::os::ConstString>& set(int i,const yarp::os::ConstString& v);
private:
	void range_check(int i) const;
};

typedef VectorBottle<int,false> IntVector;
typedef VectorBottle<double> DoubleVector;
typedef VectorBottle<yarp::os::ConstString> StringVector;
typedef VectorBottle<int,true> VocabVector;

/// Base class for a TypeSafeBottle that behaves like a struct with named variables and vectors.
class StructBottle : public TypeSafeBottle {
protected:
	StructBottle(); ///< Protected default constructor.
	StructBottle(const StructBottle&); ///< Protected copy constructor.
	void addint(const char* n,int v); ///< Utility function for derived classes
	void adddouble(const char* n,double v); ///< Utility function for derived classes
	void addstring(const char* n,const char* v); ///< Utility function for derived classes
	void addvocab(const char* n,int v); ///< Utility function for derived classes
	void addlist(const char* n); ///< Utility function for derived classes
	int getint(int i); ///< Utility function for derived classes
	double getdouble(int i); ///< Utility function for derived classes
	yarp::os::ConstString getstring(int i); ///< Utility function for derived classes
	int getvocab(int i); ///< Utility function for derived classes
	Bottle& getlist(int i); ///< Utility function for derived classes
	void setint(int i,int v); ///< Utility function for derived classes
	void setdouble(int i,double v); ///< Utility function for derived classes
	void setstring(int i,const yarp::os::ConstString& v); ///< Utility function for derived classes
	void setvocab(int i,int v); ///< Utility function for derived classes
	int size() const; ///< User has no access to yarp::os::Bottle::size()
	void clear(); ///< User has no access to yarp::os::Bottle::clear()
};

/// A template for TypeSafeBottles that wrap around other TypeSafeBottles 
/// for transmission over the same yarp::os::Port or yarp::os::BufferedPort
class WrapperBottle : public TypeSafeBottle {
public:
	int subID();
	yarp::os::ConstString subIDStr();
protected:
    template<typename U,int v> void set(const U&,const SubID2Type<U,v>&); ///< Set the contained TypeSafeBottle
    template<typename U,int v> U& get(const SubID2Type<U,v>&); ///< Get the contained TypeSafeBottle
	int size() const;
	void clear();
};

///An out-of-range error thrown when trying to access a VectorBottle with an index that is too large.
class VectorBottleRangeError : public std::range_error {
public:
	VectorBottleRangeError(const char*,int,int); ///< Message containing typeid of throwing class, index value and size of VectorBottle.
private:
	std::string createstr(const char*,int,int);
};

///An error thrown the user tries to read the wrong type of TypeSafeBottle out of a WrapperBottle
class WrapperBottleIDError : public std::range_error {
public:
	WrapperBottleIDError(const char*,const char*); ///< Typeid of the throwing class, expected and actual subID.
private:
	std::string createstr(const char*,const char*);
};

/*=====================================
* Class member function implementations
======================================*/

/*==============================
* TypeSafeBottle implementations
===============================*/

/// Protected default constructor, only allow object creation of derived classes
inline TypeSafeBottle::TypeSafeBottle() : Bottle() {

}

/// Protected copy constructor, only allow object copy of derived classes
inline TypeSafeBottle::TypeSafeBottle(const TypeSafeBottle& b) : Bottle(b) {

}

/// Proteced from-string constructor.
inline TypeSafeBottle::TypeSafeBottle(const char* t) : Bottle(t) {

}

inline void TypeSafeBottle::addInt(int v) {
	yarp::os::Bottle::addInt(v);
}
inline void TypeSafeBottle::addVocab(int v) {
	yarp::os::Bottle::addVocab(v);
}

inline void TypeSafeBottle::addDouble(double v) {
	yarp::os::Bottle::addDouble(v);
}

inline void TypeSafeBottle::addString(const char* v) {
	yarp::os::Bottle::addString(v);

}

inline void TypeSafeBottle::addString(const yarp::os::ConstString& v) {
	yarp::os::Bottle::addString(v);

}

inline void TypeSafeBottle::add(const yarp::os::Value& v) {
	yarp::os::Bottle::add(v);
}

inline void TypeSafeBottle::add(yarp::os::Value* v) {
	yarp::os::Bottle::add(v);
}

inline yarp::os::Bottle& TypeSafeBottle::addList() {
	return yarp::os::Bottle::addList();
}

inline yarp::os::Property& TypeSafeBottle::addDict() {
	return yarp::os::Bottle::addDict();
}

inline yarp::os::Value TypeSafeBottle::pop() {
	return yarp::os::Bottle::pop();
}

inline yarp::os::Value& TypeSafeBottle::get(int i) const {
	return yarp::os::Bottle::get(i);
}

inline void TypeSafeBottle::fromString(const char* t) {
	yarp::os::Bottle::fromString(t);
}

inline void TypeSafeBottle::fromBinary(const char* t,int i) {
	yarp::os::Bottle::fromBinary(t,i);
}

inline const char* TypeSafeBottle::toBinary(size_t* t) {
	return yarp::os::Bottle::toBinary(t);
}

inline bool TypeSafeBottle::check(const char* t) {
	return yarp::os::Bottle::check(t);
}

inline yarp::os::Value& TypeSafeBottle::find(const char* t) {
	return yarp::os::Bottle::find(t);
}

inline yarp::os::Bottle& TypeSafeBottle::findGroup(const char* t) {
	return yarp::os::Bottle::findGroup(t);
}

inline bool TypeSafeBottle::isNull() const {
	return yarp::os::Bottle::isNull();
}

inline void TypeSafeBottle::copy(const yarp::os::Bottle& b,int i,int l) {
	yarp::os::Bottle::copy(b,i,l);
}

inline void TypeSafeBottle::append(const yarp::os::Bottle& b) {
	yarp::os::Bottle::append(b);
}

inline yarp::os::Bottle TypeSafeBottle::tail() const {
	return yarp::os::Bottle::tail();
}

inline void TypeSafeBottle::hasChanged() {
	yarp::os::Bottle::hasChanged();
}


/*============================
* VectorBottle implementations
=============================*/

/// Indexing operator for VectorBottle of doubles
inline double VectorBottle<double,false>::operator[](int i) const { 
	range_check(i);
	return get(i).asDouble();
}

/// Add a double to the end of a VectorBottle
inline VectorBottle<double,false>& VectorBottle<double,false>::add(double v) {
	addDouble(v);
	hasChanged();
	return *this;
}

/// Set the element at index i to double value v
inline VectorBottle<double,false>& VectorBottle<double,false>::set(int i,double v) {
	range_check(i);
	get(i) = yarp::os::Value(v);
	hasChanged();
	return *this;
}

/// Indexing operator for VectorBottle of yarp::os::ConstString
inline yarp::os::ConstString VectorBottle<yarp::os::ConstString,false>::operator[](int i) const { 
	range_check(i);
	return get(i).asString();
}

/// Add a yarp::os::ConstString to the end of a VectorBottle
inline VectorBottle<yarp::os::ConstString,false>& VectorBottle<yarp::os::ConstString,false>::add(const yarp::os::ConstString& v) {
	addString(v);
	hasChanged();
	return *this;
}

/// Copy the content of yarp::os::ConstString v to the element at index i
inline VectorBottle<yarp::os::ConstString,false>& VectorBottle<yarp::os::ConstString,false>::set(int i,const yarp::os::ConstString& v) {
	range_check(i);
	get(i) = yarp::os::Value(v);
	hasChanged();
	return *this;
}

/// Test if index i is within the bounds of the VectorBottle, if not throw an out-of-range error
inline void VectorBottle<double>::range_check(int i) const {
	if (i >= size()) {
		throw VectorBottleRangeError("double",i,size());
	}
}

/// Test if index i is within the bounds of the VectorBottle, if not throw an out-of-range error
inline void VectorBottle<yarp::os::ConstString>::range_check(int i) const {
	if (i >= size()) {
		throw VectorBottleRangeError("yarp::os::ConstString",i,size());
	}
}

/*============================
* StructBottle implementations
=============================*/

/// Protected default constructor, only allow object creation of derived classes
inline StructBottle::StructBottle() : TypeSafeBottle() {

}

/// Protected copy constructor, only allow object copy of derived classes
inline StructBottle::StructBottle(const StructBottle& s) : TypeSafeBottle(s) {

}

/// Add an integer variable with name n and default value v
inline void StructBottle::addint(const char* n,int v) {
	addString(n); 
	addInt(v);
	hasChanged();
}

/// Add a double variable with name n and default value v
inline void StructBottle::adddouble(const char* n,double v) {
	addString(n); 
	addDouble(v);
	hasChanged();
}

/// Add a string variable with name n and default value v
inline void StructBottle::addstring(const char* n,const char* v) {
	addString(n); 
	addString(v);
	hasChanged();
}

/// Add an integer variable with name n and default value v
inline void StructBottle::addvocab(const char* n,int v) {
	addString(n); 
	addVocab(v);
	hasChanged();
}

/// Add an empty vector variable with name n
inline void StructBottle::addlist(const char* n) {
	addString(n); 
	addList();
	hasChanged();
}

/// Get the value of an int variable at position i in the list
inline int StructBottle::getint(int i) {
	return get(i).asInt();
}

/// Get the value of a double variable at position i in the list
inline double StructBottle::getdouble(int i) {
	return get(i).asDouble();
}

/// Get the value of a yarp::os::ConstString variable at position i in the list
inline yarp::os::ConstString StructBottle::getstring(int i) {
	return get(i).asString();
}

/// Get the value of an int variable at position i in the list
inline int StructBottle::getvocab(int i) {
	return get(i).asVocab();
}

/// Get a reference to a vector variable at position i in the list			
inline yarp::os::Bottle& StructBottle::getlist(int i) {
	return *get(i).asList();
}

/// Set the int variable at position i to value v
inline void StructBottle::setint(int i,int v) {
	get(i) = yarp::os::Value(v);
	hasChanged();
}

/// Set the double variable at position i to value v
inline void StructBottle::setdouble(int i,double v) {
	get(i) = yarp::os::Value(v);
	hasChanged();
}

/// Copy the content of yarp::os::ConstString v to the variable at position i
inline void StructBottle::setstring(int i,const yarp::os::ConstString& v) {
	get(i) = yarp::os::Value(v);
	hasChanged();
}

/// Set the int variable at position i to value v
inline void StructBottle::setvocab(int i,int v) {
	get(i) = yarp::os::Value(v,true);
	hasChanged();
}

/*=============================
* WrapperBottle implementations
==============================*/

/// Implementation of Bottle::clear function, for in-class use only
inline void WrapperBottle::clear() {
	yarp::os::Bottle::clear();
}

/// Get the int value of the Vocab subID specifier of 
inline int WrapperBottle::subID() {
	return yarp::os::Bottle::get(0).asVocab();
}

/// Get the string representation of the Vocab subID
inline yarp::os::ConstString WrapperBottle::subIDStr() {
	return yarp::os::Bottle::get(0).toString();
}

/// Template function to set a TypeSafeBottle using its SUBID
/** For each subID, SUBID defines a type, meaning that 
*any use of these template functions with a specific SUBID
*type specifier gives rise to an overloaded function,
*which forces the compiler to generate code for that type.
*/
template<typename U,int v> void WrapperBottle::set(const U& u,const SubID2Type<U,v>& subid) {
	clear();
	addVocab(subid.value);
	addList() = u;
	hasChanged();
}

/// Template function to get a TypeSafeBottle using its SUBID
/** For each subID, SUBID defines a type, meaning that 
*any use of these template functions with a specific SUBID
*type specifier gives rise to an overloaded function,
*which forces the compiler to generate code for that type.
*This function will throw an WrapperBottleIDError exception
*if you try to read a Bottle format with a different subID
*than the one that is currently wrapped.
*/
template<typename U,int v> U& WrapperBottle::get(const SubID2Type<U,v>& subid) {
	if (subID() != subid.value) {
		throw WrapperBottleIDError(typeid(U).name(),subIDStr().c_str());
	}
	return static_cast<U&>(*(yarp::os::Bottle::get(1).asList()));
}

/*======================================
* VectorBottleRangeError implementations
========================================*/

inline VectorBottleRangeError::VectorBottleRangeError(const char* type,int index,int size) 
	: std::range_error(createstr(type,index,size)) {

}
		
inline std::string VectorBottleRangeError::createstr(const char* type,int index,int size) {
	std::ostringstream conv;
	conv << "VectorBottleRangeError<" << type << ">: index out of bounds" << std::endl; 
	conv << "Index=" << index << ", Size of VectorBottle=" << size << std::endl;
	return conv.str();
}

/*======================================
* WrapperBottleIDError implementations
========================================*/

inline WrapperBottleIDError::WrapperBottleIDError(const char* type,const char* expected) 
	: std::range_error(createstr(type,expected)) {

}
		
inline std::string WrapperBottleIDError::createstr(const char* type,const char* expected) {
	std::ostringstream conv;
	conv << "WrapperBottleIDError<" << type << ">:" << std::endl; 
	conv << "Trying to access object with wrong subID, should be=\"" << expected << "\"" << std::endl;
	return conv.str();
}


} //end namespace msg

} //end namespace darwin



