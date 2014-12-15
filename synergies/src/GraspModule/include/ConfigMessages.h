//C++ code automatically generated from YarpTypeSafeBottle XML file - do not alter

#pragma once

#include "TypeSafeBottle/TypeSafeBottle.h"

using namespace darwin::msg;

namespace darwin {
	namespace grasp {

//pre-declaration of classes derived from StructBottle
class ConnectStructure;
class PumaDisplacementSuccess;
class PumaDisplacementParameters;
class PumaStopParameters;
class PumaConfigParameters;
class IcubObjectData;
class IcubTouch;
class IcubAngles;
class IcubConfigParameters;

//pre-declaration of classes derived from WrapperBottle

//named VectorBottles - implemented as typedefs
typedef StringVector ConnectField;
typedef DoubleVector IcubJointRow;
typedef VectorBottle<IcubJointRow> IcubJointMatrix;

//class declarations derived from StructBottle
class ConnectStructure : public StructBottle {
public:
	ConnectStructure();
	ConnectField& gripin();
	ConnectField& gripout();
	ConnectField& command();
	ConnectField& outcome();
	ConnectField& visualization();
};

class PumaDisplacementSuccess : public StructBottle {
public:
	PumaDisplacementSuccess();
	double low();
	double high();
	PumaDisplacementSuccess& setlow(double);
	PumaDisplacementSuccess& sethigh(double);
};

class PumaDisplacementParameters : public StructBottle {
public:
	PumaDisplacementParameters();
	PumaDisplacementSuccess& success();
	double max();
	PumaDisplacementParameters& setmax(double);
};

class PumaStopParameters : public StructBottle {
public:
	PumaStopParameters();
	double resolution();
	int steps();
	PumaStopParameters& setresolution(double);
	PumaStopParameters& setsteps(int);
};

class PumaConfigParameters : public StructBottle {
public:
	PumaConfigParameters();
	PumaStopParameters& stop();
	PumaDisplacementParameters& disp();
};

class IcubObjectData : public StructBottle {
public:
	IcubObjectData();
	yarp::os::ConstString thumb();
	int isenveloping();
	IntVector& mask();
	int min_blocked();
	int jskipped();
	IcubJointMatrix& angles();
	IcubJointMatrix& derivatives();
	IcubObjectData& setthumb(const yarp::os::ConstString&);
	IcubObjectData& setisenveloping(int);
	IcubObjectData& setmin_blocked(int);
	IcubObjectData& setjskipped(int);
};

class IcubTouch : public StructBottle {
public:
	IcubTouch();
	int taxels();
	double fing_threas();
	double perc_active_fing();
	IcubTouch& settaxels(int);
	IcubTouch& setfing_threas(double);
	IcubTouch& setperc_active_fing(double);
};

class IcubAngles : public StructBottle {
public:
	IcubAngles();
	double increment();
	double degrees_tol();
	double pausing_pre();
	double pausing_env();
	int counterMax();
	IcubJointRow& defVels();
	IcubAngles& setincrement(double);
	IcubAngles& setdegrees_tol(double);
	IcubAngles& setpausing_pre(double);
	IcubAngles& setpausing_env(double);
	IcubAngles& setcounterMax(int);
};

class IcubConfigParameters : public StructBottle {
public:
	IcubConfigParameters();
	int primitives();
	int ispreshape();
	int restored_joints();
	double jthreashold();
	int istimedValidation();
	int wait();
	int ischeckAdAb();
	IcubJointRow& released();
	IcubJointRow& fist();
	IcubTouch& touch();
	IcubAngles& angles();
	IcubConfigParameters& setprimitives(int);
	IcubConfigParameters& setispreshape(int);
	IcubConfigParameters& setrestored_joints(int);
	IcubConfigParameters& setjthreashold(double);
	IcubConfigParameters& setistimedValidation(int);
	IcubConfigParameters& setwait(int);
	IcubConfigParameters& setischeckAdAb(int);
};


//class declarations derived from WrapperBottle

//Function definitions
/*========================================
* ConnectStructure member function implementations
*=======================================*/

inline ConnectStructure::ConnectStructure() {
	addlist("gripin"); gripin() = ConnectField();
	addlist("gripout"); gripout() = ConnectField();
	addlist("command"); command() = ConnectField();
	addlist("outcome"); outcome() = ConnectField();
	addlist("visualization"); visualization() = ConnectField();
}

inline ConnectField& ConnectStructure::gripin() {
	return static_cast<ConnectField&>(getlist(1));
}

inline ConnectField& ConnectStructure::gripout() {
	return static_cast<ConnectField&>(getlist(3));
}

inline ConnectField& ConnectStructure::command() {
	return static_cast<ConnectField&>(getlist(5));
}

inline ConnectField& ConnectStructure::outcome() {
	return static_cast<ConnectField&>(getlist(7));
}

inline ConnectField& ConnectStructure::visualization() {
	return static_cast<ConnectField&>(getlist(9));
}

/*========================================
* PumaDisplacementSuccess member function implementations
*=======================================*/

inline PumaDisplacementSuccess::PumaDisplacementSuccess() {
	adddouble("low",0.0);
	adddouble("high",0.0);
}

inline double PumaDisplacementSuccess::low() {
	return getdouble(1);
}

inline double PumaDisplacementSuccess::high() {
	return getdouble(3);
}

inline PumaDisplacementSuccess& PumaDisplacementSuccess::setlow(double v) {
	setdouble(1,v);
	return *this;
}

inline PumaDisplacementSuccess& PumaDisplacementSuccess::sethigh(double v) {
	setdouble(3,v);
	return *this;
}

/*========================================
* PumaDisplacementParameters member function implementations
*=======================================*/

inline PumaDisplacementParameters::PumaDisplacementParameters() {
	addlist("success"); success() = PumaDisplacementSuccess();
	adddouble("max",0.0);
}

inline PumaDisplacementSuccess& PumaDisplacementParameters::success() {
	return static_cast<PumaDisplacementSuccess&>(getlist(1));
}

inline double PumaDisplacementParameters::max() {
	return getdouble(3);
}

inline PumaDisplacementParameters& PumaDisplacementParameters::setmax(double v) {
	setdouble(3,v);
	return *this;
}

/*========================================
* PumaStopParameters member function implementations
*=======================================*/

inline PumaStopParameters::PumaStopParameters() {
	adddouble("resolution",0.0);
	addint("steps",0);
}

inline double PumaStopParameters::resolution() {
	return getdouble(1);
}

inline int PumaStopParameters::steps() {
	return getint(3);
}

inline PumaStopParameters& PumaStopParameters::setresolution(double v) {
	setdouble(1,v);
	return *this;
}

inline PumaStopParameters& PumaStopParameters::setsteps(int v) {
	setint(3,v);
	return *this;
}

/*========================================
* PumaConfigParameters member function implementations
*=======================================*/

inline PumaConfigParameters::PumaConfigParameters() {
	addlist("stop"); stop() = PumaStopParameters();
	addlist("disp"); disp() = PumaDisplacementParameters();
}

inline PumaStopParameters& PumaConfigParameters::stop() {
	return static_cast<PumaStopParameters&>(getlist(1));
}

inline PumaDisplacementParameters& PumaConfigParameters::disp() {
	return static_cast<PumaDisplacementParameters&>(getlist(3));
}

/*========================================
* IcubObjectData member function implementations
*=======================================*/

inline IcubObjectData::IcubObjectData() {
	addstring("thumb","");
	addint("isenveloping",0);
	addlist("mask"); mask() = IntVector();
	addint("min_blocked",0);
	addint("jskipped",0);
	addlist("angles"); angles() = IcubJointMatrix();
	addlist("derivatives"); derivatives() = IcubJointMatrix();
}

inline yarp::os::ConstString IcubObjectData::thumb() {
	return getstring(1);
}

inline int IcubObjectData::isenveloping() {
	return getint(3);
}

inline IntVector& IcubObjectData::mask() {
	return static_cast<IntVector&>(getlist(5));
}

inline int IcubObjectData::min_blocked() {
	return getint(7);
}

inline int IcubObjectData::jskipped() {
	return getint(9);
}

inline IcubJointMatrix& IcubObjectData::angles() {
	return static_cast<IcubJointMatrix&>(getlist(11));
}

inline IcubJointMatrix& IcubObjectData::derivatives() {
	return static_cast<IcubJointMatrix&>(getlist(13));
}

inline IcubObjectData& IcubObjectData::setthumb(const yarp::os::ConstString& v) {
	setstring(1,v);
	return *this;
}

inline IcubObjectData& IcubObjectData::setisenveloping(int v) {
	setint(3,v);
	return *this;
}

inline IcubObjectData& IcubObjectData::setmin_blocked(int v) {
	setint(7,v);
	return *this;
}

inline IcubObjectData& IcubObjectData::setjskipped(int v) {
	setint(9,v);
	return *this;
}

/*========================================
* IcubTouch member function implementations
*=======================================*/

inline IcubTouch::IcubTouch() {
	addint("taxels",0);
	adddouble("fing_threas",0.0);
	adddouble("perc_active_fing",0.0);
}

inline int IcubTouch::taxels() {
	return getint(1);
}

inline double IcubTouch::fing_threas() {
	return getdouble(3);
}

inline double IcubTouch::perc_active_fing() {
	return getdouble(5);
}

inline IcubTouch& IcubTouch::settaxels(int v) {
	setint(1,v);
	return *this;
}

inline IcubTouch& IcubTouch::setfing_threas(double v) {
	setdouble(3,v);
	return *this;
}

inline IcubTouch& IcubTouch::setperc_active_fing(double v) {
	setdouble(5,v);
	return *this;
}

/*========================================
* IcubAngles member function implementations
*=======================================*/

inline IcubAngles::IcubAngles() {
	adddouble("increment",0.0);
	adddouble("degrees_tol",0.0);
	adddouble("pausing_pre",0.0);
	adddouble("pausing_env",0.0);
	addint("counterMax",0);
	addlist("defVels"); defVels() = IcubJointRow();
}

inline double IcubAngles::increment() {
	return getdouble(1);
}

inline double IcubAngles::degrees_tol() {
	return getdouble(3);
}

inline double IcubAngles::pausing_pre() {
	return getdouble(5);
}

inline double IcubAngles::pausing_env() {
	return getdouble(7);
}

inline int IcubAngles::counterMax() {
	return getint(9);
}

inline IcubJointRow& IcubAngles::defVels() {
	return static_cast<IcubJointRow&>(getlist(11));
}

inline IcubAngles& IcubAngles::setincrement(double v) {
	setdouble(1,v);
	return *this;
}

inline IcubAngles& IcubAngles::setdegrees_tol(double v) {
	setdouble(3,v);
	return *this;
}

inline IcubAngles& IcubAngles::setpausing_pre(double v) {
	setdouble(5,v);
	return *this;
}

inline IcubAngles& IcubAngles::setpausing_env(double v) {
	setdouble(7,v);
	return *this;
}

inline IcubAngles& IcubAngles::setcounterMax(int v) {
	setint(9,v);
	return *this;
}

/*========================================
* IcubConfigParameters member function implementations
*=======================================*/

inline IcubConfigParameters::IcubConfigParameters() {
	addint("primitives",0);
	addint("ispreshape",0);
	addint("restored_joints",0);
	adddouble("jthreashold",0.0);
	addint("istimedValidation",0);
	addint("wait",0);
	addint("ischeckAdAb",0);
	addlist("released"); released() = IcubJointRow();
	addlist("fist"); fist() = IcubJointRow();
	addlist("touch"); touch() = IcubTouch();
	addlist("angles"); angles() = IcubAngles();
}

inline int IcubConfigParameters::primitives() {
	return getint(1);
}

inline int IcubConfigParameters::ispreshape() {
	return getint(3);
}

inline int IcubConfigParameters::restored_joints() {
	return getint(5);
}

inline double IcubConfigParameters::jthreashold() {
	return getdouble(7);
}

inline int IcubConfigParameters::istimedValidation() {
	return getint(9);
}

inline int IcubConfigParameters::wait() {
	return getint(11);
}

inline int IcubConfigParameters::ischeckAdAb() {
	return getint(13);
}

inline IcubJointRow& IcubConfigParameters::released() {
	return static_cast<IcubJointRow&>(getlist(15));
}

inline IcubJointRow& IcubConfigParameters::fist() {
	return static_cast<IcubJointRow&>(getlist(17));
}

inline IcubTouch& IcubConfigParameters::touch() {
	return static_cast<IcubTouch&>(getlist(19));
}

inline IcubAngles& IcubConfigParameters::angles() {
	return static_cast<IcubAngles&>(getlist(21));
}

inline IcubConfigParameters& IcubConfigParameters::setprimitives(int v) {
	setint(1,v);
	return *this;
}

inline IcubConfigParameters& IcubConfigParameters::setispreshape(int v) {
	setint(3,v);
	return *this;
}

inline IcubConfigParameters& IcubConfigParameters::setrestored_joints(int v) {
	setint(5,v);
	return *this;
}

inline IcubConfigParameters& IcubConfigParameters::setjthreashold(double v) {
	setdouble(7,v);
	return *this;
}

inline IcubConfigParameters& IcubConfigParameters::setistimedValidation(int v) {
	setint(9,v);
	return *this;
}

inline IcubConfigParameters& IcubConfigParameters::setwait(int v) {
	setint(11,v);
	return *this;
}

inline IcubConfigParameters& IcubConfigParameters::setischeckAdAb(int v) {
	setint(13,v);
	return *this;
}


  } //end namespace grasp
} //end namespace darwin

