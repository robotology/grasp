//C++ code automatically generated from YarpTypeSafeBottle XML file - do not alter

#pragma once

#include "TypeSafeBottle/TypeSafeBottle.h"

using namespace darwin::msg;

namespace darwin {
	namespace msg {

//pre-declaration of classes derived from StructBottle
class Point2D;
class Point3D;
class CommandInterface;
class Coordinates3D;
class Cylinder1;
class Cylinder2;
class OrientedCircle;
class GraspDescriptor;

//pre-declaration of classes derived from WrapperBottle
class HandCommand;

//named VectorBottles - implemented as typedefs
typedef DoubleVector HandICub;
typedef VectorBottle<GraspDescriptor> GraspCommand;
typedef VocabVector GraspResult;

//class declarations derived from StructBottle
class Point2D : public StructBottle {
public:
	Point2D();
	double x();
	double y();
	Point2D& setx(double);
	Point2D& sety(double);
};

class Point3D : public StructBottle {
public:
	Point3D();
	double x();
	double y();
	double z();
	Point3D& setx(double);
	Point3D& sety(double);
	Point3D& setz(double);
};

class CommandInterface : public StructBottle {
public:
	CommandInterface();
	int command();
	CommandInterface& setcommand(int);
};

class Coordinates3D : public StructBottle {
public:
	Coordinates3D();
	double x();
	double y();
	double z();
	Coordinates3D& setx(double);
	Coordinates3D& sety(double);
	Coordinates3D& setz(double);
};

class Cylinder1 : public StructBottle {
public:
	Cylinder1();
	Coordinates3D& P1();
	Coordinates3D& P2();
	double Radius();
	Cylinder1& setRadius(double);
};

class Cylinder2 : public StructBottle {
public:
	Cylinder2();
	Coordinates3D& Center();
	Coordinates3D& Normal();
	double Radius();
	double Height();
	Cylinder2& setRadius(double);
	Cylinder2& setHeight(double);
};

class OrientedCircle : public StructBottle {
public:
	OrientedCircle();
	Coordinates3D& Center();
	Coordinates3D& Normal();
	double Radius();
	OrientedCircle& setRadius(double);
};

class GraspDescriptor : public StructBottle {
public:
	GraspDescriptor();
	int effector();
	int type();
	Cylinder1& object();
	GraspDescriptor& seteffector(int);
	GraspDescriptor& settype(int);
};


//class declarations derived from WrapperBottle
class HandCommand : public WrapperBottle {
public:
	HandCommand();
	HandICub& lhan();
	HandICub& rhan();
	void lhan(const HandICub&);
	void rhan(const HandICub&);
	bool islhan();
	bool isrhan();
	typedef SubID2Type<HandICub,VOCAB4('l','h','a','n')> lhanID;
	typedef SubID2Type<HandICub,VOCAB4('r','h','a','n')> rhanID;
};


//Function definitions
/*========================================
* Point2D member function implementations
*=======================================*/

inline Point2D::Point2D() {
	adddouble("x",0.0);
	adddouble("y",0.0);
}

inline double Point2D::x() {
	return getdouble(1);
}

inline double Point2D::y() {
	return getdouble(3);
}

inline Point2D& Point2D::setx(double v) {
	setdouble(1,v);
	return *this;
}

inline Point2D& Point2D::sety(double v) {
	setdouble(3,v);
	return *this;
}

/*========================================
* Point3D member function implementations
*=======================================*/

inline Point3D::Point3D() {
	adddouble("x",0.0);
	adddouble("y",0.0);
	adddouble("z",0.0);
}

inline double Point3D::x() {
	return getdouble(1);
}

inline double Point3D::y() {
	return getdouble(3);
}

inline double Point3D::z() {
	return getdouble(5);
}

inline Point3D& Point3D::setx(double v) {
	setdouble(1,v);
	return *this;
}

inline Point3D& Point3D::sety(double v) {
	setdouble(3,v);
	return *this;
}

inline Point3D& Point3D::setz(double v) {
	setdouble(5,v);
	return *this;
}

/*========================================
* CommandInterface member function implementations
*=======================================*/

inline CommandInterface::CommandInterface() {
	addvocab("command",0);
}

inline int CommandInterface::command() {
	return getvocab(1);
}

inline CommandInterface& CommandInterface::setcommand(int v) {
	setvocab(1,v);
	return *this;
}

/*========================================
* Coordinates3D member function implementations
*=======================================*/

inline Coordinates3D::Coordinates3D() {
	adddouble("x",0.0);
	adddouble("y",0.0);
	adddouble("z",0.0);
}

inline double Coordinates3D::x() {
	return getdouble(1);
}

inline double Coordinates3D::y() {
	return getdouble(3);
}

inline double Coordinates3D::z() {
	return getdouble(5);
}

inline Coordinates3D& Coordinates3D::setx(double v) {
	setdouble(1,v);
	return *this;
}

inline Coordinates3D& Coordinates3D::sety(double v) {
	setdouble(3,v);
	return *this;
}

inline Coordinates3D& Coordinates3D::setz(double v) {
	setdouble(5,v);
	return *this;
}

/*========================================
* Cylinder1 member function implementations
*=======================================*/

inline Cylinder1::Cylinder1() {
	addlist("P1"); P1() = Coordinates3D();
	addlist("P2"); P2() = Coordinates3D();
	adddouble("Radius",0.0);
}

inline Coordinates3D& Cylinder1::P1() {
	return static_cast<Coordinates3D&>(getlist(1));
}

inline Coordinates3D& Cylinder1::P2() {
	return static_cast<Coordinates3D&>(getlist(3));
}

inline double Cylinder1::Radius() {
	return getdouble(5);
}

inline Cylinder1& Cylinder1::setRadius(double v) {
	setdouble(5,v);
	return *this;
}

/*========================================
* Cylinder2 member function implementations
*=======================================*/

inline Cylinder2::Cylinder2() {
	addlist("Center"); Center() = Coordinates3D();
	addlist("Normal"); Normal() = Coordinates3D();
	adddouble("Radius",0.0);
	adddouble("Height",0.0);
}

inline Coordinates3D& Cylinder2::Center() {
	return static_cast<Coordinates3D&>(getlist(1));
}

inline Coordinates3D& Cylinder2::Normal() {
	return static_cast<Coordinates3D&>(getlist(3));
}

inline double Cylinder2::Radius() {
	return getdouble(5);
}

inline double Cylinder2::Height() {
	return getdouble(7);
}

inline Cylinder2& Cylinder2::setRadius(double v) {
	setdouble(5,v);
	return *this;
}

inline Cylinder2& Cylinder2::setHeight(double v) {
	setdouble(7,v);
	return *this;
}

/*========================================
* OrientedCircle member function implementations
*=======================================*/

inline OrientedCircle::OrientedCircle() {
	addlist("Center"); Center() = Coordinates3D();
	addlist("Normal"); Normal() = Coordinates3D();
	adddouble("Radius",0.0);
}

inline Coordinates3D& OrientedCircle::Center() {
	return static_cast<Coordinates3D&>(getlist(1));
}

inline Coordinates3D& OrientedCircle::Normal() {
	return static_cast<Coordinates3D&>(getlist(3));
}

inline double OrientedCircle::Radius() {
	return getdouble(5);
}

inline OrientedCircle& OrientedCircle::setRadius(double v) {
	setdouble(5,v);
	return *this;
}

/*========================================
* GraspDescriptor member function implementations
*=======================================*/

inline GraspDescriptor::GraspDescriptor() {
	addvocab("effector",0);
	addvocab("type",0);
	addlist("object"); object() = Cylinder1();
}

inline int GraspDescriptor::effector() {
	return getvocab(1);
}

inline int GraspDescriptor::type() {
	return getvocab(3);
}

inline Cylinder1& GraspDescriptor::object() {
	return static_cast<Cylinder1&>(getlist(5));
}

inline GraspDescriptor& GraspDescriptor::seteffector(int v) {
	setvocab(1,v);
	return *this;
}

inline GraspDescriptor& GraspDescriptor::settype(int v) {
	setvocab(3,v);
	return *this;
}

/*========================================
* HandCommand member function implementations
*=======================================*/

inline HandCommand::HandCommand() {

}

inline HandICub& HandCommand::lhan() {
	return get(lhanID());
}

inline HandICub& HandCommand::rhan() {
	return get(rhanID());
}

inline void HandCommand::lhan(const HandICub& v) {
	set(v,lhanID());
}

inline void HandCommand::rhan(const HandICub& v) {
	set(v,rhanID());
}

inline bool HandCommand::islhan() {
	return (subID() == HandCommand::lhanID::value);
}

inline bool HandCommand::isrhan() {
	return (subID() == HandCommand::rhanID::value);
}


	
	
	
	} //end namespace msg
} //end namespace darwin

	
	
	
	
	
	
		
		
	
	
		
		
		
	
  
		
	
  
    
    
    
  
  
    
    
    
  
	
		
		
		
		
	
  
    
    
    
  
	
	
	

	
  
    
    
  

  
     
     
     
  

   
	 

