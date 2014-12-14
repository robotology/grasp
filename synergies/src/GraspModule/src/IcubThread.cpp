// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * CopyGRASP_RIGHT (C)2013  King's College London
  * Author: Giuseppe Cotugno
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

// /////////
// iCub joints limits and default velocity (for our reference)
//----------
// Vel 20.0
// [LIMITS]
// Max     10   160.8     80     106      90       0     40      60    90      90      180       90       180     90      180   270
// Min    -95    	 0     -37    15.5     -90     -90    -20      0    10       0        0        0         0      0        0     0
// ////////////

/**
 * @file IcubThread.cpp
 * @brief Thread implementing grasping on the ICub
 */

#include <vector>
#include <cmath>
#include <cstdlib>
#include <assert.h>

#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Value.h>

#include "IcubThread.h"
#include "GraspThreadImpl.cpp" //include template definitions to avoid linking problems
#include "MessageFormats/VocabDefinitions.h"


using namespace darwin::grasp;
using namespace darwin::msg;

using namespace yarp::sig;
using namespace yarp::os;

// ////////////////////////////////
// Nested class methods
// ////////////////////////////////

void IcubStub::GraspState::reset(const HandICub& defVel, const vector<double> defPos){


    for(std::map<int,std::vector<double> >::iterator it=pastVals.begin();it!=pastVals.end();++it){
      //double tmp[]={360.0,360.0,360.0,360.0,360.0,360.0,360.0,360.0,360.0,360.0};
      pastVals[it->first]=defPos;//vector<double>(tmp,tmp+(sizeof(tmp)/sizeof(double)) );   //those guys should be init only once;
    }

    desiredVel=defVel;//HandICub (0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5);


    for(int i=0;i<JHandsNum;++i){
        restored[i]=false;
    }

    //we assume that the Vocab for an hand and the number of hands are the same across variables
    for(map<int,bool>::iterator it=isFirstEnvelMap.begin();it!=isFirstEnvelMap.end();++it){
        isFirstEnvelMap[it->first]=true;
        firstRowPreshape[it->first]=true;
        currIdx[it->first]=0;
        blocked[it->first]=0;
        maxMoving[it->first]=JHandsNum;
        envCounter[it->first]=0;
        restoredNum[it->first]=0;
    }


    for(map<int,int>::iterator it=resultsMap.begin();it!=resultsMap.end();++it){
        resultsMap[it->first]=0;
    }

}

void IcubStub::GraspState::reset(){
    reset(_defaultVel,_deafultFailPos);
}

void IcubStub::GraspState::setDefaultVel(const HandICub& defVel){

    _defaultVel=defVel;

}

void IcubStub::GraspState::setDefaultFailurePosture(const HandICub& defPos){

  for(int i=0;i<JHandsNum;++i){

      _deafultFailPos.push_back(defPos[i]);
  }
}

IcubStub::GraspState& IcubStub::GraspState::operator=(const GraspState& other){
    if(this==&other){return *this;}

    this->currIdx=other.currIdx;
    this->firstRowPreshape=other.firstRowPreshape;
    this->blocked=other.blocked;
    this->pastVals=other.pastVals;//we assume we are doing deep copy here.... //(other.pastVals.clone());
    this->desiredVel=other.desiredVel;
    this->maxMoving=other.maxMoving;
    this->envCounter=other.envCounter;
    this->resultsMap=other.resultsMap;
    this->restoredNum=other.restoredNum;
    this->restored=other.restored;
    this->testCase=other.testCase;


    return *this;
}


// ////////////////////////////////
// Icub grasp stub thread
// ////////////////////////////////

const int IcubStub::_NFINGERS=5;

const double IcubStub::_IDLE_PAUSE=1.0;
const double IcubStub::_LISTEN_PAUSE=0.05;

const double IcubStub::_COMMAND_PAUSE=0.02;

IcubStub::IcubStub() {
    //this has been removed, but it should be initialized by the existance of the module, decomment if you run into trouble
    //_gState.reset();    //TODO potentially dangerous? If we never initialized _defaultVel, _gState.desiredVel will be initialized to...?
}

IcubStub::~IcubStub() {

    for(GraspDescriptorMap::iterator it = _currCommand.begin();it != _currCommand.end();++it){
        delete it->second;
    }
    _currCommand.clear();

    for(MeasureMap::iterator it = _jval.begin();it != _jval.end();++it){
        delete it->second;
    }
    _jval.clear();

    for(ObjectDataMap::iterator it = _objects.begin();it != _objects.end();++it){
		delete it->second;
	}
	_objects.clear();

    for(JointMaskMap::iterator it = _jmask.begin();it != _jmask.end();++it){
        delete it->second;
    }
    _jmask.clear();


}

bool IcubStub::test(){


    GraspCommand g; GraspDescriptor l; GraspDescriptor r;
    switch (_gState.testCase) {
    case 0:
        PrintLine("Test Case:PINCH GRIP LEFT");
        l.seteffector(GRASP_LEFT);
        l.settype(GRASP_PINCH);
        g.add(l);
        break;
    case 1:
        PrintLine("Test Case:RELEASE GRIP LEFT");
        l.seteffector(GRASP_LEFT);
        l.settype(GRASP_RELEASE);
        g.add(l);
        break;
    case 2:
        PrintLine("Test Case: \"POWER GRIP\" RIGHT");
        r.seteffector(GRASP_LEFT);
        r.settype(GRASP_POWER);
        g.add(r);
        break;
    case 3:
        PrintLine("Test Case: RELEASE LEFT");
        l.seteffector(GRASP_LEFT);
        l.settype(GRASP_RELEASE);
        g.add(l);
        break;
    case 4:
        PrintLine("Test Case: RELEASE RIGHT");
        r.seteffector(GRASP_RIGHT);
        r.settype(GRASP_RELEASE);
        g.add(r);
        break;
    case 5:
        PrintLine("Test Case: GRASP PINCH BOTH");
        l.seteffector(GRASP_LEFT);
        l.settype(GRASP_PINCH);
        r.seteffector(GRASP_RIGHT);
        r.settype(GRASP_PINCH);
        g.add(l); g.add(r);
        break;
    case 6:
        PrintLine("Test Case: RELEASE BOTH");
        l.seteffector(GRASP_LEFT);
        l.settype(GRASP_RELEASE);
        r.seteffector(GRASP_RIGHT);
        r.settype(GRASP_RELEASE);
        g.add(l); g.add(r);
        break;
     case 7:
          PrintLine("Test Case: GRASP POWER BOTH");
          l.seteffector(GRASP_LEFT);
          l.settype(GRASP_POWER);
          r.seteffector(GRASP_RIGHT);
          r.settype(GRASP_POWER);
          g.add(l); g.add(r);
          break;
      case 8:
          PrintLine("Test Case: RELEASE BOTH");
          l.seteffector(GRASP_LEFT);
          l.settype(GRASP_RELEASE);
          r.seteffector(GRASP_RIGHT);
          r.settype(GRASP_RELEASE);
          g.add(l); g.add(r);
          break;
      case 9:
          PrintLine("Test Case: RELEASE BOTH");
          l.seteffector(GRASP_LEFT);
          l.settype(GRASP_RELEASE);
          r.seteffector(GRASP_RIGHT);
          r.settype(GRASP_RELEASE);
          g.add(l); g.add(r);
          break;

    }
    if (++_gState.testCase>9) {     //NOTE: this number (9) is hardcoded, it must change according to the test number
        _gState.testCase = 0;
        PrintLine("Set of tests over. Restarting.");
    }
    this->write(g);

    return true;

}

bool IcubStub::configure(ResourceFinder& rf) {

    //base class configuration
    GraspThreadImpl::configure(rf);

    PrintDebugLine("====================");
    PrintDebugLine("Configuring IcubStub");
    PrintDebugLine("====================");

    Bottle& effectors = rf.findGroup("effectors");
    for(int i=1;i<effectors.size();++i) {   //setting up state machine for the two hands
        int key=get_key(effectors,i);
        _stateMap[key]=Listening;
        HandICub* pt=new HandICub;
        for(int i=0;i<JHandsNum;++i){
            pt->add(0.0);
        }
        _jval[key]=pt;
    }

    //read the parameters as a TypeSafeBottle
    //make sure the ini file is correct, otherwise we're passing now but failing hard later
    if (!rf.check("PARAMS")) {
        throw runtime_error("IcubStub::configure: missing PARAMS section");
    }

    PrintDebugLine("IcubStub PARAMS expected:");
    PrintDebugLine(_params.toString());
    _params = *static_cast<IcubConfigParameters*>(rf.findGroup("PARAMS").get(1).asList());
    PrintDebugLine("IcubStub PARAMS read");
    PrintDebugLine(_params.toString());

	if (!rf.check("cuboid")) {
		throw runtime_error("IcubStub::configure: missing cuboid section");
	}
	if (!rf.check("pinch")) {
		throw runtime_error("IcubStub::configure: missing pinch section");
	}
	
	IcubObjectData* cuboid = static_cast<IcubObjectData*>(rf.findGroup("cuboid").get(1).asList());
	IcubObjectData* pinch = static_cast<IcubObjectData*>(rf.findGroup("pinch").get(1).asList());
	PrintDebugLine(cuboid->toString());
	PrintDebugLine(pinch->toString());
    _objects[Value(cuboid->thumb(),true).asVocab()] = new IcubObjectData(*cuboid);
    _objects[Value(pinch->thumb(),true).asVocab()] = new IcubObjectData(*pinch);

    _jrelease=(static_cast<HandICub&>(_params.released()));

    _jfist=(static_cast<HandICub&>(_params.fist()));

    _gState.setDefaultFailurePosture(_jfist);
    _gState.setDefaultVel(_params.angles().defVels());
    _gState.reset();    //cleanup, for safety

    for(std::map<int,IcubObjectData*>::iterator it = _objects.begin();it != _objects.end();++it){
        IntVector* pt = new IntVector;
        for(int i=0; i < JHandsNum; ++i){
            pt->add(it->second->mask().operator[](i));
        }
        _jmask[it->first]=pt;

    }

	PrintDebugLine("=============================");
	PrintDebugLine("Finished Configuring IcubStub");
	PrintDebugLine("=============================");

    return true;
}

void IcubStub::run() {

	PrintDebugLine("================");
	PrintDebugLine("Running IcubStub");
	PrintDebugLine("================");

    bool firstIdleRun=true;
    bool hasJustResumed=false;
    bool veryFirstIteration=true;

#ifndef GRASPMODULE_PRINT_DEBUG
    PrintLine("Debug prints DISABLED");
#endif

    while (isStopping() != true) {

        if(veryFirstIteration){
            veryFirstIteration=false;
            PrintLine("Initializing iCub hand values");
            readHands();
            PrintDebugLine("iCub is listening for a command");

        
            if(_params.istimedValidation()){
                PrintLine("Enveloping stopping criteria based on time");
            }else{
                PrintLine("Enveloping stopping criteria based on iterations");
            }
        }

        if(!_idle){
            if(hasJustResumed){
                restore();
                PrintDebugLine("iCub is listening for a command");
                hasJustResumed=false;
            }


            firstIdleRun=true;

            listen();   //non-blocking behaviour
            for(std::map<int,State>::iterator it=_stateMap.begin();it!=_stateMap.end();++it){

                switch(it->second){
                //case Idle:
                //    encoded in the if
                //break;
                case Listening:
                    //nothing happens, as Listening is a macrostate
                break;
                case Release:
                    PrintDebugLine(yarp::os::ConstString("iCub open hand ") + Vocab::decode(it->first));
                    release(it->first);
                break;
                case Releasing:
                    releasing(it->first);
                break;
                case Preshape:
                    PrintDebugLine(yarp::os::ConstString("iCub preshapes ") + Vocab::decode(it->first));
                    preshape(it->first);
                break;
                case Enveloping:
                    PrintDebugLine(yarp::os::ConstString("iCub envelopes ") + Vocab::decode(it->first));
                    envelope(it->first);
                break;
                default:
                    throw runtime_error("GSP/iCub_thread> Illegal thread state runtime_error!");
                break;

                }
            }
            checkResults();


            if (_done) {
                handleGraspDone();
            }

        }else{
            //idle state
            if(firstIdleRun){
                PrintDebugLine("iCub is idle");
                idle(); //saving stuff
                firstIdleRun=false;
                hasJustResumed=true;
            }
            Time::delay(_IDLE_PAUSE);
        }

    }


    PrintDebugLine("================");
    PrintDebugLine("Closing IcubStub");
    PrintDebugLine("================");
}


void IcubStub::release(const int handVocab){

    if(_currCommand.find(handVocab)!=_currCommand.end()){  //if we have a command
        sendCommand(handVocab,_jrelease,_gState.desiredVel);
    }

    _stateMap[handVocab]=Releasing;

}

void IcubStub::releasing(const int handVocab){


    //checks hand posture from posture control to understand if releasing is over

    readHands();


    checkApproached(handVocab,_jrelease);

    //icub joints are too unaccurate, current check might be too strict - verify
    if(_gState.restoredNum[handVocab]>_params.restored_joints()){//if number of joints at rest is X, then we are done (ideally we could be notified by the system if something breaks up and change this value, but it seems to be asking too much...)

        PrintDebugLine("IcubStub::releasing: triggering success");
        finishUp(GRASP_SUCC,handVocab);
    }

}

void IcubStub::preshape(const int handVocab){

    //iterates over the joint matrix and sends joint data
    //TODO checks if object grasped and stops before iterating throughout the matrix (must be found EXPERIMENTALLY on the robot the threadholds)

    readHands();    //TODO understand why sometimes at the beginning the thread cannot initialize the hands from the robot

    HandICub val;

    for(int i=0;i<_jval[handVocab]->size();++i){
        val.add(10.0);
    }

    sendCommand(handVocab,val,_gState.desiredVel);

    //wait a bit
    Time::delay(2);

    _stateMap[handVocab]=Enveloping;

}

void IcubStub::envelope(const int handVocab){

    //sets the motor speed to very low number
    //assigns final position to fingers
    //costantly checks whether fingers move or not
    //returns fail or succeed


    readHands();

    HandICub val;

    for(int i=0;i<_jval[handVocab]->size();++i){
        val.add(25.0);  //it will never work...
    }

    sendCommand(handVocab,val,_gState.desiredVel);

    //wait a bit
    Time::delay(1);

    PrintDebugLine("IcubStub::envelope: triggering artificial failure");
    finishUp(GRASP_FAIL,handVocab);
}

void IcubStub::listen(){

    if(_new){   //new command came in

        _stateLock.wait();

        //going to S2 (parse state)
        PrintLine("New command received, descriptors no. " + _command.size());
        for(int i=0;i<_command.size();++i){

            _currCommand[_command[i].effector()]=new GraspDescriptor(_command[i]); //overwriting behaviour
            PrintDebugLine(yarp::os::ConstString("Parsing descriptor: effector=") + Vocab::decode(_command[i].effector()) + " type=" + Vocab::decode(_command[i].type()) + " object_cylinder.toString="+ _command[i].object().toString());

            //here we shall assert that effector is either GRASP_LEFT or GRASP_RIGHT

            switch(_command[i].type()){
            case GRASP_RELEASE:
                _stateMap[_command[i].effector()]=Release;
                break;
            //if new grasping synergies are added, please extend this part until the // ### //
            case GRASP_POWER:
            case GRASP_PINCH:
                if(_params.ispreshape()){
                    PrintLine("Grasping with preshaping");
                    _stateMap[_command[i].effector()]=Preshape;
                  }else if(_objects[_command[i].type()]->isenveloping()){
                    PrintLine("Grasping with enveloping only");
                    _stateMap[_command[i].effector()]=Enveloping;
                }else{
                    _stateMap[_command[i].effector()]=Listening;
                    PrintLine("WARNING: Preshaping disabled but enveloping is also disabled! Check your config files! ICub will be idle now.");
                }
                break;
            // ### //
            case GRASP_TYPE_UNKNOWN:
                PrintLine("Grasp primitive required is not known, back to listening state");
                _stateMap[_command[i].effector()]=Listening;
                break;
            case GRASP_INTERRUPT:
                PrintLine("Interrupt not yet implemented");
                break;
            default:
                throw runtime_error("IcubStub::listen(): Undefined command received on the listening port");
            }
        }
        _new=false;

        _stateLock.post();
    }else{
        Time::delay(_LISTEN_PAUSE);
    }

}

void IcubStub::idle(){

    PrintLine("iCub is idle");
    //freezes hands and saves last valid joint values
    stopHands();
    //extended class will also keep track of the state of the preshaping matrix and enveloping stage
    contextSwitch();

}

// /////////////////
// / Helper functions
// /////////////////

void IcubStub::readHands(){

    for(MeasureMap::iterator it=_jval.begin();it!=_jval.end();++it){
        if(_inPorts[it->first].getInputCount()!=0){
            Vector* rawData=_inPorts[it->first].read();    //read from port

            if(rawData){
                for(int i=j_finger_add_abd;i<IcubJointsNum;++i){    //iterate over read Vector
                        (it->second)->set(i-j_finger_add_abd,rawData->operator [](i));
                }
            }
        }else{
            PrintDebugLine("IcubStub::readHands: I am not listening to anything");
        }

    }

}


void IcubStub::sendCommand(const int handVocab, HandICub& jvals, HandICub& dvals){

    if(_outPorts[handVocab].getOutputCount()!=0){
        HandCommand& tmp=_outPorts[handVocab].prepare();

        switch(handVocab){
            case GRASP_LEFT:
                tmp.lhan(jvals);
            break;
            case GRASP_RIGHT:
                tmp.rhan(jvals);
            break;
            default:
                throw runtime_error("IcubStub::sendCommand: I don't know which hand you want to use, so I crash your program");
        }

        _outPorts[handVocab].write();
        PrintDebugLine(ConstString("Sending: ")+(jvals.toString()));
    }else{
        PrintDebugLine("IcubStub::sendCommand: I am not writing on anything");
    }

}

//if we call this, we are assuming that we have processed commands on both hands
void IcubStub::releaseCommand(const int handVocab){


    //flagging commands as processed
    map<int,GraspDescriptor*>::iterator itL=_currCommand.find(handVocab);
    if(itL!=_currCommand.end()){
        _currCommand.erase(itL);
    }

}

void IcubStub::stopHands(){

    readHands();

    for(MeasureMap::iterator it=_jval.begin();it!=_jval.end();++it){
        sendCommand(it->first,*(it->second),_gState.desiredVel);
    }
}

void IcubStub::contextSwitch(){

    //context switch
    for(MeasureMap::iterator it=_jval.begin();it!=_jval.end();++it){

        if(_context.jval.find(it->first)==_context.jval.end()){
            _context.jval[it->first]=new HandICub;
        }

        *_context.jval[it->first]=*(it->second);

    }
    _context.gState=_gState;
}

void IcubStub::restore(){

    //we restore hand position
    readHands();

    //initializing changes mask
    map<int,bool> hasChanged;

    for(MeasureMap::iterator it=_jval.begin();it!=_jval.end();++it){
        hasChanged[it->first]=false;
    }

    for(MeasureMap::iterator it=_jval.begin();it!=_jval.end();++it){
        checkApproached(it->first,*(_context.jval[it->first]));
        if(_gState.restoredNum[it->first]>0){    //if anything is far from original position, then you must move that hand
            hasChanged[it->first]=true;
        }
    }

    //sending commands to the guys that changed
    for(MeasureMap::iterator it=_jval.begin();it!=_jval.end();++it){
        if(hasChanged[it->first]){
            sendCommand(it->first,*_context.jval[it->first],_gState.desiredVel);
        }
    }
    readHands();    //instead of swapping vectors we do a fresh read so we have more up-to-date data

    _gState=_context.gState;

}

void IcubStub::checkApproached(const int handVocab, const HandICub& myJoints){
  if(!_params.ischeckAdAb()){
      _gState.restored[finger_add_abd]=true;
    }

    for(int i=thumb_oppose;i<myJoints.size();i++){
        double diffL=abs((*_jval[handVocab])[i]-myJoints[i]);

        if(diffL<_params.jthreashold()){  //jthreashold is saying whether we approached myJoints configuration or not
            _gState.restored[i]=true;
        }

    }


    _gState.restoredNum[handVocab]=0;
    for(int i=0;i<JHandsNum;++i){
        if(_gState.restored[i]){
            _gState.restoredNum[handVocab]+=1;
        }
    }

    assert(_gState.restoredNum[handVocab]<=JHandsNum);

}



void IcubStub::finishUp(const int outcomeVocab, const int handVocab){

    PrintDebugLine("IcubStub::finishUp: finishing up");

    _gState.resultsMap[outcomeVocab]++;
    _gState.timeNowStateSession[handVocab]=0;

    _stateMap[handVocab]=Listening;

}

void IcubStub::checkResults(){

    _stateLock.wait();
    _outcome.clear();
    _stateLock.post();

    for(map<int,int>::iterator it=_gState.resultsMap.begin();it!=_gState.resultsMap.end();++it){

        //if we are done for any reason, update things and clean up _gState
        if( (it->first==GRASP_FAIL && it->second >0) || (it->first==GRASP_SUCC && it->second>=_currCommand.size() && _currCommand.size()>0)){
            _stateLock.wait();

            for(int i=0;i<_currCommand.size();++i){
                _outcome.add(it->first);
            }

            _done=true;
            _gState.reset();
            for(GraspDescriptorMap::iterator it=_currCommand.begin();it!=_currCommand.end();++it){
                releaseCommand(it->first);
            }

            _stateLock.post();

            break;
        }//else{
        //  [please insert your code for handling the interupts here (and maybe in the if)]
        //}
    }

}


int IcubStub::getPrimType(int taxanomyVocab){

    switch(taxanomyVocab){
        case GRASP_POWER:
            return THUMB_SIDE;
        break;
        case GRASP_PINCH:
            return PINCH_GRIP;
        break;
        default:
            throw runtime_error("IcubStub::getPrimType: required taxanomy is not handle/does not exist!!!");

    }

}

// ////////////////////////////////
// Icub grasp thread
// ////////////////////////////////

//NOTE: this thread can be extended to plug in more science (i.e. intelligent stopping of fingers in preshaping, add velocity based preshaping with kalman filer, add estimator for non-linear movement of fingers on enveloping (based on KF?? :P yum yum)

IcubThread::IcubThread() {

}

IcubThread::~IcubThread() {

}

void IcubThread::run() {

    PrintDebugLine("=================");
    PrintDebugLine("Running IcubThread");
    PrintDebugLine("=================");

    IcubStub::run();


    PrintDebugLine("=================");
    PrintDebugLine("Closing IcubThread");
    PrintDebugLine("=================");
}


// /////////////////
// Helper functions
// /////////////////



// /////////////////
// Science
// /////////////////

/**
  *
  */

void IcubThread::preshape(const int handVocab){

    readHands();

    int primType=getPrimType(_currCommand[handVocab]->type());

    IcubJointMatrix& angles=_objects[primType]->angles();

    ostringstream oss;

    //i check row i with row i+x, if rw_i - rw_ix > increment (for at least one joint) then I move
    //while I am moving I am checking if I am approaching the end state, if I am then I look for the next
    double time=Time::now();

    if(_gState.firstRowPreshape[handVocab]){ //if first execution
        PrintLine(yarp::os::ConstString("Preshaping, hand: ") + Vocab::decode(handVocab));
        _gState.currIdx[handVocab]=0;
        HandICub& joints=static_cast<HandICub&>(angles[_gState.currIdx[handVocab]]);
        PrintDebugLine(joints.toString());
        sendCommand(handVocab,joints,_gState.desiredVel);
        _gState.firstRowPreshape[handVocab]=false;
        _gState.timeNowStateSession[handVocab]=Time::now();
    }else{
        HandICub& joints=angles[_gState.currIdx[handVocab]];
        PrintDebugLine(joints.toString());
        checkApproached(handVocab,joints);
        //if we reached the target joint values
        if(_gState.restoredNum[handVocab]>=JHandsNum-_objects[primType]->jskipped()){

            int i;
            bool valFound=false;
            for(i=_gState.currIdx[handVocab]+1;i<angles.size() && !valFound;++i){ //in this for, I am looking for the next set of joints to send out, the set of joints should be less than angles.increment (TODO this might need to became a separate parameter)
               IcubJointRow& jointsNext=(angles[i]);                              //this for loop is a filter de facto
               for(int j=0;j<joints.size();++j){
                    double diff=abs(joints[j]-jointsNext[j]);
                    if(diff>(_params.angles().increment())){
                        valFound=true;
                        break;
                    }
               }//inner for
            }//outer for

            if(i<angles.size()){  //if false I am done as I iterated througout the whole preshaping policy
                _gState.currIdx[handVocab]=i;
            }else{
                _gState.currIdx[handVocab]=i-1;
                _gState.firstRowPreshape[handVocab]=true;

                //TODO implement function nextState() and kick all those checks out of here!
                if(_objects[primType]->isenveloping()){
                    //TODO all the below instructions go in a separate function
                    _stateMap[handVocab]=Enveloping;    //preshaping over
                    _gState.timeNowStateSession[handVocab]=0;
                }else{
                    PrintLine("Enveloping will not be triggered as requested by the loaded primitives config file.");
                    PrintLine("I am finished, I blindly assume I am successful (might not be the case?).");
                    finishUp(GRASP_SUCC,handVocab);
                }
            }
            HandICub& nextJoints=static_cast<HandICub&>(angles[_gState.currIdx[handVocab]]);

            double waiting=Time::now();
            double real_wait=abs(time-waiting);

            if(real_wait<_params.wait() ){ //if I am slowing down the thread and I haven't waited the delay time (almost certain I think) then wait before sending the command
                Time::delay(_params.wait()-real_wait);
              } //the previous if is used to slow down the module

            PrintLine(yarp::os::ConstString("Matrix Index: ") + Value(_gState.currIdx[handVocab]).toString());
            _gState.envCounter[handVocab]=0;
            sendCommand(handVocab,nextJoints,_gState.desiredVel);

            double exec_moment=Time::now();
            double exec_time=time-exec_moment;
            oss << "[PRE] Matrix Iteration time: " << real_wait << " --- Preshaping execution time: " << exec_time << endl;
        }//outest if (if has reached target joint values)

        else{ //if we are here, we are assuming that nothing moved, for safety all stopping parameters are double by 2

            oss<< " [PRE] Checking joints, ";
            if(!_params.istimedValidation()){
                oss << " Iterations: " << _gState.envCounter[handVocab];
                if(_gState.envCounter[handVocab]>=(_params.angles().counterMax()*2)){  //if we went through X iterations, then we go ahead
                    PrintLine("I am finished before iterating on all the preshaping policy, I assume I am successful.");
                    finishUp(GRASP_SUCC,handVocab);
                }else{
                    _gState.envCounter[handVocab]+=1;   //counter based iteration
                }
            }else{    //if here we are evaluating based on time rather than iterations
                double waiting_value=(_params.angles().pausing_pre());
                waiting_value+=_gState.timeNowStateSession[handVocab];
                double timeDiff=(waiting_value-Time::now());
                oss << "Time Diff: " << timeDiff;
                if(timeDiff<1.0){   //if we roughly waited as much as defined in config.ini, then we go ahead
                    PrintLine("I am finished before iterating on all the preshaping policy, I assume I am successful.");
                    finishUp(GRASP_SUCC,handVocab);
                }
             }//if timed validation
            PrintLine(oss.str().c_str()); //find a way not to flood output, and this can become a nice printout for the module
         }//outest else

    }

}

void IcubThread::envelope(const int handVocab){
    readHands();

    if(_gState.isFirstEnvelMap[handVocab]){

        PrintLine(yarp::os::ConstString("Enveloping, hand: ") + Vocab::decode(handVocab));

        HandICub theFist;
        theFist=*_jval[handVocab];
        PrintDebugLine("Ready to crash...");
        IntVector* theMask= _jmask[getPrimType(_currCommand[handVocab]->type())];  //convert from type to thumb
        PrintDebugLine(Value(handVocab,true).asString());
        PrintDebugLine(theMask->toString());
        _gState.maxMoving[handVocab]=JHandsNum;
        for(int i=0;i<theMask->size();i++){
            if(!theMask->operator[](i)) {    //if j[i] is not moving
                _gState.maxMoving[handVocab]-=1;

            }else{
                theFist.set(i,_jfist[i]);
            }
        }

        //if timed base eval?
        _gState.timeNowStateSession[handVocab]=Time::now();
        //}

        //closing command is performed
        sendCommand(handVocab,theFist,_gState.desiredVel);  //TODO this desiredVal must be converted in map when we will be using it

        _gState.isFirstEnvelMap[handVocab]=false;
    }


    ostringstream oss;
    oss<< " [ENV] Checking joints, ";
    bool readyToGo=false;
    if(!_params.istimedValidation()){ //if we verify successful grasping through iterations
        oss << " Iterations: " << _gState.envCounter[handVocab];
        if(_gState.envCounter[handVocab]>=_params.angles().counterMax()){  //if we went through X iterations, then we go ahead
            readyToGo=true;
        }else{
            _gState.envCounter[handVocab]+=1;   //counter based iteration
        }
    }else{
        double waiting_value=_params.angles().pausing_env();
        waiting_value+=_gState.timeNowStateSession[handVocab];
        double timeDiff=(waiting_value-Time::now());      //BUG HERE? we never wait, we go at maximum speed
        oss << "Time Diff: " << timeDiff ;
        if(timeDiff<1.0){   //if we roughly waited as much as defined in config.ini, then we go ahead
            readyToGo=true;
        }else{
            PrintLine(" Not ready yet to go...");
        }
    }

    //in this code block we check if the evelopment has succeded or we closed the hand as a fist
    if(readyToGo){

        _gState.blocked[handVocab]=0;
        oss << " Joint Threas Val.: " << _params.angles().degrees_tol();
        PrintLine(oss.str().c_str());
        ostringstream osso;
        osso << "Current per joint difference:";
        for(int i=0;i<_jval[handVocab]->size() && _currCommand.size()>0;++i){



            if((*_jmask[getPrimType(_currCommand[handVocab]->type())])[i]){   //if current joint of current hand is allowed to move   //NOTE: here we keep on squeezing also on a blocked finger

                //
                double diff=abs((_gState.pastVals.at(handVocab)[i])-(_jval[handVocab]->operator[](i)));
                if(diff<_params.angles().degrees_tol()){
                    _gState.blocked[handVocab]+=1;
                    oss.flush();
                    oss << " j["<< i+(j_finger_add_abd) << "] is blocked! " << endl;
                    PrintDebugLine(oss.str().c_str());
                }
                _gState.pastVals.at(handVocab)[i]=(*_jval[handVocab])[i];
                //


                osso <<" j["<< i+(j_finger_add_abd) << "]=" << diff;

                checkApproached(handVocab,_jfist);    //TODO: in this way the degrees tolerance for making a fist is less than for conventional squeezing, find out experimentally if it is ok

            }

        }//for
        PrintLine(osso.str().c_str());

        //if timed base eval
        _gState.timeNowStateSession[handVocab]=Time::now(); //refresh the value of timed session for next set of iterations
        //}


        if(_gState.restoredNum[handVocab]>(_gState.maxMoving[handVocab]-(_gState.maxMoving[handVocab]-6)) ){  //MAGIC NUMBER HERE!!! - this number must be experimentally validated, to check when we have a closed fist
            finishUp(GRASP_FAIL,handVocab);
        }else if(_gState.blocked[handVocab]>=_oMinBlocks[getPrimType(_currCommand[handVocab]->type())]){//if number of blocked joints is bigger than minim number for given primitive, we are done
            finishUp(GRASP_SUCC,handVocab);
        }

    }//if ready to gol

}

// ////////////////////
// SCIENTIFIC MODE
// ////////////////////

//void IcubScience::sendCommandSci(const int handVocab, HandICub& jvals, HandICub& dvals){
//
//  [add your extensions here]
//
//}
