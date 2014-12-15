// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
  * Copyright (C)2013  King's College London
  * Author Giuseppe Cotugno
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
  
/**
 * @file IcubThread.h
 * @brief Stub and concrete thread classes implementing Software Grasping Synergies on the iCub.
 */

#pragma once


//comment/decomment to disable/enable debug printout
//#define GRASPMODULE_PRINT_DEBUG

//std includes
#include <vector>

//yarp includes
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

//local includes
#include "GraspThreadImpl.h"
#include "ConfigMessages.h"
#include "Vocabs.h"
#include "MessageFormats/GraspMessages.h"
#include "MessageFormats/VocabDefinitions.h"

namespace darwin {
namespace grasp {

  /**
    *
    * @brief Infrastructure component of the GraspModule for the iCub.
    *
    * This class implements the infrastructure in support of the scientific component of the module. This class is implemented as a Finite State Machine (FSM). The FSM can be either in an Idle state on in the Listening macrostate.
    * The Listening macrostate implements another FSM for handling the commands received at the command port. In normal conditions the FSM should always be in listening state.
    *
    *
    * The class implements a GraspState as well, this struct encodes all the transient information needed to grasp. In the sourcecode is possible to uncomment a define to enable debug printouts. This class should not be extended/modified directly unless you are not happy with the base infrastructure of the thread.
    *
    * This class receives input from the following (default) ports:
    * 1. /grasp/left:i - Left arm joint measurements collected directly from the robot
    * 2. /grasp/right:i - Right arm joint measurements collected directly from the robot
    * 3. /grasp/command:i - Commands provided by an external module to trigger grasping actions, use this port to control the logic
    *
    * This class outputs its data on the following (default) ports:
    * 1. /grasp/left:o - Left hand joint values, only the 9 values of the hand are sent to the receiver
    * 2. /grasp/right:o - Right hand joint values, only the 9 values of the hand are sent to the receiver
    * 3. /grasp/outcome:o - Reports the outcome of the action to an external module, use this port to collect the results.
    *
    * As the joint output is given in the form of 9 joint values representing the hand active DOFs, we reccomend using this module altogether with the PostureControl module for a plug and play use.
    *
    * The class reads the config file and communicates using typesafe bottles. You don't need to use typesafe bottles to communicate with the module, they are only used internally to parse the config file.
    * Communication with the module can happen using conventional bottles as long as you comply with the format.
    *
    * Along with the file config.ini that defines general properties of the grasping, other files defining posture-specific properties are appended.
    *
    * The parameters are described in IcubThread class documentation.
    *
    * If you are using this module, please cite the following paper:
    * Simplifying Grasping Complexity through Generalization of Kinaesthetically Learned Synergies, G. Cotugno, V. Mohan, K. Althoefer, T. Nanayakkara, Proceedings of 2014 IEEE International Conference on Robotics and Automation (ICRA 2014)
    * Freely downloadable at: http://thrish.org/research-team/giuseppe-cotugno
    *
    * \author Giuseppe Cotugno
    */

class IcubStub : public GraspThreadImpl<yarp::os::BufferedPort<yarp::sig::Vector>,yarp::os::BufferedPort<HandCommand> > {

protected:

    static const int _NFINGERS;   ///< Constant number of fingers in icub

    static const double _LISTEN_PAUSE; ///< Delay after a listening operation (no command is accepted while waiting)
    static const double _IDLE_PAUSE; ///< Delay while in idle state (no command is accepted while waiting)
    static const double _COMMAND_PAUSE; ///< Delay before sending joint commands


    //effector maps
    typedef std::map<int,GraspDescriptor*> GraspDescriptorMap;
    typedef std::map<int,HandICub*> MeasureMap;
    //object maps
    typedef std::map<int,IntVector*> JointMaskMap;
    typedef std::map<int,IcubObjectData*> ObjectDataMap; //list of parameters for object grasping

    /**
      * @brief Struct cointaining the current state of the grasping thread.
      *
      * This data structure contains all the transient information needed to perform a grasp.
      */
    struct GraspState{

        std::map<int,bool> isFirstEnvelMap; ///< Indexes of the map are the hand vocabs, values are booleans stating whether the hand started eveloping or not
        std::map<int,bool> firstRowPreshape; ///< Indexes of the map are the hand vocabs, values are booleans stating whether the hand started preshaping or not

        std::map<int,int> currIdx; ///< Indexes of the map are the hand vocabs, values are the current index of the preshaping policy being checked for execution
        std::map<int,int> blocked; ///< Indexes of the map are the hand vocabs, values are how many joints are considered blocked (will not be moved in this execution)
        std::map<int,int> maxMoving;  ///< Maximum number of joints allowed to move in an enveloping session
        std::map<int,int> envCounter; ///< Number of iterations so far (before stopping)
        std::map<int,int> restoredNum;  ///< How many joints are close to the desired position and are assumed as restored (calculated from omonimous vector)

        std::map<int,std::vector<double> > pastVals;


        std::map<int,int> resultsMap;   ///< Map indexes are vocabs SUCC or FAIL, map values count the number of effectors that succeded or failed

        std::vector<bool> restored; ///< Which joint is close to the desired position (evaluated in checkApproached)
                                    //TODO this should became a map once we see we really need it

        int testCase;   //this guy never gets resetted in reset() - number of current test case


        HandICub desiredVel;  ///< Desired velocities, for future use

        std::map<int,double> timeNowStateSession; ///< Initial time value, given from Time::now(), for a session of preshaping/enveloping per hand - in enveloping phase it is refreshed at each iteration for finger movment execution


        void reset(const HandICub& defVel, const std::vector<double> defPos);
        void reset();
        void setDefaultVel(const HandICub& defVel);
        void setDefaultFailurePosture(const HandICub& defPos); ///< Set the default to given position to acknowledge failures as such
        GraspState& operator=(const GraspState& other);

        GraspState(){

          std::vector<GraspEffectorType> tmpCnt;
          tmpCnt.push_back(GRASP_LEFT);
          tmpCnt.push_back(GRASP_RIGHT);

          for(std::vector<GraspEffectorType>::iterator it=tmpCnt.begin();it!=tmpCnt.end();++it){
              isFirstEnvelMap[*it]=true;
              firstRowPreshape[*it]=true;
              currIdx[*it]=0;
              blocked[*it]=0;
              maxMoving[*it]=JHandsNum;
              envCounter[*it]=0;
              restoredNum[*it]=0;
              pastVals[*it]=std::vector<double>(JHandsNum);
          }


            resultsMap[GRASP_SUCC]=0;
            resultsMap[GRASP_FAIL]=0;


            for(int i=0;i<JHandsNum;++i){
              restored.push_back(false);
            }

            testCase=0;

        }

    private:
        HandICub _defaultVel;
        std::vector<double> _deafultFailPos;


    };

    /**
      * @brief Information concerning the grasping context
      *
      * A grasping context is given by a grasp state and the current joint values.
      *
      */

    struct Context{

        MeasureMap jval;
        GraspState gState;

        Context(){

            //TODO?

        }

        ~Context(){

            //TODO are we deleting things twice????
            for(MeasureMap::iterator it = jval.begin();it != jval.end();++it){
                delete it->second;
            }
            jval.clear();
        }
    };

    /**
      * @brief States defined in the thread's FSM
      *
      * This enum encodes all the states used in the grasping thread's FSM
      */

    enum State{
        //Idle,
        Listening,
        Release,
        Releasing,
        Preshape,
        Enveloping,
        StateNum
    };

    //primitive enum vs. how many joints shall be blocked
    std::map<int,int> _oMinBlocks;  ///< Minimum number of blocked joints required in an execution to trigger a successful evelopment


    MeasureMap _jval; ///< Actual joint values
    GraspDescriptorMap _currCommand;

    HandICub _jrelease; ///< Set of joints encoding open hand position
    HandICub _jfist;    ///< Set of joints encoding a closed hand (limit for enveloping)

    IcubConfigParameters _params; ///< Parameters read from config.ini
    ObjectDataMap _objects; ///< Primitives map, it stores all the preshaping policies and idexes them by thumb placement
    JointMaskMap _jmask;   ///< States which joint is moving (1) and which doesn't (0)


    GraspState _gState;
    Context _context;

    std::map<int,State> _stateMap;

    //states
    /**
      * @brief Freezes the execution
      *
      * Hands execution is stopped and current state variables and parameters are saved
      */
    virtual void idle();
    /**
      * @brief Waiting for new commands
      *
      * The thread is listening for new commands coming to the port /grasp/command:i commands can be of this form:
      *  1. A GraspDescriptor TypeSafeBottle (reccomended)
      *  2. Two vocabs: [lhan|rhan] [pow|pin|rel]
      * In the second case, lhan|rhan decides whether the grasp will be executed using the left or right hand, the second vocabs states whether we are interested in a power grasp, pinch grasp or opening the hand (releasing).
      */
    virtual void listen();
    /**
      * @brief Order to open the hand
      *
      * Initializes the releasing of the grip on the given hand by asking the hand joints to match the numbers listed at jrelease variable in the config file.
      *
      * @param handVocab A vocab stating the hand of interest
      */
    virtual void release(const int handVocab);
    /**
      * @brief Opens the hand
      *
      * This function supervise the releasing of process of the grip by making sure that more than the number of joints stated in config variable restored_joints reached the final posture (as triggered by IcubStub::release).
      * The opening of the hand is considered over if restored_joints number of joints are more or less close to jrelease values (as verified in IcubStub::checkApproached)
      *
      * @param handVocab A vocab stating the hand of interest
      */
    virtual void releasing(const int handVocab);
    /**
      * @brief Preshapes the fingers prior enveloping. Not used in stub.
      *
      * This function preshapes the fingers prior enveloping them around the object. This function is not implemented in the stub, the fingers are just moving to make sure everything works.
      *
      * @param handVocab A vocab stating the hand of interest
      */
    virtual void preshape(const int handVocab);
    /**
      * @brief Envelopes the fingers around the object. Not used in stub.
      *
      * This function envelops the fingers around the object, assuming it is under the palm. This function is not implemented in the stub, the fingers are just moving to make sure everything works.
      *
      * @param handVocab A vocab stating the hand of interest
      */
    virtual void envelope(const int handVocab);

    //helper func
    /**
      * @brief Reads joint data
      *
      * This function reads the hand only joint values (9 values) for both hands from ports /grasp/left:i and /grasp/right:i . Most of methods build on this function.
      *
      */
    virtual void readHands();
    /**
      * @brief Writes joint data
      *
      * This function sends the hand only joint values (9 values) to the port relative to the given hand.
      *
      * @param handVocab A vocab stating the hand of interest
      * @param jvals The joint values to be sent to the robot
      * @param dvals Vector of joint velocities - for future use.
      */
    virtual void sendCommand(const int handVocab, HandICub& jvals, HandICub& dvals);
    /**
      * @brief Flags requested motion completed
      *
      * This function is called once a command it's executed and removes its from the queue.
      *
      * @param handVocab A vocab stating the hand of interest
      */
    virtual void releaseCommand(const int handVocab);
    /**
      * @brief Stops the hands motion
      *
      */
    virtual void stopHands();
    /**
      * @brief Returns the outcome of the action and sets the FSM back to listening state
      *
      * Records successes and failures for left and right grippers at upon termination of the requested command. Resets the FSM back to listening.
      *
      * @param outcomeVocab States whether the command assigned to \b handVocab has succeeded (SUCC) or failed (FAIL)
      * @param handVocab A vocab stating the hand of interest
      */
    virtual void finishUp(const int outcomeVocab, const int handVocab);
    /**
      * @brief Verify if the command is terminated
      *
      * Verifies if a requested command is executed, if that is the case the result is propagated to IcubStub::finishUp
      */
    virtual void checkResults();
    /**
      * @brief Saves the current state of the thread
      *
      */
    virtual void contextSwitch(); //saves the current grasping data (needed for restore())
    /**
      * @brief Restores the thread's state
      *
      */
    virtual void restore();
    /**
      * @brief Checks whether the hand joints have reached a desired position
      *
      * This function verifies whether the joints of a given hand are matching a set of requested joint configuration. A position is considered as reached if the difference between j[i] and myJoints[i] is less than jthreashold.
      * jthreashold is a config file parameter.
      *
      * @param handVocab A vocab stating the hand of interest
      * @param myJoints A VectorBottle (a numeric bottle) of desired joint values to check for.
      */
    virtual void checkApproached(const int handVocab, const HandICub& myJoints);

    //temp helper func
    /**
      * @brief Maps a command into a config taxanomy
      *
      * Conversion between command vocabs and taxanomy vocabs used in the config files.
      */
    virtual int getPrimType(int taxanomyVocab);

public:	
	/**
	  * @brief Stub Constructor
	  *
	  * This constructor instantiates the stub version of the grasper module. This version does not performs the actual grasps as in the paper, it is mostly used for debugging an integration testing.
	  */
	IcubStub();

	/**
	  * @brief Destructor
	  */
	virtual ~IcubStub();

	/**
    *  Configure the ports and parameters using the config file, returns true if successful
        *  The stub class configures the thread in the same way as the main thread. In this function the parameters of the grasper module are initialised using typesafe bottle parsing of the config file.
    * @param rf reference to the resource finder
    * @return A Flag for the success
    */
    virtual bool configure(yarp::os::ResourceFinder& rf);

	/**
	*  @brief Executes the stub thread
	*
	* Main function running the thread. For the stub class, the thread only reads and writes data to and from the ports, without grasping. This method implements a Finite State Machine, the FSM drives the high level logic of grasping.
	*
	* After initializing the joint values, the thread waits for a command if it is not set to idle. As soon as a command is given, it is addressed in the appropriate FSM branch. In any case, the current state is checked by IcubStub::checkResults() at each iteration as well as whther the thread has finished and the results can be propagated to the module.
	*/
	virtual void run();
    

    /**
     * @brief function that test the network and feature of the thread
     *
     * The following tests are executed in a loop at present:
     * 1. Pinch grip (pinch.ini) using left hand
     * 2. Releasing left hand grip
     * 3. Power grip (cuboid.ini) using right hand
     * 4. Releasing left hand grip
     * 5. Releasing right hand grip
     * 6. Pinch grip using left and right hands at the same time
     * 7. Releasing both left and right grip
     * 8. Power grasp using both left and right hands at the same time
     * 9. Releasing both left and right grip
     * 10. Releasing both left and right grip
     * The number of tests is hardcoded, hence some manual edits are needed in case of modifications.
     *
     * @return the result of the analysis true/false for success/unsuccess in the test
     */
    virtual bool test();
    
};

/**
  * @brief Scientific component of the GraspModule for the iCub.
  *
  * This class implements the scientific part of the module as described in the paper. Since publication, the code has been refactored hence its overall execution is much faster.
  *
  * Consider extending this class if you want to customize the actual algorithm described in the paper.
  *
  * The class reads the config file and communicates using typesafe bottles. You don't need to use typesafe bottles to communicate with the module, they are only used internally to parse the config file.
  * Communication with the module can happen using conventional bottles as long as you comply with the format.
  *
  * Along with the file config.ini that defines general properties of the grasping, other files defining posture-specific properties are appended (preshaping policies).
  * At release time, two preshaping policies were included: cuboid.ini and pinch.ini
  *
  * Cuboid.ini implements the preshaping policy as described in the referenced paper and it performs a preshaping power grasp positioning the thumb in opposition with the other four fingers. For a list of objects suitable for this posture, please consult the referenced paper.
  * Pinch.ini is tailored to perform specifically a pinch grip using the thumb in opposition with the index and middle fingers. The pinch grip is performed through preshaping only, the enveloping function is not intended to be executed, however carefully selected enveloping parameters should lead to the same results.
  *
  * When modifying the config files IT IS VERY IMPORTANT NOT TO ALTER THE FORMATTING of the data, including spaces, commas and brakets. Failing to do so might crash the module as soon as the parameters are accessed, as the typesafe bottles might not parse the data correctly.
  *
  * The parameters in the config.ini file influence the behaviour of the module as follows:
  * 1. primitives - number of primitives used (synergies), at present this number is set to 3 and is not modifiable
  * 2. ispreshape - if set to 1 preshaping is executed, if set to 0 enveloping will be executed directly
  * 3. restored_joints - number of joints that shall go to restored hand configuration to consider a releasing operation as done
  * 4. jthreashold - motion threashold, in degrees, under which it is considered that the joint has reached its motion target. This value is used in IcubStub::checkApproached
  * 5. istimedvalidation - if set to 1, the enveloping phase termination will be evaluated based on time elapsed, otherwise it will be evaluated based on number of iterations
  * 6. iswait - it states how much a preshaping iteration should be delayed
  * 7. ischeckAdAb - if set to 1, the adduction/abduction joint will also be counted when function IcubStub::checkApproached is evaluating whether the hand has reached a desired target
  * 8. released - open hand configuration, used for releasing an object. This is parsed as a VectorBottle.
  * 9. fist - closed hand configuration, used as a target configuration while performing envelopment phase. This is parsed as VectorBottle.
  *
  * Alongside with simple parameters, there are also grouped parameters that logically refer to different termination conditions. On the technical point of view, these groups are parsed as StructBottles
  *
  * 10.  touch - group of parameters for the tactile based termination criteria for the enveloping phase. This termination criteria, originally used in the module, has not been implemented in the current release as tactile sensors are still fragile for repetetive grasps. The parameters are documented to allow future use.
  *   1. taxels - number of taxels per finger
  *   2. fing_threas - threashold to determine whether a taxel has touched an object or not
  *   3. perc_active_fing - percent of total taxels that must be active to trigger a successful grasp
  *
  *
  * 11. angle - group of parameters controlling the angular displacement termination criteria for the enveloping phase. This criteria is used in the referenced paper
  *   1. increment - threashold beyond which a set of joints is considered different enough, from current configuration, to be sent to the robot
  *   2. \b degrees_tol - threashold to determine whether an enveloping motion is small enough to consider a joint as immobile
  *   3. pausing_pre - sampling frequency for the preshaping phase, every pausing_pre seconds the configuration of the hand is compared with the next suitable set (>increment) of values
  *   4. \b pausing_env - sampling frequency for the enveloping phase, every pausing_env seconds each joint in the current configuration of the hand is compared with its counterpart of the previous configuration and, if the difference is less than degrees_tol, the joint is assumed as steady
  *   5. defVels - default joint speed. Currently not implemented and reserved for future use
  * The third parameter reported on the paper (the angular increment) has been removed in order to optimise the execution. The hand joints now are going smoothly to a final configuration (fist) rather than being incremented at each iteration. To change the angular increment it is sufficient to change the joint speeds.
  *
  * The preshaping policies files have all the same structure. Their parameters are described in the following list:
  * 1. thumb - vocab stating where the thumb is placed. This is effectively a user identifier. Currently, one policy has the thumb on the side (cuboid.ini - sd) while the other is crafted for a pinch grip (pinch.ini - pin). This parameter can be used to discriminate one policy versus another
  * 2. isenveloping - if set to 1, the enveloping phase will be executed. Setting this parameter to 0 is more efficient than setting the mask parameters (see below) to 0 as the whole function will be skipped
  * 3. mask - if an element is set to 0, its correspective joint will not be moved in enveloping phase
  * 4. min_blocked - minimum number of joints that must be considered blocked to consider the enveloping phase as successful
  * 5. jskipped - number of joints ignored when matching current with desired positions during preshaping. This parameter influences how many joints should be considered to decide whether the hand has reached a desired configuration
  * 6. angles - preshaping policy extracted from kinaesthetic demonstrations of grasping a cuboid. It is possible to hardcode a preshaping by changing these values (as long as the difference between each entry is bigger than increment)
  * 7. derivatives - derivatives of the preshaping policy. For future use.
  *
  * If you use this module, please cite the following paper:
  * Simplifying Grasping Complexity through Generalization of Kinaesthetically Learned Synergies, G. Cotugno, V. Mohan, K. Althoefer, T. Nanayakkara, Proceedings of 2014 IEEE International Conference on Robotics and Automation (ICRA 2014)
  * Freely downloadable at: http://thrish.org/research-team/giuseppe-cotugno
  *
  * \author Giuseppe Cotugno
  */

class IcubThread : public IcubStub {

protected:

    //states
    /**
      * @brief Preshapes the fingers according to the preshaping policy requested upon sending the grasping command
      *
      * This function executes a preshaping policy derived as described in the referenced paper. The code is optimised such that joint values larger than a threashold are sent to the robot.
      *
      * Specifically, an entry of the preshaping policy is sent to the robot only if its difference with the current joint values is larger than angles.increment config parameter.
      *
      * The function terminates if the policy is scanned to the end or if the object is grasped (no finger motion detected until angles.pausing_pre is elapsed)
      *
      * @param handVocab A vocab stating the hand of interest
      */

    virtual void preshape(const int handVocab);

    /**
      * @brief Envelopes the fingers around the object.
      *
      * This function linerly closes the fingers to create a fist, following the configuration described in the config parameter jfist. As a result the fingers are enveloping an object if it is under the palm.
      *
      * While the logic of the function is the same, the function has been re-engineered and its performance is much faster than reported in the referenced paper.
      * Specifically, the function send a final position to the hand and checks at each iteration whether the final position has been achieved or whether the fingers are idle for angles.pausing_env seconds.
      * The fingers are considered idle if the difference between current and previous values is less than degrees_tol.
      *
      * @param handVocab A vocab stating the hand of interest
      */
    virtual void envelope(const int handVocab);

    //helper func


public:
	/**
    * @brief Default Constructor
    */
	IcubThread();

	/**
     * @brief Destructor
     */
	virtual ~IcubThread();

	/**
	*  Main function running the thread, implementing the grasp functionality. The execution is functionally the same of IcubStub::run
	*/
	virtual void run();
};

} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------

