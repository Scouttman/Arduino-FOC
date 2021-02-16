#ifndef CURRENTSENSE_H
#define CURRENTSENSE_H


#include "../foc_utils.h"

/**
 *  Current sensing abstract class defintion
 * Each current sensoring implementation needs to extend this interface
 */
class CurrentSense{
    public:

    /**
     *  Function intialising the CurrentSense class
     *   All the necessary intialisations of adc and sync should be implemented here
     */
    virtual void init();
    /**
     *  Function rading the phase currents a, b and c
     *   This function will be used with the foc control throught the function 
     *   CurrentSense::getFOCCurrents(electrical_angle)
     *   - it returns current c equal to 0 if only two phase measurements available
     * 
     *  @return PhaseCurrent_s current values
     */
    virtual PhaseCurrent_s getPhaseCurrents();
    /**
     * Function reading the magnitude of the current set to the motor
     *  It returns the abosolute or signed magnitude if possible
     *  It can receive the motor electrical angle to help with calculation
     *  This function is used with the current control  (not foc)
     *  
     * @param angle_el - electrical angle of the motor (optional) 
     */
    virtual float getCurrent(float angle_el = 0);

    /**
     * Function used for FOC contorl, it reads the DQ currents of the motor 
     *   It uses the function getPhaseCurrents internally
     * 
     * @param angle_el - motor electrical angle
     */
    virtual DQCurrent_s getFOCCurrents(float angle_el);

};

#endif