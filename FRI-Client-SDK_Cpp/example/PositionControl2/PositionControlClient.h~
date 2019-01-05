/**
*/

#include "friLBRClient.h"

using namespace KUKA::FRI;

/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class PositionControlClient : public LBRClient
{
   
public:
      
   /**
    * \brief Constructor.
    * 
    */
   PositionControlClient();
   
   /** 
    * \brief Destructor.
    */
   ~PositionControlClient();
   

   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   	virtual void command();
	void intvalues(double MRPStep[7], double MaxJLRad[7], double MinJLRad[7]);
   	float GetTimeStep();
	int GetTimeStamp();
	double* GetMeasJoint();
	double* GetExtTor();
	double* GetComJoint();
	void onStateChange(ESessionState oldState, ESessionState newState);
	

      
private:
   
	int SecTime;
   	
   	double MaxJointLimitRad[7];
   	double MinJointLimitRad[7];
   	double MeasuredJoint[7];
   	double CommandedJoint[7];
   	double ExternalTorque[7];

public:
	double MaxRadPerStep[7];
	double MRPStep[7];
	double MaxJLRad[7];
	double MinJLRad[7];
	double LastJoint[7];
	double NextJoint[7];
	int KukaState;
	


   
};


