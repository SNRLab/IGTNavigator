/*=auto=========================================================================

  Portions (c) Copyright 2007 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $RCSfile: $
  Date:      $Date: $
  Version:   $Revision: $

=========================================================================auto=*/

// .NAME vtkRobotProbeNavLogic - slicer logic class for Locator module 
// .SECTION Description
// This class manages the logic associated with tracking device for
// IGT. 

#ifndef __vtkRobotProbeNavLogic_h
#define __vtkRobotProbeNavLogic_h

#include "vtkRobotProbeNavWin32Header.h"

#include "vtkKWTkUtilities.h"

#include "vtkSlicerBaseLogic.h"
#include "vtkSlicerModuleLogic.h"
#include "vtkSlicerApplication.h"
#include "vtkCallbackCommand.h"

#include "vtkMRMLFiducialListNode.h"
#include "vtkMRMLSliceNode.h"

#include "vtkMRMLRobotProbeNavManagerNode.h"

// 10/19/2011 ayamada
#include "vtkKWMatrixWidget.h"

// 12/22/2011 ayamada
/* IGT includes */
#include "vtkIGTPat2ImgRegistration.h"
#include "vtkMRMLAbdoNavNode.h"

class vtkMRMLAbdoNavNode;

class vtkRobotProbeNavGUI;

// 12/22/2011 ayamada
/* IGT forward declarations */
class vtkIGTPat2ImgRegistration;

class VTK_RobotProbeNAV_EXPORT vtkRobotProbeNavLogic : public vtkSlicerModuleLogic 
{
  
 public:

  // 3/8/2012 ayamada
  //BTX
  //std::string ModuleShareDirectory; // needed for model files, etc.
  //ETX
  const char* imageFilePath;
  
   //BTX
   enum 
   {  // Events
     LocatorUpdateEvent      = 50000,
     StatusUpdateEvent       = 50001,
   };
   //ETX

  
  static vtkRobotProbeNavLogic *New();
  
  vtkTypeRevisionMacro(vtkRobotProbeNavLogic,vtkObject);
  
  void SetGUI(vtkRobotProbeNavGUI* gui) { this->GUI = gui; };
  vtkRobotProbeNavGUI* GetGUI()         { return this->GUI; };

  void PrintSelf(ostream&, vtkIndent);  
  
  int  Enter();
  void TimerHandler();
  
  int  RobotStop();
  int  RobotMoveTo(float px, float py, float pz,
                   float nx, float ny, float nz,
                   float tx, float ty, float tz);
  int  RobotMoveTo(float position[3], float orientation[4]);

  int  RobotMoveTo();
  
  int  ScanStart();
  int  ScanPause();
  int  ScanStop();
  
  //BTX
  //Image* ReadCalibrationImage(const char* filename, int* width, int* height,
  //                            std::vector<float>& position, std::vector<float>& orientation);

  bool AddTargetToNeedle(std::string needleType, float* rasLocation, unsigned int & targetDescIndex);

  // Description:
  // Add volume to MRML scene and return the MRML node.
  // If volumeType is specified, then the volume is also selected as the current Calibration
  // targeting or verification volume.
  vtkMRMLScalarVolumeNode *AddVolumeToScene(const char *fileName, VolumeType volumeType=VOL_GENERIC);

  // Description:
  // Set a specific role for a loaded volume.
  int SelectVolumeInScene(vtkMRMLScalarVolumeNode* volumeNode, VolumeType volumeType);

  // Description:
  // Show/hide robot workspace. Returns with 0 in case of failure.
  int ShowWorkspaceModel(bool show);
  bool IsWorkspaceModelShown();

  // Description:
  // Show/hide robot. Returns with 0 in case of failure.
  int ShowRobotModel(bool show);
  bool IsRobotModelShown();
  
  // 9/24/2011 ayamada
  int ShowNeedleModel(bool show);
  bool IsNeedleModelShown();
  
  // 11/4/2011 ayamada
  int ShowVirtualCenterModel(bool show);
  bool IsVirtualCenterModelShown();
  
  // 10/19/2011 ayamada
  int CorrectNeedlePosition(vtkKWMatrixWidget* matrix, vtkKWMatrixWidget* oMatrix, int numRows);
  
  // 10/28/2011 ayamada
  float magnitudeOfDirectionVector;
  // 7/5/2012 ayamada
  float magnitudeOfDirectionVector2;
  // 7/5/2012 ayamada
  float carriagePositionForOrder;
  
  // 11/20/2011 ayamada
  float leftNeedleContactPosition[3];
  float tmpDis;
  float tmpDisRef;
  float tmpVectorOP[3];
  
  // 12/9/2011 ayamada
  float centerDirectionVector[3];
  float centerPassingPoint[3];
  float centerVariableK;
  float centerVariableK2;  
  float parametrizedDirectionVector[3];
  float parametrizedDirectionVector2[3];
  
  float adjusterSign;
  float adjusterSign2;
  
  // 12/22/2011 ayamada
  int PerformRegistration();
  
  // 12/26/2011 ayamada
  int workPhaseStatus;
  int workPhaseInitialFlag;
  
  /// Set and observe the parameter node.
  void SetAndObserveAbdoNavNode(vtkMRMLAbdoNavNode* node) { vtkSetAndObserveMRMLNodeMacro(this->AbdoNavNode, node); }

  // 12/28/2011 ayamada
  //void calculateVirtualCenterPosition() ;
  
  // 7/3/2012 ayamada
  vtkMRMLRobotNode* GetRobotNode();
  
  // 8/20/2012 ayamada
  float TargetPositionXYZ[3];


  //----------------------------------------------------------------
  // Logic values.
  //----------------------------------------------------------------
  /// Parameter node associated with this module.
  vtkMRMLAbdoNavNode* AbdoNavNode;
  /// Transform node holding the static registration matrix.
  vtkMRMLLinearTransformNode* RegistrationTransform;  
  /// Implementation of Horn's closed-form solution to the least-squares
  /// problem of absolute orientation.  
  vtkIGTPat2ImgRegistration *Pat2ImgReg;
  //ETX
  /// Flag indicating whether or not registration has been performed yet.
  bool RegistrationPerformed;

  // Description:
  // Switch mouse interaction mode to activate target placement
  // by clicking on the image
  // vtkMRMLInteractionNode::Place = place fiducials
  // vtkMRMLInteractionNode::ViewTransform = rotate scene
  // Return value: zero if an error occurred
  int SetMouseInteractionMode(int mode);

  // Description:
  // Select the current fidicual list in the Fiducial module
  // If the user clicks on the image in Place interaction mode, then fiducials will be added to the current fiducial list.
  int SetCurrentFiducialList(vtkMRMLFiducialListNode* fidNode);

  //ETX

  void UpdateTargetListFromMRML();

  // Description:
  // Set Slicers's 2D view orientations from the image orientation.
  void SetSliceViewFromVolume(vtkMRMLVolumeNode *volumeNode);
  
  
  // 8/4/2011 ayamada
  void GetFiducialMarkersR(int n, float px, float py, float pz);
  double FiducialMarkersR[100][3][3][3];
  void GetFiducialMarkersI(int n, float px, float py, float pz);
  double FiducialMarkersI[100][3][3][3];
  
  // 9/13/2011 ayamada
  void SendMarkersPosition();
  
  // 11/4/2011 ayamada
  void CalculateVirtualCenterPosition(void);
  
  // 2/11/2012 ayamada
  int existanceOfTarget1;
  int existanceOfTarget2;
  int existanceOfTarget3;
  int getRow;
  // parameters for linear on targets
  float linearP[3][3];
  float linearV[3][3];
  
  
  

  // 12/24/2011 ayamada
  /// Parameter node associated with this module. 
  vtkMRMLAbdoNavNode* node;  
  
  //----------------------------------------------------------------
  // Helper function to create an AbdoNavNode if none exists yet.
  //----------------------------------------------------------------
  vtkMRMLAbdoNavNode* CheckAndCreateAbdoNavNode();
  //vtkMRMLRobotProbeNavNode* CheckAndCreateAbdoNavNode();
  //vtkMRMLAbdoNavNode* AbdoNavNode;  
  
  int ObserveTrackingTransformNode();
  
  /// The relative tracking transform node created by OpenIGTLinkIF.
  vtkMRMLLinearTransformNode* RelativeTrackingTransform;  
  /// Parse NDI ToolBox ".trackProperties" file.
  //int ParseToolBoxProperties();
  
 protected:

  //BTX
  std::string GetFoRStrFromVolumeNodeID(const char* volNodeID);
  //ETX

  // Description:
  // Link targets to fiducials (when no FiducialIDs are available), based on fiducial position and label
  void LinkTargetsToFiducials();

  // Description:
  // Helper method for loading a volume via the Volume module.
  vtkMRMLScalarVolumeNode *AddArchetypeVolume(const char* fileName, const char *volumeName);  

  void UpdateAll();

  vtkRobotProbeNavLogic();
  ~vtkRobotProbeNavLogic();
  vtkRobotProbeNavLogic(const vtkRobotProbeNavLogic&);
  void operator=(const vtkRobotProbeNavLogic&);
  
  static void DataCallback(vtkObject*, unsigned long, void *, void *);

  /*
  void UpdateAll();
  void UpdateSliceDisplay();
  void UpdateLocator();
  */

  vtkCallbackCommand *DataCallbackCommand;
  
 private:
  
  int GetTargetIndexFromFiducialID(const char* fiducialID);

  int CreateCoverageVolume();
  void DeleteCoverageVolume();
  int UpdateCoverageVolumeImage();

  // 7/3/2012 ayamada
  // move to public 
  //vtkMRMLRobotNode* GetRobotNode();

  vtkRobotProbeNavGUI* GUI;

  /*
  bool  Connected;
  bool  RealtimeImageUpdate;
  */

  int   TimerOn;

};

#endif


  
