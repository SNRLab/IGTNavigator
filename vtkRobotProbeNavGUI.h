/*=auto=========================================================================

  Portions (c) Copyright 2007 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $RCSfile: $ 
  Date:      $Date: $
  Version:   $Revision: $

=========================================================================auto=*/

#ifndef __vtkRobotProbeNavGUI_h
#define __vtkRobotProbeNavGUI_h

#ifdef WIN32
#include "vtkRobotProbeNavWin32Header.h"
#endif

#include "vtkSlicerModuleGUI.h"
#include "vtkRobotProbeNavLogic.h"

#include "vtkCallbackCommand.h"
#include "vtkSlicerInteractorStyle.h"

#include <string>
#include <list>

//#include "vtkRobotProbeNavStepSetUp.h"

// 8/4/2011 ayamada
#include "vtkKWEntry.h"
#include "vtkKWEntryWithLabel.h"
#include "vtkKWPushButton.h"
#include "vtkKWFrameWithLabel.h"
#include "vtkKWMultiColumnList.h"
#include "vtkKWMultiColumnListWithScrollbars.h"

// 8/16/2011 ayamada
#include "vtkSlicerInteractorStyle.h"
#include "vtkSlicerApplicationGUI.h"
#include "vtkRobotProbeNavGUI.h"


class vtkKWPushButton;
class vtkKWPushButtonSet;
class vtkKWEntryWithLabel;
class vtkKWMenuButtonWithLabel;
class vtkKWMenuButton;
class vtkKWCheckButton;
class vtkKWScaleWithEntry;
class vtkKWEntry;
class vtkKWFrame;
class vtkKWEntryWithLabel;
class vtkKWLoadSaveButtonWithLabel;
class vtkKWMultiColumnListWithScrollbars;
class vtkKWWizardWidget;
class vtkSlicerNodeSelectorWidget;

class vtkRobotProbeNavStep;
class vtkSlicerSecondaryViewerWindow;

class vtkMRMLRobotProbeNavManagerNode;
class vtkMRMLFiducialListNode;

// 8/18/2011 ayamada     
class vtkRobotProbeNavStepSetUp;

// 9/8/2011 ayamada
class vtkKWLoadSaveButtonWithLabel;


 
// Description:    
// This class implements Slicer's Volumes GUI
//
class VTK_RobotProbeNAV_EXPORT vtkRobotProbeNavGUI : public vtkSlicerModuleGUI
{
 public:
 
  // 3/8/2012 ayamada
  //BTX
  //std::string ModuleShareDirectory; // needed for model files, etc.
  //ETX
  
  
  virtual void Register(vtkObject *o) { Superclass::Register(o); };
  virtual void UnRegister(vtkObject *o) { Superclass::UnRegister(o); };

  //BTX
  enum {
    SLICE_PLANE_RED    = 0,
    SLICE_PLANE_YELLOW = 1,
    SLICE_PLANE_GREEN  = 2
  };
  enum {
    SLICE_RTIMAGE_NONE      = 0,
    SLICE_RTIMAGE_PERP      = 1,
    SLICE_RTIMAGE_INPLANE90 = 2,
    SLICE_RTIMAGE_INPLANE   = 3
  };
  enum {
    BRING_MARKERS_TO_VIEW_KEEP_CURRENT_ORIENTATION, // show slices in their original (acquisition) directions
    BRING_MARKERS_TO_VIEW_ALIGN_TO_NEEDLE           // show needle aligned slices (parallel and perpendicular to the needle and robot main axis)
  };
  
  //ETX
  
  // Precision of the target position and orientation display
  static const int POSITION_PRECISION_DIGITS;
  static const double POSITION_PRECISION_TOLERANCE;


  
 public:
  // Description:    
  // Usual vtk class functions
  static vtkRobotProbeNavGUI* New (  );
  vtkTypeRevisionMacro ( vtkRobotProbeNavGUI, vtkSlicerModuleGUI );
  void PrintSelf (ostream& os, vtkIndent indent );
  
  //SendDATANavitrack
  // Description:    
  // Get methods on class members (no Set methods required)
  vtkGetObjectMacro ( Logic, vtkRobotProbeNavLogic );
  
  // Description:
  // API for setting VolumeNode, VolumeLogic and
  // for both setting and observing them.
  void SetModuleLogic ( vtkSlicerLogic *logic )
  { this->SetLogic ( vtkObjectPointer (&this->Logic), logic ); }
  void SetAndObserveModuleLogic ( vtkRobotProbeNavLogic *logic )
  { this->SetAndObserveLogic ( vtkObjectPointer (&this->Logic), logic ); }
  // Description: 
  // Get wizard widget
  vtkGetObjectMacro(WizardWidget, vtkKWWizardWidget);

  // Description:    
  // This method builds the IGTDemo module GUI
  virtual void BuildGUI ( );

  // Description:    
  // This method builds the IGTDemo module GUI
  virtual void TearDownGUI ( );
  
  // Description:
  // Add/Remove observers on widgets in the GUI
  virtual void AddGUIObservers ( );
  virtual void RemoveGUIObservers ( );

  void AddLogicObservers ( );
  void RemoveLogicObservers ( );

  void AddMRMLObservers();
  void RemoveMRMLObservers();

  // Description: 
  // Get the categorization of the module.
  const char *GetCategory() const { return "IGT"; }
  
  // Description:
  // Class's mediator methods for processing events invoked by
  // either the Logic, MRML or GUI.    
  virtual void ProcessLogicEvents ( vtkObject *caller, unsigned long event, void *callData );
  virtual void ProcessGUIEvents ( vtkObject *caller, unsigned long event, void *callData );
  virtual void ProcessMRMLEvents ( vtkObject *caller, unsigned long event, void *callData );

  // 8/16/2011 ayamada
  //void HandleMouseEvent(vtkSlicerInteractorStyle *style);
  //vtkRobotProbeNavStepSetUp* setupStep;
  void HandleMouseEvent(vtkSlicerInteractorStyle *style);//, vtkRobotProbeNavStepSetUp *setupStep);
  
  // 9/8/2011 ayamada
  vtkKWLoadSaveButtonWithLabel* SelectFiducialsFileButton;
  
  // 9/21/2011 ayamada
  vtkKWCheckButton* ShowRobotCheckButton;
  vtkActor* polyActor;
  vtkCamera* polyActorCamera;
  vtkPolyDataMapper* polyMapper;
  vtkMRMLModelDisplayNode* ModelDisplayNode;
  
  // 9/13/2011 ayamada
  int testImageSender;
  
  // Description:
  // Describe behavior at module startup and exit.
  virtual void Enter (vtkMRMLNode *node);
  virtual void Enter ();
  virtual void Exit ( );
  
  void Init();

  //BTX
  static void DataCallback(vtkObject *caller, 
                           unsigned long eid, void *clientData, void *callData);
  
  //ETX
  
  vtkMRMLRobotProbeNavManagerNode* GetRobotProbeNavManagerNode();

  // Description:
  // Bring a marker to view in all three slice views along its principal axes
  // N - the direction vector of the locator,
  // T - the transverse direction vector of the locator (optional)
  // P - the tip location of the locator (optional)
  // All the above values are in RAS space. 
  void BringMarkerToViewIn2DViews(double* P, double* N=NULL, double* T=NULL);

  // Description:
  // Bring current target to view in all three slice views
  void BringTargetToViewIn2DViews(int mode);

  // Description:
  // Request render in all viewer widgets
  void RequestRenderInViewerWidgets();
  void ShowSecondaryWindowCheckButtonCallback (int checked);

  
  // 8/16/2011 ayamada
  vtkKWEntryWithLabel *SlicerCoordinatesEntry;
  int numOfRows;
  //vtkRobotProbeNavStepSetUp* setupStep;// = vtkRobotProbeNavStepSetUp::New();

  // 9/8/2011 ayamada
  int LoadCSVTrackingFile(const char * path);

  // 12/28/2011 ayamada
  int LoadCSVTrackingFile2(const char * path);
  
  // 12/26/2011 ayamada
  //int workPhaseStatus;
  
  // 12/28/2011 ayamada
  double pX[50];
  double pY[50];
  double pZ[50];

  double cX[50];
  double cY[50];
  double cZ[50];
  

 protected:
  vtkRobotProbeNavGUI ( );
  virtual ~vtkRobotProbeNavGUI ( );
  
  //void SetRobotProbeNavManager(vtkMRMLRobotProbeNavManagerNode* node);
  void SetRobot(vtkMRMLRobotNode* robot);
  void SetTargetPlanList(vtkMRMLFiducialListNode* targetPlanList);

  // Return i-th worfklow step page
  vtkRobotProbeNavStep* GetStepPage(int i);

  //----------------------------------------------------------------
  // GUI widgets
  //----------------------------------------------------------------

  vtkSlicerSecondaryViewerWindow* SecondaryWindow;

  // Configuration Frame

  vtkKWCheckButton *ShowSecondaryWindowCheckButton;
  vtkSlicerNodeSelectorWidget* RobotProbeNavManagerSelectorWidget;
  vtkSlicerNodeSelectorWidget* RobotSelectorWidget;


  // 8/18/2011 ayamada
  //----------------------------------------------------------------
  // Registration Frame
  // 8/4/2011 ayamada
  vtkKWEntryWithLabel* FiducialMarkersSetup;
  vtkKWPushButton* AddFiducialMarkerPushButton;
  vtkKWFrameWithLabel* ListFrame;
  // 8/16/2011 ayamada
  vtkKWMultiColumnListWithScrollbars* PointPairMultiColumnList;
  int FiducialNumber;
  int FiducialNumber2;
  vtkKWPushButton* FixFiducialMarkersPushButton;
  vtkKWPushButton* DeletePointPairPushButton;
  vtkKWPushButton* DeleteAllPointPairPushButton;
  vtkKWFrameWithLabel* FiducialFrame;

  // 9/8/2011 ayamada
  vtkKWFrameWithLabel* ListFrame2;
  vtkKWMultiColumnListWithScrollbars* PointPairMultiColumnList2;
  vtkKWPushButton* DeletePointPairPushButton2;
  vtkKWPushButton* DeleteAllPointPairPushButton2;
  vtkKWPushButton* AddFiducialFilePushButton;
  vtkKWFrameWithLabel* FiducialFrame2;
  
  //----------------------------------------------------------------
  // Workphase Frame
  
  vtkKWFrame *StatusButtonFrame;
  vtkKWPushButtonSet *StatusButtonSet;

  vtkKWFrame *WorkphaseButtonFrame;
  vtkKWPushButtonSet *WorkphaseButtonSet;
  
  //----------------------------------------------------------------
  // Wizard Frame

  vtkSlicerModuleCollapsibleFrame *WizardFrame;
  vtkKWWizardWidget *WizardWidget;
  
  //----------------------------------------------------------------
  // Logic Values
  //----------------------------------------------------------------

  vtkRobotProbeNavLogic *Logic;

  vtkCallbackCommand *DataCallbackCommand;

  //----------------------------------------------------------------
  // Target Fiducials
  //----------------------------------------------------------------

  void UpdateGUI();  


 private:

  vtkRobotProbeNavGUI ( const vtkRobotProbeNavGUI& ); // Not implemented.
  void operator = ( const vtkRobotProbeNavGUI& ); //Not implemented.

  void BuildGUIForConfigurationFrame();
  void BuildGUIForWorkphaseFrame();
  void BuildGUIForWizardFrame();
  void BuildGUIForHelpFrame();
  
  // 8/18/2011 ayamada
  void BuildGUIForRegistrationFrame();
  // 9/21/2011 ayamada
  void BuildGUIForTargetingFrame();

  void UpdateStatusButtons();
  void UpdateWorkflowSteps();
  
  int  ChangeWorkphaseInGUI(int phase);
  const char* AddZFrameModel(const char* nodeName);

  // Description:
  // Display current target fiducial highlighted
  void UpdateCurrentTargetDisplay();
  void UpdateCurrentTargetDisplayInSecondaryWindow();

  void SetAndObserveRobotNodeID(const char *nodeID);
  vtkMRMLRobotNode* GetRobotNode();

  void SetAndObserveRobotProbeNavManagerNodeID(const char *nodeID);  

  void SetAndObserveTargetPlanListNodeID(const char *nodeID);
  vtkMRMLFiducialListNode* GetTargetPlanListNode();
  

  int Entered;

  // store the currently displayed workflow steps
  // if the same steps requested to be displayed, then nothing will happen
  vtkStringArray* DisplayedWorkflowSteps;

  vtkSetStringMacro(RobotProbeNavManagerNodeID);
  char* RobotProbeNavManagerNodeID;
  vtkMRMLRobotProbeNavManagerNode* RobotProbeNavManagerNode;

  vtkSetStringMacro(RobotNodeID);
  char* RobotNodeID;
  vtkMRMLRobotNode* RobotNode;

  vtkSetStringMacro(TargetPlanListNodeID);
  char* TargetPlanListNodeID;
  vtkMRMLFiducialListNode* TargetPlanListNode;  
};



#endif
