/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/

#ifndef __vtkRobotProbeNavTargetingStep_h
#define __vtkRobotProbeNavTargetingStep_h

#include "vtkRobotProbeNavStep.h"

class vtkKWEntry;
class vtkKWEntrySet;
class vtkKWEntryWithLabel;
class vtkKWFrame;
class vtkKWLabel;
class vtkSlicerNodeSelectorWidget;
class vtkKWMatrixWidgetWithLabel;
class vtkKWMenuButton;
class vtkKWMenuButtonWithLabel;
class vtkKWMultiColumnList;
class vtkKWMultiColumnListWithScrollbars;
class vtkKWPushButton;

// 12/8/2011 ayamada
class vtkKWScaleWithEntry;

class vtkKWText;
class vtkImageData;
class vtkMRMLScalarVolumeNode;
class vtkMRMLSelectionNode;
class vtkMRMLFiducialListNode;
class vtkKWCheckButton;

class VTK_RobotProbeNAV_EXPORT vtkRobotProbeNavTargetingStep : public vtkRobotProbeNavStep
{
public:
  static vtkRobotProbeNavTargetingStep *New();
  vtkTypeRevisionMacro(vtkRobotProbeNavTargetingStep,vtkRobotProbeNavStep);
  void PrintSelf(ostream& os, vtkIndent indent);

  virtual void ShowUserInterface();
  virtual void HideUserInterface();
  virtual void TearDownGUI();
  virtual void ProcessGUIEvents(vtkObject *caller, unsigned long event, void *callData);  
  virtual void ProcessMRMLEvents(vtkObject *caller, unsigned long event, void *callData);

  virtual void UpdateGUI();

  void AddMRMLObservers();
  void RemoveMRMLObservers();

  void OnMultiColumnListUpdate(int row, int col, char * str);
  void OnMultiColumnListSelection();
  void UpdateTargetListGUI();

  void SetShowTargetOrientation(int show);

  // 1/26/2012 ayamada
  //----------------------------------------------------------------
  // Timer
  //----------------------------------------------------------------
  int TimerFlag;
  int TimerInterval;
  
  void ProcessTimerEvents();
  
  // 7/5/2012 ayamada
  float carriagePosition[3];
  int orderOfNeedlePlacement[3];
  
  // 8/19/2012 ayamada
  int RobotControllerStatus;
  
  
protected:
  vtkRobotProbeNavTargetingStep();
  ~vtkRobotProbeNavTargetingStep();


  void ShowTargetPlanningFrame();
  void ShowTargetListFrame();
  void ShowTargetControlFrame();

  void EnableAddTargetsOnClickButton(bool enable);
  
  // 9/25/2011 ayamada
  void EnableShowAllNeedlePathsButton(bool enable);

  // 11/18/2011 ayamada
  void EnableShowRobotButton(bool enable);
  
  
  unsigned int PopulateListWithTargetDetails(unsigned int targetDescIndex);

  void AddGUIObservers();
  void RemoveGUIObservers();  
  
  //BTX
  // Description:
  // The column orders in the list box
  enum
    {
    TargetNumberColumn = 0,
    NeedleTypeColumn = 1,
    RASLocationColumn = 2,
    ReachableColumn = 3,
    RotationColumn = 4,
    NeedleAngleColumn = 5,    
    DepthColumn = 6,
    NumberOfColumns = 7,
    };
    
  //ETX

  bool ProcessingCallback;

  vtkKWFrame *MainFrame;
  
  // TargetPlanning
  vtkKWFrame *TargetPlanningFrame;
  vtkKWPushButton* LoadTargetingVolumeButton;
  vtkSlicerNodeSelectorWidget* VolumeSelectorWidget;
  vtkKWFrame *LoadVolumeDialogFrame;
  vtkKWCheckButton *ShowWorkspaceButton;  
  vtkKWCheckButton *ShowRobotButton;  
  vtkKWCheckButton *AddTargetsOnClickButton;
  
  // 9/25/2011 ayamada
  vtkKWCheckButton *ShowAllNeedlePathsButton;
  
  vtkKWMenuButtonWithLabel *NeedleTypeMenuList;  
  vtkMRMLFiducialListNode *TargetPlanListNode;

  // TargetList frame
  vtkKWFrame *TargetListFrame;
  vtkKWMultiColumnListWithScrollbars* TargetList;
  vtkKWPushButton *DeleteButton;

  // TargetControl frame
  vtkKWFrame *TargetControlFrame;
  vtkKWMatrixWidgetWithLabel* NeedlePositionMatrix;
  vtkKWMatrixWidgetWithLabel* NeedleOrientationMatrix;
  vtkKWPushButton *MoveButton;
  vtkKWPushButton *StopButton;

  vtkKWText *Message;
  vtkKWText *Message2;

  int ShowTargetOrientation;
  
  // 12/8/2011 ayamada
  vtkKWScaleWithEntry *needlePathAdjuster;
  vtkKWScaleWithEntry *needlePathAdjuster2;
  
  // 1/26/2012 ayamada
  int robotStatusSwitcher;
  int commandStatus;
  int robotStatus;
    
  float/*int*/ needlePathAdjusterValue;
  float/*int*/ needlePathAdjusterValue2;
  
  float previousPositionX;
  float previousPositionY;
  float previousPositionZ;
  float stopSignal;
  

private:
  vtkRobotProbeNavTargetingStep(const vtkRobotProbeNavTargetingStep&);
  void operator=(const vtkRobotProbeNavTargetingStep&);  
};

#endif
