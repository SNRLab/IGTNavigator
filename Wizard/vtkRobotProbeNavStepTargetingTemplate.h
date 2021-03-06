/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/

#ifndef __vtkRobotProbeNavStepTargetingTemplate_h
#define __vtkRobotProbeNavStepTargetingTemplate_h

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
class vtkKWText;
class vtkImageData;
class vtkMRMLScalarVolumeNode;
class vtkMRMLSelectionNode;
class vtkMRMLFiducialListNode;
class vtkKWCheckButton;

class VTK_RobotProbeNAV_EXPORT vtkRobotProbeNavStepTargetingTemplate : public vtkRobotProbeNavStep
{
public:
  static vtkRobotProbeNavStepTargetingTemplate *New();
  vtkTypeRevisionMacro(vtkRobotProbeNavStepTargetingTemplate,vtkRobotProbeNavStep);
  void PrintSelf(ostream& os, vtkIndent indent);

  virtual void ShowUserInterface();
  virtual void HideUserInterface();
  virtual void ProcessGUIEvents(vtkObject *caller, unsigned long event, void *callData);  
  virtual void ProcessMRMLEvents(vtkObject *caller, unsigned long event, void *callData);

  virtual void UpdateGUI();

  void AddMRMLObservers();
  void RemoveMRMLObservers();

  void OnMultiColumnListUpdate(int row, int col, char * str);
  void OnMultiColumnListSelectionChanged();
  void UpdateTargetListGUI();

protected:
  vtkRobotProbeNavStepTargetingTemplate();
  ~vtkRobotProbeNavStepTargetingTemplate();


  void ShowTargetPlanningFrame();
  void ShowTargetListFrame();
  void ShowTargetControlFrame();

  void ShowNeedle(bool show);
  void ShowTemplate(bool show);

  void EnableAddTargetsOnClickButton(bool enable);

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
  //vtkKWPushButton* LoadTargetingVolumeButton;
  vtkSlicerNodeSelectorWidget* VolumeSelectorWidget;
  vtkSlicerNodeSelectorWidget* TargetListSelectorWidget;
  vtkKWFrame *LoadVolumeDialogFrame;
  vtkKWCheckButton *ShowWorkspaceButton;
  vtkKWCheckButton *AddTargetsOnClickButton;
  vtkKWCheckButton *ShowNeedleButton;
  vtkKWCheckButton *ShowTemplateButton;
  vtkKWMenuButtonWithLabel *NeedleTypeMenuList;
  vtkMRMLFiducialListNode *TargetPlanListNode;
  vtkKWFrame *OptionFrame;

  // TargetList frame
  vtkKWFrame *TargetListFrame;
  vtkKWMultiColumnListWithScrollbars* TargetList;
  vtkKWPushButton *DeleteButton;

  // TargetControl frame
  vtkKWFrame *TargetControlFrame;
  vtkKWMatrixWidgetWithLabel* NeedlePositionMatrix;
  vtkKWMatrixWidgetWithLabel* NeedleOrientationMatrix;

  //vtkKWPushButton *MoveButton;
  //vtkKWPushButton *StopButton;
  vtkKWPushButton *GenerateListButton;

  vtkKWText *Message;

private:
  vtkRobotProbeNavStepTargetingTemplate(const vtkRobotProbeNavStepTargetingTemplate&);
  void operator=(const vtkRobotProbeNavStepTargetingTemplate&);
};

#endif
