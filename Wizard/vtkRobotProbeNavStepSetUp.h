/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/

#ifndef __vtkRobotProbeNavStepSetUp_h
#define __vtkRobotProbeNavStepSetUp_h

#include "vtkRobotProbeNavStep.h"
#include "vtkCommand.h"


class vtkKWLoadSaveButtonWithLabel;
class vtkKWFrame;
class vtkKWEntry;
class vtkKWCheckButton;
class vtkKWPushButton;
class vtkKWLabel;
class vtkSlicerNodeSelectorWidget;
class vtkMRMLTransPerinealRobotProbeRobotNode;

class VTK_RobotProbeNAV_EXPORT vtkRobotProbeNavStepSetUp :
  public vtkRobotProbeNavStep
{
public:
  static vtkRobotProbeNavStepSetUp *New();
  vtkTypeRevisionMacro(vtkRobotProbeNavStepSetUp,vtkRobotProbeNavStep);
  void PrintSelf(ostream& os, vtkIndent indent);

  virtual void ShowUserInterface();
  virtual void ProcessGUIEvents(vtkObject *caller, unsigned long event, void *callData);  

  // 12/28/2011 ayamada
  vtkKWLoadSaveButtonWithLabel *SelectConfigurationFileButton;
  vtkKWPushButton *AddConfigurationFilePushButton;
  int FiducialNumber;
  //vtkKWMultiColumnListWithScrollbars* PointPairMultiColumnList;
  
protected:
  vtkRobotProbeNavStepSetUp();
  ~vtkRobotProbeNavStepSetUp();

  vtkMRMLTransPerinealRobotProbeRobotNode* GetRobotNode();

  // GUI Widgets
 
  vtkSlicerNodeSelectorWidget* RobotConnectorSelector;
  vtkSlicerNodeSelectorWidget* ScannerConnectorSelector;
  
    
  vtkKWFrame *ConnectorFrame;

private:
  vtkRobotProbeNavStepSetUp(const vtkRobotProbeNavStepSetUp&);
  void operator=(const vtkRobotProbeNavStepSetUp&);
};

#endif
