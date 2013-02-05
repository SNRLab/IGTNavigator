/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/

#ifndef __vtkRobotProbeNavStepSetUpTemplate_h
#define __vtkRobotProbeNavStepSetUpTemplate_h

#include "vtkRobotProbeNavStep.h"
#include "vtkCommand.h"

class vtkKWLoadSaveButtonWithLabel;
class vtkKWFrame;
class vtkKWEntry;
class vtkKWCheckButton;
class vtkKWPushButton;
class vtkKWLabel;
class vtkSlicerNodeSelectorWidget;

class VTK_RobotProbeNAV_EXPORT vtkRobotProbeNavStepSetUpTemplate :
  public vtkRobotProbeNavStep
{
public:
  static vtkRobotProbeNavStepSetUpTemplate *New();
  vtkTypeRevisionMacro(vtkRobotProbeNavStepSetUpTemplate,vtkRobotProbeNavStep);
  void PrintSelf(ostream& os, vtkIndent indent);

  virtual void ShowUserInterface();
  virtual void ProcessGUIEvents(vtkObject *caller, unsigned long event, void *callData);  
  
protected:
  vtkRobotProbeNavStepSetUpTemplate();
  ~vtkRobotProbeNavStepSetUpTemplate();

  // GUI Widgets
 
  vtkSlicerNodeSelectorWidget* ScannerConnectorSelector;

  vtkKWFrame *ConnectorFrame;

private:
  vtkRobotProbeNavStepSetUpTemplate(const vtkRobotProbeNavStepSetUpTemplate&);
  void operator=(const vtkRobotProbeNavStepSetUpTemplate&);
};

#endif
