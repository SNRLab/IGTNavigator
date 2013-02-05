/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/

#ifndef __vtkRobotProbeNavManualControlStep_h
#define __vtkRobotProbeNavManualControlStep_h

#include "vtkRobotProbeNavStep.h"

class vtkKWFrame;
class vtkKWScaleWithEntry;

class VTK_RobotProbeNAV_EXPORT vtkRobotProbeNavManualControlStep : public vtkRobotProbeNavStep
{
public:
  static vtkRobotProbeNavManualControlStep *New();
  vtkTypeRevisionMacro(vtkRobotProbeNavManualControlStep,vtkRobotProbeNavStep);
  void PrintSelf(ostream& os, vtkIndent indent);

  virtual void ShowUserInterface();
  virtual void ProcessGUIEvents(vtkObject *caller, unsigned long event, void *callData);

protected:
  vtkRobotProbeNavManualControlStep();
  ~vtkRobotProbeNavManualControlStep();

  // GUI Widgets
  vtkKWFrame *MainFrame;
  vtkKWFrame *ControlFrame;

  vtkKWScaleWithEntry* PRScale;
  vtkKWScaleWithEntry* PAScale;
  vtkKWScaleWithEntry* PSScale;
  vtkKWScaleWithEntry* NRScale;
  vtkKWScaleWithEntry* NAScale;
  vtkKWScaleWithEntry* NSScale;

private:
  vtkRobotProbeNavManualControlStep(const vtkRobotProbeNavManualControlStep&);
  void operator=(const vtkRobotProbeNavManualControlStep&);
};

#endif
