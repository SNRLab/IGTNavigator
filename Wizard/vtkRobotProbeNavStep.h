/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/

#ifndef __vtkRobotProbeNavStep_h
#define __vtkRobotProbeNavStep_h

#include "vtkKWWizardStep.h"
#include "vtkRobotProbeNav.h"
#include "vtkCommand.h"

#include "vtkObserverManager.h"
#include "vtkMRMLRobotProbeNavManagerNode.h"

// 10/19/2011 ayamada
#include "vtkKWMatrixWidget.h"

class vtkRobotProbeNavGUI;
class vtkRobotProbeNavLogic;
class vtkMRMLScene;

class VTK_RobotProbeNAV_EXPORT vtkRobotProbeNavStep : public vtkKWWizardStep
{
public:
  static vtkRobotProbeNavStep *New();
  vtkTypeRevisionMacro(vtkRobotProbeNavStep,vtkKWWizardStep);
  void PrintSelf(ostream& os, vtkIndent indent);

  virtual void Register(vtkObject *o) { Superclass::Register(o); };
  virtual void UnRegister(vtkObjectBase *o) { Superclass::UnRegister(o); };

  // Description: 
  // Get/Set GUI
  vtkGetObjectMacro(GUI, vtkRobotProbeNavGUI);
  vtkGetObjectMacro(Logic, vtkRobotProbeNavLogic);

  virtual void SetGUI(vtkRobotProbeNavGUI*);
  virtual void SetLogic(vtkRobotProbeNavLogic*);
  virtual void TearDownGUI();

  vtkSetObjectMacro(RobotProbeNavManager, vtkMRMLRobotProbeNavManagerNode);
  vtkGetObjectMacro(RobotProbeNavManager, vtkMRMLRobotProbeNavManagerNode);

  vtkSetMacro(TotalSteps, int);
  vtkGetMacro(TotalSteps, int);
  vtkSetMacro(StepNumber, int);
  vtkGetMacro(StepNumber, int);

  void SetTitle(const char* title) {
    this->Title = title;
  }

  const char* GetTitle() {
    return this->Title.c_str();
  }

  void UpdateName();


  void SetInMRMLCallbackFlag (int flag) {
    this->InMRMLCallbackFlag = flag;
  }
  vtkGetMacro(InMRMLCallbackFlag, int);
  void SetInGUICallbackFlag (int flag) {
    this->InGUICallbackFlag = flag;
    }
  vtkGetMacro(InGUICallbackFlag, int);

  void SetAndObserveMRMLScene ( vtkMRMLScene *mrml )
    {
    vtkMRMLScene *oldValue = this->MRMLScene;
    this->MRMLObserverManager->SetAndObserveObject ( vtkObjectPointer( &this->MRMLScene), (vtkObject*)mrml );
    if ( oldValue != this->MRMLScene )
      {
      this->InvokeEvent (vtkCommand::ModifiedEvent);
      }
    }

  void SetAndObserveMRMLSceneEvents ( vtkMRMLScene *mrml, vtkIntArray *events )
    {
    vtkObject *oldValue = this->MRMLScene;
    this->MRMLObserverManager->SetAndObserveObjectEvents ( vtkObjectPointer( &this->MRMLScene), mrml, events );
    if ( oldValue != this->MRMLScene )
      {
      this->InvokeEvent (vtkCommand::ModifiedEvent);
      }
    }


  void SetTitleBackgroundColor (double r, double g, double b) {
    this->TitleBackgroundColor[0] = r;
    this->TitleBackgroundColor[1] = g;
    this->TitleBackgroundColor[2] = b;
  };

  void GetTitleBackgroundColor (double* r, double* g, double* b) {
    *r = this->TitleBackgroundColor[0];
    *g = this->TitleBackgroundColor[1];
    *b = this->TitleBackgroundColor[2];
  };

  // Description:
  // Reimplement the superclass's method (see vtkKWWizardStep).
  virtual void HideUserInterface();
  virtual void Validate();
  virtual int CanGoToSelf();
  virtual void ShowUserInterface();
  virtual void ProcessGUIEvents(vtkObject *caller, unsigned long event, void *callData) {};
  virtual void ProcessMRMLEvents(vtkObject *caller, unsigned long event, void *callData) {};

  virtual void UpdateGUI() {};

  void ShowWorkspaceModel(bool show);
  bool IsWorkspaceModelShown();
  void ShowRobotModel(bool show);
  bool IsRobotModelShown();
  
  // 9/25/2011 ayamaada
  void ShowNeedleModel(bool show);
  bool IsNeedleModelShown();
  
  // 11/4/2011 ayamaada
  //void ShowVirtualCenterModel(bool show);
  bool IsVirtualCenterModelShown();
  
  
  // 10/19/2011 ayamada
  bool SendTargetingData(vtkKWMatrixWidget* matrix, vtkKWMatrixWidget* oMatrix, int numRows);
  

protected:
  vtkRobotProbeNavStep();
  ~vtkRobotProbeNavStep();

  static void GUICallback(vtkObject *caller,
                          unsigned long eid, void *clientData, void *callData );

  static void MRMLCallback(vtkObject *caller,
                           unsigned long eid, void *clientData, void *callData );

protected:
  
  double TitleBackgroundColor[3];

  int InGUICallbackFlag;
  int InMRMLCallbackFlag;

  vtkRobotProbeNavGUI   *GUI;
  vtkRobotProbeNavLogic *Logic;
  vtkMRMLScene        *MRMLScene;

  vtkCallbackCommand *GUICallbackCommand;
  vtkCallbackCommand *MRMLCallbackCommand;
  vtkObserverManager *MRMLObserverManager;

  vtkMRMLRobotProbeNavManagerNode* RobotProbeNavManager;

  //BTX
  std::string Title;
  //ETX
  
  int TotalSteps;     // Total number of steps in the wizard
  int StepNumber;     // Step number for this step.

private:
  vtkRobotProbeNavStep(const vtkRobotProbeNavStep&);
  void operator=(const vtkRobotProbeNavStep&);

};

#endif
