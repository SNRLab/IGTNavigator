/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/

#include "vtkRobotProbeNavStep.h"
#include "vtkRobotProbeNavGUI.h"
#include "vtkRobotProbeNavLogic.h"

#include "vtkKWWizardWidget.h"
#include "vtkKWWizardWorkflow.h"
#include "vtkObserverManager.h"
#include "vtkMRMLFiducialListNode.h"

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkRobotProbeNavStep);
vtkCxxRevisionMacro(vtkRobotProbeNavStep, "$Revision: 1.2 $");
vtkCxxSetObjectMacro(vtkRobotProbeNavStep,GUI,vtkRobotProbeNavGUI);
vtkCxxSetObjectMacro(vtkRobotProbeNavStep,Logic,vtkRobotProbeNavLogic);

//----------------------------------------------------------------------------
vtkRobotProbeNavStep::vtkRobotProbeNavStep()
{

  //std::cerr << "vtkRobotProbeNavStep::vtkRobotProbeNavStep() start" << std::endl;
  this->GUI = NULL;
  this->Logic = NULL;
  this->MRMLScene = NULL;
  this->RobotProbeNavManager = NULL;

  this->GUICallbackCommand = vtkCallbackCommand::New();
  this->GUICallbackCommand->SetClientData( reinterpret_cast<void *>(this) );
  this->GUICallbackCommand->SetCallback(&vtkRobotProbeNavStep::GUICallback);

  this->MRMLObserverManager = vtkObserverManager::New();
  this->MRMLObserverManager->GetCallbackCommand()->SetClientData( reinterpret_cast<void *> (this) );
  this->MRMLObserverManager->GetCallbackCommand()->SetCallback(vtkRobotProbeNavStep::MRMLCallback);
  this->MRMLCallbackCommand = this->MRMLObserverManager->GetCallbackCommand();

  this->TitleBackgroundColor[0] = 0.8;
  this->TitleBackgroundColor[1] = 0.8;
  this->TitleBackgroundColor[2] = 1.0;

  this->InGUICallbackFlag = 0;
  this->InMRMLCallbackFlag = 0;

  //std::cerr << "vtkRobotProbeNavStep::vtkRobotProbeNavStep() end" << std::endl;
}

//----------------------------------------------------------------------------
vtkRobotProbeNavStep::~vtkRobotProbeNavStep()
{
  this->SetAndObserveMRMLScene ( NULL );

  if (this->MRMLObserverManager)
    {
    this->MRMLObserverManager->RemoveAllObservers();
    this->MRMLObserverManager->Delete();
    }    

  if ( this->GUICallbackCommand != NULL )
    {
    this->GUICallbackCommand->Delete ( );
    this->GUICallbackCommand = NULL;
    }

  this->SetGUI(NULL);
  this->SetLogic(NULL);

}

//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::HideUserInterface()
{
  this->Superclass::HideUserInterface();

  this->SetAndObserveMRMLScene(NULL);

  if (this->GetGUI())
    {
    this->GetGUI()->GetWizardWidget()->ClearPage();
    }
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::Validate()
{
  this->Superclass::Validate();

  vtkKWWizardWorkflow *wizardWorkflow = 
    this->GetGUI()->GetWizardWidget()->GetWizardWorkflow();

  wizardWorkflow->PushInput(vtkKWWizardStep::GetValidationSucceededInput());
  wizardWorkflow->ProcessInputs();
}

//----------------------------------------------------------------------------
int vtkRobotProbeNavStep::CanGoToSelf()
{
  return this->Superclass::CanGoToSelf() || 1;
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::ShowUserInterface()
{
  this->Superclass::ShowUserInterface();
  
  if (!this->MRMLScene)
    {
    this->SetAndObserveMRMLScene (this->GetGUI()->GetMRMLScene());
    }

  vtkKWWizardWidget *wizardWidget = this->GetGUI()->GetWizardWidget();
  wizardWidget->GetCancelButton()->SetEnabled(0);
  wizardWidget->SetTitleAreaBackgroundColor(this->TitleBackgroundColor[0],
                                            this->TitleBackgroundColor[1],
                                            this->TitleBackgroundColor[2]);

}


//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::GUICallback( vtkObject *caller,
                           unsigned long eid, void *clientData, void *callData )
{

  vtkRobotProbeNavStep *self = reinterpret_cast<vtkRobotProbeNavStep *>(clientData);
  
  if (self->GetInGUICallbackFlag())
    {
    }

  vtkDebugWithObjectMacro(self, "In vtkRobotProbeNavStep GUICallback");
  
  self->SetInGUICallbackFlag(1);
  self->ProcessGUIEvents(caller, eid, callData);
  self->SetInGUICallbackFlag(0);
  
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::MRMLCallback(vtkObject *caller, 
                                    unsigned long eid, void *clientData, void *callData)
{

  vtkRobotProbeNavStep *self = reinterpret_cast<vtkRobotProbeNavStep *>(clientData);
  
  if (self->GetInMRMLCallbackFlag())
    {
    return;
    }

  vtkDebugWithObjectMacro(self, "In vtkRobotProbeNavStep MRMLCallback");
  
  self->SetInMRMLCallbackFlag(1);
  self->ProcessMRMLEvents(caller, eid, callData);
  self->SetInMRMLCallbackFlag(0);
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::UpdateName()
{
  std::stringstream ss;
  ss << this->StepNumber << " / " << this->TotalSteps << ". " << this->Title;
  this->SetName(ss.str().c_str());
  this->Modified();
}

void vtkRobotProbeNavStep::TearDownGUI()
{
  // Override in child classes  
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::ShowWorkspaceModel(bool show)
{
  vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();
  if (!logic)
  {
    vtkErrorMacro("Invalid logic object");
    return;
  }
  logic->ShowWorkspaceModel(show);
}

//----------------------------------------------------------------------------
bool vtkRobotProbeNavStep::IsWorkspaceModelShown()
{
  vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();
  if (!logic)
  {
    vtkErrorMacro("Invalid logic object");
    return false;
  }
  return logic->IsWorkspaceModelShown();
}

// 9/25/2011 ayamada
//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::ShowNeedleModel(bool show)
{
  vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();
  if (!logic)
  {
    vtkErrorMacro("Invalid logic object");
    return;
  }
  logic->ShowNeedleModel(show);

  // 11/4/2011 ayamada
  logic->ShowVirtualCenterModel(show);
}

// 11/4/2011 ayamada
//----------------------------------------------------------------------------
bool vtkRobotProbeNavStep::IsVirtualCenterModelShown()
{
  vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();
  if (!logic)
  {
    vtkErrorMacro("Invalid logic object");
    return false;
  }
  return logic->IsVirtualCenterModelShown();
}



// 9/25/2011 ayamada
//----------------------------------------------------------------------------
bool vtkRobotProbeNavStep::IsNeedleModelShown()
{
  vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();
  if (!logic)
  {
    vtkErrorMacro("Invalid logic object");
    return false;
  }
  return logic->IsNeedleModelShown();
}

// 10/24/2011 ayamada
// see vtkRobotProbeNavTragetingStep.cxx l.1415
// see vtkRobotProbeNavLogic.cxx l.901~
bool vtkRobotProbeNavStep::SendTargetingData(vtkKWMatrixWidget* matrix, vtkKWMatrixWidget* oMatrix, int numRows)
{
  vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();
  if (!logic)
  {
    vtkErrorMacro("Invalid logic object");
    return false;
  }
  return logic->CorrectNeedlePosition(matrix, oMatrix, numRows);  
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavStep::ShowRobotModel(bool show)
{
  vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();
  if (!logic)
  {
    vtkErrorMacro("Invalid logic object");
    return;
  }
  logic->ShowRobotModel(show);
}

//----------------------------------------------------------------------------
bool vtkRobotProbeNavStep::IsRobotModelShown()
{
  vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();
  if (!logic)
  {
    vtkErrorMacro("Invalid logic object");
    return false;
  }
  return logic->IsRobotModelShown();
}
