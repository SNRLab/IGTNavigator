/*=auto=========================================================================

  Portions (c) Copyright 2007 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.  

  Program:   3D Slicer
  Module:    $RCSfile: $
  Date:      $Date: $ 
  Version:   $Revision: $   

=========================================================================auto=*/

#include "vtkObject.h"
#include "vtkObjectFactory.h"
#include "vtkSmartPointer.h"

#include "vtkMRMLRobotNode.h"
#include "vtkMRMLIGTLConnectorNode.h"

#include "vtkMRMLBrpRobotCommandNode.h"
#include "vtkMRMLRobotDisplayNode.h"

#include "vtkRobotProbeNavGUI.h"
#include "vtkSlicerApplication.h"
#include "vtkSlicerModuleCollapsibleFrame.h"
#include "vtkSlicerSliceControllerWidget.h"
#include "vtkSlicerNodeSelectorWidget.h"
#include "vtkSlicerColor.h"
#include "vtkSlicerTheme.h"

#include "vtkKWWizardWidget.h"
#include "vtkKWWizardWorkflow.h"
#include "vtkRobotProbeNavStep.h"
#include "vtkRobotProbeNavStepSetUp.h"
#include "vtkRobotProbeNavStepSetUpTemplate.h"
#include "vtkRobotProbeNavStepVerification.h"
#include "vtkRobotProbeNavCalibrationStep.h"
#include "vtkRobotProbeNavFiducialCalibrationStep.h"
#include "vtkRobotProbeNavTargetingStep.h"
#include "vtkRobotProbeNavStepTargetingTemplate.h"
#include "vtkRobotProbeNavManualControlStep.h"

#include "vtkSlicerFiducialsGUI.h"
#include "vtkSlicerFiducialsLogic.h"

#include "vtkKWRenderWidget.h"
#include "vtkKWWidget.h"
#include "vtkKWMenuButton.h"
#include "vtkKWCheckButton.h"
#include "vtkKWPushButton.h"
#include "vtkKWPushButtonSet.h"
#include "vtkKWFrameWithLabel.h"
#include "vtkKWFrame.h"
#include "vtkKWLoadSaveButton.h"
#include "vtkKWLoadSaveButtonWithLabel.h"
#include "vtkKWLoadSaveDialog.h"
#include "vtkKWEntry.h"
#include "vtkKWEntryWithLabel.h"
#include "vtkKWMenuButtonWithLabel.h"
#include "vtkKWScaleWithEntry.h"
#include "vtkKWMenu.h"
#include "vtkKWLabel.h"
#include "vtkKWMultiColumnList.h"
#include "vtkKWMessageDialog.h"
#include "vtkKWMultiColumnListWithScrollbars.h"
#include "vtkKWEvent.h"
#include "vtkKWOptions.h"

#include "vtkKWTkUtilities.h"
#include "vtkMRMLModelDisplayNode.h"
#include "vtkCylinderSource.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkActor.h"
#include "vtkProperty.h"
#include "vtkCornerAnnotation.h"
#include "vtkTextProperty.h"
#include "vtkMath.h"

// for Realtime Image
#include "vtkImageChangeInformation.h"
#include "vtkSlicerColorLogic.h"
//#include "vtkSlicerVolumesGUI.h"

#include "vtkCylinderSource.h"
#include "vtkMRMLLinearTransformNode.h"

#include "vtkMRMLRobotProbeNavManagerNode.h"
#include "vtkMRMLTransRectalRobotProbeRobotNode.h"
#include "vtkMRMLIGTProbeRobotNode.h"
#include "vtkMRMLIGTProbeTemplateNode.h"
#include "vtkSlicerSecondaryViewerWindow.h"
#include "vtkSlicerViewerWidget.h"
#include "vtkMRMLViewNode.h"

// 9/21/2011 ayamada
#include "vtkSlicerModelsGUI.h"
#include "vtkSlicerModelsLogic.h"
#include "vtkMRMLModelNode.h"
#include "vtkKWMessageDialog.h"
#include "vtkMRMLModelDisplayNode.h"

#include <vector>

// Precision of the target position and orientation display
const int vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS=1;
const double vtkRobotProbeNavGUI::POSITION_PRECISION_TOLERANCE=0.1/2.0;

//---------------------------------------------------------------------------
// This default needle set description is used when no description is found in the registry.
// In this case the description is written to the registry. Once the description is written
// to the registry, it can be customized by editing the registry.
//
static const char DEFAULT_NEEDLE_DESCRIPTION[]=
  "<NeedleList DefaultNeedle=\"BIOP_TSK_14G_000\"> \
  <Needle ID=\"GEN_000\" TargetNamePrefix=\"T\" Description=\"Generic\" Length=\"150\" TipLength=\"0\" Throw=\"0\" TargetLength=\"0\" TargetBase=\"0\" LastTargetIndex=\"0\" /> \
  <Needle ID=\"BIOP_TSK_14G_000\" TargetNamePrefix=\"B\" Description=\"Biopsy TSK 14G (NIH)\" Length=\"150\" TipLength=\"4\" Throw=\"23\" TargetLength=\"16\" TargetBase=\"1.5\" Diameter=\"2.108\" LastTargetIndex=\"0\" /> \
  <Needle ID=\"SEED_DAUM_14G_000\" TargetNamePrefix=\"S\" Description=\"Seed Daum 14G (NIH)\" Length=\"150\" TipLength=\"0\" Throw=\"0\" TargetLength=\"3\" TargetBase=\"-1.5\" Diameter=\"2.108\" LastTargetIndex=\"0\" /> \
  <Needle ID=\"BIOP_TSK_18G_000\" TargetNamePrefix=\"B\" Description=\"Biopsy TSK 18G (JHH)\" Length=\"150\" TipLength=\"1.5\" Throw=\"22\" TargetLength=\"16\" TargetBase=\"2\" Diameter=\"1.270\" LastTargetIndex=\"0\" /> \
  <Needle ID=\"BIOP_TSK_22G_000\" TargetNamePrefix=\"B\" Description=\"Biopsy unknown 22G (JHH)\" Length=\"150\" TipLength=\"0\" Throw=\"0\" TargetLength=\"0\" TargetBase=\"0\" Diameter=\"0.711\" LastTargetIndex=\"0\" /> \
  <Needle ID=\"BIOP_EZEM_18G_000\" TargetNamePrefix=\"B\" Description=\"Biopsy E-Z-EM 18G (BWH)\" Length=\"150\" TipLength=\"1.5\" Throw=\"24\" TargetLength=\"16\" TargetBase=\"3\" Diameter=\"1.270\" LastTargetIndex=\"0\" /> \
  </NeedleList>";
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
vtkStandardNewMacro (vtkRobotProbeNavGUI );
vtkCxxRevisionMacro ( vtkRobotProbeNavGUI, "$Revision: 1.0 $");
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
vtkRobotProbeNavGUI::vtkRobotProbeNavGUI ( )
{
  
  //----------------------------------------------------------------
  // Logic values
  this->Logic = NULL;
  
  this->DataCallbackCommand = vtkCallbackCommand::New();
  this->DataCallbackCommand->SetClientData( reinterpret_cast<void *> (this) );
  this->DataCallbackCommand->SetCallback(vtkRobotProbeNavGUI::DataCallback);
  
  this->RobotProbeNavManagerNodeID =  NULL;
  this->RobotProbeNavManagerNode =  NULL;

  this->TargetPlanListNodeID = NULL;
  this->TargetPlanListNode = NULL;

  this->RobotNodeID = NULL;
  this->RobotNode = NULL;

  //----------------------------------------------------------------
  // Configuration Frame

  this->ShowSecondaryWindowCheckButton = NULL; 
  this->RobotProbeNavManagerSelectorWidget = NULL;
  this->RobotSelectorWidget = NULL;

  //----------------------------------------------------------------
  // Workphase Frame
  
  this->StatusButtonFrame = vtkKWFrame::New();
  this->StatusButtonSet = NULL;

  this->WorkphaseButtonFrame = vtkKWFrame::New();
  this->WorkphaseButtonSet = NULL;

  //----------------------------------------------------------------  
  // Wizard Frame
  
  this->WizardFrame = vtkSlicerModuleCollapsibleFrame::New();
  this->WizardWidget = NULL;

  this->DisplayedWorkflowSteps=vtkStringArray::New();

  this->SecondaryWindow=NULL;

  this->Entered = 0;
  
  this->numOfRows = 0;
  
  // 8/18/2011 ayamada
  //this->setupStep = vtkRobotProbeNavStepSetUp::New();
  
  // 8/18/2011 ayamada
  //----------------------------------------------------------------
  // Registration Frame
  // 8/4/2011 ayamada
  this->FiducialMarkersSetup            = NULL;
  this->AddFiducialMarkerPushButton     = NULL;
  this->ListFrame                       = NULL;
  this->PointPairMultiColumnList        = NULL;
  this->FiducialNumber = 1;
  this->FiducialNumber2 = 1;
  this->FixFiducialMarkersPushButton    = NULL;
  this->DeletePointPairPushButton       = NULL;
  this->DeleteAllPointPairPushButton    = NULL;
  this->FiducialFrame                   = NULL;  
  
  // 9/8/2011 ayamada
  this->ListFrame2                       = NULL;
  this->SelectFiducialsFileButton = NULL;
  this->PointPairMultiColumnList2        = NULL;
  this->DeletePointPairPushButton2       = NULL;
  this->DeleteAllPointPairPushButton2    = NULL;
  this->AddFiducialFilePushButton     = NULL;
  this->FiducialFrame2                   = NULL;  
  
  
  // 9/13/2011 ayamada
  this->testImageSender = 0;
  
  // 9/21/2011 ayamada
  this->ShowRobotCheckButton = NULL;
  
  this->polyActor = NULL;
  this->polyActorCamera = NULL;
  this->polyMapper = NULL;
  this->ModelDisplayNode = NULL;
  
  // 12/26/2011 ayamada
  //this->workPhaseStatus = 0;
  

  
}



//---------------------------------------------------------------------------
vtkRobotProbeNavGUI::~vtkRobotProbeNavGUI ( )
{
  this->RemoveMRMLObservers();
  this->RemoveGUIObservers();

  if (this->DataCallbackCommand)
    {
    this->DataCallbackCommand->Delete();
    this->DataCallbackCommand=NULL;
    }  

  if (this->SecondaryWindow!=NULL)
  {
    this->SecondaryWindow->SetApplication(NULL);
    this->SecondaryWindow->Delete();
    this->SecondaryWindow=NULL;
  }

  //----------------------------------------------------------------
  // Configuration Frame

   if (this->ShowSecondaryWindowCheckButton)
    {
    this->ShowSecondaryWindowCheckButton->SetParent(NULL);
    this->ShowSecondaryWindowCheckButton->Delete();
    this->ShowSecondaryWindowCheckButton = NULL;
    }

  if (this->RobotProbeNavManagerSelectorWidget)
    {
    this->RobotProbeNavManagerSelectorWidget->SetParent(NULL );
    this->RobotProbeNavManagerSelectorWidget->Delete ( );
    this->RobotProbeNavManagerSelectorWidget=NULL;
    }

  if (this->RobotSelectorWidget)
    {
    this->RobotSelectorWidget->SetParent(NULL );
    this->RobotSelectorWidget->Delete ( );
    this->RobotSelectorWidget=NULL;
    }
  
  // 8/18/2011 ayamada
  //----------------------------------------------------------------
  // Registration Frame
  // 8/4/2011 ayamada
  if (this->FiducialMarkersSetup)
  {
    this->FiducialMarkersSetup->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->FiducialMarkersSetup->SetParent(NULL);
    this->FiducialMarkersSetup->Delete();
    this->FiducialMarkersSetup = NULL;
  }
  // 8/4/2011 ayamada
  if (this->FiducialMarkersSetup)
  {
    this->FiducialMarkersSetup->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->FiducialMarkersSetup->SetParent(NULL);
    this->FiducialMarkersSetup->Delete();
    this->FiducialMarkersSetup = NULL;
  }
  // 8/4/2011 ayamada
  if (this->AddFiducialMarkerPushButton)
  {
    this->AddFiducialMarkerPushButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->AddFiducialMarkerPushButton->SetParent(NULL );
    this->AddFiducialMarkerPushButton->Delete ( );
    this->AddFiducialMarkerPushButton = NULL;
  }
  // 8/4/2011 ayamada
  if (this->ListFrame)
  {
    this->ListFrame->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->ListFrame->SetParent(NULL );
    this->ListFrame->Delete ( );
    this->ListFrame = NULL;
  }
  // 8/4/2011 ayamada
  if (this->PointPairMultiColumnList)
  {
    this->PointPairMultiColumnList->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->PointPairMultiColumnList->SetParent(NULL );
    this->PointPairMultiColumnList->Delete ( );
    this->PointPairMultiColumnList = NULL;
  }
  // 8/4/2011 ayamada
  if (this->FixFiducialMarkersPushButton)
  {
    this->FixFiducialMarkersPushButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->FixFiducialMarkersPushButton->SetParent(NULL );
    this->FixFiducialMarkersPushButton->Delete ( );
    this->FixFiducialMarkersPushButton = NULL;
  }
  // 8/4/2011 ayamada
  if (this->DeletePointPairPushButton)
  {
    this->DeletePointPairPushButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->DeletePointPairPushButton->SetParent(NULL );
    this->DeletePointPairPushButton->Delete ( );
    this->DeletePointPairPushButton = NULL;
  }
  // 8/4/2011 ayamada
  if (this->DeleteAllPointPairPushButton)
  {
    this->DeleteAllPointPairPushButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->DeleteAllPointPairPushButton->SetParent(NULL );
    this->DeleteAllPointPairPushButton->Delete ( );
    this->DeleteAllPointPairPushButton = NULL;
  }
  // 9/8/2011 ayamada
  if (this->AddFiducialFilePushButton)
  {
    this->AddFiducialFilePushButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->AddFiducialFilePushButton->SetParent(NULL );
    this->AddFiducialFilePushButton->Delete ( );
    this->AddFiducialFilePushButton = NULL;
  }
  // 9/8/2011 ayamada
  if (this->ListFrame2)
  {
    this->ListFrame2->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->ListFrame2->SetParent(NULL );
    this->ListFrame2->Delete ( );
    this->ListFrame2 = NULL;
  }
  // 9/8/2011 ayamada
  if (this->PointPairMultiColumnList2)
  {
    this->PointPairMultiColumnList2->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->PointPairMultiColumnList2->SetParent(NULL );
    this->PointPairMultiColumnList2->Delete ( );
    this->PointPairMultiColumnList2 = NULL;
  }
  // 9/8/2011 ayamada
  if (this->DeletePointPairPushButton2)
  {
    this->DeletePointPairPushButton2->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->DeletePointPairPushButton2->SetParent(NULL );
    this->DeletePointPairPushButton2->Delete ( );
    this->DeletePointPairPushButton2 = NULL;
  }
  // 9/8/2011 ayamada
  if (this->DeleteAllPointPairPushButton2)
  {
    this->DeleteAllPointPairPushButton2->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->DeleteAllPointPairPushButton2->SetParent(NULL );
    this->DeleteAllPointPairPushButton2->Delete ( );
    this->DeleteAllPointPairPushButton2 = NULL;
  }
  
  
  // 8/4/2011 ayamada
  if (this->FiducialFrame)
  {
    this->FiducialFrame->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->FiducialFrame->SetParent(NULL );
    this->FiducialFrame->Delete ( );
    this->FiducialFrame = NULL;
  }  
  
  // 9/8/2011 ayamada
  if (this->SelectFiducialsFileButton)
  {
    this->SelectFiducialsFileButton->SetParent(NULL);
    this->SelectFiducialsFileButton->Delete();
  }

  // 9/8/2011 ayamada
  if (this->FiducialFrame2)
  {
    this->FiducialFrame2->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->FiducialFrame2->SetParent(NULL );
    this->FiducialFrame2->Delete ( );
    this->FiducialFrame2 = NULL;
  }  
  
  // 9/21/2011 ayamada
  if (this->ShowRobotCheckButton)
  {
    this->ShowRobotCheckButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand );
    this->ShowRobotCheckButton->SetParent(NULL);
    this->ShowRobotCheckButton->Delete();
  }
  
  
  //----------------------------------------------------------------
  // Workphase Frame

  if (this->StatusButtonFrame)
    {
    this->StatusButtonFrame->SetParent(NULL);
    this->StatusButtonFrame->Delete(); 
    this->StatusButtonFrame = NULL;
    }

  if (this->StatusButtonSet)
    {
    this->StatusButtonSet->SetParent(NULL);
    this->StatusButtonSet->Delete();
    this->StatusButtonSet=NULL;
    }

  if (this->WorkphaseButtonFrame)
    {
    this->WorkphaseButtonFrame->SetParent(NULL);
    this->WorkphaseButtonFrame->Delete(); 
    this->WorkphaseButtonFrame = NULL;
    }

  if (this->WorkphaseButtonSet)
    {
    this->WorkphaseButtonSet->SetParent(NULL);
    this->WorkphaseButtonSet->Delete();
    this->WorkphaseButtonSet=NULL;
    }
  this->SetModuleLogic ( NULL );


  //----------------------------------------------------------------
  // Wizard Frame

  if (this->WizardFrame)
    {
    this->WizardFrame->SetParent(NULL);
    this->WizardFrame->Delete(); 
    this->WizardFrame = NULL;
    }

  if (this->WizardWidget)
    {
    this->WizardWidget->SetParent(NULL);
    this->WizardWidget->Delete(); 
    this->WizardWidget = NULL;
    }

  if (this->DisplayedWorkflowSteps)
    {
    this->DisplayedWorkflowSteps->Delete(); 
    this->DisplayedWorkflowSteps = NULL;
    }  

  this->SetAndObserveRobotNodeID( NULL );
  this->SetAndObserveRobotProbeNavManagerNodeID( NULL );
  this->SetAndObserveTargetPlanListNodeID( NULL );
    
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::PrintSelf ( ostream& os, vtkIndent indent )
{
    this->vtkObject::PrintSelf ( os, indent );
    
    os << indent << "RobotProbeNavGUI: " << this->GetClassName ( ) << "\n";
    os << indent << "RobotProbeNavManager: ";
    if (this->RobotProbeNavManagerNodeID)
    {
      os << this->RobotProbeNavManagerNodeID << "\n";
    }
    else
    {
       os << "NULL\n";
    }
    os << indent << "RobotNode: ";
    if (this->RobotNodeID)
    {
      os << this->RobotNodeID << "\n";
    }
    else
    {
       os << "NULL\n";
    }
    os << indent << "TargetPlanListNode: ";
    if (this->TargetPlanListNodeID)
    {
      os << this->TargetPlanListNodeID << "\n";
    }
    else
    {
       os << "NULL\n";
    }
    os << indent << "Logic: " << this->GetLogic ( ) << "\n";    
   
    // print widgets?
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::RemoveGUIObservers ( )
{
  
  // 8/18/2011 ayamada
  vtkSlicerApplicationGUI *appGUI = this->GetApplicationGUI();
  if (appGUI) 
    {
    appGUI->GetMainSliceGUI("Red")->GetSliceViewer()->GetRenderWidget()
      ->GetRenderWindowInteractor()->GetInteractorStyle()->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    appGUI->GetMainSliceGUI("Yellow")->GetSliceViewer()->GetRenderWidget()
      ->GetRenderWindowInteractor()->GetInteractorStyle()->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    appGUI->GetMainSliceGUI("Green")->GetSliceViewer()->GetRenderWidget()
      ->GetRenderWindowInteractor()->GetInteractorStyle()->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }
  
  //----------------------------------------------------------------
  // Configuration Frame

  if (this->RobotProbeNavManagerSelectorWidget)
    {
    this->RobotProbeNavManagerSelectorWidget->RemoveObservers ( vtkSlicerNodeSelectorWidget::NodeSelectedEvent,  (vtkCommand *)this->GUICallbackCommand );
    }

  if (this->RobotSelectorWidget)
    {
    this->RobotSelectorWidget->RemoveObservers ( vtkSlicerNodeSelectorWidget::NodeSelectedEvent,  (vtkCommand *)this->GUICallbackCommand );
    }

  
  //----------------------------------------------------------------
  // Workphase Frame

  if (this->WorkphaseButtonSet)
    {
    for (int i = 0; i < this->WorkphaseButtonSet->GetNumberOfWidgets(); i ++)
      {
      this->WorkphaseButtonSet->GetWidget(i)->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
      }
    }
    

  //----------------------------------------------------------------
  // Wizard Frame

  if (this->WizardWidget)
    {
    this->WizardWidget->GetWizardWorkflow()->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }

  this->RemoveLogicObservers();
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::RemoveLogicObservers ( )
{
  //vtkSlicerApplicationGUI *appGUI = this->GetApplicationGUI();
  if (this->GetLogic())
    {
    this->GetLogic()->RemoveObservers(vtkCommand::ModifiedEvent,
                                      (vtkCommand *)this->LogicCallbackCommand);
    }
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::AddGUIObservers ( )
{
  this->RemoveGUIObservers();
  

  // 8/16/2011 ayamada
  //----------------------------------------------------------------
  // Main Slice GUI

  // make a user interactor style to process our events
  // look at the InteractorStyle to get our events
  
  vtkSlicerApplicationGUI *appGUI = this->GetApplicationGUI();
  
  appGUI->GetMainSliceGUI("Red")->GetSliceViewer()->GetRenderWidget()
  ->GetRenderWindowInteractor()->GetInteractorStyle()
  ->AddObserver(vtkCommand::LeftButtonPressEvent, (vtkCommand *)this->GUICallbackCommand);
  appGUI->GetMainSliceGUI("Yellow")->GetSliceViewer()->GetRenderWidget()
  ->GetRenderWindowInteractor()->GetInteractorStyle()
  ->AddObserver(vtkCommand::LeftButtonPressEvent, (vtkCommand *)this->GUICallbackCommand);
  appGUI->GetMainSliceGUI("Green")->GetSliceViewer()->GetRenderWidget()
  ->GetRenderWindowInteractor()->GetInteractorStyle()
  ->AddObserver(vtkCommand::LeftButtonPressEvent, (vtkCommand *)this->GUICallbackCommand);
  

  //----------------------------------------------------------------
  // Configuration Frame
  
  this->RobotProbeNavManagerSelectorWidget->AddObserver ( vtkSlicerNodeSelectorWidget::NodeSelectedEvent, (vtkCommand *)this->GUICallbackCommand );
  this->RobotSelectorWidget->AddObserver ( vtkSlicerNodeSelectorWidget::NodeSelectedEvent, (vtkCommand *)this->GUICallbackCommand );  

  //----------------------------------------------------------------
  // Workphase Frame

  if (this->WorkphaseButtonSet!=NULL)
    {
    for (int i = 0; i < this->WorkphaseButtonSet->GetNumberOfWidgets(); i ++)
      {
      this->WorkphaseButtonSet->GetWidget(i)
        ->AddObserver(vtkKWPushButton::InvokedEvent, (vtkCommand *)this->GUICallbackCommand);
      }
    }
  
  
  //----------------------------------------------------------------
  // Wizard Frame

  if (this->WizardWidget)
    {
    this->WizardWidget->GetWizardWorkflow()->AddObserver(vtkKWWizardWorkflow::CurrentStateChangedEvent,
      (vtkCommand *)this->GUICallbackCommand);
    }


  //----------------------------------------------------------------
  // Etc Frame

  // observer load volume button

  this->AddLogicObservers();
  
  
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::AddLogicObservers ( )
{
  this->RemoveLogicObservers();  

  if (this->GetLogic())
    {
    this->GetLogic()->AddObserver(vtkRobotProbeNavLogic::StatusUpdateEvent,
                                  (vtkCommand *)this->LogicCallbackCommand);
    }
}

void vtkRobotProbeNavGUI::AddMRMLObservers(void)
{
  // observe the scene for node deleted events
  if (this->MRMLScene!=NULL)
  {
    if (this->MRMLScene->HasObserver(vtkMRMLScene::NodeRemovedEvent, (vtkCommand *)this->MRMLCallbackCommand) < 1)
    {
      this->MRMLScene->AddObserver(vtkMRMLScene::NodeRemovedEvent, (vtkCommand *)this->MRMLCallbackCommand);
    }
    if (this->MRMLScene->HasObserver(vtkMRMLScene::NodeAboutToBeRemovedEvent, (vtkCommand *)this->MRMLCallbackCommand) < 1)
    {
      this->MRMLScene->AddObserver(vtkMRMLScene::NodeAboutToBeRemovedEvent, (vtkCommand *)this->MRMLCallbackCommand);
    }
    if (this->MRMLScene->HasObserver(vtkMRMLScene::NodeAddedEvent, (vtkCommand *)this->MRMLCallbackCommand) < 1)
    {
      this->MRMLScene->AddObserver(vtkMRMLScene::NodeAddedEvent, (vtkCommand *)this->MRMLCallbackCommand);
    }
    if (this->MRMLScene->HasObserver(vtkMRMLScene::SceneCloseEvent, (vtkCommand *)this->MRMLCallbackCommand) < 1)
    {
      this->MRMLScene->AddObserver(vtkMRMLScene::SceneCloseEvent, (vtkCommand *)this->MRMLCallbackCommand);
    }
  }
}

void vtkRobotProbeNavGUI::RemoveMRMLObservers(void)
{
  if (this->MRMLScene!=NULL)
  {
    this->MRMLScene->RemoveObservers(vtkMRMLScene::NodeRemovedEvent, (vtkCommand *)this->MRMLCallbackCommand);
    this->MRMLScene->RemoveObservers(vtkMRMLScene::NodeAboutToBeRemovedEvent, (vtkCommand *)this->MRMLCallbackCommand);
    this->MRMLScene->RemoveObservers(vtkMRMLScene::NodeAddedEvent, (vtkCommand *)this->MRMLCallbackCommand);
    this->MRMLScene->RemoveObservers(vtkMRMLScene::SceneCloseEvent, (vtkCommand *)this->MRMLCallbackCommand);
  }
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::HandleMouseEvent(vtkSlicerInteractorStyle *style)//, vtkRobotProbeNavStepSetUp *setupStep)
{

   vtkSlicerApplicationGUI *appGUI = this->GetApplicationGUI();

   vtkSlicerInteractorStyle *istyle0 = 
   vtkSlicerInteractorStyle::SafeDownCast(appGUI->GetMainSliceGUI("Red")->GetSliceViewer()->GetRenderWidget()
   ->GetRenderWindowInteractor()->GetInteractorStyle());
   
   vtkSlicerInteractorStyle *istyle1 = 
   vtkSlicerInteractorStyle::SafeDownCast(appGUI->GetMainSliceGUI("Yellow")->GetSliceViewer()->GetRenderWidget()
   ->GetRenderWindowInteractor()->GetInteractorStyle());
   
   vtkSlicerInteractorStyle *istyle2 = 
   vtkSlicerInteractorStyle::SafeDownCast(appGUI->GetMainSliceGUI("Green")->GetSliceViewer()->GetRenderWidget()
   ->GetRenderWindowInteractor()->GetInteractorStyle());
       
   vtkCornerAnnotation *anno = NULL;
   if (style == istyle0)
   {
   anno = appGUI->GetMainSliceGUI("Red")->GetSliceViewer()->GetRenderWidget()->GetCornerAnnotation();
   }
   else if (style == istyle1)
   {
   anno = appGUI->GetMainSliceGUI("Yellow")->GetSliceViewer()->GetRenderWidget()->GetCornerAnnotation();
   }
   else if (style == istyle2)
   {
   anno = appGUI->GetMainSliceGUI("Green")->GetSliceViewer()->GetRenderWidget()->GetCornerAnnotation();
   }
   
   if (anno)
   {
   const char *rasText = anno->GetText(1);
   if ( rasText != NULL )
   {
   std::string ras = std::string(rasText);
   
   // remove "R:," "A:," and "S:" from the string
   size_t loc = ras.find("R:", 0);
   if ( loc != std::string::npos ) 
   {
   ras = ras.replace(loc, 2, "");
   }
   loc = ras.find("A:", 0);
   if ( loc != std::string::npos ) 
   {
   ras = ras.replace(loc, 2, "");
   }
   loc = ras.find("S:", 0);
   if ( loc != std::string::npos ) 
   {
   ras = ras.replace(loc, 2, "");
   }
   
   // remove "\n" from the string
   size_t found = ras.find("\n", 0);
   while ( found != std::string::npos )
   {
   ras = ras.replace(found, 1, " ");
   found = ras.find("\n", 0);
   }
   
  // 9/8/2011 ayamada   
  this->numOfRows = this->PointPairMultiColumnList2->GetWidget()->GetNumberOfSelectedRows();
     
  if (numOfRows == 1)
  {
    int index[3];
    char str[32];
    const char *pc = NULL;
    sprintf(str, "12345");
    pc = str;    
    
    // 9/8/2011 ayamada
    this->PointPairMultiColumnList2->GetWidget()->GetSelectedRows(index);
    this->PointPairMultiColumnList2->GetWidget()->SetCellText(index[0], 1, ras.c_str());
  }
  
      
   }
   }
  
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::ProcessGUIEvents(vtkObject *caller,
                                         unsigned long event, void *callData)
{
  vtkMRMLRobotProbeNavManagerNode *manager=this->GetRobotProbeNavManagerNode();


  const char *eventName = vtkCommand::GetStringFromEventId(event);

  // 8/21/2011 ayamada
  if (strcmp(eventName, "LeftButtonPressEvent") == 0)
    {
      std::cerr << "leftButton was pressed in gui event." << std::endl;

    vtkSlicerInteractorStyle *style = vtkSlicerInteractorStyle::SafeDownCast(caller);

      this->HandleMouseEvent(style);//, setupStep);
            
    return;
    }

  
  //----------------------------------------------------------------
  // Configuration Frame

  else if (this->RobotProbeNavManagerSelectorWidget == vtkSlicerNodeSelectorWidget::SafeDownCast(caller) &&
           (event == vtkSlicerNodeSelectorWidget::NodeSelectedEvent || event == vtkSlicerNodeSelectorWidget::NewNodeEvent )) 
    {
    vtkMRMLRobotProbeNavManagerNode *managerNode = vtkMRMLRobotProbeNavManagerNode::SafeDownCast(this->RobotProbeNavManagerSelectorWidget->GetSelected());
    char *managerID=NULL;
    if (managerNode!=NULL)
      {
      managerID=managerNode->GetID();
      }
    this->SetAndObserveRobotProbeNavManagerNodeID(managerID);    
    return;
    }

  else if (this->RobotSelectorWidget == vtkSlicerNodeSelectorWidget::SafeDownCast(caller) &&
           (event == vtkSlicerNodeSelectorWidget::NodeSelectedEvent || event == vtkSlicerNodeSelectorWidget::NewNodeEvent )) 
    {    
    char *robotID=NULL;
    vtkMRMLRobotNode *refNode = vtkMRMLRobotNode::SafeDownCast(this->RobotSelectorWidget->GetSelected());
    if (refNode!=NULL)
      {
      robotID=refNode->GetID();
      }
    if (manager!=NULL)
      {
      manager->SetAndObserveRobotNodeID(robotID);
      }        
    SetAndObserveRobotNodeID(robotID);

    return;
    }

  //----------------------------------------------------------------
  // Check Work Phase Transition Buttons

  vtkKWPushButton* pushButtonCaller=vtkKWPushButton::SafeDownCast(caller);
  vtkKWPushButtonSet* pushButtonCallerParent=NULL;
  if (pushButtonCaller!=NULL)
  {
    pushButtonCallerParent=vtkKWPushButtonSet::SafeDownCast(pushButtonCaller->GetParent());
  }

  if ( this->WorkphaseButtonSet!=NULL && pushButtonCallerParent==this->WorkphaseButtonSet &&
    event == vtkKWPushButton::InvokedEvent && this->WorkphaseButtonSet!=NULL)
  {
    int phase;
    for (phase = 0; phase < this->WorkphaseButtonSet->GetNumberOfWidgets(); phase ++)
    {
      if (this->WorkphaseButtonSet->GetWidget(phase) == vtkKWPushButton::SafeDownCast(caller))
      {
        break;
      }
    }
    if (manager!=NULL)
    {
      if (phase < manager->GetNumberOfSteps()) // if pressed one of them
      {
        ChangeWorkphaseInGUI(phase);
      }
    }
  }

  //----------------------------------------------------------------
  // Wizard Frame

  else if (this->WizardWidget!=NULL && this->WizardWidget->GetWizardWorkflow() == vtkKWWizardWorkflow::SafeDownCast(caller) &&
      event == vtkKWWizardWorkflow::CurrentStateChangedEvent)
  {

    int phase = 0;
    vtkKWWizardStep* currentStep =  this->WizardWidget->GetWizardWorkflow()->GetCurrentStep();

    if (manager)
    {
      int numSteps = manager->GetNumberOfSteps();
      for (int i = 0; i < numSteps; i ++)
      {
        if (currentStep == GetStepPage(i))
        {
          phase = i;
        }
      }

      manager->SwitchStep(phase); // Notify manager about state change

      // Update workflow button states and current step GUI
      UpdateGUI();

    }
  }


  //----------------------------------------------------------------
  // Etc Frame

  // Process Wizard GUI (Active step only)
  else
    {
    if (manager)
      {
      int stepId = manager->GetCurrentStep();
      vtkRobotProbeNavStep *step=GetStepPage(stepId);
      if (step!=NULL)
        {
        step->ProcessGUIEvents(caller, event, callData);
        }
      }
    }
  

  // 9/8/2011 ayamada: read CSV file and list the data
  if (this->AddFiducialFilePushButton == vtkKWPushButton::SafeDownCast(caller) 
      && event == vtkKWPushButton::InvokedEvent)
  {

    const char * path = this->SelectFiducialsFileButton->GetWidget()->GetFileName();
    if (path)
    {
      std::cerr << "The file path was set." << std::endl;
      
      int numberOfData = this->LoadCSVTrackingFile(path);      
      
      if (!numberOfData/*this->LoadCSVTrackingFile(path)*/)
      {
        std::cerr << "The CVS file was not able to be read." << std::endl;
      }else{
        
        int scSize = 0;
        //int pcSize = 0;
        char str[32];
        char str2[128];
        const char *pc = NULL;
        const char *sc = NULL;//this->FiducialMarkersSetup->GetWidget()->GetValue();
        
        for(int i = 0; i<numberOfData; i++)
        {
          sprintf(str, "%d", this->FiducialNumber);
          sprintf(str2, "%.2f %.2f %.2f", this->pX[i],this->pY[i],this->pZ[i]);
          pc = str;
          sc = str2;
                
          int row = this->PointPairMultiColumnList->GetWidget()->GetNumberOfRows();
          this->PointPairMultiColumnList->GetWidget()->AddRow();
          this->PointPairMultiColumnList->GetWidget()->SetCellText(row, 0, pc);
          // 9/8/2011 ayamada
          this->PointPairMultiColumnList->GetWidget()->SetCellText(row, 1, sc);
          
          // increment of firucial marker numbers
          this->FiducialNumber++;
          this->FiducialNumber2++;
          
          // 9/8/2011 ayamada
          int row2 = this->PointPairMultiColumnList2->GetWidget()->GetNumberOfRows();
          this->PointPairMultiColumnList2->GetWidget()->AddRow();
          this->PointPairMultiColumnList2->GetWidget()->SetCellText(row, 0, pc);
          
          
        }
        
      }
      
    }
    
    
  }
    
    
  // 8/18/2011 ayamada
  //----------------------------------------------------------------
  // Registration Frame  
  // 8/4/2011 ayamada
  if (this->AddFiducialMarkerPushButton == vtkKWPushButton::SafeDownCast(caller) 
      && event == vtkKWPushButton::InvokedEvent)
  {
    
    
    
    //this->NumberOfSelectedRowFunc();
    
    int scSize = 0;
    //int pcSize = 0;
    char str[32];
    const char *pc = NULL;
    const char *sc = this->FiducialMarkersSetup->GetWidget()->GetValue();
    
    sprintf(str, "%d", this->FiducialNumber);
    pc = str;
    
    // 8/16/2011 ayamada
    std::cerr << "OK button was pressed." << std::endl;
    
    if (sc) 
    {
      const vtksys_stl::string scCor(sc);
      scSize = scCor.size();
    }
    
    
    if (/*pcSize < 5 ||*/ scSize < 5)
    {
      vtkSlicerApplication::GetInstance()->ErrorMessage("Robot coordinates are invalid."); 
    }
    else 
    {
      
      int row = this->PointPairMultiColumnList->GetWidget()->GetNumberOfRows();
      this->PointPairMultiColumnList->GetWidget()->AddRow();
      this->PointPairMultiColumnList->GetWidget()->SetCellText(row, 0, pc);
      // 9/8/2011 ayamada
      this->PointPairMultiColumnList->GetWidget()->SetCellText(row, 1, sc);
      
      // increment of firucial marker numbers
      this->FiducialNumber++;
      this->FiducialNumber2++;
      
      // 9/8/2011 ayamada
      int row2 = this->PointPairMultiColumnList2->GetWidget()->GetNumberOfRows();
      this->PointPairMultiColumnList2->GetWidget()->AddRow();
      this->PointPairMultiColumnList2->GetWidget()->SetCellText(row, 0, pc);
      
    }
    
  }
  
  // 8/4/2011 ayamada
  if (this->DeletePointPairPushButton == vtkKWPushButton::SafeDownCast(caller) 
      && event == vtkKWPushButton::InvokedEvent)
  {
    int numOfRows = this->PointPairMultiColumnList->GetWidget()->GetNumberOfSelectedRows();
    if (numOfRows == 1)
    {
      
      int index[3];
      this->PointPairMultiColumnList->GetWidget()->GetSelectedRows(index);
      this->PointPairMultiColumnList->GetWidget()->DeleteRow(index[0]);

      // 9/8/2011 ayamada for operating image coordinate list
      int row2 = this->PointPairMultiColumnList->GetWidget()->GetNumberOfRows();
      this->PointPairMultiColumnList2->GetWidget()->DeleteRow(row2);      
      
    }
    
    // 8/4/2011 ayamada
    this->FiducialNumber--;
    int row = this->PointPairMultiColumnList->GetWidget()->GetNumberOfRows();
    
    char str[32];
    const char *pc = NULL;    
    
    // 8/4/2011 ayamada: rearrange turns
    for (int r = 0; r < row; r++)
    {
      sprintf(str, "%d", r+1);
      pc = str;
      this->PointPairMultiColumnList->GetWidget()->SetCellText(r, 0, pc);      
    }
    
  }


  else if (this->DeleteAllPointPairPushButton == vtkKWPushButton::SafeDownCast(caller) 
           && event == vtkKWPushButton::InvokedEvent)
  {
    this->PointPairMultiColumnList->GetWidget()->DeleteAllRows();

    // 9/8/2011 ayamada
    this->PointPairMultiColumnList2->GetWidget()->DeleteAllRows();
    
    // reset number
    this->FiducialNumber = 1;
    
  }

  
  // 8/4/2011 ayamada
  // registor all points listed
  else if (this->FixFiducialMarkersPushButton == vtkKWPushButton::SafeDownCast(caller) 
           && event == vtkKWPushButton::InvokedEvent)
  {
    int row = this->PointPairMultiColumnList->GetWidget()->GetNumberOfRows();
    if (row < 2)
    {
      vtkSlicerApplication::GetInstance()->ErrorMessage("At least 2 pairs of landmarks are needed for patient to image registration.");
    }
    else
    {
      
      float sc1 = 0.0, sc2 = 0.0, sc3 = 0.0;
      for (int r = 0; r < row; r++)
      {
        const char *val = this->PointPairMultiColumnList->GetWidget()->GetCellText(r, 1);
        sscanf(val, "%f %f %f", &sc1, &sc2, &sc3);
        this->GetLogic()->GetFiducialMarkersR(r, sc1, sc2, sc3);

        const char *val2 = this->PointPairMultiColumnList2->GetWidget()->GetCellText(r, 1);
        sscanf(val2, "%f %f %f", &sc1, &sc2, &sc3);
        this->GetLogic()->GetFiducialMarkersI(r, sc1, sc2, sc3);

        
        
      }

      // 11/4/2011 ayamada
      this->GetLogic()->CalculateVirtualCenterPosition();
      std::cerr << "Send Robot Position Data!!" << std::endl; 
      
      
      
      
    }
  }

  
  

} 

//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::Init()
{
  // -----------------------------------------
  // Register all new MRML node classes
  {
    // Make sure that all MRML classes are registered (needed for creating/updating the node from XML)
    // SmartPointer is used to create an instance of the class, and destroy immediately after registration is complete
    this->GetMRMLScene()->RegisterNodeClass( vtkSmartPointer< vtkMRMLBrpRobotCommandNode >::New() );
    this->GetMRMLScene()->RegisterNodeClass( vtkSmartPointer< vtkMRMLRobotProbeNavManagerNode >::New() );
    this->GetMRMLScene()->RegisterNodeClass( vtkSmartPointer< vtkMRMLRobotDisplayNode >::New() );
    this->GetMRMLScene()->RegisterNodeClass( vtkSmartPointer< vtkMRMLTransRectalRobotProbeRobotNode >::New() );
    this->GetMRMLScene()->RegisterNodeClass( vtkSmartPointer< vtkMRMLIGTProbeRobotNode >::New() );    
    this->GetMRMLScene()->RegisterNodeClass( vtkSmartPointer< vtkMRMLIGTProbeTemplateNode >::New() );    
  }

}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::DataCallback(vtkObject *caller, 
        unsigned long eid, void *clientData, void *callData)
{
  vtkRobotProbeNavGUI *self = reinterpret_cast<vtkRobotProbeNavGUI *>(clientData);
  vtkDebugWithObjectMacro(self, "In vtkRobotProbeNavGUI DataCallback");
  
  self->UpdateGUI();
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::ProcessLogicEvents ( vtkObject *caller,
    unsigned long event, void *callData )
{

  if (this->GetLogic() == vtkRobotProbeNavLogic::SafeDownCast(caller))
    {
    if (event == vtkRobotProbeNavLogic::StatusUpdateEvent)
      {
      //this->UpdateDeviceStatus();
      }
    }
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::ProcessMRMLEvents ( vtkObject *caller,
    unsigned long event, void *callData )
{

  // manager node chaged
  vtkMRMLRobotProbeNavManagerNode *manager=this->GetRobotProbeNavManagerNode();
  if (manager!=NULL && manager == vtkMRMLRobotProbeNavManagerNode::SafeDownCast(caller))
    {
    switch (event)
      {
      case vtkCommand::ModifiedEvent:
        UpdateGUI();
        break;
      case vtkMRMLRobotProbeNavManagerNode::CurrentTargetChangedEvent:
        // BringTargetToViewIn2DViews(); this is now called in wizard steps
        break;
      }
    }
  // :TODO: update GUI (and observers) if robotnode or targetplanlistnode within manager node is changed

  // robot status changed
  vtkMRMLRobotNode *robotNode=GetRobotNode();
  if (robotNode!=NULL && robotNode == vtkMRMLRobotNode::SafeDownCast(caller))
    {
    if (event == vtkMRMLRobotNode::ChangeStatusEvent)
      {
      UpdateStatusButtons();
      }
    }
  
  
  if (event == vtkMRMLLinearTransformNode::TransformModifiedEvent)
  {
    std::cerr << "Test2!!" << std::endl; 
    
    vtkMRMLLinearTransformNode* node = vtkMRMLLinearTransformNode::SafeDownCast(caller);
    vtkMatrix4x4* transformToParent = node->GetMatrixTransformToParent();

  }  
  

  // current target changed
  vtkMRMLFiducialListNode* targetPlanList=NULL;
  targetPlanList=this->GetTargetPlanListNode();
  if (targetPlanList!=NULL && targetPlanList == vtkMRMLFiducialListNode::SafeDownCast(caller))
    {
    UpdateCurrentTargetDisplay();
    }

  if ( this->MRMLScene!=NULL && vtkMRMLScene::SafeDownCast(caller) == this->MRMLScene && (event == vtkMRMLScene::NodeAboutToBeRemovedEvent) )
    {
      if (GetRobotNode() != NULL && callData == GetRobotNode())
        {
        // robot node will be deleted => remove referenced nodes from the scene
        GetRobotNode()->RemoveChildNodes();
        }
    }

  // a node has been deleted
  if ( this->MRMLScene!=NULL && vtkMRMLScene::SafeDownCast(caller) == this->MRMLScene && (event == vtkMRMLScene::NodeRemovedEvent ) )
    {
    if (this->RobotProbeNavManagerNodeID != NULL && this->MRMLScene->GetNodeByID(this->RobotProbeNavManagerNodeID) == NULL)
      {
      // manager node has been deleted
      this->SetAndObserveRobotProbeNavManagerNodeID(NULL);
      }
    if (this->RobotNodeID != NULL && this->MRMLScene->GetNodeByID(this->RobotNodeID) == NULL)
      {
      // robot node has been deleted
      if (manager!=NULL)
        {
        manager->SetAndObserveRobotNodeID(NULL);
        }
      this->SetAndObserveRobotNodeID(NULL);
      }
    if (this->TargetPlanListNodeID != NULL && this->MRMLScene->GetNodeByID(this->TargetPlanListNodeID) == NULL)
      {
      // target plan list node has been deleted
      this->SetAndObserveTargetPlanListNodeID(NULL);
      }
    }
  
  // scene is closing
  if (event == vtkMRMLScene::SceneCloseEvent )
    {
    this->SetAndObserveRobotProbeNavManagerNodeID(NULL);
    this->SetAndObserveRobotNodeID(NULL);
    this->SetAndObserveTargetPlanListNodeID(NULL);
    }
}

//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::Enter()
{
  if (this->Entered == 0)
    {
    this->GetLogic()->SetGUI(this);
    this->GetLogic()->Enter();    
    this->Entered = 1;
    }

  // The user interface is hidden on Exit, show it now
  if (this->WizardWidget!=NULL)
    {
    vtkKWWizardWorkflow *wizard_workflow = this->WizardWidget->GetWizardWorkflow();
    if (wizard_workflow!=NULL)
      {
      vtkRobotProbeNavStep* step=vtkRobotProbeNavStep::SafeDownCast(wizard_workflow->GetCurrentStep());
      if (step)
        {
        step->ShowUserInterface();
        }
      }
    }


  AddMRMLObservers();

  // Anything could have been done while using an other module, so update the GUI now
  UpdateGUI();
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::Enter(vtkMRMLNode *node)
{
  Enter();

  vtkMRMLRobotProbeNavManagerNode *managerNode = vtkMRMLRobotProbeNavManagerNode::SafeDownCast(node);
  if ( managerNode )
    {
    this->RobotProbeNavManagerSelectorWidget->UpdateMenu();
    this->RobotProbeNavManagerSelectorWidget->SetSelected( managerNode ); // :TODO: check if observers are updated after this
    }

  this->UpdateGUI();
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::Exit ( )
{
  // Show the user interface, because there could be observers that must be deactivated
  RemoveMRMLObservers();

  if (this->WizardWidget!=NULL)
    {
    vtkKWWizardWorkflow *wizard_workflow = this->WizardWidget->GetWizardWorkflow();
    if (wizard_workflow!=NULL)
      {
      vtkRobotProbeNavStep* step=vtkRobotProbeNavStep::SafeDownCast(wizard_workflow->GetCurrentStep());
      if (step)
        {
        step->HideUserInterface();
        }
      }
    }
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::BuildGUI ( )
{
    this->UIPanel->AddPage ( "RobotProbeNav", "RobotProbeNav", NULL );
    BuildGUIForHelpFrame();
    BuildGUIForConfigurationFrame();
    BuildGUIForWorkphaseFrame();
  
    // 8/18/2011 ayamada
    BuildGUIForRegistrationFrame();
  
    // 9/21/2011 ayamada
    //BuildGUIForTargetingFrame();
  
  
  
    BuildGUIForWizardFrame();
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::TearDownGUI ( )
{
  // REMOVE OBSERVERS and references to MRML and Logic
  // disconnect circular references so destructor can be called

  this->RemoveMRMLObservers();
  this->RemoveGUIObservers();

  if (this->SecondaryWindow)
  {  
    this->SecondaryWindow->Destroy();
  }

  this->GetLogic()->SetGUI(NULL);

  if (this->WizardWidget!=NULL)
  {
    for (int i=0; i<this->WizardWidget->GetWizardWorkflow()->GetNumberOfSteps(); i++)
    {
      vtkRobotProbeNavStep *step=vtkRobotProbeNavStep::SafeDownCast(this->WizardWidget->GetWizardWorkflow()->GetNthStep(i));
      if (step!=NULL)
        {
        step->TearDownGUI();
        step->SetGUI(NULL);
        step->SetLogic(NULL);
        step->SetAndObserveMRMLScene(NULL);
        step->SetRobotProbeNavManager(NULL);
        }
      else
        {
        vtkErrorMacro("Invalid step page: "<<i);
        }
    }
  }
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::BuildGUIForWizardFrame()
{
    vtkKWWidget *page = this->UIPanel->GetPageWidget ( "RobotProbeNav" );
    vtkSlicerApplication *app = (vtkSlicerApplication *)this->GetApplication();

    if (!this->WizardFrame->IsCreated())
    {
      this->WizardFrame->SetParent(page);
      this->WizardFrame->Create();
      this->WizardFrame->SetLabelText("Wizard");
      this->WizardFrame->ExpandFrame();

      app->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2 -in %s",
                  this->WizardFrame->GetWidgetName(), 
                  page->GetWidgetName());
    }
}


void vtkRobotProbeNavGUI::BuildGUIForHelpFrame ()
{

  // ----------------------------------------------------------------
  // HELP FRAME         
  // ----------------------------------------------------------------

  // Define your help text here.
  std::stringstream helpss;
  helpss << "Module Version: " << vtkMRMLRobotProbeNavManagerNode::GetRobotProbeNavModuleVersion() << std::endl;
  helpss << "Source Revision: ";
  if (vtkMRMLRobotProbeNavManagerNode::GetRobotProbeNavWorkingCopyRevision() >= 0)
    {
    helpss << vtkMRMLRobotProbeNavManagerNode::GetRobotProbeNavWorkingCopyRevision() << std::endl;
    }
  else
    {
    helpss << "Unknown" << std::endl;
    }
  helpss << "The **RobotProbeNav Module** helps you to plan and navigate MRI-guided RobotProbe biopsy ";
  helpss << "and brachytherapy using transrectal and transperineal needle placement devices. \n";
  helpss << "See <a>http://www.slicer.org/slicerWiki/index.php/Modules:RobotProbeNav-Documentation-3.6</a> for details.";
  
  std::stringstream aboutss;
  aboutss << "This module was developed by Atsushi Yamada (Brigham and Women's Hospital). PI: Nobuhiko Hata. ";
  aboutss << "This module was developed based on ProstateNav module, AbdoNav module, and NeuroNav module. ";
  aboutss << "This work was supported by NCIGT, NA-MIC and BRP \"Enabling Technologies for MRI-Guided RobotProbe Intervention\" funded by NIH.";

  vtkKWWidget *page = this->UIPanel->GetPageWidget ( "RobotProbeNav" );
  this->BuildHelpAndAboutFrame (page, helpss.str().c_str(), aboutss.str().c_str());

  vtkSmartPointer<vtkKWLabel> NAMICLabel = vtkSmartPointer<vtkKWLabel>::New();
  NAMICLabel->SetParent ( this->GetLogoFrame() );
  NAMICLabel->Create();
  NAMICLabel->SetImageToIcon ( this->GetAcknowledgementIcons()->GetNAMICLogo() );    

  vtkSmartPointer<vtkKWLabel> NCIGTLabel = vtkSmartPointer<vtkKWLabel>::New();
  NCIGTLabel->SetParent ( this->GetLogoFrame() );
  NCIGTLabel->Create();
  NCIGTLabel->SetImageToIcon ( this->GetAcknowledgementIcons()->GetNCIGTLogo() );
    
  vtkSlicerApplication *app = vtkSlicerApplication::SafeDownCast (this->GetApplication() );
  if ( !app )
    {
    vtkErrorMacro ( "BuildGUIForHelpFrame: got Null SlicerApplication" );
    return;
    }

  app->Script ( "grid %s -row 0 -column 0 -padx 2 -pady 2 -sticky w", NAMICLabel->GetWidgetName());
  app->Script ("grid %s -row 0 -column 1 -padx 2 -pady 2 -sticky w", NCIGTLabel->GetWidgetName());    

}


//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::BuildGUIForConfigurationFrame ()
{
  vtkSlicerApplication *app = (vtkSlicerApplication *)this->GetApplication();
  vtkKWWidget *page = this->UIPanel->GetPageWidget ( "RobotProbeNav" );
  
  vtkSlicerModuleCollapsibleFrame *configurationFrame = vtkSlicerModuleCollapsibleFrame::New ( );
  configurationFrame->SetParent(page);
  configurationFrame->Create();
  configurationFrame->SetLabelText("Configuration");
  configurationFrame->ExpandFrame();
  app->Script("pack %s -side top -anchor center -fill x -padx 2 -pady 2 -in %s",
              configurationFrame->GetWidgetName(), page->GetWidgetName());


  // add an option to show the secondary window
  this->ShowSecondaryWindowCheckButton = vtkKWCheckButton::New();
  this->ShowSecondaryWindowCheckButton->SetParent (configurationFrame->GetFrame());
  this->ShowSecondaryWindowCheckButton->Create ( );
  this->ShowSecondaryWindowCheckButton->SetText("Show the Secondary Window");
  this->ShowSecondaryWindowCheckButton->SelectedStateOff();
  this->ShowSecondaryWindowCheckButton->SetCommand(this, "ShowSecondaryWindowCheckButtonCallback");
  this->ShowSecondaryWindowCheckButton->SetBalloonHelpString("Show the secondary window.");
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2",
               this->ShowSecondaryWindowCheckButton->GetWidgetName());
  
  //  Manager node selector widget
  this->RobotProbeNavManagerSelectorWidget = vtkSlicerNodeSelectorWidget::New() ;
  this->RobotProbeNavManagerSelectorWidget->SetParent(configurationFrame->GetFrame());
  this->RobotProbeNavManagerSelectorWidget->Create();
  // 3/8/2012 ayamada
  //this->RobotProbeNavManagerSelectorWidget->SetNodeClass("vtkMRMLRobotProbeNavManagerNode", NULL, NULL, "RobotProbeNav exam");
  this->RobotProbeNavManagerSelectorWidget->SetNodeClass("vtkMRMLRobotProbeNavManagerNode", NULL, NULL, "IGTNavigator exam");
  this->RobotProbeNavManagerSelectorWidget->SetMRMLScene(this->GetMRMLScene());
  this->RobotProbeNavManagerSelectorWidget->SetBorderWidth(2);
  this->RobotProbeNavManagerSelectorWidget->GetWidget()->GetWidget()->IndicatorVisibilityOff();
  this->RobotProbeNavManagerSelectorWidget->GetWidget()->GetWidget()->SetWidth(24);
  this->RobotProbeNavManagerSelectorWidget->SetLabelText( "Exam: ");
  this->RobotProbeNavManagerSelectorWidget->NewNodeEnabledOn();
  this->RobotProbeNavManagerSelectorWidget->SetBalloonHelpString("Select the active IGTNavigator configuration from the current scene.");
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2",
               this->RobotProbeNavManagerSelectorWidget->GetWidgetName());

  this->RobotSelectorWidget = vtkSlicerNodeSelectorWidget::New() ;
  this->RobotSelectorWidget->SetParent(configurationFrame->GetFrame());
  this->RobotSelectorWidget->Create(); 
  // 3/8/2012 ayamada
  this->RobotSelectorWidget->AddNodeClass("vtkMRMLIGTProbeRobotNode", NULL, NULL, "IGT robot");
  this->RobotSelectorWidget->SetMRMLScene(this->GetMRMLScene());
  this->RobotSelectorWidget->SetBorderWidth(2);
  this->RobotSelectorWidget->GetWidget()->GetWidget()->IndicatorVisibilityOff();
  this->RobotSelectorWidget->GetWidget()->GetWidget()->SetWidth(24);
  this->RobotSelectorWidget->SetLabelText( "Robot: ");
  this->RobotSelectorWidget->NewNodeEnabledOn();
  this->RobotSelectorWidget->SetBalloonHelpString("Select the robot.");
  this->RobotSelectorWidget->SetEnabled(0);
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2",
               this->RobotSelectorWidget->GetWidgetName());  

  configurationFrame->Delete();
}
    
//---------------------------------------------------------------------------
void vtkRobotProbeNavGUI::BuildGUIForWorkphaseFrame ()
{
  // the buttonset shall be dynamically created/destroyed when the
  // wizard GUI is built, because there is no API to remove any buttons
  // so just create the frame now

  vtkSlicerApplication *app = (vtkSlicerApplication *)this->GetApplication();
  vtkKWWidget *page = this->UIPanel->GetPageWidget ( "RobotProbeNav" );
  
  if (!this->WorkphaseButtonFrame->IsCreated())
  {
    vtkSmartPointer<vtkSlicerModuleCollapsibleFrame> workphaseFrame=vtkSmartPointer<vtkSlicerModuleCollapsibleFrame>::New();
    workphaseFrame->SetParent(page);
    workphaseFrame->Create();
    workphaseFrame->SetLabelText("Workphase");
    workphaseFrame->ExpandFrame();
    app->Script("pack %s -side top -anchor center -fill x -padx 2 -pady 2 -in %s",
                workphaseFrame->GetWidgetName(), page->GetWidgetName());

    // -----------------------------------------
    // Frames

    this->StatusButtonFrame->SetParent ( workphaseFrame->GetFrame() );
    this->StatusButtonFrame->Create ( );
    
    this->WorkphaseButtonFrame->SetParent( workphaseFrame->GetFrame());
    this->WorkphaseButtonFrame->Create();
    
    app->Script ( "pack %s %s -side top -fill x -expand y -padx 1 -pady 1",
                  this->StatusButtonFrame->GetWidgetName(),
                  this->WorkphaseButtonFrame->GetWidgetName());
  }  
    
}

// 9/21/2011 ayamada
//----------------------------------------------------------------
// Targeting Frame  
void vtkRobotProbeNavGUI::BuildGUIForTargetingFrame()
{

  vtkSlicerApplication *app = (vtkSlicerApplication *)this->GetApplication();
  vtkKWWidget *page = this->UIPanel->GetPageWidget ( "RobotProbeNav" );  
  
  vtkSlicerModuleCollapsibleFrame *targetingFrame = vtkSlicerModuleCollapsibleFrame::New ( );
  targetingFrame->SetParent(page);
  targetingFrame->Create();
  targetingFrame->SetLabelText("Targetting");
  targetingFrame->ExpandFrame();
  app->Script("pack %s -side top -anchor center -fill x -padx 2 -pady 2 -in %s",
              targetingFrame->GetWidgetName(), page->GetWidgetName());

  // from vtkSlicerModelsGUI.cxx
  // add a file browser 
  
  this->ShowRobotCheckButton = vtkKWCheckButton::New();
  this->ShowRobotCheckButton->SetParent(targetingFrame->GetFrame());
  this->ShowRobotCheckButton->Create();
  this->ShowRobotCheckButton->SelectedStateOff();
  this->ShowRobotCheckButton->SetText("Show Robot");   

  this->ShowRobotCheckButton->AddObserver ( vtkKWPushButton::InvokedEvent, 
                                                (vtkCommand *)this->GUICallbackCommand );
  
  
  this->Script("pack %s -side top -anchor nw -padx 2 -pady 4 -ipadx 0 -ipady 0", 
              this->ShowRobotCheckButton->GetWidgetName());
  
  

}  
  
// 8/18/2011 ayamada
//----------------------------------------------------------------
// Registration Frame  
void vtkRobotProbeNavGUI::BuildGUIForRegistrationFrame()
{

  vtkSlicerApplication *app = (vtkSlicerApplication *)this->GetApplication();
  vtkKWWidget *page = this->UIPanel->GetPageWidget ( "RobotProbeNav" );  

  vtkSlicerModuleCollapsibleFrame *registrationFrame = vtkSlicerModuleCollapsibleFrame::New ( );
  registrationFrame->SetParent(page);
  registrationFrame->Create();
  registrationFrame->CollapseFrame(); // 11/18/2011 ayamada

  // 8/4/2011 ayamada
  // list of defined point pairs 
  this->FiducialFrame = vtkKWFrameWithLabel::New();
  this->FiducialFrame->SetParent(registrationFrame->GetFrame());
  this->FiducialFrame->Create();
  this->FiducialFrame->SetLabelText("Fiducial Markers Setup from File");
  
  // 9/8/2011 ayamada
  this->SelectFiducialsFileButton = vtkKWLoadSaveButtonWithLabel::New();
  this->SelectFiducialsFileButton->SetParent(this->FiducialFrame->GetFrame());
  this->SelectFiducialsFileButton->Create();
  this->SelectFiducialsFileButton->GetWidget()->SetText ("Fiducial Markers File");
  this->SelectFiducialsFileButton->SetWidth(30);
  this->SelectFiducialsFileButton->GetWidget()->GetLoadSaveDialog()->SetFileTypes("{ {RobotProbeNav} {*.csv} }");
  this->SelectFiducialsFileButton->GetWidget()->GetLoadSaveDialog()
  ->RetrieveLastPathFromRegistry("OpenPath");
  
  this->SelectFiducialsFileButton->SetEnabled(1);

  
  // 9/8/2011 ayamada    
  this->AddFiducialFilePushButton = vtkKWPushButton::New();
  this->AddFiducialFilePushButton->SetParent(this->FiducialFrame->GetFrame());
  this->AddFiducialFilePushButton->Create();
  this->AddFiducialFilePushButton->SetText( "OK" );
  this->AddFiducialFilePushButton->SetWidth( 12 );
  
  this->AddFiducialFilePushButton->AddObserver ( vtkKWPushButton::InvokedEvent, 
                                                  (vtkCommand *)this->GUICallbackCommand );
  
  // 9/8/2011 ayamada
  // list of defined point pairs 
  this->FiducialFrame2 = vtkKWFrameWithLabel::New();
  this->FiducialFrame2->SetParent(registrationFrame->GetFrame());
  this->FiducialFrame2->Create();
  this->FiducialFrame2->SetLabelText("Fiducial Markers Setup from Input");
  
  // 8/4/2011 ayamada    
  this->FiducialMarkersSetup = vtkKWEntryWithLabel::New();
  this->FiducialMarkersSetup->SetParent(this->FiducialFrame2->GetFrame());
  this->FiducialMarkersSetup->Create();
  this->FiducialMarkersSetup->SetWidth(30);
  this->FiducialMarkersSetup->SetLabelWidth(13);
  this->FiducialMarkersSetup->SetLabelText("Input X Y Z:");
  this->FiducialMarkersSetup->GetWidget()->SetValue( "" );
    
  // 8/4/2011 ayamada    
  this->AddFiducialMarkerPushButton = vtkKWPushButton::New();
  this->AddFiducialMarkerPushButton->SetParent(this->FiducialFrame2->GetFrame());
  this->AddFiducialMarkerPushButton->Create();
  this->AddFiducialMarkerPushButton->SetText( "OK" );
  this->AddFiducialMarkerPushButton->SetWidth( 12 );
    
  this->AddFiducialMarkerPushButton->AddObserver ( vtkKWPushButton::InvokedEvent, 
                   (vtkCommand *)this->GUICallbackCommand );
  
  // 8/4/2011 ayamada
  // list of defined point pairs 
  this->ListFrame = vtkKWFrameWithLabel::New();
  this->ListFrame->SetParent(registrationFrame->GetFrame());
  this->ListFrame->Create();
  this->ListFrame->SetLabelText("Fiducial Markers List of Robot Coordinate");
    
  // 8/4/2011 ayamada from NeuroNav module
  // add the multicolumn list to show the points
  this->PointPairMultiColumnList = vtkKWMultiColumnListWithScrollbars::New();
  this->PointPairMultiColumnList->SetParent(this->ListFrame->GetFrame());
  this->PointPairMultiColumnList->Create();
  this->PointPairMultiColumnList->SetHeight(1);
  this->PointPairMultiColumnList->GetWidget()->SetSelectionTypeToRow();
  this->PointPairMultiColumnList->GetWidget()->MovableRowsOff();
  this->PointPairMultiColumnList->GetWidget()->MovableColumnsOff();
  // set up the columns of data for each point
  // refer to the header file for order
  this->PointPairMultiColumnList->GetWidget()->AddColumn("Numbers");
  this->PointPairMultiColumnList->GetWidget()->AddColumn("Robot Coordinates");

  // now set the attributes that are equal across the columns
  this->PointPairMultiColumnList->GetWidget()->SetColumnWidth(0, 8);
  this->PointPairMultiColumnList->GetWidget()->SetColumnAlignmentToLeft(0);
  this->PointPairMultiColumnList->GetWidget()->ColumnEditableOff(0);
    
  for (int col = 1; col < 2; col++)
  {
    this->PointPairMultiColumnList->GetWidget()->SetColumnWidth(col, 18);
    this->PointPairMultiColumnList->GetWidget()->SetColumnAlignmentToLeft(col);
    this->PointPairMultiColumnList->GetWidget()->ColumnEditableOff(col);
  }
    
  // 8/4/2011 ayamada
  // add a delete button 
  this->DeletePointPairPushButton = vtkKWPushButton::New();
  this->DeletePointPairPushButton->SetParent(this->ListFrame->GetFrame());
  this->DeletePointPairPushButton->Create();
  this->DeletePointPairPushButton->SetText("Delete Point");
  this->DeletePointPairPushButton->SetWidth(12);
  this->DeletePointPairPushButton->SetBalloonHelpString("Delete the selected point pair.");
    
  this->DeletePointPairPushButton->AddObserver ( vtkKWPushButton::InvokedEvent, 
                   (vtkCommand *)this->GUICallbackCommand );
    
  // add a delete button 
  this->DeleteAllPointPairPushButton = vtkKWPushButton::New();
  this->DeleteAllPointPairPushButton->SetParent(this->ListFrame->GetFrame());
  this->DeleteAllPointPairPushButton->Create();
  this->DeleteAllPointPairPushButton->SetText("Delete All Points");
  this->DeleteAllPointPairPushButton->SetWidth(12);
  this->DeleteAllPointPairPushButton->SetBalloonHelpString("Delete all point pairs.");
    
  this->DeleteAllPointPairPushButton->AddObserver ( vtkKWPushButton::InvokedEvent, 
                   (vtkCommand *)this->GUICallbackCommand );
  
  // 9/8/2011 ayamada
  this->ListFrame2 = vtkKWFrameWithLabel::New();
  this->ListFrame2->SetParent(registrationFrame->GetFrame());
  this->ListFrame2->Create();
  this->ListFrame2->SetLabelText("Fiducial Markers List of Image Coordinate");
  
  this->PointPairMultiColumnList2 = vtkKWMultiColumnListWithScrollbars::New();
  this->PointPairMultiColumnList2->SetParent(this->ListFrame2->GetFrame());
  this->PointPairMultiColumnList2->Create();
  this->PointPairMultiColumnList2->SetHeight(1);
  this->PointPairMultiColumnList2->GetWidget()->SetSelectionTypeToRow();
  this->PointPairMultiColumnList2->GetWidget()->MovableRowsOff();
  this->PointPairMultiColumnList2->GetWidget()->MovableColumnsOff();
  // set up the columns of data for each point
  // refer to the header file for order
  this->PointPairMultiColumnList2->GetWidget()->AddColumn("Numbers");
  this->PointPairMultiColumnList2->GetWidget()->AddColumn("Image Coordinates");
  
  // now set the attributes that are equal across the columns
  this->PointPairMultiColumnList2->GetWidget()->SetColumnWidth(0, 8);
  this->PointPairMultiColumnList2->GetWidget()->SetColumnAlignmentToLeft(0);
  this->PointPairMultiColumnList2->GetWidget()->ColumnEditableOff(0);
  
  for (int col = 1; col < 2; col++)
  {
    this->PointPairMultiColumnList2->GetWidget()->SetColumnWidth(col, 18);
    this->PointPairMultiColumnList2->GetWidget()->SetColumnAlignmentToLeft(col);
    this->PointPairMultiColumnList2->GetWidget()->ColumnEditableOff(col);
  }
  
  // add a delete button 
  this->DeletePointPairPushButton2 = vtkKWPushButton::New();
  this->DeletePointPairPushButton2->SetParent(this->ListFrame2->GetFrame());
  this->DeletePointPairPushButton2->Create();
  this->DeletePointPairPushButton2->SetText("Delete Point");
  this->DeletePointPairPushButton2->SetWidth(12);
  this->DeletePointPairPushButton2->SetBalloonHelpString("Delete the selected point pair.");
  
  this->DeletePointPairPushButton2->AddObserver ( vtkKWPushButton::InvokedEvent, 
                                                (vtkCommand *)this->GUICallbackCommand );
  
  // add a delete button 
  this->DeleteAllPointPairPushButton2 = vtkKWPushButton::New();
  this->DeleteAllPointPairPushButton2->SetParent(this->ListFrame2->GetFrame());
  this->DeleteAllPointPairPushButton2->Create();
  this->DeleteAllPointPairPushButton2->SetText("Delete All Points");
  this->DeleteAllPointPairPushButton2->SetWidth(12);
  this->DeleteAllPointPairPushButton2->SetBalloonHelpString("Delete all point pairs.");
  
  this->DeleteAllPointPairPushButton2->AddObserver ( vtkKWPushButton::InvokedEvent, 
                                                   (vtkCommand *)this->GUICallbackCommand );
    
  // 8/4/2011 ayamada
  this->FixFiducialMarkersPushButton = vtkKWPushButton::New();
  this->FixFiducialMarkersPushButton->SetParent(registrationFrame->GetFrame());
  this->FixFiducialMarkersPushButton->Create();
  this->FixFiducialMarkersPushButton->SetText( "Register" );
  this->FixFiducialMarkersPushButton->SetWidth( 12 );
    
  this->FixFiducialMarkersPushButton->AddObserver ( vtkKWPushButton::InvokedEvent, 
                   (vtkCommand *)this->GUICallbackCommand );

  registrationFrame->Delete();
  
}




//----------------------------------------------------------------------------
int vtkRobotProbeNavGUI::ChangeWorkphaseInGUI(int phase)
{
  vtkMRMLRobotProbeNavManagerNode *manager=this->GetRobotProbeNavManagerNode();
  if (manager==NULL)
  {
    return 0;
  }

  if (this->WorkphaseButtonSet==NULL)
  {
    // no GUI (e.g., exiting application)
    return 0;
  }

  vtkKWWizardStep* currentStep =  this->WizardWidget->GetWizardWorkflow()->GetCurrentStep();
  int currentPhase=-1;
  int numSteps = manager->GetNumberOfSteps();
  for (int i = 0; i < numSteps; i ++)
  {
    if (currentStep == GetStepPage(i))
    {
      currentPhase = i;
    }
  }
  
  // Switch Wizard Frame
  vtkKWWizardWorkflow *wizard = this->WizardWidget->GetWizardWorkflow();
  
  int steps =  phase - currentPhase;
  if (steps > 0)
    {
    for (int i = 0; i < steps; i ++) 
      {
      wizard->AttemptToGoToNextStep();
      }
    }
  else
    {
    steps = -steps;
    for (int i = 0; i < steps; i ++)
      {
      wizard->AttemptToGoToPreviousStep();
      }
    }    
  
  return 1;
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavGUI::UpdateGUI() // from MRML
{
  // 12/30/2011 ayamada
  // we know that we are updating the GUI and we don't need notifications about that (it's not changed by the user)
  int oldInGUICallbackFlag=GetInGUICallbackFlag();
  if (!oldInGUICallbackFlag)
  {
    SetInGUICallbackFlag(1);
  }

  // Update node selector widgets
  if (this->RobotProbeNavManagerSelectorWidget!= NULL)
  {
    vtkMRMLRobotProbeNavManagerNode *selManagerNode = vtkMRMLRobotProbeNavManagerNode::SafeDownCast(this->RobotProbeNavManagerSelectorWidget->GetSelected());
    
    // 7/25/2012 ayamada
    if (selManagerNode!=this->GetRobotProbeNavManagerNode())
    {    
      this->RobotProbeNavManagerSelectorWidget->SetSelected(this->GetRobotProbeNavManagerNode());
    }
  }
  if (this->RobotSelectorWidget != NULL)
  {
    vtkMRMLRobotNode *selRobotNode = vtkMRMLRobotNode::SafeDownCast(this->RobotSelectorWidget->GetSelected());
    if (selRobotNode!=this->GetRobotNode())
    {    
      this->RobotSelectorWidget->SetSelected(this->GetRobotNode());
    }
  }

  // Update the workphase and wizard frame
  UpdateStatusButtons();
  UpdateWorkflowSteps();

  vtkMRMLRobotProbeNavManagerNode *manager=this->GetRobotProbeNavManagerNode();
  
  // Enable robot selection only if a manager is selected
  if (this->RobotSelectorWidget!=NULL)
  {
    this->RobotSelectorWidget->SetEnabled(manager!=NULL);
  }

  if (this->WizardWidget!=NULL && manager!=NULL)
  {
    int stepId = manager->GetCurrentStep();    
    vtkRobotProbeNavStep *step=GetStepPage(stepId);
    if (step!=NULL)
      {
      step->UpdateGUI();
      }
  }
  
  // Update workflow button status (color, sunken)
  if (manager!=NULL && this->WorkphaseButtonSet!=NULL)
  {
    int numSteps = manager->GetNumberOfSteps();  
    for (int i = 0; i < numSteps; i ++)
    {
      vtkKWPushButton *pb = this->WorkphaseButtonSet->GetWidget(i);
      bool transitionable=true; // :TODO: get this information from the workflow widget state machine
      if (i == manager->GetCurrentStep())
      {
        pb->SetReliefToSunken();
      }
      else if (transitionable)
      {
        double r;
        double g;
        double b;
        GetStepPage(i)->GetTitleBackgroundColor(&r, &g, &b);

        pb->SetReliefToGroove();
        pb->SetStateToNormal();
        pb->SetBackgroundColor(r, g, b);
      }
      else
      {
        double r;
        double g;
        double b;
        GetStepPage(i)->GetTitleBackgroundColor(&r, &g, &b);
        r = r * 1.5; r = (r > 1.0) ? 1.0 : r;
        g = g * 1.5; g = (r > 1.0) ? 1.0 : g;
        b = b * 1.5; b = (r > 1.0) ? 1.0 : b;

        pb->SetReliefToGroove();
        pb->SetStateToDisabled();
        pb->SetBackgroundColor(r, g, b);
      }
    }
  }

  UpdateCurrentTargetDisplay(); // if a new node is added then it is selected by default => keep only the current target as selected
   
  // now InGUICallbackFlag==1, if previously it was ==0, then set it back to 0
  if (!oldInGUICallbackFlag)
  {
    SetInGUICallbackFlag(0);
  }
}


//----------------------------------------------------------------------------
vtkRobotProbeNavStep* vtkRobotProbeNavGUI::GetStepPage(int i)
{
  if (this->WizardWidget==NULL)
    {
    vtkErrorMacro("Invalid WizardWidget");
    return NULL;
    }
  vtkRobotProbeNavStep *step=vtkRobotProbeNavStep::SafeDownCast(this->WizardWidget->GetWizardWorkflow()->GetNthStep(i));
  if (step==NULL)
    {
    vtkErrorMacro("Invalid step page: "<<i);
    }
  return step;
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavGUI::UpdateStatusButtons()
{
  if (this->StatusButtonFrame==NULL)
  {
    return;
  }
  if (!this->StatusButtonFrame->IsCreated())
  {
    return;
  }

  vtkMRMLRobotProbeNavManagerNode *manager=this->GetRobotProbeNavManagerNode();  
  vtkMRMLRobotNode* robot=NULL;
  if (manager!=NULL)
  {
    manager->GetRobotNode();
  }
  
  // -----------------------------------------
  // there is no way to remove a button from the button set, so need to delete it, if less buttons are needed than we actually have
  int robotStatusDescriptorCount=0;
  if (robot!=NULL)
  {
    robotStatusDescriptorCount=robot->GetStatusDescriptorCount();
  }
  if (this->StatusButtonSet)
    {
    if (this->StatusButtonSet->GetNumberOfWidgets()>robotStatusDescriptorCount)
      {
      this->StatusButtonSet->SetParent(NULL);
      this->StatusButtonSet->Delete(); 
      this->StatusButtonSet = NULL;
      }
    }

  if (robot==NULL)
  {
    return;
  }

  if (this->StatusButtonSet==NULL)
    {
    this->StatusButtonSet = vtkKWPushButtonSet::New();
    this->StatusButtonSet->SetParent(this->StatusButtonFrame);
    this->StatusButtonSet->Create();
    this->StatusButtonSet->PackHorizontallyOn();
    this->StatusButtonSet->SetMaximumNumberOfWidgetsInPackingDirection(4);
    this->StatusButtonSet->SetWidgetsPadX(1);
    this->StatusButtonSet->SetWidgetsPadY(1);
    this->StatusButtonSet->UniformColumnsOn();
    this->StatusButtonSet->UniformRowsOn();
    this->Script("pack %s -side left -anchor w -fill x -padx 1 -pady 1", 
               this->StatusButtonSet->GetWidgetName());
    }
  
  int numStatDesc=robot->GetStatusDescriptorCount();
  for (int i = 0; i < numStatDesc; i ++)
  {
    std::string text;
    vtkMRMLRobotNode::STATUS_ID statusId=vtkMRMLRobotNode::StatusOff;
    robot->GetStatusDescriptor(i, text, statusId);    
    if (i>=this->StatusButtonSet->GetNumberOfWidgets())
    {
      this->StatusButtonSet->AddWidget(i);
    }
    vtkKWPushButton* button=this->StatusButtonSet->GetWidget(i);
    if (button==NULL)
    {
      vtkErrorMacro("Invalid button");
      continue;
    }
    button->SetText(text.c_str());
    button->SetBorderWidth(1);
    button->SetEnabled(false);
    switch (statusId)
    {
    case vtkMRMLRobotNode::StatusOff:
      button->SetDisabledForegroundColor(0.4, 0.4, 0.4);
      button->SetBackgroundColor(0.9, 0.9, 0.9);
      break;
    case vtkMRMLRobotNode::StatusOk:
      button->SetBackgroundColor(128.0/255.0,255.0/255.0,128.0/255.0);
      button->SetDisabledForegroundColor(0.1, 0.1, 0.1);
      break;
    case vtkMRMLRobotNode::StatusWarning:
      button->SetBackgroundColor(255.0/255.0,128.0/255.0,0.0/255.0);
      button->SetDisabledForegroundColor(0.0, 0.0, 0.0);  
      break;
    case vtkMRMLRobotNode::StatusError:
      button->SetBackgroundColor(1.0, 0, 0);
      button->SetDisabledForegroundColor(0.0, 0.0, 0.0);  
      break;
    default:
      button->SetBackgroundColor(1.0, 1.0, 0);
      button->SetDisabledForegroundColor(0.0, 0.0, 0.0);  
      break;
    }
  }

}

//----------------------------------------------------------------------------
void vtkRobotProbeNavGUI::UpdateWorkflowSteps()
{
  bool changed=true;
  vtkMRMLRobotProbeNavManagerNode *manager=this->GetRobotProbeNavManagerNode();
  if (manager!=NULL && this->DisplayedWorkflowSteps!=NULL)
  {
    int newSteps = manager->GetNumberOfSteps();
    int currentSteps = this->DisplayedWorkflowSteps->GetNumberOfValues();
    if (newSteps==currentSteps)
    {
      changed=false;
      for (int i = 0; i < newSteps; i ++)
      {
        vtkStdString stepName=manager->GetStepName(i);
        if (stepName.compare(this->DisplayedWorkflowSteps->GetValue(i))!=0)
        {
          changed=true;
          break;
        }
      }
    }
  }

  if (!changed)
  {
    return;
  }

  // the widget shall be dynamically created/destroyed when the
  // wizard GUI is built, because there is no API to remove any steps
  // from the wizard (only to add steps)

  if (this->DisplayedWorkflowSteps!=NULL)
  {
    this->DisplayedWorkflowSteps->Reset();
  }

  // Delete wizard widget
  if (this->WizardWidget)
  {

    vtkKWWizardWorkflow *wizard_workflow = this->WizardWidget->GetWizardWorkflow();

    // Hide current step
    vtkRobotProbeNavStep* currentStep=vtkRobotProbeNavStep::SafeDownCast(wizard_workflow->GetCurrentStep());
    if (currentStep)
    {
      currentStep->HideUserInterface();
    }

    // Tear down GUI for all steps before destroying
    for (int i=0; i<this->WizardWidget->GetWizardWorkflow()->GetNumberOfSteps(); i++)
    {
      vtkRobotProbeNavStep *step=vtkRobotProbeNavStep::SafeDownCast(this->WizardWidget->GetWizardWorkflow()->GetNthStep(i));
      if (step!=NULL)
      {
        step->TearDownGUI();
        step->SetGUI(NULL);
        step->SetLogic(NULL);
        step->SetAndObserveMRMLScene(NULL);
        step->SetRobotProbeNavManager(NULL);
      }
      else
      {
        vtkErrorMacro("Invalid step page: "<<i);
      }
    }

    this->WizardWidget->GetWizardWorkflow()->RemoveObserver((vtkCommand *)this->GUICallbackCommand);

    this->WizardWidget->SetParent(NULL);
    this->WizardWidget->Delete(); 
    this->WizardWidget = NULL;
  }

  // Delete workphase button set
  if (this->WorkphaseButtonSet)
  {
    for (int i = 0; i < this->WorkphaseButtonSet->GetNumberOfWidgets(); i ++)
    {
      this->WorkphaseButtonSet->GetWidget(i)->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }
    this->WorkphaseButtonSet->SetParent(NULL);
    this->WorkphaseButtonSet->Delete(); 
    this->WorkphaseButtonSet = NULL;
  }

  if (manager==NULL)
  {
    // there is no active manager node, the wizard frame shall be empty
    return;
  }

  if (this->WizardFrame==NULL)
  {
    // there is no GUI
    return;
  }
  if (!this->WizardFrame->IsCreated())
  {
    // there is no GUI
    return;
  }

  int numSteps = manager->GetNumberOfSteps();

  if (numSteps<1)
  {
    // no steps, the wizard frame shall be empty
    this->WizardFrame->CollapseFrame();
    return;
  }

  this->WizardFrame->ExpandFrame();

  // Recreate workphase button set
  this->WorkphaseButtonSet = vtkKWPushButtonSet::New();
  this->WorkphaseButtonSet->SetParent(this->WorkphaseButtonFrame);
  this->WorkphaseButtonSet->Create();
  this->WorkphaseButtonSet->PackHorizontallyOn();
  this->WorkphaseButtonSet->SetMaximumNumberOfWidgetsInPackingDirection(3);
  this->WorkphaseButtonSet->SetWidgetsPadX(1);
  this->WorkphaseButtonSet->SetWidgetsPadY(1);
  this->WorkphaseButtonSet->UniformColumnsOn();
  this->WorkphaseButtonSet->UniformRowsOn();  
  this->Script("pack %s -side left -anchor w -fill x -padx 1 -pady 1", 
    this->WorkphaseButtonSet->GetWidgetName());    

  // Recreate workphase wizard
  this->WizardWidget=vtkKWWizardWidget::New();   
  this->WizardWidget->SetParent(this->WizardFrame->GetFrame());
  this->WizardWidget->Create();
  this->WizardWidget->GetSubTitleLabel()->SetHeight(1);
  this->WizardWidget->SetClientAreaMinimumHeight(200);
  //this->WizardWidget->SetButtonsPositionToTop();
  this->WizardWidget->NextButtonVisibilityOn();
  this->WizardWidget->BackButtonVisibilityOn();
  this->WizardWidget->OKButtonVisibilityOff();
  this->WizardWidget->CancelButtonVisibilityOff();
  this->WizardWidget->FinishButtonVisibilityOff();
  this->WizardWidget->HelpButtonVisibilityOn();
  this->Script("pack %s -side top -anchor nw -fill both -expand y",
    this->WizardWidget->GetWidgetName());

  this->WizardWidget->GetWizardWorkflow()->AddObserver(vtkKWWizardWorkflow::CurrentStateChangedEvent,
      (vtkCommand *)this->GUICallbackCommand);

  // -----------------------------------------------------------------
  // Add the steps to the workflow

  vtkKWWizardWorkflow *wizard_workflow = this->WizardWidget->GetWizardWorkflow();

  // -----------------------------------------------------------------
  // Set GUI/Logic to each step and add to workflow  

  for (int i = 0; i < numSteps; i ++)
  {

    vtkStdString stepName=manager->GetStepName(i);

    this->DisplayedWorkflowSteps->InsertNextValue(stepName);

    vtkRobotProbeNavStep* newStep=NULL;

    if (!stepName.compare("SetUp"))
    {
      vtkRobotProbeNavStepSetUp* setupStep = vtkRobotProbeNavStepSetUp::New();
      setupStep->SetTitleBackgroundColor(205.0/255.0, 200.0/255.0, 177.0/255.0);
      newStep=setupStep;
    } 
    else if (!stepName.compare("SetUpTemplate"))
      {
      vtkRobotProbeNavStepSetUpTemplate* setupStep = vtkRobotProbeNavStepSetUpTemplate::New();
      setupStep->SetTitleBackgroundColor(205.0/255.0, 200.0/255.0, 177.0/255.0);
      newStep=setupStep;
      }
    else if (!stepName.compare("ZFrameCalibration"))
    {
      vtkRobotProbeNavCalibrationStep* calibrationStep = vtkRobotProbeNavCalibrationStep::New();
      calibrationStep->SetTitleBackgroundColor(193.0/255.0, 115.0/255.0, 80.0/255.0);
      newStep=calibrationStep;
    }
    else if (!stepName.compare("FiducialCalibration"))
    {
      vtkRobotProbeNavFiducialCalibrationStep* calibrationStep = vtkRobotProbeNavFiducialCalibrationStep::New();
      calibrationStep->SetTitleBackgroundColor(193.0/255.0, 115.0/255.0, 80.0/255.0);
      newStep=calibrationStep;
    }
    else if (!stepName.compare("PointTargetingWithoutOrientation"))
    {
      vtkRobotProbeNavTargetingStep* targetingStep = vtkRobotProbeNavTargetingStep::New();
      targetingStep->SetShowTargetOrientation(false);
      targetingStep->SetTitleBackgroundColor(138.0/255.0, 165.0/255.0, 111.0/255.0);
      newStep=targetingStep;
    }
    else if (!stepName.compare("PointTargeting"))
    {
      vtkRobotProbeNavTargetingStep* targetingStep = vtkRobotProbeNavTargetingStep::New();
      targetingStep->SetShowTargetOrientation(true);
      targetingStep->SetTitleBackgroundColor(138.0/255.0, 165.0/255.0, 111.0/255.0);
      newStep=targetingStep;
    }
    else if (!stepName.compare("TemplateTargeting"))
      {
      vtkRobotProbeNavStepTargetingTemplate* targetingStep = vtkRobotProbeNavStepTargetingTemplate::New();
      targetingStep->SetTitleBackgroundColor(138.0/255.0, 165.0/255.0, 111.0/255.0);
      newStep=targetingStep;
      }
    else if (!stepName.compare("IGTProbeRobotManualControl"))
    {
      // 7/1/2012 ayamada
      vtkRobotProbeNavManualControlStep* manualStep = vtkRobotProbeNavManualControlStep::New();
      manualStep->SetTitleBackgroundColor(179.0/255.0, 179.0/255.0, 230.0/255.0);
      newStep=manualStep;
    }
    else if (!stepName.compare("PointVerification"))
    {
      // 7/1/2012 ayamada
      vtkRobotProbeNavStepVerification* verificationStep = vtkRobotProbeNavStepVerification::New();
      verificationStep->SetTitleBackgroundColor(179.0/255.0, 145.0/255.0, 105.0/255.0);
      newStep=verificationStep;
    }
    else
    {
      vtkErrorMacro("Invalid step name: "<<stepName.c_str());
    }

    if (newStep!=NULL)
    {
      newStep->SetGUI(this);
      newStep->SetLogic(this->Logic);
      newStep->SetAndObserveMRMLScene(this->GetMRMLScene());
      newStep->SetRobotProbeNavManager(manager);
      newStep->SetTotalSteps(numSteps);
      newStep->SetStepNumber(i+1);
      newStep->UpdateName();

      wizard_workflow->AddNextStep(newStep);          

      this->WorkphaseButtonSet->AddWidget(i);  

      this->WorkphaseButtonSet->GetWidget(i)->AddObserver(vtkKWPushButton::InvokedEvent, (vtkCommand *)this->GUICallbackCommand);
      this->WorkphaseButtonSet->GetWidget(i)->SetWidth(16);                  
      this->WorkphaseButtonSet->GetWidget(i)->SetText(newStep->GetTitle());

      double r;
      double g;
      double b;
      newStep->GetTitleBackgroundColor(&r, &g, &b);
      this->WorkphaseButtonSet->GetWidget(i)->SetBackgroundColor(r, g, b);
      this->WorkphaseButtonSet->GetWidget(i)->SetActiveBackgroundColor(r, g, b);

      newStep->Delete();
    }      
  }
  
  // -----------------------------------------------------------------
  // Initial and finish step
  wizard_workflow->SetFinishStep(GetStepPage(numSteps-1));
  wizard_workflow->CreateGoToTransitionsToFinishStep();
  wizard_workflow->SetInitialStep(GetStepPage(0));

  // -----------------------------------------------------------------
  // Show the user interface
  vtkRobotProbeNavStep* step=vtkRobotProbeNavStep::SafeDownCast(wizard_workflow->GetCurrentStep());
  if (step)
    {
    step->ShowUserInterface();
    step->UpdateGUI();
    }
}

//--------------------------------------------------------------------------------
void vtkRobotProbeNavGUI::BringTargetToViewIn2DViews(int mode)
{
  vtkMRMLRobotProbeNavManagerNode *manager=GetRobotProbeNavManagerNode();
  if(manager==NULL)
  {
    return;
  }

  int currentTargetInd=manager->GetCurrentTargetIndex();
  if (currentTargetInd<0)
  {
    return;
  }
  
  vtkRobotProbeNavTargetDescriptor* targetDesc=manager->GetTargetDescriptorAtIndex(currentTargetInd);
  NeedleDescriptorStruct *needle = manager->GetNeedle(targetDesc); 
  if (targetDesc==NULL || needle==NULL)
  {
    vtkErrorMacro("No target or needle descriptor available for the current target");
    return;
  }  

  // get the point ras location of the target fiducial (P) that lies on the image plane
  double targetRAS[3];
  targetDesc->GetRASLocation(targetRAS);

  if (mode==BRING_MARKERS_TO_VIEW_KEEP_CURRENT_ORIENTATION)
  {
    // the slices may not be really orthogonal, they could be oblique
    // we could directly call slice node -> JumpAllSlices (r, a, s), this brings target in view
    // in all slices, but with target fiducial at the center of the view, moving (disturbing) the image altogether
    // for this function ->JumpSliceByOffsetting does the job
    BringMarkerToViewIn2DViews(targetRAS);
  }
  else
  {
    // Align the slices with the needle
    // Slice 1: orthogonal to the needle
    // Slice 2: aligned with the needle line and robot main axis
    // Slice 3: orthogonal to Slice 1 and 2


    vtkMRMLRobotNode* robot=manager->GetRobotNode();

    if (robot!=NULL)
    {      
      double needleVector[4]={0,1,0, 0};
      robot->GetNeedleDirectionAtTarget(targetDesc, needle, needleVector);
      double transverseVector[4]={0,0,1, 0};    
      // aligned transverse vector with robot base
      vtkSmartPointer<vtkMatrix4x4> transform=vtkSmartPointer<vtkMatrix4x4>::New();
      if (robot->GetRobotBaseTransform(transform))
      {
        double unalignedTransverseVector[4]={transverseVector[0],transverseVector[1],transverseVector[2], 0};
        transform->MultiplyPoint(unalignedTransverseVector, transverseVector);
      }
      BringMarkerToViewIn2DViews(targetRAS, needleVector, transverseVector);
    }
    else
    {
      BringMarkerToViewIn2DViews(targetRAS);
    }    
  }  
}

//--------------------------------------------------------------------------------
void vtkRobotProbeNavGUI::BringMarkerToViewIn2DViews(double* P, double* N/*=NULL*/, double* T/*=NULL*/)
{
  vtkSlicerSliceLogic *redSlice = vtkSlicerApplicationGUI::SafeDownCast(GetApplicationGUI())->GetApplicationLogic()->GetSliceLogic("Red");    
  vtkSlicerSliceLogic *yellowSlice = vtkSlicerApplicationGUI::SafeDownCast(GetApplicationGUI())->GetApplicationLogic()->GetSliceLogic("Yellow");    
  vtkSlicerSliceLogic *greenSlice = vtkSlicerApplicationGUI::SafeDownCast(GetApplicationGUI())->GetApplicationLogic()->GetSliceLogic("Green");    

  int redOldModify=redSlice->GetSliceNode()->StartModify();
  int yellowOldModify=yellowSlice->GetSliceNode()->StartModify();
  int greenOldModify=greenSlice->GetSliceNode()->StartModify();

  if (N!=NULL && T!=NULL) // slice orientation is specified
  {
    redSlice->GetSliceNode()->SetSliceToRASByNTP(N[0], N[1], N[2], T[0], T[1], T[2], P[0], P[1], P[2], 0);
    yellowSlice->GetSliceNode()->SetSliceToRASByNTP(N[0], N[1], N[2], T[0], T[1], T[2], P[0], P[1], P[2], 1);
    greenSlice->GetSliceNode()->SetSliceToRASByNTP(N[0], N[1], N[2], T[0], T[1], T[2], P[0], P[1], P[2], 2);
  }
  else
  {
    redSlice->GetSliceNode()->JumpSliceByOffsetting(P[0], P[1], P[2]);
    yellowSlice->GetSliceNode()->JumpSliceByOffsetting(P[0], P[1], P[2]);
    greenSlice->GetSliceNode()->JumpSliceByOffsetting(P[0], P[1], P[2]);
  }

  redSlice->GetSliceNode()->EndModify(redOldModify);
  yellowSlice->GetSliceNode()->EndModify(yellowOldModify);
  greenSlice->GetSliceNode()->EndModify(greenOldModify);
}


//--------------------------------------------------------------------------------
void vtkRobotProbeNavGUI::UpdateCurrentTargetDisplay()
{
  vtkMRMLRobotProbeNavManagerNode *manager=GetRobotProbeNavManagerNode();
  if(manager==NULL)
  {
    return;
  }

  vtkMRMLFiducialListNode* fidList = manager->GetTargetPlanListNode();
  if(fidList==NULL)
  {
    return;
  }

  int currentTargetInd=manager->GetCurrentTargetIndex();
  std::string selectedFidID="INVALID";  
  if (currentTargetInd>=0)
  {
    vtkRobotProbeNavTargetDescriptor* targetDesc=manager->GetTargetDescriptorAtIndex(currentTargetInd);
    if (targetDesc!=NULL)
    {
      selectedFidID=targetDesc->GetFiducialID();
    }
  }

  // Changing the fiducial selection state takes quite a long time. Check if we need modification, and request a modification
  // just if it is really necessary to avoid unnecessary lengthy updates.
  bool modificationStarted=false;
  int oldModify=0;
  for (int i = 0; i < fidList->GetNumberOfFiducials(); i ++)
  {              
    // select only the active target
    int selectedAlready=fidList->GetNthFiducialSelected(i);
    int selectionNeeded=selectedFidID.compare(fidList->GetNthFiducialID(i))==0;
    if (selectedAlready!=selectionNeeded)
    {
      if (!modificationStarted)
      {
        oldModify=fidList->StartModify();
        modificationStarted=true;
      }
      fidList->SetNthFiducialSelected(i, selectionNeeded);
    }
  }
  if (modificationStarted)
  {
    fidList->EndModify(oldModify);
    // StartModify/EndModify discarded vtkMRMLFiducialListNode::FiducialModifiedEvent-s, so we have to resubIGT them now
    fidList->InvokeEvent(vtkMRMLFiducialListNode::FiducialModifiedEvent, NULL);
  }  

  UpdateCurrentTargetDisplayInSecondaryWindow();
}

void vtkRobotProbeNavGUI::UpdateCurrentTargetDisplayInSecondaryWindow()
{  
  vtkMRMLRobotProbeNavManagerNode *manager=GetRobotProbeNavManagerNode();
  if(manager==NULL)
  {
    return;
  }

  if (this->SecondaryWindow==NULL || 
    this->SecondaryWindow->GetViewerWidget()==NULL ||
    this->SecondaryWindow->GetViewerWidget()->GetMainViewer()==NULL)
  {
    // Secondary window is not available
    return;
  }
  
  vtkCornerAnnotation *anno = this->SecondaryWindow->GetViewerWidget()->GetMainViewer()->GetCornerAnnotation();
  if (anno==NULL)
  {
    vtkErrorMacro("Corner annotation is not available");
    return;
  }

  static double lsf=10.0;
  static double nlsf=0.35;
  anno->SetLinearFontScaleFactor(lsf);
  anno->SetNonlinearFontScaleFactor(nlsf);

  if (anno->GetTextProperty()!=NULL)
  {
    anno->GetTextProperty()->ShadowOn();
    anno->GetTextProperty()->SetShadowOffset(2,2);
    anno->GetTextProperty()->BoldOn();
    anno->GetTextProperty()->SetColor(1.0,1.0,0.0); // yellow
  }
  
  vtkMRMLRobotNode* robot=manager->GetRobotNode();
  vtkRobotProbeNavTargetDescriptor *targetDesc = manager->GetTargetDescriptorAtIndex(manager->GetCurrentTargetIndex()); 
  NeedleDescriptorStruct *needle=manager->GetNeedle(targetDesc);
  std::string mainInfo;
  std::string additionalInfo;
  if (robot!=NULL && targetDesc!=NULL && needle!=NULL)
  {
    std::string info=robot->GetTargetInfoText(targetDesc, needle);
    robot->SplitTargetInfoText(info, mainInfo, additionalInfo);   
  }
  else
  {
    // no target info available for the current robot with the current target    
    mainInfo="No target";
  }
  const int TARGET_MAIN_INFO_CORNER_ID=3;
  const int TARGET_ADDITIONAL_INFO_CORNER_ID=1;
  anno->SetText(TARGET_MAIN_INFO_CORNER_ID, mainInfo.c_str());
  anno->SetText(TARGET_ADDITIONAL_INFO_CORNER_ID, additionalInfo.c_str());

  this->SecondaryWindow->GetViewerWidget()->GetMainViewer()->CornerAnnotationVisibilityOn();
  // update the annotations
  this->SecondaryWindow->GetViewerWidget()->GetMainViewer()->Render();
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavGUI::SetAndObserveRobotNodeID(const char *nodeID)
{
  bool modified=false;
  if (nodeID!=NULL && this->RobotNodeID!=NULL)
  {
    if (strcmp(nodeID, this->RobotNodeID)!=0)
    {
      modified=true;
    }
  }
  else if (nodeID!=this->RobotNodeID)
  {
    modified=true;
  }

  vtkSetAndObserveMRMLObjectMacro(this->RobotNode, NULL);
  this->SetRobotNodeID(nodeID);
  vtkMRMLRobotNode *tnode = this->GetRobotNode();

  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  events->InsertNextValue(vtkMRMLRobotNode::ChangeStatusEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->RobotNode, tnode, events);

  if (!modified)
  {
    return;
  }

  if (this->RobotNode!=NULL)
  {
    //this will fire a manager update message, which will trigger a GUI update
    vtksys_stl::string moduleShareDir=this->GetLogic()->GetModuleShareDirectory();
    this->RobotNode->Init(vtkSlicerApplication::SafeDownCast(this->GetApplication()), moduleShareDir.c_str()); // :TODO: init is called every time a robot is selected, however it would need to be called when the robot is created
  }

  UpdateGUI();

  vtkMRMLRobotProbeNavManagerNode *manager= this->GetRobotProbeNavManagerNode();
  if (manager!=NULL)
  {    
    ChangeWorkphaseInGUI(manager->GetCurrentStep()); //always start with the the first step
  }
  
  // 3/8/2012 ayamada
  //this->ModuleShareDirectory = this->GetLogic()->GetModuleShareDirectory(); 
  //std::cerr << "navgui = " << this->ModuleShareDirectory  << std::endl;
  
  
  
}

//----------------------------------------------------------------------------
vtkMRMLRobotNode* vtkRobotProbeNavGUI::GetRobotNode()
{
  vtkMRMLRobotNode* node = NULL;
  if (this->MRMLScene && this->RobotNodeID != NULL )
    {
    vtkMRMLNode* snode = this->MRMLScene->GetNodeByID(this->RobotNodeID);
    node = vtkMRMLRobotNode::SafeDownCast(snode);
    }
  return node;
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavGUI::SetAndObserveRobotProbeNavManagerNodeID(const char *nodeID)
{
  bool modified=false;
  if (nodeID!=NULL && this->RobotProbeNavManagerNodeID!=NULL)
  {
    if (strcmp(nodeID, this->RobotProbeNavManagerNodeID)!=0)
    {
      modified=true;
    }
  }
  else if (nodeID!=this->RobotProbeNavManagerNodeID)
  {
    modified=true;
  }

  vtkSetAndObserveMRMLObjectMacro(this->RobotProbeNavManagerNode, NULL);
  this->SetRobotProbeNavManagerNodeID(nodeID);
  vtkMRMLRobotProbeNavManagerNode *tnode = this->GetRobotProbeNavManagerNode();

  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  events->InsertNextValue(vtkMRMLRobotProbeNavManagerNode::CurrentTargetChangedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->RobotProbeNavManagerNode, tnode, events);

  if (!modified)
  {
    return;
  }

  // Add default needle list info
  if (this->RobotProbeNavManagerNode != NULL)
    {
    const char regSectionName[]="RobotProbeNav";
    const char regDefaultNeedleListKeyName[]="RobotProbeNav";      

    if (this->GetApplication()->HasRegistryValue (2, regSectionName, regDefaultNeedleListKeyName))
      {
      char *defNeedleDesc=NULL;
      this->GetApplication()->GetRegistryValue(2, regSectionName, regDefaultNeedleListKeyName, defNeedleDesc);
      this->RobotProbeNavManagerNode->Init(defNeedleDesc);
      delete[] defNeedleDesc;
      defNeedleDesc=NULL;
      }
    else
      {      
      this->GetApplication()->SetRegistryValue(2,"RobotProbeNav","DefaultNeedleList","%s",DEFAULT_NEEDLE_DESCRIPTION);
      this->RobotProbeNavManagerNode->Init(DEFAULT_NEEDLE_DESCRIPTION);          
      }

    }

  if (this->RobotProbeNavManagerNode!=NULL)
  {
    SetAndObserveRobotNodeID(this->RobotProbeNavManagerNode->GetRobotNodeID());
    SetAndObserveTargetPlanListNodeID(this->RobotProbeNavManagerNode->GetTargetPlanListNodeID());
  }
  else
  {
    SetAndObserveRobotNodeID(NULL);
    SetAndObserveTargetPlanListNodeID(NULL);
  }  
  
  // Update manager node in the workflow steps
  if (this->WizardWidget!=NULL)
  {
    for (int i=0; i<this->WizardWidget->GetWizardWorkflow()->GetNumberOfSteps(); i++)
    {
      vtkRobotProbeNavStep *step=vtkRobotProbeNavStep::SafeDownCast(this->WizardWidget->GetWizardWorkflow()->GetNthStep(i));
      if (step!=NULL)
        {
        step->SetRobotProbeNavManager(this->RobotProbeNavManagerNode);
        }
      else
        {
        vtkErrorMacro("Invalid step page: "<<i);
        }
    }
  }

  UpdateGUI();
}

//----------------------------------------------------------------------------
vtkMRMLRobotProbeNavManagerNode* vtkRobotProbeNavGUI::GetRobotProbeNavManagerNode()
{
  vtkMRMLRobotProbeNavManagerNode* node = NULL;
  if (this->MRMLScene && this->RobotProbeNavManagerNodeID != NULL )
    {
    vtkMRMLNode* snode = this->MRMLScene->GetNodeByID(this->RobotProbeNavManagerNodeID);
    node = vtkMRMLRobotProbeNavManagerNode::SafeDownCast(snode);
    }
  return node;
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavGUI::SetAndObserveTargetPlanListNodeID(const char *nodeID)
{
  bool modified=false;
  if (nodeID!=NULL && this->TargetPlanListNodeID!=NULL)
  {
    if (strcmp(nodeID, this->TargetPlanListNodeID)!=0)
    {
      modified=true;
    }
  }
  else if (nodeID!=this->TargetPlanListNodeID)
  {
    modified=true;
  }

  vtkSetAndObserveMRMLObjectMacro(this->TargetPlanListNode, NULL);
  this->SetTargetPlanListNodeID(nodeID);
  vtkMRMLFiducialListNode *tnode = this->GetTargetPlanListNode();

  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
  events->InsertNextValue(vtkMRMLFiducialListNode::DisplayModifiedEvent);
  events->InsertNextValue(vtkMRMLFiducialListNode::FiducialModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->TargetPlanListNode, tnode, events);

  if (!modified)
  {
    return;
  }
  
  UpdateCurrentTargetDisplay();
}

//----------------------------------------------------------------------------
vtkMRMLFiducialListNode* vtkRobotProbeNavGUI::GetTargetPlanListNode()
{
  vtkMRMLFiducialListNode* node = NULL;
  if (this->MRMLScene && this->TargetPlanListNodeID != NULL )
    {
    vtkMRMLNode* snode = this->MRMLScene->GetNodeByID(this->TargetPlanListNodeID);
    node = vtkMRMLFiducialListNode::SafeDownCast(snode);
    }
  return node;
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavGUI::RequestRenderInViewerWidgets()
{
  if (GetApplicationGUI()==NULL)
  {
    vtkWarningMacro("Application GUI is null");
    return;
  }

  int numberOfWidgets=GetApplicationGUI()->GetNumberOfViewerWidgets();
  for (int i=0; i<numberOfWidgets; i++)
  {
    vtkSlicerViewerWidget* widget=GetApplicationGUI()->GetNthViewerWidget(0); // main viewer is the first viewer
    if (widget!=NULL)
    {
      widget->RequestRender();
    }
  }
}


void vtkRobotProbeNavGUI::ShowSecondaryWindowCheckButtonCallback(int checked)
{
  // ShowSecondaryWindowCheckButton Pressed
  if (this->ShowSecondaryWindowCheckButton)
    {
    if (checked)
      {
       if (this->SecondaryWindow==NULL)
         {
         this->SecondaryWindow=vtkSlicerSecondaryViewerWindow::New();
         }
         if (!this->SecondaryWindow->IsCreated())
           {
           this->SecondaryWindow->SetApplication(this->GetApplication());
           this->SecondaryWindow->Create();
           vtkSlicerViewerWidget* viewerWidget=this->SecondaryWindow->GetViewerWidget();
           if (viewerWidget!=NULL && viewerWidget->GetViewNode()!=NULL)
             {
             viewerWidget->GetViewNode()->SetRenderMode(vtkMRMLViewNode::Orthographic);
             }
           }
         this->SecondaryWindow->DisplayOnSecondaryMonitor();
      } 
    else
    {
    }
    }
}


// 9/8/2011 ayamada
// based on a simulator of OpenIGTLink by J. Tokuda
//----------------------------------------------------------------------------
int vtkRobotProbeNavGUI::LoadCSVTrackingFile(const char * path)
{
 
  
  //this->Mutex->Lock();
  //this->TrackingData.clear();
  
  std::ifstream fin(path);
  std::string sr;
  
  /*
  if (!fin.is_open())
  {
    this->Mutex->Unlock();
    return 0;
  }
  */
  
  int switchFlag = 0;
  
  /*
  std::vector<double> row;
  row.resize(100);

  for (int i = 0; i < 100; i ++)
  {
    row[i] = 0.0;
  }
  */
  
  int i = 0;
  double d;  
  
  while (std::getline(fin, sr))
  {
    std::stringstream ssr(sr);
    std::string sc;
    while (std::getline(ssr, sc, ','))
    {
      std::stringstream ssc(sc);
      ssc >> d;
      if (i < 100/3)
      {
        
        if(switchFlag == 0){
          //row[i] = d;
          this->pX[i] = d;
          std::cerr << "this->pX[i] = " << this->pX[i] << std::endl;
          std::cerr << i << std::endl;
          switchFlag++;
        }
        else if(switchFlag == 1){
          //row[i] = d;
          this->pY[i] = d;
          std::cerr << "this->pY[i] = " << this->pY[i] << std::endl;
          std::cerr << i << std::endl;
          switchFlag++;
          
        }else if(switchFlag == 2){
          //row[i] = d;
          this->pZ[i] = d;
          std::cerr << "this->pZ[i] = " << this->pZ[i] << std::endl;
          std::cerr << i << std::endl;
          switchFlag = 0;   
          i++;
        }
                      
      }
      //i++;
      
      
    }
    //this->TrackingData.push_back(row);
  }
  
  //this->TrackingDataIndex = 0;
  //this->Mutex->Unlock();
  
  //return this->TrackingData.size();
  return i;
  
}

//----------------------------------------------------------------------------
int vtkRobotProbeNavGUI::LoadCSVTrackingFile2(const char * path)
{
  
  
  std::ifstream fin(path);
  std::string sr;
  
  int switchFlag = 0;  
  int i = 0;
  double d;  
  
  while (std::getline(fin, sr))
  {
    std::stringstream ssr(sr);
    std::string sc;
    while (std::getline(ssr, sc, ','))
    {
      std::stringstream ssc(sc);
      ssc >> d;
      if (i < 100/3)
      {
        
        if(switchFlag == 0){
          this->cX[i] = d;
          std::cerr << "this->cX[i] = " << this->cX[i] << std::endl;
          std::cerr << i << std::endl;
          switchFlag++;
        }
        else if(switchFlag == 1){
          this->cY[i] = d;
          std::cerr << "this->cY[i] = " << this->cY[i] << std::endl;
          std::cerr << i << std::endl;
          switchFlag++;
          
        }else if(switchFlag == 2){
          this->cZ[i] = d;
          std::cerr << "this->cZ[i] = " << this->cZ[i] << std::endl;
          std::cerr << i << std::endl;
          switchFlag = 0;   
          i++;
        }
        
      }
      
    }
  }
  
  return i;
  
}



