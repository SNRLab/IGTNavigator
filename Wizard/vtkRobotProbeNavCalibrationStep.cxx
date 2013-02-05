/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $  

==========================================================================*/

#include "vtkRobotProbeNavCalibrationStep.h"

#include "vtkRobotProbeNavGUI.h"
#include "vtkRobotProbeNavLogic.h"

#include "vtkKWFrame.h"
#include "vtkKWWizardWidget.h"
#include "vtkKWWizardWorkflow.h"
#include "vtkKWLoadSaveButton.h"
#include "vtkKWLoadSaveButtonWithLabel.h"
#include "vtkCylinderSource.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkTransform.h"
#include "vtkAppendPolyData.h"

#include "vtkSlicerVolumesGUI.h"
#include "vtkSlicerModuleGUI.h"
#include "vtkSlicerVolumesLogic.h"

#include "vtkMRMLModelNode.h"
#include "vtkMRMLModelDisplayNode.h"
#include "vtkMRMLLinearTransformNode.h"

#include "vtkMRMLVolumeNode.h"
#include "vtkMRMLScalarVolumeNode.h"
#include "vtkMRMLScalarVolumeDisplayNode.h"
#include "vtkMRMLVolumeHeaderlessStorageNode.h"
#include "vtkMRMLVolumeArchetypeStorageNode.h"
#include "vtkCollection.h"

#include "vtkSlicerColorLogic.h"

#include "vtkMRMLBrpRobotCommandNode.h"

#include "vtkMRMLRobotNode.h"

#include "vtkKWMatrixWidget.h"

// 12/25/2011 ayamada
#include "vtkKWMessageDialog.h"

// 6/28/2012 ayamada
#include "vtkMRMLAbdoNavNode.h"

// 3/8/2012 ayamada
#include "vtkJPEGReader.h"

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkRobotProbeNavCalibrationStep);
vtkCxxRevisionMacro(vtkRobotProbeNavCalibrationStep, "$Revision: 1.1 $");


//class vtkMRMLAbdoNavNode;

//---------------------------------------------------------------------------
vtkMRMLAbdoNavNode* vtkRobotProbeNavCalibrationStep::CheckAndCreateAbdoNavNode()
{

  if (this->AbdoNavNode == NULL && this->GetLogic()!=NULL)
  {
    
    // no AbdoNav node present yet, thus create a new one  
    vtkMRMLAbdoNavNode* node = vtkMRMLAbdoNavNode::New();

    
    // add the new node to this MRML scene but don't notify: 
    // this way, it is known that a node added event is only
    // invoked when the user loads a previously saved scene
    // containing an AbdoNavNode
    this->MRMLScene->AddNodeNoNotify(node);

        // set and observe the new node in Logic
        this->GetLogic()->SetAndObserveAbdoNavNode(node);
        // set and observe the new node in GUI

        vtkSetAndObserveMRMLNodeMacro(this->AbdoNavNode, node);
    
    node->Delete();

  }
  
  return this->AbdoNavNode;
}

//---------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::UpdateGUIFromMRML()
{
  vtkMRMLAbdoNavNode* node = this->AbdoNavNode;
  
  if (node != NULL)
  {
    // set GUI widgets from AbdoNav parameter node
    vtkMRMLNode* tnode = this->MRMLScene/*GetMRMLScene()*/->GetNodeByID(node->GetTrackingTransformID());
    
    vtkMRMLFiducialListNode* fnode = vtkMRMLFiducialListNode::SafeDownCast(this->MRMLScene/*GetMRMLScene()*/->GetNodeByID(node->GetRegistrationFiducialListID()));
    if (fnode != NULL)
    {
      // need to set all values to NaN in case AbdoNav's fiducial list was modified,
      // i.e. in case a fiducial was removed or renamed (unsupported identifier) ex-
      // ternally via the Fiducials module
      this->Point1REntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
      this->Point1AEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
      this->Point1SEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
      this->Point2REntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
      this->Point2AEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
      this->Point2SEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
      this->Point3REntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
      this->Point3AEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
      this->Point3SEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
      
      float* tmp = NULL;
      for (int i = 0; i < fnode->GetNumberOfFiducials(); i++)
      {
        // 12/27/2011 ayamada
        if (!strcmp(tip, fnode->GetNthFiducialLabelText(i)))
        {
          tmp = fnode->GetNthFiducialXYZ(i);
          this->Point1REntry->SetValueAsDouble(tmp[0]);
          this->Point1AEntry->SetValueAsDouble(tmp[1]);
          this->Point1SEntry->SetValueAsDouble(tmp[2]);
        }
        else if (!strcmp(markerA, fnode->GetNthFiducialLabelText(i)))
        {
          tmp = fnode->GetNthFiducialXYZ(i);
          this->Point2REntry->SetValueAsDouble(tmp[0]);
          this->Point2AEntry->SetValueAsDouble(tmp[1]);
          this->Point2SEntry->SetValueAsDouble(tmp[2]);
        }
        else if (!strcmp(markerB, fnode->GetNthFiducialLabelText(i)))
        {
          tmp = fnode->GetNthFiducialXYZ(i);
          this->Point3REntry->SetValueAsDouble(tmp[0]);
          this->Point3AEntry->SetValueAsDouble(tmp[1]);
          this->Point3SEntry->SetValueAsDouble(tmp[2]);
        }
        else if (!strcmp(markerC, fnode->GetNthFiducialLabelText(i)))
        {
          tmp = fnode->GetNthFiducialXYZ(i);
          this->Point4REntry->SetValueAsDouble(tmp[0]);
          this->Point4AEntry->SetValueAsDouble(tmp[1]);
          this->Point4SEntry->SetValueAsDouble(tmp[2]);
        }
        else if (!strcmp(markerD, fnode->GetNthFiducialLabelText(i)))
        {
          tmp = fnode->GetNthFiducialXYZ(i);
          this->Point5REntry->SetValueAsDouble(tmp[0]);
          this->Point5AEntry->SetValueAsDouble(tmp[1]);
          this->Point5SEntry->SetValueAsDouble(tmp[2]);
        }
        // 1/21/2012ayamada
        else if (!strcmp(markerE, fnode->GetNthFiducialLabelText(i)))
        {
          tmp = fnode->GetNthFiducialXYZ(i);
          this->Point6REntry->SetValueAsDouble(tmp[0]);
          this->Point6AEntry->SetValueAsDouble(tmp[1]);
          this->Point6SEntry->SetValueAsDouble(tmp[2]);
        }
      }
    }
  }
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::ProcessMRMLEvents(vtkObject* caller, unsigned long event, void* callData)
{
  
  
  if (event == vtkMRMLScene::NodeAddedEvent)
  {
    // update the GUI if an AbdoNavNode was added; an AbdoNavNode is only added
    // when the user loads a previously saved scene that contains an AbdoNavNode
    vtkMRMLAbdoNavNode* anode = vtkMRMLAbdoNavNode::SafeDownCast((vtkObject*)callData);
    if (anode != NULL)
    {
      // a new AbdoNavNode was created and added
      if (this->AbdoNavNode == NULL)
      {
        // set and observe the new node in Logic
        this->GetLogic()/*AbdoNavLogic*/->SetAndObserveAbdoNavNode(anode);
        // set and observe the new node in GUI
        vtkSetAndObserveMRMLNodeMacro(this->AbdoNavNode, anode);
        // if an AbdoNav fiducial list is part of the loaded scene, observe it in order to update the GUI
        vtkMRMLFiducialListNode* fiducialList = vtkMRMLFiducialListNode::SafeDownCast(this->MRMLScene/*GetMRMLScene()*/->GetNodeByID(this->AbdoNavNode->GetRegistrationFiducialListID()));
        if (fiducialList != NULL)
        {
          // observe fiducial list in order to update the GUI whenever a fiducial is moved via drag & drop or renamed or renumbered externally via the Fiducials module
          fiducialList->AddObserver(vtkMRMLFiducialListNode::FiducialModifiedEvent, (vtkCommand*)this->MRMLCallbackCommand);
          // observe fiducial list in order to update the GUI whenever a fiducial is added externally via the Fiducials module
          fiducialList->AddObserver(vtkMRMLScene::NodeAddedEvent, (vtkCommand*)this->MRMLCallbackCommand);
          // observe fiducial list in order to update the GUI whenever a fiducial is removed externally via the Fiducials module
          fiducialList->AddObserver(vtkMRMLScene::NodeRemovedEvent, (vtkCommand*)this->MRMLCallbackCommand);
          // observe fiducial list in order to update the GUI whenever all fiducials are removed externally via the Fiducials module
          fiducialList->AddObserver(vtkCommand::ModifiedEvent, (vtkCommand*)this->MRMLCallbackCommand);
          // no need to observe vtkMRMLFiducialListNode::DisplayModifiedEvent or vtkMRMLFiducialListNode::FiducialIndexModifiedEvent
        }
        this->UpdateGUIFromMRML();
      }
    }
    
    // update the GUI if a fiducial was added externally via the Fiducials module
    vtkMRMLFiducialListNode* fnode = vtkMRMLFiducialListNode::SafeDownCast(caller);
    if (fnode != NULL && this->AbdoNavNode != NULL)
    {
      if (!strcmp(fnode->GetID(), this->AbdoNavNode->GetRegistrationFiducialListID()))
      {
        //std::cout << "fiducial added" << std::endl;
        this->UpdateGUIFromMRML();
      }
    }
  }
  else if (event == vtkMRMLScene::NodeRemovedEvent)
  {
    // update the GUI if a fiducial was removed externally via the Fiducials module
    vtkMRMLFiducialListNode* fnode = vtkMRMLFiducialListNode::SafeDownCast(caller);
    if (fnode != NULL && this->AbdoNavNode != NULL)
    {
      if (!strcmp(fnode->GetID(), this->AbdoNavNode->GetRegistrationFiducialListID()))
      {
        //std::cout << "fiducial removed" << std::endl;
        this->UpdateGUIFromMRML();
      }
    }
  }
  else if (event == vtkCommand::ModifiedEvent)
  {
    // update the GUI if an existing AbdoNavNode was modified
    vtkMRMLAbdoNavNode* anode = vtkMRMLAbdoNavNode::SafeDownCast(caller);
    if (anode != NULL && this->AbdoNavNode != NULL)
    {
      if (!strcmp(anode->GetID(), this->AbdoNavNode->GetID()))
      {
        this->UpdateGUIFromMRML();
      }
    }
    
    // update the GUI if all fiducials were removed externally via the Fiducials module
    vtkMRMLFiducialListNode* fnode = vtkMRMLFiducialListNode::SafeDownCast(caller);
    if (fnode != NULL && this->AbdoNavNode != NULL)
    {
      if (!strcmp(fnode->GetID(), this->AbdoNavNode->GetRegistrationFiducialListID()))
      {
        //std::cout << "all fiducials removed" << std::endl;
        this->UpdateGUIFromMRML();
      }
    }
  }
  else if (event == vtkMRMLFiducialListNode::FiducialModifiedEvent)
  {
    // update the GUI if a fiducial was moved via drag & drop or renamed or renumbered externally via the Fiducials module
    vtkMRMLFiducialListNode* fnode = vtkMRMLFiducialListNode::SafeDownCast(caller);
    if (fnode != NULL && this->AbdoNavNode != NULL)
    {
      if (!strcmp(fnode->GetID(), this->AbdoNavNode->GetRegistrationFiducialListID()))
      {
        //std::cout << "fiducial moved or renamed" << std::endl;
        this->UpdateGUIFromMRML();
      }
    }
  }
  
  
}

//---------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::UpdateMRMLFromGUI()
{

  // 6/30/2012 ayamada
  // create an AbdoNavNode if none exists yet
  vtkMRMLAbdoNavNode* node = this->CheckAndCreateAbdoNavNode();
  
  // save old node parameters for undo mechanism
  this->GetLogic()/*AbdoNavLogic*/->GetMRMLScene()->SaveStateForUndo(node);
  
  // set new node parameters from GUI widgets:
  // make sure that only ONE modified event is invoked (if
  // at all) instead of one modified event per changed value
  int modifiedFlag = node->StartModify();
  node->EndModify(modifiedFlag);
  
 
}



//----------------------------------------------------------------------------
vtkRobotProbeNavCalibrationStep::vtkRobotProbeNavCalibrationStep()
{
  // 12/26/2011 ayamada
  this->SetTitle("Registration");  
  this->SetDescription("Perform point to point registration.");

  this->SelectImageFrame  = NULL;
  this->ZFrameImageSelectorWidget = NULL;
  this->SliceRangeMatrix = NULL;
  this->CalibrateButton   = NULL;

  this->ZFrameSettingFrame       = NULL;
  this->ShowZFrameCheckButton    = NULL;
  this->ShowWorkspaceCheckButton = NULL;

  // 12/24/2011 ayamada
  this->robotFrame = NULL;
  this->robotTransformSelector = NULL;
  this->robotPositionFrame = NULL;
  
  // 3/8/2012 ayamada
  this->referenceViewFrame = NULL;
  this->referenceImageSelecterButtonFrame = NULL;
  this->imageConfigurationButton1 = NULL;
  this->imageConfigurationButton2 = NULL;
  this->imageConfigurationButton3 = NULL;
  

  this->point1Frame = NULL;
  this->Point1CheckButton = NULL;
  this->Point1REntry = NULL;
  this->Point1AEntry = NULL;
  this->Point1SEntry = NULL;

  this->point2Frame = NULL;
  this->Point2CheckButton = NULL;
  this->Point2REntry = NULL;
  this->Point2AEntry = NULL;
  this->Point2SEntry = NULL;

  this->point3Frame = NULL;
  this->Point3CheckButton = NULL;
  this->Point3REntry = NULL;
  this->Point3AEntry = NULL;
  this->Point3SEntry = NULL;

  this->point4Frame = NULL;
  this->Point4CheckButton = NULL;
  this->Point4REntry = NULL;
  this->Point4AEntry = NULL;
  this->Point4SEntry = NULL;
  
  this->point5Frame = NULL;
  this->Point5CheckButton = NULL;
  this->Point5REntry = NULL;
  this->Point5AEntry = NULL;
  this->Point5SEntry = NULL;

  // 1/15/2012 ayamada  
  this->point6Frame = NULL;
  this->Point6CheckButton = NULL;
  this->Point6REntry = NULL;
  this->Point6AEntry = NULL;
  this->Point6SEntry = NULL;
  
  
  // 12/25/2011 ayamada
  this->AbdoNavNode = NULL;//vtkMRMLAbdoNavNode::New();//NULL;
  this->node = NULL;//vtkMRMLAbdoNavNode::New();//NULL;
  // 12/27/2011 ayamada
  this->FiducialMarkersSetup            = NULL;
  this->AddFiducialMarkerPushButton     = NULL;
  
  this->FiducialFrame                   = NULL;  
  this->SelectFiducialsFileButton       = NULL;
  this->AddFiducialFilePushButton       = NULL;

  this->ListFrame                       = NULL;
  this->PointPairMultiColumnList        = NULL;  
  this->FiducialNumber = 1;
  this->DeletePointPairPushButton       = NULL;
  this->DeleteAllPointPairPushButton    = NULL;
  this->FiducialNumber2 = 1;
  this->PointPairMultiColumnList2       = NULL;

  this->registrationMatrixSelectorFrame = NULL;  
  
  // 12/28/2011 ayamada
  //this->pX = NULL;
  //this->pY = NULL;
  //this->pZ = NULL;
  
  // 12/28/2011 ayamada
  this->ShowRobotButton=NULL;

  // 1/15/2012 ayamada
  this->CalibrateButton = NULL;
  this->ShowZFrameCheckButton = NULL;
  this->ShowWorkspaceCheckButton = NULL;
  this->robotTransformSelector = NULL;
  this->Point1CheckButton = NULL;
  this->Point2CheckButton = NULL;
  this->Point3CheckButton = NULL;
  this->Point4CheckButton = NULL;
  this->Point5CheckButton = NULL;
  this->Point6CheckButton = NULL;
  this->ResetRegistrationPushButton = NULL;
  this->PerformRegistrationPushButton = NULL;
  this->ShowRobotButton = NULL;
  
  // 3/8/2012 ayamada
  this->imageConfigurationButton1 = NULL;
  this->imageConfigurationButton2 = NULL;
  this->imageConfigurationButton3 = NULL;
  
  this->BackgroundRenderer = NULL;
  this->BackgroundActor = NULL;
  
  this->imageFilePath = (char*)calloc(500, sizeof(char));


  
  // 1/17/2012 ayamada
  this->LoadTargetingVolumeButton = NULL;
  this->VolumeSelectorWidget = NULL;  
  
  this->TimerLog = NULL;
  
}

//----------------------------------------------------------------------------
vtkRobotProbeNavCalibrationStep::~vtkRobotProbeNavCalibrationStep()
{
  if (this->SelectImageFrame)
    {
    this->SelectImageFrame->SetParent(NULL);
    this->SelectImageFrame->Delete();
    }
  if (this->ZFrameImageSelectorWidget)
    {
    this->ZFrameImageSelectorWidget->SetParent(NULL);
    this->ZFrameImageSelectorWidget->Delete();
    }
  if (this->SliceRangeMatrix)
    {
    this->SliceRangeMatrix->SetParent(NULL);
    this->SliceRangeMatrix->Delete();
    }
  if (this->CalibrateButton)
    {
    this->CalibrateButton->SetParent(NULL);
    this->CalibrateButton->Delete();
    }
  if (this->ZFrameSettingFrame)
    {
    this->ZFrameSettingFrame->SetParent(NULL);
    this->ZFrameSettingFrame->Delete();
    }
  if (this->ShowZFrameCheckButton)
    {
    this->ShowZFrameCheckButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand );
    this->ShowZFrameCheckButton->SetParent(NULL);
    this->ShowZFrameCheckButton->Delete();
    }
  if (this->ShowWorkspaceCheckButton)
    {
    this->ShowWorkspaceCheckButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand );
    this->ShowWorkspaceCheckButton->SetParent(NULL);
    this->ShowWorkspaceCheckButton->Delete();
    }
  
  // 12/24/2011 ayamada
  if (this->robotFrame)
  {
    this->robotFrame->SetParent(NULL);
    this->robotFrame->Delete();
  }
  if (this->robotTransformSelector)
  {
    this->robotTransformSelector->SetParent(NULL);
    this->robotTransformSelector->Delete();
  }
  if (this->robotPositionFrame)
  {
    this->robotPositionFrame->SetParent(NULL);
    this->robotPositionFrame->Delete();
  }
  
  // 3/8/2012 ayamada
  if (this->referenceViewFrame)
  {
    this->referenceViewFrame->SetParent(NULL);
    this->referenceViewFrame->Delete();
  }
  if (this->referenceImageSelecterButtonFrame)
  {
    this->referenceImageSelecterButtonFrame->SetParent(NULL);
    this->referenceImageSelecterButtonFrame->Delete();
  }
  
  if (this->point1Frame)
  {
    this->point1Frame->SetParent(NULL);
    this->point1Frame->Delete();
  }
  
  // 3/8/2012 ayamada
  if (this->imageConfigurationButton1)
  {
    this->imageConfigurationButton1->SetParent(NULL);
    this->imageConfigurationButton1->Delete();
  }
  if (this->imageConfigurationButton2)
  {
    this->imageConfigurationButton2->SetParent(NULL);
    this->imageConfigurationButton2->Delete();
  }
  if (this->imageConfigurationButton3)
  {
    this->imageConfigurationButton3->SetParent(NULL);
    this->imageConfigurationButton3->Delete();
  }
  
  
  if (this->Point1CheckButton)
  {
    this->Point1CheckButton->SetParent(NULL);
    this->Point1CheckButton->Delete();
  }
  if (this->Point1REntry)
  {
    this->Point1REntry->SetParent(NULL);
    this->Point1REntry->Delete();
  }
  if (this->Point1AEntry)
  {
    this->Point1AEntry->SetParent(NULL);
    this->Point1AEntry->Delete();
  }
  if (this->Point1SEntry)
  {
    this->Point1SEntry->SetParent(NULL);
    this->Point1SEntry->Delete();
  }
  
  if (this->point2Frame)
  {
    this->point2Frame->SetParent(NULL);
    this->point2Frame->Delete();
  }
  if (this->Point2CheckButton)
  {
    this->Point2CheckButton->SetParent(NULL);
    this->Point2CheckButton->Delete();
  }
  if (this->Point2REntry)
  {
    this->Point2REntry->SetParent(NULL);
    this->Point2REntry->Delete();
  }
  if (this->Point2AEntry)
  {
    this->Point2AEntry->SetParent(NULL);
    this->Point2AEntry->Delete();
  }
  if (this->Point2SEntry)
  {
    this->Point2SEntry->SetParent(NULL);
    this->Point2SEntry->Delete();
  }
  

  if (this->point3Frame)
  {
    this->point3Frame->SetParent(NULL);
    this->point3Frame->Delete();
  }
  if (this->Point3CheckButton)
  {
    this->Point3CheckButton->SetParent(NULL);
    this->Point3CheckButton->Delete();
  }
  if (this->Point3REntry)
  {
    this->Point3REntry->SetParent(NULL);
    this->Point3REntry->Delete();
  }
  if (this->Point3AEntry)
  {
    this->Point3AEntry->SetParent(NULL);
    this->Point3AEntry->Delete();
  }
  if (this->Point3SEntry)
  {
    this->Point3SEntry->SetParent(NULL);
    this->Point3SEntry->Delete();
  }
  

  if (this->point4Frame)
  {
    this->point4Frame->SetParent(NULL);
    this->point4Frame->Delete();
  }
  if (this->Point4CheckButton)
  {
    this->Point4CheckButton->SetParent(NULL);
    this->Point4CheckButton->Delete();
  }
  if (this->Point4REntry)
  {
    this->Point4REntry->SetParent(NULL);
    this->Point4REntry->Delete();
  }
  if (this->Point4AEntry)
  {
    this->Point4AEntry->SetParent(NULL);
    this->Point4AEntry->Delete();
  }
  if (this->Point4SEntry)
  {
    this->Point4SEntry->SetParent(NULL);
    this->Point4SEntry->Delete();
  }
  

  if (this->point5Frame)
  {
    this->point5Frame->SetParent(NULL);
    this->point5Frame->Delete();
  }
  if (this->Point5CheckButton)
  {
    this->Point5CheckButton->SetParent(NULL);
    this->Point5CheckButton->Delete();
  }
  if (this->Point5REntry)
  {
    this->Point5REntry->SetParent(NULL);
    this->Point5REntry->Delete();
  }
  if (this->Point5AEntry)
  {
    this->Point5AEntry->SetParent(NULL);
    this->Point5AEntry->Delete();
  }
  if (this->Point5SEntry)
  {
    this->Point5SEntry->SetParent(NULL);
    this->Point5SEntry->Delete();
  }


  // 1/21/2012 ayamada
  if (this->point6Frame)
  {
    this->point6Frame->SetParent(NULL);
    this->point6Frame->Delete();
  }
  if (this->Point6CheckButton)
  {
    this->Point6CheckButton->SetParent(NULL);
    this->Point6CheckButton->Delete();
  }
  if (this->Point6REntry)
  {
    this->Point6REntry->SetParent(NULL);
    this->Point6REntry->Delete();
  }
  if (this->Point6AEntry)
  {
    this->Point6AEntry->SetParent(NULL);
    this->Point6AEntry->Delete();
  }
  if (this->Point6SEntry)
  {
    this->Point6SEntry->SetParent(NULL);
    this->Point6SEntry->Delete();
  }
  
  
  
  // 12/26/2011 ayamada
  // this is important!!
  // if you forget to make this->AbdoNavNode NULL, you never create new node 
  // in this->CheckAndCreateAbdoNavNode() around l. 1050.  
  this->AbdoNavNode = NULL;
  
  // 12/27/2011 ayamada
  if (this->FiducialMarkersSetup)
  {
    this->FiducialMarkersSetup->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->FiducialMarkersSetup->SetParent(NULL);
    this->FiducialMarkersSetup->Delete();
    this->FiducialMarkersSetup = NULL;
  }

  if (this->AddFiducialMarkerPushButton)
  {
    this->AddFiducialMarkerPushButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->AddFiducialMarkerPushButton->SetParent(NULL );
    this->AddFiducialMarkerPushButton->Delete ( );
    this->AddFiducialMarkerPushButton = NULL;
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
  if (this->AddFiducialFilePushButton)
  {
    this->AddFiducialFilePushButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->AddFiducialFilePushButton->SetParent(NULL );
    this->AddFiducialFilePushButton->Delete ( );
    this->AddFiducialFilePushButton = NULL;
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
  
  if (this->registrationMatrixSelectorFrame)
  {
    this->registrationMatrixSelectorFrame->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->registrationMatrixSelectorFrame->SetParent(NULL );
    this->registrationMatrixSelectorFrame->Delete ( );
    this->registrationMatrixSelectorFrame = NULL;
  }  
  
  // 12/28/2011 ayamada
  //this->ShowRobotButton = NULL;
  if (this->ShowRobotButton)
  {
    this->ShowRobotButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->ShowRobotButton->SetParent(NULL );
    this->ShowRobotButton->Delete ( );
    this->ShowRobotButton = NULL;
  }
  
  // 12/30/2011 ayamada
  vtkSetAndObserveMRMLNodeMacro(this->AbdoNavNode, NULL);

  
  // 1/21/2012 ayamada
  if (this->LoadTargetingVolumeButton)
  {
    //this->LoadTargetingVolumeButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->LoadTargetingVolumeButton->SetParent(NULL );
    this->LoadTargetingVolumeButton->Delete ( );
    this->LoadTargetingVolumeButton = NULL;
  }
  
  if (this->VolumeSelectorWidget)
  {
    //this->LoadTargetingVolumeButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->VolumeSelectorWidget->SetParent(NULL );
    this->VolumeSelectorWidget->Delete ( );
    this->VolumeSelectorWidget = NULL;
  }
  
  
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::ShowUserInterface()
{

    
  this->Superclass::ShowUserInterface();

  vtkKWWizardWidget *wizardWidget = this->GetGUI()->GetWizardWidget();
  vtkKWWidget *parent = wizardWidget->GetClientArea();

  // Create frame
  if (!this->SelectImageFrame)
  {
    this->SelectImageFrame = vtkKWFrame::New();
  }
  if (!this->SelectImageFrame->IsCreated())
  {
    this->SelectImageFrame->SetParent(parent);
    this->SelectImageFrame->Create();
  }
  
  this->Script("pack %s -side top -anchor nw -expand n -fill x -padx 2 -pady 2",
               this->SelectImageFrame->GetWidgetName());  
  
  // -----------------------------------------
  // 12/24/2011 ayamada
  // Calibration Part based on AbdoNav
  
  if (!this->registrationMatrixSelectorFrame)
  {  
    this->registrationMatrixSelectorFrame = vtkKWFrameWithLabel::New();
    this->registrationMatrixSelectorFrame->SetParent(this->SelectImageFrame);
    this->registrationMatrixSelectorFrame->Create();
    this->registrationMatrixSelectorFrame->SetLabelText("Registration Matrix and Volume Select");
  
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2",
               this->registrationMatrixSelectorFrame->GetWidgetName());
  }  
  
  if (!this->robotTransformSelector)
  {  
    //----------------------------------------------------------------
    // Create widgets to specify the tracking information.
    //----------------------------------------------------------------
    // create selector to specify the input tracker transform node
    this->robotTransformSelector = vtkSlicerNodeSelectorWidget::New();
    this->robotTransformSelector->SetParent(this->registrationMatrixSelectorFrame->GetFrame());
    this->robotTransformSelector->Create();
    this->robotTransformSelector->SetLabelText("Registration matrix transform node:\t\t");
    this->robotTransformSelector->SetNodeClass("vtkMRMLLinearTransformNode", NULL, NULL, "LinearTransform"); // filter: only show vtkMRMLLinearTransformNodes
    this->robotTransformSelector->SetMRMLScene(this->MRMLScene/*this->GetMRMLScene()*/);
    this->robotTransformSelector->SetNewNodeEnabled(0); // turn off user option to create new linear transform nodes
    this->robotTransformSelector->SetDefaultEnabled(0); // turn off autoselecting nodes
    this->robotTransformSelector->GetWidget()->GetWidget()->IndicatorVisibilityOff(); // don't show indicator
    
    // add tracker transform selector
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->robotTransformSelector->GetWidgetName());
  }
  
  // 12/28/2011 ayamada
  if (!this->ShowRobotButton) 
  {
    this->ShowRobotButton = vtkKWCheckButton::New();    
    this->ShowRobotButton->SetParent(this->registrationMatrixSelectorFrame->GetFrame());
    this->ShowRobotButton->Create();
    this->ShowRobotButton->SelectedStateOff();
    this->ShowRobotButton->SetText("Show Robot");
    this->ShowRobotButton->SetBalloonHelpString("Show the robot");
    
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->ShowRobotButton->GetWidgetName());
    
  }
  
  
  // 1/21/2012 ayamada
  if (!this->LoadTargetingVolumeButton)
  {
    this->LoadTargetingVolumeButton = vtkKWPushButton::New();
    this->LoadTargetingVolumeButton->SetParent(this->registrationMatrixSelectorFrame->GetFrame());
    this->LoadTargetingVolumeButton->Create();
    this->LoadTargetingVolumeButton->SetBorderWidth(2);
    this->LoadTargetingVolumeButton->SetReliefToRaised();       
    this->LoadTargetingVolumeButton->SetHighlightThickness(2);
    this->LoadTargetingVolumeButton->SetBackgroundColor(0.85,0.85,0.85);
    this->LoadTargetingVolumeButton->SetActiveBackgroundColor(1,1,1);        
    this->LoadTargetingVolumeButton->SetText( "Load volume");
    this->LoadTargetingVolumeButton->SetBalloonHelpString("Click to load a volume. Need to additionally select the volume to make it the current targeting volume.");
    
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->LoadTargetingVolumeButton->GetWidgetName());
    
  }
  
  if (!this->VolumeSelectorWidget)
  {
    this->VolumeSelectorWidget = vtkSlicerNodeSelectorWidget::New();
    this->VolumeSelectorWidget->SetParent(this->registrationMatrixSelectorFrame->GetFrame());
    this->VolumeSelectorWidget->Create();
    this->VolumeSelectorWidget->SetBorderWidth(2);  
    this->VolumeSelectorWidget->SetNodeClass("vtkMRMLVolumeNode", NULL, NULL, NULL);
    this->VolumeSelectorWidget->SetMRMLScene(this->GetLogic()->GetApplicationLogic()->GetMRMLScene());
    this->VolumeSelectorWidget->SetNoneEnabled(true);
    this->VolumeSelectorWidget->GetWidget()->GetWidget()->IndicatorVisibilityOff();
    this->VolumeSelectorWidget->GetWidget()->GetWidget()->SetWidth(24);
    this->VolumeSelectorWidget->SetLabelText( "Targeting Volume: ");
    this->VolumeSelectorWidget->SetBalloonHelpString("Select the targeting volume from the current scene.");
    
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->VolumeSelectorWidget->GetWidgetName());
    
  }
  

  
  // 12/27/2011 ayamaada
  if (!this->FiducialFrame)
  {  
    // 8/4/2011 ayamada
    // list of defined point pairs 
    this->FiducialFrame = vtkKWFrameWithLabel::New();
    this->FiducialFrame->SetParent(this->SelectImageFrame);
    this->FiducialFrame->Create();
    this->FiducialFrame->SetLabelText("Fiducial Markers Setup from File");
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2",
                 this->FiducialFrame->GetWidgetName());
    
    // 9/8/2011 ayamada
    this->SelectFiducialsFileButton = vtkKWLoadSaveButtonWithLabel::New();
    this->SelectFiducialsFileButton->SetParent(this->FiducialFrame->GetFrame());
    this->SelectFiducialsFileButton->Create();
    this->SelectFiducialsFileButton->GetWidget()->SetText ("Fiducial Markers File");
    this->SelectFiducialsFileButton->SetWidth(30);
    this->SelectFiducialsFileButton->SetLabelWidth(13);
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
    
    
    this->Script("pack %s %s -side left -anchor w -padx 2 -pady 2", 
                 this->SelectFiducialsFileButton->GetWidgetName(),
                 this->AddFiducialFilePushButton->GetWidgetName());
    
  }
  
    
  // create labelled frame to hold widgets for specifying the tracking information
  if (!this->robotFrame)
  {
    this->robotFrame = vtkKWFrameWithLabel::New();
    this->robotFrame->SetParent(this->SelectImageFrame);
    this->robotFrame->Create();
    this->robotFrame->SetLabelText("Fiducial marker setup from input");
    this->robotFrame->CollapseFrame();
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->robotFrame->GetWidgetName());
  }
  
    
  if (!this->FiducialMarkersSetup)
  {
  // 8/4/2011 ayamada    
  this->FiducialMarkersSetup = vtkKWEntryWithLabel::New();
  this->FiducialMarkersSetup->SetParent(this->robotFrame->GetFrame());
  this->FiducialMarkersSetup->Create();
  this->FiducialMarkersSetup->SetWidth(30);
  this->FiducialMarkersSetup->SetLabelWidth(13);
  this->FiducialMarkersSetup->SetLabelText("Input X Y Z:");
  this->FiducialMarkersSetup->GetWidget()->SetValue( "" );
  
  // 8/4/2011 ayamada    
  this->AddFiducialMarkerPushButton = vtkKWPushButton::New();
  this->AddFiducialMarkerPushButton->SetParent(this->robotFrame->GetFrame());
  this->AddFiducialMarkerPushButton->Create();
  this->AddFiducialMarkerPushButton->SetText( "OK" );
  this->AddFiducialMarkerPushButton->SetWidth( 12 );
  
  this->AddFiducialMarkerPushButton->AddObserver ( vtkKWPushButton::InvokedEvent, 
                                                  (vtkCommand *)this->GUICallbackCommand );
    
  // 9/8/2011 ayamada
  this->Script("pack %s %s -side left -anchor w -padx 2 -pady 2", 
                this->FiducialMarkersSetup->GetWidgetName(),
                this->AddFiducialMarkerPushButton->GetWidgetName());     
    
  }  
  
  // 12/27/2011 ayamada

  if (!this->ListFrame)
  {  
  
  // 8/4/2011 ayamada
  // list of defined point pairs 
  this->ListFrame = vtkKWFrameWithLabel::New();
  this->ListFrame->SetParent(this->SelectImageFrame);
  this->ListFrame->Create();
  this->ListFrame->SetLabelText("Fiducial Markers List of Robot Coordinate");
  this->ListFrame->CollapseFrame();
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2",
               this->ListFrame->GetWidgetName());
  
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
  
  // make the selected column editable by checkbox
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
  
  this->Script ( "pack %s -fill both -expand true",
                this->PointPairMultiColumnList->GetWidgetName());
  
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
  
  this->Script("pack %s %s -side left -anchor w -padx 2 -pady 2", 
               this->DeletePointPairPushButton->GetWidgetName(),
               this->DeleteAllPointPairPushButton->GetWidgetName()); 
  
  }  
  
  // 3/8/2012 ayamada
  if (!this->robotPositionFrame)
  {  
    this->referenceViewFrame = vtkKWFrameWithLabel::New();
    this->referenceViewFrame->SetParent(this->SelectImageFrame);
    this->referenceViewFrame->Create();
    this->referenceViewFrame->SetLabelText("Reference images to check marker positions");
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->referenceViewFrame->GetWidgetName());

    // frame
    this->referenceImageSelecterButtonFrame = vtkKWFrame::New();
    this->referenceImageSelecterButtonFrame->SetParent(this->referenceViewFrame->GetFrame());
    this->referenceImageSelecterButtonFrame->Create();
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->referenceImageSelecterButtonFrame->GetWidgetName());

    // view configuration button
    this->imageConfigurationButton1 = vtkKWCheckButton::New();
    this->imageConfigurationButton1->SetParent(this->referenceImageSelecterButtonFrame);
    this->imageConfigurationButton1->Create();
    this->imageConfigurationButton1->SetText(":Top view");
    this->imageConfigurationButton1->SetSelectedState(false);

    this->imageConfigurationButton2 = vtkKWCheckButton::New();
    this->imageConfigurationButton2->SetParent(this->referenceImageSelecterButtonFrame);
    this->imageConfigurationButton2->Create();
    this->imageConfigurationButton2->SetText(":Side view");
    this->imageConfigurationButton2->SetSelectedState(false);
    
    this->imageConfigurationButton3 = vtkKWCheckButton::New();
    this->imageConfigurationButton3->SetParent(this->referenceImageSelecterButtonFrame);
    this->imageConfigurationButton3->Create();
    this->imageConfigurationButton3->SetText(":Front view");
    this->imageConfigurationButton3->SetSelectedState(false);

    this->Script ("pack %s %s %s -side left -anchor ne -padx 2 -pady 2",
                  this->imageConfigurationButton1->GetWidgetName(),
                  this->imageConfigurationButton2->GetWidgetName(),
                  this->imageConfigurationButton3->GetWidgetName());
    
  }

  // create labelled frame to hold widgets for identifying the guidance needle
  //vtkKWFrameWithLabel* guidanceNeedleFrame = vtkKWFrameWithLabel::New();
  if (!this->robotPositionFrame)
  {  
  this->robotPositionFrame = vtkKWFrameWithLabel::New();
  this->robotPositionFrame->SetParent(this->SelectImageFrame);
  this->robotPositionFrame->Create();
  this->robotPositionFrame->SetLabelText("Identify robot position");
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->robotPositionFrame->GetWidgetName());
  //}
  //----------------------------------------------------------------
  // Create widgets to identify the guidance needle tip.
  //----------------------------------------------------------------
  // create frame required to display the check button and RAS coordinate entries on the left and right side respectively
  this->point1Frame = vtkKWFrame::New();
  this->point1Frame->SetParent(this->robotPositionFrame->GetFrame());
  this->point1Frame->Create();
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->point1Frame->GetWidgetName());
  this->Point1CheckButton = vtkKWCheckButton::New();
  this->Point1CheckButton->SetParent(this->point1Frame);
  this->Point1CheckButton->Create();
  this->Point1CheckButton->SetText("Marker 1:");
  this->Point1CheckButton->SetBalloonHelpString("Identify the tip of the guidance needle in the CT/MR image.");
  this->Point1CheckButton->SetSelectedState(false);
  this->Point1REntry = vtkKWEntry::New();
  this->Point1REntry->SetParent(point1Frame);
  this->Point1REntry->Create();
  this->Point1REntry->SetBalloonHelpString("Guidance needle tip, R coordinate.");
  this->Point1REntry->SetWidth(8);
  this->Point1REntry->SetReadOnly(1);
  this->Point1REntry->SetRestrictValueToDouble();
  this->Point1REntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  this->Point1AEntry = vtkKWEntry::New();
  this->Point1AEntry->SetParent(point1Frame);
  this->Point1AEntry->Create();
  this->Point1AEntry->SetBalloonHelpString("Guidance needle tip, A coordinate.");
  this->Point1AEntry->SetWidth(8);
  this->Point1AEntry->SetReadOnly(1);
  this->Point1AEntry->SetRestrictValueToDouble();
  this->Point1AEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  this->Point1SEntry = vtkKWEntry::New();
  this->Point1SEntry->SetParent(point1Frame);
  this->Point1SEntry->Create();
  this->Point1SEntry->SetBalloonHelpString("Guidance needle tip, S coordinate.");
  this->Point1SEntry->SetWidth(8);
  this->Point1SEntry->SetReadOnly(1);
  this->Point1SEntry->SetRestrictValueToDouble();
  this->Point1SEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
    
  // add check button for the guidance needle tip
  this->Script ("pack %s -side left -anchor nw  -padx 2 -pady 2", this->Point1CheckButton->GetWidgetName());
  // add RAS coordinate entries for the guidance needle tip
  this->Script ("pack %s %s %s -side right -anchor ne -padx 2 -pady 2",
                this->Point1SEntry->GetWidgetName(),
                this->Point1AEntry->GetWidgetName(),
                this->Point1REntry->GetWidgetName());
  }  
  
  
  //----------------------------------------------------------------
  // Create widgets to identify marker A's center.
  //----------------------------------------------------------------
  // create frame required to display the check button and RAS coordinate entries on the left and right side respectively
  if (!this->point2Frame)
  {  
  this->point2Frame = vtkKWFrame::New();
  this->point2Frame->SetParent(this->robotPositionFrame->GetFrame());
  this->point2Frame->Create();
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->point2Frame->GetWidgetName());
  
  // create check button to select marker A's center
  this->Point2CheckButton = vtkKWCheckButton::New();
  this->Point2CheckButton->SetParent(this->point2Frame);
  this->Point2CheckButton->Create();
  this->Point2CheckButton->SetText("Marker 2:");
  this->Point2CheckButton->SetBalloonHelpString("Identify the center of marker A in the CT/MR image.");
  this->Point2CheckButton->SetSelectedState(false);
  this->Point2REntry = vtkKWEntry::New();
  this->Point2REntry->SetParent(point2Frame);
  this->Point2REntry->Create();
  this->Point2REntry->SetBalloonHelpString("Marker A's center, R coordinate.");
  this->Point2REntry->SetWidth(8);
  this->Point2REntry->SetReadOnly(1);
  this->Point2REntry->SetRestrictValueToDouble();
  this->Point2REntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  this->Point2AEntry = vtkKWEntry::New();
  this->Point2AEntry->SetParent(point2Frame);
  this->Point2AEntry->Create();
  this->Point2AEntry->SetBalloonHelpString("Marker A's center, A coordinate.");
  this->Point2AEntry->SetWidth(8);
  this->Point2AEntry->SetReadOnly(1);
  this->Point2AEntry->SetRestrictValueToDouble();
  this->Point2AEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  this->Point2SEntry = vtkKWEntry::New();
  this->Point2SEntry->SetParent(point2Frame);
  this->Point2SEntry->Create();
  this->Point2SEntry->SetBalloonHelpString("Marker A's center, S coordinate.");
  this->Point2SEntry->SetWidth(8);
  this->Point2SEntry->SetReadOnly(1);
  this->Point2SEntry->SetRestrictValueToDouble();
  this->Point2SEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  
  // add check button for marker A's center
  this->Script ("pack %s -side left -anchor nw  -padx 2 -pady 2", this->Point2CheckButton->GetWidgetName());
  // add RAS coordinate entries for marker A's center
  this->Script ("pack %s %s %s -side right -anchor ne -padx 2 -pady 2",
                this->Point2SEntry->GetWidgetName(),
                this->Point2AEntry->GetWidgetName(),
                this->Point2REntry->GetWidgetName());
  }  
  

  //----------------------------------------------------------------
  // Create widgets to identify marker B's center.
  //----------------------------------------------------------------
  // create frame required to display the check button and RAS coordinate entries on the left and right side respectively
  if (!this->point3Frame)
  {  
  this->point3Frame = vtkKWFrame::New();
  this->point3Frame->SetParent(this->robotPositionFrame->GetFrame());
  this->point3Frame->Create();
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->point3Frame->GetWidgetName());
  
  // create check button to select marker B's center
  this->Point3CheckButton = vtkKWCheckButton::New();
  this->Point3CheckButton->SetParent(this->point3Frame);
  this->Point3CheckButton->Create();
  this->Point3CheckButton->SetText("Marker 3:");
  this->Point3CheckButton->SetBalloonHelpString("Identify the center of marker B in the CT/MR image.");
  this->Point3CheckButton->SetSelectedState(false);
  this->Point3REntry = vtkKWEntry::New();
  this->Point3REntry->SetParent(point3Frame);
  this->Point3REntry->Create();
  this->Point3REntry->SetBalloonHelpString("Marker B's center, R coordinate.");
  this->Point3REntry->SetWidth(8);
  this->Point3REntry->SetReadOnly(1);
  this->Point3REntry->SetRestrictValueToDouble();
  this->Point3REntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  this->Point3AEntry = vtkKWEntry::New();
  this->Point3AEntry->SetParent(point3Frame);
  this->Point3AEntry->Create();
  this->Point3AEntry->SetBalloonHelpString("Marker B's center, A coordinate.");
  this->Point3AEntry->SetWidth(8);
  this->Point3AEntry->SetReadOnly(1);
  this->Point3AEntry->SetRestrictValueToDouble();
  this->Point3AEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  this->Point3SEntry = vtkKWEntry::New();
  this->Point3SEntry->SetParent(point3Frame);
  this->Point3SEntry->Create();
  this->Point3SEntry->SetBalloonHelpString("Marker B's center, S coordinate.");
  this->Point3SEntry->SetWidth(8);
  this->Point3SEntry->SetReadOnly(1);
  this->Point3SEntry->SetRestrictValueToDouble();
  this->Point3SEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  
  // add check button for marker B's center
  this->Script ("pack %s -side left -anchor nw  -padx 2 -pady 2", this->Point3CheckButton->GetWidgetName());
  // add RAS coordinate entries for marker B's center
  this->Script ("pack %s %s %s -side right -anchor ne -padx 2 -pady 2",
                this->Point3SEntry->GetWidgetName(),
                this->Point3AEntry->GetWidgetName(),
                this->Point3REntry->GetWidgetName());
  }  

  
  //----------------------------------------------------------------
  // Create widgets to identify marker C's center.
  //----------------------------------------------------------------
  // create frame required to display the check button and RAS coordinate entries on the left and right side respectively
  if (!this->point4Frame)
  {  
  this->point4Frame = vtkKWFrame::New();
  this->point4Frame->SetParent(this->robotPositionFrame->GetFrame());
  this->point4Frame->Create();
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->point4Frame->GetWidgetName());
  
  // create check button to select marker C's center
  this->Point4CheckButton = vtkKWCheckButton::New();
  this->Point4CheckButton->SetParent(this->point4Frame);
  this->Point4CheckButton->Create();
  this->Point4CheckButton->SetText("Marker 4:");
  this->Point4CheckButton->SetBalloonHelpString("Identify the center of marker C in the CT/MR image.");
  this->Point4CheckButton->SetSelectedState(false);
  this->Point4REntry = vtkKWEntry::New();
  this->Point4REntry->SetParent(point4Frame);
  this->Point4REntry->Create();
  this->Point4REntry->SetBalloonHelpString("Marker C's center, R coordinate.");
  this->Point4REntry->SetWidth(8);
  this->Point4REntry->SetReadOnly(1);
  this->Point4REntry->SetRestrictValueToDouble();
  this->Point4REntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  // create entry to hold the A coordinate of marker C's center
  this->Point4AEntry = vtkKWEntry::New();
  this->Point4AEntry->SetParent(point4Frame);
  this->Point4AEntry->Create();
  this->Point4AEntry->SetBalloonHelpString("Marker C's center, A coordinate.");
  this->Point4AEntry->SetWidth(8);
  this->Point4AEntry->SetReadOnly(1);
  this->Point4AEntry->SetRestrictValueToDouble();
  this->Point4AEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  // create entry to hold the S coordinate of marker C's center
  this->Point4SEntry = vtkKWEntry::New();
  this->Point4SEntry->SetParent(point4Frame);
  this->Point4SEntry->Create();
  this->Point4SEntry->SetBalloonHelpString("Marker C's center, S coordinate.");
  this->Point4SEntry->SetWidth(8);
  this->Point4SEntry->SetReadOnly(1);
  this->Point4SEntry->SetRestrictValueToDouble();
  this->Point4SEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  
  // add check button for marker C's center
  this->Script ("pack %s -side left -anchor nw  -padx 2 -pady 2", this->Point4CheckButton->GetWidgetName());
  // add RAS coordinate entries for marker C's center
  this->Script ("pack %s %s %s -side right -anchor ne -padx 2 -pady 2",
                this->Point4SEntry->GetWidgetName(),
                this->Point4AEntry->GetWidgetName(),
                this->Point4REntry->GetWidgetName());
  }
  

  //----------------------------------------------------------------
  // Create widgets to identify marker D's center.
  //----------------------------------------------------------------
  // create frame required to display the check button and RAS coordinate entries on the left and right side respectively
  if (!this->point5Frame)
  {  
  this->point5Frame = vtkKWFrame::New();
  this->point5Frame->SetParent(this->robotPositionFrame->GetFrame());
  this->point5Frame->Create();
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->point5Frame->GetWidgetName());
  
  // create check button to select marker D's center
  this->Point5CheckButton = vtkKWCheckButton::New();
  this->Point5CheckButton->SetParent(this->point5Frame);
  this->Point5CheckButton->Create();
  this->Point5CheckButton->SetText("Marker 5:");
  this->Point5CheckButton->SetBalloonHelpString("Identify the center of marker D in the CT/MR image.");
  this->Point5CheckButton->SetSelectedState(false);
  // create entry to hold the R coordinate of marker D's center
  this->Point5REntry = vtkKWEntry::New();
  this->Point5REntry->SetParent(point5Frame);
  this->Point5REntry->Create();
  this->Point5REntry->SetBalloonHelpString("Marker D's center, R coordinate.");
  this->Point5REntry->SetWidth(8);
  this->Point5REntry->SetReadOnly(1);
  this->Point5REntry->SetRestrictValueToDouble();
  this->Point5REntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  // create entry to hold the A coordinate of marker D's center
  this->Point5AEntry = vtkKWEntry::New();
  this->Point5AEntry->SetParent(point5Frame);
  this->Point5AEntry->Create();
  this->Point5AEntry->SetBalloonHelpString("Marker D's center, A coordinate.");
  this->Point5AEntry->SetWidth(8);
  this->Point5AEntry->SetReadOnly(1);
  this->Point5AEntry->SetRestrictValueToDouble();
  this->Point5AEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  // create entry to hold the S coordinate of marker D's center
  this->Point5SEntry = vtkKWEntry::New();
  this->Point5SEntry->SetParent(point5Frame);
  this->Point5SEntry->Create();
  this->Point5SEntry->SetBalloonHelpString("Marker D's center, S coordinate.");
  this->Point5SEntry->SetWidth(8);
  this->Point5SEntry->SetReadOnly(1);
  this->Point5SEntry->SetRestrictValueToDouble();
  this->Point5SEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
  
  // add check button for marker D's center
  this->Script ("pack %s -side left -anchor nw  -padx 2 -pady 2", this->Point5CheckButton->GetWidgetName());
  // add RAS coordinate entries for marker D's center
  this->Script ("pack %s %s %s -side right -anchor ne -padx 2 -pady 2",
                this->Point5SEntry->GetWidgetName(),
                this->Point5AEntry->GetWidgetName(),
                this->Point5REntry->GetWidgetName());
  }

  // 12/27/2011 ayamada
  //----------------------------------------------------------------
  // Create widgets to identify marker D's center.
  //----------------------------------------------------------------
  // create frame required to display the check button and RAS coordinate entries on the left and right side respectively
  if (!this->point6Frame)
  {  
    this->point6Frame = vtkKWFrame::New();
    this->point6Frame->SetParent(this->robotPositionFrame->GetFrame());
    this->point6Frame->Create();
    this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2", this->point6Frame->GetWidgetName());
    
    // create check button to select marker D's center
    this->Point6CheckButton = vtkKWCheckButton::New();
    this->Point6CheckButton->SetParent(this->point6Frame);
    this->Point6CheckButton->Create();
    this->Point6CheckButton->SetText("Marker 6:");
    this->Point6CheckButton->SetBalloonHelpString("Identify the center of marker D in the CT/MR image.");
    this->Point6CheckButton->SetSelectedState(false);
    // create entry to hold the R coordinate of marker D's center
    this->Point6REntry = vtkKWEntry::New();
    this->Point6REntry->SetParent(point6Frame);
    this->Point6REntry->Create();
    this->Point6REntry->SetBalloonHelpString("Marker D's center, R coordinate.");
    this->Point6REntry->SetWidth(8);
    this->Point6REntry->SetReadOnly(1);
    this->Point6REntry->SetRestrictValueToDouble();
    this->Point6REntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
    // create entry to hold the A coordinate of marker D's center
    this->Point6AEntry = vtkKWEntry::New();
    this->Point6AEntry->SetParent(point6Frame);
    this->Point6AEntry->Create();
    this->Point6AEntry->SetBalloonHelpString("Marker D's center, A coordinate.");
    this->Point6AEntry->SetWidth(8);
    this->Point6AEntry->SetReadOnly(1);
    this->Point6AEntry->SetRestrictValueToDouble();
    this->Point6AEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
    // create entry to hold the S coordinate of marker D's center
    this->Point6SEntry = vtkKWEntry::New();
    this->Point6SEntry->SetParent(point6Frame);
    this->Point6SEntry->Create();
    this->Point6SEntry->SetBalloonHelpString("Marker D's center, S coordinate.");
    this->Point6SEntry->SetWidth(8);
    this->Point6SEntry->SetReadOnly(1);
    this->Point6SEntry->SetRestrictValueToDouble();
    this->Point6SEntry->SetValueAsDouble(std::numeric_limits<double>::quiet_NaN());
    
    // add check button for marker D's center
    this->Script ("pack %s -side left -anchor nw  -padx 2 -pady 2", this->Point6CheckButton->GetWidgetName());
    // add RAS coordinate entries for marker D's center
    this->Script ("pack %s %s %s -side right -anchor ne -padx 2 -pady 2",
                  this->Point6SEntry->GetWidgetName(),
                  this->Point6AEntry->GetWidgetName(),
                  this->Point6REntry->GetWidgetName());
  }
  
  
  
  //----------------------------------------------------------------
  // Create buttons to reset and perform the registration.
  //----------------------------------------------------------------
  // create button to reset the registration
  if (!this->ResetRegistrationPushButton)
  {  
  this->ResetRegistrationPushButton = vtkKWPushButton::New();
  this->ResetRegistrationPushButton->SetParent(this->SelectImageFrame);
  this->ResetRegistrationPushButton->Create();
  this->ResetRegistrationPushButton->SetText("Reset Registration");
  this->ResetRegistrationPushButton->SetBalloonHelpString("Redo identification of guidance needle.");
  this->ResetRegistrationPushButton->SetEnabled(false);
  
  // add reset registration button
  this->Script("pack %s -side left -anchor nw -padx 2 -pady 2", this->ResetRegistrationPushButton->GetWidgetName());
  
  // create button to perform the registration
  this->PerformRegistrationPushButton = vtkKWPushButton::New();
  this->PerformRegistrationPushButton->SetParent(this->SelectImageFrame);
  this->PerformRegistrationPushButton->Create();
  this->PerformRegistrationPushButton->SetText("Perform Registration");
  this->PerformRegistrationPushButton->SetBalloonHelpString("Perform registration based on current identification of guidance needle.");
  
  // add perform registration button
  this->Script("pack %s -side right -anchor ne -padx 2 -pady 2", this->PerformRegistrationPushButton->GetWidgetName());
  
  // clean up
  this->robotFrame->Delete();
  this->robotPositionFrame->Delete();
  this->point1Frame->Delete();
  this->point2Frame->Delete();
  this->point3Frame->Delete();
  this->point4Frame->Delete();
  this->point5Frame->Delete();
  // 1/21/2012 ayamada
  this->point6Frame->Delete();
  }
  
  // -----------------------------------------
  

  if (!this->ZFrameImageSelectorWidget)
    {
    this->ZFrameImageSelectorWidget = vtkSlicerNodeSelectorWidget::New() ;
    this->ZFrameImageSelectorWidget->SetParent(this->SelectImageFrame);
    this->ZFrameImageSelectorWidget->Create(); 
    this->ZFrameImageSelectorWidget->AddNodeClass("vtkMRMLScalarVolumeNode", NULL, NULL, NULL);
    this->ZFrameImageSelectorWidget->SetMRMLScene(this->MRMLScene);
    this->ZFrameImageSelectorWidget->SetBorderWidth(2);
    this->ZFrameImageSelectorWidget->GetWidget()->GetWidget()->IndicatorVisibilityOff();
    this->ZFrameImageSelectorWidget->GetWidget()->GetWidget()->SetWidth(24);
    this->ZFrameImageSelectorWidget->SetLabelText( "ZFrame Image: ");
    this->ZFrameImageSelectorWidget->NewNodeEnabledOn();
    this->ZFrameImageSelectorWidget->SetBalloonHelpString("Select Z-frame image node");
    this->ZFrameImageSelectorWidget->SetEnabled(1);
    }

  if (!this->SliceRangeMatrix)
    {
    this->SliceRangeMatrix = vtkKWMatrixWidgetWithLabel::New();
    this->SliceRangeMatrix->SetParent(this->SelectImageFrame);
    this->SliceRangeMatrix->Create();
    this->SliceRangeMatrix->SetLabelText("Slice range:");
    this->SliceRangeMatrix->ExpandWidgetOff();
    this->SliceRangeMatrix->GetLabel()->SetWidth(18);
    this->SliceRangeMatrix->SetBalloonHelpString("Set the needle position");

    vtkKWMatrixWidget *matrix =  this->SliceRangeMatrix->GetWidget();
    matrix->SetNumberOfColumns(2);
    matrix->SetNumberOfRows(1);
    matrix->SetElementWidth(12);
    matrix->SetRestrictElementValueToInteger();
    matrix->SetElementChangedCommandTriggerToAnyChange();
    matrix->SetElementValueAsInt(0, 0, 1);
    matrix->SetElementValueAsInt(0, 1, 12);
    }


  if (!this->CalibrateButton)
    {
    this->CalibrateButton = vtkKWPushButton::New();
    this->CalibrateButton->SetParent (this->SelectImageFrame);
    this->CalibrateButton->Create ( );
    this->CalibrateButton->SetText ("Perform Calibration");
    this->CalibrateButton->SetBalloonHelpString("Send Calibration Data to the Robot");
    }

  
  if (!this->ZFrameSettingFrame)
    {
    this->ZFrameSettingFrame = vtkKWFrame::New();
    this->ZFrameSettingFrame->SetParent(parent);
    this->ZFrameSettingFrame->Create();

  this->Script("pack %s -side top -anchor nw -expand n -padx 2 -pady 2",
               this->ZFrameSettingFrame->GetWidgetName());
    }
  
  if (!this->ShowZFrameCheckButton)
    {
    this->ShowZFrameCheckButton = vtkKWCheckButton::New();
    this->ShowZFrameCheckButton->SetParent(this->ZFrameSettingFrame);
    this->ShowZFrameCheckButton->Create();
    this->ShowZFrameCheckButton->SelectedStateOff();
    this->ShowZFrameCheckButton->SetText("Show ZFrame");   
    }

  if (!this->ShowWorkspaceCheckButton)
    {
    this->ShowWorkspaceCheckButton = vtkKWCheckButton::New();
    this->ShowWorkspaceCheckButton->SetParent(this->ZFrameSettingFrame);
    this->ShowWorkspaceCheckButton->Create();
    this->ShowWorkspaceCheckButton->SelectedStateOff();
    this->ShowWorkspaceCheckButton->SetText("Show Range of Motion");
    }

  this->AddGUIObservers();
  
  // 12/26/2011 ayamada
  if(this->GetLogic())
  {
    this->GetLogic()->workPhaseStatus = 1;
  }
  
  
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::HideUserInterface()
{
  Superclass::HideUserInterface();
  RemoveGUIObservers();
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::ProcessGUIEvents(vtkObject *caller,
                                           unsigned long event, void *callData)
{

  if(this->GetLogic()->workPhaseStatus == 1)  
  {
  if (this->CalibrateButton == vtkKWPushButton::SafeDownCast(caller)
      && event == vtkKWPushButton::InvokedEvent)
    {
    vtkKWMatrixWidget *matrix =  this->SliceRangeMatrix->GetWidget();
    int s_index = matrix->GetElementValueAsInt(0, 0);
    int e_index = matrix->GetElementValueAsInt(0, 1);

    vtkMRMLScalarVolumeNode *volumeNode = vtkMRMLScalarVolumeNode::SafeDownCast(this->ZFrameImageSelectorWidget->GetSelected());

    // shift the index (on GUI, the index starts at 1)
    s_index --;
    e_index --;
    if (s_index < 1) s_index = 1;

    PerformZFrameCalibration(volumeNode, s_index, e_index);
    }
  
  // 12/24/2011 ayamada
  //----------------------------------------------------------------
  // Slice views.
  //
  // If the user clicked in one of the slice views with one of the
  // check buttons associated with the RAS coordinates of
  //  - the guidance needle tip
  //  - the center of marker A
  //  - the center of marker B
  //  - the center of marker C
  //  - the center of marker D
  // being active at the same time, the corresponding fiducial in
  // AbdoNav's fiducial list is being updated. If there exists no
  // corresponding fiducial in AbdoNav's fiducial list yet, a new
  // fiducial is added.
  //
  // The steps are:
  // 0. determine whether or not AbdoNav is the selected module;
  //    neither update nor create a fiducial if it isn't, thus
  //    exit this function
  // 1. if AbdoNav is the selected module, determine which check
  //    button is active (if there is an active one at all)
  // 2. if there is an active check button:
  //      2.0. create/retrieve AbdoNav's fiducial list
  //      2.1. transform XY mouse coordinates into RAS coordinates
  //      2.2. determine whether or not AbdoNav's fiducial list
  //           already contains a fiducial corresponding to the
  //           active check button
  //      2.3. if AbdoNav's fiducial list already contains a corres-
  //           ponding fiducial, then update it; otherwise, add a new
  //           fiducial
  //----------------------------------------------------------------  
  
  // 12/26/2011 ayamada
  
  vtkSlicerInteractorStyle* style = vtkSlicerInteractorStyle::SafeDownCast(caller);
  if (style != NULL && event == vtkCommand::LeftButtonPressEvent)
  {

    // exit this function if AbdoNav isn't the selected module (this->GetModuleName()
    // returns NULL, thus use this->GetGUIName())
    vtkMRMLLayoutNode* layout = vtkMRMLLayoutNode::SafeDownCast(this->MRMLScene->GetNthNodeByClass(0, "vtkMRMLLayoutNode"));
    if (layout && layout->GetSelectedModule() && strcmp(layout->GetSelectedModule(), this->GetGUI()->GetGUIName()) != 0)
    {
      // another module is currently selected, thus exit
      return;
    }    
    
    
  // 3/8/2012 ayamada
  // image select buttons
    if (this->imageConfigurationButton1->GetSelectedState() && this->imageConfigurationButton1->GetEnabled())
    {
    }
    else if (this->imageConfigurationButton2->GetSelectedState() && this->imageConfigurationButton2->GetEnabled())
    {
    }
    else if (this->imageConfigurationButton3->GetSelectedState() && this->imageConfigurationButton3->GetEnabled())
    {

    }
    
    
    
  // determine which check button is active
  std::string activeCheckButton = "";
  if (this->Point1CheckButton->GetSelectedState() && this->Point1CheckButton->GetEnabled())
  {
    // 12/27/2011 ayamada
    activeCheckButton = tip;
  }
  else if (this->Point2CheckButton->GetSelectedState() && this->Point2CheckButton->GetEnabled())
  {
    activeCheckButton = markerA;
  }
  else if (this->Point3CheckButton->GetSelectedState() && this->Point3CheckButton->GetEnabled())
  {
    activeCheckButton = markerB;
  }
  else if (this->Point4CheckButton->GetSelectedState() && this->Point4CheckButton->GetEnabled())
  {
    activeCheckButton = markerC;
  }
  else if (this->Point5CheckButton->GetSelectedState() && this->Point5CheckButton->GetEnabled())
  {
    activeCheckButton = markerD;
  }
  // 1/21/2012 ayamada
  else if (this->Point6CheckButton->GetSelectedState() && this->Point6CheckButton->GetEnabled())
  {
    activeCheckButton = markerE;
  }
    
    // if there is an active check button
    if (strcmp(activeCheckButton.c_str(), ""))
    {

      // create an AbdoNavNode if none exists yet
      // 12/25/2011 ayamada
      vtkMRMLAbdoNavNode* anode = this->CheckAndCreateAbdoNavNode();

      
      // create/retrieve AbdoNav's fiducial list
      vtkMRMLFiducialListNode* fiducialList;
      if (anode->GetRegistrationFiducialListID() == NULL)
      {

        // AbdoNav registration fiducial list doesn't exist yet, thus create it
        fiducialList = vtkMRMLFiducialListNode::New();
        fiducialList->SetName("AbdoNav-RegistrationFiducialList");
        fiducialList->SetDescription("Created by AbdoNav");
        // change default look ("StarBurst2D", 5) since it doesn't really
        // suit the purpose of needle identification; the user can always
        // return to the default look using Slicer's Fiducials module
        fiducialList->SetGlyphTypeFromString("Sphere3D");
        fiducialList->SetSymbolScale(2); // called "Glyph scale" in the Fiducials module
        this->MRMLScene->AddNode(fiducialList);
        fiducialList->Delete();
        // update MRML node
        anode->SetRegistrationFiducialListID(fiducialList->GetID());
        // observe fiducial list in order to update the GUI whenever a fiducial is moved via drag & drop or renamed or renumbered externally via the Fiducials module
        fiducialList->AddObserver(vtkMRMLFiducialListNode::FiducialModifiedEvent, (vtkCommand*)this->MRMLCallbackCommand);
        // observe fiducial list in order to update the GUI whenever a fiducial is added externally via the Fiducials module
        fiducialList->AddObserver(vtkMRMLScene::NodeAddedEvent, (vtkCommand*)this->MRMLCallbackCommand);
        // observe fiducial list in order to update the GUI whenever a fiducial is removed externally via the Fiducials module
        fiducialList->AddObserver(vtkMRMLScene::NodeRemovedEvent, (vtkCommand*)this->MRMLCallbackCommand);
        // observe fiducial list in order to update the GUI whenever all fiducials are removed externally via the Fiducials module
        fiducialList->AddObserver(vtkCommand::ModifiedEvent, (vtkCommand*)this->MRMLCallbackCommand);
        // no need to observe vtkMRMLFiducialListNode::DisplayModifiedEvent or vtkMRMLFiducialListNode::FiducialIndexModifiedEvent
      }
      else
      {

        // AbdoNav fiducial list already exists, thus retrieve it
        fiducialList = vtkMRMLFiducialListNode::SafeDownCast(this->MRMLScene->GetNodeByID(anode->GetRegistrationFiducialListID()));
      }
      
      // transform XY mouse coordinates into RAS coordinates
      //
      // first, find out in which slice view the user clicked
      // (necessary information for the XY to RAS conversion)
      vtkSlicerSliceGUI* sliceGUI = this->GetGUI()->GetApplicationGUI()->GetMainSliceGUI("Red");
      vtkRenderWindowInteractor* rwi = sliceGUI->GetSliceViewer()->GetRenderWidget()->GetRenderWindowInteractor();
      
      int index = 0;
      while (style != rwi->GetInteractorStyle() && index < 2)
      {
        index++;
        if (index == 1)
        {
          sliceGUI = this->GetGUI()->GetApplicationGUI()->GetMainSliceGUI("Yellow");
        }
        else
        {
          sliceGUI = this->GetGUI()->GetApplicationGUI()->GetMainSliceGUI("Green");
        }
        rwi = sliceGUI->GetSliceViewer()->GetRenderWidget()->GetRenderWindowInteractor();
      }
      
      // second, transform XY mouse coordinates into RAS coordinates
      int xyPos[2];
      rwi->GetLastEventPosition(xyPos);
      double xyVec[4] = {xyPos[0], xyPos[1], 0, 1};
      double rasVec[4];
      vtkMatrix4x4* matrix = sliceGUI->GetLogic()->GetSliceNode()->GetXYToRAS();
      matrix->MultiplyPoint(xyVec, rasVec);
      
      // determine whether or not AbdoNav's fiducial list
      // already contains a fiducial corresponding to the
      // active check button
      //
      // if so, update it; otherwise, add a new fiducial
      bool fiducialExists = false;
      for (int i = 0; i < fiducialList->GetNumberOfFiducials(); i++)
      {
        if (!strcmp(activeCheckButton.c_str(), fiducialList->GetNthFiducialLabelText(i)))
        {
          // corresponding fiducial already exists, thus update it
          // (fiducial list implementation will invoke proper event
          // and thereby trigger vtkAbdoNavGUI::UpdateGUIFromMRML())
          fiducialList->SetNthFiducialXYZ(i, rasVec[0], rasVec[1], rasVec[2]);
          fiducialExists = true;
        }
      }
      if (fiducialExists == false)
      {
        // corresponding fiducial doesn't exist yet, thus create it
        // (fiducial list implementation will invoke proper event
        // and thereby trigger vtkAbdoNavGUI::UpdateGUIFromMRML())
        fiducialList->AddFiducialWithLabelXYZSelectedVisibility(activeCheckButton.c_str(), rasVec[0], rasVec[1], rasVec[2], 1, 1);
      }
      
    }    
    
    
    
  } // the end of   if (style != NULL && event == vtkCommand::LeftButtonPressEvent)
    
  //----------------------------------------------------------------
  // Registration frame.
  //----------------------------------------------------------------
  else if (this->robotTransformSelector == vtkSlicerNodeSelectorWidget::SafeDownCast(caller) && event == vtkSlicerNodeSelectorWidget::NodeSelectedEvent)
  {
    //vtkMRMLLinearTransformNode* tnode = vtkMRMLLinearTransformNode::SafeDownCast(this->robotTransformSelector->GetSelected());
    
    // 12/25/2011 ayamada
    this->UpdateMRMLFromGUI();
  }
  // 3/8/2012 ayamada
  // image select button  
  else if (this->imageConfigurationButton1 == vtkKWCheckButton::SafeDownCast(caller) && event == vtkKWCheckButton::SelectedStateChangedEvent)
  {

    this->hideImages();

    if (this->imageConfigurationButton1->GetSelectedState())
    {
      this->imageConfigurationButton2->SelectedStateOff();
      this->imageConfigurationButton3->SelectedStateOff();
      
      this->calcPath("markers1.jpg");
      this->showImages(this->imageFilePath);
      
    }        
    
  }
  else if (this->imageConfigurationButton2 == vtkKWCheckButton::SafeDownCast(caller) && event == vtkKWCheckButton::SelectedStateChangedEvent)
  {

    this->hideImages();
    
    if (this->imageConfigurationButton2->GetSelectedState())
    {
      this->imageConfigurationButton1->SelectedStateOff();
      this->imageConfigurationButton3->SelectedStateOff();
      
      this->calcPath("markers2.jpg");
      this->showImages(this->imageFilePath);
      
    }        
    
  }
  else if (this->imageConfigurationButton3 == vtkKWCheckButton::SafeDownCast(caller) && event == vtkKWCheckButton::SelectedStateChangedEvent)
  {
    
    this->hideImages();

    if (this->imageConfigurationButton3->GetSelectedState())
    {
      this->imageConfigurationButton1->SelectedStateOff();
      this->imageConfigurationButton2->SelectedStateOff();
      
      this->calcPath("markers3.jpg");
      this->showImages(this->imageFilePath);
      
    }        
    
  }
    
    
    
  else if (this->Point1CheckButton == vtkKWCheckButton::SafeDownCast(caller) && event == vtkKWCheckButton::SelectedStateChangedEvent)
  {
    
    
    if (this->Point1CheckButton->GetSelectedState())
    {
      if (this->TimerLog == NULL)
      {
        // timer has not been started yet, thus start it now
        this->TimerLog = vtkTimerLog::New();
        this->TimerLog->StartTimer();
      }
      // mimic check button set behavior, i.e. only one check button allowed to be selected at a time
      this->Point2CheckButton->SelectedStateOff();
      this->Point3CheckButton->SelectedStateOff();
      this->Point4CheckButton->SelectedStateOff();
      this->Point5CheckButton->SelectedStateOff();
      // 1/21/2012 ayamada
      this->Point6CheckButton->SelectedStateOff();
    }
  }
  else if (this->Point2CheckButton == vtkKWCheckButton::SafeDownCast(caller) && event == vtkKWCheckButton::SelectedStateChangedEvent)
  {
    if (this->Point2CheckButton->GetSelectedState())
    {
      if (this->TimerLog == NULL)
      {
        // timer has not been started yet, thus start it now
        this->TimerLog = vtkTimerLog::New();
        this->TimerLog->StartTimer();
      }
      // mimic check button set behavior, i.e. only one check button allowed to be selected at a time
      this->Point1CheckButton->SelectedStateOff();
      this->Point3CheckButton->SelectedStateOff();
      this->Point4CheckButton->SelectedStateOff();
      this->Point5CheckButton->SelectedStateOff();
      // 1/21/2012 ayamada
      this->Point6CheckButton->SelectedStateOff();
    }
  }
  else if (this->Point3CheckButton == vtkKWCheckButton::SafeDownCast(caller) && event == vtkKWCheckButton::SelectedStateChangedEvent)
  {
    if (this->Point3CheckButton->GetSelectedState())
    {
      if (this->TimerLog == NULL)
      {
        // timer has not been started yet, thus start it now
        this->TimerLog = vtkTimerLog::New();
        this->TimerLog->StartTimer();
      }
      // mimic check button set behavior, i.e. only one check button allowed to be selected at a time
      this->Point1CheckButton->SelectedStateOff();
      this->Point2CheckButton->SelectedStateOff();
      this->Point4CheckButton->SelectedStateOff();
      this->Point5CheckButton->SelectedStateOff();
      // 1/21/2012 ayamada
      this->Point6CheckButton->SelectedStateOff();
    }
  }
  else if (this->Point4CheckButton == vtkKWCheckButton::SafeDownCast(caller) && event == vtkKWCheckButton::SelectedStateChangedEvent)
  {
    if (this->Point4CheckButton->GetSelectedState())
    {
      if (this->TimerLog == NULL)
      {
        // timer has not been started yet, thus start it now
        this->TimerLog = vtkTimerLog::New();
        this->TimerLog->StartTimer();
      }
      // mimic check button set behavior, i.e. only one check button allowed to be selected at a time
      this->Point1CheckButton->SelectedStateOff();
      this->Point2CheckButton->SelectedStateOff();
      this->Point3CheckButton->SelectedStateOff();
      this->Point5CheckButton->SelectedStateOff();
      // 1/21/2012 ayamada
      this->Point6CheckButton->SelectedStateOff();
    }
  }
  else if (this->Point5CheckButton == vtkKWCheckButton::SafeDownCast(caller) && event == vtkKWCheckButton::SelectedStateChangedEvent)
  {
    if (this->Point5CheckButton->GetSelectedState())
    {
      if (this->TimerLog == NULL)
      {
        // timer has not been started yet, thus start it now
        this->TimerLog = vtkTimerLog::New();
        this->TimerLog->StartTimer();
      }
      // mimic check button set behavior, i.e. only one check button allowed to be selected at a time
      this->Point1CheckButton->SelectedStateOff();
      this->Point2CheckButton->SelectedStateOff();
      this->Point3CheckButton->SelectedStateOff();
      this->Point4CheckButton->SelectedStateOff();
      // 1/21/2012 ayamada
      this->Point6CheckButton->SelectedStateOff();
    }
  }
  // 1/21/2012 ayamada
  else if (this->Point6CheckButton == vtkKWCheckButton::SafeDownCast(caller) && event == vtkKWCheckButton::SelectedStateChangedEvent)
  {
    if (this->Point6CheckButton->GetSelectedState())
    {
      if (this->TimerLog == NULL)
      {
        // timer has not been started yet, thus start it now
        this->TimerLog = vtkTimerLog::New();
        this->TimerLog->StartTimer();
      }
      // mimic check button set behavior, i.e. only one check button allowed to be selected at a time
      this->Point1CheckButton->SelectedStateOff();
      this->Point2CheckButton->SelectedStateOff();
      this->Point3CheckButton->SelectedStateOff();
      this->Point4CheckButton->SelectedStateOff();
      this->Point5CheckButton->SelectedStateOff();
    }
  }
  else if (this->ResetRegistrationPushButton == vtkKWPushButton::SafeDownCast(caller) && event == vtkKWPushButton::InvokedEvent)
  {
    this->robotTransformSelector->SetEnabled(true);
    this->Point1CheckButton->SetEnabled(true);
    this->Point2CheckButton->SetEnabled(true);
    this->Point3CheckButton->SetEnabled(true);
    this->Point4CheckButton->SetEnabled(true);
    this->Point5CheckButton->SetEnabled(true);
    // 1/21/2012 ayamada
    this->Point6CheckButton->SetEnabled(true);
    
    this->PerformRegistrationPushButton->SetEnabled(true);
    this->ResetRegistrationPushButton->SetEnabled(false);
    vtkMRMLFiducialListNode* fnode = vtkMRMLFiducialListNode::SafeDownCast(this->MRMLScene->GetNodeByID(this->AbdoNavNode->GetRegistrationFiducialListID()));
    if (fnode)
    {
      fnode->SetLocked(0);
    }
  }
  else if (this->PerformRegistrationPushButton == vtkKWPushButton::SafeDownCast(caller) && event == vtkKWPushButton::InvokedEvent)
  {
    
    // 12/28/2011 ayamada
    this->translatePointData();
    
    
    // verify registration input parameters
    // 12/25/2011 ayamada
    this->CheckAndCreateAbdoNavNode();
    
      if (this->AbdoNavNode->GetRegistrationFiducialListID() == NULL || (this->AbdoNavNode->GetRegistrationFiducialListID() != NULL &&
                                                                            vtkMRMLFiducialListNode::SafeDownCast(this->MRMLScene->GetNodeByID(this->AbdoNavNode->GetRegistrationFiducialListID()))->GetNumberOfFiducials() < 3))
    {
      vtkKWMessageDialog::PopupMessage(this->GetApplication(),
                                       this->GetGUI()->GetApplicationGUI()->GetMainSlicerWindow(),
                                       "AbdoNav",
                                       "Need to identify at least three fiducials in image space!",
                                       vtkKWMessageDialog::ErrorIcon);
    }
    else
    {
      
        if (this->GetLogic()->PerformRegistration() == EXIT_SUCCESS)
        {

          if (this->TimerLog != NULL)
          {
            std::cout.setf(ios::scientific, ios::floatfield);
            std::cout.precision(8);
            std::cout.unsetf(ios::floatfield);
            std::cout.precision(6);
            this->TimerLog = NULL;
          }
          
          this->robotTransformSelector->SetEnabled(false);
          this->Point1CheckButton->SetEnabled(false);
          this->Point1CheckButton->SetSelectedState(false);
          this->Point2CheckButton->SetEnabled(false);
          this->Point2CheckButton->SetSelectedState(false);
          this->Point3CheckButton->SetEnabled(false);
          this->Point3CheckButton->SetSelectedState(false);
          this->Point4CheckButton->SetEnabled(false);
          this->Point4CheckButton->SetSelectedState(false);
          this->Point5CheckButton->SetEnabled(false);
          this->Point5CheckButton->SetSelectedState(false);
          this->Point6CheckButton->SetEnabled(false);
          this->Point6CheckButton->SetSelectedState(false);
          this->PerformRegistrationPushButton->SetEnabled(false);
          this->ResetRegistrationPushButton->SetEnabled(true);
          // lock fiducial list
          vtkMRMLFiducialListNode* fnode = vtkMRMLFiducialListNode::SafeDownCast(this->MRMLScene->GetNodeByID(this->AbdoNavNode->GetRegistrationFiducialListID()));
          
          if (fnode)
          {
            fnode->SetLocked(1);
          }
          this->GetLogic()->ObserveTrackingTransformNode();
        }
        else
        {
          vtkKWMessageDialog::PopupMessage(this->GetApplication(),
                                           this->GetGUI()->GetApplicationGUI()->GetMainSlicerWindow(),
                                           "AbdoNav",
                                           "Registration failed, check input parameters!",
                                           vtkKWMessageDialog::ErrorIcon);
        }
    }
  }
    
  } // end of this->GetLogic()->workPhaseStatus = 1 loop  

  
  // process GUI event
  // 9/8/2011 ayamada: read CSV file and list the data
  if (this->AddFiducialFilePushButton == vtkKWPushButton::SafeDownCast(caller) 
      && event == vtkKWPushButton::InvokedEvent)
  {
    
    const char * path = this->SelectFiducialsFileButton->GetWidget()->GetFileName();
    if (path)
    {
      // 12/28/2011 ayamada
      int numberOfData = this->GetGUI()->LoadCSVTrackingFile(path);      
  
      if (!numberOfData)
      {
      }else{

        char str[32];
        char str2[128];
        const char *pc = NULL;
        const char *sc = NULL;

        for(int i = 0; i<numberOfData; i++)
        {

          sprintf(str, "%d", this->FiducialNumber);
          sprintf(str2, "%.2f %.2f %.2f", this->GetGUI()->pX[i],this->GetGUI()->pY[i],this->GetGUI()->pZ[i]);
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
    

    if ( scSize < 5)
    {
      vtkSlicerApplication::GetInstance()->ErrorMessage("Robot coordinates are invalid."); 
    }
    else 
    {
      int row = this->PointPairMultiColumnList->GetWidget()->GetNumberOfRows();
      this->PointPairMultiColumnList->GetWidget()->AddRow();
      this->PointPairMultiColumnList->GetWidget()->SetCellText(row, 0, pc);
      this->PointPairMultiColumnList->GetWidget()->SetCellText(row, 1, sc);
      // increment of firucial marker numbers
      this->FiducialNumber++;
      this->FiducialNumber2++;
    }
    
  }  
  
  
  // 8/4/2011 ayamada
  // delete fiducial maraker list 
  if (this->DeletePointPairPushButton == vtkKWPushButton::SafeDownCast(caller) 
      && event == vtkKWPushButton::InvokedEvent)
  {
    int numOfRows = this->PointPairMultiColumnList->GetWidget()->GetNumberOfSelectedRows();
    if (numOfRows == 1)
    {
      
      int index[3];
      this->PointPairMultiColumnList->GetWidget()->GetSelectedRows(index);
      this->PointPairMultiColumnList->GetWidget()->DeleteRow(index[0]);
      
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
    
    // reset number
    this->FiducialNumber = 1;
    
  }
  
  // 12/28/2011 ayamada
  // show robot button
  if (this->ShowRobotButton && this->ShowRobotButton == vtkKWCheckButton::SafeDownCast(caller) && (event == vtkKWCheckButton::SelectedStateChangedEvent))
  {
    this->ShowRobotModel(this->ShowRobotButton->GetSelectedState() == 1);
  }
  
  
  // 1/22/2012 ayamada-b
  // load targeting volume dialog button
  if (this->LoadTargetingVolumeButton && this->LoadTargetingVolumeButton == vtkKWPushButton::SafeDownCast(caller) && (event == vtkKWPushButton::InvokedEvent))
  {
    this->GetApplication()->Script("::LoadVolume::ShowDialog");
  }
  
  if (this->VolumeSelectorWidget == vtkSlicerNodeSelectorWidget::SafeDownCast(caller) &&
      event == vtkSlicerNodeSelectorWidget::NodeSelectedEvent ) 
  {
    vtkMRMLScalarVolumeNode *volume = vtkMRMLScalarVolumeNode::SafeDownCast(this->VolumeSelectorWidget->GetSelected());
    if (volume != NULL)
    {
      this->GetGUI()->GetLogic()->SelectVolumeInScene(volume, VOL_TARGETING);
    }
  }
  
  
  
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::ShowZFrameModel(bool show)
{

  vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(this->GetRobotProbeNavManager()->GetRobotNode()->GetCalibrationObjectModelId()));
  vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
  displayNode->SetVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::PerformZFrameCalibration(const char* filename)
{
  std::cerr << "Loading " << filename << std::endl;

  vtkSlicerModuleGUI *m = vtkSlicerApplication::SafeDownCast(this->GetApplication())->GetModuleGUIByName("Volumes"); 
  if ( m != NULL ) 
  {
    vtkSlicerVolumesLogic* volume_logic = 
      vtkSlicerVolumesGUI::SafeDownCast(m)->GetLogic();
    volume_logic->AddArchetypeVolume(filename, "ZFrameImage", 0x0004);

    vtkMRMLScalarVolumeNode* volumeNode = NULL;   // Event Source MRML node 
    vtkCollection* collection = this->MRMLScene->GetNodesByName("ZFrameImage");
    int nItems = collection->GetNumberOfItems();
    for (int i = 0; i < nItems; i ++)
    {
      vtkMRMLNode* node = vtkMRMLNode::SafeDownCast(collection->GetItemAsObject(i));
      if (strcmp(node->GetNodeTagName(), "Volume") == 0)
      {
        volumeNode = vtkMRMLScalarVolumeNode::SafeDownCast(node);
        break;
      }
    }
    if (volumeNode)
    {
      this->GetRobotProbeNavManager()->GetRobotNode()->PerformRegistration(volumeNode);
    }
  }
  else
  {
    std::cerr << "Couldn't find ZFrame image in the MRML scene." << std::endl;
  }
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::PerformZFrameCalibration(vtkMRMLScalarVolumeNode* node, int s_index, int e_index)
{
  if (node)
    {
    this->GetRobotProbeNavManager()->GetRobotNode()->PerformRegistration(node, s_index, e_index);
    }

}


//-----------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::AddGUIObservers()
{
  this->RemoveGUIObservers();

  if (this->CalibrateButton)
    {
    this->CalibrateButton->AddObserver(vtkKWPushButton::InvokedEvent,(vtkCommand *)this->GUICallbackCommand);
    }
  if (this->ShowZFrameCheckButton)
    {
    this->ShowZFrameCheckButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand *)this->GUICallbackCommand);
    }
  if (this->ShowWorkspaceCheckButton)
    {
    this->ShowWorkspaceCheckButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand *)this->GUICallbackCommand);
    }
  
  // 12/24/2011 ayamada
  //----------------------------------------------------------------
  // Set observers on slice views.
  //----------------------------------------------------------------
  vtkSlicerApplicationGUI* appGUI = this->GetGUI()->GetApplicationGUI();
  if (appGUI) 
  {  
  appGUI->GetMainSliceGUI("Red")
  ->GetSliceViewer()->GetRenderWidget()->GetRenderWindowInteractor()->GetInteractorStyle()
  ->AddObserver(vtkCommand::LeftButtonPressEvent, (vtkCommand*)this->GUICallbackCommand);
  appGUI->GetMainSliceGUI("Yellow")
  ->GetSliceViewer()->GetRenderWidget()->GetRenderWindowInteractor()->GetInteractorStyle()
  ->AddObserver(vtkCommand::LeftButtonPressEvent, (vtkCommand*)this->GUICallbackCommand);
  appGUI->GetMainSliceGUI("Green")
  ->GetSliceViewer()->GetRenderWidget()->GetRenderWindowInteractor()->GetInteractorStyle()
  ->AddObserver(vtkCommand::LeftButtonPressEvent, (vtkCommand*)this->GUICallbackCommand);
  }
  //----------------------------------------------------------------
  // Registration frame.
  //----------------------------------------------------------------
  this->robotTransformSelector->AddObserver(vtkSlicerNodeSelectorWidget::NodeSelectedEvent, (vtkCommand*)this->GUICallbackCommand);
  // 3/8/2012 ayamada
  this->imageConfigurationButton1->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand*)this->GUICallbackCommand);
  this->imageConfigurationButton2->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand*)this->GUICallbackCommand);
  this->imageConfigurationButton3->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand*)this->GUICallbackCommand);
  
  this->Point1CheckButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand*)this->GUICallbackCommand);
  this->Point2CheckButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand*)this->GUICallbackCommand);
  this->Point3CheckButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand*)this->GUICallbackCommand);
  this->Point4CheckButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand*)this->GUICallbackCommand);
  this->Point5CheckButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand*)this->GUICallbackCommand);
  // 1/21/2012 ayamada
  this->Point6CheckButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand*)this->GUICallbackCommand);
  this->ResetRegistrationPushButton->AddObserver(vtkKWPushButton::InvokedEvent, (vtkCommand*)this->GUICallbackCommand);
  this->PerformRegistrationPushButton->AddObserver(vtkKWPushButton::InvokedEvent, (vtkCommand*)this->GUICallbackCommand);
  
  // 12/28/2011 ayamada
  if (this->ShowRobotButton)
  {
    this->ShowRobotButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand *)this->GUICallbackCommand);
  }  
  

  // 1/21/2012 ayamada-b
  if (this->LoadTargetingVolumeButton)
  {
    this->LoadTargetingVolumeButton->AddObserver(vtkKWPushButton::InvokedEvent, (vtkCommand *)this->GUICallbackCommand); 
  }
  
  if (this->VolumeSelectorWidget)
  {
    this->VolumeSelectorWidget->AddObserver ( vtkSlicerNodeSelectorWidget::NodeSelectedEvent, (vtkCommand *)this->GUICallbackCommand);  
  }
  
  
}


//---------------------------------------------------------------------------
// 12/25/2011 ayamada
void vtkRobotProbeNavCalibrationStep::AddLogicObservers()
{
  this->RemoveLogicObservers();
  
  if (this->GetLogic())
  {
  }
}

//---------------------------------------------------------------------------
// 12/25/2011 ayamada
void vtkRobotProbeNavCalibrationStep::RemoveLogicObservers()
{
  if (this->GetLogic())
  {
  }
}

//-----------------------------------------------------------------------------
void vtkRobotProbeNavCalibrationStep::RemoveGUIObservers()
{
  if (this->CalibrateButton)
    {
    this->CalibrateButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }
  if (this->ShowZFrameCheckButton)
    {
    this->ShowZFrameCheckButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }
  if (this->ShowWorkspaceCheckButton)
    {
    this->ShowWorkspaceCheckButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }
  
  // 3/8/2012 ayamada
  if (this->imageConfigurationButton1)
  {
    this->imageConfigurationButton1->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  if (this->imageConfigurationButton2)
  {
    this->imageConfigurationButton2->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  if (this->imageConfigurationButton3)
  {
    this->imageConfigurationButton3->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  
  
  // 12/24/2011 ayamada
  if (this->robotTransformSelector)
  {
    this->robotTransformSelector->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  if (this->Point1CheckButton)
  {
    this->Point1CheckButton->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  if (this->Point2CheckButton)
  {
    this->Point2CheckButton->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  if (this->Point3CheckButton)
  {
    this->Point3CheckButton->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  if (this->Point4CheckButton)
  {
    this->Point4CheckButton->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  if (this->Point5CheckButton)
  {
    this->Point5CheckButton->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  // 1/21/2012 ayamada
  if (this->Point6CheckButton)
  {
    this->Point6CheckButton->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  if (this->ResetRegistrationPushButton)
  {
    this->ResetRegistrationPushButton->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  if (this->PerformRegistrationPushButton)
  {
    this->PerformRegistrationPushButton->RemoveObserver((vtkCommand*)this->GUICallbackCommand);
  }
  
  // 12/28/2011 ayamada
  if (this->ShowRobotButton)
  {
    this->ShowRobotButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
  }  

  
  // 1/21/2012 ayamada-b
  if (this->LoadTargetingVolumeButton)
  {
    this->LoadTargetingVolumeButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand); 
  }
  
  if (this->VolumeSelectorWidget)
  {
    this->VolumeSelectorWidget->RemoveObserver ((vtkCommand *)this->GUICallbackCommand);  
  }
  
  
}



// 12/27/2011 ayamada
void vtkRobotProbeNavCalibrationStep::translatePointData()
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
            
    }
    
    vtkMRMLScene* scene = this->GetLogic()->GetMRMLScene();      
    vtkMRMLNode* node = scene->GetNodeByID("vtkMRMLLinearTransformNode6");
    vtkMRMLLinearTransformNode* tnode = vtkMRMLLinearTransformNode::SafeDownCast(node);
    
  }

}

// 3/8/2012 ayamada
// this code is based on ViewerBackgroundOn function by J.Tokuda (BWH)
void vtkRobotProbeNavCalibrationStep::showImages(const char* filename)
{

  vtkSlicerViewerWidget* vwidget = this->GetGUI()->GetApplicationGUI()->GetNthViewerWidget(0);
  vtkKWRenderWidget* rwidget;
  vtkRenderWindow* rwindow;
    
  // 3/5/2012 ayamada
  vtkJPEGReader* jpgreader = vtkJPEGReader::New();
  if(jpgreader->CanReadFile(filename))
  {
    jpgreader->SetFileName(filename);
  }
  
  if (vwidget&&
      (rwidget = vwidget->GetMainViewer()) &&
      (rwindow = rwidget->GetRenderWindow()))
  {
    if (rwidget->GetNumberOfRenderers() == 1)
    {
      this->BackgroundRenderer = vtkRenderer::New();
      this->BackgroundActor = vtkImageActor::New();
      this->BackgroundActor->SetInput(jpgreader->GetOutput());
      this->BackgroundRenderer->AddActor(this->BackgroundActor);
      this->BackgroundRenderer->InteractiveOff();
      this->BackgroundRenderer->SetLayer(0);
      
      // Adjust camera position so that image covers the draw area.
      
      this->BackgroundActor->Modified();
      rwidget->GetNthRenderer(0)->SetLayer(1);
      rwidget->AddRenderer(this->BackgroundRenderer);
      rwindow->Render();
      
      vtkCamera* camera = this->BackgroundRenderer->GetActiveCamera();
      double x, y, z;
      camera->GetPosition(x, y, z);
      camera->SetViewAngle(90.0);
      camera->SetPosition(x, y, y); 
      
      // The following code fixes a issue that
      // video doesn't show up on the viewer.
      vtkCamera* fcamera = rwidget->GetNthRenderer(0)->GetActiveCamera();
      if (fcamera)
      {
        fcamera->Modified();
      }
      
    }
  }
  
}

// this code is based on ViewerBackgroundOff function by J.Tokuda (BWH)
void vtkRobotProbeNavCalibrationStep::hideImages()
{

  vtkSlicerViewerWidget* vwidget = this->GetGUI()->GetApplicationGUI()->GetNthViewerWidget(0);
  vtkKWRenderWidget* rwidget;
  vtkRenderWindow* rwindow;
  
  if (vwidget&&
      (rwidget = vwidget->GetMainViewer()) &&
      (rwindow = rwidget->GetRenderWindow()))
  {
    if (rwidget->GetNumberOfRenderers() > 1)
    {
      rwidget->RemoveNthRenderer(1);
      rwidget->GetNthRenderer(0)->SetLayer(0);
      rwindow->Render();
      this->BackgroundRenderer = NULL;
      this->BackgroundActor = NULL;
    }
  }
  
  
}

// 3/9/2012 ayamada
void vtkRobotProbeNavCalibrationStep::calcPath(const char* path)
{

  vtksys_stl::string filename="";
  filename = this->GetLogic()->GetModuleShareDirectory();
    
  //filepath
  const char* filePathSource = filename.c_str();
  int cnt = 0;
  int totalCnt = 0;
  
  // count number of char
  for(int i = 0; filePathSource[i] != NULL; i++)
  {
    cnt++; 
  }
  
  totalCnt = cnt;
  
  // cut the path
  int cntOfLayer = 0;
  int ModuleNameNumber = 0;
  
  while(cntOfLayer < 4)
  {
    cnt--;
    if(filePathSource[cnt] == '/')
    {
      if(cntOfLayer == 0)
      {
        ModuleNameNumber = cnt;
      }
      cntOfLayer++;
    }
    
  }
    
  // create right filepath
  char* moduleName;
  moduleName = (char*)calloc(500, sizeof(char));
  
  int eralse_build_number = 7;
  
  for(int i = 0; i <= cnt-eralse_build_number; i++)
  {
    this->imageFilePath[i] = filePathSource[i]; 
  }
  
  for(int i = ModuleNameNumber; i <= totalCnt; i++)
  {
    moduleName[i-ModuleNameNumber] = filePathSource[i+1]; 
  }
    
  const char* additionaPath = "/Modules/";
  const char* materialPath = "/MITRobot/";
  
  int cnt2 = 0;  
  for(int i = 0; additionaPath[i] != NULL; i++)
  {
    cnt2++; 
  }
  
  for(int i = 1; i<=cnt2; i++)
  {
    this->imageFilePath[cnt-eralse_build_number+i] = additionaPath[i-1]; 
  }
  for(int i = 1; i<=totalCnt-ModuleNameNumber; i++)
  {
    this->imageFilePath[cnt-eralse_build_number+cnt2+i] = moduleName[i-1]; 
  }
  int c = cnt-eralse_build_number+cnt2+totalCnt-ModuleNameNumber;
  
  int cnt3 = 0;  
  for(int i = 0; materialPath[i] != NULL; i++)
  {
    cnt3++; 
  }
  
  for(int i = 1; i<=cnt3; i++)
  {
    this->imageFilePath[i+c-1] = materialPath[i-1]; 
  }
  
  int cnt4 = 0;  
  for(int i = 0; path[i] != NULL; i++)
  {
    cnt4++; 
  }
  
  for(int i = 1; i <= cnt4; i++)
  {
    this->imageFilePath[i+c-1+cnt3] = path[i-1];     
  }
    
  std::cerr << "correct file path = " << this->imageFilePath << std::endl;
  
}