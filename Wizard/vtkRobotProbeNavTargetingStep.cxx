
/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt   
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $ 
  Version:   $Revision: $ 

==========================================================================*/

#include "vtkRobotProbeNavTargetingStep.h"

#include "vtkRobotProbeNavGUI.h"
#include "vtkRobotProbeNavLogic.h"

#include "vtkKWMatrixWidget.h"
#include "vtkKWMatrixWidgetWithLabel.h"
#include "vtkSlicerNodeSelectorWidget.h"

#include "vtkSlicerFiducialsGUI.h"
#include "vtkSlicerFiducialsLogic.h"
#include "vtkMRMLSelectionNode.h"

#include "vtkMRMLLinearTransformNode.h"
#include "igtlMath.h"

#include "vtkMRMLRobotNode.h"

////

#include "vtkRobotProbeNavTargetDescriptor.h"
#include "vtkSlicerApplication.h"
#include "vtkSlicerApplicationLogic.h"
#include "vtkMRMLFiducialListNode.h"
#include "vtkSlicerSliceLogic.h"
#include "vtkMRMLSliceNode.h"
#include "vtkSlicerVolumesGUI.h"
#include "vtkMRMLInteractionNode.h"

#include "vtkKWFrame.h"
#include "vtkKWWizardWidget.h"
#include "vtkKWWizardWorkflow.h"
#include "vtkKWFrame.h"
#include "vtkKWEntry.h"
#include "vtkKWLabel.h"
#include "vtkKWEntryWithLabel.h"
#include "vtkKWEntrySet.h"
#include "vtkKWMessageDialog.h"
#include "vtkKWText.h"
#include "vtkKWPushButton.h"
#include "vtkKWMenuButton.h"
#include "vtkKWMenuButtonWithLabel.h"
#include "vtkKWMultiColumnList.h"
#include "vtkKWMultiColumnListWithScrollbars.h"
#include "vtkKWCheckButton.h"

// 1/22/2012 ayamada
#include "vtkMRMLIGTProbeRobotNode.h"

// 1/26/2012 ayamada
#include "vtkKWTkUtilities.h"

static const char TARGET_INDEX_ATTR[]="TARGET_IND";

// 7/20/2012 ayamada
#define PI4matrix 3.141592654

#define DELETE_IF_NULL_WITH_SETPARENT_NULL(obj) \
  if (obj) \
    { \
    obj->SetParent(NULL); \
    obj->Delete(); \
    obj = NULL; \
    };

// Definition of target list columns
enum
{
  NEEDLE_KIND = 0, // 10/28/2011 ayamada
  STATUS,
  ORDER,
  DIS2TARGET, // 10/28/2011 ayamada
  DIS2SKINY, // 10/28/2011 ayamada
  COL_NAME,// = 3, // 10/28/2011 ayamada 1, // 9/25/2011 ayamada //0,
  COL_X,
  COL_Y,
  COL_Z,
  COL_NEEDLE,
  COL_OR_W,
  COL_OR_X,
  COL_OR_Y,
  COL_OR_Z,
  COL_COUNT // all valid columns should be inserted above this line
};
//static const char* COL_LABELS[COL_COUNT] = { "Name", "R", "A", "S", "Needle", "OrW", "OrX", "OrY", "OrZ" };
//static const int COL_WIDTHS[COL_COUNT] = { 8, 6, 6, 6, 10, 6, 6, 6, 6 };

// 10/28/2011 ayamada
//static const char* COL_LABELS[COL_COUNT] = { "Holder Position", "Order", "Dist. to Target", "Dist. Skin to Target", "Name", "R", "A", "S", "Needle", "OrW", "OrX", "OrY", "OrZ" };
//static const int COL_WIDTHS[COL_COUNT] = {8, 8, 10, 10, 8, 6, 6, 6, 10, 6, 6, 6, 6 };

// 1/28/2012 ayamada
static const char* COL_LABELS[COL_COUNT] = { "Holder Position", "Status", "Order", "Dist. to Target", "Dist. Skin to Target", "Name", "R", "A", "S", "Needle", "OrW", "OrX", "OrY", "OrZ" };
static const int COL_WIDTHS[COL_COUNT] = {8, 8, 8, 10, 10, 8, 6, 6, 6, 10, 6, 6, 6, 6 };

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkRobotProbeNavTargetingStep);
vtkCxxRevisionMacro(vtkRobotProbeNavTargetingStep, "$Revision: 1.1 $");

//----------------------------------------------------------------------------
vtkRobotProbeNavTargetingStep::vtkRobotProbeNavTargetingStep()
{
  //this->SetName("Targeting");
  this->SetTitle("Targeting");
  this->SetDescription("Set target points.");

  this->MainFrame=NULL;
  
  // TargetPlanning frame
  this->TargetPlanningFrame=NULL;
  this->LoadTargetingVolumeButton=NULL;
  this->ShowWorkspaceButton=NULL;
  this->ShowRobotButton=NULL;
  this->VolumeSelectorWidget=NULL;
  this->TargetPlanningFrame=NULL;  
  this->AddTargetsOnClickButton=NULL;
  this->NeedleTypeMenuList=NULL; 

  // 9/25/2011 ayamada
  this->ShowAllNeedlePathsButton=NULL;
  
  
  // TargetList frame
  this->TargetListFrame=NULL;
  this->TargetList=NULL;
  this->DeleteButton=NULL;

  // TargetControl frame
  this->TargetControlFrame=NULL;
  this->NeedlePositionMatrix=NULL;
  this->NeedleOrientationMatrix=NULL;
  this->MoveButton=NULL;
  this->StopButton=NULL;

  this->Message=NULL;
  this->Message2=NULL;

  this->TitleBackgroundColor[0] = 0.8;
  this->TitleBackgroundColor[1] = 0.8;
  this->TitleBackgroundColor[2] = 0.8;

  this->ProcessingCallback = false;

  this->TargetPlanListNode=NULL;

  this->ShowTargetOrientation = 0;
  
  // 12/8/2011 ayamada
  this->needlePathAdjuster = NULL;
  this->needlePathAdjuster2 = NULL;  
  this->needlePathAdjusterValue = 0.0;//2;
  this->needlePathAdjusterValue2 = 0.0;//2;
  
  // 12/26/2011 ayamada
  //this->GetLogic()->workPhaseStatus = 2;
  
  this->TimerFlag = 0;
  this->TimerInterval = 0;
  
  // 1/26/2012 ayamada
  this->robotStatusSwitcher = 0;
  this->commandStatus = 0;
  this->robotStatus = 0;
  
  this->previousPositionX = 0.0;
  this->previousPositionY = 0.0;
  this->previousPositionZ = 0.0;
  this->stopSignal = 0.0;
  
  // 7/5/2012 ayamada
  this->carriagePosition[0] = -100.0;
  this->carriagePosition[1] = -110.0;
  this->carriagePosition[2] = -120.0;
  
  this->orderOfNeedlePlacement[0] = 0;
  this->orderOfNeedlePlacement[1] = 0;
  this->orderOfNeedlePlacement[2] = 0;
  
  // 8/19/2012 ayamada
  this->RobotControllerStatus = 0;
  
  
  
  
}

//----------------------------------------------------------------------------
vtkRobotProbeNavTargetingStep::~vtkRobotProbeNavTargetingStep()
{
  RemoveGUIObservers();

  DELETE_IF_NULL_WITH_SETPARENT_NULL(MainFrame);
  
  // TargetPlanning
  DELETE_IF_NULL_WITH_SETPARENT_NULL(TargetPlanningFrame);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(LoadTargetingVolumeButton);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(VolumeSelectorWidget);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(TargetPlanningFrame);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(ShowWorkspaceButton);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(ShowRobotButton);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(AddTargetsOnClickButton);

  // 9/25/2011 ayamada
  DELETE_IF_NULL_WITH_SETPARENT_NULL(ShowAllNeedlePathsButton);
  
  
  DELETE_IF_NULL_WITH_SETPARENT_NULL(NeedleTypeMenuList); 

  // TargetList frame
  DELETE_IF_NULL_WITH_SETPARENT_NULL(TargetListFrame);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(TargetList);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(DeleteButton);

  // 12/8/2011 ayamada
  DELETE_IF_NULL_WITH_SETPARENT_NULL(needlePathAdjuster);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(needlePathAdjuster2);
  
  
  // TargetControl frame
  DELETE_IF_NULL_WITH_SETPARENT_NULL(TargetControlFrame);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(NeedlePositionMatrix);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(NeedleOrientationMatrix);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(MoveButton);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(StopButton);

  DELETE_IF_NULL_WITH_SETPARENT_NULL(Message);
  DELETE_IF_NULL_WITH_SETPARENT_NULL(Message2);
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::ShowUserInterface()
{
  
  // 12/26/2011 ayamada
  if(this->GetLogic())
  {
  this->GetLogic()->workPhaseStatus = 2;    
  //std::cerr << "workPhaseStatus = 2!!!!!!!!!!!!!!"  << std::endl;
  }
  
  this->Superclass::ShowUserInterface();

  this->ShowTargetPlanningFrame();
  this->ShowTargetListFrame();
  this->ShowTargetControlFrame();

  AddMRMLObservers();
  
  this->AddGUIObservers();

    
  EnableAddTargetsOnClickButton(this->AddTargetsOnClickButton->GetSelectedState()==1);

  // 11/18/2011 ayamada
  // 9/25/2011 ayamada
  //EnableShowAllNeedlePathsButton(this->ShowAllNeedlePathsButton->GetSelectedState()==1);

  // 11/18/2011 ayamada
  //EnableShowRobotButton(this->ShowRobotButton->GetSelectedState()==1);
  
  UpdateGUI();
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::ShowTargetPlanningFrame()
{
  vtkKWWidget *parent = this->GetGUI()->GetWizardWidget()->GetClientArea();

  if (!this->TargetPlanningFrame)
    {
    this->TargetPlanningFrame = vtkKWFrame::New();
    }
  if (!this->TargetPlanningFrame->IsCreated())
    {
    this->TargetPlanningFrame->SetParent(parent);
    this->TargetPlanningFrame->Create();
    }

  this->Script("pack %s -side top -anchor nw -padx 0 -pady 2",
               this->TargetPlanningFrame->GetWidgetName());
  
  if (!this->LoadTargetingVolumeButton)
    {
     this->LoadTargetingVolumeButton = vtkKWPushButton::New();
    }
  if (!this->LoadTargetingVolumeButton->IsCreated())
    {
    this->LoadTargetingVolumeButton->SetParent(this->TargetPlanningFrame);
    this->LoadTargetingVolumeButton->Create();
    this->LoadTargetingVolumeButton->SetBorderWidth(2);
    this->LoadTargetingVolumeButton->SetReliefToRaised();       
    this->LoadTargetingVolumeButton->SetHighlightThickness(2);
    this->LoadTargetingVolumeButton->SetBackgroundColor(0.85,0.85,0.85);
    this->LoadTargetingVolumeButton->SetActiveBackgroundColor(1,1,1);        
    this->LoadTargetingVolumeButton->SetText( "Load volume");
    this->LoadTargetingVolumeButton->SetBalloonHelpString("Click to load a volume. Need to additionally select the volume to make it the current targeting volume.");
    }

  if (!this->VolumeSelectorWidget)
    {
     this->VolumeSelectorWidget = vtkSlicerNodeSelectorWidget::New();
    }
  if (!this->VolumeSelectorWidget->IsCreated())
    {
    this->VolumeSelectorWidget->SetParent(this->TargetPlanningFrame);
    this->VolumeSelectorWidget->Create();
    this->VolumeSelectorWidget->SetBorderWidth(2);  
    this->VolumeSelectorWidget->SetNodeClass("vtkMRMLVolumeNode", NULL, NULL, NULL);
    this->VolumeSelectorWidget->SetMRMLScene(this->GetLogic()->GetApplicationLogic()->GetMRMLScene());
    this->VolumeSelectorWidget->SetNoneEnabled(true);
    this->VolumeSelectorWidget->GetWidget()->GetWidget()->IndicatorVisibilityOff();
    this->VolumeSelectorWidget->GetWidget()->GetWidget()->SetWidth(24);
    this->VolumeSelectorWidget->SetLabelText( "Targeting Volume: ");
    this->VolumeSelectorWidget->SetBalloonHelpString("Select the targeting volume from the current scene.");
    }

  if (!this->ShowRobotButton)
  {
    this->ShowRobotButton = vtkKWCheckButton::New();
  } 
  if (!this->ShowRobotButton->IsCreated()) {
    this->ShowRobotButton->SetParent(this->TargetPlanningFrame);
    this->ShowRobotButton->Create();
    this->ShowRobotButton->SelectedStateOff();
    this->ShowRobotButton->SetText("Show Robot");
    this->ShowRobotButton->SetBalloonHelpString("Show the robot");
  }

  if (!this->ShowWorkspaceButton)
  {
    this->ShowWorkspaceButton = vtkKWCheckButton::New();
  } 
  // 1/17/2011 ayamada
  /*
  if (!this->ShowWorkspaceButton->IsCreated()) {
    this->ShowWorkspaceButton->SetParent(this->TargetPlanningFrame);
    this->ShowWorkspaceButton->Create();
    this->ShowWorkspaceButton->SelectedStateOff();
    this->ShowWorkspaceButton->SetText("Show Workspace");
    this->ShowWorkspaceButton->SetBalloonHelpString("Show workspace of the robot");
  }  
  */
  if (!this->AddTargetsOnClickButton)
  {
    this->AddTargetsOnClickButton = vtkKWCheckButton::New();
  } 
  if (!this->AddTargetsOnClickButton->IsCreated()) {
    this->AddTargetsOnClickButton->SetParent(this->TargetPlanningFrame);
    this->AddTargetsOnClickButton->Create();
    this->AddTargetsOnClickButton->SelectedStateOff();
    this->AddTargetsOnClickButton->SetText("Add target by image clicking");
    this->AddTargetsOnClickButton->SetBalloonHelpString("Add a target if image is clicked, with the current needle");
  }

  // 9/25/2011 ayamada
  if (!this->ShowAllNeedlePathsButton)
  {
    this->ShowAllNeedlePathsButton = vtkKWCheckButton::New();
  } 
  if (!this->ShowAllNeedlePathsButton->IsCreated()) {
    this->ShowAllNeedlePathsButton->SetParent(this->TargetPlanningFrame);
    this->ShowAllNeedlePathsButton->Create();
    this->ShowAllNeedlePathsButton->SelectedStateOff();
    this->ShowAllNeedlePathsButton->SetText("Show all planned needle paths");
    this->ShowAllNeedlePathsButton->SetBalloonHelpString("Show all planned needle paths");
  }
  

  // add combo box in the frame
  if (!this->NeedleTypeMenuList)
    {
    this->NeedleTypeMenuList = vtkKWMenuButtonWithLabel::New();
    }
  if (!this->NeedleTypeMenuList->IsCreated())
    {
    this->NeedleTypeMenuList->SetParent(this->TargetPlanningFrame);
    this->NeedleTypeMenuList->Create();
    this->NeedleTypeMenuList->SetLabelText("Needle type");
    this->NeedleTypeMenuList->SetBalloonHelpString("Select the needle type");
    }
    
  // 9/16/2012 ayamada
  this->Script("grid %s -row 2 -column 0 -padx 2 -pady 2 -sticky e", this->ShowRobotButton->GetWidgetName());
  this->Script("grid %s -row 2 -column 1 -padx 2 -pady 2 -sticky e", this->AddTargetsOnClickButton->GetWidgetName());
  this->Script("grid %s -row 2 -column 2 -padx 2 -pady 2 -sticky e", this->ShowAllNeedlePathsButton->GetWidgetName());
  
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::ShowTargetListFrame()
{
  vtkKWWidget *parent = this->GetGUI()->GetWizardWidget()->GetClientArea();

  if (!this->TargetListFrame)
    {
    this->TargetListFrame = vtkKWFrame::New();
    }
  if (!this->TargetListFrame->IsCreated())
    {
    this->TargetListFrame->SetParent(parent);
    this->TargetListFrame->Create();
    }
  this->Script("pack %s -side top -anchor nw -expand n -fill x -padx 2 -pady 2",
               this->TargetListFrame->GetWidgetName());

  if (!this->TargetList)
    {
    this->TargetList = vtkKWMultiColumnListWithScrollbars::New();
    this->TargetList->SetParent(this->TargetListFrame);
    this->TargetList->Create();
    this->TargetList->SetHeight(1);
    this->TargetList->GetWidget()->SetSelectionTypeToRow();
    this->TargetList->GetWidget()->SetSelectionBackgroundColor(1,0,0);
    this->TargetList->GetWidget()->MovableRowsOff();
    this->TargetList->GetWidget()->MovableColumnsOff();

      
    for (int col = 0; col < COL_COUNT; col ++)
      {
      if (this->ShowTargetOrientation || (col!=COL_OR_W && col!=COL_OR_X && col!=COL_OR_Y && col!=COL_OR_Z))
        {
          this->TargetList->GetWidget()->AddColumn(COL_LABELS[col]);
          this->TargetList->GetWidget()->SetColumnWidth(col, COL_WIDTHS[col]);
          this->TargetList->GetWidget()->SetColumnAlignmentToLeft(col);
          this->TargetList->GetWidget()->ColumnEditableOff(col);
        }
      }

    // Set editing options
      
    // 10/28/2011 ayamada
    this->TargetList->GetWidget()->ColumnEditableOn(NEEDLE_KIND);
    this->TargetList->GetWidget()->SetColumnEditWindowToEntry(NEEDLE_KIND);
      

    this->TargetList->GetWidget()->ColumnEditableOn(COL_NAME);
    this->TargetList->GetWidget()->SetColumnEditWindowToEntry(COL_NAME);

    for (int col = COL_X; col <= COL_Z; col ++)
      {
      this->TargetList->GetWidget()->ColumnEditableOn(col);
      this->TargetList->GetWidget()->SetColumnEditWindowToSpinBox(col);
      }
    if (this->ShowTargetOrientation)
      {
      for (int col = COL_OR_W; col <= COL_OR_Z; col ++)
        {
        this->TargetList->GetWidget()->ColumnEditableOn(col);
        this->TargetList->GetWidget()->SetColumnEditWindowToSpinBox(col);
        }
      }
    }
  this->Script( "pack %s -side top -anchor nw -expand n -fill x -padx 2 -pady 2",
                this->TargetList->GetWidgetName());


  if(!this->DeleteButton)
    {
    this->DeleteButton = vtkKWPushButton::New();
    }
  if(!this->DeleteButton->IsCreated())
    {
    this->DeleteButton->SetParent(this->TargetListFrame);
    this->DeleteButton->SetText("Delete selected target");
    this->DeleteButton->SetBalloonHelpString("Delete selected target point from the target list");
    this->DeleteButton->Create();
    }    
  this->Script("pack %s -side top -anchor ne -padx 2 -pady 4", 
                    this->DeleteButton->GetWidgetName());
  
  
  // 12/8/2011 ayamada
  // parameter bar
  if(!this->needlePathAdjuster)
  {
  this->needlePathAdjuster = vtkKWScaleWithEntry::New(); 
  }
  if(!this->needlePathAdjuster->IsCreated())
  {
  this->needlePathAdjuster->SetParent (this->TargetListFrame);
  this->needlePathAdjuster->Create( );
  this->needlePathAdjuster->SetLabelText("Left Groove");
  this->needlePathAdjuster->SetRange(-360, -350);
  this->needlePathAdjuster->SetValue(this->needlePathAdjusterValue);
  this->needlePathAdjuster->SetResolution(0.01);  
  }
  
  if(!this->needlePathAdjuster2)
  {  
  this->needlePathAdjuster2 = vtkKWScaleWithEntry::New();  
  }
  if(!this->needlePathAdjuster2->IsCreated())
  {
  this->needlePathAdjuster2->SetParent (this->TargetListFrame);
  this->needlePathAdjuster2->Create( );
  this->needlePathAdjuster2->SetLabelText("Right Groove");
  this->needlePathAdjuster2->SetRange(-360, 360);
  this->needlePathAdjuster2->SetValue(this->needlePathAdjusterValue2);
  this->needlePathAdjuster2->SetResolution(0.1);  
  }
  
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::ShowTargetControlFrame()
{
  vtkKWWidget *parent = this->GetGUI()->GetWizardWidget()->GetClientArea();

  if (!this->TargetControlFrame)
    {
    this->TargetControlFrame = vtkKWFrame::New();
    }
  if (!this->TargetControlFrame->IsCreated())
    {
    this->TargetControlFrame->SetParent(parent);
    this->TargetControlFrame->Create();
    }
  this->Script("pack %s -side top -anchor nw -expand n -fill x -padx 2 -pady 2",
               this->TargetControlFrame->GetWidgetName());

    if (!this->NeedlePositionMatrix)
    {
    this->NeedlePositionMatrix = vtkKWMatrixWidgetWithLabel::New();
    this->NeedlePositionMatrix->SetParent(this->TargetControlFrame);
    this->NeedlePositionMatrix->Create();
    this->NeedlePositionMatrix->SetLabelText("Position (X, Y, Z):");
    this->NeedlePositionMatrix->ExpandWidgetOff();
    this->NeedlePositionMatrix->GetLabel()->SetWidth(18);
    this->NeedlePositionMatrix->SetBalloonHelpString("Set the needle position");

    vtkKWMatrixWidget *matrix =  this->NeedlePositionMatrix->GetWidget();
    matrix->SetNumberOfColumns(3);
    matrix->SetNumberOfRows(1);
    matrix->SetElementWidth(12);
    matrix->SetRestrictElementValueToDouble();
    matrix->SetElementChangedCommandTriggerToAnyChange();
    }

  if (!this->NeedleOrientationMatrix && this->ShowTargetOrientation)
    {
    this->NeedleOrientationMatrix = vtkKWMatrixWidgetWithLabel::New();
    this->NeedleOrientationMatrix->SetParent(this->TargetControlFrame);
    this->NeedleOrientationMatrix->Create();
    this->NeedleOrientationMatrix->SetLabelText("Orientation (W, X, Y, Z):");
    this->NeedleOrientationMatrix->ExpandWidgetOff();
    this->NeedleOrientationMatrix->GetLabel()->SetWidth(18);
    this->NeedleOrientationMatrix->SetBalloonHelpString("Set the needle orientation");

    vtkKWMatrixWidget *matrix =  this->NeedleOrientationMatrix->GetWidget();
    matrix->SetNumberOfColumns(4);
    matrix->SetNumberOfRows(1);
    matrix->SetElementWidth(12);
    matrix->SetRestrictElementValueToDouble();
    matrix->SetElementChangedCommandTriggerToAnyChange();
    }

  if (this->ShowTargetOrientation)
    {
    // 1/17/2011 ayamada
    //this->Script("pack %s %s -side top -anchor nw -expand n -padx 2 -pady 2",
    //           this->NeedlePositionMatrix->GetWidgetName(),
    //           this->NeedleOrientationMatrix->GetWidgetName());
    this->Script("pack %s -side top -anchor nw -expand n -padx 2 -pady 2",
                  this->NeedlePositionMatrix->GetWidgetName());
    }
  else 
    {
    this->Script("pack %s -side top -anchor nw -expand n -padx 2 -pady 2",
               this->NeedlePositionMatrix->GetWidgetName());
    }


  if(!this->Message)
    {
    this->Message = vtkKWText::New();
    }
  if(!this->Message->IsCreated())
    {
    this->Message->SetParent(this->TargetControlFrame);
    this->Message->Create();
    this->Message->SetText("Command Status");      
    this->Message->SetBackgroundColor(0.7, 0.7, 0.95);
    this->Message->SetHeight(4);
    this->Message->SetWrapToWord();
    this->Message->ReadOnlyOn();
    this->Message->SetBorderWidth(2);
    this->Message->SetReliefToGroove();
    this->Message->SetFont("times 11 bold");
    }
  this->Script("pack %s -side top -anchor nw -expand n -fill x -padx 2 -pady 6", 
                this->Message->GetWidgetName());

  if(!this->Message2)
  {
    this->Message2 = vtkKWText::New();
  }
  if(!this->Message2->IsCreated())
  {
    this->Message2->SetParent(this->TargetControlFrame);
    this->Message2->Create();
    this->Message2->SetText("Robot Status");      
    this->Message2->SetBackgroundColor(0.7, 0.7, 0.95);
    this->Message2->SetHeight(4);
    this->Message2->SetWrapToWord();
    this->Message2->ReadOnlyOn();
    this->Message2->SetBorderWidth(2);
    this->Message2->SetReliefToGroove();
    this->Message2->SetFont("times 11 bold");
  }
  this->Script("pack %s -side top -anchor nw -expand n -fill x -padx 2 -pady 6", 
               this->Message2->GetWidgetName());
  
  
  if (!this->MoveButton)
    {
    this->MoveButton = vtkKWPushButton::New();
    this->MoveButton->SetParent (this->TargetControlFrame);
    this->MoveButton->Create();
    this->MoveButton->SetText("Move");
    this->MoveButton->SetBalloonHelpString("Move the robot to the position");
    }

  if (!this->StopButton)
    {
    this->StopButton = vtkKWPushButton::New();
    this->StopButton->SetParent (this->TargetControlFrame);
    this->StopButton->Create();
    this->StopButton->SetText("Stop");
    this->StopButton->SetBalloonHelpString("Stop the robot");
    }

  this->Script("pack %s %s -side left -anchor nw -expand n -padx 2 -pady 2",
               this->MoveButton->GetWidgetName(),
               this->StopButton->GetWidgetName());

}


//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::ProcessGUIEvents(vtkObject *caller,
                                          unsigned long event, void *callData)
{
  
  // 12/26/2011 ayamada
  if(this->GetLogic()->workPhaseStatus == 2)  
  {
  // -----------------------------------------------------------------
  // Move Button Pressed

  if (this->MoveButton == vtkKWPushButton::SafeDownCast(caller)
      && event == vtkKWPushButton::InvokedEvent)
    {
  
      // 1/27/2012 ayamada
      this->commandStatus = 1;
      
      // 1/26/2012 ayamada
      if (this->TimerFlag == 0)
      {
        this->TimerFlag = 1;
        this->TimerInterval = 500;  //1000 = 1 sec // 100 ms 4/25/2010 ayamada
        
        ProcessTimerEvents();
      
      }        
  
      
    if (this->Logic && this->NeedlePositionMatrix)
      {
      float position[3]={0,0,0};   // position parameters
      float orientation[4]={1,0,0,0}; // orientation parameters

      vtkKWMatrixWidget* matrix = this->NeedlePositionMatrix->GetWidget();
      
      position[0] = (float) matrix->GetElementValueAsDouble(0, 0);
      position[1] = (float) matrix->GetElementValueAsDouble(0, 1);
      position[2] = (float) matrix->GetElementValueAsDouble(0, 2);
      
      // send position data to process timer event as initial data for each needle.
      this->previousPositionX = position[0];
      this->previousPositionY = position[1];
      this->previousPositionZ = position[2];

      if (this->ShowTargetOrientation && this->NeedleOrientationMatrix)
      {
        matrix = this->NeedleOrientationMatrix->GetWidget();
        orientation[0] = (float) matrix->GetElementValueAsDouble(0, 0);
        orientation[1] = (float) matrix->GetElementValueAsDouble(0, 1);
        orientation[2] = (float) matrix->GetElementValueAsDouble(0, 2);
        orientation[3] = (float) matrix->GetElementValueAsDouble(0, 3);
      }

      
      vtkMRMLNode* node = this->GetLogic()->GetApplicationLogic()->GetMRMLScene()->GetNodeByID(this->GetRobotProbeNavManager()->GetRobotNode()->GetMarkersPositionTransformId());
      vtkMRMLLinearTransformNode* transformNode = vtkMRMLLinearTransformNode::SafeDownCast(node); //this->GetRobotProbeNavManager()->GetNeedlePathTransformNode1()

      if (transformNode)
        {
          
        // ---------------------------------------------------------
        // 7/3/2012 ayamada
        // convert from RAS coordinate to robot coordinate  
        vtkMRMLRobotNode* robot=this->Logic->GetRobotNode();
        vtkMRMLNode*   robotNode = vtkMRMLNode::New();
        robotNode = vtkMRMLNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetCalibrationObjectTransformId()));
        vtkMRMLLinearTransformNode* robotTransformNode = vtkMRMLLinearTransformNode::SafeDownCast(robotNode);
        vtkMatrix4x4* robotTransform = robotTransformNode->GetMatrixTransformToParent();
          
        // get matrix that converts RAS coordinates to XY
        vtkMatrix4x4* RASToXY = vtkMatrix4x4::New();
        RASToXY->DeepCopy(robotTransform);
        // invert it and convert registered tracking coordinates from RAS to XY
        RASToXY->Invert();
        
        // 9/4/2012 ayamada  
          vtkMatrix4x4* positionMatrix = vtkMatrix4x4::New();//transformNode->GetMatrixTransformToParent();
        positionMatrix->Identity(); 
        
        vtkMatrix4x4* positionMatrixAfterProcess = vtkMatrix4x4::New();//transformNode->GetMatrixTransformToParent();
        positionMatrixAfterProcess->Identity(); 
          
        // 8/18/2012 ayamada: change elements
        positionMatrix->SetElement(0,3,(float) position[0]);
        positionMatrix->SetElement(1,3,(float) position[1]);
        positionMatrix->SetElement(2,3,(float) position[2]);
          
        vtkMatrix4x4::Multiply4x4(RASToXY, positionMatrix, positionMatrixAfterProcess);
        // ---------------------------------------------------------
          
                    
        vtkMatrix4x4* matrix = transformNode->GetMatrixTransformToParent();
        igtl::Matrix4x4 igtlmatrix;

          
        // 2/24/2012 ayamada; target position
        igtl::QuaternionToMatrix(orientation, igtlmatrix);
          
        // 7/3/2012 ayamada
        // change order
        // 8/20/2012 ayamada modified
          igtlmatrix[0][3] = positionMatrixAfterProcess->GetElement(0,3);
          igtlmatrix[1][3] = positionMatrixAfterProcess->GetElement(1,3);
          igtlmatrix[2][3] = positionMatrixAfterProcess->GetElement(2,3);

        // 1/27/2012 ayamada
        igtlmatrix[1][0] = this->stopSignal;
        
        // 8/25/2012 ayamada
        igtlmatrix[2][0] = this->RobotControllerStatus;
        igtlmatrix[0][2] = this->robotStatus+1;
        igtlmatrix[1][2] = this->orderOfNeedlePlacement[this->robotStatus];
        
        // 10/3/2012 ayamada
        // put q1 value first to avoid moving the carriage to zero.
        matrix->SetElement(0, 3, igtlmatrix[0][3]);
        
        matrix->SetElement(1, 2, 0.0);
          
        matrix->SetElement(0, 0, 1.0);//igtlmatrix[0][0]);
        matrix->SetElement(1, 0, 0.0);//igtlmatrix[1][0]);
        // 8/19/2012 ayamada
        matrix->SetElement(2, 0, igtlmatrix[2][0]);
        matrix->SetElement(3, 0, 0.0);//igtlmatrix[3][0]);

        matrix->SetElement(0, 1, 0.0);//igtlmatrix[0][1]);
        matrix->SetElement(1, 1, 1.0);//igtlmatrix[1][1]);
        matrix->SetElement(2, 1, 0.0);//igtlmatrix[2][1]);
        matrix->SetElement(3, 1, 0.0);//igtlmatrix[3][1]);

        matrix->SetElement(0, 2, igtlmatrix[0][2]/*this->robotStatus+1*/); // 7/17/2012 ayamada: this->robotStatus l.2330
        matrix->SetElement(2, 2, 1.0);//igtlmatrix[2][2]);
        matrix->SetElement(3, 2, 0.0);//igtlmatrix[3][2]);

        matrix->SetElement(1, 3, igtlmatrix[1][3]);
        matrix->SetElement(2, 3, igtlmatrix[2][3]);
        matrix->SetElement(3, 3, 1.0);//igtlmatrix[3][3]);
        
        // 9/4/2012 ayamada
        matrix->SetElement(1, 2, igtlmatrix[1][2]); // 7/17/2012 ayamada: this->orderOfNeedlePlacement[this->robotStatus] l.1820
  
          
        std::cout << "ProcessGUIevent, l.847(vtkRobotProbeNavTargetingStep.cxx) TARGETPOISITION = " << std::endl
          << matrix->GetElement(0, 0) << ", " << matrix->GetElement(0, 1) << ", " << matrix->GetElement(0, 2) << ", " << matrix->GetElement(0, 3) << ", "
          << matrix->GetElement(1, 0) << ", " << matrix->GetElement(1, 1) << ", " << matrix->GetElement(1, 2) << ", " << matrix->GetElement(1, 3) << ", "
          << matrix->GetElement(2, 0) << ", " << matrix->GetElement(2, 1) << ", " << matrix->GetElement(2, 2) << ", " << matrix->GetElement(2, 3) << ", "
          << matrix->GetElement(3, 0) << ", " << matrix->GetElement(3, 1) << ", " << matrix->GetElement(3, 2) << ", " << matrix->GetElement(3, 3)
          << std::endl;
          
          
        char str3[512];  
        sprintf(str3, "(%.2f %.2f %.2f %.2f\n %.2f %.2f %.2f %.2f\n %.2f %.2f %.2f %.2f\n %.2f %.2f %.2f %.2f\n)", 
                  igtlmatrix[0][0],igtlmatrix[0][1],igtlmatrix[0][2],igtlmatrix[0][3],
                  igtlmatrix[1][0],igtlmatrix[1][1],igtlmatrix[1][2],igtlmatrix[1][3],
                  igtlmatrix[2][0],igtlmatrix[2][1],igtlmatrix[2][2],igtlmatrix[2][3],
                  igtlmatrix[3][0],igtlmatrix[3][1],igtlmatrix[3][2],igtlmatrix[3][3]
                );
        this->Message->SetText(str3);  
          
        vtkMatrix4x4* transformToParent = transformNode->GetMatrixTransformToParent();
        transformToParent->DeepCopy(matrix);

          
        // 1/18/2012 ayamada
        this->Message->SetText("Data was created\nTarget position was sent to Robot controller\n");
          
        // 1/22/2012 ayamada  
        // Send move to command 
        // -> go to process timer event  
        this->GetRobotProbeNavManager()->GetRobotNode()->MoveTo(transformNode->GetID());

        }
      }
    }

  // -----------------------------------------------------------------
  // Stop Button Pressed

  else if (this->StopButton == vtkKWPushButton::SafeDownCast(caller)
      && event == vtkKWPushButton::InvokedEvent)
    {
      // 1/18/2012 ayamada
      this->Message->SetText("Stop signal was sent to Robot controller\n");
      
      // 1/27/2012 ayamada
      this->stopSignal = 10;

    }

  /////////

  vtkMRMLRobotProbeNavManagerNode *mrmlNode = this->GetGUI()->GetRobotProbeNavManagerNode();

  if(!mrmlNode)
      return;

  if (this->DeleteButton == vtkKWPushButton::SafeDownCast(caller)
      && event == vtkKWPushButton::InvokedEvent)
    {
      
      std::cout << "DeleteButton was pushed!!" << endl;
      
      vtkRobotProbeNavTargetDescriptor *targetDesc = mrmlNode->GetTargetDescriptorAtIndex(mrmlNode->GetCurrentTargetIndex());       
      if (this->TargetPlanListNode!=NULL && targetDesc!=NULL)
      {
        int fidIndex=this->TargetPlanListNode->GetFiducialIndex(targetDesc->GetFiducialID());
        if (fidIndex>=0)
        {
          this->TargetPlanListNode->RemoveFiducial(fidIndex);
          mrmlNode->SetCurrentTargetIndex(-1);
          UpdateTargetListGUI();
          
          // 2/12/2012 ayamada
          if(fidIndex==3)
          {
            this->GetLogic()->existanceOfTarget3 = 0;            
          }else if(fidIndex==2)
          {
            this->GetLogic()->existanceOfTarget3 = 0;
            this->GetLogic()->existanceOfTarget2 = 0;            
          }else if(fidIndex==1)
          {
            this->GetLogic()->existanceOfTarget3 = 0;
            this->GetLogic()->existanceOfTarget2 = 0;            
            this->GetLogic()->existanceOfTarget1 = 0;                        
          }
          
        }
        else
        {
          vtkErrorMacro("Cannot delete target, fiducial not found");
        }
      }
      else
      {
        vtkErrorMacro("Cannot delete target, fiducial or target descriptor is invalid");
      }
    }
  
  // 12/8/2011 ayamada
  if (vtkKWScale::SafeDownCast(caller) == this->needlePathAdjuster->GetWidget()
    && static_cast<int>(event) == vtkKWScale::ScaleValueChangingEvent)
  {
  
    this->needlePathAdjusterValue = this->needlePathAdjuster->GetWidget()->GetValue();
    
    if(this->needlePathAdjusterValue > 180){
     
    this->GetLogic()->centerVariableK = this->needlePathAdjusterValue - 360;
    this->GetLogic()->adjusterSign = -1.0;
      
      
    }else if(this->needlePathAdjusterValue < -180){
    
    this->GetLogic()->centerVariableK = this->needlePathAdjusterValue + 360;
    this->GetLogic()->adjusterSign = -1.0;
      
      
    }else{

    this->GetLogic()->adjusterSign = 1.0;
    this->GetLogic()->centerVariableK = this->needlePathAdjusterValue;
    
    }
    
    this->UpdateTargetListGUI();
    //std::cout << "needlePathAdjusterValue =" << this->needlePathAdjusterValue << endl; 
    
  }
  
  if (vtkKWScale::SafeDownCast(caller) == this->needlePathAdjuster2->GetWidget()
      && static_cast<int>(event) == vtkKWScale::ScaleValueChangingEvent)
  {
    
    this->needlePathAdjusterValue2 = this->needlePathAdjuster2->GetWidget()->GetValue();
    
    if(this->needlePathAdjusterValue2 > 180){
      
      this->GetLogic()->centerVariableK2 = this->needlePathAdjusterValue2 - 360;
      this->GetLogic()->adjusterSign2 = -1.0;
      
      
    }else if(this->needlePathAdjusterValue2 < -180){
      
      this->GetLogic()->centerVariableK2 = this->needlePathAdjusterValue2 + 360;
      this->GetLogic()->adjusterSign2 = -1.0;
      
      
    }else{
      
      this->GetLogic()->adjusterSign2 = 1.0;
      this->GetLogic()->centerVariableK2 = this->needlePathAdjusterValue2;
      
    }    
    
    //this->needlePathAdjusterValue2 = this->needlePathAdjuster2->GetWidget()->GetValue();
    //this->GetLogic()->centerVariableK2 = this->needlePathAdjusterValue2;
    
    this->UpdateTargetListGUI();
    //std::cout << "needlePathAdjusterValue2 =" << this->needlePathAdjusterValue2 << endl; 
    
    
  }
  
  
  // load targeting volume dialog button
  if (this->LoadTargetingVolumeButton && this->LoadTargetingVolumeButton == vtkKWPushButton::SafeDownCast(caller) && (event == vtkKWPushButton::InvokedEvent))
    {
    this->GetApplication()->Script("::LoadVolume::ShowDialog");
    }

  // show workspace button
  if (this->ShowWorkspaceButton && this->ShowWorkspaceButton == vtkKWCheckButton::SafeDownCast(caller) && (event == vtkKWCheckButton::SelectedStateChangedEvent))
    {
      this->ShowWorkspaceModel(this->ShowWorkspaceButton->GetSelectedState() == 1);
    }

  // show robot button
  if (this->ShowRobotButton && this->ShowRobotButton == vtkKWCheckButton::SafeDownCast(caller) && (event == vtkKWCheckButton::SelectedStateChangedEvent))
    {
      this->ShowRobotModel(this->ShowRobotButton->GetSelectedState() == 1);
    }

 // activate fiducial placement
 if (this->AddTargetsOnClickButton && this->AddTargetsOnClickButton == vtkKWCheckButton::SafeDownCast(caller) && (event == vtkKWCheckButton::SelectedStateChangedEvent))
  {
    // Activate target fiducials in the Fiducial GUI
    if (this->GetLogic()==NULL)
    {
      vtkErrorMacro("Logic is invalid");
    }    
    else
    {
      EnableAddTargetsOnClickButton(this->AddTargetsOnClickButton->GetSelectedState()==1);
    }

  }

  
  // 9/25/2011 ayamada
  if (this->ShowAllNeedlePathsButton && this->ShowAllNeedlePathsButton == vtkKWCheckButton::SafeDownCast(caller) && (event == vtkKWCheckButton::SelectedStateChangedEvent))
  {

    this->ShowNeedleModel(this->ShowAllNeedlePathsButton->GetSelectedState() == 1);

    // 11/4/2011 ayamada
    //this->ShowVirtualCenterModel(this->ShowAllNeedlePathsButton->GetSelectedState() == 1);
    
    
    /*
    // Activate target fiducials in the Fiducial GUI
    if (this->GetLogic()==NULL)
    {
      vtkErrorMacro("Logic is invalid");
    }    
    else
    {
      EnableShowAllNeedlePathsButton(this->ShowAllNeedlePathsButton->GetSelectedState()==1);
    }
    */
    
  }
  
  

  if (this->NeedleTypeMenuList && this->NeedleTypeMenuList->GetWidget()->GetMenu() == vtkKWMenu::SafeDownCast(caller) && (event == vtkKWMenu::MenuItemInvokedEvent))
    {
      mrmlNode->SetCurrentNeedleIndex(this->NeedleTypeMenuList->GetWidget()->GetMenu()->GetIndexOfSelectedItem());
    }

  if (this->VolumeSelectorWidget == vtkSlicerNodeSelectorWidget::SafeDownCast(caller) &&
    event == vtkSlicerNodeSelectorWidget::NodeSelectedEvent ) 
  {
    vtkMRMLScalarVolumeNode *volume = vtkMRMLScalarVolumeNode::SafeDownCast(this->VolumeSelectorWidget->GetSelected());
    if (volume != NULL)
    {
      this->GetGUI()->GetLogic()->SelectVolumeInScene(volume, VOL_TARGETING);
      this->AddTargetsOnClickButton->SetSelectedState(1);
      
      // 9/25/2011 ayamada
      this->ShowAllNeedlePathsButton->SetSelectedState(1);

      // 11/18/2011 ayamada
      this->ShowRobotButton->SetSelectedState(1);
      
      
    }
  }
    
  } // end of this->GetLogic()->workPhaseStatus = 2 loop  
    
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::ProcessMRMLEvents(vtkObject *caller,
                                         unsigned long event, void *callData)
{

  /*
  // 12/26/2011 ayamada
  if((this->GetLogic()->workPhaseStatus != 2) && (this->GetLogic()->workPhaseInitialFlag == 2))
  {
      std::cerr << "checkInitialFlag!!!!!!!!!!!!!!" << std::endl;
      EnableAddTargetsOnClickButton(this->AddTargetsOnClickButton->GetSelectedState()==1);
  //  this->GetLogic()->workPhaseInitialFlag++;
  }
  std::cerr << "workPhaseInitialFlag = " << this->GetLogic()->workPhaseInitialFlag << std::endl;
  */
  
  
  // 12/26/2011 ayamada
  if(this->GetLogic()->workPhaseStatus == 2)  
    //if(this->GetGUI()->workPhaseStatus == 2)  
  {
    
  
  vtkMRMLFiducialListNode *targetNode = vtkMRMLFiducialListNode::SafeDownCast(caller);

        
  if (targetNode!=NULL && targetNode == this->TargetPlanListNode )
  {
    switch (event)
    {
    case vtkCommand::ModifiedEvent:
    case vtkMRMLScene::NodeAddedEvent: // Node Added Event : when a fiducial is added to the list
    case vtkMRMLScene::NodeRemovedEvent: // Node Removed Event : when a fiducial is reomved from the list
    case vtkMRMLFiducialListNode::FiducialModifiedEvent:
    case vtkMRMLFiducialListNode::DisplayModifiedEvent:
      UpdateTargetListGUI();
      break;
    }
  }

  if ( vtkMRMLScene::SafeDownCast(caller) == this->MRMLScene 
    && event == vtkMRMLScene::NodeAddedEvent )
    {
    vtkMRMLScalarVolumeNode *volumeNode = vtkMRMLScalarVolumeNode::SafeDownCast((vtkMRMLNode*)(callData));

    if (volumeNode!=NULL && this->VolumeSelectorWidget!=NULL && volumeNode!=this->VolumeSelectorWidget->GetSelected() )
      {
      // a new volume is loaded, set as the current targeting volume
      this->VolumeSelectorWidget->SetSelected(volumeNode);
      }
    }

  vtkMRMLRobotProbeNavManagerNode *managerNode = vtkMRMLRobotProbeNavManagerNode::SafeDownCast(caller);
  if (managerNode!=NULL && managerNode==GetRobotProbeNavManager())
    {
    switch (event)
      {
      case vtkMRMLRobotProbeNavManagerNode::CurrentTargetChangedEvent:
        // UpdateGUI is called anyways, no additional actions are needed
        break;
      }
    }

  if (this->MRMLScene!=NULL)
  {
    vtkMRMLInteractionNode *interactionNode = vtkMRMLInteractionNode::SafeDownCast(this->MRMLScene->GetNthNodeByClass(0, "vtkMRMLInteractionNode"));
    if ( vtkMRMLInteractionNode::SafeDownCast(caller) == interactionNode
      && interactionNode!=NULL && event == vtkMRMLInteractionNode::InteractionModeChangedEvent )
    {
      if (this->AddTargetsOnClickButton->GetSelectedState() == 1
        && interactionNode->GetCurrentInteractionMode()!=vtkMRMLInteractionNode::Place)
      {
        // the add points on click box is checked, but the interaction mode is not "Place" any more
        // uncheck the checkbox to show the user that click will not add point
        this->AddTargetsOnClickButton->SetSelectedState(0);
      }
      
      // 11/18/2011 ayamada comment out
      /*
      // 9/25/2011 ayamada
      if (this->ShowAllNeedlePathsButton->GetSelectedState() == 1
          && interactionNode->GetCurrentInteractionMode()!=vtkMRMLInteractionNode::Place)
      {
        // the add points on click box is checked, but the interaction mode is not "Place" any more
        // uncheck the checkbox to show the user that click will not add point
        this->ShowAllNeedlePathsButton->SetSelectedState(0);
      }
      */
      
    } 
  }
    
    
  
    
  } // end of this->GetLogic()->workPhaseStatus = 2 loop  
    
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::AddMRMLObservers()
{
  vtkMRMLRobotProbeNavManagerNode* manager=this->GetRobotProbeNavManager();
  if (manager==NULL)
  {
    return;
  }

  vtkMRMLFiducialListNode* plan = manager->GetTargetPlanListNode();
  if (plan)
  {
    vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
    events->InsertNextValue(vtkCommand::ModifiedEvent);
    events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
    events->InsertNextValue(vtkMRMLScene::NodeRemovedEvent);    
    events->InsertNextValue(vtkMRMLFiducialListNode::DisplayModifiedEvent);
    events->InsertNextValue(vtkMRMLFiducialListNode::FiducialModifiedEvent);

    // Set and observe target plan list
    //vtkObject *oldNode = this->TargetPlanListNode;
    this->MRMLObserverManager->SetAndObserveObjectEvents(vtkObjectPointer(&(this->TargetPlanListNode)),(plan),(events));
  }
 
  if (manager->HasObserver(vtkMRMLRobotProbeNavManagerNode::CurrentTargetChangedEvent, (vtkCommand *)this->MRMLCallbackCommand) < 1)
  {
    manager->AddObserver(vtkMRMLRobotProbeNavManagerNode::CurrentTargetChangedEvent, this->MRMLCallbackCommand);
  }  

  if (this->MRMLScene!=NULL)
  {
    if (this->MRMLScene->HasObserver(vtkMRMLScene::NodeAddedEvent, (vtkCommand *)this->MRMLCallbackCommand) < 1)
    {
      this->MRMLScene->AddObserver(vtkMRMLScene::NodeAddedEvent, this->MRMLCallbackCommand);
    }    

    vtkMRMLInteractionNode *interactionNode = vtkMRMLInteractionNode::SafeDownCast(this->MRMLScene->GetNthNodeByClass(0, "vtkMRMLInteractionNode"));
    if (interactionNode!=NULL)
    {
      if (interactionNode->HasObserver(vtkMRMLInteractionNode::InteractionModeChangedEvent, (vtkCommand *)this->MRMLCallbackCommand) < 1)
      {
        interactionNode->AddObserver(vtkMRMLInteractionNode::InteractionModeChangedEvent, this->MRMLCallbackCommand);
      }    
    }
  }
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::RemoveMRMLObservers()
{
  if (this->TargetPlanListNode!=NULL)
  {    
    this->MRMLObserverManager->SetAndObserveObjectEvents(vtkObjectPointer(&(this->TargetPlanListNode)), NULL, NULL);
  }
  vtkMRMLRobotProbeNavManagerNode* manager=this->GetRobotProbeNavManager();
  if (manager!=NULL)
  {
    manager->RemoveObservers(vtkMRMLRobotProbeNavManagerNode::CurrentTargetChangedEvent, this->MRMLCallbackCommand);        
  }
  if (this->MRMLScene!=NULL)
  {
    this->MRMLScene->RemoveObservers(vtkMRMLScene::NodeAddedEvent, this->MRMLCallbackCommand);

    vtkMRMLInteractionNode *interactionNode = vtkMRMLInteractionNode::SafeDownCast(this->MRMLScene->GetNthNodeByClass(0, "vtkMRMLInteractionNode"));
    if (interactionNode!=NULL)
    {
      interactionNode->RemoveObservers(vtkMRMLInteractionNode::InteractionModeChangedEvent, this->MRMLCallbackCommand);
    }
  }
  
    
  
}

//---------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::OnMultiColumnListUpdate(int row, int col, char * str)
{
  
  // 12/26/2011 ayamada
  if(this->GetLogic()->workPhaseStatus == 2)  
    //if(this->GetGUI()->workPhaseStatus == 2)  
  {
    

  vtkMRMLFiducialListNode* fidList = this->GetRobotProbeNavManager()->GetTargetPlanListNode();

  if (fidList == NULL)
    {
    return;
    }

  // 1/17/2012 ayamada
  //fidList->SetLocked(0);  
    
  bool updated=false;

  // make sure that the row and column exists in the table
  if ((row >= 0) && (row < this->TargetList->GetWidget()->GetNumberOfRows()) &&
      (col >= 0) && (col < this->TargetList->GetWidget()->GetNumberOfColumns()))
    {
      
      
    // 10/28/2011 ayamada
    if(col == NEEDLE_KIND)
    {
      fidList->SetNthFiducialLabelText(row, str);
      updated=true;
    }
      
    // now update the requested value
    if (col == COL_NAME) // 9/25/2011 ayamada
      {
      fidList->SetNthFiducialLabelText(row, str);
        
      // 10/28/2011 ayamada  
      //fidList->SetNthFiducialLabelText(row-4, str);
        
      updated=true;
      }
    else if (col >= COL_X && col <= COL_Z)
      {
      // get the current xyz
      float * xyz = fidList->GetNthFiducialXYZ(row);
      // now set the new one
      float newCoordinate = atof(str);
        
         
      // 10/19/2011 ayamada: receiving test
      //std::cerr << "fiducial list xyz" << xyz[0] << xyz[1] << xyz[2] <<std::endl; 
      //std::cerr << "fiducial list xyz" << xyz[0] <<std::endl; 
      //std::cerr << "fiducial list xyz" <<std::endl; 

        
      
      // Update the GUI with the number that was converted from the entered string
      // (this way the user gets feedback if the conversion failed, sue to an invalid character accidentally entered, etc.)
      if (this->TargetList && this->TargetList->GetWidget())
      {
        std::ostrstream os;    
        os << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
        os << newCoordinate << std::ends;
        this->TargetList->GetWidget()->SetCellText(row,col,os.str());
        os.rdbuf()->freeze();        
      }

      if ( xyz )
        {
        if (col == COL_X)
          {
          fidList->SetNthFiducialXYZ(row, newCoordinate, xyz[1], xyz[2]);
          updated=true;
          }
        if (col == COL_Y)
          {
          fidList->SetNthFiducialXYZ(row, xyz[0], newCoordinate, xyz[2]);
          updated=true;
          }
        if (col == COL_Z)
          {
          fidList->SetNthFiducialXYZ(row, xyz[0], xyz[1], newCoordinate);
          updated=true;
          }
        }            
      }
    else if (this->ShowTargetOrientation && col >= COL_OR_W  && col <= COL_OR_Z)
      {
      float * wxyz = fidList->GetNthFiducialOrientation(row);
      float newCoordinate = atof(str);
      if (col == COL_OR_W)
        {
        fidList->SetNthFiducialOrientation(row, newCoordinate, wxyz[1], wxyz[2], wxyz[3]);
        updated=true;
        }
      if (col == COL_OR_X)
        {
        fidList->SetNthFiducialOrientation(row, wxyz[0], newCoordinate, wxyz[2], wxyz[3]);
        updated=true;
        }
      if (col == COL_OR_Y)
        {
        fidList->SetNthFiducialOrientation(row, wxyz[0], wxyz[1], newCoordinate, wxyz[3]);
        updated=true;
        }
      if (col == COL_OR_Z)
        {
        fidList->SetNthFiducialOrientation(row, wxyz[0], wxyz[1], wxyz[2], newCoordinate);
        updated=true;
        }
      }
    else
      {
      return;
      }
    }
  else
    {
    }
  if (updated)
  {
    this->GetLogic()->UpdateTargetListFromMRML();
    // Current target has changed, force refresh (if SetCurrentTargetIndex is called with the current target index, then it is ignored)
    int currentTarget=this->GetRobotProbeNavManager()->GetCurrentTargetIndex();
    this->GetRobotProbeNavManager()->SetCurrentTargetIndex(-1);
    this->GetRobotProbeNavManager()->SetCurrentTargetIndex(currentTarget);
  }
    
  } // workPhaseStatus
    
}

//---------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::OnMultiColumnListSelection()
{
  vtkMRMLFiducialListNode* fidList = this->GetRobotProbeNavManager()->GetTargetPlanListNode();

  if (fidList == NULL)
    {
    return;
    }

  int numRows = this->TargetList->GetWidget()->GetNumberOfSelectedRows();
  if (numRows == 1)
    {   
    
    int rowIndex = this->TargetList->GetWidget()->GetIndexOfFirstSelectedRow();    
    int targetIndex=this->TargetList->GetWidget()->GetRowAttributeAsInt(rowIndex, TARGET_INDEX_ATTR);
    vtkRobotProbeNavTargetDescriptor* targetDesc=this->GetRobotProbeNavManager()->GetTargetDescriptorAtIndex(targetIndex);    

    if (targetDesc==NULL)
      {
      vtkErrorMacro("Target descriptor not found");
      return;
      }
   
    this->GetRobotProbeNavManager()->SetCurrentTargetIndex(targetIndex);
    //if (this->NeedlePositionMatrix)
    // 10/24/2011 ayamada
    if (this->NeedlePositionMatrix && this->NeedleOrientationMatrix)
      {
      vtkKWMatrixWidget* matrix = this->NeedlePositionMatrix->GetWidget();
        
      matrix->SetElementValueAsDouble(0, 0, targetDesc->GetRASLocation()[0]);
      matrix->SetElementValueAsDouble(0, 1, targetDesc->GetRASLocation()[1]);
      matrix->SetElementValueAsDouble(0, 2, targetDesc->GetRASLocation()[2]);

      vtkKWMatrixWidget* oMatrix = this->NeedleOrientationMatrix->GetWidget();
  
      // 10/24/2011 dummy orientation
      // If you have to use OpenIGTLink, this flow will be useful for you.
      oMatrix->SetElementValueAsDouble(0, 0, 1/*5*//*targetDesc->GetRASLocation()[0]*/);
      oMatrix->SetElementValueAsDouble(0, 1, 0/*5*//*targetDesc->GetRASLocation()[1]*/);
      oMatrix->SetElementValueAsDouble(0, 2, 0/*5*//*targetDesc->GetRASLocation()[2]*/);
      oMatrix->SetElementValueAsDouble(0, 3, 0/*targetDesc->GetRASLocation()[3]*/);
        
        
        
      // 10/19/2011 ayamada
      this->SendTargetingData(matrix, oMatrix,targetIndex);
        
      }
    if (this->ShowTargetOrientation && this->NeedleOrientationMatrix)
      {
      }
      
      // 10/19/2011 ayamada
      /*
      std::cerr << "TARGETPOISITION = "
      << targetDesc->GetRASLocation()[0] << ", "
      << targetDesc->GetRASLocation()[1] << ", "
      << targetDesc->GetRASLocation()[2] << std::endl;
      */

    /* it would be more rational to set the slice orientation here, but it is too slow
    // Set slice orientation to match the original acquisitions
    const char* volNodeID = this->GetRobotProbeNavManager()->GetTargetingVolumeNodeRef();
    vtkMRMLScalarVolumeNode *volNode=vtkMRMLScalarVolumeNode::SafeDownCast(this->GetLogic()->GetApplicationLogic()->GetMRMLScene()->GetNodeByID(volNodeID));
    if ( volNode!=NULL)
    {
      this->GetGUI()->GetLogic()->SetSliceViewFromVolume(volNode);
    }
    */

    // Don't move slices to the current target immediately, because it would some time, and the delay would confuse KWWidget's double-click detection algorithm
    // (double-clicks wouldn't be sensed, so cell editing by double-click wouldn't work).
    //this->GUI->BringTargetToViewIn2DViews(vtkRobotProbeNavGUI::BRING_MARKERS_TO_VIEW_KEEP_CURRENT_ORIENTATION);
    this->Script("after 1000 \"%s BringTargetToViewIn2DViews %i\"", this->GUI->GetTclName(), vtkRobotProbeNavGUI::BRING_MARKERS_TO_VIEW_KEEP_CURRENT_ORIENTATION);
    
    }
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::UpdateTargetListGUI()
{
  
  // 9/25/2011 test
  //std::cout << "UpdateTargetListGUI()" << endl;

  // 12/26/2011 ayamada
  if(this->GetLogic()->workPhaseStatus == 2)  
    //if(this->GetGUI()->workPhaseStatus == 2)  
  {
    
  
  
  if (this->TargetList==NULL)
  {
    return; // there is no GUI, nothing to update
  }
  if (this->TargetList->GetWidget()==NULL)
  {
    return; // there is no GUI, nothing to update
  }

  vtkMRMLFiducialListNode* activeFiducialListNode=NULL;
  if (this->GetRobotProbeNavManager()!=NULL)
  {
    activeFiducialListNode=this->GetRobotProbeNavManager()->GetTargetPlanListNode();
  }

  if (activeFiducialListNode == NULL)    //clear out the list box
  {
    if (this->TargetList)
    {
      if (this->TargetList->GetWidget()->GetNumberOfRows() != 0)
      {
        this->TargetList->GetWidget()->DeleteAllRows();
      }
    }
    return;
  }

  // create new target points, if necessary
  this->GetLogic()->UpdateTargetListFromMRML();

  vtkMRMLRobotProbeNavManagerNode *manager = this->GetGUI()->GetRobotProbeNavManagerNode();
  if (!manager)
  {
    return;
  }

  //int numPoints = activeFiducialListNode->GetNumberOfFiducials();
  int numPoints = manager->GetTotalNumberOfTargets();

  bool deleteFlag = true;

  if (numPoints != this->TargetList->GetWidget()->GetNumberOfRows())
  {
    // clear out the multi column list box and fill it in with the
    // new list
    this->TargetList->GetWidget()->DeleteAllRows();
  }
  else
  {
    deleteFlag = false;
  }

  double *xyz;
  double *wxyz;

  for (int row = 0; row < numPoints; row++)
  {      
    int targetIndex=row;
    vtkRobotProbeNavTargetDescriptor* target = manager->GetTargetDescriptorAtIndex(targetIndex);
    NeedleDescriptorStruct* needle = manager->GetNeedle(target);

    if (deleteFlag)
    {
      // add a row for this point
      this->TargetList->GetWidget()->AddRow();
    }
    this->TargetList->GetWidget()->SetRowAttributeAsInt(row, TARGET_INDEX_ATTR, targetIndex);

    xyz=target->GetRASLocation();
    wxyz=target->GetRASOrientation();

    if (xyz == NULL)
    {
      vtkErrorMacro ("UpdateTargetListGUI: ERROR: got null xyz for point " << row << endl);
    }

    
    // 10/28/2011 ayamada; selected
    //vtkKWMultiColumnList* columnList = this->TargetList->GetWidget();

    
    if (target->GetName().compare(this->TargetList->GetWidget()->GetCellText(row,COL_NAME)) != 0)
    {
      this->TargetList->GetWidget()->SetCellText(row,COL_NAME,target->GetName().c_str());

      // 10/28/2011 ayamada
      /*
      this->OnMultiColumnListSelection();
      
      std::ostrstream osDistanceToHolder;    
      osDistanceToHolder << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
      osDistanceToHolder << this->Logic->magnitudeOfDirectionVector << std::ends;
      columnList->SetCellText(row,DIS2TARGET,osDistanceToHolder.str());
      //osDistanceToHolder.rdbuf()->freeze();
      */
      if(row==0)
      {
        this->TargetList->GetWidget()->SetCellText(row,NEEDLE_KIND,"Center Groove");
        //columnList->SetCellText(row,DIS2TARGET,osDistanceToHolder.str());
        // 2/11/2012 ayamada
        this->GetLogic()->existanceOfTarget1 = 1;
        this->GetLogic()->getRow = 1;
      }else if(row==1)
      {
        this->TargetList->GetWidget()->SetCellText(row,NEEDLE_KIND,"Left Groove");          
        //columnList->SetCellText(row,DIS2TARGET,osDistanceToHolder.str());        
        // 2/11/2012 ayamada
        this->GetLogic()->existanceOfTarget2 = 1;
        this->GetLogic()->getRow = 2;
      }else if(row==2)
      {
        this->TargetList->GetWidget()->SetCellText(row,NEEDLE_KIND,"Right Groove");        
        //columnList->SetCellText(row,DIS2TARGET,osDistanceToHolder.str());
        // 2/11/2012 ayamada
        this->GetLogic()->existanceOfTarget3 = 1;
        this->GetLogic()->getRow = 3;
      }
      //osDistanceToHolder.rdbuf()->freeze();

    }               
    
    // selected
    vtkKWMultiColumnList* columnList = this->TargetList->GetWidget();
    
    
    // 10/29/2011 ayamada
    // display distance to target
    
    // before calculation, you'll read the function on vtkRobotProbeNavLogix.cxx. Then, you should use the valculated variables.
    //this->OnMultiColumnListSelection();
    //this->Logic->magnitudeOfDirectionVector;
    
    //std::ostrstream osDistanceToHolder;    
    //osDistanceToHolder << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
    //osDistanceToHolder << this->Logic->magnitudeOfDirectionVector << std::ends;
    //columnList->SetCellText(row,DIS2TARGET,osDistanceToHolder.str());
    //osDistanceToHolder.rdbuf()->freeze();
    int numRows = this->TargetList->GetWidget()->GetIndexOfFirstSelectedRow();//this->TargetList->GetWidget()->GetNumberOfSelectedRows();

    // 12/8/2011 ayamada
    if(numRows/*row*/==0)
    {
      this->OnMultiColumnListSelection();
      std::ostrstream osDistanceToHolder2;    
      osDistanceToHolder2 << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
      osDistanceToHolder2 << this->Logic->magnitudeOfDirectionVector2 << std::ends;
      columnList->SetCellText(numRows,DIS2SKINY/*DIS2TARGET*/,osDistanceToHolder2.str());
      // 7/4/2012 ayamada
      std::ostrstream osDistanceToHolder;    
      osDistanceToHolder << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
      osDistanceToHolder << this->Logic->magnitudeOfDirectionVector << std::ends;
      columnList->SetCellText(numRows,DIS2TARGET,osDistanceToHolder.str()/*"NO DATA"*/);
      osDistanceToHolder.rdbuf()->freeze();
      
      // 7/5/2012 ayamada
      // make order
      this->carriagePosition[0] = this->Logic->carriagePositionForOrder;
      
      // 1/18/2012 ayamada
      int rowIndex = this->TargetList->GetWidget()->GetIndexOfFirstSelectedRow();    
      int targetIndex=this->TargetList->GetWidget()->GetRowAttributeAsInt(rowIndex, TARGET_INDEX_ATTR);
      vtkRobotProbeNavTargetDescriptor* targetDesc=this->GetRobotProbeNavManager()->GetTargetDescriptorAtIndex(targetIndex);    
      // l.1265
      char str2[128];
      
      // 8/20/2012 ayamada: display XYZ position of the target
      //sprintf(str2, "Center Groove was selected\nTarget Tgts-P position is (%.2f %.2f %.2f)", 
      //        targetDesc->GetRASLocation()[0],targetDesc->GetRASLocation()[1],targetDesc->GetRASLocation()[2]);
      sprintf(str2, "Center Groove was selected\nTarget Tgts-P position is (%.2f %.2f %.2f)", 
              this->Logic->TargetPositionXYZ[0],this->Logic->TargetPositionXYZ[1],this->Logic->TargetPositionXYZ[2]);
      
      this->Message->SetText(str2);
      //this->Message->SetText("Center groove was selected\n");
      this->Message->SetBackgroundColor(0.60, 0.60, 0.95);      

      // 1/27/2012 ayamada
      if(this->commandStatus == 0)
      {
      this->robotStatus = 0;
      }
      
    }else if(numRows/*row*/==1)
    {
      this->OnMultiColumnListSelection();
      std::ostrstream osDistanceToHolder2;    
      osDistanceToHolder2 << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
      osDistanceToHolder2 << this->Logic->magnitudeOfDirectionVector2 << std::ends;
      columnList->SetCellText(numRows,DIS2SKINY/*DIS2TARGET*/,osDistanceToHolder2.str());
      // 7/4/2012 ayamada
      std::ostrstream osDistanceToHolder;    
      osDistanceToHolder << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
      osDistanceToHolder << this->Logic->magnitudeOfDirectionVector << std::ends;
      columnList->SetCellText(numRows,DIS2TARGET,osDistanceToHolder.str()/*"NO DATA"*/);
      osDistanceToHolder.rdbuf()->freeze();
      
      // 7/5/2012 ayamada
      // make order
      this->carriagePosition[1] = this->Logic->carriagePositionForOrder;
      
      // 1/18/2012 ayamada
      int rowIndex = this->TargetList->GetWidget()->GetIndexOfFirstSelectedRow();    
      int targetIndex=this->TargetList->GetWidget()->GetRowAttributeAsInt(rowIndex, TARGET_INDEX_ATTR);
      vtkRobotProbeNavTargetDescriptor* targetDesc=this->GetRobotProbeNavManager()->GetTargetDescriptorAtIndex(targetIndex);    
      // l.1265
      char str2[128];

      // 8/20/2012 ayamada: display XYZ position of the target
      //sprintf(str2, "Left Groove was selected\nTarget Tgts-P1 position is (%.2f %.2f %.2f)", targetDesc->GetRASLocation()[0],targetDesc->GetRASLocation()[1],targetDesc->GetRASLocation()[2]);
      sprintf(str2, "Left Groove was selected\nTarget Tgts-P1 position is (%.2f %.2f %.2f)", 
              this->Logic->TargetPositionXYZ[0],this->Logic->TargetPositionXYZ[1],this->Logic->TargetPositionXYZ[2]);

      this->Message->SetText(str2);
      //this->Message->SetText("Left groove was selected\n");
      this->Message->SetBackgroundColor(0.95, 0.60, 0.60);

      // 1/27/2012 ayamada
      if(this->commandStatus == 0)
      {
      this->robotStatus = 1;
      }
      
    }else if(numRows/*row*/==2)
    {
      this->OnMultiColumnListSelection();
      std::ostrstream osDistanceToHolder2;    
      osDistanceToHolder2 << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
      osDistanceToHolder2 << this->Logic->magnitudeOfDirectionVector2 << std::ends;
      columnList->SetCellText(numRows,DIS2SKINY/*DIS2TARGET*/,osDistanceToHolder2.str());
      // 7/4/2012 ayamada
      std::ostrstream osDistanceToHolder;    
      osDistanceToHolder << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
      osDistanceToHolder << this->Logic->magnitudeOfDirectionVector << std::ends;
      columnList->SetCellText(numRows,DIS2TARGET,osDistanceToHolder.str()/*"NO DATA"*/);
      osDistanceToHolder.rdbuf()->freeze();
      
      // 7/5/2012 ayamada
      // make order
      this->carriagePosition[2] = this->Logic->carriagePositionForOrder;
      
      // 1/18/2012 ayamada
      int rowIndex = this->TargetList->GetWidget()->GetIndexOfFirstSelectedRow();    
      int targetIndex=this->TargetList->GetWidget()->GetRowAttributeAsInt(rowIndex, TARGET_INDEX_ATTR);
      vtkRobotProbeNavTargetDescriptor* targetDesc=this->GetRobotProbeNavManager()->GetTargetDescriptorAtIndex(targetIndex);    
      // l.1265
      char str2[128];

      // 8/20/2012 ayamada: display XYZ position of the target
      //sprintf(str2, "Right Groove was selected\nTarget Tgts-P2 position is (%.2f %.2f %.2f)", targetDesc->GetRASLocation()[0],targetDesc->GetRASLocation()[1],targetDesc->GetRASLocation()[2]);
      sprintf(str2, "Right Groove was selected\nTarget Tgts-P2 position is (%.2f %.2f %.2f)", 
              this->Logic->TargetPositionXYZ[0],this->Logic->TargetPositionXYZ[1],this->Logic->TargetPositionXYZ[2]);

      this->Message->SetText(str2);
      //this->Message->SetText("Right groove was selected\n");
      this->Message->SetBackgroundColor(0.60, 0.95, 0.60);

      // 1/27/2012 ayamada
      if(this->commandStatus == 0)
      {
      this->robotStatus = 2;
      }        
    
    }
    //osDistanceToHolder.rdbuf()->freeze();
    
    if(carriagePosition[0] >= carriagePosition[1] && carriagePosition[0] >= carriagePosition[2])
    {
      this->orderOfNeedlePlacement[0] = 3;
      if(carriagePosition[1] >= carriagePosition[2])
      {
        this->orderOfNeedlePlacement[1] = 2;
        this->orderOfNeedlePlacement[2] = 1;
      }else{
        this->orderOfNeedlePlacement[1] = 1;
        this->orderOfNeedlePlacement[2] = 2;       
      }
    }else if(carriagePosition[1] >= carriagePosition[0] && carriagePosition[1] >= carriagePosition[2])
    {
      this->orderOfNeedlePlacement[1] = 3;
      if(carriagePosition[0] >= carriagePosition[2])
      {
        this->orderOfNeedlePlacement[0] = 2;
        this->orderOfNeedlePlacement[2] = 1;
      }else{
        this->orderOfNeedlePlacement[0] = 1;
        this->orderOfNeedlePlacement[2] = 2;       
      }      
    }else if(carriagePosition[2] >= carriagePosition[0] && carriagePosition[2] >= carriagePosition[1])
    {
      this->orderOfNeedlePlacement[2] = 3;
      if(carriagePosition[0] >= carriagePosition[1])
      {
        this->orderOfNeedlePlacement[0] = 2;
        this->orderOfNeedlePlacement[1] = 1;
      }else{
        this->orderOfNeedlePlacement[0] = 1;
        this->orderOfNeedlePlacement[1] = 2;       
      }      
    }   
    
    
    // 7/5/2012 ayamada
    // write the order on the table
    std::ostrstream osOrder1;    
    osOrder1 << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
    osOrder1 << this->orderOfNeedlePlacement[0] << std::ends;
    std::ostrstream osOrder2;    
    osOrder2 << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
    osOrder2 << this->orderOfNeedlePlacement[1] << std::ends;
    std::ostrstream osOrder3;    
    osOrder3 << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
    osOrder3 << this->orderOfNeedlePlacement[2] << std::ends;
    
    columnList->SetCellText(0,ORDER,osOrder1.str());
    osOrder1.rdbuf()->freeze();
    columnList->SetCellText(1,ORDER,osOrder2.str());
    osOrder2.rdbuf()->freeze();
    columnList->SetCellText(2,ORDER,osOrder3.str());
    osOrder3.rdbuf()->freeze();
    
    
    if (xyz != NULL)
    {
      for (int i = 0; i < 3; i ++) // for position (x, y, z)
      {
        if (deleteFlag || fabs(columnList->GetCellTextAsDouble(row,COL_X+i)-xyz[i])>vtkRobotProbeNavGUI::POSITION_PRECISION_TOLERANCE)
        {
          std::ostrstream os;    
          os << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
          os << xyz[i] << std::ends;
          columnList->SetCellText(row,COL_X+i,os.str());
          os.rdbuf()->freeze();
        }
      }
    }
    if (this->ShowTargetOrientation && wxyz != NULL)
    {
      for (int i = 0; i < 4; i ++) // for orientation (w, x, y, z)
      {
        if (deleteFlag || fabs(columnList->GetCellTextAsDouble(row, COL_OR_W+i)-wxyz[i])>vtkRobotProbeNavGUI::POSITION_PRECISION_TOLERANCE)
        {
          std::ostrstream os;    
          os << std::setiosflags(ios::fixed | ios::showpoint) << std::setprecision(vtkRobotProbeNavGUI::POSITION_PRECISION_DIGITS);
          os << wxyz[i] << std::ends;
          columnList->SetCellText(row,COL_OR_W+i,os.str());
          os.rdbuf()->freeze();
        }
      }
    }

    if (needle->mDescription.compare(this->TargetList->GetWidget()->GetCellText(row,COL_NEEDLE)) != 0)
    {
      this->TargetList->GetWidget()->SetCellText(row,COL_NEEDLE,needle->mDescription.c_str());
    }

  }       

  int currentTargetIndex=this->GetRobotProbeNavManager()->GetCurrentTargetIndex();
  if (currentTargetIndex<0)
  {
    this->TargetList->GetWidget()->ClearSelection();
  }
  else
  {
    int selectedTargetIndex=-1;
    int numRows = this->TargetList->GetWidget()->GetNumberOfSelectedRows();
    if (numRows == 1)
    {       
      int rowIndex = this->TargetList->GetWidget()->GetIndexOfFirstSelectedRow();    
      selectedTargetIndex=this->TargetList->GetWidget()->GetRowAttributeAsInt(rowIndex, TARGET_INDEX_ATTR);
    }
    if (currentTargetIndex!=selectedTargetIndex)
    {
      for (int rowIndex=0; rowIndex<this->TargetList->GetWidget()->GetNumberOfRows(); rowIndex++)
      {
        if (this->TargetList->GetWidget()->GetRowAttributeAsInt(rowIndex, TARGET_INDEX_ATTR)==currentTargetIndex)
        {
          // found the row corresponding to the current target
          this->TargetList->GetWidget()->SelectSingleRow(rowIndex);
          break;
        }
      }
    } 
  }
    
  } // workPhaseStatus

}



//-----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::AddGUIObservers()
{
  this->RemoveGUIObservers();

  if (this->LoadTargetingVolumeButton)
    {
    this->LoadTargetingVolumeButton->AddObserver(vtkKWPushButton::InvokedEvent, (vtkCommand *)this->GUICallbackCommand); 
    }
  if (this->VolumeSelectorWidget)
    {
    this->VolumeSelectorWidget->AddObserver ( vtkSlicerNodeSelectorWidget::NodeSelectedEvent, (vtkCommand *)this->GUICallbackCommand);  
    }
  if (this->ShowWorkspaceButton)
    {
      this->ShowWorkspaceButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand *)this->GUICallbackCommand);
    }  
  if (this->ShowRobotButton)
    {
      this->ShowRobotButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand *)this->GUICallbackCommand);
    }  
  if (this->AddTargetsOnClickButton)
    {
      this->AddTargetsOnClickButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand *)this->GUICallbackCommand);
    }  

  // 9/25/2011 ayamada
  if (this->ShowAllNeedlePathsButton)
  {
    this->ShowAllNeedlePathsButton->AddObserver(vtkKWCheckButton::SelectedStateChangedEvent, (vtkCommand *)this->GUICallbackCommand);
  }  
  
  
  if (this->NeedleTypeMenuList)
    {
    this->NeedleTypeMenuList->GetWidget()->GetMenu()->AddObserver(vtkKWMenu::MenuItemInvokedEvent, (vtkCommand *)this->GUICallbackCommand);
    }
  if (this->DeleteButton)
    {
    this->DeleteButton->AddObserver(vtkKWPushButton::InvokedEvent, (vtkCommand *)this->GUICallbackCommand);
    }
  if (this->MoveButton)
    {
    this->MoveButton->AddObserver(vtkKWPushButton::InvokedEvent, (vtkCommand *)this->GUICallbackCommand);
    }
  if (this->StopButton)
    {
    this->StopButton->AddObserver(vtkKWPushButton::InvokedEvent, (vtkCommand *)this->GUICallbackCommand);
    }
  if (this->TargetList)
    {
    this->TargetList->GetWidget()->SetCellUpdatedCommand(this, "OnMultiColumnListUpdate");    
    this->TargetList->GetWidget()->SetSelectionCommand(this, "OnMultiColumnListSelection");    // allows updates when a target is re-selected
    }
  
  // 12/9/2011 ayamada
  this->needlePathAdjuster->GetWidget()->AddObserver(vtkKWScale::ScaleValueChangingEvent, (vtkCommand *)this->GUICallbackCommand );
  this->needlePathAdjuster->GetWidget()->AddObserver(vtkKWScale::ScaleValueStartChangingEvent, (vtkCommand *)this->GUICallbackCommand );
  this->needlePathAdjuster2->GetWidget()->AddObserver(vtkKWScale::ScaleValueChangingEvent, (vtkCommand *)this->GUICallbackCommand );
  this->needlePathAdjuster2->GetWidget()->AddObserver(vtkKWScale::ScaleValueStartChangingEvent, (vtkCommand *)this->GUICallbackCommand );
  
  
}
//-----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::RemoveGUIObservers()
{
  if (this->LoadTargetingVolumeButton)
    {
    this->LoadTargetingVolumeButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand); 
    }
  if (this->VolumeSelectorWidget)
    {
    this->VolumeSelectorWidget->RemoveObserver ((vtkCommand *)this->GUICallbackCommand);  
    }
  if (this->ShowWorkspaceButton)
    {
    this->ShowWorkspaceButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }  
  if (this->ShowRobotButton)
    {
    this->ShowRobotButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }  
  if (this->AddTargetsOnClickButton)
    {
      this->AddTargetsOnClickButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }  

  // 9/25/2011 ayamada
  if (this->ShowAllNeedlePathsButton)
  {
    this->ShowAllNeedlePathsButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
  }  
  
  
  if (this->NeedleTypeMenuList)
    {
      this->NeedleTypeMenuList->GetWidget()->GetMenu()->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }
  if (this->DeleteButton)
    {
    this->DeleteButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }
  if (this->MoveButton)
    {
    this->MoveButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }
  if (this->StopButton)
    {
    this->StopButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    }
  if (this->TargetList)
    {
    this->TargetList->GetWidget()->SetCellUpdatedCommand(this, "");
    this->TargetList->GetWidget()->SetSelectionCommand(this, "");
    }
  
  // 12/9/2011 ayamada
  this->needlePathAdjuster->GetWidget()->RemoveObservers(vtkKWScale::ScaleValueChangingEvent, (vtkCommand *)this->GUICallbackCommand );
  this->needlePathAdjuster->GetWidget()->RemoveObservers(vtkKWScale::ScaleValueStartChangingEvent, (vtkCommand *)this->GUICallbackCommand );
  this->needlePathAdjuster2->GetWidget()->RemoveObservers(vtkKWScale::ScaleValueChangingEvent, (vtkCommand *)this->GUICallbackCommand );
  this->needlePathAdjuster2->GetWidget()->RemoveObservers(vtkKWScale::ScaleValueStartChangingEvent, (vtkCommand *)this->GUICallbackCommand );
  
  
  
}

//--------------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::UpdateGUI()
{
  
  // 12/26/2011 ayamada
  if(this->GetLogic()->workPhaseStatus == 2)  
    //if(this->GetGUI()->workPhaseStatus == 2)  
  {
    
  
  vtkMRMLRobotProbeNavManagerNode *mrmlNode = this->GetGUI()->GetRobotProbeNavManagerNode();

  if (!mrmlNode)
  {
    return;
  }

  
  const char* volNodeID = mrmlNode->GetTargetingVolumeNodeRef();
  vtkMRMLScalarVolumeNode *volNode=vtkMRMLScalarVolumeNode::SafeDownCast(this->GetLogic()->GetApplicationLogic()->GetMRMLScene()->GetNodeByID(volNodeID));
  if ( volNode!=NULL && this->VolumeSelectorWidget!=NULL && this->VolumeSelectorWidget->IsCreated())
  {
    this->VolumeSelectorWidget->UpdateMenu();
    this->VolumeSelectorWidget->SetSelected( volNode );
  }

  vtkMRMLRobotNode* robot=NULL;
  if (this->GetRobotProbeNavManager()!=NULL)
  {
    robot=this->GetRobotProbeNavManager()->GetRobotNode();
  }
  vtkRobotProbeNavTargetDescriptor *targetDesc = mrmlNode->GetTargetDescriptorAtIndex(mrmlNode->GetCurrentTargetIndex()); 
  NeedleDescriptorStruct *needle = mrmlNode->GetNeedle(targetDesc); 

  // Display information about the currently selected target descriptor    
  if (this->Message)
  {        
    if (robot!=NULL && targetDesc!=NULL && needle!=NULL)
    {
      // Get target info text then split it to remove the separator
      std::string info=robot->GetTargetInfoText(targetDesc, needle);
      std::string mainInfo;
      std::string additionalInfo;
      robot->SplitTargetInfoText(info, mainInfo, additionalInfo);
      std::string displayedInfo=mainInfo+additionalInfo;
      this->Message->SetText(displayedInfo.c_str());
    }
    else
    {
      // no target info available for the current robot with the current target
      //this->Message->SetText("");
      // 1/18/2012 ayamada
      //this->Message->SetBackgroundColor(0.7, 0.0, 0.0);
    }

  }


  vtkKWMatrixWidget* needlePosMatrix = NULL;
  if (this->NeedlePositionMatrix!=NULL)
  {
    needlePosMatrix=this->NeedlePositionMatrix->GetWidget();
  }
  vtkKWMatrixWidget* needleOrientationMatrix = NULL;
  if (this->ShowTargetOrientation && this->NeedlePositionMatrix!=NULL)
  {
    needleOrientationMatrix=this->NeedleOrientationMatrix->GetWidget();
  }
  if (targetDesc!=NULL)
  {
    // Copy the values to inputs         
    if (needlePosMatrix!=NULL)
    {
      double* xyz=targetDesc->GetRASLocation();
      needlePosMatrix->SetElementValueAsDouble(0, 0, xyz[0]);
      needlePosMatrix->SetElementValueAsDouble(0, 1, xyz[1]);
      needlePosMatrix->SetElementValueAsDouble(0, 2, xyz[2]);
    }
    if (this->NeedleOrientationMatrix!=NULL)
    {
      double* wxyz=targetDesc->GetRASOrientation();
      needlePosMatrix->SetElementValueAsDouble(0, 0, wxyz[0]);
      needlePosMatrix->SetElementValueAsDouble(0, 1, wxyz[1]);
      needlePosMatrix->SetElementValueAsDouble(0, 2, wxyz[2]);
      needlePosMatrix->SetElementValueAsDouble(0, 3, wxyz[3]);
    }
  }
  else
  {
    if (needlePosMatrix!=NULL)
    {
      needlePosMatrix->SetElementValue(0, 0, "");
      needlePosMatrix->SetElementValue(0, 1, "");
      needlePosMatrix->SetElementValue(0, 2, "");
    }
    if (this->NeedleOrientationMatrix!=NULL)
    {
      needlePosMatrix->SetElementValue(0, 0, "");
      needlePosMatrix->SetElementValue(0, 1, "");
      needlePosMatrix->SetElementValue(0, 2, "");
      needlePosMatrix->SetElementValue(0, 3, "");
    }
  }

  UpdateTargetListGUI();

  if (this->NeedleTypeMenuList!=NULL && this->NeedleTypeMenuList->GetWidget()!=NULL)
    {
    this->NeedleTypeMenuList->GetWidget()->GetMenu()->DeleteAllItems();
    for (int i = 0; i < mrmlNode->GetNumberOfNeedles(); i++)
      {
      NeedleDescriptorStruct needleDesc;
      mrmlNode->GetNeedle(i, needleDesc);
      std::ostrstream needleTitle;
      needleTitle << needleDesc.mDescription << " <" << needleDesc.mTargetNamePrefix <<"> ("
        <<needleDesc.GetOvershoot()<<"mm overshoot, "
        <<needleDesc.mLength<<"mm length"
        << ")" << std::ends;      
      this->NeedleTypeMenuList->GetWidget()->GetMenu()->AddRadioButton(needleTitle.str());
      needleTitle.rdbuf()->freeze();
      needleTitle.clear();
      }
    int needleIndex=mrmlNode->GetCurrentNeedleIndex();
    this->NeedleTypeMenuList->GetWidget()->GetMenu()->SelectItem(needleIndex);
    }

  
  // 11/18/2011 ayamada
  /*
  if (this->ShowRobotButton &&this->ShowRobotButton->IsCreated()) 
  {
    // 9/25/2011 ayamada
    this->ShowRobotButton->SetSelectedState(IsRobotModelShown());
  }
  */
  
  
  if (this->ShowWorkspaceButton &&this->ShowWorkspaceButton->IsCreated()) 
  {
    this->ShowWorkspaceButton->SetSelectedState(IsWorkspaceModelShown());  
  }
    
  } // workPhaseStatus
    
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::HideUserInterface()
{
  TearDownGUI(); // HideUserInterface deletes the reference to the scene, so TearDownGUI shall be done before calling HideUserInterface
  Superclass::HideUserInterface();
}

//----------------------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::TearDownGUI()
{
  RemoveMRMLObservers();
  RemoveGUIObservers();
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::EnableAddTargetsOnClickButton(bool enable)
{
  if (this->GetRobotProbeNavManager()==NULL)
  {
    return;
  }
  vtkMRMLFiducialListNode* fidNode = this->GetRobotProbeNavManager()->GetTargetPlanListNode();
  GetLogic()->SetCurrentFiducialList(fidNode);
  GetLogic()->SetMouseInteractionMode( (enable) ? 
    vtkMRMLInteractionNode::Place : 
    vtkMRMLInteractionNode::ViewTransform
    );
  
  // 1/17/2012 ayamada
  if (fidNode)
  {
    fidNode->SetLocked(0);
  }
  
}

// 9/25/2011 ayamada
void vtkRobotProbeNavTargetingStep::EnableShowAllNeedlePathsButton(bool enable)
{
  if (this->GetRobotProbeNavManager()==NULL)
  {
    return;
  }
  vtkMRMLFiducialListNode* fidNode = this->GetRobotProbeNavManager()->GetTargetPlanListNode();
  GetLogic()->SetCurrentFiducialList(fidNode);
  GetLogic()->SetMouseInteractionMode( (enable) ? 
                                      vtkMRMLInteractionNode::Place : 
                                      vtkMRMLInteractionNode::ViewTransform
                                      );
}

// 11/18/2011 ayamada
void vtkRobotProbeNavTargetingStep::EnableShowRobotButton(bool enable)
{
  if (this->GetRobotProbeNavManager()==NULL)
  {
    return;
  }
  vtkMRMLFiducialListNode* fidNode = this->GetRobotProbeNavManager()->GetTargetPlanListNode();
  GetLogic()->SetCurrentFiducialList(fidNode);
  GetLogic()->SetMouseInteractionMode( (enable) ? 
                                      vtkMRMLInteractionNode::Place : 
                                      vtkMRMLInteractionNode::ViewTransform
                                      );
}



//----------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::SetShowTargetOrientation(int show)
{
  if (this->TargetList)
  {
    vtkErrorMacro("ShowTargetOrientation cannot be changed after the GUI has been built");
    return;
  }
  this->ShowTargetOrientation = show;  
}


//---------------------------------------------------------------------------
void vtkRobotProbeNavTargetingStep::ProcessTimerEvents()
{
  if (this->TimerFlag){
    
    
    // 1/26/2012 ayamada
    //std::cout << "timerevent!!" << endl;

    // received message from Robot

    if(this->GetRobotProbeNavManager()->GetRobotNode()->GetMarkersPosition2TransformId())
    {
      //std::cerr << "timer process is working now!!!!!"  << std::endl;

      vtkMRMLNode* node = this->GetLogic()->GetApplicationLogic()->GetMRMLScene()->GetNodeByID(this->GetRobotProbeNavManager()->GetRobotNode()->GetMarkersPosition2TransformId());
      vtkMRMLLinearTransformNode* transformNode = vtkMRMLLinearTransformNode::SafeDownCast(node); //this->GetRobotProbeNavManager()->GetNeedlePathTransformNode1()
      vtkMatrix4x4* matrix = transformNode->GetMatrixTransformToParent();
      //igtl::Matrix4x4 igtlmatrix;

      // 7/20/2012 ayamada
      vtkMRMLNode* RobotPartNode1 = this->GetLogic()->GetApplicationLogic()->GetMRMLScene()->GetNodeByID(this->GetRobotProbeNavManager()->GetRobotNode()->GetIGTRobotTransform4ID());
      vtkMRMLLinearTransformNode* RobotPart1TransformNode = vtkMRMLLinearTransformNode::SafeDownCast(RobotPartNode1); 
      vtkMatrix4x4* RobotPart1Matrix = RobotPart1TransformNode->GetMatrixTransformToParent();

      // 7/22/2012 ayamada
      vtkMRMLNode* RobotPartNode2 = this->GetLogic()->GetApplicationLogic()->GetMRMLScene()->GetNodeByID(this->GetRobotProbeNavManager()->GetRobotNode()->GetIGTRobotTransform5ID());
      vtkMRMLLinearTransformNode* RobotPart2TransformNode = vtkMRMLLinearTransformNode::SafeDownCast(RobotPartNode2); 
      vtkMatrix4x4* RobotPart2Matrix = RobotPart2TransformNode->GetMatrixTransformToParent();
      
      // 7/22/2012 ayamada
      vtkMatrix4x4* RobotPartTmp1Matrix = vtkMatrix4x4::New();
      RobotPartTmp1Matrix->Identity(); 

      vtkMatrix4x4* RobotPartTmp2Matrix = vtkMatrix4x4::New();
      RobotPartTmp2Matrix->Identity(); 

      vtkMatrix4x4* RobotPartTmp3Matrix = vtkMatrix4x4::New();
      RobotPartTmp3Matrix->Identity(); 
      // ------------------------------------
      
      // 8/20/2012 ayamada
      // Update robot matrix
      // Since the rotational direction for each angle of kinematics is opposite for the display angle, minus is added for each angle.
      
      // for hooper posture
      float phi = 0.0;
      float theta = 0.0;
      // 10/2/2012 ayamada
      //float psy = -1.0*(matrix->GetElement(0,3))/180.0*PI4matrix;
      float psy = 1.0*(matrix->GetElement(0,3))/180.0*PI4matrix;

      // for needle grove posture
      float phi2 = 0.0;
      //float theta2 = 1.0*(matrix->GetElement(1,3))/180.0*PI4matrix;
      // 10/3/2012 ayamada
      float theta2 = -1.0*(matrix->GetElement(1,3))/180.0*PI4matrix;
      float psy2 = 0.0;

      // for rotation after input
      float phi3 = 0.0;
      float theta3 = 0.0;
      float psy3 = 0.0;
      
    
      // for hooper kinematics
      RobotPart1Matrix->SetElement(0,0,(float)(cos(phi)*cos(theta)));
      RobotPart1Matrix->SetElement(0,1,(float)(cos(phi)*sin(theta)*sin(psy)-sin(phi)*cos(psy)));
      RobotPart1Matrix->SetElement(0,2,(float)(cos(phi)*sin(theta)*cos(psy)+sin(phi)*sin(psy)));
      RobotPart1Matrix->SetElement(0,3,(float)(0.0*RobotPart1Matrix->GetElement(0,1)));
      
      RobotPart1Matrix->SetElement(1,0,(float)(sin(phi)*cos(theta)));
      RobotPart1Matrix->SetElement(1,1,(float)(sin(phi)*sin(theta)*sin(psy)+cos(phi)*cos(psy)));
      RobotPart1Matrix->SetElement(1,2,(float)(sin(phi)*sin(theta)*cos(psy)-cos(phi)*sin(psy)));
      RobotPart1Matrix->SetElement(1,3,(float)(0.0*RobotPart1Matrix->GetElement(1,1)));
      
      RobotPart1Matrix->SetElement(2,0,(float)(-sin(theta)));
      RobotPart1Matrix->SetElement(2,1,(float)(cos(theta)*sin(psy)));
      RobotPart1Matrix->SetElement(2,2,(float)(cos(theta)*cos(psy)));
      RobotPart1Matrix->SetElement(2,3,(float)(0.0*RobotPart1Matrix->GetElement(2,1)));

      RobotPart1Matrix->SetElement(3,0,0.0);
      RobotPart1Matrix->SetElement(3,1,0.0);
      RobotPart1Matrix->SetElement(3,2,0.0);
      RobotPart1Matrix->SetElement(3,3,1.0);

      // for grove kinematics
      RobotPartTmp2Matrix->SetElement(0,0,(float)(cos(phi2)*cos(theta2)));
      RobotPartTmp2Matrix->SetElement(0,1,(float)(cos(phi2)*sin(theta2)*sin(psy2)-sin(phi2)*cos(psy2)));
      RobotPartTmp2Matrix->SetElement(0,2,(float)(cos(phi2)*sin(theta2)*cos(psy2)+sin(phi2)*sin(psy2)));
      RobotPartTmp2Matrix->SetElement(0,3,(float)(0.0*RobotPartTmp2Matrix->GetElement(0,1)));
      
      RobotPartTmp2Matrix->SetElement(1,0,(float)(sin(phi2)*cos(theta2)));
      RobotPartTmp2Matrix->SetElement(1,1,(float)(sin(phi2)*sin(theta2)*sin(psy2)+cos(phi2)*cos(psy2)));
      RobotPartTmp2Matrix->SetElement(1,2,(float)(sin(phi2)*sin(theta2)*cos(psy2)-cos(phi2)*sin(psy2)));
      RobotPartTmp2Matrix->SetElement(1,3,(float)(0.0*RobotPartTmp2Matrix->GetElement(1,1)));
      
      RobotPartTmp2Matrix->SetElement(2,0,(float)(-sin(theta2)));
      RobotPartTmp2Matrix->SetElement(2,1,(float)(cos(theta2)*sin(psy2)));
      RobotPartTmp2Matrix->SetElement(2,2,(float)(cos(theta2)*cos(psy2)));
      RobotPartTmp2Matrix->SetElement(2,3,(float)(0.0*RobotPartTmp2Matrix->GetElement(2,1)));
      
      RobotPartTmp2Matrix->SetElement(3,0,0.0);
      RobotPartTmp2Matrix->SetElement(3,1,0.0);
      RobotPartTmp2Matrix->SetElement(3,2,0.0);
      RobotPartTmp2Matrix->SetElement(3,3,1.0);

      
      RobotPartTmp3Matrix->SetElement(0,0,(float)(cos(phi3)*cos(theta3)));
      RobotPartTmp3Matrix->SetElement(0,1,(float)(cos(phi3)*sin(theta3)*sin(psy3)-sin(phi3)*cos(psy3)));
      RobotPartTmp3Matrix->SetElement(0,2,(float)(cos(phi3)*sin(theta3)*cos(psy3)+sin(phi3)*sin(psy3)));
      RobotPartTmp3Matrix->SetElement(0,3,0.0);
      
      RobotPartTmp3Matrix->SetElement(1,0,(float)(sin(phi3)*cos(theta3)));
      RobotPartTmp3Matrix->SetElement(1,1,(float)(sin(phi3)*sin(theta3)*sin(psy3)+cos(phi3)*cos(psy3)));
      RobotPartTmp3Matrix->SetElement(1,2,(float)(sin(phi3)*sin(theta3)*cos(psy3)-cos(phi3)*sin(psy3)));
      RobotPartTmp3Matrix->SetElement(1,3,0.0);
      
      RobotPartTmp3Matrix->SetElement(2,0,(float)(-sin(theta3)));
      RobotPartTmp3Matrix->SetElement(2,1,(float)(cos(theta3)*sin(psy3)));
      RobotPartTmp3Matrix->SetElement(2,2,(float)(cos(theta3)*cos(psy3)));
      RobotPartTmp3Matrix->SetElement(2,3,0.0);
      
      RobotPartTmp3Matrix->SetElement(3,0,0.0);
      RobotPartTmp3Matrix->SetElement(3,1,0.0);
      RobotPartTmp3Matrix->SetElement(3,2,0.0);
      RobotPartTmp3Matrix->SetElement(3,3,1.0);
      
      // 7/22/2012 ayamada
      //vtkMatrix4x4::Multiply4x4(RobotPartTmp2Matrix, RobotPartTmp3Matrix, RobotPartTmp1Matrix);
      vtkMatrix4x4::Multiply4x4(RobotPart1Matrix, RobotPartTmp2Matrix, RobotPartTmp1Matrix);
      
      for(int i = 0; i<=3; i++)
      {
        for(int j = 0; j<=3; j++)
        {
          RobotPart2Matrix->SetElement(i,j,RobotPartTmp1Matrix->GetElement(i,j)); 
        }
      }
      
      
      if(matrix->GetElement(0,0) != 1 && matrix->GetElement(0,0) != 10){
      
      // send message from Slicer
      //this->GetRobotProbeNavManager()->GetRobotNode()->MoveTo(transformNode->GetID());
      if (this->Logic && this->NeedlePositionMatrix)
      {
        float position[3]={0,0,0};   // position parameters
        float orientation[4]={1,0,0,0}; // orientation parameters
        
        
        vtkMRMLNode* node = this->GetLogic()->GetApplicationLogic()->GetMRMLScene()->GetNodeByID(this->GetRobotProbeNavManager()->GetRobotNode()->GetMarkersPositionTransformId());
        vtkMRMLLinearTransformNode* transformNodeOut = vtkMRMLLinearTransformNode::SafeDownCast(node); //this->GetRobotProbeNavManager()->GetNeedlePathTransformNode1()
                
      }
      }
      
      // 7/17/2012 ayamada
      if(matrix->GetElement(2,3) == 2)
      {
        // 8/19/2012 ayamada
        this->RobotControllerStatus = 0;
        
        if(this->robotStatusSwitcher == 0)
        {
          if(this->robotStatus == 0)
          {
            this->Message2->SetText("Robot: MOVING..\nTarget: Tgts-P (Center Groove)");
            this->Message2->SetBackgroundColor(0.80, 0.70, 0.0);              
          }else if(this->robotStatus == 1)
          {
            this->Message2->SetText("Robot: MOVING..\nTarget: Tgts-P1 (Left Groove)");            
            this->Message2->SetBackgroundColor(0.80, 0.70, 0.0);              
          }else if(this->robotStatus == 2)
          {
            this->Message2->SetText("Robot: MOVING..\nTarget: Tgts-P2 (Right Groove)");            
            this->Message2->SetBackgroundColor(0.80, 0.70, 0.0);              
          }
          this->robotStatusSwitcher = 1;
          
        }else{
          if(this->robotStatus == 0)
          {
            this->Message2->SetText("Robot: MOVING....\nTarget: Tgts-P (Center Groove)");
            this->Message2->SetBackgroundColor(0.80, 0.80, 0.0);              
          }else if(this->robotStatus == 1)
          {
            this->Message2->SetText("Robot: MOVING....\nTarget: Tgts-P1 (Left Groove)");            
            this->Message2->SetBackgroundColor(0.80, 0.80, 0.0);              
          }else if(this->robotStatus == 2)
          {
            this->Message2->SetText("Robot: MOVING....\nTarget: Tgts-P2 (Right Groove)");            
            this->Message2->SetBackgroundColor(0.80, 0.80, 0.0);              
          }
          this->robotStatusSwitcher = 0;
        }
        //this->Message2->SetBackgroundColor(0.80, 0.80, 0.0);              
      }else if(matrix->GetElement(2,3) == 1 || this->RobotControllerStatus == 1)
      {
        if(this->commandStatus == 1)
        {
        this->Message2->SetText("Robot: NO MOVING");
        this->Message2->SetBackgroundColor(0.80, 0.0, 0.20);
        }else{
        this->Message2->SetText("Robot: READY");
        //this->Message2->SetBackgroundColor(0.5, 0.5, 0.95);          
        this->Message2->SetBackgroundColor(0.4, 0.8, 0.0); 
        }
      }else if(matrix->GetElement(2,3) == 3 && this->RobotControllerStatus == 0)
      {
        this->Message2->SetText("Robot: MOVING WAS FINISHED");      
        this->Message2->SetBackgroundColor(0.40, 0.80, 0.0);
        this->commandStatus = 0;
        
        // 8/19/2012 ayamada
        this->RobotControllerStatus = 1;
        
        // 2/24/2012 ayamada
        // finish signal update
        this->TargetList->GetWidget()->SetCellText(this->robotStatus,STATUS,"Finished");  
        
        
      }
      // 8/19/2012 ayamada
      if(this->stopSignal == 10/*matrix->GetElement(2,3) == 9*/)
        // 1/27/2012 ayamada
      {
        this->Message2->SetText("Robot: EMERGENCY STOPPED ");      
        this->Message2->SetBackgroundColor(0.40, 0.80, 0.0);
        this->commandStatus = 0;  
        this->stopSignal = 0;
      }
      
    }
    
    //std::cerr << "timer is working now!!!!!"  << std::endl;
    
    //----
    // update timer
    vtkKWTkUtilities::CreateTimerHandler(vtkKWApplication::GetMainInterp(),
                                         this->TimerInterval,
                                         this, "ProcessTimerEvents");
    
  }
}

