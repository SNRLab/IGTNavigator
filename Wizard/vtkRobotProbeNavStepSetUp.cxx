/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/

#include "vtkRobotProbeNavStepSetUp.h"

#include "vtkRobotProbeNavGUI.h"
#include "vtkRobotProbeNavLogic.h"

#include "vtkKWFrame.h"
#include "vtkKWFrameWithLabel.h"
#include "vtkKWLabel.h"
#include "vtkKWEntry.h"
#include "vtkKWCheckButton.h"
#include "vtkKWWizardWidget.h"
#include "vtkKWWizardWorkflow.h"
#include "vtkKWPushButton.h"
#include "vtkKWLoadSaveButton.h"
#include "vtkKWLoadSaveButtonWithLabel.h"

#include "vtkSlicerNodeSelectorWidget.h"

#include "vtkOpenIGTLinkIFLogic.h"
#include "vtkOpenIGTLinkIFGUI.h"

#include "vtkMRMLRobotNode.h"
#include "vtkMRMLIGTProbeRobotNode.h"

// 8/16/2011 ayamada
//#include "vtkCornerAnnotation.h"

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkRobotProbeNavStepSetUp);
vtkCxxRevisionMacro(vtkRobotProbeNavStepSetUp, "$Revision: 1.1 $");

//----------------------------------------------------------------------------
vtkRobotProbeNavStepSetUp::vtkRobotProbeNavStepSetUp()
{

  //this->SetName("Configuration");
  this->SetTitle("Configuration");
  this->SetDescription("System configuration.");

  this->ConnectorFrame                  = NULL;
  this->RobotConnectorSelector          = NULL;
  this->ScannerConnectorSelector        = NULL;
  
  // 12/28/2011 ayamadad
  this->SelectConfigurationFileButton    = NULL;
  this->AddConfigurationFilePushButton   = NULL;
  
  this->FiducialNumber = 1;
  //this->PointPairMultiColumnList        = NULL;
  
}


//----------------------------------------------------------------------------
vtkRobotProbeNavStepSetUp::~vtkRobotProbeNavStepSetUp()
{
  
  // 12/28/2011 ayamada
  
  this->FiducialNumber = NULL;
  /*
  if (this->PointPairMultiColumnList)
  {
    this->PointPairMultiColumnList->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->PointPairMultiColumnList->SetParent(NULL );
    this->PointPairMultiColumnList->Delete ( );
    this->PointPairMultiColumnList = NULL;
  }  
  */
  // Connectors
  if (this->ConnectorFrame)
    {
    this->ConnectorFrame->SetParent(NULL);
    this->ConnectorFrame->Delete();
    this->ConnectorFrame = NULL;
    }

  if (this->RobotConnectorSelector)
    {
    this->RobotConnectorSelector->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->RobotConnectorSelector->SetParent(NULL);
    this->RobotConnectorSelector->Delete();
    this->RobotConnectorSelector = NULL;
    }
  if (this->ScannerConnectorSelector)
    {
    this->ScannerConnectorSelector->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->ScannerConnectorSelector->SetParent(NULL);
    this->ScannerConnectorSelector->Delete();
    this->ScannerConnectorSelector = NULL;
    }

  // 12/28/2011 ayamadad
  if (this->SelectConfigurationFileButton)
  {
    this->SelectConfigurationFileButton->SetParent(NULL);
    this->SelectConfigurationFileButton->Delete();
  }
  if (this->AddConfigurationFilePushButton)
  {
    this->AddConfigurationFilePushButton->RemoveObserver((vtkCommand *)this->GUICallbackCommand);
    this->AddConfigurationFilePushButton->SetParent(NULL );
    this->AddConfigurationFilePushButton->Delete ( );
    this->AddConfigurationFilePushButton = NULL;
  }

}


//----------------------------------------------------------------------------
void vtkRobotProbeNavStepSetUp::ShowUserInterface()
{

  this->Superclass::ShowUserInterface();
  vtkKWWizardWidget *wizardWidget = this->GetGUI()->GetWizardWidget();
  vtkKWWidget *parent = wizardWidget->GetClientArea();

  // Connector Frame

  if (!this->ConnectorFrame)
    {
    this->ConnectorFrame = vtkKWFrame::New();
    this->ConnectorFrame->SetParent ( parent );
    this->ConnectorFrame->Create ( );
    }

  this->Script ( "pack %s -side top -fill x",
                 this->ConnectorFrame->GetWidgetName());
  
  
  // 12/28/2011 ayamada
  if (!this->SelectConfigurationFileButton)
  {
  this->SelectConfigurationFileButton = vtkKWLoadSaveButtonWithLabel::New();
  this->SelectConfigurationFileButton->SetParent(this->ConnectorFrame);
  this->SelectConfigurationFileButton->Create();
  this->SelectConfigurationFileButton->GetWidget()->SetText ("Robot Configuration File");
  this->SelectConfigurationFileButton->SetWidth(30);
  //this->SelectConfigurationFileButton->SetLabelWidth(0);
  this->SelectConfigurationFileButton->GetWidget()->GetLoadSaveDialog()->SetFileTypes("{ {RobotProbeNav} {*.csv} }");
  this->SelectConfigurationFileButton->GetWidget()->GetLoadSaveDialog()
  ->RetrieveLastPathFromRegistry("OpenPath");
  
  this->SelectConfigurationFileButton->SetEnabled(1);
  
  // 9/8/2011 ayamada    
  this->AddConfigurationFilePushButton = vtkKWPushButton::New();
  this->AddConfigurationFilePushButton->SetParent(this->ConnectorFrame);
  this->AddConfigurationFilePushButton->Create();
  this->AddConfigurationFilePushButton->SetText( "OK" );
  this->AddConfigurationFilePushButton->SetWidth( 12 );
  
  this->AddConfigurationFilePushButton->AddObserver ( vtkKWPushButton::InvokedEvent, 
                                                (vtkCommand *)this->GUICallbackCommand );
  
  
  //  this->Script("pack %s -side top -anchor w -fill x -padx 2 -pady 2", 
  
  // 9/16/2012 ayamada
  //this->Script("pack %s %s -side top -anchor nw -fill x -padx 2 -pady 2", 
  //             this->SelectConfigurationFileButton->GetWidgetName(),
  //             this->AddConfigurationFilePushButton->GetWidgetName());
  }
  
  

  if (!this->RobotConnectorSelector)
    {
    this->RobotConnectorSelector = vtkSlicerNodeSelectorWidget::New() ;
    this->RobotConnectorSelector->SetParent(this->ConnectorFrame);
    this->RobotConnectorSelector->Create();
    this->RobotConnectorSelector->SetNodeClass("vtkMRMLIGTLConnectorNode", NULL, NULL, "RobotConnector");
    this->RobotConnectorSelector->SetMRMLScene(this->MRMLScene);
    this->RobotConnectorSelector->SetBorderWidth(2);
    this->RobotConnectorSelector->GetWidget()->GetWidget()->IndicatorVisibilityOff();
    this->RobotConnectorSelector->GetWidget()->GetWidget()->SetWidth(24);
    this->RobotConnectorSelector->SetNoneEnabled(1);
    this->RobotConnectorSelector->SetNewNodeEnabled(1);
    this->RobotConnectorSelector->SetLabelText( "Robot connector: ");
    this->RobotConnectorSelector->SetBalloonHelpString("Select or create a target list.");
    this->RobotConnectorSelector->ExpandWidgetOff();
    this->RobotConnectorSelector->SetLabelWidth(22);

    this->RobotConnectorSelector
      ->AddObserver(vtkSlicerNodeSelectorWidget::NodeSelectedEvent,
                    (vtkCommand *)this->GUICallbackCommand );
    this->RobotConnectorSelector
      ->AddObserver(vtkSlicerNodeSelectorWidget::NewNodeEvent,
                    (vtkCommand *)this->GUICallbackCommand );
    }    

  if (!this->ScannerConnectorSelector)
    {
    this->ScannerConnectorSelector = vtkSlicerNodeSelectorWidget::New() ;
    this->ScannerConnectorSelector->SetParent(this->ConnectorFrame);
    this->ScannerConnectorSelector->Create();
    this->ScannerConnectorSelector->SetNodeClass("vtkMRMLIGTLConnectorNode", NULL, NULL, "ScannerConnector");
    this->ScannerConnectorSelector->SetMRMLScene(this->MRMLScene);
    this->ScannerConnectorSelector->SetBorderWidth(2);
    this->ScannerConnectorSelector->GetWidget()->GetWidget()->IndicatorVisibilityOff();
    this->ScannerConnectorSelector->GetWidget()->GetWidget()->SetWidth(24);
    this->ScannerConnectorSelector->SetNoneEnabled(1);
    this->ScannerConnectorSelector->SetNewNodeEnabled(1);
    //this->ScannerConnectorSelector->SetLabelText( "Scanner Connector: ");
    // 1/17/2012 ayamada
    this->ScannerConnectorSelector->SetLabelText( "Robot Status Connector: ");
    this->ScannerConnectorSelector->SetBalloonHelpString("Select or create a target list.");
    this->ScannerConnectorSelector->ExpandWidgetOff();
    this->ScannerConnectorSelector->SetLabelWidth(22);

    this->ScannerConnectorSelector
      ->AddObserver(vtkSlicerNodeSelectorWidget::NodeSelectedEvent,
                    (vtkCommand *)this->GUICallbackCommand );
    this->ScannerConnectorSelector
      ->AddObserver(vtkSlicerNodeSelectorWidget::NewNodeEvent,
                    (vtkCommand *)this->GUICallbackCommand );
    }    

  // 9/16/2012 ayamada
  //this->Script("pack %s %s -side top -anchor nw -fill x -padx 2 -pady 2",
  //             this->RobotConnectorSelector->GetWidgetName(),
  //             this->ScannerConnectorSelector->GetWidgetName());
  this->Script("pack %s -side top -anchor nw -fill x -padx 2 -pady 2",
               this->RobotConnectorSelector->GetWidgetName());
  
    
}


//----------------------------------------------------------------------------
void vtkRobotProbeNavStepSetUp::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavStepSetUp::ProcessGUIEvents( vtkObject *caller,
                                         unsigned long event, void *callData )
{

  vtkMRMLIGTProbeRobotNode* robotNode=GetRobotNode();

  if (this->RobotConnectorSelector == vtkSlicerNodeSelectorWidget::SafeDownCast(caller)
      && (event == vtkSlicerNodeSelectorWidget::NewNodeEvent || event == vtkSlicerNodeSelectorWidget::NodeSelectedEvent))
    {
    vtkMRMLIGTLConnectorNode* node = vtkMRMLIGTLConnectorNode::SafeDownCast(this->RobotConnectorSelector->GetSelected());
    if (robotNode && node)
      {      
      vtksys_stl::string moduleShareDir=this->GetLogic()->GetModuleShareDirectory();
      robotNode->Init(vtkSlicerApplication::SafeDownCast(this->GetApplication()),moduleShareDir.c_str()); // :TODO: this may not be the best place for robot initialization (e.g., when scene is loaded from MRML the GUI will not be used)    
      robotNode->SetAndObserveRobotConnectorNodeID(node->GetID());
      }
    }

  if (this->ScannerConnectorSelector == vtkSlicerNodeSelectorWidget::SafeDownCast(caller)
      && (event == vtkSlicerNodeSelectorWidget::NewNodeEvent || event == vtkSlicerNodeSelectorWidget::NodeSelectedEvent))
    {
    vtkMRMLIGTLConnectorNode* node = vtkMRMLIGTLConnectorNode::SafeDownCast(this->ScannerConnectorSelector->GetSelected());
    if (robotNode && node)
      {
      robotNode->SetAndObserveScannerConnectorNodeID(node->GetID());
      }
    }  
  
  
  // 12/28/2011 ayamada
  // process GUI event
  // 9/8/2011 ayamada: read CSV file and list the data
  if (this->AddConfigurationFilePushButton == vtkKWPushButton::SafeDownCast(caller) 
      && event == vtkKWPushButton::InvokedEvent)
  {
    
    const char * path = this->SelectConfigurationFileButton->GetWidget()->GetFileName();
    if (path)
    {
      std::cerr << "The file path was set." << std::endl;
      
      // 12/28/2011 ayamada
      //int numberOfData = this->LoadCSVTrackingFile(path);      
      int numberOfData = this->GetGUI()->LoadCSVTrackingFile2(path);      
      
      std::cerr << "The file path was set2" << std::endl;
      
      if (!numberOfData)
      {
        std::cerr << "The CVS file was not able to be read." << std::endl;
      }else{
        
        std::cerr << "The file path was set3" << std::endl;
        
        int scSize = 0;
        //int pcSize = 0;
        char str[32];
        char str2[128];
        const char *pc = NULL;
        const char *sc = NULL;//this->FiducialMarkersSetup->GetWidget()->GetValue();
        
        std::cerr << "The file path was set4" << std::endl;        
        
        for(int i = 0; i<numberOfData; i++)
        {
          
          std::cerr << "The file path was set5" << std::endl;
          
          sprintf(str, "%d", this->FiducialNumber);
          //sprintf(str2, "%.2f %.2f %.2f", this->pX[i],this->pY[i],this->pZ[i]);
          sprintf(str2, "%.2f %.2f %.2f", this->GetGUI()->cX[i],this->GetGUI()->cY[i],this->GetGUI()->cZ[i]);
          pc = str;
          sc = str2;
    
          std::cerr << "StepSetUp: GetGUI()->cX[i] = " << this->GetGUI()->cX[i] << std::endl; 
          std::cerr << "StepSetUp: GetGUI()->cX[i] = " << this->GetGUI()->cY[i] << std::endl; 
          std::cerr << "StepSetUp: GetGUI()->cX[i] = " << this->GetGUI()->cZ[i] << std::endl; 
          
          std::cerr << "The file path was set6" << std::endl;
          
          //int row = this->PointPairMultiColumnList->GetWidget()->GetNumberOfRows();
          //this->PointPairMultiColumnList->GetWidget()->AddRow();
          //this->PointPairMultiColumnList->GetWidget()->SetCellText(row, 0, pc);
          // 9/8/2011 ayamada
          //this->PointPairMultiColumnList->GetWidget()->SetCellText(row, 1, sc);
          
          // increment of firucial marker numbers
          this->FiducialNumber++;
          //this->FiducialNumber2++;
          std::cerr << "The file path was set11" << std::endl;
          
          // 9/8/2011 ayamada
          //int row2 = this->PointPairMultiColumnList2->GetWidget()->GetNumberOfRows();
          //this->PointPairMultiColumnList2->GetWidget()->AddRow();
          //this->PointPairMultiColumnList2->GetWidget()->SetCellText(row, 0, pc);
          
          
        }
        
      }
      
    }
    
  // 12/28/2011 ayamada
  //this->GetLogic()->calculateVirtualCenterPosition();
    
    
  }
  
  
  
  

}

vtkMRMLIGTProbeRobotNode* vtkRobotProbeNavStepSetUp::GetRobotNode()
{
  if (this->RobotProbeNavManager==NULL)
  {
    return NULL;
  }
  vtkMRMLIGTProbeRobotNode* robotNode = vtkMRMLIGTProbeRobotNode::SafeDownCast(this->RobotProbeNavManager->GetRobotNode());
  return robotNode;
}

