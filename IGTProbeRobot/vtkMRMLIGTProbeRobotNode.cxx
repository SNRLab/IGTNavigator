/*=auto=========================================================================

Portions (c) Copyright 2005 Brigham and Women's Hospital (BWH) All Rights Reserved.

See Doc/copyright/copyright.txt
or http://www.slicer.org/copyright/copyright.txt for details.

Program:   3D Slicer
Module:    $RCSfile: vtkMRMLIGTProbeRobotNode.cxx,v $
Date:      $Date: 2006/03/17 15:10:10 $
Version:   $Revision: 1.2 $

=========================================================================auto=*/

#include "vtkObjectFactory.h"

#include "vtkMRMLIGTProbeRobotNode.h"
#include "vtkMRMLScene.h"

#include "vtkOpenIGTLinkIFGUI.h"
#include "vtkOpenIGTLinkIFLogic.h"
#include "vtkIGTLToMRMLCoordinate.h"
#include "vtkIGTLToMRMLBrpRobotCommand.h"

#include "vtkMRMLIGTLConnectorNode.h"
#include "vtkMRMLBrpRobotCommandNode.h"

#include "vtkZFrameRobotToImageRegistration.h"

#include "vtkRobotProbeNavTargetDescriptor.h"

#include "vtkTriangleFilter.h"


// 9/22/2011 ayamada
#include "vtkSTLReader.h"

// 11/3/2011 ayamada
#include "vtkSphereSource.h"

// 11/4/2011 ayamada
#include "vtkRobotProbeNavLogic.h"


//-------------------------------------

static const unsigned int STATUS_ROBOT=0;
static const unsigned int STATUS_SCANNER=1;

static const char STATUS_ROBOT_OFF[]="Robot: OFF";
static const char STATUS_ROBOT_ON[]="Robot: ON";

static const char STATUS_SCANNER_OFF[]="Scanner: OFF";
static const char STATUS_SCANNER_ON[]="Scanner: ON";

//------------------------------------------------------------------------------
vtkMRMLIGTProbeRobotNode* vtkMRMLIGTProbeRobotNode::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkMRMLIGTProbeRobotNode");
  if(ret)
    {
      return (vtkMRMLIGTProbeRobotNode*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkMRMLIGTProbeRobotNode;
}

//----------------------------------------------------------------------------
vtkMRMLIGTProbeRobotNode* vtkMRMLIGTProbeRobotNode::CreateNodeInstance()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkMRMLIGTProbeRobotNode");
  if(ret)
    {
      return (vtkMRMLIGTProbeRobotNode*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkMRMLIGTProbeRobotNode;
}

//----------------------------------------------------------------------------
vtkMRMLIGTProbeRobotNode::vtkMRMLIGTProbeRobotNode()
{
  // Node references

  this->RobotCommandNodeID=NULL;
  this->RobotCommandNode=NULL;

  this->RobotConnectorNodeID=NULL;
  this->RobotConnectorNode=NULL;

  this->ScannerConnectorNodeID=NULL;
  this->ScannerConnectorNode=NULL;

  this->ZFrameModelNodeID=NULL;
  this->ZFrameModelNode=NULL;

  // 9/22/2011 ayamada
  this->MITRobotModelNodeID=NULL;
  this->MITRobotModelNode=NULL;

  // 9/23/2011 ayamada
  this->MITRobotModelNode2ID=NULL;
  this->MITRobotModelNode2=NULL;
  this->MITRobotModelNode3ID=NULL;
  this->MITRobotModelNode3=NULL;
  this->MITRobotModelNode4ID=NULL;
  this->MITRobotModelNode4=NULL;
  this->MITRobotModelNode5ID=NULL;
  this->MITRobotModelNode5=NULL;

  // 9/24/2011 ayamada
  this->NeedleModelNodeID=NULL;
  this->NeedleModelNode=NULL;
  this->NeedleModelNode1ID=NULL;
  this->NeedleModelNode1=NULL;
  this->NeedleModelNode2ID=NULL;
  this->NeedleModelNode2=NULL;
  
  // 11/3/2011 ayamada
  this->VirtualCenterModelNodeID=NULL;
  this->VirtualCenterModelNode=NULL;

  this->VirtualCenterTransformNodeID=NULL;
  this->VirtualCenterTransformNode=NULL;
  
  
  this->ZFrameTransformNodeID=NULL;
  this->ZFrameTransformNode=NULL;  
  
  // 10/06/2011 ayamada
  this->NeedlePathTransformNode1ID=NULL;
  this->NeedlePathTransformNode1=NULL;  
  this->NeedlePathTransformNode2ID=NULL;
  this->NeedlePathTransformNode2=NULL;  
  this->NeedlePathTransformNode3ID=NULL;
  this->NeedlePathTransformNode3=NULL;  

  
  this->MITRobotTransformNode1ID=NULL;
  this->MITRobotTransformNode1=NULL;  
  this->MITRobotTransformNode2ID=NULL;
  this->MITRobotTransformNode2=NULL;  
  this->MITRobotTransformNode3ID=NULL;
  this->MITRobotTransformNode3=NULL;  
  this->MITRobotTransformNode4ID=NULL;
  this->MITRobotTransformNode4=NULL;  
  this->MITRobotTransformNode5ID=NULL;
  this->MITRobotTransformNode5=NULL;  
  // 7//21/2012 ayamada
  this->MITRobotTransformNode6ID=NULL;
  this->MITRobotTransformNode6=NULL;  
  this->MITRobotTransformNode7ID=NULL;
  this->MITRobotTransformNode7=NULL;  
  
  // 2/9/2012 ayamada
  this->TargetPlaneTransformNodeID=NULL;
  this->TargetPlaneTransformNode=NULL;
  
  // 9/13/2011 ayamada
  this->MarkersPositionTransformNodeID=NULL;
  this->MarkersPositionTransformNode=NULL;  

  this->MarkersPosition2TransformNodeID=NULL;
  this->MarkersPosition2TransformNode=NULL;  
  
  
  this->WorkspaceModelNodeID=NULL;
  this->WorkspaceModelNode=NULL;

  // Other

  this->CoordinateConverter = vtkIGTLToMRMLCoordinate::New();
  this->CommandConverter = vtkIGTLToMRMLBrpRobotCommand::New();

  this->RobotWorkPhase       = -1;
  this->ScannerWorkPhase     = -1;

  this->ScannerConnectedFlag = 0;
  this->RobotConnectedFlag   = 0;
  
  // 6/28/2012 ayamada
  this->imageFilePath = (char*)calloc(500, sizeof(char));


  StatusDescriptor s;
  s.indicator=StatusOff;
  s.text=STATUS_ROBOT_OFF;  
  this->StatusDescriptors.push_back(s); // STATUS_ROBOT=0

  s.indicator=StatusOff;
  s.text=STATUS_SCANNER_OFF;
  this->StatusDescriptors.push_back(s); // STATUS_SCANNER=1
}

//----------------------------------------------------------------------------
vtkMRMLIGTProbeRobotNode::~vtkMRMLIGTProbeRobotNode()
{
  if (this->RobotCommandNodeID) 
  {
    SetAndObserveRobotCommandNodeID(NULL);
  }
  if (this->RobotConnectorNodeID) 
  {
    SetAndObserveRobotConnectorNodeID(NULL);
  }
  if (this->ScannerConnectorNodeID) 
  {
    SetAndObserveScannerConnectorNodeID(NULL);
  }
  if (this->ZFrameModelNodeID) 
  {
    SetAndObserveZFrameModelNodeID(NULL);
  }
  
  // 9/21/2011 ayamada
  if (this->MITRobotModelNodeID) 
  {
    SetAndObserveMITRobotModelNodeID(NULL);
  }

  // 9/23/2011 ayamada
  if (this->MITRobotModelNode2ID) 
  {
    SetAndObserveMITRobotModelNode2ID(NULL);
  }
  if (this->MITRobotModelNode3ID) 
  {
    SetAndObserveMITRobotModelNode3ID(NULL);
  }
  if (this->MITRobotModelNode4ID) 
  {
    SetAndObserveMITRobotModelNode4ID(NULL);
  }
  if (this->MITRobotModelNode5ID) 
  {
    SetAndObserveMITRobotModelNode5ID(NULL);
  }

  // 9/24/2011 ayamada
  if (this->NeedleModelNodeID) 
  {
    SetAndObserveNeedleModelNodeID(NULL);
  }
  if (this->NeedleModelNode1ID) 
  {
    SetAndObserveNeedleModelNode1ID(NULL);
  }
  if (this->NeedleModelNode2ID) 
  {
    SetAndObserveNeedleModelNode2ID(NULL);
  }
  
  // 11/3/2011 ayamada
  if (this->VirtualCenterModelNodeID) 
  {
    SetAndObserveVirtualCenterModelNodeID(NULL);
  }

  if (this->VirtualCenterTransformNodeID) 
  {
    SetAndObserveVirtualCenterTransformNodeID(NULL);
  }
  
  
  if (this->ZFrameTransformNodeID) 
  {
    SetAndObserveZFrameTransformNodeID(NULL);
  }
  if (this->MarkersPositionTransformNodeID) 
  {
    SetAndObserveMarkersPositionTransformNodeID(NULL);
  }
  if (this->MarkersPosition2TransformNodeID) 
  {
    SetAndObserveMarkersPosition2TransformNodeID(NULL);
  }
  
  // 10/6/2011 ayamada
  if (this->NeedlePathTransformNode1) 
  {
    SetAndObserveNeedlePathTransformNode1ID(NULL);
  }
  if (this->NeedlePathTransformNode2) 
  {
    SetAndObserveNeedlePathTransformNode2ID(NULL);
  }
  if (this->NeedlePathTransformNode3) 
  {
    SetAndObserveNeedlePathTransformNode3ID(NULL);
  }
  
  // ------
  if (this->MITRobotTransformNode1) 
  {
    SetAndObserveMITRobotTransformNode1ID(NULL);
  }
  if (this->MITRobotTransformNode2) 
  {
    SetAndObserveMITRobotTransformNode2ID(NULL);
  }
  if (this->MITRobotTransformNode3) 
  {
    SetAndObserveMITRobotTransformNode3ID(NULL);
  }
  if (this->MITRobotTransformNode4) 
  {
    SetAndObserveMITRobotTransformNode4ID(NULL);
  }
  if (this->MITRobotTransformNode5) 
  {
    SetAndObserveMITRobotTransformNode5ID(NULL);
  }
  // 7/21/2012 ayamada
  if (this->MITRobotTransformNode6) 
  {
    SetAndObserveMITRobotTransformNode6ID(NULL);
  }
  if (this->MITRobotTransformNode7) 
  {
    SetAndObserveMITRobotTransformNode7ID(NULL);
  }
  
  // 2/9/2012 ayamada
  if (this->TargetPlaneTransformNode) 
  {
    SetAndObserveTargetPlaneTransformNodeID(NULL);
  }
  

  if (this->CoordinateConverter)
  { 
    this->CoordinateConverter->Delete();
    this->CoordinateConverter = NULL;
  }
  if (this->CommandConverter)
  {
    this->CommandConverter->Delete();
    this->CommandConverter = NULL;
  }
}


//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::WriteXML(ostream& of, int nIndent)
{
  // Start by having the superclass write its information
  Superclass::WriteXML(of, nIndent);

  vtkIndent indent(nIndent);

  if (this->RobotCommandNodeID != NULL) 
    {
    of << indent << " RobotCommandNodeRef=\"" << this->RobotCommandNodeID << "\"";
    }
  if (this->RobotConnectorNodeID != NULL) 
    {
    of << indent << " RobotConnectorNodeRef=\"" << this->RobotConnectorNodeID << "\"";
    }
  if (this->ScannerConnectorNodeID != NULL) 
    {
    of << indent << " ScannerConnectorNodeRef=\"" << this->ScannerConnectorNodeID << "\"";
    }
  if (this->ZFrameModelNodeID != NULL) 
    {
    of << indent << " ZFrameModelNodeRef=\"" << this->ZFrameModelNodeID << "\"";
    }
  if (this->ZFrameTransformNodeID != NULL) 
    {
    of << indent << " ZFrameTransformNodeRef=\"" << this->ZFrameTransformNodeID << "\"";
    }

  // 11/3/2011 ayamada
  if (this->VirtualCenterModelNodeID != NULL) 
  {
    of << indent << " VirtualCenterModelNodeRef=\"" << this->VirtualCenterModelNodeID << "\"";
  }
  if (this->VirtualCenterTransformNodeID != NULL) 
  {
    of << indent << " VirtualCenterTransformlNodeRef=\"" << this->VirtualCenterTransformNodeID << "\"";
  }
  
  
  // 10/6/2011 ayamada
  if (this->NeedlePathTransformNode1ID != NULL) 
  {
    of << indent << " GetNeedlePathTransformNode1Ref=\"" << this->NeedlePathTransformNode1ID << "\"";
  }
  if (this->NeedlePathTransformNode2ID != NULL) 
  {
    of << indent << " GetNeedlePathTransformNode2Ref=\"" << this->NeedlePathTransformNode2ID << "\"";
  }
  if (this->NeedlePathTransformNode3ID != NULL) 
  {
    of << indent << " GetNeedlePathTransformNode3Ref=\"" << this->NeedlePathTransformNode3ID << "\"";
  }
    
  if (this->MITRobotTransformNode1ID != NULL) 
  {
    of << indent << " GetMITRobotTransformNode1Ref=\"" << this->MITRobotTransformNode1ID << "\"";
  }
  if (this->MITRobotTransformNode2ID != NULL) 
  {
    of << indent << " GetMITRobotTransformNode2Ref=\"" << this->MITRobotTransformNode2ID << "\"";
  }
  if (this->MITRobotTransformNode3ID != NULL) 
  {
    of << indent << " GetMITRobotTransformNode3Ref=\"" << this->MITRobotTransformNode3ID << "\"";
  }
  if (this->MITRobotTransformNode4ID != NULL) 
  {
    of << indent << " GetMITRobotTransformNode4Ref=\"" << this->MITRobotTransformNode4ID << "\"";
  }
  if (this->MITRobotTransformNode5ID != NULL) 
  {
    of << indent << " GetMITRobotTransformNode5Ref=\"" << this->MITRobotTransformNode5ID << "\"";
  }
  // 7/21/2012 ayamada
  if (this->MITRobotTransformNode6ID != NULL) 
  {
    of << indent << " GetMITRobotTransformNode6Ref=\"" << this->MITRobotTransformNode6ID << "\"";
  }
  if (this->MITRobotTransformNode7ID != NULL) 
  {
    of << indent << " GetMITRobotTransformNode7Ref=\"" << this->MITRobotTransformNode7ID << "\"";
  }
  
  
  
  // 2/9/2012 ayamada
  if (this->TargetPlaneTransformNodeID != NULL) 
  {
    of << indent << " GetTargetPlaneTransformNodeRef=\"" << this->TargetPlaneTransformNodeID << "\"";
  }
  
  
  
  if (this->MarkersPositionTransformNodeID != NULL) 
  {
    of << indent << " MarkersPositionTransformNodeRef=\"" << this->MarkersPositionTransformNodeID << "\"";
  }
  if (this->MarkersPosition2TransformNodeID != NULL) 
  {
    of << indent << " MarkersPosition2TransformNodeRef=\"" << this->MarkersPosition2TransformNodeID << "\"";
  }
  
  //switch (this->Type)
  //  {
  //  case TYPE_SERVER:
  //    of << " connectorType=\"" << "SERVER" << "\" ";
  //    break;
  //  case TYPE_CLIENT:
  //    of << " connectorType=\"" << "CLIENT" << "\" ";
  //    of << " serverHostname=\"" << this->ServerHostname << "\" ";
  //    break;
  //  default:
  //    of << " connectorType=\"" << "NOT_DEFINED" << "\" ";
  //    break;
  //  }
  //
  //of << " serverPort=\"" << this->ServerPort << "\" ";
  //of << " restrictDeviceName=\"" << this->RestrictDeviceName << "\" ";


}


//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::ReadXMLAttributes(const char** atts)
{
  Superclass::ReadXMLAttributes(atts);

  // Read all MRML node attributes from two arrays of names and values
  const char* attName;
  const char* attValue;

  while (*atts != NULL)
    {
    attName  = *(atts++);
    attValue = *(atts++);

    if (!strcmp(attName, "RobotCommandNodeRef")) 
      {
      this->SetAndObserveRobotCommandNodeID(attValue);
      }
    if (!strcmp(attName, "RobotConnectorNodeRef")) 
      {
      this->SetAndObserveRobotConnectorNodeID(attValue);
      }
    if (!strcmp(attName, "ScannerConnectorNodeRef")) 
      {
      this->SetAndObserveScannerConnectorNodeID(attValue);
      }
    if (!strcmp(attName, "ZFrameModelNodeRef")) 
      {
      this->SetAndObserveZFrameModelNodeID(attValue);
      }

    // 9/21/2011 ayamada
    if (!strcmp(attName, "MITRobotModelNodeRef")) 
      {
        this->SetAndObserveMITRobotModelNodeID(attValue);
      }

      // 9/23/2011 ayamada
      if (!strcmp(attName, "MITRobotModelNode2Ref")) 
      {
        this->SetAndObserveMITRobotModelNode2ID(attValue);
      }
      if (!strcmp(attName, "MITRobotModelNode3Ref")) 
      {
        this->SetAndObserveMITRobotModelNode3ID(attValue);
      }
      if (!strcmp(attName, "MITRobotModelNode4Ref")) 
      {
        this->SetAndObserveMITRobotModelNode4ID(attValue);
      }
      if (!strcmp(attName, "MITRobotModelNode5Ref")) 
      {
        this->SetAndObserveMITRobotModelNode5ID(attValue);
      }

      
      // 9/24/2011 ayamada
      if (!strcmp(attName, "MITNeedleModelNodeRef")) 
      {
        this->SetAndObserveNeedleModelNodeID(attValue);
      }
      if (!strcmp(attName, "MITNeedleModelNode1Ref")) 
      {
        this->SetAndObserveNeedleModelNode1ID(attValue);
      }
      if (!strcmp(attName, "MITNeedleModelNode2Ref")) 
      {
        this->SetAndObserveNeedleModelNode2ID(attValue);
      }
      
      // 11/3/2011 ayamada
      if (!strcmp(attName, "VirtualCenterModelNodeRef")) 
      {
        this->SetAndObserveVirtualCenterModelNodeID(attValue);
      }
      if (!strcmp(attName, "VirtualCenterTransformlNodeRef")) 
      {
        this->SetAndObserveVirtualCenterTransformNodeID(attValue);
      }
      
      
      
    if (!strcmp(attName, "ZFrameTransformNodeRef")) 
      {
      this->SetAndObserveZFrameTransformNodeID(attValue);
      }

    // 10/6/2011 ayamada
    if (!strcmp(attName, "NeedlePathTransformNode1Ref")) 
      {
        this->SetAndObserveNeedlePathTransformNode1ID(attValue);
      }
    if (!strcmp(attName, "NeedlePathTransformNode2Ref")) 
      {
        this->SetAndObserveNeedlePathTransformNode2ID(attValue);
      }
    if (!strcmp(attName, "NeedlePathTransformNode3Ref")) 
      {
        this->SetAndObserveNeedlePathTransformNode3ID(attValue);
      }
      
      // ----
      if (!strcmp(attName, "MITRobotTransformNode1Ref")) 
      {
        this->SetAndObserveMITRobotTransformNode1ID(attValue);
      }
      if (!strcmp(attName, "MITRobotTransformNode2Ref")) 
      {
        this->SetAndObserveMITRobotTransformNode2ID(attValue);
      }
      if (!strcmp(attName, "MITRobotTransformNode3Ref")) 
      {
        this->SetAndObserveMITRobotTransformNode3ID(attValue);
      }
      if (!strcmp(attName, "MITRobotTransformNode4Ref")) 
      {
        this->SetAndObserveMITRobotTransformNode4ID(attValue);
      }
      if (!strcmp(attName, "MITRobotTransformNode5Ref")) 
      {
        this->SetAndObserveMITRobotTransformNode5ID(attValue);
      }
      // 7/21/2012 ayamada
      if (!strcmp(attName, "MITRobotTransformNode6Ref")) 
      {
        this->SetAndObserveMITRobotTransformNode6ID(attValue);
      }
      if (!strcmp(attName, "MITRobotTransformNode7Ref")) 
      {
        this->SetAndObserveMITRobotTransformNode7ID(attValue);
      }
      
      
      // 2/9/2012 ayamada
      if (!strcmp(attName, "TargetPlaneTransformNodeRef")) 
      {
        this->SetAndObserveTargetPlaneTransformNodeID(attValue);
      }
      
      
      if (!strcmp(attName, "MarkersPositionTransformNodeRef")) 
      {
      this->SetAndObserveMarkersPositionTransformNodeID(attValue);
      }
      if (!strcmp(attName, "MarkersPosition2TransformNodeRef")) 
      {
        this->SetAndObserveMarkersPosition2TransformNodeID(attValue);
      }
      
    }
  
//}


/*
    if (!strcmp(attName, "connectorType"))
      {
      if (!strcmp(attValue, "SERVER"))
        {
        type = TYPE_SERVER;
        }
      else if (!strcmp(attValue, "CLIENT"))
        {
        type = TYPE_CLIENT;
        }
      else
        {
        type = TYPE_NOT_DEFINED;
        }
      }
    if (!strcmp(attName, "serverHostname"))
      {
      serverHostname = attValue;
      }
    if (!strcmp(attName, "serverPort"))
      {
      std::stringstream ss;
      ss << attValue;
      ss >> port;
      }
    if (!strcmp(attName, "restrictDeviceName"))
      {
      std::stringstream ss;
      ss << attValue;
      ss >> restrictDeviceName;;
      }
*/

  /*
  switch(type)
    {
    case TYPE_SERVER:
      this->SetTypeServer(port);
      this->SetRestrictDeviceName(restrictDeviceName);
      break;
    case TYPE_CLIENT:
      this->SetTypeClient(serverHostname, port);
      this->SetRestrictDeviceName(restrictDeviceName);
      break;
    default: // not defined
      // do nothing
      break;
    }
  */

}


//----------------------------------------------------------------------------
// Copy the node's attributes to this object.
void vtkMRMLIGTProbeRobotNode::Copy(vtkMRMLNode *anode)
{  
  int disabledModify = this->StartModify();

  Superclass::Copy(anode);
  vtkMRMLIGTProbeRobotNode *node = vtkMRMLIGTProbeRobotNode::SafeDownCast(anode);
  if (node!=NULL)
  {
    this->SetAndObserveRobotCommandNodeID(NULL); // remove observer
    this->SetRobotCommandNodeID(node->RobotCommandNodeID);
    this->SetAndObserveRobotConnectorNodeID(NULL); // remove observer
    this->SetRobotConnectorNodeID(node->RobotConnectorNodeID);
    this->SetAndObserveScannerConnectorNodeID(NULL); // remove observer
    this->SetScannerConnectorNodeID(node->ScannerConnectorNodeID);
    this->SetAndObserveZFrameModelNodeID(NULL); // remove observer

    // 9/21/2011 ayamada    
    this->SetAndObserveMITRobotModelNodeID(NULL); // remove observer

    // 9/21/2011 ayamada    
    this->SetAndObserveMITRobotModelNode2ID(NULL); // remove observer
    // 9/23/2011 ayamada    
    this->SetAndObserveMITRobotModelNode3ID(NULL); // remove observer
    this->SetAndObserveMITRobotModelNode4ID(NULL); // remove observer
    this->SetAndObserveMITRobotModelNode5ID(NULL); // remove observer
    
    // 9/24/2011 ayamada
    this->SetAndObserveNeedleModelNodeID(NULL); // remove observer
    this->SetAndObserveNeedleModelNode1ID(NULL); // remove observer
    this->SetAndObserveNeedleModelNode2ID(NULL); // remove observer
    
    // 11/3/2011 ayamada
    this->SetAndObserveVirtualCenterModelNodeID(NULL); // remove observer

    //this->SetAndObserveVirtualCenterTransformNodeID(node->VirtualCenterTransformNodeID); // remove observer
    
    this->SetZFrameModelNodeID(node->ZFrameModelNodeID);
    this->SetAndObserveZFrameTransformNodeID(NULL); // remove observer
    this->SetZFrameTransformNodeID(node->ZFrameTransformNodeID);

    // 11/18/2011 ayamada
    this->SetVirtualCenterModelNodeID(node->VirtualCenterModelNodeID); // remove observer
    this->SetAndObserveVirtualCenterTransformNodeID(NULL); // remove observer
    this->SetVirtualCenterTransformNodeID(node->VirtualCenterTransformNodeID);
    
    this->SetAndObserveMarkersPositionTransformNodeID(NULL); // remove observer
    this->SetMarkersPositionTransformNodeID(node->MarkersPositionTransformNodeID);
    this->SetAndObserveMarkersPosition2TransformNodeID(NULL); // remove observer
    this->SetMarkersPosition2TransformNodeID(node->MarkersPosition2TransformNodeID);

    // 11/18/2011 ayamada
    this->SetAndObserveNeedleModelNodeID(node->NeedleModelNodeID);
    this->SetAndObserveNeedleModelNode1ID(node->NeedleModelNode1ID);
    this->SetAndObserveNeedleModelNode2ID(node->NeedleModelNode2ID);
    
    this->SetNeedlePathTransformNode1ID(NULL);
    this->SetNeedlePathTransformNode2ID(NULL);
    this->SetNeedlePathTransformNode3ID(NULL);
    
    // 10/6/2011 ayamada
    this->SetNeedlePathTransformNode1ID(node->NeedlePathTransformNode1ID);
    this->SetNeedlePathTransformNode2ID(node->NeedlePathTransformNode2ID);
    this->SetNeedlePathTransformNode3ID(node->NeedlePathTransformNode3ID);
 
    // 11/18/2011 ayamada
    this->SetAndObserveMITRobotModelNodeID(node->MITRobotModelNodeID);
    this->SetAndObserveMITRobotModelNode2ID(node->MITRobotModelNode2ID);
    this->SetAndObserveMITRobotModelNode3ID(node->MITRobotModelNode3ID);
    this->SetAndObserveMITRobotModelNode4ID(node->MITRobotModelNode4ID);
    this->SetAndObserveMITRobotModelNode5ID(node->MITRobotModelNode5ID);

    this->SetMITRobotTransformNode1ID(NULL);
    this->SetMITRobotTransformNode2ID(NULL);
    this->SetMITRobotTransformNode3ID(NULL);
    this->SetMITRobotTransformNode4ID(NULL);
    this->SetMITRobotTransformNode5ID(NULL);
    // 7/21/2012 ayamada
    this->SetMITRobotTransformNode6ID(NULL);
    this->SetMITRobotTransformNode7ID(NULL);
    
    // 2/9/2012 ayamada
    this->SetTargetPlaneTransformNodeID(NULL);
    
    this->SetMITRobotTransformNode1ID(node->MITRobotTransformNode1ID);
    this->SetMITRobotTransformNode2ID(node->MITRobotTransformNode2ID);
    this->SetMITRobotTransformNode3ID(node->MITRobotTransformNode3ID);
    this->SetMITRobotTransformNode4ID(node->MITRobotTransformNode4ID);
    this->SetMITRobotTransformNode5ID(node->MITRobotTransformNode5ID);
    // 7/21/2012 ayamada
    this->SetMITRobotTransformNode6ID(node->MITRobotTransformNode7ID);
    this->SetMITRobotTransformNode7ID(node->MITRobotTransformNode7ID);
    
    // 2/9/2012 ayamada
    this->SetTargetPlaneTransformNodeID(node->TargetPlaneTransformNodeID);
    
    
  }
  else
  {
    vtkErrorMacro("Invalid node");
  }

  this->StatusDescriptors.clear();
  for (unsigned int i=0; i<node->StatusDescriptors.size(); i++)
  {    
    this->StatusDescriptors.push_back(node->StatusDescriptors[i]);
  }

  this->EndModify(disabledModify);
}

//-----------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::UpdateReferences()
{
  Superclass::UpdateReferences();

  if (this->RobotCommandNodeID != NULL && this->Scene->GetNodeByID(this->RobotCommandNodeID) == NULL)
    {
    this->SetAndObserveRobotCommandNodeID(NULL);
    }
  if (this->RobotConnectorNodeID != NULL && this->Scene->GetNodeByID(this->RobotConnectorNodeID) == NULL)
    {
    this->SetAndObserveRobotConnectorNodeID(NULL);
    }
  if (this->ScannerConnectorNodeID != NULL && this->Scene->GetNodeByID(this->ScannerConnectorNodeID) == NULL)
    {
    this->SetAndObserveScannerConnectorNodeID(NULL);
    }
  if (this->ZFrameModelNodeID != NULL && this->Scene->GetNodeByID(this->ZFrameModelNodeID) == NULL)
    {
    this->SetAndObserveZFrameModelNodeID(NULL);
    }

  // 9/21/2011 ayamada  
  if (this->MITRobotModelNodeID != NULL && this->Scene->GetNodeByID(this->MITRobotModelNodeID) == NULL)
  {
    this->SetAndObserveMITRobotModelNodeID(NULL);
  }

  // 9/21/2011 ayamada  
  if (this->MITRobotModelNode2ID != NULL && this->Scene->GetNodeByID(this->MITRobotModelNode2ID) == NULL)
  {
    this->SetAndObserveMITRobotModelNode2ID(NULL);
  }
  // 9/23/2011 ayamada  
  if (this->MITRobotModelNode3ID != NULL && this->Scene->GetNodeByID(this->MITRobotModelNode3ID) == NULL)
  {
    this->SetAndObserveMITRobotModelNode3ID(NULL);
  }
  if (this->MITRobotModelNode4ID != NULL && this->Scene->GetNodeByID(this->MITRobotModelNode4ID) == NULL)
  {
    this->SetAndObserveMITRobotModelNode4ID(NULL);
  }
  if (this->MITRobotModelNode5ID != NULL && this->Scene->GetNodeByID(this->MITRobotModelNode5ID) == NULL)
  {
    this->SetAndObserveMITRobotModelNode5ID(NULL);
  }

  // 9/24/2011 ayamada
  if (this->NeedleModelNodeID != NULL && this->Scene->GetNodeByID(this->NeedleModelNodeID) == NULL)
  {
    this->SetAndObserveNeedleModelNodeID(NULL);
  }
  if (this->NeedleModelNode1ID != NULL && this->Scene->GetNodeByID(this->NeedleModelNode1ID) == NULL)
  {
    this->SetAndObserveNeedleModelNode1ID(NULL);
  }
  if (this->NeedleModelNode2ID != NULL && this->Scene->GetNodeByID(this->NeedleModelNode2ID) == NULL)
  {
    this->SetAndObserveNeedleModelNode2ID(NULL);
  }
  
  // 11/3/2011 ayamada
  if (this->VirtualCenterModelNodeID != NULL && this->Scene->GetNodeByID(this->VirtualCenterModelNodeID) == NULL)
  {
    this->SetAndObserveVirtualCenterModelNodeID(NULL);
  }
  if (this->VirtualCenterTransformNodeID != NULL && this->Scene->GetNodeByID(this->VirtualCenterTransformNodeID) == NULL)
  {
    this->SetAndObserveVirtualCenterTransformNodeID(NULL);
  }
  
  if (this->ZFrameTransformNodeID != NULL && this->Scene->GetNodeByID(this->ZFrameTransformNodeID) == NULL)
    {
    this->SetAndObserveZFrameTransformNodeID(NULL);
    }

  // 10/06/2011 ayamada
  if (this->NeedlePathTransformNode1ID != NULL && this->Scene->GetNodeByID(this->NeedlePathTransformNode1ID) == NULL)
  {
    this->SetAndObserveNeedlePathTransformNode1ID(NULL);
  }
  if (this->NeedlePathTransformNode2ID != NULL && this->Scene->GetNodeByID(this->NeedlePathTransformNode2ID) == NULL)
  {
    this->SetAndObserveNeedlePathTransformNode2ID(NULL);
  }
  if (this->NeedlePathTransformNode3ID != NULL && this->Scene->GetNodeByID(this->NeedlePathTransformNode3ID) == NULL)
  {
    this->SetAndObserveNeedlePathTransformNode3ID(NULL);
  }
  
  
  
  if (this->MarkersPositionTransformNodeID != NULL && this->Scene->GetNodeByID(this->MarkersPositionTransformNodeID) == NULL)
  {
    this->SetAndObserveMarkersPositionTransformNodeID(NULL);
  }
  if (this->MarkersPosition2TransformNodeID != NULL && this->Scene->GetNodeByID(this->MarkersPosition2TransformNodeID) == NULL)
  {
    this->SetAndObserveMarkersPosition2TransformNodeID(NULL);
  }
}

//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::UpdateReferenceID(const char *oldID, const char *newID)
{
  Superclass::UpdateReferenceID(oldID, newID);
  if (this->RobotCommandNodeID && !strcmp(oldID, this->RobotCommandNodeID))
    {
    this->SetAndObserveRobotCommandNodeID(newID);
    }
  if (this->RobotConnectorNodeID && !strcmp(oldID, this->RobotConnectorNodeID))
    {
    this->SetAndObserveRobotConnectorNodeID(newID);
    }
  if (this->ScannerConnectorNodeID && !strcmp(oldID, this->ScannerConnectorNodeID))
    {
    this->SetAndObserveScannerConnectorNodeID(newID);
    }
  if (this->ZFrameModelNodeID && !strcmp(oldID, this->ZFrameModelNodeID))
    {
    this->SetAndObserveZFrameModelNodeID(newID);
    }

  // 9/21/2011 ayamada
  if (this->MITRobotModelNodeID && !strcmp(oldID, this->MITRobotModelNodeID))
  {
    this->SetAndObserveMITRobotModelNodeID(newID);
  }
  
  // 9/23/2011 ayamada
  if (this->MITRobotModelNode2ID && !strcmp(oldID, this->MITRobotModelNode2ID))
  {
    this->SetAndObserveMITRobotModelNode2ID(newID);
  }
  if (this->MITRobotModelNode3ID && !strcmp(oldID, this->MITRobotModelNode3ID))
  {
    this->SetAndObserveMITRobotModelNode3ID(newID);
  }
  if (this->MITRobotModelNode4ID && !strcmp(oldID, this->MITRobotModelNode4ID))
  {
    this->SetAndObserveMITRobotModelNode4ID(newID);
  }
  if (this->MITRobotModelNode5ID && !strcmp(oldID, this->MITRobotModelNode5ID))
  {
    this->SetAndObserveMITRobotModelNode5ID(newID);
  }
  

  // 9/24/2011 ayamada
  if (this->NeedleModelNodeID && !strcmp(oldID, this->NeedleModelNodeID))
  {
    this->SetAndObserveNeedleModelNodeID(newID);
  }
  if (this->NeedleModelNode1ID && !strcmp(oldID, this->NeedleModelNode1ID))
  {
    this->SetAndObserveNeedleModelNode1ID(newID);
  }
  if (this->NeedleModelNode2ID && !strcmp(oldID, this->NeedleModelNode2ID))
  {
    this->SetAndObserveNeedleModelNode2ID(newID);
  }
  
  // 11/3/2011 ayamada
  if (this->VirtualCenterModelNodeID && !strcmp(oldID, this->VirtualCenterModelNodeID))
  {
    this->SetAndObserveVirtualCenterModelNodeID(newID);
  }
  if (this->VirtualCenterTransformNodeID && !strcmp(oldID, this->VirtualCenterTransformNodeID))
  {
    this->SetAndObserveVirtualCenterTransformNodeID(newID);
  }
  
  
  if (this->ZFrameTransformNodeID && !strcmp(oldID, this->ZFrameTransformNodeID))
    {
    this->SetAndObserveZFrameTransformNodeID(newID);
    }

  // 10/6/2011 ayamada
  if (this->NeedlePathTransformNode1ID && !strcmp(oldID, this->NeedlePathTransformNode1ID))
  {
    this->SetAndObserveNeedlePathTransformNode1ID(newID);
  }
  if (this->NeedlePathTransformNode2ID && !strcmp(oldID, this->NeedlePathTransformNode2ID))
  {
    this->SetAndObserveNeedlePathTransformNode2ID(newID);
  }
  if (this->NeedlePathTransformNode3ID && !strcmp(oldID, this->NeedlePathTransformNode3ID))
  {
    this->SetAndObserveNeedlePathTransformNode3ID(newID);
  }

  // ----
  if (this->MITRobotTransformNode1ID && !strcmp(oldID, this->MITRobotTransformNode1ID))
  {
    this->SetAndObserveMITRobotTransformNode1ID(newID);
  }
  if (this->MITRobotTransformNode2ID && !strcmp(oldID, this->MITRobotTransformNode2ID))
  {
    this->SetAndObserveMITRobotTransformNode2ID(newID);
  }
  if (this->MITRobotTransformNode3ID && !strcmp(oldID, this->MITRobotTransformNode3ID))
  {
    this->SetAndObserveMITRobotTransformNode3ID(newID);
  }
  if (this->MITRobotTransformNode4ID && !strcmp(oldID, this->MITRobotTransformNode4ID))
  {
    this->SetAndObserveMITRobotTransformNode4ID(newID);
  }
  if (this->MITRobotTransformNode5ID && !strcmp(oldID, this->MITRobotTransformNode5ID))
  {
    this->SetAndObserveMITRobotTransformNode5ID(newID);
  }
  // 7/21/2012 ayamada
  if (this->MITRobotTransformNode6ID && !strcmp(oldID, this->MITRobotTransformNode6ID))
  {
    this->SetAndObserveMITRobotTransformNode6ID(newID);
  }
  if (this->MITRobotTransformNode7ID && !strcmp(oldID, this->MITRobotTransformNode7ID))
  {
    this->SetAndObserveMITRobotTransformNode7ID(newID);
  }
  
  // 2/9/2012 ayamada
  if (this->TargetPlaneTransformNodeID && !strcmp(oldID, this->TargetPlaneTransformNodeID))
  {
    this->SetAndObserveTargetPlaneTransformNodeID(newID);
  }
  
  
  
  if (this->MarkersPositionTransformNodeID && !strcmp(oldID, this->MarkersPositionTransformNodeID))
  {
    this->SetAndObserveMarkersPositionTransformNodeID(newID);
  }
  if (this->MarkersPosition2TransformNodeID && !strcmp(oldID, this->MarkersPosition2TransformNodeID))
  {
    this->SetAndObserveMarkersPosition2TransformNodeID(newID);
  }
  
}

//-----------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::UpdateScene(vtkMRMLScene *scene)
{
   Superclass::UpdateScene(scene);
   this->SetAndObserveRobotCommandNodeID(this->GetRobotCommandNodeID());
   this->SetAndObserveRobotConnectorNodeID(this->GetRobotConnectorNodeID());
   this->SetAndObserveScannerConnectorNodeID(this->GetScannerConnectorNodeID());
   this->SetAndObserveZFrameModelNodeID(this->GetZFrameModelNodeID());
  
   // 9/21/2011 ayamada
   this->SetAndObserveMITRobotModelNodeID(this->GetMITRobotModelNodeID());
   // 9/23/2011 ayamada
   this->SetAndObserveMITRobotModelNode2ID(this->GetMITRobotModelNode2ID());
   this->SetAndObserveMITRobotModelNode3ID(this->GetMITRobotModelNode3ID());
   this->SetAndObserveMITRobotModelNode4ID(this->GetMITRobotModelNode4ID());
   this->SetAndObserveMITRobotModelNode5ID(this->GetMITRobotModelNode5ID());
  
   // 11/3/2011 ayamada
   this->SetAndObserveVirtualCenterModelNodeID(this->GetVirtualCenterObjectModelId());
   this->SetAndObserveVirtualCenterTransformNodeID(this->GetVirtualCenterTransformId());

   // 10/6/2011 ayamada 
   this->SetAndObserveNeedlePathTransformNode1ID(this->GetNeedlePathTransformNode1ID());
   this->SetAndObserveNeedlePathTransformNode2ID(this->GetNeedlePathTransformNode2ID());
   this->SetAndObserveNeedlePathTransformNode3ID(this->GetNeedlePathTransformNode3ID());

   this->SetAndObserveMITRobotTransformNode1ID(this->GetMITRobotTransformNode1ID());
   this->SetAndObserveMITRobotTransformNode2ID(this->GetMITRobotTransformNode2ID());
   this->SetAndObserveMITRobotTransformNode3ID(this->GetMITRobotTransformNode3ID());
   this->SetAndObserveMITRobotTransformNode4ID(this->GetMITRobotTransformNode4ID());
   this->SetAndObserveMITRobotTransformNode5ID(this->GetMITRobotTransformNode5ID());
   // 7/21/2012 ayamada
   this->SetAndObserveMITRobotTransformNode6ID(this->GetMITRobotTransformNode6ID());
   this->SetAndObserveMITRobotTransformNode7ID(this->GetMITRobotTransformNode7ID());
  
   // 2/9/2012 ayamada
   this->SetAndObserveTargetPlaneTransformNodeID(this->GetTargetPlaneTransformNodeID());
  
   this->SetAndObserveZFrameTransformNodeID(this->GetZFrameTransformNodeID());
   this->SetAndObserveMarkersPositionTransformNodeID(this->GetMarkersPositionTransformNodeID());
   this->SetAndObserveMarkersPosition2TransformNodeID(this->GetMarkersPosition2TransformNodeID());
}

//-----------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::ProcessMRMLEvents( vtkObject *caller, unsigned long event, void *callData )
{
  Superclass::ProcessMRMLEvents(caller, event, callData);

  if (this->StatusDescriptors.size()<=STATUS_SCANNER || this->StatusDescriptors.size()<=STATUS_ROBOT)
  {
    vtkErrorMacro("StatusDescriptors does not contain scanner and robot status");
  }
  else
  {
    if (this->GetScannerConnectorNode() && this->GetScannerConnectorNode() == vtkMRMLIGTLConnectorNode::SafeDownCast(caller))
      {
      switch (event)
        {
        case vtkMRMLIGTLConnectorNode::ConnectedEvent:
          this->StatusDescriptors[STATUS_SCANNER].indicator=StatusOk;
          this->StatusDescriptors[STATUS_SCANNER].text=STATUS_SCANNER_ON;
          this->InvokeEvent(vtkMRMLRobotNode::ChangeStatusEvent);
          break;
        case vtkMRMLIGTLConnectorNode::DisconnectedEvent:
          this->StatusDescriptors[STATUS_SCANNER].indicator=StatusOff;
          this->StatusDescriptors[STATUS_SCANNER].text=STATUS_SCANNER_OFF;
          this->InvokeEvent(vtkMRMLRobotNode::ChangeStatusEvent);
          break;
        }
      }
    else if (this->GetRobotConnectorNode() && this->GetRobotConnectorNode() == vtkMRMLIGTLConnectorNode::SafeDownCast(caller))
      {
      switch (event)
        {
        case vtkMRMLIGTLConnectorNode::ConnectedEvent:
          this->StatusDescriptors[STATUS_ROBOT].indicator=StatusOk;
          this->StatusDescriptors[STATUS_ROBOT].text=STATUS_ROBOT_ON;
          this->InvokeEvent(vtkMRMLRobotNode::ChangeStatusEvent);
          break;
        case vtkMRMLIGTLConnectorNode::DisconnectedEvent:
          this->StatusDescriptors[STATUS_ROBOT].indicator=StatusOff;
          this->StatusDescriptors[STATUS_ROBOT].text=STATUS_ROBOT_OFF;
          this->InvokeEvent(vtkMRMLRobotNode::ChangeStatusEvent);
          break;
        }
      }
  }
}


//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}


int vtkMRMLIGTProbeRobotNode::Init(vtkSlicerApplication* app, const char* moduleShareDir)
{ 
  this->Superclass::Init(app, moduleShareDir);

  vtkOpenIGTLinkIFGUI* igtlGUI = vtkOpenIGTLinkIFGUI::SafeDownCast(app->GetModuleGUIByName("OpenIGTLink IF"));
  if (igtlGUI)
    {
    igtlGUI->Enter();    
    //igtlGUI->GetLogic()->RegisterMessageConverter(this->CoordinateConverter);
    igtlGUI->GetLogic()->RegisterMessageConverter(this->CommandConverter);
    }

  //std::cerr << "Adding robot command" << std::endl;

  // Set up robot command node.
  if (GetRobotCommandNode()==NULL)
  {  
    vtkSmartPointer<vtkMRMLBrpRobotCommandNode> node = vtkSmartPointer<vtkMRMLBrpRobotCommandNode>::New();
    node->SetName("BRPRobotCommand");
    node->SetScene(this->GetScene());
    this->Scene->SaveStateForUndo();
    this->Scene->AddNode(node);
    node->Modified();
    this->Scene->Modified();
    SetAndObserveRobotCommandNodeID(node->GetID());
  }

  // Z-Frame transform node
  if (GetZFrameTransformNode()==NULL)
  {
    vtkMRMLLinearTransformNode* ztnode = vtkMRMLLinearTransformNode::New();
    // 1/17/2011 ayamada
    //ztnode->SetName("ZFrameTransform");
    ztnode->SetName("RobotTransform");
    vtkMatrix4x4* ztransform = vtkMatrix4x4::New();
    ztransform->Identity();
    //transformNode->SetAndObserveImageData(transform);
    ztnode->ApplyTransform(ztransform);
    ztnode->SetScene(this->Scene);
    ztransform->Delete();
    this->Scene->AddNode(ztnode);
    SetAndObserveZFrameTransformNodeID(ztnode->GetID());
  }


  // 11/4/2011 ayamada
  // Virtual center transform node
  if (GetVirtualCenterTransformNode()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("VirtualCenterTransformNode");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();
    
    // 11/4/2011 ayamada
    // reference from vtkRobotProbeNavTargetingStep.cxx l.590
    //vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();
    
    /*
    int test;
    vtkRobotProbeNavLogic* navLogic = vtkRobotProbeNavLogic::New();
    test = navLogic->FiducialMarkersR[0][1][0][0];
    std::cout << "test=" << float(test) << endl;
    nptransform->SetElement(0,3, 5);//(float)test);
    */
    
    // you should update the data like needle position and orientation update.
    
    
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    // 10/27/2011 ayamada; comment out
    // 12/29/2011 ayamada added
    nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveVirtualCenterTransformNodeID(nptnode->GetID());
  }
  
  
  
  // 10/6/2011 ayamada
  // Needle path transform node
  if (GetNeedlePathTransformNode1()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("NeedlePathTransform1");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();
    
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    // 10/27/2011 ayamada; comment out
    //nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveNeedlePathTransformNode1ID(nptnode->GetID());
  }

  if (GetNeedlePathTransformNode2()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("NeedlePathTransform2");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();

    //transformNode->SetAndObserveImageData(transform);
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    // 10/27/2011 ayamada; comment out
    //nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveNeedlePathTransformNode2ID(nptnode->GetID());
  }

  if (GetNeedlePathTransformNode3()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("NeedlePathTransform3");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();
    //transformNode->SetAndObserveImageData(transform);
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    // 10/27/2011 ayamada; comment out
    //nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());      
        
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveNeedlePathTransformNode3ID(nptnode->GetID());
  }

  // ----- 
  // 2/9/2012 ayamada
  if (GetTargetPlaneTransformNode()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("TargetPlaneTransform");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();
    
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    nptnode->SetAndObserveTransformNodeID(GetTargetPlaneTransformNodeID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveTargetPlaneTransformNodeID(nptnode->GetID());
  }  
  
  if (GetMITRobotTransformNode1()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("RobotTransform1");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();    
    // 7/2/2012 ayamada: modify the rotation of the stl file.
    // applied 45 degree rotation to compensate the difference of angle between CAD coordinate and RAS coordinate
    /*
    nptransform->SetElement(0, 0, 0.0);//0.707107);
    nptransform->SetElement(0, 2, 1.0);//0.707107);
    nptransform->SetElement(2, 0, -1.0);//-0.707107);
    nptransform->SetElement(2, 2, 0.0);//0.707107);
    */
    
    // 7/3/2012 ayamada: for compensation offset between CAD coordinate and robot coordinate
    // 7/17/2012 ayamada: need to update
    //nptransform->SetElement(1, 3, 4.0);
    
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveMITRobotTransformNode1ID(nptnode->GetID());
  }
  if (GetMITRobotTransformNode2()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("RobotTransform2");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();
    // 7/2/2012 ayamada: modify the rotation of the stl file.
    /*
    nptransform->SetElement(0, 0, 0.0);//0.707107);
    nptransform->SetElement(0, 2, 1.0);//0.707107);
    nptransform->SetElement(2, 0, -1.0);//-0.707107);
    nptransform->SetElement(2, 2, 0.0);//0.707107);
    */
    
    // 7/3/2012 ayamada: for compensation offset between CAD coordinate and robot coordinate
    // 7/17/2012 ayamada: need to update
    //nptransform->SetElement(1, 3, 4.0);
    
    
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveMITRobotTransformNode2ID(nptnode->GetID());
  }
  if (GetMITRobotTransformNode3()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("RobotTransform3");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();
    // 7/2/2012 ayamada: modify the rotation of the stl file.
    /*
    nptransform->SetElement(0, 0, 0.0);//0.707107);
    nptransform->SetElement(0, 2, 1.0);//0.707107);
    nptransform->SetElement(2, 0, -1.0);//-0.707107);
    nptransform->SetElement(2, 2, 0.0);//0.707107);
    */
    // 7/3/2012 ayamada: for compensation offset between CAD coordinate and robot coordinate
    // 7/17/2012 ayamada: need to update
    //nptransform->SetElement(1, 3, 4.0);

    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveMITRobotTransformNode3ID(nptnode->GetID());
  }
  if (GetMITRobotTransformNode4()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("RobotTransform4");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();
    // 7/2/2012 ayamada: modify the rotation of the stl file.
    /*
    nptransform->SetElement(0, 0, 0.0);//0.707107);
    nptransform->SetElement(0, 2, 1.0);//0.707107);
    nptransform->SetElement(2, 0, -1.0);//-0.707107);
    nptransform->SetElement(2, 2, 0.0);//0.707107);
    */
    
    // 7/3/2012 ayamada: for compensation offset between CAD coordinate and robot coordinate
    // 7/17/2012 ayamada: need to update
    //nptransform->SetElement(1, 3, 4.0);
    
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveMITRobotTransformNode4ID(nptnode->GetID());
  }
  if (GetMITRobotTransformNode5()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("RobotTransform5");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();

    // 7/22/2012 ayamada
    vtkMatrix4x4* nptransform2 = vtkMatrix4x4::New();
    nptransform2->Identity();

    vtkMatrix4x4* nptransform3 = vtkMatrix4x4::New();
    nptransform3->Identity();

    vtkMatrix4x4* nptransformTmp = vtkMatrix4x4::New();
    nptransformTmp->Identity();
    
    // 7/12/2012 ayamada cut
    // 7/2/2012 ayamada: modify the rotation of the stl file.
    /*
    nptransform->SetElement(0, 0, 0.0);//0.707107);
    nptransform->SetElement(0, 2, 1.0);//0.707107);
    nptransform->SetElement(2, 0, -1.0);//-0.707107);
    nptransform->SetElement(2, 2, 0.0);//0.707107);
    */
    
    // 7/3/2012 ayamada: for compensation offset between CAD coordinate and robot coordinate
    //nptransform->SetElement(1, 3, 4.0);
        
    // 7/12/2012 ayamada
    nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID()/*GetMITRobotTransformNode4ID()*/);
    
    
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    // 1/21/2012 ayamada
    //nptnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveMITRobotTransformNode5ID(nptnode->GetID());
  }
  // 7/21/2012 ayamada
  //std::cout << "GetMITRobotTransformNode6Before" << endl;

  if (GetMITRobotTransformNode6()==NULL)
  {
    
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("RobotTransform6");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();
    //std::cout << "GetMITRobotTransformNode6" << endl;
    
    // 7/2/2012 ayamada: modify the rotation of the stl file.
    //nptransform->SetElement(0, 0, 0.707107);
    //nptransform->SetElement(0, 2, 0.707107);
    //nptransform->SetElement(2, 0, -0.707107);
    //nptransform->SetElement(2, 2, 0.707107);
    
    // 7/3/2012 ayamada: for compensation offset between CAD coordinate and robot coordinate
    // 7/17/2012 ayamada: need to update
    //nptransform->SetElement(1, 3, 4.0);
    
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    
    nptnode->SetAndObserveTransformNodeID(GetMITRobotTransformNode4ID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveMITRobotTransformNode6ID(nptnode->GetID());
    
  }
  if (GetMITRobotTransformNode7()==NULL)
  {
    vtkMRMLLinearTransformNode* nptnode = vtkMRMLLinearTransformNode::New();
    nptnode->SetName("RobotTransform7");
    vtkMatrix4x4* nptransform = vtkMatrix4x4::New();
    nptransform->Identity();
    // 7/2/2012 ayamada: modify the rotation of the stl file.
    //nptransform->SetElement(0, 0, 0.707107);
    //nptransform->SetElement(0, 2, 0.707107);
    //nptransform->SetElement(2, 0, -0.707107);
    //nptransform->SetElement(2, 2, 0.707107);
    
    // 7/3/2012 ayamada: for compensation offset between CAD coordinate and robot coordinate
    // 7/17/2012 ayamada: need to update
    //nptransform->SetElement(1, 3, 4.0);
    
    // 10/6/2011 ayamada; based on l.978 around 9/21/2011 ayamada function
    nptnode->SetAndObserveTransformNodeID(GetMITRobotTransformNode5ID());      
    
    nptnode->ApplyTransform(nptransform);
    nptnode->SetScene(this->Scene);
    nptransform->Delete();
    this->Scene->AddNode(nptnode);
    SetAndObserveMITRobotTransformNode7ID(nptnode->GetID());
  }
  
  
  // 9/13/2011 ayamada
  if (GetMarkersPositionTransformNode()==NULL)
  {
    vtkMRMLLinearTransformNode* mptnode = vtkMRMLLinearTransformNode::New();
    mptnode->SetName("MarkersPositionTransform");
    vtkMatrix4x4* mptransform = vtkMatrix4x4::New();
    mptransform->Identity();
    //transformNode->SetAndObserveImageData(transform);
    mptnode->ApplyTransform(mptransform);
    mptnode->SetScene(this->Scene);
    mptransform->Delete();
    this->Scene->AddNode(mptnode);
    SetAndObserveMarkersPositionTransformNodeID(mptnode->GetID());
  }

  // 9/13/2011 ayamada
  if (GetMarkersPosition2TransformNode()==NULL)
  {
    vtkMRMLLinearTransformNode* mp2tnode = vtkMRMLLinearTransformNode::New();
    // 1/26/2012 ayamada
    //mp2tnode->SetName("MarkersPosition2Transform");

    // 7/10/2012 ayamada
    //mp2tnode->SetName("MarkersPosition2");
    //mp2tnode->SetName("pose");
    mp2tnode->SetName("MRIRobotTransform");
    vtkMatrix4x4* mp2transform = vtkMatrix4x4::New();
    mp2transform->Identity();
    //transformNode->SetAndObserveImageData(transform);
    mp2tnode->ApplyTransform(mp2transform);
    mp2tnode->SetScene(this->Scene);
    
    // 9/22/2011 ayamada, for experiment  
    // 1/21/2012 ayamada
    //mp2tnode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());    
    
    mp2transform->Delete();
    this->Scene->AddNode(mp2tnode);
    SetAndObserveMarkersPosition2TransformNodeID(mp2tnode->GetID());
  }

  
  // MIT Needle Insertion Robot based on ZFrame model
  // This part should be moved to Robot Display Node.
  
  if (GetMITRobotModelNode()==NULL)
  {
     
    const char* nodeID = AddMITRobotModel("Robot-LoopCoil",0);
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
    {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 9/21/2011 ayamada  
      modelNode->SetAndObserveTransformNodeID(GetMITRobotTransformNode1ID());      
      SetAndObserveMITRobotModelNodeID(nodeID);
    }
  }

  if (GetMITRobotModelNode2()==NULL)
  {
  
    const char* nodeID = AddMITRobotModel("Robot-Body",1);
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
    {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 9/21/2011 ayamada  
      modelNode->SetAndObserveTransformNodeID(GetMITRobotTransformNode2ID());
      SetAndObserveMITRobotModelNode2ID(nodeID);
    }
  }

  if (GetMITRobotModelNode3()==NULL)
  {
  
    const char* nodeID = AddMITRobotModel("Robot-Actuators",2);
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
    {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 9/21/2011 ayamada  
      modelNode->SetAndObserveTransformNodeID(GetMITRobotTransformNode3ID());
      SetAndObserveMITRobotModelNode3ID(nodeID);
    }
  }

  if (GetMITRobotModelNode4()==NULL)
  {
    
    const char* nodeID = AddMITRobotModel("Robot-hooper",3);
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
    {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 7/21/2012 ayamada updated  
      modelNode->SetAndObserveTransformNodeID(GetMITRobotTransformNode4ID());
      SetAndObserveMITRobotModelNode4ID(nodeID);
    }
  }

  if (GetMITRobotModelNode5()==NULL)
  {
  
    const char* nodeID = AddMITRobotModel("Robot-needleHolder",4);
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
    {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 9/21/2011 ayamada  
      modelNode->SetAndObserveTransformNodeID(GetMITRobotTransformNode5ID());
      SetAndObserveMITRobotModelNode5ID(nodeID);
    }
    
    
  }  

  // 9/24/2011 ayamada
  if (GetNeedleModelNode()==NULL)
  {
    
    const char* nodeID = AddNeedleModel("Planned Needle Path 1",0);
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
    {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 10/6/2011 ayamada  
      modelNode->SetAndObserveTransformNodeID(GetNeedlePathTransformNode1ID()/*GetZFrameTransformNodeID()*/);
      SetAndObserveNeedleModelNodeID(nodeID);
      
      
    }
    
    
  }  

  if (GetNeedleModelNode1()==NULL)
  {
    
    const char* nodeID = AddNeedleModel("Planned Needle Path 2",1);
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
    {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 9/21/2011 ayamada  
      modelNode->SetAndObserveTransformNodeID(GetNeedlePathTransformNode2ID()/*GetZFrameTransformNodeID()*/);
      SetAndObserveNeedleModelNode1ID(nodeID);
    }
    
    
  }  

  if (GetNeedleModelNode2()==NULL)
  {
    
    const char* nodeID = AddNeedleModel("Planned Needle Path 3",2);
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
    {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 9/21/2011 ayamada  
      modelNode->SetAndObserveTransformNodeID(GetNeedlePathTransformNode3ID()/*GetZFrameTransformNodeID()*/);
      SetAndObserveNeedleModelNode2ID(nodeID);
    }
    
    
  }  
  

  // Virtual Center Model
  // 11/3/2011 ayamada
  if (GetVirtualCenterModelNode()==NULL)
  {
    
    const char* nodeID = AddVirtualCenterModel("Virtual Center");
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
    {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 9/21/2011 ayamada  
      modelNode->SetAndObserveTransformNodeID(GetVirtualCenterTransformId());
      SetAndObserveVirtualCenterModelNodeID(nodeID);
    }
    
    
  }  
  
  // ZFrame model
  // This part should be moved to Robot Display Node.
  if (GetZFrameModelNode()==NULL)
  {
    const char* nodeID = AddZFrameModel("ZFrame");
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
      {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      
      // 9/21/2011 ayamada  
      modelNode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());
        
      SetAndObserveZFrameModelNodeID(nodeID);
      }
  }

  // Workspace model
  // This part should be moved to Robot Display Node.
  if (GetWorkspaceModelNode()==NULL)
  {
    const char* nodeID = AddWorkspaceModel("Range of Motion");
    vtkMRMLModelNode*  modelNode = vtkMRMLModelNode::SafeDownCast(this->Scene->GetNodeByID(nodeID));
    if (modelNode)
      {
      vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
      displayNode->SetVisibility(0);
      modelNode->Modified();
      this->Scene->Modified();
      modelNode->SetAndObserveTransformNodeID(GetZFrameTransformNodeID());
      SetAndObserveWorkspaceModelNodeID(nodeID);
      }
  }
  
  // 3/8/2012 ayamada
  //this->ModuleShareDirectory = GetGUI()->ModuleShareDirectory;
  

  return 1;
}

int vtkMRMLIGTProbeRobotNode::OnTimer()
{
    if (this->GetRobotCommandNode())
      {
      //cnode->PushOutgoingCommand("GET_COORDINATE");
      this->GetRobotCommandNode()->PushOutgoingCommand("GET_COORDINA");
      this->GetRobotCommandNode()->InvokeEvent(vtkCommand::ModifiedEvent);
      }
//    std::cerr << "void vtkRobotProbeNavLogic::TimerHandler() is called" << std::endl;
    return 1;
}

//---------------------------------------------------------------------------
int  vtkMRMLIGTProbeRobotNode::MoveTo(const char *transformNodeId)
{
  if (this->GetRobotCommandNode()==NULL)
    {
    return 0;
    }

  this->GetRobotCommandNode()->SetTargetTransformNodeID(transformNodeId);
  this->GetRobotCommandNode()->PushOutgoingCommand("MOVE_TO");
  this->GetRobotCommandNode()->InvokeEvent(vtkCommand::ModifiedEvent);
  
  return 1;
}

//---------------------------------------------------------------------------
int vtkMRMLIGTProbeRobotNode::SendZFrame()
{
  if (this->GetRobotCommandNode()==NULL)
    {
    return 0;
    }

  std::cerr << "int vtkRobotProbeNavLogic::SendZFrame(): " << this->GetZFrameTransformNodeID() << std::endl;

  this->GetRobotCommandNode()->SetZFrameTransformNodeID(this->GetZFrameTransformNodeID());
  this->GetRobotCommandNode()->PushOutgoingCommand("SET_Z_FRAME");
  this->GetRobotCommandNode()->InvokeEvent(vtkCommand::ModifiedEvent);

  return 1;
  
}


//----------------------------------------------------------------------------
const char* vtkMRMLIGTProbeRobotNode::AddWorkspaceModel(const char* nodeName)
{
  vtkMRMLModelNode           *workspaceModel;
  vtkMRMLModelDisplayNode    *workspaceDisp;
  
  workspaceModel = vtkMRMLModelNode::New();
  workspaceDisp  = vtkMRMLModelDisplayNode::New();

  this->Scene->SaveStateForUndo();
  this->Scene->AddNode(workspaceDisp);
  this->Scene->AddNode(workspaceModel);

  workspaceDisp->SetScene(this->Scene);
  workspaceModel->SetName(nodeName);
  workspaceModel->SetScene(this->Scene);
  workspaceModel->SetAndObserveDisplayNodeID(workspaceDisp->GetID());
  workspaceModel->SetHideFromEditors(0);

  // construct Z-frame model
  
  // Parameters
  // offset from z-frame -- this will be a class variable to let users configure it in the future.
  const double offsetFromZFrame[] = {0, 22.76, 150.0};
  const double length = 200.0;

  //----- cylinder 1 (R-A) -----
  vtkCylinderSource *cylinder1 = vtkCylinderSource::New();
  cylinder1->SetResolution(24);
  cylinder1->SetRadius(25.0);
  cylinder1->SetHeight(length);
  cylinder1->SetCenter(0, 0, 0);
  cylinder1->Update();
  
  vtkTransform* trans1 =   vtkTransform::New();
  trans1->Translate(offsetFromZFrame);
  trans1->RotateX(90.0);
  trans1->Update();

  vtkTransformPolyDataFilter *tfilter1 = vtkTransformPolyDataFilter::New();
  tfilter1->SetInput(cylinder1->GetOutput());
  tfilter1->SetTransform(trans1);
  tfilter1->Update();


  vtkAppendPolyData *apd = vtkAppendPolyData::New();
  apd->AddInput(tfilter1->GetOutput());
  //apd->AddInput(tfilter2->GetOutput());
  apd->Update();

  vtkSmartPointer<vtkTriangleFilter> cleaner=vtkSmartPointer<vtkTriangleFilter>::New();
  cleaner->SetInputConnection(apd->GetOutputPort());
  
  workspaceModel->SetAndObservePolyData(cleaner->GetOutput());

  double color[3];
  color[0] = 0.5;
  color[1] = 0.5;
  color[2] = 1.0;
  workspaceDisp->SetPolyData(workspaceModel->GetPolyData());
  workspaceDisp->SetColor(color);
  workspaceDisp->SetOpacity(0.5);
  
  trans1->Delete();
  tfilter1->Delete();
  cylinder1->Delete();
  apd->Delete();

  const char* modelID = workspaceModel->GetID();

  workspaceDisp->Delete();
  workspaceModel->Delete();

  return modelID;

}


//----------------------------------------------------------------------------
const char* vtkMRMLIGTProbeRobotNode::AddZFrameModel(const char* nodeName)
{
  vtkMRMLModelNode           *zframeModel;
  vtkMRMLModelDisplayNode    *zframeDisp;

  zframeModel = vtkMRMLModelNode::New();
  zframeDisp = vtkMRMLModelDisplayNode::New();

  this->Scene->SaveStateForUndo();
  this->Scene->AddNode(zframeDisp);
  this->Scene->AddNode(zframeModel);  

  zframeDisp->SetScene(this->Scene);
  zframeModel->SetName(nodeName);
  zframeModel->SetScene(this->Scene);
  zframeModel->SetAndObserveDisplayNodeID(zframeDisp->GetID());
  zframeModel->SetHideFromEditors(0);

  // construct Z-frame model
  const double length = 60; // mm

  //----- cylinder 1 (R-A) -----
  vtkCylinderSource *cylinder1 = vtkCylinderSource::New();
  cylinder1->SetRadius(1.5);
  cylinder1->SetHeight(length);
  cylinder1->SetCenter(0, 0, 0);
  cylinder1->Update();
  
  vtkTransformPolyDataFilter *tfilter1 = vtkTransformPolyDataFilter::New();
  vtkTransform* trans1 =   vtkTransform::New();
  trans1->Translate(length/2.0, length/2.0, 0.0);
  trans1->RotateX(90.0);
  trans1->Update();
  tfilter1->SetInput(cylinder1->GetOutput());
  tfilter1->SetTransform(trans1);
  tfilter1->Update();


  //----- cylinder 2 (R-center) -----
  vtkCylinderSource *cylinder2 = vtkCylinderSource::New();
  cylinder2->SetRadius(1.5);
  cylinder2->SetHeight(length*1.4142135);
  cylinder2->SetCenter(0, 0, 0);
  cylinder2->Update();

  vtkTransformPolyDataFilter *tfilter2 = vtkTransformPolyDataFilter::New();
  vtkTransform* trans2 =   vtkTransform::New();
  trans2->Translate(length/2.0, 0.0, 0.0);
  trans2->RotateX(90.0);
  trans2->RotateX(45.0);
  trans2->Update();
  tfilter2->SetInput(cylinder2->GetOutput());
  tfilter2->SetTransform(trans2);
  tfilter2->Update();


  //----- cylinder 3 (R-P) -----
  vtkCylinderSource *cylinder3 = vtkCylinderSource::New();
  cylinder3->SetRadius(1.5);
  cylinder3->SetHeight(length);
  cylinder3->SetCenter(0, 0, 0);
  cylinder3->Update();

  vtkTransformPolyDataFilter *tfilter3 = vtkTransformPolyDataFilter::New();
  vtkTransform* trans3 =   vtkTransform::New();
  trans3->Translate(length/2.0, -length/2.0, 0.0);
  trans3->RotateX(90.0);
  trans3->Update();
  tfilter3->SetInput(cylinder3->GetOutput());
  tfilter3->SetTransform(trans3);
  tfilter3->Update();


  //----- cylinder 4 (center-P) -----  
  vtkCylinderSource *cylinder4 = vtkCylinderSource::New();
  cylinder4->SetRadius(1.5);
  cylinder4->SetHeight(length*1.4142135);
  cylinder4->SetCenter(0, 0, 0);
  cylinder4->Update();

  vtkTransformPolyDataFilter *tfilter4 = vtkTransformPolyDataFilter::New();
  vtkTransform* trans4 =   vtkTransform::New();
  trans4->Translate(0.0, -length/2.0, 0.0);
  trans4->RotateX(90.0);
  trans4->RotateZ(45.0);
  trans4->Update();
  tfilter4->SetInput(cylinder4->GetOutput());
  tfilter4->SetTransform(trans4);
  tfilter4->Update();


  //----- cylinder 5 (L-P) -----  
  vtkCylinderSource *cylinder5 = vtkCylinderSource::New();
  cylinder5->SetRadius(1.5);
  cylinder5->SetHeight(length);
  cylinder5->SetCenter(0, 0, 0);
  cylinder5->Update();

  vtkTransformPolyDataFilter *tfilter5 = vtkTransformPolyDataFilter::New();
  vtkTransform* trans5 =   vtkTransform::New();
  trans5->Translate(-length/2.0, -length/2.0, 0.0);
  trans5->RotateX(90.0);
  trans5->Update();
  tfilter5->SetInput(cylinder5->GetOutput());
  tfilter5->SetTransform(trans5);
  tfilter5->Update();


  //----- cylinder 6 (L-center) -----  
  vtkCylinderSource *cylinder6 = vtkCylinderSource::New();
  cylinder6->SetRadius(1.5);
  cylinder6->SetHeight(length*1.4142135);
  cylinder6->SetCenter(0, 0, 0);
  cylinder6->Update();

  vtkTransformPolyDataFilter *tfilter6 = vtkTransformPolyDataFilter::New();
  vtkTransform* trans6 =   vtkTransform::New();
  trans6->Translate(-length/2.0, 0.0, 0.0);
  trans6->RotateX(90.0);
  //trans6->RotateX(45.0);
  trans6->RotateX(-45.0);
  trans6->Update();
  tfilter6->SetInput(cylinder6->GetOutput());
  tfilter6->SetTransform(trans6);
  tfilter6->Update();


  //----- cylinder 7 (L-A) -----  
  vtkCylinderSource *cylinder7 = vtkCylinderSource::New();
  cylinder7->SetRadius(1.5);
  cylinder7->SetHeight(length);
  cylinder7->SetCenter(0, 0, 0);
  cylinder7->Update();

  vtkTransformPolyDataFilter *tfilter7 = vtkTransformPolyDataFilter::New();
  vtkTransform* trans7 =   vtkTransform::New();
  trans7->Translate(-length/2.0, length/2.0, 0.0);
  trans7->RotateX(90.0);
  trans7->Update();
  tfilter7->SetInput(cylinder7->GetOutput());
  tfilter7->SetTransform(trans7);
  tfilter7->Update();


  vtkAppendPolyData *apd = vtkAppendPolyData::New();
  apd->AddInput(tfilter1->GetOutput());
  apd->AddInput(tfilter2->GetOutput());
  apd->AddInput(tfilter3->GetOutput());
  apd->AddInput(tfilter4->GetOutput());
  apd->AddInput(tfilter5->GetOutput());
  apd->AddInput(tfilter6->GetOutput());
  apd->AddInput(tfilter7->GetOutput());
  apd->Update();
  
  zframeModel->SetAndObservePolyData(apd->GetOutput());

  double color[3];
  color[0] = 1.0;
  color[1] = 0.5;
  color[2] = 0.0;
  zframeDisp->SetPolyData(zframeModel->GetPolyData());
  zframeDisp->SetColor(color);
  
  trans1->Delete();
  trans2->Delete();
  trans3->Delete();
  trans4->Delete();
  trans5->Delete();
  trans6->Delete();
  trans7->Delete();
  tfilter1->Delete();
  tfilter2->Delete();
  tfilter3->Delete();
  tfilter4->Delete();
  tfilter5->Delete();
  tfilter6->Delete();
  tfilter7->Delete();
  cylinder1->Delete();
  cylinder2->Delete();
  cylinder3->Delete();
  cylinder4->Delete();
  cylinder5->Delete();
  cylinder6->Delete();
  cylinder7->Delete();

  apd->Delete();

  const char* modelID = zframeModel->GetID();

  zframeDisp->Delete();
  zframeModel->Delete();  

  return modelID;
}

// 9/24/2011 ayamada
const char* vtkMRMLIGTProbeRobotNode::AddNeedleModel(const char* nodeName, int needleNumber)
{

  vtkMRMLModelNode           *workspaceModel;
  vtkMRMLModelDisplayNode    *workspaceDisp;
  
  workspaceModel = vtkMRMLModelNode::New();
  workspaceDisp  = vtkMRMLModelDisplayNode::New();
  
  this->Scene->SaveStateForUndo();
  this->Scene->AddNode(workspaceDisp);
  this->Scene->AddNode(workspaceModel);
  
  workspaceDisp->SetScene(this->Scene);
  workspaceModel->SetName(nodeName);
  workspaceModel->SetScene(this->Scene);
  workspaceModel->SetAndObserveDisplayNodeID(workspaceDisp->GetID());
  workspaceModel->SetHideFromEditors(0);
  
  // construct Z-frame model
  
  // Parameters
  // offset from z-frame -- this will be a class variable to let users configure it in the future.
  //const double offsetFromZFrame[] = {0, 22.76, 150.0};
  const double length = 250.0;//330.0;//15.0;//200.0;
  // 11/19/2011 ayamada
  const double offsetFromZFrame[] = {0, 0, int(length/2.0)};
  
  //----- cylinder 1 (R-A) -----
  vtkCylinderSource *cylinder1 = vtkCylinderSource::New();
  vtkTransformPolyDataFilter *tfilter1 = vtkTransformPolyDataFilter::New();
  vtkTransform* trans1 =   vtkTransform::New();
  vtkAppendPolyData *apd = vtkAppendPolyData::New();
  
  cylinder1->SetResolution(100);
  cylinder1->SetRadius(0.7365);//SetRadius(0.5);
  cylinder1->SetHeight(length);
  cylinder1->SetCenter(0, 0, 0);
  cylinder1->Update();

  trans1->Translate(offsetFromZFrame);
  trans1->RotateX(90.0);
  trans1->Update();
  
  tfilter1->SetInput(cylinder1->GetOutput());
  tfilter1->SetTransform(trans1);
  tfilter1->Update();

  apd->AddInput(tfilter1->GetOutput());
  apd->Update();
  vtkSmartPointer<vtkTriangleFilter> cleaner=vtkSmartPointer<vtkTriangleFilter>::New();
  cleaner->SetInputConnection(apd->GetOutputPort());
  
  workspaceModel->SetAndObservePolyData(cleaner->GetOutput());
  
  double color[3];
  
  switch(needleNumber)
  {
  case 0:
      color[0] = 0.0;//0.5;
      color[1] = 0.0;//0.5;
    color[2] = 1.0;
    break;
  case 1:
    color[0] = 1.0;
      color[1] = 0.0;//0.5;
      color[2] = 0.0;//0.5;
    break;
  case 2:
      color[0] = 0.0;//0.5;
    color[1] = 1.0;
      color[2] = 0.0;//0.5;
    break;
  }
    
  workspaceDisp->SetPolyData(workspaceModel->GetPolyData());
  workspaceDisp->SetColor(color);
  workspaceDisp->SetOpacity(1.0/*0.5*/);

  // 11/20/2011
  workspaceDisp->SetAmbient(70);
  workspaceDisp->SetDiffuse(100);
  
  
  trans1->Delete();
  tfilter1->Delete();
  cylinder1->Delete();

  apd->Delete();
  
  const char* modelID = workspaceModel->GetID();
  
  workspaceDisp->Delete();
  workspaceModel->Delete();
  
  return modelID;  
  
}


// 11/3/2011 ayamada
const char* vtkMRMLIGTProbeRobotNode::AddVirtualCenterModel(const char* nodeName)
{
  
  vtkMRMLModelNode           *workspaceModel;
  vtkMRMLModelDisplayNode    *workspaceDisp;
  
  workspaceModel = vtkMRMLModelNode::New();
  workspaceDisp  = vtkMRMLModelDisplayNode::New();
  
  this->Scene->SaveStateForUndo();
  this->Scene->AddNode(workspaceDisp);
  this->Scene->AddNode(workspaceModel);
  
  workspaceDisp->SetScene(this->Scene);
  workspaceModel->SetName(nodeName);
  workspaceModel->SetScene(this->Scene);
  workspaceModel->SetAndObserveDisplayNodeID(workspaceDisp->GetID());
  workspaceModel->SetHideFromEditors(0);
  
  // construct Z-frame model
  
  // Parameters
  // offset from z-frame -- this will be a class variable to let users configure it in the future.
  //const double offsetFromZFrame[] = {0, 22.76, 150.0};
  
  // 11/4/2011 ayamada
  //const double length = 15.0;//200.0;
  const double offsetFromZFrame[] = {0, 0, 0};
    
  // 11/3/2011 ayamada
  vtkSphereSource *sphere1 = vtkSphereSource::New(); 
  vtkSphereSource *sphere2 = vtkSphereSource::New(); 
  
  vtkTransform* trans1 =   vtkTransform::New();
  vtkTransform* trans2 =   vtkTransform::New();
  vtkAppendPolyData *apd = vtkAppendPolyData::New();

  vtkTransformPolyDataFilter *tfilter1 = vtkTransformPolyDataFilter::New();
  vtkTransformPolyDataFilter *tfilter2 = vtkTransformPolyDataFilter::New();
  
  
  sphere1->SetCenter(0,0,0);
  sphere1->SetRadius(1.5);
  sphere1->SetThetaResolution(100);
  sphere1->SetPhiResolution(100);
  
  sphere1->Update();

  sphere2->SetCenter(0,0,0);
  sphere2->SetRadius(3.0); // 3.0// 11/18/2011 ayamada
  sphere2->SetThetaResolution(100);
  sphere2->SetPhiResolution(100);
  sphere2->Update();
  
  
  //vtkTransform* trans1 =   vtkTransform::New();
  trans1->Translate(offsetFromZFrame);
  trans1->RotateX(90.0);
  trans1->Update();
  
  //vtkTransformPolyDataFilter *tfilter1 = vtkTransformPolyDataFilter::New();
  tfilter1->SetInput(sphere1->GetOutput());
  tfilter1->SetTransform(trans1);
  tfilter1->Update();
  
  tfilter2->SetInput(sphere2->GetOutput());
  tfilter2->SetTransform(trans1);
  tfilter2->Update();
  
  //vtkAppendPolyData *apd = vtkAppendPolyData::New();
  apd->AddInput(tfilter1->GetOutput());
  apd->AddInput(tfilter2->GetOutput());
  apd->Update();
  //}
  
  vtkSmartPointer<vtkTriangleFilter> cleaner=vtkSmartPointer<vtkTriangleFilter>::New();
  cleaner->SetInputConnection(apd->GetOutputPort());
  
  workspaceModel->SetAndObservePolyData(cleaner->GetOutput());
  
  double color[3];
  
      color[0] = 0.5;
      color[1] = 0.5;
      color[2] = 1.0;
  
  workspaceDisp->SetPolyData(workspaceModel->GetPolyData());
  workspaceDisp->SetColor(color);
  workspaceDisp->SetOpacity(0.6);
  
  trans1->Delete();
  tfilter1->Delete();
  sphere1->Delete();

  trans2->Delete();
  tfilter2->Delete();
  sphere2->Delete();
  
  apd->Delete();
  
  const char* modelID = workspaceModel->GetID();
  
  workspaceDisp->Delete();
  workspaceModel->Delete();
  
  return modelID;  
  
}


// 9/22/2011 ayamada
//----------------------------------------------------------------------------
const char* vtkMRMLIGTProbeRobotNode::AddMITRobotModel(const char* nodeName, int partsNumber)
{

  // --------------------------------------------
  // registor to MRML 
  // --------------------------------------------
  vtkMRMLModelNode           *robotModel;
  vtkMRMLModelDisplayNode    *robotDisp;
  
  robotModel = vtkMRMLModelNode::New();
  robotDisp = vtkMRMLModelDisplayNode::New();

  this->Scene->SaveStateForUndo();
  this->Scene->AddNode(robotDisp);
  this->Scene->AddNode(robotModel);  
  
  robotDisp->SetScene(this->Scene);
  robotModel->SetName(nodeName);
  robotModel->SetScene(this->Scene);
  robotModel->SetAndObserveDisplayNodeID(robotDisp->GetID());
  robotModel->SetHideFromEditors(0);
  // --------------------------------------------
  
  // ---------------------
  //----------------------------------------------
  //vtkRobotProbeNavLogic *logic=this->GetGUI()->GetLogic();

  // 9/22/2011 ayamada
  vtkSmartPointer <vtkSTLReader> modelReader=vtkSmartPointer<vtkSTLReader>::New();
  vtksys_stl::string modelFileName="";

  // 3/8/2012 ayamada
  vtksys_stl::string imageFilePath="";
  vtkRobotProbeNavLogic* navLogic = vtkRobotProbeNavLogic::New();
  //navLogic->ModuleShareDirectory = this->ModuleShareDirectory;
  imageFilePath = this->ModuleShareDirectory;
  //navLogic->imageFilePath = imageFilePath.c_str();
  
  switch(partsNumber)
  {
    case 0:
  
      this->calcPath2("new-loopCoil.stl");
      std::cout << "correct path = " << this->imageFilePath << endl;
      //std::cout << "path = " << this->ModuleShareDirectory << endl;
      
      modelFileName = this->imageFilePath;
      robotDisp->SetOpacity(1.0);
      //robotDisp->SetColor(0.5,1,1);
      robotDisp->SetColor(1.0,1.0,1.0);
      modelReader->SetFileName(modelFileName.c_str());
      modelReader->Update();
      break;
    case 1:
      //modelFileName=this->ModuleShareDirectory+"/MITRobot/prototype-baseFiducialCenter.stl";
      //modelFileName=this->ModuleShareDirectory+"/MITRobot/baseTest3.stl";

      this->calcPath2("new-bodyMechanisms.stl");
      //this->calcPath2("prototype-baseFid.stl");
      std::cout << "correct path = " << this->imageFilePath << endl;
      //std::cout << "path = " << this->ModuleShareDirectory << endl;
      
      modelFileName = this->imageFilePath;
      
      robotDisp->SetOpacity(0.7/*1.0*/);
      robotDisp->SetColor(0.5,0.5,0.5);
      modelReader->SetFileName(modelFileName.c_str());
      modelReader->Update();
      break;
    case 2:
      //modelFileName=this->ModuleShareDirectory+"/MITRobot/prototype-baseSimp.stl";
      //modelFileName=this->ModuleShareDirectory+"/MITRobot/baseTest3.stl";

      this->calcPath2("new-motorMechanisms.stl");
      //this->calcPath2("prototype-baseSimp.stl");
      std::cout << "correct path = " << this->imageFilePath << endl;
      //std::cout << "path = " << this->ModuleShareDirectory << endl;
      
      modelFileName = this->imageFilePath;
      
      robotDisp->SetOpacity(0.4/*0.9*/);
      //robotDisp->SetColor(0,0.3,1);
      robotDisp->SetColor(0,0.3,0.7);
      //robotDisp->SetColor(1.0,1.0,1.0);
      modelReader->SetFileName(modelFileName.c_str());
      modelReader->Update();
      break;
    case 3:
      //modelFileName=this->ModuleShareDirectory+"/MITRobot/prototype-hoopRev.stl";

      this->calcPath2("new-hooperMechanisms.stl");
      //this->calcPath2("prototype-hoopRev.stl");
      std::cout << "correct path = " << this->imageFilePath << endl;
      //std::cout << "path = " << this->ModuleShareDirectory << endl;
      
      modelFileName = this->imageFilePath;
      
      robotDisp->SetOpacity(0.6/*1.0*/);
      //robotDisp->SetColor(0.7,0.7,0.7);
      robotDisp->SetColor(0.7,0.5,0.0);
      modelReader->SetFileName(modelFileName.c_str());
      modelReader->Update();
      break;
    case 4:
      //modelFileName=this->ModuleShareDirectory+"/MITRobot/prototype-needleHolder.stl";

      this->calcPath2("new-needleBoxMechanisms.stl");
      //this->calcPath2("prototype-needleHolder.stl");
      std::cout << "correct path = " << this->imageFilePath << endl;
      //std::cout << "path = " << this->ModuleShareDirectory << endl;

      modelFileName = this->imageFilePath;
      
      robotDisp->SetOpacity(1.0/*0.4*/);
      //robotDisp->SetColor(1.2,1.2,1.2);
      robotDisp->SetColor(0,0.5,0.7);
      modelReader->SetFileName(modelFileName.c_str());
      modelReader->Update();
      break;
    //default:
    //  break;
  }
 
  vtkSmartPointer<vtkTransformPolyDataFilter> polyTrans = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  polyTrans->SetInputConnection(modelReader->GetOutputPort());
  vtkSmartPointer<vtkTransform> modelTransform=vtkSmartPointer<vtkTransform>::New();
  polyTrans->SetTransform(modelTransform);
  
  GetRobotBaseTransform(modelTransform->GetMatrix());

  vtkSmartPointer<vtkTriangleFilter> cleaner=vtkSmartPointer<vtkTriangleFilter>::New();
  cleaner->SetInputConnection(polyTrans->GetOutputPort());
  cleaner->Update();
  
  // to registor the .stl model to MRML
  robotModel->SetAndObservePolyData(cleaner->GetOutput());
  robotModel->SetModifiedSinceRead(1);
  robotDisp->SetModifiedSinceRead(1); 
  
  // 9/25/2011 ayamada
  robotDisp->SliceIntersectionVisibilityOff();
  //robotDisp->SliceIntersectionVisibilityOn();
  //robotDisp->SetSliceIntersectionVisibility(show);  // vtkRobotProbeNavLogic.cxx l.795
  robotDisp->VisibilityOn();
  
  const char* modelID = robotModel->GetID();
  return modelID;
}



//----------------------------------------------------------------------------
int vtkMRMLIGTProbeRobotNode::PerformRegistration(vtkMRMLScalarVolumeNode* volumeNode)
{
  vtkZFrameRobotToImageRegistration* registration = vtkZFrameRobotToImageRegistration::New();
  registration->SetFiducialVolume(volumeNode);

  // Set base Z-frame orientation
  vtkMatrix4x4 * baseOrientation;
  baseOrientation = vtkMatrix4x4::New();
  baseOrientation->Identity();
  baseOrientation->SetElement(0, 0, -1.0);
  baseOrientation->SetElement(1, 1, -1.0);
  baseOrientation->SetElement(2, 2, 1.0);

  registration->SetZFrameBaseOrientation(baseOrientation);

  vtkMRMLLinearTransformNode* transformNode = vtkMRMLLinearTransformNode::SafeDownCast(this->Scene->GetNodeByID(this->GetZFrameTransformNodeID()));
  if (transformNode != NULL)
    {
    registration->SetRobotToImageTransform(transformNode);
    registration->DoRegistration();
    
    // 11/4/2011 ayamada for test
    //this->GetLogic()->SendZFrame();

    std::cerr << "Sending Z-frame Data" << std::endl;
    if (this->GetRobotCommandNode()==NULL)
      {
      return 0;
      }
    this->GetRobotCommandNode()->SetZFrameTransformNodeID(transformNode->GetID());
    this->GetRobotCommandNode()->PushOutgoingCommand("SET_Z_FRAME");
    this->GetRobotCommandNode()->InvokeEvent(vtkCommand::ModifiedEvent);
    }

  return 1;
}


//----------------------------------------------------------------------------

int vtkMRMLIGTProbeRobotNode::PerformRegistration(vtkMRMLScalarVolumeNode* volumeNode, int param1, int param2)
{
  vtkZFrameRobotToImageRegistration* registration = vtkZFrameRobotToImageRegistration::New();
  registration->SetFiducialVolume(volumeNode);

  // Set base Z-frame orientation
  vtkMatrix4x4 * baseOrientation;
  baseOrientation = vtkMatrix4x4::New();
  baseOrientation->Identity();
  baseOrientation->SetElement(0, 0, -1.0);
  baseOrientation->SetElement(1, 1, -1.0);
  baseOrientation->SetElement(2, 2, 1.0);

  registration->SetZFrameBaseOrientation(baseOrientation);

  vtkMRMLLinearTransformNode* transformNode = vtkMRMLLinearTransformNode::SafeDownCast(this->Scene->GetNodeByID(this->GetZFrameTransformNodeID()));
  if (transformNode != NULL)
    {
    registration->SetRobotToImageTransform(transformNode);
    registration->SetSliceRange(param1, param2);
    registration->DoRegistration();

    std::cerr << "Sending Z-frame Data" << std::endl;
    if (this->GetRobotCommandNode()==NULL)
      {
      return 0;
      }
    this->GetRobotCommandNode()->SetZFrameTransformNodeID(transformNode->GetID());
    this->GetRobotCommandNode()->PushOutgoingCommand("SET_Z_FRAME");
    this->GetRobotCommandNode()->InvokeEvent(vtkCommand::ModifiedEvent);
    }

  return 1;
}



//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::SwitchStep(const char *stepName)
{
  if (this->GetRobotCommandNode())
    {
    this->GetRobotCommandNode()->SwitchStep(stepName);
    }
}


//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::SetAndObserveRobotCommandNodeID(const char *nodeId)
{
if (nodeId==this->GetRobotCommandNodeID())
  {
    // no change
    return;
  }
  if (this->GetRobotCommandNodeID()!=NULL && nodeId!=NULL && strcmp(nodeId, this->GetRobotCommandNodeID())==0)
  {
    // no change
    return;
  }
  vtkSetAndObserveMRMLObjectMacro(this->RobotCommandNode, NULL);
  this->SetRobotCommandNodeID(nodeId);
  vtkMRMLBrpRobotCommandNode *tnode = this->GetRobotCommandNode();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->RobotCommandNode, tnode, events);
  this->Modified();
}

//----------------------------------------------------------------------------
vtkMRMLBrpRobotCommandNode* vtkMRMLIGTProbeRobotNode::GetRobotCommandNode()
{
  if (this->GetScene() && this->RobotCommandNodeID != NULL )
    {    
    return vtkMRMLBrpRobotCommandNode::SafeDownCast(this->GetScene()->GetNodeByID(this->RobotCommandNodeID));
    }
  return NULL;
}

//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::SetAndObserveRobotConnectorNodeID(const char *nodeId)
{
  if (nodeId==this->GetRobotConnectorNodeID())
  {
    // no change
    return;
  }
  if (this->GetRobotConnectorNodeID()!=NULL && nodeId!=NULL && strcmp(nodeId, this->GetRobotConnectorNodeID())==0)
  {
    // no change
    return;
  }

  vtkSetAndObserveMRMLObjectMacro(this->RobotConnectorNode, NULL);
  this->SetRobotConnectorNodeID(nodeId);
  vtkMRMLIGTLConnectorNode *tnode = this->GetRobotConnectorNode();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkMRMLIGTLConnectorNode::ConnectedEvent);
  events->InsertNextValue(vtkMRMLIGTLConnectorNode::DisconnectedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->RobotConnectorNode, tnode, events);

  // 1/22/2012 ayamada
  /*
  if ( this->GetRobotConnectorNode() && this->GetRobotCommandNode() )
  {
    this->GetRobotConnectorNode()->RegisterOutgoingMRMLNode( this->GetRobotCommandNode() );
    this->GetRobotConnectorNode()->RegisterIncomingMRMLNode( this->GetRobotCommandNode() );    
  }
  */
  
  //this->GetRobotCommandNode()->SetTargetTransformNodeID(transformNodeId);
  //this->GetRobotCommandNode()->PushOutgoingCommand("MOVE_TO");
  //this->GetRobotCommandNode()->InvokeEvent(vtkCommand::ModifiedEvent);
  
  
  // 1/21/2012 ayamada
  if ( this->GetRobotConnectorNode() && this->GetMarkersPositionTransformNode() && this->GetMarkersPosition2TransformNode() )
  {
    this->GetRobotConnectorNode()->RegisterOutgoingMRMLNode( this->GetMarkersPositionTransformNode() );
    // 1/22/2012 ayamada
    //this->GetRobotCommandNode()->PushOutgoingCommand("MOVE_TO");
    //this->GetRobotCommandNode()->InvokeEvent(vtkCommand::ModifiedEvent);
    this->GetRobotConnectorNode()->RegisterIncomingMRMLNode( this->GetMarkersPosition2TransformNode() );
  }
  
  

  this->Modified();
}


//----------------------------------------------------------------------------
vtkMRMLIGTLConnectorNode* vtkMRMLIGTProbeRobotNode::GetRobotConnectorNode()
{
  if (this->GetScene() && this->GetRobotConnectorNodeID()!=NULL )
    {    
    return vtkMRMLIGTLConnectorNode::SafeDownCast(this->GetScene()->GetNodeByID(this->GetRobotConnectorNodeID()));
    }
  return NULL;
}

//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::SetAndObserveScannerConnectorNodeID(const char *nodeId)
{
  if (nodeId==this->GetScannerConnectorNodeID())
  {
    // no change
    return;
  }
  if (this->GetScannerConnectorNodeID()!=NULL && nodeId!=NULL && strcmp(nodeId, this->GetScannerConnectorNodeID())==0)
  {
    // no change
    return;
  }
  vtkSetAndObserveMRMLObjectMacro(this->ScannerConnectorNode, NULL);
  this->SetScannerConnectorNodeID(nodeId);
  vtkMRMLIGTLConnectorNode *tnode = this->GetScannerConnectorNode();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkMRMLIGTLConnectorNode::ConnectedEvent);
  events->InsertNextValue(vtkMRMLIGTLConnectorNode::DisconnectedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->ScannerConnectorNode, tnode, events);
}


//----------------------------------------------------------------------------
vtkMRMLIGTLConnectorNode* vtkMRMLIGTProbeRobotNode::GetScannerConnectorNode()
{
  if (this->GetScene() && this->ScannerConnectorNodeID != NULL )
    {    
    return vtkMRMLIGTLConnectorNode::SafeDownCast(this->GetScene()->GetNodeByID(this->ScannerConnectorNodeID));
    }
  return NULL;
}

//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::SetAndObserveZFrameModelNodeID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->ZFrameModelNode, NULL);
  this->SetZFrameModelNodeID(nodeId);
  vtkMRMLModelNode *tnode = this->GetZFrameModelNode();

  // 9/21/2011 ayamada
  //vtkMRMLModelNode *mrnode = this->GetMITRobotModelNode();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->ZFrameModelNode, tnode, events);
  
  // 9/21/2011 ayamada
  //vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotModelNode, mrnode, events);
  
}


// 9/21/2011 ayamada
//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotModelNodeID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotModelNode, NULL);
  this->SetMITRobotModelNodeID(nodeId);
  
  // 9/21/2011 ayamada
  vtkMRMLModelNode *mrnode = this->GetMITRobotModelNode();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  
  // 9/21/2011 ayamada
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotModelNode, mrnode, events);
  
}

// 9/23/2011 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotModelNode2ID(const char *nodeId)
{

  vtkSetAndObserveMRMLObjectMacro(this->MITRobotModelNode2, NULL);
  this->SetMITRobotModelNode2ID(nodeId);
  
  // 9/21/2011 ayamada
  vtkMRMLModelNode *mrnode = this->GetMITRobotModelNode2();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  
  // 9/21/2011 ayamada
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotModelNode2, mrnode, events);
  
}

// 9/23/2011 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotModelNode3ID(const char *nodeId)
{
  
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotModelNode3, NULL);
  this->SetMITRobotModelNode3ID(nodeId);
  
  // 9/21/2011 ayamada
  vtkMRMLModelNode *mrnode = this->GetMITRobotModelNode3();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  
  // 9/21/2011 ayamada
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotModelNode3, mrnode, events);
  
}

// 9/23/2011 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotModelNode4ID(const char *nodeId)
{
  
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotModelNode4, NULL);
  this->SetMITRobotModelNode4ID(nodeId);
  
  // 9/21/2011 ayamada
  vtkMRMLModelNode *mrnode = this->GetMITRobotModelNode4();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  
  // 9/21/2011 ayamada
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotModelNode4, mrnode, events);
  
}

// 9/23/2011 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotModelNode5ID(const char *nodeId)
{
  
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotModelNode5, NULL);
  this->SetMITRobotModelNode5ID(nodeId);
  
  // 9/21/2011 ayamada
  vtkMRMLModelNode *mrnode = this->GetMITRobotModelNode5();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  
  // 9/21/2011 ayamada
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotModelNode5, mrnode, events);
  
}


// 9/24/2011 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveNeedleModelNodeID(const char *nodeId)
{
  
  vtkSetAndObserveMRMLObjectMacro(this->NeedleModelNode, NULL);
  this->SetNeedleModelNodeID(nodeId);
  
  // 9/21/2011 ayamada
  vtkMRMLModelNode *mrnode = this->GetNeedleModelNode();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  
  // 9/21/2011 ayamada
  vtkSetAndObserveMRMLObjectEventsMacro(this->NeedleModelNode, mrnode, events);
  
}

void vtkMRMLIGTProbeRobotNode::SetAndObserveNeedleModelNode1ID(const char *nodeId)
{
  
  vtkSetAndObserveMRMLObjectMacro(this->NeedleModelNode1, NULL);
  this->SetNeedleModelNode1ID(nodeId);
  
  // 9/21/2011 ayamada
  vtkMRMLModelNode *mrnode = this->GetNeedleModelNode1();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  
  // 9/21/2011 ayamada
  vtkSetAndObserveMRMLObjectEventsMacro(this->NeedleModelNode1, mrnode, events);
  
}

void vtkMRMLIGTProbeRobotNode::SetAndObserveNeedleModelNode2ID(const char *nodeId)
{
  
  vtkSetAndObserveMRMLObjectMacro(this->NeedleModelNode2, NULL);
  this->SetNeedleModelNode2ID(nodeId);
  
  // 9/21/2011 ayamada
  vtkMRMLModelNode *mrnode = this->GetNeedleModelNode2();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  
  // 9/21/2011 ayamada
  vtkSetAndObserveMRMLObjectEventsMacro(this->NeedleModelNode2, mrnode, events);
  
}

// 11/3/2011 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveVirtualCenterModelNodeID(const char *nodeId)
{
  
  vtkSetAndObserveMRMLObjectMacro(this->VirtualCenterModelNode, NULL);
  this->SetVirtualCenterModelNodeID(nodeId);
  
  // 9/21/2011 ayamada
  vtkMRMLModelNode *mrnode = this->GetVirtualCenterModelNode();
  
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  
  // 9/21/2011 ayamada
  vtkSetAndObserveMRMLObjectEventsMacro(this->VirtualCenterModelNode, mrnode, events);
  
}



//----------------------------------------------------------------------------
vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetZFrameModelNode()
{
  if (this->GetScene() && this->ZFrameModelNodeID != NULL )
    {    
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->ZFrameModelNodeID));
    }
  return NULL;
}

// 9/21/2011 ayamada
vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetMITRobotModelNode()
{

  if (this->GetScene() && this->MITRobotModelNodeID != NULL )
  {    
    // 9/22/2011 ayamada
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotModelNodeID));
  }
  return NULL;
}

// 9/23/2011 ayamada
vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetMITRobotModelNode2()
{
  
  if (this->GetScene() && this->MITRobotModelNode2ID != NULL )
  {    
    // 9/22/2011 ayamada
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotModelNode2ID));
  }
  return NULL;
}

// 9/23/2011 ayamada
vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetMITRobotModelNode3()
{
  
  if (this->GetScene() && this->MITRobotModelNode3ID != NULL )
  {    
    // 9/22/2011 ayamada
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotModelNode3ID));
  }
  return NULL;
}

// 9/23/2011 ayamada
vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetMITRobotModelNode4()
{
  
  if (this->GetScene() && this->MITRobotModelNode4ID != NULL )
  {    
    // 9/22/2011 ayamada
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotModelNode4ID));
  }
  return NULL;
}

// 9/23/2011 ayamada
vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetMITRobotModelNode5()
{
  
  if (this->GetScene() && this->MITRobotModelNode5ID != NULL )
  {    
    // 9/22/2011 ayamada
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotModelNode5ID));
  }
  return NULL;
}


// 9/24/2011 ayamada
vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetNeedleModelNode()
{
  
  if (this->GetScene() && this->NeedleModelNodeID != NULL )
  {    
    // 9/22/2011 ayamada
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->NeedleModelNodeID));
  }
  return NULL;
}

vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetNeedleModelNode1()
{
  
  if (this->GetScene() && this->NeedleModelNode1ID != NULL )
  {    
    // 9/22/2011 ayamada
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->NeedleModelNode1ID));
  }
  return NULL;
}

vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetNeedleModelNode2()
{
  
  if (this->GetScene() && this->NeedleModelNode2ID != NULL )
  {    
    // 9/22/2011 ayamada
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->NeedleModelNode2ID));
  }
  return NULL;
}

// 11/3/2011 ayamada
vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetVirtualCenterModelNode()
{
  
  if (this->GetScene() && this->VirtualCenterModelNodeID != NULL )
  {    
    // 9/22/2011 ayamada
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->VirtualCenterModelNodeID));
  }
  return NULL;
}



//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::SetAndObserveWorkspaceModelNodeID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->WorkspaceModelNode, NULL);
  this->SetWorkspaceModelNodeID(nodeId);
  vtkMRMLModelNode *tnode = this->GetWorkspaceModelNode();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->WorkspaceModelNode, tnode, events);
}

//----------------------------------------------------------------------------
vtkMRMLModelNode* vtkMRMLIGTProbeRobotNode::GetWorkspaceModelNode()
{
  if (this->GetScene() && this->WorkspaceModelNodeID != NULL )
    {    
    return vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetNodeByID(this->WorkspaceModelNodeID));
    }
  return NULL;
}


//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::SetAndObserveZFrameTransformNodeID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->ZFrameTransformNode, NULL);
  this->SetZFrameTransformNodeID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetZFrameTransformNode();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->ZFrameTransformNode, tnode, events);
}

// 10/6/2011 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveNeedlePathTransformNode1ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->NeedlePathTransformNode1, NULL);
  this->SetNeedlePathTransformNode1ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetNeedlePathTransformNode1();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->NeedlePathTransformNode1, tnode, events);
}

void vtkMRMLIGTProbeRobotNode::SetAndObserveNeedlePathTransformNode2ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->NeedlePathTransformNode2, NULL);
  this->SetNeedlePathTransformNode2ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetNeedlePathTransformNode2();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->NeedlePathTransformNode2, tnode, events);
}

void vtkMRMLIGTProbeRobotNode::SetAndObserveNeedlePathTransformNode3ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->NeedlePathTransformNode3, NULL);
  this->SetNeedlePathTransformNode3ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetNeedlePathTransformNode3();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->NeedlePathTransformNode3, tnode, events);
}

// 2/9/2012 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveTargetPlaneTransformNodeID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->TargetPlaneTransformNode, NULL);
  this->SetTargetPlaneTransformNodeID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetTargetPlaneTransformNode();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->TargetPlaneTransformNode, tnode, events);
}

void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotTransformNode1ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotTransformNode1, NULL);
  this->SetMITRobotTransformNode1ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetMITRobotTransformNode1();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotTransformNode1, tnode, events);
}
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotTransformNode2ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotTransformNode2, NULL);
  this->SetMITRobotTransformNode2ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetMITRobotTransformNode2();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotTransformNode2, tnode, events);
}
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotTransformNode3ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotTransformNode3, NULL);
  this->SetMITRobotTransformNode3ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetMITRobotTransformNode3();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotTransformNode3, tnode, events);
}
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotTransformNode4ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotTransformNode4, NULL);
  this->SetMITRobotTransformNode4ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetMITRobotTransformNode4();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotTransformNode4, tnode, events);
}
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotTransformNode5ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotTransformNode5, NULL);
  this->SetMITRobotTransformNode5ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetMITRobotTransformNode5();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotTransformNode5, tnode, events);
}

// 7/21/2012 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotTransformNode6ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotTransformNode6, NULL);
  this->SetMITRobotTransformNode6ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetMITRobotTransformNode6();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotTransformNode6, tnode, events);
}
void vtkMRMLIGTProbeRobotNode::SetAndObserveMITRobotTransformNode7ID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MITRobotTransformNode7, NULL);
  this->SetMITRobotTransformNode7ID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetMITRobotTransformNode7();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->MITRobotTransformNode7, tnode, events);
}


// 11/4/2011 ayamada
void vtkMRMLIGTProbeRobotNode::SetAndObserveVirtualCenterTransformNodeID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->VirtualCenterTransformNode, NULL);
  this->SetVirtualCenterTransformNodeID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetVirtualCenterTransformNode();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->VirtualCenterTransformNode, tnode, events);
}




// 9/13/2011 ayamada
//----------------------------------------------------------------------------
void vtkMRMLIGTProbeRobotNode::SetAndObserveMarkersPositionTransformNodeID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MarkersPositionTransformNode, NULL);
  this->SetMarkersPositionTransformNodeID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetMarkersPositionTransformNode();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->MarkersPositionTransformNode, tnode, events);
}

void vtkMRMLIGTProbeRobotNode::SetAndObserveMarkersPosition2TransformNodeID(const char *nodeId)
{
  vtkSetAndObserveMRMLObjectMacro(this->MarkersPosition2TransformNode, NULL);
  this->SetMarkersPosition2TransformNodeID(nodeId);
  vtkMRMLLinearTransformNode *tnode = this->GetMarkersPosition2TransformNode();
  vtkSmartPointer<vtkIntArray> events = vtkSmartPointer<vtkIntArray>::New();
  events->InsertNextValue(vtkCommand::ModifiedEvent);
  vtkSetAndObserveMRMLObjectEventsMacro(this->MarkersPosition2TransformNode, tnode, events);
}




//----------------------------------------------------------------------------
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetZFrameTransformNode()
{
  if (this->GetScene() && this->ZFrameTransformNodeID != NULL )
    {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->ZFrameTransformNodeID));
    }
  return NULL;
}

// 10/6/2011 ayamada
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetNeedlePathTransformNode1()
{
  if (this->GetScene() && this->NeedlePathTransformNode1ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->NeedlePathTransformNode1ID));
  }
  return NULL;
}
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetNeedlePathTransformNode2()
{
  if (this->GetScene() && this->NeedlePathTransformNode2ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->NeedlePathTransformNode2ID));
  }
  return NULL;
}
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetNeedlePathTransformNode3()
{
  if (this->GetScene() && this->NeedlePathTransformNode3ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->NeedlePathTransformNode3ID));
  }
  return NULL;
}

// 2/9/2012 ayamada
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetTargetPlaneTransformNode()
{
  if (this->GetScene() && this->TargetPlaneTransformNodeID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->TargetPlaneTransformNodeID));
  }
  return NULL;
}

vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetMITRobotTransformNode1()
{
  if (this->GetScene() && this->MITRobotTransformNode1ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotTransformNode1ID));
  }
  return NULL;
}
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetMITRobotTransformNode2()
{
  if (this->GetScene() && this->MITRobotTransformNode2ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotTransformNode2ID));
  }
  return NULL;
}
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetMITRobotTransformNode3()
{
  if (this->GetScene() && this->MITRobotTransformNode3ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotTransformNode3ID));
  }
  return NULL;
}
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetMITRobotTransformNode4()
{
  if (this->GetScene() && this->MITRobotTransformNode4ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotTransformNode4ID));
  }
  return NULL;
}
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetMITRobotTransformNode5()
{
  if (this->GetScene() && this->MITRobotTransformNode5ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotTransformNode5ID));
  }
  return NULL;
}
// 7/21/2012 ayamada
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetMITRobotTransformNode6()
{
  if (this->GetScene() && this->MITRobotTransformNode6ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotTransformNode6ID));
  }
  return NULL;
}
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetMITRobotTransformNode7()
{
  if (this->GetScene() && this->MITRobotTransformNode7ID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MITRobotTransformNode7ID));
  }
  return NULL;
}



// 11/4/2011 ayamada
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetVirtualCenterTransformNode()
{
  if (this->GetScene() && this->VirtualCenterTransformNodeID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->VirtualCenterTransformNodeID));
  }
  return NULL;
}




// 9/13/2011 ayamada
//----------------------------------------------------------------------------
vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetMarkersPositionTransformNode()
{
  if (this->GetScene() && this->MarkersPositionTransformNodeID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MarkersPositionTransformNodeID));
  }
  return NULL;
}

vtkMRMLLinearTransformNode* vtkMRMLIGTProbeRobotNode::GetMarkersPosition2TransformNode()
{
  if (this->GetScene() && this->MarkersPosition2TransformNodeID != NULL )
  {    
    return vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MarkersPosition2TransformNodeID));
  }
  return NULL;
}

// 9/14/2011 ayamada
void vtkMRMLIGTProbeRobotNode::sendImagePositionData(void)
{
  
  //vtkMRMLLinearTransformNode* transformNode = 
  //vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->MarkersPositionTransformNodeID));
  //vtkMRMLLinearTransformNode* transformNode = 
//vtkMRMLLinearTransformNode::SafeDownCast(this->GetScene()->GetNodeByID(this->ZFrameTransformNodeID));
  
  //vtkMRMLLinearTransformNode::SafeDownCast(this->Scene->GetNodeByID(this->GetMarkersPositionTransformNodeID()));
  //vtkMRMLLinearTransformNode* transformNode = this->GetMarkersPositionTransformNode();
  
  this->GetRobotCommandNode()->SetZFrameTransformNodeID(this->GetZFrameTransformNodeID());
  //std::cerr << "Test2!!" << std::endl;   

  
  
}

//----------------------------------------------------------------------------
std::string vtkMRMLIGTProbeRobotNode::GetTargetInfoText(vtkRobotProbeNavTargetDescriptor *targetDesc, NeedleDescriptorStruct *needle)
{
  // :TODO: construct a string that contains useful information for the current target (reachable, etc.)
  return "";
}


// --------
// 6/29/2012 ayamada
void vtkMRMLIGTProbeRobotNode::calcPath2(const char* path)
{
  
  vtksys_stl::string imageFilePath="";
  vtkRobotProbeNavLogic* navLogic = vtkRobotProbeNavLogic::New();
  imageFilePath = this->ModuleShareDirectory;
  navLogic->imageFilePath = imageFilePath.c_str();      
  
  vtkMRMLRobotNode *robotNode = vtkMRMLRobotNode::New();
  vtksys_stl::string filename="";
    
  filename = navLogic->imageFilePath;//this->GetLogic()->GetModuleShareDirectory();
  
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

