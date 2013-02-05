/*=auto=========================================================================

  Portions (c) Copyright 2005 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $RCSfile: vtkMRMLTransPerinealRobotProbeRobotNode.h,v $
  Date:      $Date: 2006/03/19 17:12:29 $
  Version:   $Revision: 1.3 $

=========================================================================auto=*/
#ifndef __vtkMRMLTransPerinealRobotNode_h
#define __vtkMRMLTransPerinealRobotNode_h

#include "vtkOpenIGTLinkIFWin32Header.h"
#include "vtkMRML.h"
#include "vtkMRMLRobotNode.h"
#include "vtkMRMLStorageNode.h"

#include "vtkCylinderSource.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkTransform.h"
#include "vtkAppendPolyData.h"

#include "vtkObject.h"
#include "vtkRobotProbeNavWin32Header.h" 

#include "vtkMRMLIGTLConnectorNode.h"
#include "vtkMRMLBrpRobotCommandNode.h"

#include "vtkMRMLLinearTransformNode.h"
#include "vtkMRMLModelNode.h"

class vtkTransform;
class vtkIGTLToMRMLCoordinate;
class vtkIGTLToMRMLBrpRobotCommand;
class vtkSlicerApplication;

class VTK_RobotProbeNAV_EXPORT vtkMRMLTransPerinealRobotProbeRobotNode : public vtkMRMLRobotNode
{

 public:

  //----------------------------------------------------------------
  // Standard methods for MRML nodes
  //----------------------------------------------------------------

  static vtkMRMLTransPerinealRobotProbeRobotNode *New();
  vtkTypeMacro(vtkMRMLTransPerinealRobotProbeRobotNode,vtkMRMLRobotNode);  

  void PrintSelf(ostream& os, vtkIndent indent);

  virtual vtkMRMLTransPerinealRobotProbeRobotNode* CreateNodeInstance();

  // Description:
  // Set node attributes
  virtual void ReadXMLAttributes( const char** atts);

  // Description:
  // Write this node's information to a MRML file in XML format.
  virtual void WriteXML(ostream& of, int indent);

  // Description:
  // Copy the node's attributes to this object
  virtual void Copy(vtkMRMLNode *node);

  // Description:
  // Update the stored reference to another node in the scene
  void UpdateReferenceID(const char *oldID, const char *newID);

  // Description:
  // Updates this node if it depends on other nodes 
  // when the node is deleted in the scene
  void UpdateReferences();

  // Description:
  // Updates other nodes in the scene depending on this node
  // or updates this node if it depends on other nodes
  virtual void UpdateScene(vtkMRMLScene *);

  // Description:
  // Get node XML tag name (like Volume, Model)
  virtual const char* GetNodeTagName()
    {return "TransPerinealRobotProbeRobot";};

  // method to propagate events generated in mrml
  virtual void ProcessMRMLEvents ( vtkObject *caller, unsigned long event, void *callData );

  //----------------------------------------------------------------
  // Commands
  //----------------------------------------------------------------

  // Description:
  // Initialize the robot
  virtual int Init(vtkSlicerApplication* app, const char* moduleShareDir);

  vtkGetStringMacro(RobotCommandNodeID);
  vtkMRMLBrpRobotCommandNode* GetRobotCommandNode();
  void SetAndObserveRobotCommandNodeID(const char *nodeID);
  
  vtkGetStringMacro(RobotConnectorNodeID);
  vtkMRMLIGTLConnectorNode* GetRobotConnectorNode();
  void SetAndObserveRobotConnectorNodeID(const char *nodeID);

  vtkGetStringMacro(ScannerConnectorNodeID);
  vtkMRMLIGTLConnectorNode* GetScannerConnectorNode();
  void SetAndObserveScannerConnectorNodeID(const char *nodeID);

  vtkGetStringMacro(ZFrameModelNodeID);
  vtkMRMLModelNode* GetZFrameModelNode();
  void SetAndObserveZFrameModelNodeID(const char *nodeID);
  
  // 9/21/2011 ayamada
  vtkGetStringMacro(MITRobotModelNodeID);
  vtkMRMLModelNode* GetMITRobotModelNode();
  void SetAndObserveMITRobotModelNodeID(const char *nodeID);

  // 9/23/2011 ayamada
  vtkGetStringMacro(MITRobotModelNode2ID);
  vtkMRMLModelNode* GetMITRobotModelNode2();
  void SetAndObserveMITRobotModelNode2ID(const char *nodeID);

  vtkGetStringMacro(MITRobotModelNode3ID);
  vtkMRMLModelNode* GetMITRobotModelNode3();
  void SetAndObserveMITRobotModelNode3ID(const char *nodeID);

  vtkGetStringMacro(MITRobotModelNode4ID);
  vtkMRMLModelNode* GetMITRobotModelNode4();
  void SetAndObserveMITRobotModelNode4ID(const char *nodeID);

  vtkGetStringMacro(MITRobotModelNode5ID);
  vtkMRMLModelNode* GetMITRobotModelNode5();
  void SetAndObserveMITRobotModelNode5ID(const char *nodeID);
  
  // 9/24/2011 ayamada
  vtkGetStringMacro(NeedleModelNodeID);
  vtkMRMLModelNode* GetNeedleModelNode();
  void SetAndObserveNeedleModelNodeID(const char *nodeID);

  vtkGetStringMacro(NeedleModelNode1ID);
  vtkMRMLModelNode* GetNeedleModelNode1();
  void SetAndObserveNeedleModelNode1ID(const char *nodeID);

  vtkGetStringMacro(NeedleModelNode2ID);
  vtkMRMLModelNode* GetNeedleModelNode2();
  void SetAndObserveNeedleModelNode2ID(const char *nodeID);
  
  // 11/3/2011 ayamada
  vtkGetStringMacro(VirtualCenterModelNodeID);
  vtkMRMLModelNode* GetVirtualCenterModelNode();
  void SetAndObserveVirtualCenterModelNodeID(const char *nodeID);

  vtkGetStringMacro(VirtualCenterTransformNodeID);
  vtkMRMLLinearTransformNode* GetVirtualCenterTransformNode();
  void SetAndObserveVirtualCenterTransformNodeID(const char *nodeID);  
  
  
  
  
  vtkGetStringMacro(WorkspaceModelNodeID);
  vtkMRMLModelNode* GetWorkspaceModelNode();
  void SetAndObserveWorkspaceModelNodeID(const char *nodeID);

  vtkGetStringMacro(ZFrameTransformNodeID);
  vtkMRMLLinearTransformNode* GetZFrameTransformNode();
  void SetAndObserveZFrameTransformNodeID(const char *nodeID);

  // 10/6/2011 ayamada
  vtkGetStringMacro(NeedlePathTransformNode1ID);
  vtkMRMLLinearTransformNode* GetNeedlePathTransformNode1();
  void SetAndObserveNeedlePathTransformNode1ID(const char *nodeID);

  vtkGetStringMacro(NeedlePathTransformNode2ID);
  vtkMRMLLinearTransformNode* GetNeedlePathTransformNode2();
  void SetAndObserveNeedlePathTransformNode2ID(const char *nodeID);

  vtkGetStringMacro(NeedlePathTransformNode3ID);
  vtkMRMLLinearTransformNode* GetNeedlePathTransformNode3();
  void SetAndObserveNeedlePathTransformNode3ID(const char *nodeID);

  // -----
  vtkGetStringMacro(MITRobotTransformNode1ID);
  vtkMRMLLinearTransformNode* GetMITRobotTransformNode1();
  void SetAndObserveMITRobotTransformNode1ID(const char *nodeID);

  vtkGetStringMacro(MITRobotTransformNode2ID);
  vtkMRMLLinearTransformNode* GetMITRobotTransformNode2();
  void SetAndObserveMITRobotTransformNode2ID(const char *nodeID);

  vtkGetStringMacro(MITRobotTransformNode3ID);
  vtkMRMLLinearTransformNode* GetMITRobotTransformNode3();
  void SetAndObserveMITRobotTransformNode3ID(const char *nodeID);

  vtkGetStringMacro(MITRobotTransformNode4ID);
  vtkMRMLLinearTransformNode* GetMITRobotTransformNode4();
  void SetAndObserveMITRobotTransformNode4ID(const char *nodeID);

  vtkGetStringMacro(MITRobotTransformNode5ID);
  vtkMRMLLinearTransformNode* GetMITRobotTransformNode5();
  void SetAndObserveMITRobotTransformNode5ID(const char *nodeID);
  
  // 7/21/2012 ayamada
  vtkGetStringMacro(MITRobotTransformNode6ID);
  vtkMRMLLinearTransformNode* GetMITRobotTransformNode6();
  void SetAndObserveMITRobotTransformNode6ID(const char *nodeID);

  vtkGetStringMacro(MITRobotTransformNode7ID);
  vtkMRMLLinearTransformNode* GetMITRobotTransformNode7();
  void SetAndObserveMITRobotTransformNode7ID(const char *nodeID);
  
  
  // 2/9/2012 ayamada
  vtkGetStringMacro(TargetPlaneTransformNodeID);
  vtkMRMLLinearTransformNode* GetTargetPlaneTransformNode();
  void SetAndObserveTargetPlaneTransformNodeID(const char *nodeID);
  
  
  // 9/13/2011 ayamada
  vtkGetStringMacro(MarkersPositionTransformNodeID);
  vtkMRMLLinearTransformNode* GetMarkersPositionTransformNode();
  void SetAndObserveMarkersPositionTransformNodeID(const char *nodeID);

  vtkGetStringMacro(MarkersPosition2TransformNodeID);
  vtkMRMLLinearTransformNode* GetMarkersPosition2TransformNode();
  void SetAndObserveMarkersPosition2TransformNodeID(const char *nodeID);
  
  // 6/28/2012 ayamada
  void calcPath2(const char* path);
  char* imageFilePath; 
  
  
  virtual int  MoveTo(const char *transformNodeId);

  virtual void SwitchStep(const char *stepName);

  virtual int OnTimer();

  //BTX
  virtual std::string GetTargetInfoText(vtkRobotProbeNavTargetDescriptor *targetDesc, NeedleDescriptorStruct *needle);
  //ETX

  // 9/13/2011 ayamada
  //virtual const char* GetCalibrationObjectModelId() { return GetMarkersPositionNodeID(); };
  //virtual const char* GetCalibrationObjectTransformId() { return GetMarkersPositionTransformNodeID(); };

  // 1/22/2012 ayamada
  virtual const char* GetMarkersPositionTransformId() { return GetMarkersPositionTransformNodeID(); };
  
  // 1/26/2012 ayamada
  virtual const char* GetMarkersPosition2TransformId() { return GetMarkersPosition2TransformNodeID(); };
  
  virtual const char* GetCalibrationObjectModelId() { return GetZFrameModelNodeID(); };

  // 9/21/2011 ayamada
  virtual const char* GetCalibrationObjectModel2Id() { return GetMITRobotModelNodeID(); };
  
  // 9/23/2011 ayamada
  virtual const char* GetCalibrationObjectModel3Id() { return GetMITRobotModelNode2ID(); };
  virtual const char* GetCalibrationObjectModel4Id() { return GetMITRobotModelNode3ID(); };
  virtual const char* GetCalibrationObjectModel5Id() { return GetMITRobotModelNode4ID(); };
  virtual const char* GetCalibrationObjectModel6Id() { return GetMITRobotModelNode5ID(); };
  
  // 9/24/2011 ayamada
  virtual const char* GetNeedleObjectModelId() {return GetNeedleModelNodeID(); };
  virtual const char* GetNeedleObjectModel1Id() {return GetNeedleModelNode1ID(); };
  virtual const char* GetNeedleObjectModel2Id() {return GetNeedleModelNode2ID(); };
  
  // 11/3/2011 ayamada
  virtual const char* GetVirtualCenterObjectModelId() {return GetVirtualCenterModelNodeID(); };
  virtual const char* GetVirtualCenterTransformId() {return GetVirtualCenterTransformNodeID(); };
  
  // 10/6/2011 ayamada
  virtual const char* GetNeedlePathTransform1ID() {return GetNeedlePathTransformNode1ID(); };
  virtual const char* GetNeedlePathTransform2ID() {return GetNeedlePathTransformNode2ID(); };
  virtual const char* GetNeedlePathTransform3ID() {return GetNeedlePathTransformNode3ID(); };
  
  // 10/6/2011 ayamada
  virtual const char* GetMITRobotTransform1ID() {return GetMITRobotTransformNode1ID(); };
  virtual const char* GetMITRobotTransform2ID() {return GetMITRobotTransformNode2ID(); };
  virtual const char* GetMITRobotTransform3ID() {return GetMITRobotTransformNode3ID(); };
  virtual const char* GetMITRobotTransform4ID() {return GetMITRobotTransformNode4ID(); };
  virtual const char* GetMITRobotTransform5ID() {return GetMITRobotTransformNode5ID(); };
  // 7/21/2012 ayamada
  virtual const char* GetMITRobotTransform6ID() {return GetMITRobotTransformNode6ID(); };
  virtual const char* GetMITRobotTransform7ID() {return GetMITRobotTransformNode7ID(); };
  
  // 2/9/2012 ayamada
  virtual const char* GetTargetPlaneTransformID() {return GetTargetPlaneTransformNodeID(); };
  
  
  virtual const char* GetCalibrationObjectTransformId() { return GetZFrameTransformNodeID(); };
  virtual const char* GetWorkspaceObjectModelId() { return GetWorkspaceModelNodeID(); };

  // 1/22/2012 ayamada
  //virtual const char* GetMarkersPositionTransformId() { return GetMarkersPositionTransformNodeID(); };
  
  
  
  virtual int PerformRegistration(vtkMRMLScalarVolumeNode* volumeNode);
  virtual int PerformRegistration(vtkMRMLScalarVolumeNode* volumeNode, int param1, int param2);

  virtual const char* GetWorkflowStepsString()
    {return "SetUp ZFrameCalibration PointTargeting PointVerification TransperinealRobotProbeRobotManualControl"; };

  
  
  // 9/13/2011 ayamada
  vtkSetReferenceStringMacro(MarkersPositionTransformNodeID); 
  char *MarkersPositionTransformNodeID;
  vtkMRMLLinearTransformNode* MarkersPositionTransformNode;  
  
  vtkSetReferenceStringMacro(MarkersPosition2TransformNodeID); 
  char *MarkersPosition2TransformNodeID;
  vtkMRMLLinearTransformNode* MarkersPosition2TransformNode;  

  // 9/14/2011 ayamada
  void sendImagePositionData(void);

  
 protected:

  vtkMRMLTransPerinealRobotProbeRobotNode();
  virtual ~vtkMRMLTransPerinealRobotProbeRobotNode();
  vtkMRMLTransPerinealRobotProbeRobotNode(const vtkMRMLTransPerinealRobotProbeRobotNode&);
  void operator=(const vtkMRMLTransPerinealRobotProbeRobotNode&);

  ///////////

  int  SendZFrame();

  vtkGetMacro ( Connection,              bool );
  vtkGetMacro ( RobotWorkPhase,           int );
  vtkGetMacro ( ScannerWorkPhase,         int );

  const char* AddWorkspaceModel(const char* nodeName);
  const char* AddZFrameModel(const char* nodeName);

  // 9/21/2011 ayamada
  const char* AddMITRobotModel(const char* nodeName, int partsNumber);

  // 9/24/2011 ayamada
  const char* AddNeedleModel(const char* nodeName, int needleNumber);
  
  // 11/3/2011 ayamada
  const char* AddVirtualCenterModel(const char* nodeName);
  
  // NOTE: Since we couldn't update ScannerStatusLabelDisp and RobotStatusLabelDisp
  // directly from ProcessMRMLEvent(), we added following flags to update those GUI
  // widgets in the timer handler.
  // if flag == 0, the widget does not need to be updated()
  // if flag == 1, the connector has connected to the target
  // if flag == 2, the connector has disconnected from the target
  int ScannerConnectedFlag;
  int RobotConnectedFlag;

  
  
private:

  // Node references

  vtkSetReferenceStringMacro(RobotCommandNodeID);
  char *RobotCommandNodeID;
  vtkMRMLBrpRobotCommandNode* RobotCommandNode;

  vtkSetReferenceStringMacro(RobotConnectorNodeID);
  char *RobotConnectorNodeID;
  vtkMRMLIGTLConnectorNode* RobotConnectorNode;

  vtkSetReferenceStringMacro(ScannerConnectorNodeID);
  char *ScannerConnectorNodeID;
  vtkMRMLIGTLConnectorNode* ScannerConnectorNode;

  vtkSetReferenceStringMacro(ZFrameModelNodeID);
  char *ZFrameModelNodeID;
  vtkMRMLModelNode* ZFrameModelNode;
  
  // 9/21/2011 ayamada
  vtkSetReferenceStringMacro(MITRobotModelNodeID);
  char *MITRobotModelNodeID;
  vtkMRMLModelNode* MITRobotModelNode;
  
  // 9/23/2011 ayamada
  vtkSetReferenceStringMacro(MITRobotModelNode2ID);
  char *MITRobotModelNode2ID;
  vtkMRMLModelNode* MITRobotModelNode2;

  vtkSetReferenceStringMacro(MITRobotModelNode3ID);
  char *MITRobotModelNode3ID;
  vtkMRMLModelNode* MITRobotModelNode3;

  vtkSetReferenceStringMacro(MITRobotModelNode4ID);
  char *MITRobotModelNode4ID;
  vtkMRMLModelNode* MITRobotModelNode4;

  vtkSetReferenceStringMacro(MITRobotModelNode5ID);
  char *MITRobotModelNode5ID;
  vtkMRMLModelNode* MITRobotModelNode5;
  
  
  // 9/24/2011 ayamada
  vtkSetReferenceStringMacro(NeedleModelNodeID);
  char *NeedleModelNodeID;
  vtkMRMLModelNode* NeedleModelNode;

  vtkSetReferenceStringMacro(NeedleModelNode1ID);
  char *NeedleModelNode1ID;
  vtkMRMLModelNode* NeedleModelNode1;

  vtkSetReferenceStringMacro(NeedleModelNode2ID);
  char *NeedleModelNode2ID;
  vtkMRMLModelNode* NeedleModelNode2;
  
  // 11/3/2011 ayamada
  vtkSetReferenceStringMacro(VirtualCenterModelNodeID);
  char *VirtualCenterModelNodeID;
  vtkMRMLModelNode* VirtualCenterModelNode;

  vtkSetReferenceStringMacro(VirtualCenterTransformNodeID);
  char *VirtualCenterTransformNodeID;
  vtkMRMLLinearTransformNode* VirtualCenterTransformNode;
  
  
/*  
  // 9/13/2011 ayamada
  vtkSetReferenceStringMacro(MarkersPositionTransformNodeID); 
  char *MarkersPositionTransformNodeID;
  vtkMRMLLinearTransformNode* MarkersPositionTransformNode;  

  vtkSetReferenceStringMacro(MarkersPosition2TransformNodeID); 
  char *MarkersPosition2TransformNodeID;
  vtkMRMLLinearTransformNode* MarkersPosition2TransformNode;  
*/  
  
  vtkSetReferenceStringMacro(ZFrameTransformNodeID); 
  char *ZFrameTransformNodeID;
  vtkMRMLLinearTransformNode* ZFrameTransformNode;  

  // 10/6/2011 ayamada
  vtkSetReferenceStringMacro(NeedlePathTransformNode1ID); 
  char *NeedlePathTransformNode1ID;
  vtkMRMLLinearTransformNode* NeedlePathTransformNode1;  

  vtkSetReferenceStringMacro(NeedlePathTransformNode2ID); 
  char *NeedlePathTransformNode2ID;
  vtkMRMLLinearTransformNode* NeedlePathTransformNode2;  

  vtkSetReferenceStringMacro(NeedlePathTransformNode3ID); 
  char *NeedlePathTransformNode3ID;
  vtkMRMLLinearTransformNode* NeedlePathTransformNode3;  

  // ----------
  vtkSetReferenceStringMacro(MITRobotTransformNode1ID); 
  char *MITRobotTransformNode1ID;
  vtkMRMLLinearTransformNode* MITRobotTransformNode1;  

  vtkSetReferenceStringMacro(MITRobotTransformNode2ID); 
  char *MITRobotTransformNode2ID;
  vtkMRMLLinearTransformNode* MITRobotTransformNode2;  

  vtkSetReferenceStringMacro(MITRobotTransformNode3ID); 
  char *MITRobotTransformNode3ID;
  vtkMRMLLinearTransformNode* MITRobotTransformNode3;  

  vtkSetReferenceStringMacro(MITRobotTransformNode4ID); 
  char *MITRobotTransformNode4ID;
  vtkMRMLLinearTransformNode* MITRobotTransformNode4;  

  vtkSetReferenceStringMacro(MITRobotTransformNode5ID); 
  char *MITRobotTransformNode5ID;
  vtkMRMLLinearTransformNode* MITRobotTransformNode5;  

  // 7/21/2012 ayamada
  vtkSetReferenceStringMacro(MITRobotTransformNode6ID); 
  char *MITRobotTransformNode6ID;
  vtkMRMLLinearTransformNode* MITRobotTransformNode6;  
  
  vtkSetReferenceStringMacro(MITRobotTransformNode7ID); 
  char *MITRobotTransformNode7ID;
  vtkMRMLLinearTransformNode* MITRobotTransformNode7;  
  
  // 2/9/2012 ayamada
  vtkSetReferenceStringMacro(TargetPlaneTransformNodeID); 
  char *TargetPlaneTransformNodeID;
  vtkMRMLLinearTransformNode* TargetPlaneTransformNode;  
  
  vtkSetReferenceStringMacro(WorkspaceModelNodeID);
  char *WorkspaceModelNodeID;
  vtkMRMLModelNode* WorkspaceModelNode;

  // Other member variables

  vtkIGTLToMRMLCoordinate* CoordinateConverter;
  vtkIGTLToMRMLBrpRobotCommand* CommandConverter;

  bool  Connection;  
  int   RobotWorkPhase;
  int   ScannerWorkPhase;
  
};

#endif

