/*==========================================================================

  Portions (c) Copyright 2008 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/

#ifndef __vtkRobotProbeNavCalibrationStep_h
#define __vtkRobotProbeNavCalibrationStep_h

#include "vtkRobotProbeNavStep.h"

#include "vtkKWPushButton.h"
#include "vtkKWCheckButton.h"
#include "vtkKWLoadSaveButtonWithLabel.h"
#include "vtkKWFrame.h"

#include "vtkMRMLScalarVolumeNode.h"
#include "vtkMRMLLinearTransformNode.h"
#include "vtkKWMatrixWidgetWithLabel.h"

#include "vtkSlicerNodeSelectorWidget.h"

// 12/24/2011 ayamada
#include "vtkKWFrameWithLabel.h"
//#include "vtkMRMLRobotProbeNavNode.h"
#include "vtkMRMLAbdoNavNode.h"
/* MRML includes */
#include "vtkMRMLCrosshairNode.h"
/* IGT forward declarations */
#include "vtkSlicerModuleLogic.h"
//class vtkIGTPat2ImgRegistration;
//class vtkMRMLAbdoNavNode;
//class vtkMRMLRobotProbeNavNode;

// 12/27/2011 ayamada
#include "vtkKWMultiColumnList.h"
#include "vtkKWMultiColumnListWithScrollbars.h"

// 3/8/2012 ayamada
#include "vtkImageActor.h"
#include "vtkRenderer.h"

class vtkIGTPat2ImgRegistration;

class VTK_RobotProbeNAV_EXPORT vtkRobotProbeNavCalibrationStep : public vtkRobotProbeNavStep
{
public:
  static vtkRobotProbeNavCalibrationStep *New();
  vtkTypeRevisionMacro(vtkRobotProbeNavCalibrationStep,vtkRobotProbeNavStep);
  void PrintSelf(ostream& os, vtkIndent indent);

  virtual void ShowUserInterface();
  virtual void HideUserInterface();

  virtual void AddGUIObservers();
  virtual void RemoveGUIObservers();

  virtual void ProcessGUIEvents(vtkObject *caller, unsigned long event, void *callData);  

  void ShowZFrameModel(bool show);

  // Description:
  // If a file name is specified, the function will import an image from the file
  // to the MRML scene and call Z-Frame calibration code.
  void PerformZFrameCalibration(const char* filename);
  void PerformZFrameCalibration(vtkMRMLScalarVolumeNode* node, int s_index, int e_index);
  
  // 12/25/2011 ayamada
  /// Set and observe the parameter node.
  void SetAndObserveAbdoNavNode(vtkMRMLAbdoNavNode* node) { vtkSetAndObserveMRMLNodeMacro(this->AbdoNavNode, node); }
  //void SetAndObserveAbdoNavNode(vtkMRMLAbdoNavNode* node) { vtkSetAndObserveMRMLNodeMacro(this->RobotProbeNavNode, node); }
  
  // 12/27/2011 ayamada
  vtkKWEntryWithLabel* FiducialMarkersSetup;
  vtkKWPushButton* AddFiducialMarkerPushButton;

  vtkKWFrameWithLabel* FiducialFrame;
  vtkKWLoadSaveButtonWithLabel* SelectFiducialsFileButton;
  vtkKWPushButton* AddFiducialFilePushButton;

  vtkKWFrameWithLabel* ListFrame;
  vtkKWMultiColumnListWithScrollbars* PointPairMultiColumnList;
  int FiducialNumber;
  vtkKWPushButton* DeletePointPairPushButton;
  vtkKWPushButton* DeleteAllPointPairPushButton;
  
  // 12/27/2011 ayamada
  void translatePointData();
  vtkKWFrameWithLabel* registrationMatrixSelectorFrame;
  
  // 1/17/2012 ayamada
  vtkKWPushButton* LoadTargetingVolumeButton;
  vtkSlicerNodeSelectorWidget* VolumeSelectorWidget;
  // 1/21/2012 ayamada
  //virtual void UpdateGUI();
  
protected:
  vtkRobotProbeNavCalibrationStep();
  ~vtkRobotProbeNavCalibrationStep();

  vtkKWFrame       *SelectImageFrame;
  //vtkKWLoadSaveButtonWithLabel *SelectImageButton;
  vtkSlicerNodeSelectorWidget *ZFrameImageSelectorWidget;
  vtkKWMatrixWidgetWithLabel* SliceRangeMatrix;
  vtkKWPushButton  *CalibrateButton;
  vtkKWCheckButton *ShowZFrameCheckButton;
  vtkKWCheckButton *ShowWorkspaceCheckButton;

  vtkKWFrame       *ZFrameSettingFrame;

  
  // 12/24/2011 ayamada
  vtkKWFrameWithLabel *robotFrame;
  vtkSlicerNodeSelectorWidget *robotTransformSelector;
  vtkKWFrameWithLabel *robotPositionFrame;

  // 3/8/2012 ayamada
  vtkKWFrameWithLabel *referenceViewFrame;
  vtkKWFrame *referenceImageSelecterButtonFrame;
  vtkKWCheckButton *imageConfigurationButton1;
  vtkKWCheckButton *imageConfigurationButton2;
  vtkKWCheckButton *imageConfigurationButton3;
  
  //----------------------------------------------------------------
  // Video import
  //----------------------------------------------------------------  
  vtkRenderer*   BackgroundRenderer;
  vtkImageActor* BackgroundActor;
  
  
  vtkKWFrame       *point1Frame;
  vtkKWCheckButton *Point1CheckButton;
  vtkKWEntry *Point1REntry;
  vtkKWEntry *Point1AEntry;
  vtkKWEntry *Point1SEntry;
  
  vtkKWFrame       *point2Frame;
  vtkKWCheckButton *Point2CheckButton;
  vtkKWEntry *Point2REntry;
  vtkKWEntry *Point2AEntry;
  vtkKWEntry *Point2SEntry;
  
  vtkKWFrame       *point3Frame;
  vtkKWCheckButton *Point3CheckButton;
  vtkKWEntry *Point3REntry;
  vtkKWEntry *Point3AEntry;
  vtkKWEntry *Point3SEntry;
  
  vtkKWFrame       *point4Frame;
  vtkKWCheckButton *Point4CheckButton;
  vtkKWEntry *Point4REntry;
  vtkKWEntry *Point4AEntry;
  vtkKWEntry *Point4SEntry;
  
  vtkKWFrame       *point5Frame;
  vtkKWCheckButton *Point5CheckButton;
  vtkKWEntry *Point5REntry;
  vtkKWEntry *Point5AEntry;
  vtkKWEntry *Point5SEntry;

  // 12/27/2011 ayamada
  vtkKWFrame       *point6Frame;
  vtkKWCheckButton *Point6CheckButton;
  vtkKWEntry *Point6REntry;
  vtkKWEntry *Point6AEntry;
  vtkKWEntry *Point6SEntry;
  
  
  vtkKWPushButton* ResetRegistrationPushButton;
  vtkKWPushButton* PerformRegistrationPushButton;

  vtkMRMLAbdoNavNode* AbdoNavNode;  
  vtkMRMLAbdoNavNode* node;
  
  //vtkMRMLRobotProbeNavNode* RobotProbeNavNode;  
  
  /*
  /// Parameter node associated with this module. 
  vtkMRMLAbdoNavNode* AbdoNavNode;  

  //----------------------------------------------------------------
  // Helper function to create an AbdoNavNode if none exists yet.
  //----------------------------------------------------------------
  vtkMRMLAbdoNavNode* CheckAndCreateAbdoNavNode();
  //vtkMRMLRobotProbeNavNode* CheckAndCreateAbdoNavNode();
  //vtkMRMLAbdoNavNode* AbdoNavNode;  
  */
  vtkMRMLAbdoNavNode* CheckAndCreateAbdoNavNode();
  //vtkMRMLRobotProbeNavNode* CheckAndCreateAbdoNavNode2();
  /// Timer to record the time it took to perform registration
  /// (selecting one of the three check buttons will start the
  /// timer, performing registration will stop the timer).
  vtkTimerLog* TimerLog;
  
  void         AddLogicObservers();    // must be called manually
  void         RemoveLogicObservers(); // must be called manually
  
  // 12/26/2011 ayamada
  /// Update this module's MRML parameter node based on the values specified in the GUI.
  void UpdateMRMLFromGUI();
  /// Update the GUI based on the values stored in this module's MRML parameter node.  
  void UpdateGUIFromMRML();
  /// Process events generated by MRML.
  virtual void ProcessMRMLEvents(vtkObject* caller, unsigned long event, void* callData);
  
  // 3/8/2012 ayamada
  void showImages(const char* filename);
  void hideImages();
  void calcPath(const char* path);
  char* imageFilePath; 
  
  // 9/8/2011 ayamada
  //int LoadCSVTrackingFile(const char * path);
/*
  double pX[50];
  double pY[50];
  double pZ[50];
 */
  //double *pX;
  //double *pY;
  //double *pZ;
  vtkKWMultiColumnListWithScrollbars* PointPairMultiColumnList2;
  int FiducialNumber2;
  
  // 12/28/2011 ayamada
  vtkKWCheckButton *ShowRobotButton;
  
  //vtkMRMLModelNode* ZFrameModelNode;
  //BTX
  //std::string ZFrameModelNodeID;
  //std::string ZFrameTransformNodeID;
  //ETX
  

private:
  vtkRobotProbeNavCalibrationStep(const vtkRobotProbeNavCalibrationStep&);
  void operator=(const vtkRobotProbeNavCalibrationStep&);
};

#endif
