/*==========================================================================

  Portions (c) Copyright 2010-2011 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: $
  Date:      $Date: $
  Version:   $Revision: $

==========================================================================*/
/// vtkMRMLAbdoNavNode - manages the data of the abdominal navigation module
///
/// This class stores references and data used by the abdominal navigation
/// module.

#ifndef __vtkMRMLAbdoNavNode_h
#define __vtkMRMLAbdoNavNode_h

/* AbdoNav includes */
//#include "vtkAbdoNavWin32Header.h"

// 12/25/2011 ayamada
// We need 5 include commands to add source code.
#include "vtkObject.h"
#include "vtkStdString.h"
#include "vtkRobotProbeNavWin32Header.h" 
#include "vtkMRMLFiducialListNode.h"
#include "vtkRobotProbeNavTargetDescriptor.h"

/* MRML includes */
#include "vtkMRMLNode.h"

/* VTK includes */
#include "vtkCollection.h"

//----------------------------------------------------------------
// Definition of string constants (expected labels of fiducials in
// the registration fiducial list) used by both, GUI and Logic.
//----------------------------------------------------------------

/*
const char* const     tip = "tip";
const char* const markerA = "markerA";
const char* const markerB = "markerB";
const char* const markerC = "markerC";
const char* const markerD = "markerD";
*/
// 12/27/2011 ayamada
const char* const     tip = "Marker1";
const char* const markerA = "Marker2";
const char* const markerB = "Marker3";
const char* const markerC = "Marker4";
const char* const markerD = "Marker5";
const char* const markerE = "Marker6";

class VTK_RobotProbeNAV_EXPORT vtkMRMLAbdoNavNode : public vtkMRMLNode
{
 public:
  //----------------------------------------------------------------
  // Usual VTK class functions.
  //----------------------------------------------------------------
  static vtkMRMLAbdoNavNode* New();
  vtkTypeRevisionMacro(vtkMRMLAbdoNavNode, vtkMRMLNode);
  void PrintSelf(ostream& os, vtkIndent indent);

  //----------------------------------------------------------------
  // Functions to be implemented by subclasses of vtkMRMLNode.
  //----------------------------------------------------------------
  /// Create an instance of an AbdoNav node.
  virtual vtkMRMLNode* CreateNodeInstance();
  /// Set node attributes from name/value pairs.
  virtual void ReadXMLAttributes(const char** atts);
  /// Write this node's information to a MRML file in XML format.
  virtual void WriteXML(ostream& os, int indent);
  /// Copy parameters (not including ID and Scene) from another node of the same type.
  virtual void Copy(vtkMRMLNode* node);
  /// Get unique node XML tag name (like Volume, Model, etc.).
  virtual const char* GetNodeTagName() { return "AbdoNav"; }
  /// Update the IDs of stored references.
  virtual void UpdateReferenceID(const char* oldID, const char* newID);

  //----------------------------------------------------------------
  // Getters and Setters for the references and data stored in this node.
  //----------------------------------------------------------------
  /// Get the identifier of the relative tracking transform.
  vtkGetStringMacro(TrackingTransformID);
  /// Set the identifier of the relative tracking transform.
  vtkSetStringMacro(TrackingTransformID);
  /// Get the identifier of the static registration transform.
  vtkGetStringMacro(RegistrationTransformID);
  /// Set the identifier of the static registration transform.
  vtkSetStringMacro(RegistrationTransformID);
  /// Get the identifier of the registration fiducial list.
  vtkGetStringMacro(RegistrationFiducialListID);
  /// Set the identifier of the registration fiducial list.
  vtkSetStringMacro(RegistrationFiducialListID);
  /// Get the identifier of the target fiducial list.
  vtkGetStringMacro(TargetFiducialListID);
  /// Set the identifier of the target fiducial list.
  vtkSetStringMacro(TargetFiducialListID);
  /// Get the identifier of the guidance needle tool type.
  vtkGetStringMacro(GuidanceToolType);
  /// Set the identifier of the guidance needle tool type.
  vtkSetStringMacro(GuidanceToolType);
  /// Get the path to the NDI ToolBox ".trackProperties" file.
  vtkGetStringMacro(ToolBoxPropertiesFile);
  /// Set the path to the NDI ToolBox ".trackProperties" file.
  vtkSetStringMacro(ToolBoxPropertiesFile);
  /// Get the offset of the guidance needle.
  vtkGetVector3Macro(GuidanceTipOffset, float);
  /// Set the offset of the guidance needle.
  vtkSetVector3Macro(GuidanceTipOffset, float);
  /// Get the Fiducial Registration Error (FRE).
  vtkGetMacro(FRE, double);
  /// Set the Fiducial Registration Error (FRE).
  vtkSetMacro(FRE, double);
  /// Get the time it took to perform registration.
  vtkGetMacro(ElapsedTime, double);
  /// Set the time it took to perform registration.
  vtkSetMacro(ElapsedTime, double);
  /// Get the recorded locator positions.
  vtkGetObjectMacro(RecordedPositions, vtkCollection);
  /// Set the recorded locator positions.
  vtkSetObjectMacro(RecordedPositions, vtkCollection);

 protected:
  //----------------------------------------------------------------
  // Usual VTK class functions.
  //----------------------------------------------------------------
  vtkMRMLAbdoNavNode();
  virtual ~vtkMRMLAbdoNavNode();

 private:
  //----------------------------------------------------------------
  // Usual VTK class functions.
  //----------------------------------------------------------------
  vtkMRMLAbdoNavNode(const vtkMRMLAbdoNavNode&); // not implemented
  void operator=(const vtkMRMLAbdoNavNode&);     // not implemented

  //----------------------------------------------------------------
  // The specific references and data stored in this MRML node.
  //----------------------------------------------------------------
  /// Identifier of the relative tracking transform.
  char* TrackingTransformID;
  /// Identifier of the static registration transform.
  char* RegistrationTransformID;
  /// Identifier of the fiducial list holding the image coordinates
  /// of the guidance needle artifact needed for registration. That
  /// is, at least three of the following:
  ///  - the guidance needle tip,
  ///  - the center of marker A,
  ///  - the center of marker B,
  ///  - the center of marker C,
  ///  - the center of marker D.
  char* RegistrationFiducialListID;
  /// Identifier of the fiducial list holding the image coordinates
  /// of the targets.
  char* TargetFiducialListID;
  /// Identifier of the tool type used as/attached to the guidance
  /// needle.
  char* GuidanceToolType;
  /// Path to the NDI ToolBox ".trackProperties" file.
  char* ToolBoxPropertiesFile;
  /// Offset of the guidance needle in terms of the guidance needle's
  /// local coordinate system.
  float GuidanceTipOffset[3];

  //----------------------------------------------------------------
  // Evaluation data. Only written, never read.
  //----------------------------------------------------------------
  /// Fiducial Registration Error (FRE).
  double FRE;
  /// Time it took to perform registration.
  double ElapsedTime;
  /// Recorded locator positions (target positions in terms of the
  /// evaluation).
  /// TODO: This collection currently stores objects of type
  ///       vtkMatrix4x4 which is obviously a very inefficient
  ///       way to store a triple of double values for each
  ///       position recorded.
  ///       Also, vtkMRMLAbdoNavNode::Copy(...) currently does
  ///       not take this class member into account.
  vtkCollection* RecordedPositions;

};

#endif // __vtkMRMLAbdoNavNode_h
