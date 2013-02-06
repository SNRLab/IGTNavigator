/*=auto=========================================================================

  Portions (c) Copyright 2007 Brigham and Women's Hospital (BWH) All Rights Reserved. 

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer 
  Module:    $RCSfile: $    
  Date:      $Date: $ this is the test data  
  Version:   $Revision: $  

=========================================================================auto=*/

#include "vtkObjectFactory.h"
#include "vtkCallbackCommand.h"

#include "vtkRobotProbeNavLogic.h"

#include "vtkMRMLModelDisplayNode.h"
#include "vtkMRMLScalarVolumeNode.h"
#include "vtkMRMLLinearTransformNode.h"
#include "vtkSlicerApplication.h"
#include "vtkSlicerApplicationGUI.h"
#include "vtkSlicerFiducialsGUI.h"
#include "vtkSlicerColorLogic.h"
#include "vtkMRMLLabelMapVolumeDisplayNode.h"

#include "vtkMRMLRobotProbeNavManagerNode.h"
#include "vtkMRMLRobotNode.h"

#include "vtkKWRadioButton.h"

// for DICOM read 
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImage.h"
#include "itkMetaDataDictionary.h"
#include "itkMetaDataObject.h"
#include "itkGDCMImageIO.h"
#include "itkSpatialOrientationAdapter.h"

#include "vtkMRMLBrpRobotCommandNode.h"

#include "vtkRobotProbeNavGUI.h"

#include "RobotProbeNavMath.h"

// 9/13/2011 ayamada
#include "vtkMRMLIGTProbeRobotNode.h"
#include "math.h"

// 7/1/2012 ayamada
#define PI 3.14159

vtkCxxRevisionMacro(vtkRobotProbeNavLogic, "$Revision: 1.9.12.1 $");
vtkStandardNewMacro(vtkRobotProbeNavLogic);

//---------------------------------------------------------------------------
vtkRobotProbeNavLogic::vtkRobotProbeNavLogic()
{
  // Timer Handling
  this->DataCallbackCommand = vtkCallbackCommand::New();
  this->DataCallbackCommand->SetClientData( reinterpret_cast<void *> (this) );
  this->DataCallbackCommand->SetCallback(vtkRobotProbeNavLogic::DataCallback);

  this->TimerOn = 0;
  
  // 10/28/2011 ayamada
  this->magnitudeOfDirectionVector = 0.0;
  // 7/5/2012 ayamada
  this->magnitudeOfDirectionVector2 = 0.0;
  
  // 11/20/2011 ayamada
  this->tmpDis = 0.0;
  this->tmpDisRef = 0.0;
  
  this->tmpVectorOP[0] = 0.0;
  this->tmpVectorOP[1] = 0.0;
  this->tmpVectorOP[2] = 0.0;
  
  this->leftNeedleContactPosition[0] = 0.0;
  this->leftNeedleContactPosition[1] = 0.0;
  this->leftNeedleContactPosition[2] = 0.0;
  
  // 12/9/2011 ayamada
  this->centerDirectionVector[0] = 0.0;
  this->centerDirectionVector[1] = 0.0;
  this->centerDirectionVector[2] = 0.0;
  
  this->centerPassingPoint[0] = 0.0;
  this->centerPassingPoint[1] = 0.0;
  this->centerPassingPoint[2] = 0.0;
  
  this->centerVariableK = 2.0;
  this->centerVariableK2 = 2.0;
  
  this->parametrizedDirectionVector[0] = 0.0;
  this->parametrizedDirectionVector[1] = 0.0;
  this->parametrizedDirectionVector[2] = 0.0;
  
  this->parametrizedDirectionVector2[0] = 0.0;
  this->parametrizedDirectionVector2[1] = 0.0;
  this->parametrizedDirectionVector2[2] = 0.0;
  
  this->adjusterSign = 1.0;
  this->adjusterSign2 = 1.0;
  
  // 12/22/2011 ayamada
  this->Pat2ImgReg = vtkIGTPat2ImgRegistration::New();
  
  // 12/25/2011 ayamada
  this->RelativeTrackingTransform = vtkMRMLLinearTransformNode::New();//NULL;
  
  // 12/26/2011 ayamada
  this->workPhaseStatus = 0;
  this->workPhaseInitialFlag = 0;
  
  // 7/5/2012 ayamada
  this->carriagePositionForOrder = 0.0;
    
  // 2/11/2012 ayamada
  this->existanceOfTarget1 = 0;
  this->existanceOfTarget2 = 0;
  this->existanceOfTarget3 = 0;
  this->getRow = 0;
  
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      this->linearP[i][j];
      this->linearV[i][j];
    }
  }
  
  // 3/8/2012 ayamada
  this->imageFilePath = NULL;
  
  this->TargetPositionXYZ[0] = 0.0;
  this->TargetPositionXYZ[1] = 0.0;
  this->TargetPositionXYZ[2] = 0.0;
  
  this->AbdoNavNode = vtkMRMLAbdoNavNode::New();
  this->RegistrationTransform = vtkMRMLLinearTransformNode::New();
  this->RegistrationPerformed = NULL;
  this->node = vtkMRMLAbdoNavNode::New();
  this->GUI = vtkRobotProbeNavGUI::New();
}


//---------------------------------------------------------------------------
vtkRobotProbeNavLogic::~vtkRobotProbeNavLogic()
{
  if (this->DataCallbackCommand)
  {
    this->DataCallbackCommand->Delete();
  }
  
  // 12/25/2011 ayamada
  if (this->RelativeTrackingTransform)
  {
    this->RelativeTrackingTransform->Delete();
  }  
  
  // 12/26/2011 ayamada
  this->workPhaseStatus = NULL;
  this->workPhaseInitialFlag = NULL;
 
}

//---------------------------------------------------------------------------
void vtkRobotProbeNavLogic::PrintSelf(ostream& os, vtkIndent indent)
{
    this->vtkObject::PrintSelf(os, indent);

    os << indent << "vtkRobotProbeNavLogic:             " << this->GetClassName() << "\n";

}

//---------------------------------------------------------------------------
void vtkRobotProbeNavLogic::DataCallback(vtkObject *caller, 
                                       unsigned long eid, void *clientData, void *callData)
{
    vtkRobotProbeNavLogic *self = reinterpret_cast<vtkRobotProbeNavLogic *>(clientData);
    vtkDebugWithObjectMacro(self, "In vtkRobotProbeNavLogic DataCallback");
    self->UpdateAll();
}

//---------------------------------------------------------------------------
void vtkRobotProbeNavLogic::UpdateAll()
{

}


int vtkRobotProbeNavLogic::Enter()
{      
  vtkKWTkUtilities::CreateTimerHandler(this->GetGUI()->GetApplication(), 200, this, "TimerHandler");
  return 1;
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavLogic::TimerHandler()
{
  if (this->TimerOn)
  {
    vtkMRMLRobotNode* robot=GetRobotNode();
    if (robot!=NULL)
    {
      robot->OnTimer();
    }
  }
}

//--------------------------------------------------------------------------- 
int vtkRobotProbeNavLogic::RobotStop()
{

  std::cerr << "vtkRobotProbeNavLogic::RobotStop()" << std::endl;
  return 1;

}


//---------------------------------------------------------------------------
int vtkRobotProbeNavLogic::RobotMoveTo(float px, float py, float pz,
                                     float nx, float ny, float nz,
                                     float tx, float ty, float tz)
{

  std::cerr << "vtkRobotProbeNavLogic::RobotMoveTo()" << std::endl;
  return 1;
}


//---------------------------------------------------------------------------
int vtkRobotProbeNavLogic::RobotMoveTo(float position[3], float orientation[3])
{
  std::cerr << "vtkRobotProbeNavLogic::RobotMoveTo()" << std::endl;

  return 1;
}

//---------------------------------------------------------------------------
int vtkRobotProbeNavLogic::RobotMoveTo()
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    return 0; // failed
  }
  return robot->MoveTo(robot->GetTargetTransformNodeID());  
}
//---------------------------------------------------------------------------
int vtkRobotProbeNavLogic::ScanStart()
{

  return 1;
}

//---------------------------------------------------------------------------
int vtkRobotProbeNavLogic::ScanPause()
{
  return 1;
}

//---------------------------------------------------------------------------
int vtkRobotProbeNavLogic::ScanStop()
{

  return 1;
}

//---------------------------------------------------------------------------
void vtkRobotProbeNavLogic::SetSliceViewFromVolume(vtkMRMLVolumeNode *volumeNode)
{
  if (!volumeNode)
    {
    return;
    }

  vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
  vtkSmartPointer<vtkMatrix4x4> permutationMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
  vtkSmartPointer<vtkMatrix4x4> rotationMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

  volumeNode->GetIJKToRASDirectionMatrix(matrix);
  vtkMRMLTransformNode *transformNode = volumeNode->GetParentTransformNode();
  if ( transformNode )
    {
    vtkSmartPointer<vtkMatrix4x4> rasToRAS = vtkSmartPointer<vtkMatrix4x4>::New();
    transformNode->GetMatrixTransformToWorld(rasToRAS);
    vtkMatrix4x4::Multiply4x4 (rasToRAS, matrix, matrix);
    }


  int permutation[3];
  int flip[3];
  RobotProbeNavMath::ComputePermutationFromOrientation(matrix, permutation, flip);


  permutationMatrix->SetElement(0,0,0);
  permutationMatrix->SetElement(1,1,0);
  permutationMatrix->SetElement(2,2,0);

  permutationMatrix->SetElement(0, permutation[0],
                     (flip[permutation[0]] ? -1 : 1));
  permutationMatrix->SetElement(1, permutation[1],
                     (flip[permutation[1]] ? -1 : 1));
  permutationMatrix->SetElement(2, permutation[2],
                     (flip[permutation[2]] ? -1 : 1));


  permutationMatrix->Invert();
  vtkMatrix4x4::Multiply4x4(matrix, permutationMatrix, rotationMatrix); 

  vtkSlicerApplicationLogic *appLogic = this->GetGUI()->GetApplicationLogic();

  
  // Set the slice views to match the volume slice orientation
  for (int i = 0; i < 3; i++)
    {
    static const char *panes[3] = { "Red", "Yellow", "Green" };

    vtkMatrix4x4 *newMatrix = vtkMatrix4x4::New();

    vtkSlicerSliceLogic *slice = appLogic->GetSliceLogic(
      const_cast<char *>(panes[i]));
    
    vtkMRMLSliceNode *sliceNode = slice->GetSliceNode();

    // Need to find window center and rotate around that

    // Default matrix orientation for slice
    newMatrix->SetElement(0, 0, 0.0);
    newMatrix->SetElement(1, 1, 0.0);
    newMatrix->SetElement(2, 2, 0.0);
    if (i == 0)
      {
      newMatrix->SetElement(0, 0, -1.0);
      newMatrix->SetElement(1, 1, 1.0);
      newMatrix->SetElement(2, 2, 1.0);
      }
    else if (i == 1)
      {
      newMatrix->SetElement(1, 0, -1.0);
      newMatrix->SetElement(2, 1, 1.0);
      newMatrix->SetElement(0, 2, 1.0);
      }
    else if (i == 2)
      {
      newMatrix->SetElement(0, 0, -1.0);
      newMatrix->SetElement(2, 1, 1.0);
      newMatrix->SetElement(1, 2, 1.0);
      }

    // Next, set the orientation to match the volume
    sliceNode->SetOrientationToReformat();
    vtkMatrix4x4::Multiply4x4(rotationMatrix, newMatrix, newMatrix);
    sliceNode->SetSliceToRAS(newMatrix);
    sliceNode->UpdateMatrices();
    newMatrix->Delete();
    }

}
//---------------------------------------------------------------------------
vtkMRMLScalarVolumeNode *vtkRobotProbeNavLogic::AddVolumeToScene(const char *fileName, VolumeType volumeType/*=VOL_GENERIC*/)
{
  if (fileName==0)
  {
    vtkErrorMacro("AddVolumeToScene: invalid filename");
    return 0;
  }

  vtksys_stl::string volumeNameString = vtksys::SystemTools::GetFilenameName(fileName);
  vtkMRMLScalarVolumeNode *volumeNode = this->AddArchetypeVolume(fileName, volumeNameString.c_str());

  if (volumeNode==NULL)
  {
    vtkErrorMacro("Error adding volume to the scene");
    return NULL;
  }

  vtkMRMLRobotProbeNavManagerNode* manager=this->GUI->GetRobotProbeNavManagerNode();
  if (manager==NULL)
  {
    vtkErrorMacro("Error adding volume to the scene, manager is invalid");
    return NULL;
  }

  this->SetSliceViewFromVolume(volumeNode);

  switch (volumeType)
  {
  case VOL_GENERIC:
    // don't store a reference in the manager node
    break;
  case VOL_TARGETING:
    manager->SetTargetingVolumeNodeRef(volumeNode->GetID());
    break;
  case VOL_VERIFICATION:
    manager->SetVerificationVolumeNodeRef(volumeNode->GetID());
    break;
  default:
    vtkErrorMacro("AddVolumeToScene: unknown volume type: " << volumeType);
  }
  
  volumeNode->Modified();
  this->Modified();

  return volumeNode;
}

//---------------------------------------------------------------------------
int vtkRobotProbeNavLogic::SelectVolumeInScene(vtkMRMLScalarVolumeNode* volumeNode, VolumeType volumeType)
{
  if (volumeNode==0)
  {
    vtkErrorMacro("SelectVolumeInScene: invalid volume");
    return 0;
  }

  vtkMRMLRobotProbeNavManagerNode* manager=this->GUI->GetRobotProbeNavManagerNode();
  if (manager==NULL)
  {
    vtkErrorMacro("Error adding volume to the scene, manager is invalid");
    return 0;
  }

  this->SetSliceViewFromVolume(volumeNode);

  this->GetGUI()->GetApplicationLogic()->GetSelectionNode()->SetActiveVolumeID( volumeNode->GetID() );
  this->GetGUI()->GetApplicationLogic()->PropagateVolumeSelection();

  switch (volumeType)
  {
  case VOL_GENERIC:
    // don't store a reference in the manager node
    break;
  case VOL_TARGETING:
    manager->SetTargetingVolumeNodeRef(volumeNode->GetID());
    break;
  case VOL_VERIFICATION:
    manager->SetVerificationVolumeNodeRef(volumeNode->GetID());
    break;
  default:
    vtkErrorMacro("AddVolumeToScene: unknown volume type: " << volumeType);
  }
  
  //volumeNode->Modified();
  this->Modified();

  return 1;
}

//---------------------------------------------------------------------------
vtkMRMLScalarVolumeNode *vtkRobotProbeNavLogic::AddArchetypeVolume(const char* fileName, const char *volumeName)
{
  // Set up storageNode
  vtkSmartPointer<vtkMRMLVolumeArchetypeStorageNode> storageNode = vtkSmartPointer<vtkMRMLVolumeArchetypeStorageNode>::New(); 
  storageNode->SetFileName(fileName);
  // check to see if can read this type of file
  if (storageNode->SupportedFileType(fileName) == 0)
    {
    vtkErrorMacro("AddArchetypeVolume: can't read this kind of file: " << fileName);
    return 0;
    }
  storageNode->SetCenterImage(false);
  storageNode->SetSingleFile(false);
  storageNode->SetUseOrientationFromFile(true);

  // Set up scalarNode
  vtkSmartPointer<vtkMRMLScalarVolumeNode> scalarNode = vtkSmartPointer<vtkMRMLScalarVolumeNode>::New();
  scalarNode->SetName(volumeName);
  scalarNode->SetLabelMap(false);

  // Set up displayNode
  vtkSmartPointer<vtkMRMLScalarVolumeDisplayNode> displayNode = vtkSmartPointer<vtkMRMLScalarVolumeDisplayNode>::New();   
  displayNode->SetAutoWindowLevel(false);
  displayNode->SetInterpolate(true);  
  vtkSmartPointer<vtkSlicerColorLogic> colorLogic = vtkSmartPointer<vtkSlicerColorLogic>::New(); 
  displayNode->SetAndObserveColorNodeID(colorLogic->GetDefaultVolumeColorNodeID());
  
  // Add nodes to scene
  vtkDebugMacro("LoadArchetypeVolume: adding storage node to the scene");
  storageNode->SetScene(this->GetMRMLScene());
  this->GetMRMLScene()->AddNode(storageNode);
  vtkDebugMacro("LoadArchetypeVolume: adding display node to the scene");
  displayNode->SetScene(this->GetMRMLScene());
  this->GetMRMLScene()->AddNode(displayNode);
  vtkDebugMacro("LoadArchetypeVolume: adding scalar node to the scene");
  scalarNode->SetScene(this->GetMRMLScene());
  this->GetMRMLScene()->AddNode(scalarNode);
  scalarNode->SetAndObserveStorageNodeID(storageNode->GetID());
  scalarNode->SetAndObserveDisplayNodeID(displayNode->GetID());
  
  // Read the volume into the node
  vtkDebugMacro("AddArchetypeVolume: about to read data into scalar node " << scalarNode->GetName());
  storageNode->AddObserver(vtkCommand::ProgressEvent, this->LogicCallbackCommand);
  if (this->GetDebug())
    {
    storageNode->DebugOn();
    }
  storageNode->ReadData(scalarNode);
  vtkDebugMacro("AddArchetypeVolume: finished reading data into scalarNode");
  storageNode->RemoveObservers(vtkCommand::ProgressEvent, this->LogicCallbackCommand);
 
  return scalarNode;
}

//--------------------------------------------------------------------------------------
std::string vtkRobotProbeNavLogic::GetFoRStrFromVolumeNodeID(const char* volNodeID)
{  
  vtkMRMLScalarVolumeNode *volNode=vtkMRMLScalarVolumeNode::SafeDownCast(this->GetMRMLScene()->GetNodeByID(volNodeID));  
  if (volNode==NULL)
  {
    vtkErrorMacro("Cannot get FoR, VolumeNode is undefined");
    return std::string("");
  }

  // remaining information to be had from the meta data dictionary     
  const itk::MetaDataDictionary &volDictionary = volNode->GetMetaDataDictionary();
  std::string tagValue; 

  // frame of reference uid
  tagValue.clear();
  itk::ExposeMetaData<std::string>( volDictionary, "0020|0052", tagValue );

  // Need to remove extra '0x00' characters that may be present at the end of the string
  if (!tagValue.empty())
  {
    tagValue=tagValue.c_str();
  }  
  
  return tagValue;
}

void vtkRobotProbeNavLogic::UpdateTargetListFromMRML()
{
  vtkMRMLRobotProbeNavManagerNode* manager=this->GUI->GetRobotProbeNavManagerNode();
  if (manager==NULL)
  {
    vtkErrorMacro("Error updating targetlist from mrml, manager is invalid");
    return;
  }
  vtkMRMLFiducialListNode* fidNode=manager->GetTargetPlanListNode();
  if (fidNode==NULL)
  {
    vtkErrorMacro("Error updating targetlist from mrml, fiducial node is invalid");
    return;
  }

  // True if we modified fiducials at all (typically the label has to be changed)
  bool fidNodeModified=false;
  // If we modified fiducials, then do it in one step, with Start/EndModify. For that we need to remember the previous state.
  int fidNodeModifyOld=0;

  LinkTargetsToFiducials();

  // Remove fiducials in the manager that doesn't have a corresponding FiducialNode
  for (int i=0; i<manager->GetTotalNumberOfTargets(); i++)
  {
    vtkRobotProbeNavTargetDescriptor *t=manager->GetTargetDescriptorAtIndex(i);
    if (fidNode->GetFiducialIndex(t->GetFiducialID())<0)
    {
      // fiducial not found, need to delete it
      manager->RemoveTargetDescriptorAtIndex(i);
      i--; // repeat check on the i-th element
    }
  }

  for (int i=0; i<fidNode->GetNumberOfFiducials(); i++)
  {
    int targetIndex=GetTargetIndexFromFiducialID(fidNode->GetNthFiducialID(i));
    if (targetIndex<0)
    {

      if (!fidNodeModified)
      {
        fidNodeModified=true;
        fidNodeModifyOld=fidNode->StartModify();
      }

      // New fiducial, create associated target
      vtkSmartPointer<vtkRobotProbeNavTargetDescriptor> targetDesc=vtkSmartPointer<vtkRobotProbeNavTargetDescriptor>::New();      
      
      targetDesc->SetFiducialID(fidNode->GetNthFiducialID(i));

      int needleIndex=manager->GetCurrentNeedleIndex();
      NeedleDescriptorStruct needleDesc;
      if (!manager->GetNeedle(needleIndex, needleDesc))
      {
        vtkErrorMacro("Failed to get info for needle "<<needleIndex);
      }
      targetDesc->SetNeedleID(needleDesc.mID); // just to make the passed target and needle info consistent

      needleDesc.mLastTargetIndex++;
      if (!manager->SetNeedle(needleIndex, needleDesc))
      {
        vtkErrorMacro("Failed to set info for needle "<<needleIndex);
      }

      // Haiying: this fixes the issue of target name changes 
      // when you switch among fiducial lists on the Targeting tab.  
      //std::ostrstream strvalue;
      //strvalue << needleDesc.mTargetNamePrefix << needleDesc.mLastTargetIndex << std::ends;        
      //fidNode->SetNthFiducialLabelText(i,strvalue.str());
      //strvalue.rdbuf()->freeze(0);     
      

      std::string FoR = this->GetFoRStrFromVolumeNodeID(manager->GetTargetingVolumeNodeRef());
      targetDesc->SetTargetingVolumeFoR(FoR);

      manager->AddTargetDescriptor(targetDesc);
    }

    targetIndex=GetTargetIndexFromFiducialID(fidNode->GetNthFiducialID(i));
    if (targetIndex>=0)
    {
      // Update fiducial
      vtkRobotProbeNavTargetDescriptor* targetDesc = manager->GetTargetDescriptorAtIndex(targetIndex);
      if (targetDesc!=NULL)
      {
        float *rasLocation=fidNode->GetNthFiducialXYZ(i);
        targetDesc->SetRASLocation(rasLocation[0], rasLocation[1], rasLocation[2]);

        float *rasOrientation=fidNode->GetNthFiducialOrientation(i);
        targetDesc->SetRASOrientation(rasOrientation[0], rasOrientation[1], rasOrientation[2], rasOrientation[3]);

        targetDesc->SetName(fidNode->GetNthFiducialLabelText(i));

        // :TODO: update needle,  etc. parameters ?

      }
      else
      {
        vtkErrorMacro("Invalid target descriptor");
      }
    }
    else
    {
      vtkErrorMacro("Invalid Fiducial ID");
    }
  }

  if (fidNodeModified)
  {
    fidNode->EndModify(fidNodeModifyOld);
    // StartModify/EndModify discarded vtkMRMLFiducialListNode::FiducialModifiedEvent-s, so we have to resubIGT them now
    fidNode->InvokeEvent(vtkMRMLFiducialListNode::FiducialModifiedEvent, NULL);
  }
}

//----------------------------------------------------------------------------
void vtkRobotProbeNavLogic::LinkTargetsToFiducials()
{
  vtkMRMLRobotProbeNavManagerNode* manager=this->GUI->GetRobotProbeNavManagerNode();
  if (manager==NULL)
  {
    vtkErrorMacro("Error in LinkTargetsToFiducials, manager is invalid");
    return;
  }
  vtkMRMLFiducialListNode* fidNode=manager->GetTargetPlanListNode();
  if (fidNode==NULL)
  {
    vtkErrorMacro("Error in LinkTargetsToFiducials, fiducial node is invalid");
    return;
  }

  // if all the targets have empty FiducialID reference it means that the targets are not linked
  // to the fiducials yet
  for (int i=0; i<manager->GetTotalNumberOfTargets(); i++)
  {
    vtkRobotProbeNavTargetDescriptor *t=manager->GetTargetDescriptorAtIndex(i);
    if (!t->GetFiducialID().empty())
    {
      // a non-empty FiducialID is found, it means that there is already existing linking
      return;
    }
  }

  const float rasTolerance=0.1;
  for (int targetInd=0; targetInd<manager->GetTotalNumberOfTargets(); targetInd++)
  {
    vtkRobotProbeNavTargetDescriptor *t=manager->GetTargetDescriptorAtIndex(targetInd);
    float targetXYZ[3]={0,0,0};
    targetXYZ[0]=t->GetRASLocation()[0];
    targetXYZ[1]=t->GetRASLocation()[1];
    targetXYZ[2]=t->GetRASLocation()[2];
    for (int fidIndex=0; fidNode->GetNumberOfFiducials(); fidIndex++)
    {
      float* fidXYZ=fidNode->GetNthFiducialXYZ(fidIndex);
      if ( (fabs(targetXYZ[0]-fidXYZ[0])<rasTolerance)
        && (fabs(targetXYZ[1]-fidXYZ[1])<rasTolerance)
        && (fabs(targetXYZ[2]-fidXYZ[2])<rasTolerance)
        && (t->GetName().compare(fidNode->GetNthFiducialLabelText(fidIndex))==0) )
      {
        // matching fiducial found
        t->SetFiducialID(fidNode->GetNthFiducialID(fidIndex));
        break;
      }
    }
  }
}

//----------------------------------------------------------------------------
int vtkRobotProbeNavLogic::GetTargetIndexFromFiducialID(const char* fiducialID)
{
  if (fiducialID==NULL)
  {
    vtkWarningMacro("Fiducial ID is invalid");
    return -1;
  }
  vtkMRMLRobotProbeNavManagerNode* manager=this->GUI->GetRobotProbeNavManagerNode();
  if (manager==NULL)
  {
    vtkErrorMacro("Manager is invalid");
    return -1;
  }
  for (int i=0; i<manager->GetTotalNumberOfTargets(); i++)
  {
    vtkRobotProbeNavTargetDescriptor *t=manager->GetTargetDescriptorAtIndex(i);
    if (t->GetFiducialID().compare(fiducialID)==0)
    {
      // found the target corresponding to the fiducialID
      return i;
    }
  }
  return -1;
}

//----------------------------------------------------------------------------
int vtkRobotProbeNavLogic::SetMouseInteractionMode(int mode)
{  
  if (GetApplicationLogic()==NULL)
  {
   vtkErrorMacro("Application logic is invalid");
    return 0;
  }
  if (GetApplicationLogic()->GetMRMLScene()==NULL)
  {
    vtkErrorMacro("Scene is invalid");
    return 0;
  }
  vtkMRMLInteractionNode *interactionNode = vtkMRMLInteractionNode::SafeDownCast(GetApplicationLogic()->GetMRMLScene()->GetNthNodeByClass(0, "vtkMRMLInteractionNode"));
  if (interactionNode==NULL)
  {
    vtkErrorMacro("Interaction node is invalid");
    return 0;
  }
  
  if (this->GetGUI()==NULL)
  {
    vtkErrorMacro("GUI is invalid");
    return 0;
  }  
  vtkSlicerApplication* app=vtkSlicerApplication::SafeDownCast(this->GetGUI()->GetApplication());
  if (app==NULL)
  {
    vtkErrorMacro("Application is invalid");
    return 0;
  }
  vtkSlicerApplicationGUI* appGUI = app->GetApplicationGUI();
  if (appGUI==NULL)
  {
    vtkErrorMacro("Application GUI is invalid");
    return 0;
  }
  vtkSlicerToolbarGUI *tGUI = appGUI->GetApplicationToolbar();
  if (tGUI==NULL)
  {
    vtkErrorMacro("Application toolbar GUI is invalid");
    return 0;
  }

  // Set logic state
  interactionNode->SetCurrentInteractionMode(mode); 

  // Set pick/place state to persistent (stay in the staet after picking/placing a fiducial)
  if (mode==vtkMRMLInteractionNode::Place)
  {
    interactionNode->SetPlaceModePersistence(1);
  }
  else if (mode==vtkMRMLInteractionNode::PickManipulate)
  {
    interactionNode->SetPickModePersistence(1);
  }
  
  return 1;
}

//----------------------------------------------------------------------------
int vtkRobotProbeNavLogic::SetCurrentFiducialList(vtkMRMLFiducialListNode* fidNode)
{
  if (fidNode==NULL)
  {
    vtkErrorMacro("Fiducial node is invalid");
    return 0;
  }

  if (this->GetGUI()==NULL)
  {
    vtkErrorMacro("GUI is invalid");
    return 0;
  }  
  vtkSlicerApplication* app=vtkSlicerApplication::SafeDownCast(this->GetGUI()->GetApplication());
  if (app==NULL)
  {
    vtkErrorMacro("Application is invalid");
    return 0;
  }

  vtkSlicerFiducialsGUI* fidGUI = vtkSlicerFiducialsGUI::SafeDownCast ( app->GetModuleGUIByName ("Fiducials"));
  if (fidGUI==NULL)
  {
    vtkErrorMacro("Fiducial GUI is invalid");
    return 0;
  }
  
  // Activate target fiducials in the Fiducial GUI
  fidGUI->Enter();
  fidGUI->SetFiducialListNodeID(fidNode->GetID());
  
  return 1;
}

vtkMRMLRobotNode* vtkRobotProbeNavLogic::GetRobotNode()
{
  if (this->GUI==NULL)
  {
    return NULL;
  }
  if (this->GUI->GetRobotProbeNavManagerNode()==NULL)
  {
    return NULL;
  }
  return this->GUI->GetRobotProbeNavManagerNode()->GetRobotNode();
  {
    return NULL;
  }
}

//----------------------------------------------------------------------------
int vtkRobotProbeNavLogic::ShowWorkspaceModel(bool show)
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, robot is invalid");
    return 0; // failed
  }
  vtkMRMLModelNode*   modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetWorkspaceObjectModelId()));
  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, workspace model node is invalid");
    return 0; // failed
  }
  vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();
  return 1;
} 

// 10/19/2011 ayamada
int vtkRobotProbeNavLogic::CorrectNeedlePosition(vtkKWMatrixWidget* matrix, vtkKWMatrixWidget* oMatrix, int numRows)
{
  
  int flag = 0;
   
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, robot is invalid");
    return 0; // failed
  }
  
  float position[3]={0,0,0};   // position parameters
  float positionRAS[3]={0,0,0};
  float orientation[4]={1,0,0,0}; // orientation parameters
  
  // 10/20/2011 ayamada
  vtkMRMLNode*   node = vtkMRMLNode::New();
  
  if(numRows == 0)
  {
    node = vtkMRMLNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetNeedlePathTransform1ID()));
  }else if(numRows == 1)
  {
    node = vtkMRMLNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetNeedlePathTransform2ID()));    
  }else if(numRows == 2)
  {
    node = vtkMRMLNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetNeedlePathTransform3ID()));    
  }else{
    flag = 1;
  }
  
  if(flag == 0)
  {
  vtkMRMLLinearTransformNode* transformNode = vtkMRMLLinearTransformNode::SafeDownCast(node);
  vtkMatrix4x4* transform = transformNode->GetMatrixTransformToParent();
  
  // see vtkRobotProbeNavTargetingStep.cxx from l.545~
  //igtl::Matrix4x4 igtlmatrix;
  //igtl::QuaternionToMatrix(orientation, igtlmatrix);

  vtkMatrix4x4* targetPositionMatrix = vtkMatrix4x4::New();
  targetPositionMatrix->Identity(); 

    targetPositionMatrix->SetElement(0,3,(float) matrix->GetElementValueAsDouble(0, 0));
    targetPositionMatrix->SetElement(1,3,(float) matrix->GetElementValueAsDouble(0, 1));
    targetPositionMatrix->SetElement(2,3,(float) matrix->GetElementValueAsDouble(0, 2));
    
    positionRAS[0] = (float) matrix->GetElementValueAsDouble(0, 0); //-1.0;// x position of the target
    positionRAS[1] = (float) matrix->GetElementValueAsDouble(0, 1); //84.9;// y position of the target
    positionRAS[2] = (float) matrix->GetElementValueAsDouble(0, 2); //5.1;// z position of the target
  
    // 8/19/2012 ayamada
    std::cout << "positionRAS[0] = " << positionRAS[0] << std::endl;
    std::cout << "positionRAS[1] = " << positionRAS[1] << std::endl;
    std::cout << "positionRAS[2] = " << positionRAS[2] << std::endl;
    
  // 6/30/2012 ayamada
  // ---------------------------------  
  // Inverse matrix from vtkAbdoNavLogic.cxx l.1124~  

  vtkMRMLNode*   robotNode = vtkMRMLNode::New();
  robotNode = vtkMRMLNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetCalibrationObjectTransformId()));
  vtkMRMLLinearTransformNode* robotTransformNode = vtkMRMLLinearTransformNode::SafeDownCast(robotNode);
  vtkMatrix4x4* robotTransform = robotTransformNode->GetMatrixTransformToParent();
    
  // get matrix that converts RAS coordinates to XY
  vtkMatrix4x4* RASToXY = vtkMatrix4x4::New();
  RASToXY->DeepCopy(robotTransform);
  // invert it and convert registered tracking coordinates from RAS to XY
  RASToXY->Invert();
  
    // 7/1/2012 ayamada
    // calculate target position on XYZ coordinate
    // RASToXY is inverse matrix of robot coordinate.
    // targetPositionMatrix is target position matrix of RAS coordinate.
    // targetXY is target position matrix of XY coordinate
    
    vtkMatrix4x4* targetXY = vtkMatrix4x4::New();
    targetXY->Identity(); 
    vtkMatrix4x4::Multiply4x4(RASToXY, targetPositionMatrix, targetXY);
    
    // 7/2/2012 ayamada
    // process for offset between CAD coordinate and robot coordinate since there are difference
    // between CAD and robot coordinates.
    // we have to consider CAD coordinate to display robot on 3D Slicer.
    vtkMatrix4x4* offsetMatrix = vtkMatrix4x4::New();
    offsetMatrix->Identity();    
    vtkMatrix4x4* processedMatrix = vtkMatrix4x4::New();
    processedMatrix->Identity();

    vtkMatrix4x4* targetPositionMatrix2 = vtkMatrix4x4::New();
    targetPositionMatrix2->Identity();
    
    // 8/19/2012 ayamada
    targetPositionMatrix2->SetElement(0,3,(float) targetXY->GetElement(0, 3));
    targetPositionMatrix2->SetElement(1,3,(float) targetXY->GetElement(1, 3));
    targetPositionMatrix2->SetElement(2,3,(float) targetXY->GetElement(2, 3));
        
    vtkMatrix4x4::Multiply4x4(offsetMatrix, targetPositionMatrix2, processedMatrix);    
    
    position[0] = processedMatrix->GetElement(0,3);
    position[1] = processedMatrix->GetElement(1,3);
    position[2] = processedMatrix->GetElement(2,3);

    // 8/20/2012 ayamada: for displaying target position data in vtkRobotProbeNavTargetingStep.cxx
    this->TargetPositionXYZ[0] = position[0];
    this->TargetPositionXYZ[1] = position[1];
    this->TargetPositionXYZ[2] = position[2];
    
    // 8/19/2012 ayamada
    std::cout << "positionXY[0] = " << position[0] << std::endl;
    std::cout << "positionXY[1] = " << position[1] << std::endl;
    std::cout << "positionXY[2] = " << position[2] << std::endl;
    
    // 11/8/2011 ayamada
    // Gram-Schmidt orthonormalization
    float u1[3] = {0,0,0};
    float u2[3] = {0,0,0};
    float u3[3] = {0,0,0};
    
    float a1[3] = {0,0,0};
    float a2[3] = {0,0,0};
    float a3[3] = {0,0,0};
    //float Sa1,Sa2,Sa3;
    
    float b1[3] = {0,0,0};
    float b2[3] = {0,0,0};
    float b3[3] = {0,0,0};
    float Sb1,Sb2,Sb3;    
    float b3temp1[3] = {0,0,0};
    float b3temp2[3] = {0,0,0};
    
    // 7/1/2012 ayamada
    // --------------------------------
    // Robot kinematics
    // 7/1/2012 coded by ayamada
    // --------------------------------
    //  double outputVector[3];
    double hooperPositionNewXY[3];
    float hooperPositionNewRAS[3];
      // Robot spec
    double d = 3.0;//3;   // clearance between needles
    double R = 82.0;//84.0;//80;  // radius of arc

    double xt2 = position[0];
    double yt2 = position[1];
    double zt2 = position[2];    
    
    // r = 7; // Pully's radius (mm)
    
    double r = sqrt(xt2*xt2+yt2*yt2+zt2*zt2);
    double delta_q2 = atan(d/r);// 7/23 updated. //2.0*asin(L/(2.0*r));
      
      // Calc of q1 for the three needles:
      // 9/18/2012 ayamada
      double q1_track2 = atan(yt2/zt2);
      // 8/20/2012 ayamada    
      std::cout << "q1_track2 = " << q1_track2/PI*180.0 << endl;

      // Calc of q2 for each of the three needles:      
      double q2_track = 0.0;
      
      if(numRows == 0)
      {
        q2_track  = asin(-xt2/r); // in radian
      }else if(numRows == 1)
      {
        q2_track  = asin(-xt2/r) + delta_q2; // in radian
      }else if(numRows == 2)
      {
        q2_track  = asin(-xt2/r) - delta_q2; // in radian
      }
    
    // 8/20/2012 ayamada    
    std::cout << "q2_track = " << q2_track/PI*180.0 << endl;
        
    // 7/26/2012 ayamada
    // q2 from Java UI
    // double q1comp = Math.abs(q1p-q1);
    // q2=((180/Math.PI)*Math.asin(-xTarget/insLength)+(track-2)*dq2)*R/r-q1comp;
    
    // hooper position on RAS coordinate
    hooperPositionNewXY[0] = R*sin(q2_track);
    hooperPositionNewXY[1] = R*cos(q2_track)*sin(q1_track2);
    hooperPositionNewXY[2] = R*cos(q2_track)*cos(q1_track2);

    // 8/19/2012 ayamada
    std::cout << "hooperPositionNewXY[0] = " << hooperPositionNewXY[0] << std::endl;
    std::cout << "hooperPositionNewXY[1] = " << hooperPositionNewXY[1] << std::endl;
    std::cout << "hooperPositionNewXY[2] = " << hooperPositionNewXY[2] << std::endl;
    
    
    // 7/24/2012 Faye's comment
    // Just to clarify, by hopper you mean the carriage that rides on the hoop, right? 
    // This calculates the x-y-z coordinate of the center of the carriage. 
    // The actual needle would be 14 deg off in q1 direction and 3 mm off in q2 
    // (depending on the track number).
    
    // ---------------------------
    // 7/5/2012 ayamada:
    // save the carriage y position to decide the order of needle insertion
    // ---------------------------
    this->carriagePositionForOrder = hooperPositionNewXY[1];
    
    // ---------------------------
    // 7/5/2012 ayamada: 
    // calculate 1) distance from carriage to target point     
    //           2) distance from carriage to skin entry point
    // ---------------------------
    
    // 1) distance from carriage to target point
    float linea, lineb, linec, lined, linee,linef;
    
    linea = hooperPositionNewXY[0];
    lineb = hooperPositionNewXY[1];
    linec = hooperPositionNewXY[2];
    lined = linea-xt2;
    linee = lineb-yt2;
    linef = linec-zt2;
    
    this->magnitudeOfDirectionVector = sqrt(lined*lined + linee*linee + linef*linef);
    
    // 2) distance from carriage to skin entry point
    float plainA, plainB, plainC, plainD;
    float pointa, pointb, pointc, pointd, pointe, pointf;
    float plaint;
    float skinx, skiny, skinz;
    
    // introduce plain equation
    // point1 = (0,0,-21), point2 = (5,3,-21), point3 = (2,3,-21)....
    // There is a distance between skin entry point and Origin, that is an offset.
    // point2-point1 = (a,b,c), point3-point1 = (d,e,f)
    pointa = 5.0;
    pointb = 3.0;
    pointc = 0.0;
    pointd = 2.0;
    pointe = 3.0;
    pointf = 0.0;
    
    plainA = pointb*pointf - pointc*pointe;
    plainB = pointc*pointd - pointa*pointf;
    plainC = pointa*pointe - pointb*pointd;
    plainD = plainA*0.0+plainB*0.0+plainC*(-21.0);
    
    plaint = -1.0*(linea*plainA+lineb*plainB+linec*plainC+plainD)/(lined*plainA+linee*plainB+linef*plainC);
    
    skinx = linea+plaint*lined;
    skiny = lineb+plaint*linee;
    skinz = linec+plaint*linef;
    
    this->magnitudeOfDirectionVector2 = sqrt(
    (skinx-linea)*(skinx-linea)+(skiny-lineb)*(skiny-lineb)+(skinz-linec)*(skinz-linec)
    );
    
    // ---------------------------
    
    // 7/2/2012 ayamada 
    // process offset
    //offsetMatrix->SetElement(1,3,(float)(0.0));
    //offsetMatrix->SetElement(2,3,(float)(4.0));
    offsetMatrix->Invert();
    vtkMatrix4x4* hooperPositionBeforeProcess = vtkMatrix4x4::New();
    hooperPositionBeforeProcess->Identity(); 
    vtkMatrix4x4* hooperPositionAfterProcess = vtkMatrix4x4::New();
    hooperPositionAfterProcess->Identity(); 
    
    hooperPositionBeforeProcess->SetElement(0,3,(float) hooperPositionNewXY[0]);
    hooperPositionBeforeProcess->SetElement(1,3,(float) hooperPositionNewXY[1]);
    hooperPositionBeforeProcess->SetElement(2,3,(float) hooperPositionNewXY[2]);
    vtkMatrix4x4::Multiply4x4(offsetMatrix, hooperPositionBeforeProcess, hooperPositionAfterProcess);

    // calculate hooper position on XYZ coordinate
    // RASToXY is inverse matrix of robot coordinate.
    // hooperPositionMatrixRAS is hooper position matrix of RAS coordinate.
    // hooperPositionMatrixXY is hooper position matrix of XY coordinate    
    vtkMatrix4x4* hooperPositionMatrixRAS = vtkMatrix4x4::New();
    hooperPositionMatrixRAS->Identity(); 

    vtkMatrix4x4* hooperPositionMatrixXY = vtkMatrix4x4::New();
    hooperPositionMatrixXY->Identity(); 
    
    // 8/19/2012 ayamada
    hooperPositionMatrixXY->SetElement(0,3,(float) hooperPositionAfterProcess->GetElement(0,3));
    hooperPositionMatrixXY->SetElement(1,3,(float) hooperPositionAfterProcess->GetElement(1,3));
    hooperPositionMatrixXY->SetElement(2,3,(float) hooperPositionAfterProcess->GetElement(2,3));
    
    vtkMatrix4x4::Multiply4x4(robotTransform, hooperPositionMatrixXY, hooperPositionMatrixRAS);
    
    hooperPositionNewRAS[0] = hooperPositionMatrixRAS->GetElement(0,3);
    hooperPositionNewRAS[1] = hooperPositionMatrixRAS->GetElement(1,3);
    hooperPositionNewRAS[2] = hooperPositionMatrixRAS->GetElement(2,3);
    
    // draw each needle
    
    // 7/1/2012 ayamada
    // ------------------------
    a1[0] = hooperPositionNewRAS[0] - positionRAS[0];
    a1[1] = hooperPositionNewRAS[1] - positionRAS[1]; 
    a1[2] = hooperPositionNewRAS[2] - positionRAS[2];
    // ------------------------
      
    b1[0] = a1[0];
    b1[1] = a1[1];
    b1[2] = a1[2];
    
    Sb1 = sqrt(b1[0]*b1[0]+b1[1]*b1[1]+b1[2]*b1[2]);//this->magnitudeOfDirectionVector;
    
    // 7/5/2012 ayamada: cut off and rewrite around l.1180~
    // 12/8/2011 ayamada
    //this->magnitudeOfDirectionVector = Sb1;
    
    u1[0] = b1[0]/Sb1;
    u1[1] = b1[1]/Sb1;
    u1[2] = b1[2]/Sb1;
        
    // 11/8/2011 ayamada
    // Gram-Schmidt orthonormalization

    // calculate u2
    
    a2[0] = 2.0*a1[0];
    a2[1] = a1[1];
    a2[2] = 5.0*a1[2];
    
    b2[0] = a2[0] - (a2[0]*u1[0]+a2[1]*u1[1]+a2[2]*u1[2])*u1[0];
    b2[1] = a2[1] - (a2[0]*u1[0]+a2[1]*u1[1]+a2[2]*u1[2])*u1[1];
    b2[2] = a2[2] - (a2[0]*u1[0]+a2[1]*u1[1]+a2[2]*u1[2])*u1[2];
    
    Sb2 = sqrt(b2[0]*b2[0]+b2[1]*b2[1]+b2[2]*b2[2]);
    
    u2[0] = b2[0]/Sb2;
    u2[1] = b2[1]/Sb2;
    u2[2] = b2[2]/Sb2;
    
    // calculate u3
    a3[0] = -2.0*a1[0];
    a3[1] = a1[1];
    a3[2] = -5.0*a1[2];
    
    b3temp1[0] = a3[0] - (a3[0]*u1[0]+a3[1]*u1[1]+a3[2]*u1[2])*u1[0];
    b3temp1[1] = a3[1] - (a3[0]*u1[0]+a3[1]*u1[1]+a3[2]*u1[2])*u1[1];
    b3temp1[2] = a3[2] - (a3[0]*u1[0]+a3[1]*u1[1]+a3[2]*u1[2])*u1[2];
    
    b3temp2[0] = (a3[0]*u2[0]+a3[1]*u2[1]+a3[2]*u2[2])*u2[0];
    b3temp2[1] = (a3[0]*u2[0]+a3[1]*u2[1]+a3[2]*u2[2])*u2[1];
    b3temp2[2] = (a3[0]*u2[0]+a3[1]*u2[1]+a3[2]*u2[2])*u2[2];
    
    b3[0] = b3temp1[0] - b3temp2[0];
    b3[1] = b3temp1[1] - b3temp2[1];
    b3[2] = b3temp1[2] - b3temp2[2];
    
    Sb3 = sqrt(b3[0]*b3[0]+b3[1]*b3[1]+b3[2]*b3[2]);
    
    u3[0] = b3[0]/Sb3;
    u3[1] = b3[1]/Sb3;
    u3[2] = b3[2]/Sb3;
    
 
  // 10/25/2011 ayamada
  // about the element of orientation, we don't need to transfer data from l.1023 of vtkRobotProbeNavTargetingStep.cxx
  // in this test case, I used the transform of the robot element. But actually, you should use the product of calibration matrix and transform matrix.
  orientation[0] = (float) oMatrix->GetElementValueAsDouble(0, 0);
  orientation[1] = (float) oMatrix->GetElementValueAsDouble(0, 1);
  orientation[2] = (float) oMatrix->GetElementValueAsDouble(0, 2);
  orientation[3] = (float) oMatrix->GetElementValueAsDouble(0, 3);
   
 
  if (transformNode)
  {

    // see vtkRobotProbeNavTargetingStep.cxx from l.545~
    
    igtl::Matrix4x4 igtlmatrix;

    igtlmatrix[0][3] = positionRAS[0];
    igtlmatrix[1][3] = positionRAS[1];
    igtlmatrix[2][3] = positionRAS[2];
    
    // 11/4/2011 ayamada; modified
    igtlmatrix[0][0] = u2[0];
    igtlmatrix[1][0] = u2[1];
    igtlmatrix[2][0] = u2[2];
    igtlmatrix[3][0] = 0.0;
    
    igtlmatrix[0][1] = u3[0];
    igtlmatrix[1][1] = u3[1];
    igtlmatrix[2][1] = u3[2];
    igtlmatrix[3][1] = 0.0;
    
    igtlmatrix[0][2] = u1[0];
    igtlmatrix[1][2] = u1[1];
    igtlmatrix[2][2] = u1[2];
    igtlmatrix[3][2] = 0.0;
    
    igtlmatrix[3][3] = 1.0;
        
    // 7/1/2012 ayamada
    // transform = needlePathTransform: see the above part of this function
 
    transform->SetElement(0, 3, igtlmatrix[0][3]);
    transform->SetElement(1, 3, igtlmatrix[1][3]);
    transform->SetElement(2, 3, igtlmatrix[2][3]);
    transform->SetElement(3, 3, igtlmatrix[3][3]);

    // 10/27/2011 ayamada
        
    transform->SetElement(0, 0, igtlmatrix[0][0]);
    transform->SetElement(1, 0, igtlmatrix[1][0]);
    transform->SetElement(2, 0, igtlmatrix[2][0]);
    transform->SetElement(3, 0, igtlmatrix[3][0]);

    transform->SetElement(0, 1, igtlmatrix[0][1]);
    transform->SetElement(1, 1, igtlmatrix[1][1]);
    transform->SetElement(2, 1, igtlmatrix[2][1]);
    transform->SetElement(3, 1, igtlmatrix[3][1]);

    transform->SetElement(0, 2, igtlmatrix[0][2]);
    transform->SetElement(1, 2, igtlmatrix[1][2]);
    transform->SetElement(2, 2, igtlmatrix[2][2]);
    transform->SetElement(3, 2, igtlmatrix[3][2]);
    
  }
  
  }
  
  //std::cerr << "CorrectNeedlePosition Function is working now!! numRows = " << numRows << std::endl;
  
  return 1;
  
}

// 9/24/2011 ayamada
//----------------------------------------------------------------------------
int vtkRobotProbeNavLogic::ShowNeedleModel(bool show)
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, robot is invalid");
    return 0; // failed
  }
  
  
  // 0
  vtkMRMLModelNode*   modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetNeedleObjectModelId()));
  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, workspace model node is invalid");
    return 0; // failed
  }
  vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();
  
  // 1
  modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetNeedleObjectModel1Id()));
  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, workspace model node is invalid");
    return 0; // failed
  }
  displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();

  // 2
  modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetNeedleObjectModel2Id()));
  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, workspace model node is invalid");
    return 0; // failed
  }
  displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();

  
  return 1;
} 

//----------------------------------------------------------------------------
bool vtkRobotProbeNavLogic::IsNeedleModelShown()
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    return false;
  }
  vtkMRMLModelNode*   modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetNeedleObjectModelId()));
  if (modelNode==NULL)
  {
    return 0;
  }
  vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    return 0;
  }
  return displayNode->GetVisibility();
}


// 11/4/2011 ayamada
//----------------------------------------------------------------------------
int vtkRobotProbeNavLogic::ShowVirtualCenterModel(bool show)
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, robot is invalid");
    return 0; // failed
  }
  
  // 0
  vtkMRMLModelNode*   modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetVirtualCenterObjectModelId()));
  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, workspace model node is invalid");
    return 0; // failed
  }
  vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();
  
  return 1;
} 

//----------------------------------------------------------------------------
bool vtkRobotProbeNavLogic::IsVirtualCenterModelShown()
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    return false;
  }
  vtkMRMLModelNode*   modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetVirtualCenterObjectModelId()));
  if (modelNode==NULL)
  {
    return 0;
  }
  vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    return 0;
  }
  return displayNode->GetVisibility();
}



//----------------------------------------------------------------------------
bool vtkRobotProbeNavLogic::IsWorkspaceModelShown()
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    return false;
  }
  vtkMRMLModelNode*   modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetWorkspaceObjectModelId()));
  if (modelNode==NULL)
  {
    return 0;
  }
  vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    return 0;
  }
  return displayNode->GetVisibility();
}

// 9/23/2011 modified by ayamada
//----------------------------------------------------------------------------
int vtkRobotProbeNavLogic::ShowRobotModel(bool show)
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    vtkWarningMacro("Cannot show robot model, robot is invalid");
    return 0; // failed
  }
  
  // 1
  // 9/23/2011 ayamada
  vtkMRMLModelNode*   modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetCalibrationObjectModel2Id()));
  
  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, workspace model node is invalid");
    return 0; // failed
  }
  vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();

  // 2
  modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetCalibrationObjectModel3Id()));

  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, workspace model node is invalid");
    return 0; // failed
  }
  displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();

  // 3
  modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetCalibrationObjectModel4Id()));
  
  
  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, workspace model node is invalid");
    return 0; // failed
  }
  displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();

  // 4
  modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetCalibrationObjectModel5Id()));
  
  
  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, workspace model node is invalid");
    return 0; // failed
  }
  displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();

  // 5
  modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetCalibrationObjectModel6Id()));
  

  if (modelNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, workspace model node is invalid");
    return 0; // failed
  }
  displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    vtkWarningMacro("Cannot show robot model, displayNode is invalid");
    return 0; // failed
  }
  displayNode->SetVisibility(show);
  displayNode->SetSliceIntersectionVisibility(show);
  modelNode->Modified();
  this->MRMLScene->Modified();
  
  
  return 1;
} 


// 9/25/2011 ayamada
//----------------------------------------------------------------------------
bool vtkRobotProbeNavLogic::IsRobotModelShown()
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
    return false;
  }
  
  
  // 1
  vtkMRMLModelNode*   modelNode = vtkMRMLModelNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetCalibrationObjectModelId()));
  if (modelNode==NULL)
  {
    return 0;
  }
  vtkMRMLDisplayNode* displayNode = modelNode->GetDisplayNode();
  if (displayNode==NULL)
  {
    return 0;
  }
  
  // we will judge whether the robot was displaied or not by the part 1 of the robot. 
  
  // 2

  // 3

  // 4

  // 5
  
  return displayNode->GetVisibility();
}




// 8/4/2011 ayamada
void vtkRobotProbeNavLogic::GetFiducialMarkersR(int n, float px, float py, float pz)
{
  
  this->FiducialMarkersR[n][1][0][0] = px;
  this->FiducialMarkersR[n][0][1][0] = py;
  this->FiducialMarkersR[n][0][0][1] = pz;
  
}

// 9/8/2011 ayamada
void vtkRobotProbeNavLogic::GetFiducialMarkersI(int n, float px, float py, float pz)
{
  
  this->FiducialMarkersI[n][1][0][0] = px;
  this->FiducialMarkersI[n][0][1][0] = py;
  this->FiducialMarkersI[n][0][0][1] = pz;
  
}

// 9/13/2011 ayamada
void vtkRobotProbeNavLogic::SendMarkersPosition(void)
{


  
  vtkMRMLRobotNode* robot=GetRobotNode();
  if (robot==NULL)
  {
  //  return 0; // failed
  }
  //return 
  robot->MoveTo(robot->GetTargetTransformNodeID());  
  
}

// 11/4/2011 ayamada
// based on CorrectNeedlePosition on l.806~
void vtkRobotProbeNavLogic::CalculateVirtualCenterPosition(void)
{
  vtkMRMLRobotNode* robot=GetRobotNode();
  int flag = 0;
  
  if (robot==NULL)
  {
    vtkWarningMacro("Cannot show workspace model, robot is invalid");
    flag = 1; // failed
  }

  if(flag == 0)
  {
  vtkMRMLNode*   node = vtkMRMLNode::New();
  node = vtkMRMLNode::SafeDownCast(this->MRMLScene->GetNodeByID(robot->GetVirtualCenterTransformId()));

  vtkMRMLLinearTransformNode* transformNode = vtkMRMLLinearTransformNode::SafeDownCast(node);
  vtkMatrix4x4* transform = transformNode->GetMatrixTransformToParent();

  // based on Init in vtkMRMLIGTProbeRobotNode.cxx l.1012
  transform->Identity();
    

  transform->SetElement(0, 3, (float)0.0);
  transform->SetElement(1, 3, (float)0.0);
  transform->SetElement(2, 3, (float)0.0);
    
  // apply the registration matrix 
  //transformNode->ApplyTransform(transform);
  
  }
  
}
  
// 12/22/2011 ayamada
int vtkRobotProbeNavLogic::PerformRegistration()
{
  vtkMRMLFiducialListNode* fnode;
  float* tipOffset = NULL;
  
  if (this->AbdoNavNode == NULL)
  {
    vtkErrorMacro("in vtkAbdoNavLogic::PerformRegistration(...): "
                  "Couldn't retrieve AbdoNavNode!");
    return EXIT_FAILURE;
  }
  else
  {
    fnode = vtkMRMLFiducialListNode::SafeDownCast(this->GetMRMLScene()->GetNodeByID(this->AbdoNavNode->GetRegistrationFiducialListID()));
    tipOffset = this->AbdoNavNode->GetGuidanceTipOffset();
    
    if (fnode == NULL)
    {
      vtkErrorMacro("in vtkAbdoNavLogic::PerformRegistration(...): "
                    "Couldn't retrieve registration fiducial list!");
      return EXIT_FAILURE;
    }
    else if (fnode->GetNumberOfFiducials() < 3)
    {
      vtkErrorMacro("in vtkAbdoNavLogic::PerformRegistration(...): "
                    "Need to identify at least three fiducials in image space!");
      return EXIT_FAILURE;
    }
  }
  
  
  // initialize marker geometry definition depending on
  // which guidance needle tool type is being used
  //float tool[4][3];
  float tool[5][3];
  
  tool[0][0] = 0.0;
  tool[0][1] = 0.0;
  tool[0][2] = 0.0;
  tool[1][0] = 0.0;
  tool[1][1] = 0.0;
  tool[1][2] = 0.0;
  tool[2][0] = 0.0;
  tool[2][1] = 0.0;
  tool[2][2] = 0.0;
  tool[3][0] = 0.0;
  tool[3][1] = 0.0;
  tool[3][2] = 0.0;
  tool[4][0] = 0.0;
  tool[4][1] = 0.0;
  tool[4][2] = 0.0;
  
  // -----------------------------  
  // 12/28/2011 ayamada
  // receive input data and substitute into the initialized matrix
  // -----------------------------
  tipOffset[0] = this->FiducialMarkersR[0][1][0][0];
  tipOffset[1] = this->FiducialMarkersR[0][0][1][0];
  tipOffset[2] = this->FiducialMarkersR[0][0][0][1];
  
  tool[0][0] = this->FiducialMarkersR[1][1][0][0];
  tool[0][1] = this->FiducialMarkersR[1][0][1][0];
  tool[0][2] = this->FiducialMarkersR[1][0][0][1];
  
  tool[1][0] = this->FiducialMarkersR[2][1][0][0];
  tool[1][1] = this->FiducialMarkersR[2][0][1][0];
  tool[1][2] = this->FiducialMarkersR[2][0][0][1];
  
  tool[2][0] = this->FiducialMarkersR[3][1][0][0];
  tool[2][1] = this->FiducialMarkersR[3][0][1][0];
  tool[2][2] = this->FiducialMarkersR[3][0][0][1];
  
  tool[3][0] = this->FiducialMarkersR[4][1][0][0];
  tool[3][1] = this->FiducialMarkersR[4][0][1][0];
  tool[3][2] = this->FiducialMarkersR[4][0][0][1];

  tool[4][0] = this->FiducialMarkersR[5][1][0][0];
  tool[4][1] = this->FiducialMarkersR[5][0][1][0];
  tool[4][2] = this->FiducialMarkersR[5][0][0][1];  
  
  // -----------------------------  
  
  // Temporarily store source and target landmarks in order
  // to calculate the FRE (target == image cosy, source ==
  // guidance cosy).
  // TODO: It would be better to implement Getters in class
  //       vtkIGTPat2ImgRegistration and even better if the
  //       FRE were calculated in there since it is just a
  //       wrapper for class vtkLandmarkTransform anyway.
  //       Another option might be to use vtkLandmarkTransform
  //       directly.
  vtkPoints* targetLandmarks = vtkPoints::New();
  targetLandmarks->SetDataTypeToFloat();
  targetLandmarks->SetNumberOfPoints(fnode->GetNumberOfFiducials());
  vtkPoints* sourceLandmarks = vtkPoints::New();
  sourceLandmarks->SetDataTypeToFloat();
  sourceLandmarks->SetNumberOfPoints(fnode->GetNumberOfFiducials());
  
  // initialize least-squares solver
  this->Pat2ImgReg->SetNumberOfPoints(fnode->GetNumberOfFiducials());
  
  // pass point-pairs to least-squares solver
  float* tmp = NULL;
  std::cout.setf(ios::scientific, ios::floatfield);
  std::cout.precision(8);
  std::cout << "===========================================================================" << std::endl;
  std::cout << "Registration input parameters:" << std::endl;

  
  for (int i = 0; i < fnode->GetNumberOfFiducials(); i++)
  {
    if (!strcmp(tip, fnode->GetNthFiducialLabelText(i)))
    {
      tmp = fnode->GetNthFiducialXYZ(i);
      this->Pat2ImgReg->AddPoint(i, tmp[0], tmp[1], tmp[2], tipOffset[0], tipOffset[1], tipOffset[2]);
      targetLandmarks->InsertPoint(i, tmp[0], tmp[1], tmp[2]);
      sourceLandmarks->InsertPoint(i, tipOffset[0], tipOffset[1], tipOffset[2]);
      std::cout << tip << ",," << tmp[0]       << ",," << tmp[1]       << ",," << tmp[2]       << ",,[RAS]" << std::endl;
      std::cout << tip << ",," << tipOffset[0] << ",," << tipOffset[1] << ",," << tipOffset[2] << ",,[XYZ]" << std::endl;
    }
    else if (!strcmp(markerA, fnode->GetNthFiducialLabelText(i)))
    {
      tmp = fnode->GetNthFiducialXYZ(i);
      this->Pat2ImgReg->AddPoint(i, tmp[0], tmp[1], tmp[2], tool[0][0], tool[0][1], tool[0][2]);
      targetLandmarks->InsertPoint(i, tmp[0], tmp[1], tmp[2]);
      sourceLandmarks->InsertPoint(i, tool[0][0], tool[0][1], tool[0][2]);
      std::cout << markerA << ",," << tmp[0]       << ",," << tmp[1]       << ",," << tmp[2]       << ",,[RAS]" << std::endl;
      std::cout << markerA << ",," << tool[0][0]   << ",," << tool[0][1]   << ",," << tool[0][2]   << ",,[XYZ]" << std::endl;
    }
    else if (!strcmp(markerB, fnode->GetNthFiducialLabelText(i)))
    {
      tmp = fnode->GetNthFiducialXYZ(i);
      this->Pat2ImgReg->AddPoint(i, tmp[0], tmp[1], tmp[2], tool[1][0], tool[1][1], tool[1][2]);
      targetLandmarks->InsertPoint(i, tmp[0], tmp[1], tmp[2]);
      sourceLandmarks->InsertPoint(i, tool[1][0], tool[1][1], tool[1][2]);
      std::cout << markerB << ",," << tmp[0]       << ",," << tmp[1]       << ",," << tmp[2]       << ",,[RAS]" << std::endl;
      std::cout << markerB << ",," << tool[1][0]   << ",," << tool[1][1]   << ",," << tool[1][2]   << ",,[XYZ]" << std::endl;
    }
    else if (!strcmp(markerC, fnode->GetNthFiducialLabelText(i)))
    {
      tmp = fnode->GetNthFiducialXYZ(i);
      this->Pat2ImgReg->AddPoint(i, tmp[0], tmp[1], tmp[2], tool[2][0], tool[2][1], tool[2][2]);
      targetLandmarks->InsertPoint(i, tmp[0], tmp[1], tmp[2]);
      sourceLandmarks->InsertPoint(i, tool[2][0], tool[2][1], tool[2][2]);
      std::cout << markerC << ",," << tmp[0]       << ",," << tmp[1]       << ",," << tmp[2]       << ",,[RAS]" << std::endl;
      std::cout << markerC << ",," << tool[2][0]   << ",," << tool[2][1]   << ",," << tool[2][2]   << ",,[XYZ]" << std::endl;
    }
    else if (!strcmp(markerD, fnode->GetNthFiducialLabelText(i)))
    {
      tmp = fnode->GetNthFiducialXYZ(i);
      this->Pat2ImgReg->AddPoint(i, tmp[0], tmp[1], tmp[2], tool[3][0], tool[3][1], tool[3][2]);
      targetLandmarks->InsertPoint(i, tmp[0], tmp[1], tmp[2]);
      sourceLandmarks->InsertPoint(i, tool[3][0], tool[3][1], tool[3][2]);
      std::cout << markerD << ",," << tmp[0]       << ",," << tmp[1]       << ",," << tmp[2]       << ",,[RAS]" << std::endl;
      std::cout << markerD << ",," << tool[3][0]   << ",," << tool[3][1]   << ",," << tool[3][2]   << ",,[XYZ]" << std::endl;
    }
    // 1/15/2011 ayamada
    else if (!strcmp(markerE, fnode->GetNthFiducialLabelText(i)))
    {
      tmp = fnode->GetNthFiducialXYZ(i);
      this->Pat2ImgReg->AddPoint(i, tmp[0], tmp[1], tmp[2], tool[4][0], tool[4][1], tool[4][2]);
      targetLandmarks->InsertPoint(i, tmp[0], tmp[1], tmp[2]);
      sourceLandmarks->InsertPoint(i, tool[4][0], tool[4][1], tool[4][2]);
      std::cout << markerE << ",," << tmp[0]       << ",," << tmp[1]       << ",," << tmp[2]       << ",,[RAS]" << std::endl;
      std::cout << markerE << ",," << tool[4][0]   << ",," << tool[4][1]   << ",," << tool[4][2]   << ",,[XYZ]" << std::endl;
    }
  }  

  
  // calculate registration matrix
  int error = this->Pat2ImgReg->DoRegistration();
  if (error)
  {
    return EXIT_FAILURE;
  }
  
  // get registration matrix
  vtkMatrix4x4* registrationMatrix = vtkMatrix4x4::New();
  registrationMatrix->DeepCopy(this->Pat2ImgReg->GetLandmarkTransformMatrix());
  
  //----------------------------------------------------------------
  // Calculate FRE.
  //
  // NOTE: FRE calculation *MUST* be performed before replacing the
  //       translatory component of the registration matrix by the
  //       guidance needle's tip offset!
  //----------------------------------------------------------------
  // target == image cosy
  double target[3];
  // source == guidance cosy
  double source3[3];
  double source4[4];
  double registeredSource4[4];
  
  double sum = 0.0;
  
  for (int r = 0; r < fnode->GetNumberOfFiducials(); r++)
  {
    targetLandmarks->GetPoint(r, target);
    sourceLandmarks->GetPoint(r, source3);
    source4[0] = source3[0];
    source4[1] = source3[1];
    source4[2] = source3[2];
    source4[3] = 1;
    registrationMatrix->MultiplyPoint(source4, registeredSource4);
    
    sum = sum + sqrt(( registeredSource4[0] - target[0] ) * ( registeredSource4[0] - target[0] )
                     + ( registeredSource4[1] - target[1] ) * ( registeredSource4[1] - target[1] )
                     + ( registeredSource4[2] - target[2] ) * ( registeredSource4[2] - target[2] ));
  }  

  // 4/20/2012 ayamada  
  sum = sum / fnode->GetNumberOfFiducials();
  //sum = sum * sqrt(1.0-1.0/2.0/fnode->GetNumberOfFiducials());
  
  // 4/20/2012 ayamada
  std::cout << "sum,," << sum << std::endl;
  std::cout << "fnode->GetNumberOfFiducials(),," << fnode->GetNumberOfFiducials() << std::endl;
  
  
  this->AbdoNavNode->SetFRE(sum);
  std::cout << "===========================================================================" << std::endl;
  std::cout << "FRE,," << this->AbdoNavNode->GetFRE() << std::endl;
  std::cout << "===========================================================================" << std::endl;
  std::cout << "Registration matrix *BEFORE* replacement:" << std::endl;
  std::cout << registrationMatrix->GetElement(0, 0) << ",," << registrationMatrix->GetElement(0, 1) << ",," << registrationMatrix->GetElement(0, 2) << ",," << registrationMatrix->GetElement(0, 3) << std::endl;
  std::cout << registrationMatrix->GetElement(1, 0) << ",," << registrationMatrix->GetElement(1, 1) << ",," << registrationMatrix->GetElement(1, 2) << ",," << registrationMatrix->GetElement(1, 3) << std::endl;
  std::cout << registrationMatrix->GetElement(2, 0) << ",," << registrationMatrix->GetElement(2, 1) << ",," << registrationMatrix->GetElement(2, 2) << ",," << registrationMatrix->GetElement(2, 3) << std::endl;
  std::cout << registrationMatrix->GetElement(3, 0) << ",," << registrationMatrix->GetElement(3, 1) << ",," << registrationMatrix->GetElement(3, 2) << ",," << registrationMatrix->GetElement(3, 3) << std::endl;
  
  // The calculated registration matrix describes the relationship between
  // the image coordinate system and the guidance needle's local coordinate
  // system at markerA. By performing the pivoting procedure, however, the
  // guidance needle's local coordinate system is translated from markerA
  // to the guidance needle's tip. Thus, the translatory component of the
  // registration matrix needs to be replaced by the guidance needle's tip
  // offset.
  
  
  // 12/28/2011 ayamada
  // cut these calculation because the offset never be used in our purpose.
  std::cout << "===========================================================================" << std::endl;
  std::cout << "Registration matrix *AFTER* replacement:" << std::endl;
  std::cout << registrationMatrix->GetElement(0, 0) << ",," << registrationMatrix->GetElement(0, 1) << ",," << registrationMatrix->GetElement(0, 2) << ",," << registrationMatrix->GetElement(0, 3) << std::endl;
  std::cout << registrationMatrix->GetElement(1, 0) << ",," << registrationMatrix->GetElement(1, 1) << ",," << registrationMatrix->GetElement(1, 2) << ",," << registrationMatrix->GetElement(1, 3) << std::endl;
  std::cout << registrationMatrix->GetElement(2, 0) << ",," << registrationMatrix->GetElement(2, 1) << ",," << registrationMatrix->GetElement(2, 2) << ",," << registrationMatrix->GetElement(2, 3) << std::endl;
  std::cout << registrationMatrix->GetElement(3, 0) << ",," << registrationMatrix->GetElement(3, 1) << ",," << registrationMatrix->GetElement(3, 2) << ",," << registrationMatrix->GetElement(3, 3) << std::endl;
  std::cout.unsetf(ios::floatfield);
  std::cout.precision(6);
  // create/retrieve registration transform node
  if (this->AbdoNavNode->GetRegistrationTransformID() == NULL)
  {
    // create a new registration transform node and add it to the scene
    this->RegistrationTransform = vtkMRMLLinearTransformNode::New();
    this->RegistrationTransform->SetName("AbdoNav-RegistrationTransform");
    this->RegistrationTransform->SetDescription("Created by AbdoNav");
    this->GetMRMLScene()->AddNode(this->RegistrationTransform);
    this->RegistrationTransform->Delete();
    // set registration transform node ID in AbdoNavNode
    this->AbdoNavNode->SetRegistrationTransformID(this->RegistrationTransform->GetID());
    // get relative tracking transform node and make it observe the registration transform node
    if (this->AbdoNavNode->GetTrackingTransformID() != NULL)
    {
      vtkMRMLLinearTransformNode* relativeTrackingTransform = vtkMRMLLinearTransformNode::SafeDownCast(this->GetMRMLScene()->GetNodeByID(this->AbdoNavNode->GetTrackingTransformID()));
      relativeTrackingTransform->SetAndObserveTransformNodeID(this->RegistrationTransform->GetID());
    }
    else
    {
      vtkErrorMacro("in vtkAbdoNavLogic::PerformRegistration(): "
                    "Couldn't move relative tracking transform node below registration transform node "
                    "because vtkMRMLAbdoNavNode::GetTrackingTransformID() returned NULL!");
    }
  }
  else
  {
    this->RegistrationTransform = vtkMRMLLinearTransformNode::SafeDownCast(this->GetMRMLScene()->GetNodeByID(this->AbdoNavNode->GetRegistrationTransformID()));
  }
  
  // copy registration matrix into registration transform node
  this->RegistrationTransform->GetMatrixTransformToParent()->DeepCopy(registrationMatrix);
  // indicate that the Scene contains unsaved changes; make Slicer's save dialog list this transform as modified
  this->RegistrationTransform->SetModifiedSinceRead(1);
  // indicate that registration has been performed
  this->RegistrationPerformed = 1;
  
  
  // 12/27/2011 ayamada
  // copy the registration matrix to the z-frame matrix
  vtkMRMLRobotNode* robot=GetRobotNode();
  // 1/20/2012 ayamada
  //vtkMRMLBrpRobotCommandNode* commandNode;
  
  if (robot/*commandNode*/==NULL)
  {
    
    //std::cerr << "matrix copy 1" << std::endl;
    
    return 0; // failed
  }else{

  //std::cerr << "matrix copy 2" << std::endl;
    
  vtkMRMLLinearTransformNode* rTransformNode 
    = vtkMRMLLinearTransformNode::SafeDownCast(this->GetMRMLScene()->GetNodeByID(robot->GetCalibrationObjectTransformId()));
  vtkMatrix4x4* rTransform = vtkMatrix4x4::New();
  
  // copy registration matrix into registration transform node
  rTransformNode->GetMatrixTransformToParent()->DeepCopy(registrationMatrix);
  // indicate that the Scene contains unsaved changes; make Slicer's save dialog list this transform as modified
  rTransformNode->SetModifiedSinceRead(1);
    
    
  }
  
  // 12/28/2011 ayamada
  this->ShowRobotModel(0);
  this->ShowRobotModel(1);
  
  // 12/28/2011 ayamada
  this->CalculateVirtualCenterPosition();

  
  // clean up
  registrationMatrix->Delete();
  targetLandmarks->Delete();
  sourceLandmarks->Delete();
  
  return EXIT_SUCCESS;
  
  
}


//---------------------------------------------------------------------------
vtkMRMLAbdoNavNode* vtkRobotProbeNavLogic::CheckAndCreateAbdoNavNode()
{
}

//---------------------------------------------------------------------------
int vtkRobotProbeNavLogic::ObserveTrackingTransformNode()
{
  
  if (this->AbdoNavNode == NULL)
  {
    vtkErrorMacro("in vtkAbdoNavLogic::ObserveTrackingTransformNode(): "
                  "Couldn't retrieve AbdoNavNode!");
    return EXIT_FAILURE;
  }
  
  vtkMRMLLinearTransformNode* tnode = vtkMRMLLinearTransformNode::SafeDownCast(this->GetMRMLScene()->GetNodeByID(this->AbdoNavNode->GetTrackingTransformID()));
  if (tnode)
  {
    vtkIntArray* nodeEvents = vtkIntArray::New();
    nodeEvents->InsertNextValue(vtkMRMLTransformableNode::TransformModifiedEvent);
    vtkSetAndObserveMRMLNodeEventsMacro(this->RelativeTrackingTransform, tnode, nodeEvents);
    nodeEvents->Delete();
    tnode->InvokeEvent(vtkMRMLTransformableNode::TransformModifiedEvent);
  }
  else
  {
    vtkErrorMacro("in vtkAbdoNavLogic::ObserveTrackingTransformNode(): "
                  "Couldn't retrieve tracking transform node!");
    return EXIT_FAILURE;
  }
  
  return EXIT_SUCCESS;
}


