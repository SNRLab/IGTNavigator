PROJECT(RobotProbeNav)

cmake_minimum_required(VERSION 2.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# --------------------------------------------------------------------------
# Find Slicer3

if(NOT Slicer3_SOURCE_DIR)
  find_package(Slicer3 REQUIRED)
  include(${Slicer3_USE_FILE})
  slicer3_set_default_install_prefix_for_external_projects()
endif(NOT Slicer3_SOURCE_DIR)

# --------------------------------------------------------------------------
# Find Slicer3

#find_package(Subversion)
#if (Subversion_FOUND)
#  Subversion_WC_INFO(${CMAKE_HOME_DIRECTORY} RobotProbeNav)
#  set(RobotProbeNav_REVISION "${RobotProbeNav_WC_REVISION}")
#endif (Subversion_FOUND)

# --------------------------------------------------------------------------
# Include dirs


#
# Include dirs
#


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeTemplate
  ${CMAKE_CURRENT_SOURCE_DIR}/TransRectalRobotProbeRobot
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat
  ${CMAKE_CURRENT_SOURCE_DIR}/ZFrame
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Slicer3_Libs_INCLUDE_DIRS}
  ${Slicer3_Base_INCLUDE_DIRS}
  ${OpenIGTLinkIF_SOURCE_DIR}
  ${OpenIGTLinkIF_BINARY_DIR}
  ${OpenIGTLink_INCLUDE_DIRS}
  ${vtkSlicerVolumesModuleLogic_SOURCE_DIR}
  ${vtkSlicerVolumesModuleLogic_BINARY_DIR}
  ${Volumes_SOURCE_DIR}
  ${Volumes_BINARY_DIR}
  )


#
# Configure include file
#
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/vtkRobotProbeNavConfigure.h.in 
  ${CMAKE_CURRENT_BINARY_DIR}/vtkRobotProbeNavConfigure.h
  )
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/vtkRobotProbeNavVersion.h.in 
  ${CMAKE_CURRENT_BINARY_DIR}/vtkRobotProbeNavVersion.h
  )

#
# Installation
#

file(GLOB headers 
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeTemplate/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/TransRectalRobotProbeRobot/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Wizard/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/newmat/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/ZFrame/*.h"
  )

install(FILES 
  ${headers} 
  "${CMAKE_CURRENT_BINARY_DIR}/vtkRobotProbeNavConfigure.h"
  DESTINATION ${Slicer3_INSTALL_MODULES_INCLUDE_DIR}/${PROJECT_NAME} COMPONENT Development
  )

# Install transrectal robot files
set(TransRectalRobotSubdir TransRectalRobotProbeRobot)
foreach(TransRectalRobotFile 
  Sheath.stl
  Workspace150mm.stl
  Workspace200mm.stl
  )

  # Copy to the build tree (for Slicer execution in the Slicer3-build directory)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/${TransRectalRobotSubdir}/${TransRectalRobotFile}   
    ${CMAKE_BINARY_DIR}/${Slicer3_INSTALL_MODULES_SHARE_DIR}/${PROJECT_NAME}/${TransRectalRobotSubdir}/${TransRectalRobotFile}
    COPYONLY
    )
  # Copy to the installation target (for Slicer execution when it is installed)
  install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/${TransRectalRobotSubdir}/${TransRectalRobotFile} 
    DESTINATION ${Slicer3_INSTALL_MODULES_SHARE_DIR}/${PROJECT_NAME}/${TransRectalRobotSubdir}
    )
endforeach(TransRectalRobotFile)  


# 9/22/2011 ayamada
# Install IGT robot files
set(IGTRobotSubdir IGTRobot)
foreach(IGTRobotFile 
  new-loopCoil.stl
  new-bodyMechanisms.stl
  new-motorMechanisms.stl
  new-hooperMechanisms.stl
  new-needleBoxMechanisms.stl
  )
  # Copy to the build tree (for Slicer execution in the Slicer3-build directory)
  configure_file(
    # 9/22/2011 ayamada
    ${CMAKE_CURRENT_SOURCE_DIR}/${IGTRobotSubdir}/${IGTRobotFile}
    #${CMAKE_BINARY_DIR}/${Slicer3_INSTALL_MODULES_SHARE_DIR}/${PROJECT_NAME}/${IGTRobotSubdir}/${IGTRobotFile}
    COPYONLY
    )
  # Copy to the installation target (for Slicer execution when it is installed)
  install(
    # 9/22/2011 ayamada
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/${IGTRobotSubdir}/${IGTRobotFile}
    DESTINATION ${Slicer3_INSTALL_MODULES_SHARE_DIR}/${PROJECT_NAME}/${IGTRobotSubdir}
    )
endforeach(IGTRobotFile)  


# --------------------------------------------------------------------------
# Sources

set(RobotProbeNav_SRCS 
  vtkMRMLRobotProbeNavManagerNode.cxx
  vtkRobotProbeNavLogic.cxx
  vtkRobotProbeNavGUI.cxx
  vtkSlicerSecondaryViewerWindow.cxx 
  vtkMRMLRobotNode.cxx
  vtkMRMLRobotDisplayNode.cxx
  vtkRobotProbeNavTargetDescriptor.cxx  
  vtkRobotToImageRegistration.cxx
  vtkZFrameRobotToImageRegistration.cxx
  # 12/25/2011 ayamada
  vtkMRMLAbdoNavNode.cxx
  #vtkMRMLRobotProbeNavNode.cxx
  #vtkMRMLRobotProbeNavManagerNode2.cxx

  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot/vtkIGTLToMRMLCoordinate.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot/vtkIGTLToMRMLBrpRobotCommand.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot/vtkMRMLBrpRobotCommandNode.cxx  
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot/vtkMRMLIGTProbeRobotNode.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeTemplate/vtkMRMLIGTProbeTemplateNode.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/TransRectalRobotProbeRobot/vtkMRMLTransRectalRobotProbeRobotNode.cxx  
  ${CMAKE_CURRENT_SOURCE_DIR}/TransRectalRobotProbeRobot/vtkTransRectalFiducialCalibrationAlgo.cxx  
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard/vtkRobotProbeNavStep.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard/vtkRobotProbeNavStepSetUp.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard/vtkRobotProbeNavStepSetUpTemplate.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard/vtkRobotProbeNavCalibrationStep.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard/vtkRobotProbeNavFiducialCalibrationStep.cxx   
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard/vtkRobotProbeNavTargetingStep.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard/vtkRobotProbeNavManualControlStep.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard/vtkRobotProbeNavStepVerification.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Wizard/vtkRobotProbeNavStepTargetingTemplate.cxx
  )

set(RobotProbeNavIGTL_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot/igtlCoordinateMessage.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot/igtlHeaderMessage.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot/igtlMoveToMessage.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IGTProbeRobot/igtlSetZFrameMessage.cxx
)

set(RobotProbeNavNewmat_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmat9.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmatrm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/nm_misc.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/bandmat.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmat1.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/solution.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/cholesky.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/hholder.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmat2.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmatex.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmat3.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/sort.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/evalue.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/jacobi.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmat4.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/submat.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/example.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmat5.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/svd.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/myexcept.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmat6.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmatnl.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/fft.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmat7.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newfft.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/newmat8.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/newmat/ZLinAlg.cxx
)

set(RobotProbeNavZFrame_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/ZFrame/ZFrameCalibration.cxx
)


# --------------------------------------------------------------------------
# Wrapping

include("${VTK_CMAKE_DIR}/vtkWrapTcl.cmake")
vtk_wrap_tcl3(RobotProbeNav RobotProbeNav_TCL_SRCS "${RobotProbeNav_SRCS}" "")


#---------------------------------------------------------------------------
# Add Loadable Module support

generatelm(RobotProbeNav_SRCS RobotProbeNav.txt)


# --------------------------------------------------------------------------
# Build and install the library

set(lib_name RobotProbeNav)

add_library(${lib_name}
  ${RobotProbeNav_SRCS} 
  ${RobotProbeNavIGTL_SRCS}
  ${RobotProbeNavNewmat_SRCS} 
  ${RobotProbeNavZFrame_SRCS}
  ${RobotProbeNav_TCL_SRCS}
  )
slicer3_set_modules_output_path(${lib_name})

#link_directories(
#  ${Slicer3_DIR}/${Slicer3_INSTALL_MODULES_LIB_DIR}
#)


target_link_libraries(${lib_name}
  ${Slicer3_Libs_LIBRARIES}
  ${Slicer3_Base_LIBRARIES}
  ${KWWidgets_LIBRARIES}
  ${ITK_LIBRARIES}
  OpenIGTLinkIF
  vtkSlicerVolumesModuleLogic
  Volumes
  )


slicer3_install_modules(${lib_name})

