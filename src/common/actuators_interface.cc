#include "actuators_interface.h"

int8_t actuatorsInterfaceSetup(const char *type, ActuatorsInterface_t *interfacePtr)
{
  if (strcmp(type, "real") == 0)
  {
    interfacePtr->init = EM_init;
    interfacePtr->deInit = EM_deInit;
    interfacePtr->setJointOffset = EM_setPositionsOffset;
    interfacePtr->setJointPosition = EM_setPositions;
    interfacePtr->setJointVelocity = EM_setVelocities;
    interfacePtr->setJointTorque = EM_setTorques;
    interfacePtr->getJointData = EM_getData;
    printf("Info: actuatorsInterfaceSetup success!\n");
    return 0;
  }
  printf("Error: actuatorsInterfaceSetup failed!\n");
  return -1;
}