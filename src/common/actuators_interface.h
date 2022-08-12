#ifndef _actuators_interface_h_
#define _actuators_interface_h_

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "elmo_motor.h"

using JointParam_t = MotorParam_t;

typedef struct
{
  int8_t (*init)(const char *ifname, double dt, MotorOptions_t opt);
  int8_t (*deInit)(void);
  int8_t (*setJointOffset)(double_t *offset, uint16_t len);
  int8_t (*setJointPosition)(uint8_t *ids, uint8_t id_num, JointParam_t *param);
  int8_t (*setJointVelocity)(uint8_t *ids, uint8_t id_num, JointParam_t *param);
  int8_t (*setJointTorque)(uint8_t *ids, uint8_t id_num, JointParam_t *param);
  int8_t (*getJointData)(uint8_t *ids, uint8_t id_num, JointParam_t *data);
} ActuatorsInterface_t;

int8_t actuatorsInterfaceSetup(const char *type, ActuatorsInterface_t *interfacePtr);

#endif
