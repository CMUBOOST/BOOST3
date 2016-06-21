/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_INFO_H
#define HEBI_INFO_H

#include "stdint.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiInfo* HebiInfoPtr;

HebiInfoPtr hebiInfoCreate();

// Define all fields
typedef enum InfoFloatField {
InfoFloatPositionKp, ///Proportional PID gain for position
InfoFloatPositionKi, ///Integral PID gain for position
InfoFloatPositionKd, ///Derivative PID gain for position
InfoFloatPositionFeedForward, ///TODO: describe gain
InfoFloatPositionDeadZone, ///TODO: describe gain
InfoFloatPositionIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
InfoFloatPositionPunch, ///TODO: describe gain
InfoFloatPositionMinTarget, ///TODO: describe gain
InfoFloatPositionMaxTarget, ///TODO: describe gain
InfoFloatPositionTargetLowpass, ///TODO: describe gain
InfoFloatPositionMinOutput, ///TODO: describe gain
InfoFloatPositionMaxOutput, ///TODO: describe gain
InfoFloatPositionOutputLowpass, ///TODO: describe gain
InfoFloatVelocityKp, ///Proportional PID gain for velocity
InfoFloatVelocityKi, ///Integral PID gain for velocity
InfoFloatVelocityKd, ///Derivative PID gain for velocity
InfoFloatVelocityFeedForward, ///TODO: describe gain
InfoFloatVelocityDeadZone, ///TODO: describe gain
InfoFloatVelocityIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
InfoFloatVelocityPunch, ///TODO: describe gain
InfoFloatVelocityMinTarget, ///TODO: describe gain
InfoFloatVelocityMaxTarget, ///TODO: describe gain
InfoFloatVelocityTargetLowpass, ///TODO: describe gain
InfoFloatVelocityMinOutput, ///TODO: describe gain
InfoFloatVelocityMaxOutput, ///TODO: describe gain
InfoFloatVelocityOutputLowpass, ///TODO: describe gain
InfoFloatTorqueKp, ///Proportional PID gain for torque
InfoFloatTorqueKi, ///Integral PID gain for torque
InfoFloatTorqueKd, ///Derivative PID gain for torque
InfoFloatTorqueFeedForward, ///TODO: describe gain
InfoFloatTorqueDeadZone, ///TODO: describe gain
InfoFloatTorqueIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
InfoFloatTorquePunch, ///TODO: describe gain
InfoFloatTorqueMinTarget, ///TODO: describe gain
InfoFloatTorqueMaxTarget, ///TODO: describe gain
InfoFloatTorqueTargetLowpass, ///TODO: describe gain
InfoFloatTorqueMinOutput, ///TODO: describe gain
InfoFloatTorqueMaxOutput, ///TODO: describe gain
InfoFloatTorqueOutputLowpass, ///TODO: describe gain
InfoFloatSpringConstant, ///The spring constant of the module.
} InfoFloatField;

typedef enum InfoStringField {
InfoStringName, ///Sets the name for this module. Name must be null-terminated character string for the name; must be <= 20 characters.
InfoStringFamily, ///Sets the family for this module. Name must be null-terminated character string for the family; must be <= 20 characters.
} InfoStringField;

typedef enum InfoFlagField {
InfoFlagSaveCurrentSettings, ///Indicates if the module should save the current values of all of its settings.
InfoFlagPositionDOnError, ///TODO: describe gain
InfoFlagVelocityDOnError, ///TODO: describe gain
InfoFlagTorqueDOnError, ///TODO: describe gain
} InfoFlagField;

typedef enum InfoEnumField {
InfoEnumControlStrategy, ///How the PID loops are connected; see docs.hebi.us for descriptions of the available options.
} InfoEnumField;

typedef enum InfoLedField {
InfoLedLed, ///The module's LED.
} InfoLedField;

/**
 * Returns value for given field. If HasFloat() returns 0, this results in
 * undefined behavior.
 */
float hebiInfoGetFloat(HebiInfoPtr, InfoFloatField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiInfoHasFloat(HebiInfoPtr, InfoFloatField);

/**
 * Returns value for given field. If HasString() returns 0, this results in
 * undefined behavior.
 */
const char* hebiInfoGetString(HebiInfoPtr, InfoStringField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiInfoHasString(HebiInfoPtr, InfoStringField);

/**
 * Checks whether this flag is set.  Returns '1' for yes, '0' for no.
 */
int hebiInfoHasFlag(HebiInfoPtr, InfoFlagField);

/**
 * Returns value for given field. If HasEnum() returns 0, this results in
 * undefined behavior.
 */
int hebiInfoGetEnum(HebiInfoPtr, InfoEnumField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiInfoHasEnum(HebiInfoPtr, InfoEnumField);

/**
 * Returns led color into the three output integers (each 0-255). If HasLedColor() returns 0, this results in
 * undefined behavior.
 */
void hebiInfoGetLedColor(HebiInfoPtr, InfoLedField, uint8_t *r, uint8_t *g, uint8_t *b);
/**
 * Indicates whether the led color is set; returns '1' for yes and '0' for no. For command-style messages, this refers
 * to a command to override the module's default control of the LED.
 */
int hebiInfoHasLedColor(HebiInfoPtr, InfoLedField);

void hebiInfoDestroy(HebiInfoPtr);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_INFO_H
