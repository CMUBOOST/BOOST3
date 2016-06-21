#ifndef FEEDBACK_HPP
#define FEEDBACK_HPP

#include "hebi_feedback.h"
#include "color.hpp"
#include "vector_3_f.hpp"
#include "util.hpp"

namespace hebi {

class Feedback final
{
    class FloatField final
    {
      public:
        FloatField(HebiFeedbackPtr internal, FeedbackFloatField field);
        bool has() const;
        float get() const;

      private:
        HebiFeedbackPtr const internal_;
        FeedbackFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(FloatField)
    };
    class HighResAngleField final
    {
      public:
        HighResAngleField(HebiFeedbackPtr internal, FeedbackHighResAngleField field);
        bool has() const;
        double get() const;
        void get(int64_t* revolutions, float* radian_offset) const;

      private:
        HebiFeedbackPtr const internal_;
        FeedbackHighResAngleField const field_;

        HEBI_DISABLE_COPY_MOVE(HighResAngleField)
    };

    class NumberedFloatField final
    {
      public:
        NumberedFloatField(HebiFeedbackPtr internal, FeedbackNumberedFloatField field);
        bool has(int fieldNumber) const;
        float get(int fieldNumber) const;

      private:
        HebiFeedbackPtr const internal_;
        FeedbackNumberedFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(NumberedFloatField)
    };

    class Vector3fField final
    {
      public:
        Vector3fField(HebiFeedbackPtr internal, FeedbackVector3fField field);
        bool has() const;
        Vector3f get() const;

      private:
        HebiFeedbackPtr const internal_;
        FeedbackVector3fField const field_;

        HEBI_DISABLE_COPY_MOVE(Vector3fField)
    };

    class LedField final
    {
      public:
        LedField(HebiFeedbackPtr internal, FeedbackLedField field);
        bool hasColor() const;
        Color getColor() const;

      private:
        HebiFeedbackPtr const internal_;
        FeedbackLedField const field_;

        HEBI_DISABLE_COPY_MOVE(LedField)
    };

    class Actuator final
    {
      public:
        Actuator(HebiFeedbackPtr internal)
          : internal_(internal),
            velocity_(internal, FeedbackFloatVelocity),
            torque_(internal, FeedbackFloatTorque),
            velocity_command_(internal, FeedbackFloatVelocityCommand),
            torque_command_(internal, FeedbackFloatTorqueCommand),
            deflection_(internal, FeedbackFloatDeflection),
            deflection_velocity_(internal, FeedbackFloatDeflectionVelocity),
            motor_velocity_(internal, FeedbackFloatMotorVelocity),
            motor_current_(internal, FeedbackFloatMotorCurrent),
            motor_temperature_(internal, FeedbackFloatMotorTemperature),
            motor_winding_current_(internal, FeedbackFloatMotorWindingCurrent),
            motor_winding_temperature_(internal, FeedbackFloatMotorWindingTemperature),
            actuator_temperature_(internal, FeedbackFloatActuatorTemperature),
            position_(internal, FeedbackHighResAnglePosition),
            position_command_(internal, FeedbackHighResAnglePositionCommand)
        {
        }
    
        const FloatField& velocity() const { return velocity_; }
        const FloatField& torque() const { return torque_; }
        const FloatField& velocityCommand() const { return velocity_command_; }
        const FloatField& torqueCommand() const { return torque_command_; }
        const FloatField& deflection() const { return deflection_; }
        const FloatField& deflectionVelocity() const { return deflection_velocity_; }
        const FloatField& motorVelocity() const { return motor_velocity_; }
        const FloatField& motorCurrent() const { return motor_current_; }
        const FloatField& motorTemperature() const { return motor_temperature_; }
        const FloatField& motorWindingCurrent() const { return motor_winding_current_; }
        const FloatField& motorWindingTemperature() const { return motor_winding_temperature_; }
        const FloatField& actuatorTemperature() const { return actuator_temperature_; }
        const HighResAngleField& position() const { return position_; }
        const HighResAngleField& positionCommand() const { return position_command_; }
    
      private:
        HebiFeedbackPtr const internal_;
    
        FloatField velocity_;
        FloatField torque_;
        FloatField velocity_command_;
        FloatField torque_command_;
        FloatField deflection_;
        FloatField deflection_velocity_;
        FloatField motor_velocity_;
        FloatField motor_current_;
        FloatField motor_temperature_;
        FloatField motor_winding_current_;
        FloatField motor_winding_temperature_;
        FloatField actuator_temperature_;
        HighResAngleField position_;
        HighResAngleField position_command_;
    
        HEBI_DISABLE_COPY_MOVE(Actuator)
    };

    class Imu final
    {
      public:
        Imu(HebiFeedbackPtr internal)
          : internal_(internal),
            accelerometer_(internal, FeedbackVector3fAccelerometer),
            gyro_(internal, FeedbackVector3fGyro)
        {
        }
    
        const Vector3fField& accelerometer() const { return accelerometer_; }
        const Vector3fField& gyro() const { return gyro_; }
    
      private:
        HebiFeedbackPtr const internal_;
    
        Vector3fField accelerometer_;
        Vector3fField gyro_;
    
        HEBI_DISABLE_COPY_MOVE(Imu)
    };

  public:
    /**
     * C-style feedback object.
     * NOTE: this should not be used except by internal library functions!
     */
    HebiFeedbackPtr const internal_;

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    bool manage_pointer_lifetime_;

  public:
    /**
     * Create a new feedback object for a single module.
     */
    Feedback();
    /**
     * Wraps an existing C-style feedback object; object lifetime is assumed to
     * be managed by the caller.
     * NOTE: this should not be used except by internal library functions!
     */
    Feedback(HebiFeedbackPtr feedback);
    /**
     * Move constructor (necessary for containment in STL template classes)
     */
    Feedback(Feedback&& other);

    /**
     * Destructor cleans up feedback object as necessary.
     */
    virtual ~Feedback() noexcept; /* annotating specified destructor as noexcept is best-practice */

    // Submessage getters: Note that the returned reference should not be used
    // after the lifetime of this parent.
    const Actuator& actuator() const;
    const Imu& imu() const;

    const FloatField& ambientTemperature() const;
    const FloatField& processorTemperature() const;
    const FloatField& voltage() const;
    const NumberedFloatField& debug() const;
    const LedField& led() const;

  private:
    Actuator actuator_;
    Imu imu_;

    FloatField ambient_temperature_;
    FloatField processor_temperature_;
    FloatField voltage_;
    NumberedFloatField debug_;
    LedField led_;

    /**
     * Disable copy constructor/assignment operators
     */
    HEBI_DISABLE_COPY(Feedback)

    /* Disable move assigment operator. */
    Feedback& operator= (const Feedback&& other) = delete;
};

} // namespace hebi

#endif // FEEDBACK_HPP
