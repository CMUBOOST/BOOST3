#ifndef COMMAND_HPP
#define COMMAND_HPP

#include "hebi_command.h"
#include "color.hpp"
#include <string>
#include "util.hpp"

namespace hebi {

class Command final
{
  public:
    enum ControlStrategy {
      Off,
      DirectPWM,
      Strategy2,
      Strategy3,
      Strategy4,
    };

  private:
    class FloatField final
    {
      public:
        FloatField(HebiCommandPtr internal, CommandFloatField field);
        bool has() const;
        float get() const;
        void set(float value);
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(FloatField)
    };
    class HighResAngleField final
    {
      public:
        HighResAngleField(HebiCommandPtr internal, CommandHighResAngleField field);
        bool has() const;
        double get() const;
        void get(int64_t* revolutions, float* radian_offset) const;
        void set(double radians);
        void set(int64_t revolutions, float radian_offset);
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandHighResAngleField const field_;

        HEBI_DISABLE_COPY_MOVE(HighResAngleField)
    };

    class NumberedFloatField final
    {
      public:
        NumberedFloatField(HebiCommandPtr internal, CommandNumberedFloatField field);
        bool has(int fieldNumber) const;
        float get(int fieldNumber) const;
        void set(int fieldNumber, float value);
        void clear(int fieldNumber);

      private:
        HebiCommandPtr const internal_;
        CommandNumberedFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(NumberedFloatField)
    };

    class StringField final
    {
      public:
        StringField(HebiCommandPtr internal, CommandStringField field);
        bool has() const;
        std::string get() const;
        void set(const std::string& value);
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandStringField const field_;

        HEBI_DISABLE_COPY_MOVE(StringField)
    };

    class FlagField final
    {
      public:
        FlagField(HebiCommandPtr internal, CommandFlagField field);
        bool has() const;
        void set();
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandFlagField const field_;

        HEBI_DISABLE_COPY_MOVE(FlagField)
    };

    template <class T>
    class EnumField final
    {
      public:
        EnumField(HebiCommandPtr internal, CommandEnumField field)
          : internal_(internal), field_(field) {}
        bool has() const { return (hebiCommandHasEnum(internal_, field_) == 1); }
        T get() const { return (T)hebiCommandGetEnum(internal_, field_); }
        void set(T value) { hebiCommandSetEnum(internal_, field_, value); }
        void clear() { hebiCommandClearEnum(internal_, field_); }

      private:
        HebiCommandPtr const internal_;
        CommandEnumField const field_;

        HEBI_DISABLE_COPY_MOVE(EnumField)
    };

    class LedField final
    {
      public:
        LedField(HebiCommandPtr internal, CommandLedField field);
        bool hasColor() const;
        bool hasModuleControl() const;
        Color getColor() const;
        void setOverrideColor(const Color& new_color);
        void setModuleControl();
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandLedField const field_;

        HEBI_DISABLE_COPY_MOVE(LedField)
    };

    class Settings final
    {
      class Actuator final
      {
        class PositionGains final
        {
          public:
            PositionGains(HebiCommandPtr internal)
              : internal_(internal),
                position_kp_(internal, CommandFloatPositionKp),
                position_ki_(internal, CommandFloatPositionKi),
                position_kd_(internal, CommandFloatPositionKd),
                position_feed_forward_(internal, CommandFloatPositionFeedForward),
                position_dead_zone_(internal, CommandFloatPositionDeadZone),
                position_i_clamp_(internal, CommandFloatPositionIClamp),
                position_punch_(internal, CommandFloatPositionPunch),
                position_min_target_(internal, CommandFloatPositionMinTarget),
                position_max_target_(internal, CommandFloatPositionMaxTarget),
                position_target_lowpass_(internal, CommandFloatPositionTargetLowpass),
                position_min_output_(internal, CommandFloatPositionMinOutput),
                position_max_output_(internal, CommandFloatPositionMaxOutput),
                position_output_lowpass_(internal, CommandFloatPositionOutputLowpass),
                position_d_on_error_(internal, CommandFlagPositionDOnError)
            {
            }
        
            FloatField& positionKp() { return position_kp_; }
            FloatField& positionKi() { return position_ki_; }
            FloatField& positionKd() { return position_kd_; }
            FloatField& positionFeedForward() { return position_feed_forward_; }
            FloatField& positionDeadZone() { return position_dead_zone_; }
            FloatField& positionIClamp() { return position_i_clamp_; }
            FloatField& positionPunch() { return position_punch_; }
            FloatField& positionMinTarget() { return position_min_target_; }
            FloatField& positionMaxTarget() { return position_max_target_; }
            FloatField& positionTargetLowpass() { return position_target_lowpass_; }
            FloatField& positionMinOutput() { return position_min_output_; }
            FloatField& positionMaxOutput() { return position_max_output_; }
            FloatField& positionOutputLowpass() { return position_output_lowpass_; }
            FlagField& positionDOnError() { return position_d_on_error_; }
        
          private:
            HebiCommandPtr const internal_;
        
            FloatField position_kp_;
            FloatField position_ki_;
            FloatField position_kd_;
            FloatField position_feed_forward_;
            FloatField position_dead_zone_;
            FloatField position_i_clamp_;
            FloatField position_punch_;
            FloatField position_min_target_;
            FloatField position_max_target_;
            FloatField position_target_lowpass_;
            FloatField position_min_output_;
            FloatField position_max_output_;
            FloatField position_output_lowpass_;
            FlagField position_d_on_error_;
        
            HEBI_DISABLE_COPY_MOVE(PositionGains)
        };
      
        class VelocityGains final
        {
          public:
            VelocityGains(HebiCommandPtr internal)
              : internal_(internal),
                velocity_kp_(internal, CommandFloatVelocityKp),
                velocity_ki_(internal, CommandFloatVelocityKi),
                velocity_kd_(internal, CommandFloatVelocityKd),
                velocity_feed_forward_(internal, CommandFloatVelocityFeedForward),
                velocity_dead_zone_(internal, CommandFloatVelocityDeadZone),
                velocity_i_clamp_(internal, CommandFloatVelocityIClamp),
                velocity_punch_(internal, CommandFloatVelocityPunch),
                velocity_min_target_(internal, CommandFloatVelocityMinTarget),
                velocity_max_target_(internal, CommandFloatVelocityMaxTarget),
                velocity_target_lowpass_(internal, CommandFloatVelocityTargetLowpass),
                velocity_min_output_(internal, CommandFloatVelocityMinOutput),
                velocity_max_output_(internal, CommandFloatVelocityMaxOutput),
                velocity_output_lowpass_(internal, CommandFloatVelocityOutputLowpass),
                velocity_d_on_error_(internal, CommandFlagVelocityDOnError)
            {
            }
        
            FloatField& velocityKp() { return velocity_kp_; }
            FloatField& velocityKi() { return velocity_ki_; }
            FloatField& velocityKd() { return velocity_kd_; }
            FloatField& velocityFeedForward() { return velocity_feed_forward_; }
            FloatField& velocityDeadZone() { return velocity_dead_zone_; }
            FloatField& velocityIClamp() { return velocity_i_clamp_; }
            FloatField& velocityPunch() { return velocity_punch_; }
            FloatField& velocityMinTarget() { return velocity_min_target_; }
            FloatField& velocityMaxTarget() { return velocity_max_target_; }
            FloatField& velocityTargetLowpass() { return velocity_target_lowpass_; }
            FloatField& velocityMinOutput() { return velocity_min_output_; }
            FloatField& velocityMaxOutput() { return velocity_max_output_; }
            FloatField& velocityOutputLowpass() { return velocity_output_lowpass_; }
            FlagField& velocityDOnError() { return velocity_d_on_error_; }
        
          private:
            HebiCommandPtr const internal_;
        
            FloatField velocity_kp_;
            FloatField velocity_ki_;
            FloatField velocity_kd_;
            FloatField velocity_feed_forward_;
            FloatField velocity_dead_zone_;
            FloatField velocity_i_clamp_;
            FloatField velocity_punch_;
            FloatField velocity_min_target_;
            FloatField velocity_max_target_;
            FloatField velocity_target_lowpass_;
            FloatField velocity_min_output_;
            FloatField velocity_max_output_;
            FloatField velocity_output_lowpass_;
            FlagField velocity_d_on_error_;
        
            HEBI_DISABLE_COPY_MOVE(VelocityGains)
        };
      
        class TorqueGains final
        {
          public:
            TorqueGains(HebiCommandPtr internal)
              : internal_(internal),
                torque_kp_(internal, CommandFloatTorqueKp),
                torque_ki_(internal, CommandFloatTorqueKi),
                torque_kd_(internal, CommandFloatTorqueKd),
                torque_feed_forward_(internal, CommandFloatTorqueFeedForward),
                torque_dead_zone_(internal, CommandFloatTorqueDeadZone),
                torque_i_clamp_(internal, CommandFloatTorqueIClamp),
                torque_punch_(internal, CommandFloatTorquePunch),
                torque_min_target_(internal, CommandFloatTorqueMinTarget),
                torque_max_target_(internal, CommandFloatTorqueMaxTarget),
                torque_target_lowpass_(internal, CommandFloatTorqueTargetLowpass),
                torque_min_output_(internal, CommandFloatTorqueMinOutput),
                torque_max_output_(internal, CommandFloatTorqueMaxOutput),
                torque_output_lowpass_(internal, CommandFloatTorqueOutputLowpass),
                torque_d_on_error_(internal, CommandFlagTorqueDOnError)
            {
            }
        
            FloatField& torqueKp() { return torque_kp_; }
            FloatField& torqueKi() { return torque_ki_; }
            FloatField& torqueKd() { return torque_kd_; }
            FloatField& torqueFeedForward() { return torque_feed_forward_; }
            FloatField& torqueDeadZone() { return torque_dead_zone_; }
            FloatField& torqueIClamp() { return torque_i_clamp_; }
            FloatField& torquePunch() { return torque_punch_; }
            FloatField& torqueMinTarget() { return torque_min_target_; }
            FloatField& torqueMaxTarget() { return torque_max_target_; }
            FloatField& torqueTargetLowpass() { return torque_target_lowpass_; }
            FloatField& torqueMinOutput() { return torque_min_output_; }
            FloatField& torqueMaxOutput() { return torque_max_output_; }
            FloatField& torqueOutputLowpass() { return torque_output_lowpass_; }
            FlagField& torqueDOnError() { return torque_d_on_error_; }
        
          private:
            HebiCommandPtr const internal_;
        
            FloatField torque_kp_;
            FloatField torque_ki_;
            FloatField torque_kd_;
            FloatField torque_feed_forward_;
            FloatField torque_dead_zone_;
            FloatField torque_i_clamp_;
            FloatField torque_punch_;
            FloatField torque_min_target_;
            FloatField torque_max_target_;
            FloatField torque_target_lowpass_;
            FloatField torque_min_output_;
            FloatField torque_max_output_;
            FloatField torque_output_lowpass_;
            FlagField torque_d_on_error_;
        
            HEBI_DISABLE_COPY_MOVE(TorqueGains)
        };
      
        public:
          Actuator(HebiCommandPtr internal)
            : internal_(internal),
              position_gains_(internal),
              velocity_gains_(internal),
              torque_gains_(internal),
              spring_constant_(internal, CommandFloatSpringConstant),
              control_strategy_(internal, CommandEnumControlStrategy)
          {
          }
      
          PositionGains& positionGains() { return position_gains_; }
      
          VelocityGains& velocityGains() { return velocity_gains_; }
      
          TorqueGains& torqueGains() { return torque_gains_; }
      
          FloatField& springConstant() { return spring_constant_; }
          EnumField<ControlStrategy>& controlStrategy() { return control_strategy_; }
      
        private:
          HebiCommandPtr const internal_;
      
          PositionGains position_gains_;
          VelocityGains velocity_gains_;
          TorqueGains torque_gains_;
      
          FloatField spring_constant_;
          EnumField<ControlStrategy> control_strategy_;
      
          HEBI_DISABLE_COPY_MOVE(Actuator)
      };
    
      public:
        Settings(HebiCommandPtr internal)
          : internal_(internal),
            actuator_(internal),
            name_(internal, CommandStringName),
            family_(internal, CommandStringFamily),
            save_current_settings_(internal, CommandFlagSaveCurrentSettings)
        {
        }
    
        Actuator& actuator() { return actuator_; }
    
        StringField& name() { return name_; }
        StringField& family() { return family_; }
        FlagField& saveCurrentSettings() { return save_current_settings_; }
    
      private:
        HebiCommandPtr const internal_;
    
        Actuator actuator_;
    
        StringField name_;
        StringField family_;
        FlagField save_current_settings_;
    
        HEBI_DISABLE_COPY_MOVE(Settings)
    };

    class Actuator final
    {
      public:
        Actuator(HebiCommandPtr internal)
          : internal_(internal),
            velocity_(internal, CommandFloatVelocity),
            torque_(internal, CommandFloatTorque),
            position_(internal, CommandHighResAnglePosition)
        {
        }
    
        FloatField& velocity() { return velocity_; }
        FloatField& torque() { return torque_; }
        HighResAngleField& position() { return position_; }
    
      private:
        HebiCommandPtr const internal_;
    
        FloatField velocity_;
        FloatField torque_;
        HighResAngleField position_;
    
        HEBI_DISABLE_COPY_MOVE(Actuator)
    };

  public:
    /**
     * C-style command object.
     * NOTE: this should not be used except by internal library functions!
     */
    HebiCommandPtr const internal_;

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    bool manage_pointer_lifetime_;

  public:
    /**
     * Create a new command object for a single module.
     */
    Command();
    /**
     * Wraps an existing C-style command object; object lifetime is assumed to
     * be managed by the caller.
     * NOTE: this should not be used except by internal library functions!
     */
    Command(HebiCommandPtr command);
    /**
     * Move constructor (necessary for containment in STL template classes)
     */
    Command(Command&& other);

    /**
     * Destructor cleans up command object as necessary.
     */
    virtual ~Command() noexcept; /* annotating specified destructor as noexcept is best-practice */

    // Submessage getters: Note that the returned reference should not be used
    // after the lifetime of this parent.
    Settings& settings();
    Actuator& actuator();

    NumberedFloatField& debug();
    LedField& led();

  private:
    Settings settings_;
    Actuator actuator_;

    NumberedFloatField debug_;
    LedField led_;

    /**
     * Disable copy constructor/assignment operators
     */
    HEBI_DISABLE_COPY(Command)

    /* Disable move assigment operator. */
    Command& operator= (const Command&& other) = delete;
};

} // namespace hebi

#endif // COMMAND_HPP
