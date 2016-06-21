#ifndef INFO_HPP
#define INFO_HPP

#include "hebi_info.h"
#include "color.hpp"
#include <string>
#include "util.hpp"

namespace hebi {

class Info final
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
        FloatField(HebiInfoPtr internal, InfoFloatField field);
        bool has() const;
        float get() const;

      private:
        HebiInfoPtr const internal_;
        InfoFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(FloatField)
    };
    class StringField final
    {
      public:
        StringField(HebiInfoPtr internal, InfoStringField field);
        bool has() const;
        std::string get() const;

      private:
        HebiInfoPtr const internal_;
        InfoStringField const field_;

        HEBI_DISABLE_COPY_MOVE(StringField)
    };

    class FlagField final
    {
      public:
        FlagField(HebiInfoPtr internal, InfoFlagField field);
        bool has() const;

      private:
        HebiInfoPtr const internal_;
        InfoFlagField const field_;

        HEBI_DISABLE_COPY_MOVE(FlagField)
    };

    template <class T>
    class EnumField final
    {
      public:
        EnumField(HebiInfoPtr internal, InfoEnumField field)
          : internal_(internal), field_(field) {}
        bool has() const { return (hebiInfoHasEnum(internal_, field_) == 1); }
        T get() const { return (T)hebiInfoGetEnum(internal_, field_); }

      private:
        HebiInfoPtr const internal_;
        InfoEnumField const field_;

        HEBI_DISABLE_COPY_MOVE(EnumField)
    };

    class LedField final
    {
      public:
        LedField(HebiInfoPtr internal, InfoLedField field);
        bool hasColor() const;
        Color getColor() const;

      private:
        HebiInfoPtr const internal_;
        InfoLedField const field_;

        HEBI_DISABLE_COPY_MOVE(LedField)
    };

    class Settings final
    {
      public:
        Settings(HebiInfoPtr internal)
          : internal_(internal),
            name_(internal, InfoStringName),
            family_(internal, InfoStringFamily),
            save_current_settings_(internal, InfoFlagSaveCurrentSettings)
        {
        }
    
        const StringField& name() const { return name_; }
        const StringField& family() const { return family_; }
        const FlagField& saveCurrentSettings() const { return save_current_settings_; }
    
      private:
        HebiInfoPtr const internal_;
    
        StringField name_;
        StringField family_;
        FlagField save_current_settings_;
    
        HEBI_DISABLE_COPY_MOVE(Settings)
    };

    class Actuator final
    {
      class PositionGains final
      {
        public:
          PositionGains(HebiInfoPtr internal)
            : internal_(internal),
              position_kp_(internal, InfoFloatPositionKp),
              position_ki_(internal, InfoFloatPositionKi),
              position_kd_(internal, InfoFloatPositionKd),
              position_feed_forward_(internal, InfoFloatPositionFeedForward),
              position_dead_zone_(internal, InfoFloatPositionDeadZone),
              position_i_clamp_(internal, InfoFloatPositionIClamp),
              position_punch_(internal, InfoFloatPositionPunch),
              position_min_target_(internal, InfoFloatPositionMinTarget),
              position_max_target_(internal, InfoFloatPositionMaxTarget),
              position_target_lowpass_(internal, InfoFloatPositionTargetLowpass),
              position_min_output_(internal, InfoFloatPositionMinOutput),
              position_max_output_(internal, InfoFloatPositionMaxOutput),
              position_output_lowpass_(internal, InfoFloatPositionOutputLowpass),
              position_d_on_error_(internal, InfoFlagPositionDOnError)
          {
          }
      
          const FloatField& positionKp() const { return position_kp_; }
          const FloatField& positionKi() const { return position_ki_; }
          const FloatField& positionKd() const { return position_kd_; }
          const FloatField& positionFeedForward() const { return position_feed_forward_; }
          const FloatField& positionDeadZone() const { return position_dead_zone_; }
          const FloatField& positionIClamp() const { return position_i_clamp_; }
          const FloatField& positionPunch() const { return position_punch_; }
          const FloatField& positionMinTarget() const { return position_min_target_; }
          const FloatField& positionMaxTarget() const { return position_max_target_; }
          const FloatField& positionTargetLowpass() const { return position_target_lowpass_; }
          const FloatField& positionMinOutput() const { return position_min_output_; }
          const FloatField& positionMaxOutput() const { return position_max_output_; }
          const FloatField& positionOutputLowpass() const { return position_output_lowpass_; }
          const FlagField& positionDOnError() const { return position_d_on_error_; }
      
        private:
          HebiInfoPtr const internal_;
      
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
          VelocityGains(HebiInfoPtr internal)
            : internal_(internal),
              velocity_kp_(internal, InfoFloatVelocityKp),
              velocity_ki_(internal, InfoFloatVelocityKi),
              velocity_kd_(internal, InfoFloatVelocityKd),
              velocity_feed_forward_(internal, InfoFloatVelocityFeedForward),
              velocity_dead_zone_(internal, InfoFloatVelocityDeadZone),
              velocity_i_clamp_(internal, InfoFloatVelocityIClamp),
              velocity_punch_(internal, InfoFloatVelocityPunch),
              velocity_min_target_(internal, InfoFloatVelocityMinTarget),
              velocity_max_target_(internal, InfoFloatVelocityMaxTarget),
              velocity_target_lowpass_(internal, InfoFloatVelocityTargetLowpass),
              velocity_min_output_(internal, InfoFloatVelocityMinOutput),
              velocity_max_output_(internal, InfoFloatVelocityMaxOutput),
              velocity_output_lowpass_(internal, InfoFloatVelocityOutputLowpass),
              velocity_d_on_error_(internal, InfoFlagVelocityDOnError)
          {
          }
      
          const FloatField& velocityKp() const { return velocity_kp_; }
          const FloatField& velocityKi() const { return velocity_ki_; }
          const FloatField& velocityKd() const { return velocity_kd_; }
          const FloatField& velocityFeedForward() const { return velocity_feed_forward_; }
          const FloatField& velocityDeadZone() const { return velocity_dead_zone_; }
          const FloatField& velocityIClamp() const { return velocity_i_clamp_; }
          const FloatField& velocityPunch() const { return velocity_punch_; }
          const FloatField& velocityMinTarget() const { return velocity_min_target_; }
          const FloatField& velocityMaxTarget() const { return velocity_max_target_; }
          const FloatField& velocityTargetLowpass() const { return velocity_target_lowpass_; }
          const FloatField& velocityMinOutput() const { return velocity_min_output_; }
          const FloatField& velocityMaxOutput() const { return velocity_max_output_; }
          const FloatField& velocityOutputLowpass() const { return velocity_output_lowpass_; }
          const FlagField& velocityDOnError() const { return velocity_d_on_error_; }
      
        private:
          HebiInfoPtr const internal_;
      
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
          TorqueGains(HebiInfoPtr internal)
            : internal_(internal),
              torque_kp_(internal, InfoFloatTorqueKp),
              torque_ki_(internal, InfoFloatTorqueKi),
              torque_kd_(internal, InfoFloatTorqueKd),
              torque_feed_forward_(internal, InfoFloatTorqueFeedForward),
              torque_dead_zone_(internal, InfoFloatTorqueDeadZone),
              torque_i_clamp_(internal, InfoFloatTorqueIClamp),
              torque_punch_(internal, InfoFloatTorquePunch),
              torque_min_target_(internal, InfoFloatTorqueMinTarget),
              torque_max_target_(internal, InfoFloatTorqueMaxTarget),
              torque_target_lowpass_(internal, InfoFloatTorqueTargetLowpass),
              torque_min_output_(internal, InfoFloatTorqueMinOutput),
              torque_max_output_(internal, InfoFloatTorqueMaxOutput),
              torque_output_lowpass_(internal, InfoFloatTorqueOutputLowpass),
              torque_d_on_error_(internal, InfoFlagTorqueDOnError)
          {
          }
      
          const FloatField& torqueKp() const { return torque_kp_; }
          const FloatField& torqueKi() const { return torque_ki_; }
          const FloatField& torqueKd() const { return torque_kd_; }
          const FloatField& torqueFeedForward() const { return torque_feed_forward_; }
          const FloatField& torqueDeadZone() const { return torque_dead_zone_; }
          const FloatField& torqueIClamp() const { return torque_i_clamp_; }
          const FloatField& torquePunch() const { return torque_punch_; }
          const FloatField& torqueMinTarget() const { return torque_min_target_; }
          const FloatField& torqueMaxTarget() const { return torque_max_target_; }
          const FloatField& torqueTargetLowpass() const { return torque_target_lowpass_; }
          const FloatField& torqueMinOutput() const { return torque_min_output_; }
          const FloatField& torqueMaxOutput() const { return torque_max_output_; }
          const FloatField& torqueOutputLowpass() const { return torque_output_lowpass_; }
          const FlagField& torqueDOnError() const { return torque_d_on_error_; }
      
        private:
          HebiInfoPtr const internal_;
      
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
        Actuator(HebiInfoPtr internal)
          : internal_(internal),
            position_gains_(internal),
            velocity_gains_(internal),
            torque_gains_(internal),
            spring_constant_(internal, InfoFloatSpringConstant),
            control_strategy_(internal, InfoEnumControlStrategy)
        {
        }
    
        const PositionGains& positionGains() const { return position_gains_; }
    
        const VelocityGains& velocityGains() const { return velocity_gains_; }
    
        const TorqueGains& torqueGains() const { return torque_gains_; }
    
        const FloatField& springConstant() const { return spring_constant_; }
        const EnumField<ControlStrategy>& controlStrategy() const { return control_strategy_; }
    
      private:
        HebiInfoPtr const internal_;
    
        PositionGains position_gains_;
        VelocityGains velocity_gains_;
        TorqueGains torque_gains_;
    
        FloatField spring_constant_;
        EnumField<ControlStrategy> control_strategy_;
    
        HEBI_DISABLE_COPY_MOVE(Actuator)
    };

  public:
    /**
     * C-style info object.
     * NOTE: this should not be used except by internal library functions!
     */
    HebiInfoPtr const internal_;

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    bool manage_pointer_lifetime_;

  public:
    /**
     * Create a new info object for a single module.
     */
    Info();
    /**
     * Wraps an existing C-style info object; object lifetime is assumed to
     * be managed by the caller.
     * NOTE: this should not be used except by internal library functions!
     */
    Info(HebiInfoPtr info);
    /**
     * Move constructor (necessary for containment in STL template classes)
     */
    Info(Info&& other);

    /**
     * Destructor cleans up info object as necessary.
     */
    virtual ~Info() noexcept; /* annotating specified destructor as noexcept is best-practice */

    // Submessage getters: Note that the returned reference should not be used
    // after the lifetime of this parent.
    const Settings& settings() const;
    const Actuator& actuator() const;

    const LedField& led() const;

  private:
    Settings settings_;
    Actuator actuator_;

    LedField led_;

    /**
     * Disable copy constructor/assignment operators
     */
    HEBI_DISABLE_COPY(Info)

    /* Disable move assigment operator. */
    Info& operator= (const Info&& other) = delete;
};

} // namespace hebi

#endif // INFO_HPP
