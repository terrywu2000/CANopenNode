#ifndef XNET_CO_DRIVE402_HPP
#define XNET_CO_DRIVE402_HPP

/**
 * @file
 * CiA 402 drive core: power-drive state machine and PP profile generation at
 * the motor model boundary. This unit has no CANopen or XNET dependency. The
 * device passes OD values in through Inputs and reads Status back out, all under
 * the device runtime mutex, so the core itself is not thread-safe.
 */

#include <cstdint>
#include "CO_402_protocol.h"

namespace xco::d402 {

/** Power drive state (CiA 402 PDS). */
enum class Pds : uint8_t {
    not_ready_to_switch_on,
    switch_on_disabled,
    ready_to_switch_on,
    switched_on,
    operation_enabled,
    quick_stop_active,
    fault_reaction_active,
    fault,
};

constexpr int8_t mode_profile_position = 1;

/** Actual axis state reported by a motor model for one cycle. */
struct Feedback {
    int32_t position = 0; /**< Actual position in counts. */
    int32_t velocity = 0; /**< Actual velocity in counts/s. */
    bool fault = false; /**< Model-reported fault; forces the drive into fault reaction. */
};

/** Motor model boundary. Implementations are driven under the device runtime mutex. */
class MotorModel {
public:
    virtual ~MotorModel() = default;
    /** Advance by dt_us from the drive demand and return the axis actual state. */
    virtual Feedback sample(int32_t demand_position, int32_t demand_velocity, uint32_t dt_us) = 0;
    /** True when feedback is current enough to be used as actual values. */
    virtual bool fresh() const = 0;
};

/** Default model: the axis follows the demand instantly and never faults. */
class InternalModel final : public MotorModel {
public:
    Feedback sample(int32_t demand_position, int32_t demand_velocity, uint32_t dt_us) override;
    bool fresh() const override { return true; }
};

/**
 * External model: feedback posted by the host. Feedback older than the timeout is
 * stale; the drive treats stale feedback as a model fault. Actuals freeze at the
 * last value while stale.
 */
class ExternalModel final : public MotorModel {
public:
    explicit ExternalModel(uint32_t timeout_us) : timeout_us_(timeout_us) {}
    /** Store feedback and reset its age. */
    void post(const Feedback& feedback);
    Feedback sample(int32_t demand_position, int32_t demand_velocity, uint32_t dt_us) override;
    bool fresh() const override { return has_feedback_ && age_us_ < timeout_us_; }
    bool has_feedback() const { return has_feedback_; }
    void reset() { age_us_ = 0; has_feedback_ = false; last_ = {}; }

private:
    uint32_t timeout_us_;
    uint64_t age_us_ = 0;
    bool has_feedback_ = false;
    Feedback last_{};
};

/** Per-cycle OD inputs, read by the device from the OD before each step. */
struct Inputs {
    uint16_t controlword = 0;
    int8_t mode_requested = 0; /**< 0x6060. */
    bool nmt_operational = true; /**< NMT state is owned by the lower CANopen layer. */
    int32_t target_position = 0; /**< 0x607A. */
    uint32_t profile_velocity = 0; /**< 0x6081 PP profile velocity. */
    uint32_t max_profile_velocity = 0; /**< 0x607F. */
    uint32_t profile_acceleration = 0; /**< 0x6083. */
    uint32_t profile_deceleration = 0; /**< 0x6084. */
    uint32_t quick_stop_deceleration = 0; /**< 0x6085. */
    uint32_t position_tolerance = 0; /**< Position difference accepted for target reached. */
    uint32_t target_reached_time_us = 0; /**< Required in-tolerance duration. */
    bool model_ready = true; /**< External model has acknowledged readiness for PP operation. */
    bool reset_permitted = true; /**< External model permits the fault-reset edge. */
    uint32_t action_response_seq = 0;
    uint8_t action_result = CO_402_ACTION_RESULT_NONE;
    int16_t quick_stop_option = 2; /**< 0x605A. */
    int16_t fault_reaction_option = 0; /**< 0x605E. */
};

/** Host-visible drive snapshot. */
struct Status {
    Pds pds = Pds::switch_on_disabled;
    uint16_t statusword = 0;
    int8_t mode_display = 0; /**< 0x6061; 0 when the requested mode is not implemented. */
    int32_t demand_position = 0, demand_velocity = 0;
    int32_t actual_position = 0, actual_velocity = 0;
    bool feedback_fresh = false;
    bool model_fault = false;
    bool external_model = false;
    bool model_ready = true;
    uint8_t action = CO_402_ACTION_NONE;
    uint32_t action_seq = 1;
    bool action_pending = false;
    bool output_allowed = false;
};

/**
 * Drive state machine and PP profile generator. Additional modes are future work.
 */
class Drive {
public:
    explicit Drive(uint32_t feedback_timeout_us = 1000000);

    /** Select the internal default model (false) or the external model (true). */
    void use_external_model(bool external);
    bool external_model() const { return use_external_; }
    /** Post host feedback; false when the external model is not selected. */
    bool post_feedback(const Feedback& feedback);
    /** Set an external transport/startup fault that cannot be inferred from feedback alone. */
    void set_external_fault(bool fault);
    /** Set PP readiness and reset permission from the latest accepted model frame. */
    void set_external_permissions(bool ready, bool reset_permitted);

    /** Run one cycle: state transitions, profile update, then the model sample. */
    void step(const Inputs& in, uint32_t dt_us);

    /** Snapshot for statusword, mode display, and host status. */
    Status status() const;

    /** Return to switch-on-disabled and zero the profile. */
    void reset();

    static bool mode_supported(int8_t mode);

private:
    MotorModel& model();
    void transition(const Inputs& in);
    void request_transition(uint8_t action, Pds target);
    void motion(const Inputs& in, double dt_s);
    void latch_setpoint(const Inputs& in);
    void approach(double target_velocity, uint32_t accel, uint32_t decel, double dt_s);
    void track_target(double dt_s, double vmax, uint32_t accel, uint32_t decel);

    InternalModel internal_;
    ExternalModel external_;
    bool use_external_ = false;

    Pds pds_ = Pds::switch_on_disabled;
    int8_t mode_display_ = 0;
    int8_t previous_mode_ = 0;

    double demand_pos_ = 0.0, demand_vel_ = 0.0;
    double target_pos_ = 0.0;
    bool pp_setpoint_ack_ = false;
    bool target_reached_ = false;
    bool pending_valid_ = false;
    double pending_pos_ = 0.0;
    bool halt_ = false;
    bool model_fault_ = false;
    bool external_transport_fault_ = false;
    bool external_model_ready_ = false;
    bool external_reset_permitted_ = false;
    uint8_t action_ = CO_402_ACTION_NONE;
    uint32_t action_seq_ = 1;
    bool action_pending_ = false;
    uint8_t pending_action_ = CO_402_ACTION_NONE;
    Pds pending_pds_ = Pds::switch_on_disabled;
    bool action_failed_ = false;
    bool speed_reached_ = false;
    uint64_t target_reached_age_us_ = 0;
    int16_t reaction_option_ = 0;
    int16_t quick_stop_option_ = 2;
    /** Model fault, or external feedback that is stale (a timeout is a fault). */
    bool model_faulted() const;
    bool fault_reset_prev_ = false;
    Feedback feedback_{};
};

} // namespace xco::d402

#endif
