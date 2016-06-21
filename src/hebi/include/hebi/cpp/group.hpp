#ifndef GROUP_HPP
#define GROUP_HPP

#include "hebi_group.h"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "group_info.hpp"
#include "util.hpp"

#include <functional>
#include <mutex>

namespace hebi {

/**
 * \example command_control_strategy_group_example.cpp
 *
 * \example command_group_example.cpp
 *
 * \example command_settings_group_example.cpp
 *
 * \example feedback_async_group_example.cpp
 *
 * \example feedback_sync_group_example.cpp
 *
 * \example info_sync_group_example.cpp
 *
 * \example master_slave_group_example.cpp
 *
 * \example master_slave_async_group_example.cpp
 *
 * \example command_spring_constants_group_example.cpp
 *
 */

typedef std::function<void (const GroupFeedback*)> GroupFeedbackHandler;

class Group final
{
  private:
    /**
     * C-style group object
     */
    HebiGroupPtr internal_;

    /**
     * The number of modules in this group.
     */
    const int number_of_modules_;

    /**
     * Protects access to the group feedback handler vector.
     */
    std::mutex handler_lock_;

    /**
     * A list of handler functions that are called when the internal C API
     * feedback callback is made.
     */
    std::vector<GroupFeedbackHandler> handlers_;

    /**
     * Intermediary to convert C-style function callbacks to C++ style, and
     * change callback parameter types.
     */
    friend void callbackWrapper(HebiGroupFeedbackPtr group_feedback, void* user_data);

    /**
     * Calls any attached C++ handler functions.  Should only be called from the
     * internal C thread via the callbackWrapper translation function.
     */
    void callAttachedHandlers(HebiGroupFeedbackPtr group_feedback);

  public:
    /**
     * The default timeout for any send-with-acknowledgement or request
     * operation is 500 ms.
     */
    static const int DEFAULT_TIMEOUT_MS = 500;

    /**
     * Creates a group from the underlying C-style group object. This should
     * only be called to create groups from the lookup class, not from user
     * code!
     */
    Group(HebiGroupPtr group);

    /**
     * Destructor cleans up group.
     */
    virtual ~Group() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * Returns the number of modules in the group
     */
    int size();

    /**
     * Send a command to the given group without requesting an acknowledgement.
     *
     * Appropriate for high-frequency applications.
     * 
     * \param command The GroupCommand object containing information to be
     * sent to the group.
     *
     * \returns true if the command was successfully sent, false otherwise.
     */
    bool sendCommand(const GroupCommand& group_command);

    /**
     * Send a command to the given group, requesting an acknowledgement of
     * transmission to be sent back.
     *
     * \param command The GroupCommand object containing information to be
     * sent to the group.
     * \param timeout_ms Indicates how many milliseconds to wait for a response
     * after sending a packet.  For typical networks, '15' ms is a value that
     * can be reasonably expected to encompass the time required for a
     * round-trip transmission.
     *
     * \returns true if an acknowledgement was successfully received
     * (guaranteeing the group received this command), or a negative number for
     * an error otherwise.
     *
     * Note: A non-true return does not indicate a specific failure, and may
     * result from an error while sending or simply a timeout/dropped response
     * packet after a successful transmission.
     */
    bool sendCommandWithAcknowledgement(const GroupCommand& group_command, int timeout_ms=DEFAULT_TIMEOUT_MS);

    /** 
     * Request feedback from the group, and store it in the passed-in feedback
     * object.
     *
     * \returns true if the request was successful within the specified timeout;
     * in this case 'feedback' has been updated. Otherwise, returns false and
     * does not update 'feedback'.
     */
    bool requestFeedback(GroupFeedback* feedback, int timeout_ms=DEFAULT_TIMEOUT_MS);

    /** 
     * Request info from the group, and store it in the passed-in info object.
     *
     * \returns true if the request was successful within the specified timeout;
     * in this case 'info' has been updated. Otherwise, returns false and does
     * not update 'info'.
     */
    bool requestInfo(GroupInfo* info, int timeout_ms=DEFAULT_TIMEOUT_MS);

    /**
     * Starts log (stopping any active log); returns true on success.
     *
     * NOTE: FUNCTION INTERFACE SUBJECT TO CHANGE.
     */
    bool startLog();

    /**
     * Stops any active log; returns true if a log was running and was
     * successfully stopped, false otherwise.
     *
     * NOTE: FUNCTION INTERFACE SUBJECT TO CHANGE.
     */
    bool stopLog();

    /**
     * Sets the frequency of the internal feedback request + callback thread.
     */
    bool setFeedbackFrequencyHz(float frequency);
    /**
     * Gets the frequency of the internal feedback request + callback thread.
     */
    float getFeedbackFrequencyHz();
    /**
     * Adds a handler function to be called by the internal feedback request
     * thread.
     */
    void addFeedbackHandler(GroupFeedbackHandler handler);
    /**
     * Removes all feedback handlers presently added.
     */
    void clearFeedbackHandlers();

  private:
    /**
     * Disable copy and move constructors and assignment operators
     */
    HEBI_DISABLE_COPY_MOVE(Group)
};

} // namespace hebi

#endif // GROUP_HPP
