#ifndef MODULE_HPP
#define MODULE_HPP

#include "hebi_module.h"
#include "command.hpp"
#include "feedback.hpp"
#include "info.hpp"
#include "util.hpp"

#include <functional>
#include <mutex>
#include <vector>

namespace hebi {

/**
 * \example command_module_example.cpp
 *
 * \example feedback_sync_module_example.cpp
 *
 */

typedef std::function<void (const Feedback*)> ModuleFeedbackHandler;

class Module final
{
  private:
    /**
     * C-style module object
     */
    HebiModulePtr internal_;

    /**
     * Protects access to the feedback handler vector.
     */
    std::mutex handler_lock_;

    /**
     * A list of handler functions that are called when the internal C API
     * feedback callback is made.
     */
    std::vector<ModuleFeedbackHandler> handlers_;

    /**
     * Intermediary to convert C-style function callbacks to C++ style, and
     * change callback parameter types.
     */
    friend void callbackWrapper(HebiFeedbackPtr feedback, void* user_data);

    /**
     * Calls any attached C++ handler functions.  Should only be called from the
     * internal C thread via the callbackWrapper translation function.
     */
    void callAttachedHandlers(HebiFeedbackPtr feedback);

  public:
    /**
     * The default timeout for any send-with-acknowledgement or request
     * operation is 500 ms.
     */
    static const int DEFAULT_TIMEOUT_MS = 500;

    /**
     * Creates a module from the underlying C-style module object. This should
     * only be called to create modules from the lookup class, not from user
     * code!
     */
    Module(HebiModulePtr module);

    /**
     * Destructor cleans up module.
     */
    virtual ~Module() noexcept; /* annotating specified destructor as noexcept is best-practice */
   
    /**
     * Send a command to the given module without requesting an acknowledgement.
     *
     * Appropriate for high-frequency applications.
     * 
     * \param command The Command object containing information to be
     * sent to the module.
     *
     * \returns true if the command was successfully sent, false otherwise.
     */
    bool sendCommand(const Command& command);

    /**
     * Send a command to the given module, requesting an acknowledgement of
     * transmission to be sent back.
     *
     * \param command The Command object containing information to be
     * sent to the module.
     * \param timeout_ms Indicates how many milliseconds to wait for a response
     * after sending a packet.  For typical networks, '15' ms is a value that
     * can be reasonably expected to encompass the time required for a
     * round-trip transmission.
     *
     * \returns true if an acknowledgement was successfully received
     * (guaranteeing the module received this command), or a negative number for
     * an error otherwise.
     *
     * Note: A non-true return does not indicate a specific failure, and may
     * result from an error while sending or simply a timeout/dropped response
     * packet after a successful transmission.
     */
    bool sendCommandWithAcknowledgement(const Command& command, int timeout_ms=DEFAULT_TIMEOUT_MS);

    /** 
     * Request feedback from the module, and store it in the passed-in feedback
     * object.
     *
     * \returns true if the request was successful within the specified timeout;
     * in this case 'feedback' has been updated. Otherwise, returns false and
     * does not update 'feedback'.
     */
    bool requestFeedback(Feedback* feedback, int timeout_ms=DEFAULT_TIMEOUT_MS);

    /** 
     * Request info from the module, and store it in the passed-in info object.
     *
     * \returns true if the request was successful within the specified timeout;
     * in this case 'info' has been updated. Otherwise, returns false and does
     * not update 'info'.
     */
    bool requestInfo(Info* info, int timeout_ms=DEFAULT_TIMEOUT_MS);

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
    void addFeedbackHandler(ModuleFeedbackHandler handler);
    /**
     * Removes all feedback handlers presently added.
     */
    void clearFeedbackHandlers();

  private:
    /**
     * Disable copy and move constructors and assignment operators
     */
    HEBI_DISABLE_COPY_MOVE(Module)
};

} // namespace hebi

#endif // MODULE_HPP
