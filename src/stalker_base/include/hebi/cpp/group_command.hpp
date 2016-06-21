#ifndef GROUP_COMMAND_HPP
#define GROUP_COMMAND_HPP

#include "hebi_group_command.h"
#include "command.hpp"
#include <vector>

namespace hebi {

class GroupCommand final
{
  public:
    /**
     * C-style group command object.
     * NOTE: this should not be used except by library functions!
     */
    HebiGroupCommandPtr const internal_;

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    const bool manage_pointer_lifetime_;
    /**
     * The number of modules in this group command.
     */
    const int number_of_modules_;
    /**
     * The list of Command subobjects
     */
    std::vector<Command> commands_;

  public:
    /**
     * Create a group command with the specified number of modules.
     */
    GroupCommand(int number_of_modules);

    /**
     * Destructor cleans up group command object as necessary.
     */
    virtual ~GroupCommand() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * Returns the number of module commands in this group command.
     */
    int size() const;

    /**
     * Access the command for an individual module.
     */
    Command& operator[](int index);
};

} // namespace hebi

#endif // GROUP_COMMAND_HPP
