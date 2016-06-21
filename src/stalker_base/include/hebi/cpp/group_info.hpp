#ifndef GROUP_INFO_HPP
#define GROUP_INFO_HPP

#include "hebi_group_info.h"
#include "info.hpp"
#include <vector>

namespace hebi {

class GroupInfo final
{
  public:
    /**
     * C-style group info object.
     * NOTE: this should not be used except by library functions!
     */
    HebiGroupInfoPtr const internal_;

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    const bool manage_pointer_lifetime_;
    /**
     * The number of modules in this group info.
     */
    const int number_of_modules_;
    /**
     * The list of Info subobjects
     */
    std::vector<Info> infos_;

  public:
    /**
     * Create a group info with the specified number of modules.
     */
    GroupInfo(int number_of_modules);

    /**
     * Destructor cleans up group info object as necessary.
     */
    virtual ~GroupInfo() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * Returns the number of module infos in this group info.
     */
    int size() const;

    /**
     * Access the info for an individual module.
     */
    const Info& operator[](int index) const;
};

} // namespace hebi

#endif // GROUP_INFO_HPP
